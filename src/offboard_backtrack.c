/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <rc_math.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "macros.h"
#include "offboard_backtrack.h"
#include "misc.h"

#define RATE 30 // loop rate hz

static int running = 0;
static pthread_t offboard_backtrack_thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t *path;
static int steps = 0;

static int path_index = 0;
static int valid_indices = 0;

static void _init_path(void)
{
	steps = backtrack_seconds * RATE;

	path = (mavlink_set_position_target_local_ned_t*) calloc(steps, sizeof(mavlink_set_position_target_local_ned_t));

	for(int i = 0; i < steps; i++) {
		path[i].time_boot_ms = 0;
		path[i].coordinate_frame = MAV_FRAME_LOCAL_NED;
		path[i].type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE  |
							POSITION_TARGET_TYPEMASK_VX_IGNORE  |
							POSITION_TARGET_TYPEMASK_VY_IGNORE  |
							POSITION_TARGET_TYPEMASK_VZ_IGNORE  |
							POSITION_TARGET_TYPEMASK_AX_IGNORE  |
							POSITION_TARGET_TYPEMASK_AY_IGNORE  |
							POSITION_TARGET_TYPEMASK_AZ_IGNORE  |
							POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

		// These will all get updated as the vehicle moves
		path[i].x = 0.0f;
		path[i].y = 0.0f;
		path[i].z = 0.0f;
		path[i].yaw = 0.0f;

		path[i].target_system = 0; // will reset later when sending
		path[i].target_component = AUTOPILOT_COMPID;
	}

	return;
}

static void* _offboard_backtrack_thread_func(__attribute__((unused)) void* arg)
{
	// for loop sleep
	int64_t next_time = 0;

	_init_path();

	bool armed = false;
	bool offboard_mode = false;
	bool rc_healthy = false;
	bool offboard_mode_change_request_sent = false;
	bool position_mode_change_request_sent = false;
	bool backtrack_active = false;
	mavlink_set_position_target_local_ned_t setpoint;

	// wait for the system to be armed and in offboard mode
	while (running) {

		armed = autopilot_monitor_is_armed();
		offboard_mode = (autopilot_monitor_get_main_mode() == PX4_MAIN_MODE_OFFBOARD);
		rc_healthy = autopilot_monitor_get_rc_state();

		if (armed) {
			if (offboard_mode) {

				if ( ! backtrack_active) {
					printf("Starting to backtrack for %u seconds\n", backtrack_seconds);
					backtrack_active = true;
				}

				offboard_mode_change_request_sent = false;

				if (valid_indices) {
					valid_indices--;

					if (path_index == 0) path_index = steps - 1;
					else path_index--;
				}

				// Monitor the backtrack switch. If it has toggled back to
				// non-backtrack then transition back to position hold mode
				if ((!position_mode_change_request_sent) &&
					(autopilot_monitor_get_rc_value(backtrack_rc_chan) < backtrack_rc_thresh)) {
					mavlink_set_mode_t set_mode;
					set_mode.custom_mode = PX4_MAIN_MODE_POSCTL << 16;
					set_mode.target_system = autopilot_monitor_get_sysid();
					set_mode.base_mode = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
					printf("Sending request to transition back to position mode due to RC switch\n");
					mavlink_io_send_set_mode(autopilot_monitor_get_sysid(), VOXL_COMPID, set_mode);
					position_mode_change_request_sent = true;
				}

				setpoint = path[path_index];

			} else {
				if (! offboard_mode_change_request_sent) {
					if (backtrack_active) {
						printf("Backtrack completed\n");
						path_index = 0;
						valid_indices = 0;
						offboard_mode_change_request_sent = false;
						position_mode_change_request_sent = false;
						backtrack_active = false;
					}

					mavlink_odometry_t odom = autopilot_monitor_get_odometry();

					path[path_index].x = odom.x;
					path[path_index].y = odom.y;
					path[path_index].z = odom.z;

					double q[4] = {(double) odom.q[0], (double) odom.q[1], (double) odom.q[2], (double) odom.q[3]};
					double tb[3];
					
					if (rc_quaternion_to_tb_array(q, tb) == 0) {
						path[path_index].yaw = tb[2];
					}

					setpoint = path[path_index];

					bool transition_to_offboard = false;
					if (! rc_healthy) {
						printf("Sending request to transition to offboard mode due to RC loss\n");
						transition_to_offboard = true;
					} else if (autopilot_monitor_get_rc_value(backtrack_rc_chan) > backtrack_rc_thresh) {
						printf("Sending request to transition to offboard mode due to RC switch\n");
						transition_to_offboard = true;
					}

					if (transition_to_offboard) {
						mavlink_set_mode_t set_mode;
		 				set_mode.custom_mode = PX4_MAIN_MODE_OFFBOARD << 16;
		 				set_mode.target_system = autopilot_monitor_get_sysid();
		 				set_mode.base_mode = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
						mavlink_io_send_set_mode(autopilot_monitor_get_sysid(), VOXL_COMPID, set_mode);
						offboard_mode_change_request_sent = true;
					}

					// Update index and handle wrap around
					if (path_index == (steps - 1)) path_index = 0;
					else path_index++;

					if (valid_indices < steps) valid_indices++;

				} else {
					// If we are not in offboard mode but we have already sent off
					// the request to enter offboard mode then just keep sending the last
					// valid setpoint and don't store any new ones.
					// Once we get into offboard mode we will start backtracking.
					int hold_index = path_index;
					if (hold_index) hold_index--;
					else hold_index = steps - 1;
					setpoint = path[hold_index];
				}
			}

			mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),
										   VOXL_COMPID, setpoint);

		} else {
			// Reset everything if we aren't armed
			path_index = 0;
			valid_indices = 0;
			offboard_mode_change_request_sent = false;
			backtrack_active = false;
		}

		if (my_loop_sleep(RATE, &next_time)) {
			fprintf(stderr, "WARNING backtrack thread fell behind\n");
		}

		fflush(stdout);
	}


	printf("exiting offboard backtrack thread\n");
	return NULL;
}


int offboard_backtrack_init(void)
{
	running = 1;
	pipe_pthread_create(&offboard_backtrack_thread_id, _offboard_backtrack_thread_func, NULL, OFFBOARD_THREAD_PRIORITY);
	return 0;
}


int offboard_backtrack_stop(int blocking)
{
	if(running==0) return 0;
	running = 0;
	if(blocking){
		pthread_join(offboard_backtrack_thread_id, NULL);
	}
	return 0;
}

void offboard_backtrack_en_print_debug(int debug)
{
	if(debug) en_debug = 1;
}
