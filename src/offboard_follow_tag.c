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
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "pose_filter.h"
#include "offboard_follow_tag.h"
#include "misc.h"


#define FLIGHT_ALTITUDE -1.5f
#define RATE            30  // loop rate hz
#define FILTER_LENGTH   3


static int running = 0;
static pthread_t offboard_follow_tag_thread_id;
static mavlink_set_position_target_local_ned_t setpoint;
static int en_debug = 0;

static rc_vector_t T_setpoint_wrt_tag = RC_VECTOR_INITIALIZER;
static rc_vector_t T_setpoint_wrt_local = RC_VECTOR_INITIALIZER;

/*
// stuff for landing target in the future!!
static rc_pose_filter_t filter_target_wrt_local = RC_POSE_FILTER_INITIALIZER;

// coordinate frame of the apriltag is different than how we want the drone to
// be oriented when we land on it. If the drone lined up with the tag's coordinate
// frame it would be upside down and pointing to the right. This rotation matrix
// converts from the desired landing orientation to the tag frame
static rc_matrix_t R_target_to_tag = RC_MATRIX_INITIALIZER;
    rc_pose_filter_alloc(&filter_target_wrt_local, FILTER_LENGTH);
    rc_matrix_alloc(&R_target_to_tag,3,3);
    R_target_to_tag.d[0][0] =  0.0;
    R_target_to_tag.d[0][1] =  1.0;
    R_target_to_tag.d[0][2] =  0.0;
    R_target_to_tag.d[1][0] = -1.0;
    R_target_to_tag.d[1][1] =  0.0;
    R_target_to_tag.d[1][2] =  0.0;
    R_target_to_tag.d[2][0] =  0.0;
    R_target_to_tag.d[2][1] =  0.0;
    R_target_to_tag.d[2][2] =  1.0;
*/


static void _send_setpoint(void)
{
    mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,setpoint);
    return;
}

static void _update_setpoint_to_current_position(void)
{
    static rc_vector_t T_body_wrt_local = RC_VECTOR_INITIALIZER;
    double roll, pitch, yaw;
    geometry_get_T_body_wrt_local(&T_body_wrt_local);
    geometry_get_tait_bryan_body_wrt_local(&roll, &pitch, &yaw);
    setpoint.x = T_body_wrt_local.d[0];
    setpoint.y = T_body_wrt_local.d[1];
    setpoint.z = T_body_wrt_local.d[2];
    setpoint.yaw = yaw;
    return;
}

static void* _offboard_follow_tag_thread_func(__attribute__((unused)) void* arg)
{
    int i;
    int64_t next_time = 0;

    //send a few setpoints before starting
    for(i = 100; running && i > 0; --i){
        _update_setpoint_to_current_position();
        _send_setpoint();
        if(my_loop_sleep(RATE, &next_time)){
            fprintf(stderr, "WARNING follow tag thread fell behind\n");
        }
    }


HOME:
    // wait for the system to be armed and in offboard mode
    // untill that is true, keep the setpoint at current position/rotation
    while(running && !autopilot_monitor_is_armed_and_in_offboard_mode()){
        _update_setpoint_to_current_position();
        _send_setpoint();
        if(my_loop_sleep(RATE, &next_time)){
            fprintf(stderr, "WARNING follow tag thread fell behind\n");
        }
        fflush(stdout);
    }

    // now we have broken out of the previous loop, we are in offboard mode!
    while(running){
        // return to home position if px4 falls out of offboard mode or disarms
        if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
        _send_setpoint();
        if(my_loop_sleep(RATE, &next_time)){
            fprintf(stderr, "WARNING follow tag thread fell behind\n");
        }
    }

    printf("exiting offboard follow tag thread\n");
    return NULL;
}


int offboard_follow_tag_init(void)
{
    // prefill the setpoint mavlink struct, only xyz yaw will be changed later
    setpoint.time_boot_ms = 0;
    setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;
    setpoint.type_mask =   POSITION_TARGET_TYPEMASK_VX_IGNORE |
                                POSITION_TARGET_TYPEMASK_VY_IGNORE |
                                POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                POSITION_TARGET_TYPEMASK_AX_IGNORE |
                                POSITION_TARGET_TYPEMASK_AY_IGNORE |
                                POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    setpoint.x = 0.0f;
    setpoint.y = 0.0f;
    setpoint.z = 0.0;
    setpoint.yaw = 0.0;
    setpoint.target_system = 0; // will reset later when sending
    setpoint.target_component = AUTOPILOT_COMPID;

    // initialize the vector from tag to dron position setpoint
    // TODO put this in config file
    rc_vector_alloc(&T_setpoint_wrt_local, 3);
    rc_vector_alloc(&T_setpoint_wrt_tag, 3);
    T_setpoint_wrt_tag.d[0] = 0;
    T_setpoint_wrt_tag.d[1] = 0;
    T_setpoint_wrt_tag.d[2] = -1.5; // 1.5 meters OUT of the tag

    // set the running flag and start the thread!
    running = 1;
    pipe_pthread_create(&offboard_follow_tag_thread_id, _offboard_follow_tag_thread_func, \
    							NULL, OFFBOARD_THREAD_PRIORITY);
    return 0;
}


int offboard_follow_tag_stop(int blocking)
{
    if(running==0){
        fprintf(stderr, "ERROR in offboard_follow_tag_stop, not running\n");
        return -1;
    }
    running = 0;
    if(blocking){
        pthread_join(offboard_follow_tag_thread_id, NULL);
    }
    return 0;
}


int offboard_follow_tag_add_detection(int64_t frame_timestamp_ns, rc_matrix_t R_tag_to_cam, rc_vector_t T_tag_wrt_cam)
{
    if(!running) return -1;

    // static matrices so we don't realloc each detection
    static rc_matrix_t new_R_tag_to_local;
    static rc_vector_t new_T_tag_wrt_local;

    // ask geometry module for the position and rotation of the tag in local
    // frame. This goes back in time and interpolates given timestamp.
    if(geometry_calc_R_T_tag_in_local_frame(frame_timestamp_ns, R_tag_to_cam, \
                    T_tag_wrt_cam, &new_R_tag_to_local, &new_T_tag_wrt_local)){
        return -1;
    }

    // calculate new position of sepoint in local frame
    rc_matrix_times_col_vec(new_R_tag_to_local, T_setpoint_wrt_tag, &T_setpoint_wrt_local);
    rc_vector_sum_inplace(&T_setpoint_wrt_local, new_T_tag_wrt_local);

    // limit altitude so drone doesn't hit the ground. NOTE, this assumes drone
    // initialized on the ground. maybe have the limit reference fixed frame?
    if(T_setpoint_wrt_local.d[2] > -1.0) T_setpoint_wrt_local.d[2]=-1.0;

    // calculate yaw in local frame that points drone at the tag
    double dx = new_T_tag_wrt_local.d[0] - T_setpoint_wrt_local.d[0];
    double dy = new_T_tag_wrt_local.d[1] - T_setpoint_wrt_local.d[1];
    double yaw = atan2(dy,dx);

    // copy into setpoint struct
    setpoint.x = T_setpoint_wrt_local.d[0];
    setpoint.y = T_setpoint_wrt_local.d[1];
    setpoint.z = T_setpoint_wrt_local.d[2];
    setpoint.yaw = yaw;


    if(en_debug){
        printf("setpoint_wrt_local:%4.1f %4.1f %4.1f  yaw: %4.1f\n", \
            T_setpoint_wrt_local.d[0],T_setpoint_wrt_local.d[1],T_setpoint_wrt_local.d[2],\
            yaw);
    }

    return 0;
}

void offboard_follow_tag_en_print_debug(int debug)
{
    if(debug) en_debug = 1;
}
