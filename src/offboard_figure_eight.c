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
#include "macros.h"
#include "offboard_figure_eight.h"
#include "misc.h"

#define FLIGHT_ALTITUDE	-1.5f
#define RATE			30	// loop rate hz
#define RADIUS			1.0	// radius of figure 8 in meters
#define CYCLE_S			8	// time to complete one figure 8 cycle in seconds
#define STEPS			(CYCLE_S*RATE)

static int running = 0;
static pthread_t offboard_figure_eight_thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t path[STEPS];
static mavlink_set_position_target_local_ned_t home_position;


// generate a path following Bernoulli's lemiscate as a parametric equation
// note this is in ENU coordinates since mavros will convert to NED
// x right, y forward, z up.
static void _init_path(void)
{
	int i;
	const double dt = 1.0/RATE;
	const double dadt = (TWO_PI)/CYCLE_S; // first derivative of angle with respect to time
	const double r = RADIUS;

	for(i=0;i<STEPS;i++){
		// basic fields in the message
		path[i].time_boot_ms = 0;
		path[i].coordinate_frame = MAV_FRAME_LOCAL_NED;
		path[i].type_mask = 0; // use everything!!
		// path[i].type_mask = 	POSITION_TARGET_TYPEMASK_AX_IGNORE |
		// 						POSITION_TARGET_TYPEMASK_AY_IGNORE |
		// 						POSITION_TARGET_TYPEMASK_AZ_IGNORE;
		path[i].target_system = 0; // will reset later when sending
		path[i].target_component = AUTOPILOT_COMPID;

		// calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
		// through the curve
		double a = (-PI_2) + i*(TWO_PI/STEPS);
		double c = cos(a);
		double c2a = cos(2.0*a);
		double c4a = cos(4.0*a);
		double c2am3 = c2a-3.0;
		double c2am3_cubed = c2am3*c2am3*c2am3;
		double s = sin(a);
		double cc = c*c;
		double ss = s*s;
		double sspo = (s*s)+1.0; // sin squared plus one
		double ssmo = (s*s)-1.0; // sin squared minus one
		double sspos = sspo*sspo;

		// Position
		// https://www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
		path[i].x = -(r*c*s) / sspo;
		// https://www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
		path[i].y =  (r*c)   / sspo;
		path[i].z =  FLIGHT_ALTITUDE;

		// Velocity
		// https://www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].vx =   dadt*r* ( ss*ss + ss + (ssmo*cc) )   /  sspos;
		// https://www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].vy =  -dadt*r* s*( ss + 2.0*cc + 1.0 )  / sspos;
		path[i].vz =  0.0f;

		// Acceleration
		// https://www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].afx =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/ c2am3_cubed;
		// https://www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].afy =  dadt*dadt*r*c*((44.0*c2a) + c4a - 21.0) / c2am3_cubed;
		path[i].afz =  0.0f;

		// calculate yaw as direction of velocity
		path[i].yaw = atan2(path[i].vy, path[i].vx);
	}

	// calculate yaw_rate by dirty differentiating yaw
	for(i=0;i<STEPS;i++){
		double next = path[(i+1)%STEPS].yaw;
		double curr = path[i].yaw;
		// account for wrap around +- PI
		if((next-curr) < -PI) next+=(TWO_PI);
		if((next-curr) >  PI) next-=(TWO_PI);
		path[i].yaw_rate = (next-curr)/dt;
	}

	if(en_debug){
		// dump out the trajectory for debugging.
		printf("===========================================================\n");
		printf("   X     Y\n");
		for(i=0;i<STEPS;i++){
			printf("x:%7.3f  y:%7.3f\n", (double)path[i].x, (double)path[i].y);
		}
		printf("===========================================================\n");
		printf("   vx    dx/dt     vy    dy/dt\n");
		for(i=0;i<STEPS;i++){
			double dx = (double)(path[(i+1)%STEPS].x - path[i].x)/dt;
			double dy = (double)(path[(i+1)%STEPS].y - path[i].y)/dt;
			printf("vx:%7.3f dx/dt:%7.3f  vy:%7.3f dy/dt:%7.3f\n", (double)path[i].vx, dx, (double)path[i].vy, dy);
		}
		printf("===========================================================\n");
		printf("   ax    d^2x/dt     ay    d^2y/dt\n");
		for(i=0;i<STEPS;i++){
			double d2x = (double)(path[(i+1)%STEPS].vx - path[i].vx)/dt;
			double d2y = (double)(path[(i+1)%STEPS].vy - path[i].vy)/dt;
			printf("Ax:%7.3f d2x/dt:%7.3f  Ay:%7.3f d2y/dt:%7.3f\n", (double)path[i].afx, d2x, (double)path[i].afy, d2y);
		}
		printf("===========================================================\n");
		printf("   yaw     yaw_rate\n");
		for(i=0;i<STEPS;i++){
			printf("yaw:%7.1f deg  yaw_rate: %7.1f deg/s\n", (double)(path[i].yaw)*180.0/PI, (double)(path[i].yaw_rate)*180.0/PI);
		}
		printf("===========================================================\n");
	}

	// now set home position
	// this will move later if figure_eight_move_home is enabled
	home_position.time_boot_ms = 0;
	home_position.coordinate_frame = path[0].coordinate_frame;
	home_position.type_mask =   POSITION_TARGET_TYPEMASK_VX_IGNORE |
								POSITION_TARGET_TYPEMASK_VY_IGNORE |
								POSITION_TARGET_TYPEMASK_VZ_IGNORE |
								POSITION_TARGET_TYPEMASK_AX_IGNORE |
								POSITION_TARGET_TYPEMASK_AY_IGNORE |
								POSITION_TARGET_TYPEMASK_AZ_IGNORE |
								POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
	home_position.x = 0.0f;
	home_position.y = 0.0f;
	home_position.z = path[0].z;
	home_position.yaw = path[0].yaw;
	home_position.target_system = 0; // will reset later when sending
	home_position.target_component = AUTOPILOT_COMPID;
	return;
}


static void _send_position_in_path(int i)
{
	if(i>=STEPS || i<0) return;

	mavlink_set_position_target_local_ned_t pos = path[i];
	if(figure_eight_move_home){
		pos.x += home_position.x;
		pos.y += home_position.y;
		pos.z  = home_position.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,pos);

	return;
}

static void _send_home_position(void)
{
	if(figure_eight_move_home){
		mavlink_odometry_t odom = autopilot_monitor_get_odometry();
		home_position.x = odom.x;
		home_position.y = odom.y;
		home_position.z = odom.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,home_position);

	return;
}


static void* _offboard_figure_eight_thread_func(__attribute__((unused)) void* arg)
{
	// for loop sleep
	int64_t next_time = 0;

	int i;

	_init_path();

	//send a few setpoints before starting
	for(i = 100; running && i > 0; --i){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}


HOME:
	// wait for the system to be armed and in offboard mode
	while(running && !autopilot_monitor_is_armed_and_in_offboard_mode()){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
		fflush(stdout);
	}

	// give the system 2 seconds to get to home position
	i = RATE * 2;
	//while(running && i>0){
	while(running && i>0){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		i--;
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}

	// now begin figure 8 path,
	i=0;
	while(running){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		_send_position_in_path(i);
		i++;
		if(i>=STEPS) i=0;
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}

	printf("exiting offboard figure eight thread\n");
	return NULL;
}


int offboard_figure_eight_init(void)
{
	running = 1;
	pipe_pthread_create(&offboard_figure_eight_thread_id, _offboard_figure_eight_thread_func, NULL, OFFBOARD_THREAD_PRIORITY);
	return 0;
}


int offboard_figure_eight_stop(int blocking)
{
	if(running==0) return 0;
	running = 0;
	if(blocking){
		pthread_join(offboard_figure_eight_thread_id, NULL);
	}
	return 0;
}

void offboard_figure_eight_en_print_debug(int debug)
{
	if(debug) en_debug = 1;
}
