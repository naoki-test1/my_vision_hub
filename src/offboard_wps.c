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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "macros.h"
#include "offboard_wps.h"
#include "misc.h"
#include <modal_pipe_server.h>
#include "state_manager.h"

// Maximum allowed waypoints
#define MAX_WPS 1024
#define MAX_ALT 10000
#define FLIGHT_ALTITUDE	-1.5f
#define RATE			10	// loop rate hz

static int running = 0;
static pthread_t offboard_wps_thread_id;
static pthread_t offboard_wps_photo_thread_id;
static int en_debug = 0;
//static double min_target_dist  = 0.15;

static mavlink_set_position_target_local_ned_t setpoint;
static mavlink_set_position_target_local_ned_t last_setpoint;


static bool reload_mapping = true;
//static bool under_waypoint = false;
//static bool start_planner = false;
static bool use_planner = false;

// FRD/LOCAL NED Cooridnate system
typedef struct _waypoints
{
	double x[MAX_WPS];
	double y[MAX_WPS];
	double z[MAX_WPS];
	int size;
	int next_wp_idx;
	int photo[MAX_WPS];
} Waypoints;

typedef struct pos_vel_t{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
}pos_vel_t;

typedef struct _vfc_pos_setpoint_t
{
	float x_des_position;
	float y_des_position;
	float z_des_position;
	float yaw_des;

	float roll_des;
    float pitch_des;
    float yaw_rate_des;
    float thrust_des;

    float vx_des;
    float vy_des;
    float vz_des;

} vfc_pos_setpoint;


static Waypoints file_wps;
static Waypoints wps;
static char *wp_file_name = "/tmp/voxl-mapper-traj_waypoints.csv";
static double takeoff_alt = 0.0;
float default_mpc_vel_max =-1.0;
float default_mpc_xy_cruise = -1.0;
float default_mpc_vel_man = -1.0;
float default_mpc_nav_acc_rad = 0.5;
float default_mpc_tko_v = 0.5;

vfc_pos_setpoint vfc_setpoint;
pos_vel_t cur_position_pos_vel;
float cur_position_pos_yaw;
float cur_position_pos_yaw_des;
int cur_yaw_direction;
float alpha_lpf_position = 1.0f;
float alpha_lpf_flow = 1.0f;
bool reset_a_des_filt = true;
float ax_des_world_filt = 0;
float ay_des_world_filt = 0;
float gravity = 9.81;
int gravity_dir = -1;  // DOWN NED
bool px4_mpc = false;
bool takeoff_detected = false;


static const char* getfield(char *line, int num)
{
	const char *tok;
	for (tok = strtok(line, ","); tok && *tok; tok = strtok(NULL, ",\n"))
	{
		if (!--num)
			return tok;
	}
	return NULL ;
}

static double map_double(double x, double in_min, double in_max, double out_min,
		double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void set_gravity_direction()
{
	gravity_dir = -1;  // default NEDs
}

static int init_waypoints()
{
	if (reload_mapping)
	{
		file_wps.size = 0;
		file_wps.next_wp_idx = -1;

		printf("\n************ MAPPING MODE get Mapper data... ************\n");
		FILE *stream = fopen(wp_file_name, "r");

		if (stream == NULL)
		{
			printf("WARN: No mapper file\n");
			file_wps.size = 0;
			return 0;
		}

		char line[64];
		while (fgets(line, 64, stream))
		{

			char *tmp = strdup(line);

			if (strlen(tmp) < 5)
			{
				printf("[ERROR] Waypoint data is corrupted, check traj file! %s\n", line);
				printf("[ERROR] Waypoint data is corrupted, check traj file! %s\n", line);
				return 0;
			}

			file_wps.x[file_wps.size] = atof(getfield(tmp, 1));

			free(tmp);
			tmp = strdup(line);
			file_wps.y[file_wps.size] = atof(getfield(tmp, 2));



			// Note the Z should be in FRD space (1:1 to local NED) and
			// thus Z IS negative for going up
			free(tmp);
			tmp = strdup(line);
			const char *alt_str = getfield(tmp, 3);
			// TODO maybe apply US based FAR107 LIMITS for max 400 ft

			if (alt_str != NULL)
			{
				file_wps.z[file_wps.size] = gravity_dir * fabs(atof(alt_str));  // NED NED NED!!!!!
			}
			else
			{
				file_wps.z[file_wps.size] = MAX_ALT; // Do not use Z
			}

			free(tmp);
			tmp = strdup(line);
			const char *photo_str = getfield(tmp, 4);
			if (photo_str != NULL && atof(photo_str) == 1)
			{
				printf("PHOTO ON WP %d\n", file_wps.size);
				file_wps.photo[file_wps.size] = 1;
			}
			else
			{
				file_wps.photo[file_wps.size] = 0;
			}

			free(tmp);

			file_wps.size++;
		}

		for (int z = 0; z < file_wps.size; z++)
		{
			if (file_wps.z[z] == MAX_ALT)
				printf("Plan Waypoints (xyz, FRD/LocalNED): %d %f %f [NO Z USED]\n", z, file_wps.x[z], file_wps.y[z]);
			else
				printf("Plan Waypoints (xyz, FRD/LocalNED): %d %f %f %f %d\n", z, file_wps.x[z], file_wps.y[z], file_wps.z[z], file_wps.photo[z]);
		}

		wps.size = 0;
		wps.next_wp_idx = -1;

		if (wps_stride == 0)
		{
			for (int z = 0; z < file_wps.size; z++)
			{
				wps.x[wps.size] = file_wps.x[z];
				wps.y[wps.size] = file_wps.y[z];
				wps.z[wps.size] = file_wps.z[z];
				wps.photo[wps.size] = file_wps.photo[z];
				wps.size++;
			}
		}
		else
		{
			//  interpolate the wps into the executable array
			for (int z = 0; z < file_wps.size-1; z++)
			{			
				wps.x[wps.size] = file_wps.x[z];
				wps.y[wps.size] = file_wps.y[z];
				wps.z[wps.size] = file_wps.z[z];
				wps.photo[wps.size] = file_wps.photo[z];
				wps.size++;

				// distance between waypoints
//				double distx =  file_wps.x[z+1] - file_wps.x[z];
//				double disty =  file_wps.y[z+1] - file_wps.y[z];
//				double distz =  file_wps.z[z+1] - file_wps.z[z];
//				double dist = sqrt(distx*distx +  disty*disty + distz*distz);
				
				
				double  step_size = 1.0 / (double)wps_stride;
				
				// calc points in between
				 for (double t = step_size; t < 1.0; t += step_size) {
					 wps.x[wps.size] = file_wps.x[z]  + (file_wps.x[z+1]  - file_wps.x[z] ) * t;
					 wps.y[wps.size] = file_wps.y[z]  + (file_wps.y[z+1]  - file_wps.y[z] ) * t;
					 wps.z[wps.size] = file_wps.z[z]  + (file_wps.z[z+1]  - file_wps.z[z] ) * t;
 					 wps.size++;
				  }
			}
		}
		
		for (int z = 0; z < wps.size; z++)
		{
			if (wps.z[z] == MAX_ALT)
				printf("INTERP Waypoints (xyz, FRD/LocalNED): %d %f %f [NO Z USED]\n", z, wps.x[z], wps.y[z]);
			else
				printf("INTERP Waypoints (xyz, FRD/LocalNED): %d %f %f %f %d\n", z, wps.x[z], wps.y[z], wps.z[z], wps.photo[z]);
		}
		reload_mapping = false;
	}

	return wps.size;
}

void offboard_wps_vfc_mission(int vfc_mission)
{
	wps_vfc_mission = (bool)vfc_mission;
}

void offboard_wps_vfc_mission_loop(int vfc_mission_loop)
{
	wps_vfc_mission_loop = (bool)vfc_mission_loop;
}

void offboard_wps_vfc_mission_to_ramp(double vfc_mission_to_ramp)
{
	wps_vfc_mission_to_ramp = vfc_mission_to_ramp;
}

void offboard_wps_vfc_mission_to_kp(double vfc_mission_to_kp)
{
	wps_vfc_mission_to_kp = vfc_mission_to_kp;
}

void offboard_wps_set_pause_time(double timeout)
{
	wps_timeout = timeout;
}

void offboard_wps_set_stride(double stride)
{
	wps_stride = stride;
}

void offboard_wps_damp(double damp)
{
	wps_damp = damp;
}
void offboard_wps_vfc_mission_cruise_speed(double v)
{
	wps_vfc_mission_cruise_speed = v;
}


static void _send_setpoint(void)
{
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),
			VOXL_COMPID, setpoint);
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

	// make takeoff slightly lower than measured?
	if (!autopilot_monitor_is_armed_and_in_offboard_mode() && takeoff_alt == 0.0)
		takeoff_alt = (double)setpoint.z;


//	printf("[%f %f %f]\n", setpoint.x ,setpoint.y ,setpoint.z );

	return;
}

static void _update_setpoint_to_waypoint(void)
{
	setpoint.x = wps.x[wps.next_wp_idx];
	setpoint.y = wps.y[wps.next_wp_idx];

	if (px4_mpc)
	{
		if (wps.z[wps.next_wp_idx] < 0)
		{
			double z_rel = wps.z[wps.next_wp_idx] + takeoff_alt;
			if (z_rel > 0)
			{
				z_rel = 0;
			}
			setpoint.z = z_rel;
		}
	}
	else
	{
		setpoint.z = wps.z[wps.next_wp_idx];
	}

//	printf("_update_setpoint_to_waypoint [%f %f %f] at %d\n", setpoint.x ,setpoint.y ,setpoint.z, wps.next_wp_idx );

	return;
}

static void _update_last_setpoint(void)
{
	last_setpoint.x = setpoint.x;
	last_setpoint.y = setpoint.y;
	last_setpoint.z = setpoint.z;
	
	return;
}

static void reset_vel_behavior()
{
	
	printf("reset_vel_behavior\n");
    mavlink_message_t msg;

    if (default_mpc_vel_max > 0)
    {
		mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, 
				autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "MPC_XY_VEL_MAX", default_mpc_vel_max, MAV_PARAM_TYPE_REAL32);
		mavlink_io_send_msg_to_ap(&msg);
    }
    
    if (default_mpc_xy_cruise > 0)
    {
		mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, 
			autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "MPC_XY_CRUISE", default_mpc_xy_cruise, MAV_PARAM_TYPE_REAL32);
		mavlink_io_send_msg_to_ap(&msg);
    }
    
    if (default_mpc_vel_man > 0)
    {
    	mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, 
    			autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "MPC_VEL_MANUAL", default_mpc_vel_man, MAV_PARAM_TYPE_REAL32);
    	mavlink_io_send_msg_to_ap(&msg);
    }
	
}


void set_mpc_vel_param_v(float v)
{
	if (default_mpc_vel_max <0)
		default_mpc_vel_max = v;
	printf("\tPX4 default MPC_VEL_MAX is %f\n", (double)default_mpc_vel_max);
}

void  set_mpc_xy_cruise_param_v(float v) 
{
	if (default_mpc_xy_cruise <0)
		default_mpc_xy_cruise = v;
	printf("\tPX4 default MPC_XY_CRUISE is %f\n", (double)default_mpc_xy_cruise);
}

void  set_mpc_vel_man_param_v(float v) 
{
	if (default_mpc_vel_man < 0)	
		default_mpc_vel_man = v;
	printf("\tPX4 default MPC_VEL_MANUAL is %f\n", (double)default_mpc_vel_man);
}

void  set_mpc_nav_acc_rad(float v) 
{
	default_mpc_nav_acc_rad = v;
	printf("\tPX4 default NAV_ACC_RAD is %f\n", (double)default_mpc_nav_acc_rad);
}
void set_mpc_tko_speed(float v)
{
	default_mpc_tko_v = v;
	printf("\tPX4 default MPC_TKO_SPEED is %f\n", (double)default_mpc_tko_v);
}


static void* _offboard_wps_thread_func(__attribute__((unused)) void *arg)
{
	int i;
	int64_t next_time = 0;
	// TODO delete or use this variable
	// bool takeoff_detected = false;
	mavlink_message_t msg;

	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), 
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, 
			"MPC_XY_VEL_MAX", -1);
	mavlink_io_send_msg_to_ap(&msg);

	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), 
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, 
			"MPC_XY_CRUISE", -1);	
	mavlink_io_send_msg_to_ap(&msg);

	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), 
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, 
			"MPC_VEL_MANUAL", -1);	
	mavlink_io_send_msg_to_ap(&msg);


	//send a few setpoints before starting
	for (i = 100; running && i > 0; --i)
	{
		_update_setpoint_to_current_position();
		_send_setpoint();
		my_loop_sleep(RATE, &next_time);
	}

	printf("wps running? %d,  is in_armed&offboard? %d\n", (int) running, (int) autopilot_monitor_is_armed_and_in_offboard_mode());	

	double last_vel_req = 0.0;


HOME:

	reset_vel_behavior();
	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), 
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, 
			"NAV_ACC_RAD", -1);	
	mavlink_io_send_msg_to_ap(&msg);

	// wait for the system to be armed and in offboard mode
	// until that is true, keep the setpoint at current position/rotation

	while (running && (autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD))
//	while (running && !autopilot_monitor_is_armed_and_in_offboard_mode())
	{
		
		_send_setpoint();
		my_loop_sleep(RATE, &next_time);
		fflush(stdout);

		// if disarmed then I haven't taken off
//		if (!autopilot_monitor_is_armed())
//		{
//			takeoff_detected = false;
//		}

	}
	
	printf("==>RESTART HOME");
	reload_mapping = true;
	if (init_waypoints() > 0)
	{
		use_planner = true;
	}
	last_vel_req = 0.0;
	

	// now we have broken out of the previous loop, we are in offboard mode!
	while (running)
	{
		// return to home position if px4 falls out of offboard mode or disarms
		if ((autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD))
		{
			printf("Offboard interrupted, going to current hold mode\n");
			use_planner = false;
			goto HOME;
		}

		if (use_planner)
		{
//			double real_timeout =
//					(my_time_monotonic_ns() - last_tag_received_ns);

	        static rc_vector_t _local_pos = RC_VECTOR_INITIALIZER;
	        geometry_get_T_body_wrt_local(&_local_pos);

	        // if (_local_pos.d[2] <= -0.05)
	        // {
	        // 	takeoff_detected  = true;
	        // }

	        double dist_to_wp_x = fabsf(setpoint.x - (float) _local_pos.d[0]);
	        double dist_to_wp_y = fabsf(setpoint.y - (float) _local_pos.d[1]);
	        double dist_to_wp_z = fabsf(setpoint.z - (float) _local_pos.d[2]);
	        double dist_to_wp = sqrt(dist_to_wp_x*dist_to_wp_x + dist_to_wp_y*dist_to_wp_y + dist_to_wp_z*dist_to_wp_z);

	        double vel_max = dist_to_wp * (double)wps_damp; 
	        
	        // clip max
	        if  (dist_to_wp > ((double)default_mpc_xy_cruise*1.25))
	        {
	        	vel_max = default_mpc_xy_cruise;
	        }

	        // clip min
	        if (vel_max <= 0.1)
	        {
	        	vel_max = 0.1;  
	        }
	        
	        // set vel
	    	if (vel_max != last_vel_req && vel_max > 0)
	    	{
				mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, 
					autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "MPC_XY_VEL_MAX", vel_max, MAV_PARAM_TYPE_REAL32);
				mavlink_io_send_msg_to_ap(&msg);
				printf("setting velocity to: (%fm) %f\n", dist_to_wp, vel_max);
	    	}
	    	
	    	last_vel_req = vel_max;
	        
			if (dist_to_wp <= (double)default_mpc_nav_acc_rad)   
			{
				// pause
				double timeout_start =  my_time_monotonic_ns();

				if (wps_timeout > 0)
				{
					while (((my_time_monotonic_ns() - timeout_start)*1e-9) < (double)wps_timeout)  
					{
						//_update_setpoint_to_current_position();
						_send_setpoint();
						my_loop_sleep(RATE, &next_time);
					}
				}
				
				_update_last_setpoint();
				
				wps.next_wp_idx++;

				if (wps_vfc_mission_loop)
				{
					wps.next_wp_idx = wps.next_wp_idx % wps.size;
				}
				else if (wps.next_wp_idx >= wps.size)
				{
					_update_setpoint_to_current_position();
					use_planner = false;
					//
					// ACTIVE LAND MODE
				}
				else
					_update_setpoint_to_waypoint();

				printf("INFO: next waypoint %d of %d\n", wps.next_wp_idx+1, wps.size);
			}
			else
				_update_setpoint_to_waypoint();
		}

		_send_setpoint();
		my_loop_sleep(RATE, &next_time);

	}
	printf("exiting offboard wps thread\n");
	return NULL ;
}


static void limit_xy_mag(float *x, float *y, float max_val)
{
    float mag = sqrtf((*x)*(*x) + (*y)*(*y));

    if (mag > max_val)
    {
        float scale = max_val/mag;
        *x *= scale;
        *y *= scale;
    }

    return;
}


static float constrainf(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    else if (value > max) {
        return max;
    }
    else {
        return value;
    }
}

static void get_current_position()
{
    vio_data_t position_data;

    if (get_latest_state_data(&position_data, 2) == 0) {
    	cur_position_pos_vel.x  = position_data.T_imu_wrt_vio[0];
    	cur_position_pos_vel.y  = position_data.T_imu_wrt_vio[1];
    	cur_position_pos_vel.z  = position_data.T_imu_wrt_vio[2];
    	cur_position_pos_vel.vx = position_data.vel_imu_wrt_vio[0];
    	cur_position_pos_vel.vy = position_data.vel_imu_wrt_vio[1];
    	cur_position_pos_vel.vz = position_data.vel_imu_wrt_vio[2];
    	cur_position_pos_yaw  = atan2f(position_data.R_imu_to_vio[1][0], position_data.R_imu_to_vio[0][0]);
//		printf("pos: (%f, %f, %f)\n", (double)position_data.T_imu_wrt_vio[0], (double)position_data.T_imu_wrt_vio[1], (double)position_data.T_imu_wrt_vio[2]);
    }
    else
    	printf("No OVINS Data\n");
}


static rc_vector_t rc_quaternion_from_rpy(double roll, double pitch, double yaw) {
    rc_vector_t q = RC_VECTOR_INITIALIZER;
    rc_vector_t tb = RC_VECTOR_INITIALIZER;

    double rpy[] = {roll, pitch, yaw};
    rc_vector_from_array(&tb, rpy, 3);

    rc_quaternion_from_tb(tb, &q);

    return q;
}

static mavlink_set_attitude_target_t _fill_attitude_target(float thrust, rc_vector_t q, float yaw_rate) {
    mavlink_set_attitude_target_t attitude_target;

    attitude_target.time_boot_ms = 0;
    attitude_target.target_system = 0;
    attitude_target.target_component = 0;
    attitude_target.type_mask = ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                                ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;
    attitude_target.q[0] = (float)q.d[0];
    attitude_target.q[1] = (float)q.d[1];
    attitude_target.q[2] = (float)q.d[2];
    attitude_target.q[3] = (float)q.d[3];
    attitude_target.body_roll_rate = 0;
    attitude_target.body_pitch_rate = 0;
    attitude_target.body_yaw_rate = yaw_rate;
    attitude_target.thrust_body[0] = 0;
    attitude_target.thrust_body[1] = 0;
    attitude_target.thrust_body[2] = 0;
    attitude_target.thrust = thrust;

    return attitude_target;
}

static void get_roll_pitch_des(  float x_des, float x_est,\
                                 float vx_des, float vx_est,\
                                 float y_des, float y_est, \
                                 float vy_des, float vy_est,\
                                 float yaw, float kp_xy, float kd_xy, \
                                 float alpha_lpf, \
                                 float *roll_des, float *pitch_des)
{
    float ax_des_world_raw = kp_xy * (x_des - x_est) + kd_xy * (vx_des - vx_est);
    float ay_des_world_raw = kp_xy * (y_des - y_est) + kd_xy * (vy_des - vy_est);

    if (reset_a_des_filt)
    {
        ax_des_world_filt = ax_des_world_raw;
        ay_des_world_filt = ay_des_world_raw;
        reset_a_des_filt = false;
    }
    else
    {
        // low pass filter the desired world acceleration
        ax_des_world_filt = alpha_lpf * ax_des_world_raw  + (1 - alpha_lpf) * ax_des_world_filt;
        ay_des_world_filt = alpha_lpf * ay_des_world_raw  + (1 - alpha_lpf) * ay_des_world_filt;
    }

    // yaw is the yaw angle of body w.r.t. to the x y data frame
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);

    float ax_des = ax_des_world_filt*cos_yaw + ay_des_world_filt*sin_yaw;
    float ay_des = -ax_des_world_filt*sin_yaw + ay_des_world_filt*cos_yaw;

    float a_des = sqrtf(ax_des*ax_des + ay_des*ay_des);

    if (a_des < 0.0001f) {
        *roll_des = 0;
        *pitch_des = 0;
        return;
    }

    float a_max = gravity*tanf(vfc_params.tilt_max);

    float tilt_angle = 0;

    if (a_des < a_max) {
        tilt_angle = atanf(a_des/gravity);
    }
    else {
        tilt_angle = vfc_params.tilt_max;
    }

    *roll_des = ay_des/a_des * tilt_angle;
    *pitch_des = -ax_des/a_des * tilt_angle;

    if (en_debug) {
        printf("ax, ay, roll_des, pitch_des: %f, %f, %f, %f\n", (double)ax_des, (double)ay_des, (double)*roll_des, (double)*pitch_des);
    }
}

static float get_thrust_altitude(float z_desired, float z_est, float vz_desired, float vz_est, float kp_z, float kd_z) {
    //TODO, compensate for tilt of vehicle

    //positive z is down
    float acc_des = kp_z * (z_desired - z_est) + kd_z * (vz_desired - vz_est);

    //positive thrust is up
    float thrust_des = vfc_params.thrust_hover * (1.0f - acc_des/gravity);

    thrust_des = constrainf(thrust_des, vfc_params.min_thrust, vfc_params.max_thrust);

    return thrust_des;
}


static float get_thrust_altitude_takeoff(float v_int, float z_desired, float z_est, float vz_desired, float vz_est, float kp_z, float kd_z) {

    //positive z is down
    float acc_des = kp_z * (z_desired - z_est) + kd_z * (vz_desired - vz_est);

    float thrust_des = (vfc_params.thrust_hover * (1.0f - acc_des/gravity)) * v_int;

    float limit_max = vfc_params.thrust_hover * (float)1.25;

    thrust_des = constrainf(thrust_des, vfc_params.min_thrust, limit_max);

    return thrust_des;
}

static double calculate_heading_angle(double x0, double y0, double x1, double y1) {
    double dx = x1 - x0;
    double dy = y1 - y0;

    if (dx == 0 && dy == 0) {
        return 0; // Or any arbitrary angle
    }

    double angle_radians = atan2(dy, dx);

    printf("\t\t\t%f %f %f\n", dy, dx, angle_radians);

    // lfet turn or right

    return angle_radians;
}


static void _send_setpoint_vfc(double velocity_magnitude, bool takeoff_clamp, bool wp_clamp)
{
	get_current_position();


	float xy_vel_max = vfc_params.vxy_max*0.8f;
	float z_vel_max = vfc_params.vz_max*0.8f;
	float accel_jerk_max = vfc_params.xy_acc_limit_vio*0.25;
	float vel_ff = vfc_params.vel_ff_factor_vio*0.25;
	float xy_kp = vfc_params.kp_xy_vio*0.8f;
	float xy_kd = vfc_params.kd_xy_vio*0.8f;
	float to_z_kp = vfc_params.kp_z_vio*0.3f;
	float to_z_kd = vfc_params.kd_z_vio*0.3f;
	float wp_z_kp = vfc_params.kp_z_vio*0.5f;
	float wp_z_kd = vfc_params.kd_z_vio*0.5f;


    vfc_setpoint.x_des_position = setpoint.x;
    vfc_setpoint.y_des_position = setpoint.y;
    vfc_setpoint.z_des_position = setpoint.z;

    // localized
    vfc_setpoint.yaw_des = cur_position_pos_yaw_des;


    if (vfc_setpoint.yaw_des < -PI_F) vfc_setpoint.yaw_des += (TWO_PI_F);
    if (vfc_setpoint.yaw_des > PI_F) vfc_setpoint.yaw_des -= (TWO_PI_F);


    rc_vector_t q_yaw = rc_quaternion_from_rpy(0, 0, vfc_setpoint.yaw_des);

    float vx_des_body, vy_des_body, vz_des_body, cos_yaw, sin_yaw;
    float vx_des_body_limited = 0;
    float vy_des_body_limited = 0;
    const float dt = 1.0f/vfc_params.rate;

    // MAX vel is checked in limit_xy_mag

    double dx = vfc_setpoint.x_des_position - cur_position_pos_vel.x;
    double dy = vfc_setpoint.y_des_position - cur_position_pos_vel.y;
    double dz = vfc_setpoint.z_des_position - cur_position_pos_vel.z;
    double direction_magnitude = sqrt(dx * dx + dy * dy + dz * dz);

    if (direction_magnitude != 0) {
        double unit_vector_x = dx / direction_magnitude;
        double unit_vector_y = dy / direction_magnitude;
        double unit_vector_z = dz / direction_magnitude;
        vx_des_body = velocity_magnitude * unit_vector_x;
        vy_des_body = velocity_magnitude * unit_vector_y;
        vz_des_body = velocity_magnitude * unit_vector_z;

        limit_xy_mag(&vx_des_body, &vy_des_body, xy_vel_max);

        int z_sign = 1;
        if (vz_des_body < 0)
        	z_sign = -1;

        float target_vz = z_vel_max;
        if (wp_clamp)
        {
        	target_vz = z_vel_max * 0.80f;
        	if (target_vz < 0.05)
        	{
        		target_vz = 0.05;
        	}
        }

        if (fabs(vz_des_body) > (double)target_vz)
        	vz_des_body = z_sign * target_vz;
    }
    else
    {
    	vx_des_body = 0;
    	vy_des_body = 0;
    	vz_des_body = 0;
    }


    if (accel_jerk_max > 0) {
        //vx_des_body and vy_des_body are the targets
        float delta_vx = vx_des_body - vx_des_body_limited;
        float delta_vy = vy_des_body - vy_des_body_limited;

        limit_xy_mag(&delta_vx, &delta_vy, dt*(accel_jerk_max));

        vx_des_body_limited += delta_vx;
        vy_des_body_limited += delta_vy;
    }
    else {
        vx_des_body_limited = vx_des_body;
        vy_des_body_limited = vy_des_body;
    }

    cos_yaw = cosf(cur_position_pos_yaw);
    sin_yaw = sinf(cur_position_pos_yaw);

    // FAST start, slow down as we get to destination
    vfc_setpoint.vx_des = vx_des_body_limited*cos_yaw - vy_des_body_limited*sin_yaw;
    vfc_setpoint.vy_des = vx_des_body_limited*sin_yaw + vy_des_body_limited*cos_yaw;

    float vx_ff = 0;
    float vy_ff = 0;
    vx_ff = vfc_setpoint.vx_des*(vel_ff);
    vy_ff = vfc_setpoint.vy_des*(vel_ff);

    get_roll_pitch_des(	vfc_setpoint.x_des_position,
    					cur_position_pos_vel.x,
						vx_ff,
						cur_position_pos_vel.vx,
						vfc_setpoint.y_des_position,
						cur_position_pos_vel.y,
						vy_ff,
						cur_position_pos_vel.vy,
						cur_position_pos_yaw,
						xy_kp,
						xy_kd,
						alpha_lpf_position,
						&vfc_setpoint.roll_des,
						&vfc_setpoint.pitch_des);


    float tilt_angle_des = sqrtf(vfc_setpoint.roll_des*vfc_setpoint.roll_des + vfc_setpoint.pitch_des*vfc_setpoint.pitch_des);

    rc_vector_t tilt_axis = RC_VECTOR_INITIALIZER;
    rc_vector_t q_tilt = RC_VECTOR_INITIALIZER;
    rc_vector_zeros(&tilt_axis, 3);

    if (tilt_angle_des > 0.0001f) {
        tilt_axis.d[0] = vfc_setpoint.roll_des/tilt_angle_des;
        tilt_axis.d[1] = vfc_setpoint.pitch_des/tilt_angle_des;
    }
    else {
        tilt_axis.d[0] = 1.0;
    }

    if (tilt_angle_des > vfc_params.tilt_max) {
        tilt_angle_des = vfc_params.tilt_max;
    }
    rc_axis_angle_to_quaternion(tilt_axis, tilt_angle_des, &q_tilt);

    rc_vector_t q_des = RC_VECTOR_INITIALIZER;

    // Q_BODYDES_WORLD = Q_BODYDES_YAWFRAME * Q_YAWFRAME_WORLD
    rc_quaternion_multiply(q_tilt, q_yaw, &q_des);

    // THURST AND Z VEL
    vfc_setpoint.vz_des = vz_des_body;


    if (takeoff_clamp)
    {
		vfc_setpoint.thrust_des = get_thrust_altitude_takeoff(
				velocity_magnitude,
				vfc_setpoint.z_des_position,
				cur_position_pos_vel.z,
				vfc_setpoint.vz_des,
				cur_position_pos_vel.vz,
				to_z_kp,
				to_z_kd);
    }
    else
    {
    	if (wp_clamp)
    	{
    		vfc_setpoint.thrust_des = get_thrust_altitude(
    				vfc_setpoint.z_des_position,
    				cur_position_pos_vel.z,
    				vfc_setpoint.vz_des,
    				cur_position_pos_vel.vz,
					wp_z_kp,
					wp_z_kd);
    	}
    	else
    	{
    		vfc_setpoint.thrust_des = get_thrust_altitude(
    				vfc_setpoint.z_des_position,
    				cur_position_pos_vel.z,
    				vfc_setpoint.vz_des,
    				cur_position_pos_vel.vz,
    				vfc_params.kp_z_vio,
    				vfc_params.kd_z_vio);

    	}

    }

    if (en_debug)
    	printf("ID %d DistWP: %f %f %f (%f) :  cur: [%f %f %f] wp: [%f %f %f]  wp-RPY: [%f, %f, %f, %f] [%f %f %f]\n",
						wps.next_wp_idx,
						velocity_magnitude,
						(double)setpoint.x,
						(double)setpoint.y,
						(double)setpoint.z,
						(double)cur_position_pos_vel.x,
						(double)cur_position_pos_vel.y,
						(double)cur_position_pos_vel.z,
						(double)vfc_setpoint.x_des_position,
						(double)vfc_setpoint.y_des_position,
						(double)vfc_setpoint.z_des_position,
						(double)vfc_setpoint.pitch_des,
						(double)vfc_setpoint.roll_des,
						(double)vfc_setpoint.thrust_des,
						(double)vfc_setpoint.yaw_des,
						(double)vz_des_body,
						(double)vfc_setpoint.vz_des,
						(double)vfc_setpoint.thrust_des
									   );


    // SMOOTHING
//        float time_since_transition = my_time_monotonic_ns()/1.e9f - t_transition;
//        if (time_since_transition >= 0 && time_since_transition < vfc_params.att_transition_time) {
//            float time_ratio = constrainf(time_since_transition/vfc_params.att_transition_time, 0.0f, 1.0f);
//
//            rc_vector_t q_des_final = RC_VECTOR_INITIALIZER;
//            rc_quaternion_slerp(q_at_transition, q_des, time_ratio, &q_des_final);
//
//            // printf("quat smoothing: %f\n", (double)time_ratio);
//            // rc_vector_print(q_at_transition);
//            // rc_vector_print(q_des);
//            // rc_vector_print(q_des_final);
//
//            q_des = q_des_final;
//        }

    mavlink_set_attitude_target_t attitude_target;
    attitude_target = _fill_attitude_target(vfc_setpoint.thrust_des, q_des, vfc_setpoint.yaw_rate_des);
    mavlink_io_send_attitude_target(autopilot_monitor_get_sysid(), VOXL_COMPID, attitude_target);


	return;
}

static void* _take_photos_thread_func(__attribute__((unused)) void *arg)
{
	int status;

	status = system("/usr/bin/voxl-send-command hires_snapshot snapshot");
	sleep(1);
	status = system("/usr/bin/voxl-send-command hires_snapshot snapshot");
	sleep(1);
	status = system("/usr/bin/voxl-send-command hires_snapshot snapshot");
	sleep(1);

	return NULL ;
}


static void trigger_photo()
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&offboard_wps_photo_thread_id, NULL, _take_photos_thread_func,
						NULL);
    if (pthread_create(&offboard_wps_photo_thread_id, &attr, _take_photos_thread_func, NULL) != 0) {
        perror("Cannot take photo");
        return;
    }
    pthread_attr_destroy(&attr); // Clean up attribute

}


#define RAMP_TIME  3
#define BRAKE_TIME 2



static void* _offboard_wps_thread_vfc_func(__attribute__((unused)) void *arg)
{

	// TODO delete or use this variable
	int64_t next_time = 0;
	int print_once = 0;

	mavlink_message_t msg;

	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
			"NAV_ACC_RAD", -1);
	mavlink_io_send_msg_to_ap(&msg);

	mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
			VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
			"MPC_TKO_SPEED", -1);
	mavlink_io_send_msg_to_ap(&msg);


	// TODO: FIX
	for (int i=0; i<10; i++)
	{
		my_loop_sleep(RATE, &next_time);
		set_gravity_direction();
	}
	printf("Gravity direction NED %s (%d)\n", gravity_dir > 0 ? "UP" : "DOWN", gravity_dir);

HOME:
	// wait for the system to be armed and in offboard mode
	// until that is true, keep the setpoint at current position/rotation
	while (running && (autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD))
	{
        _update_setpoint_to_current_position();
        _send_setpoint_vfc(0, false, false);
		my_loop_sleep(RATE, &next_time);
		fflush(stdout);

//		if isarmed then I haven't taken off
		if (!autopilot_monitor_is_armed())
		{
			if (!print_once)
				printf("====> Reset Mission\n");
			print_once = 1;

			// reset takeoff flag, disarmed means I have to be on the ground.
			takeoff_detected = false;
		}

	}

	printf("==>RESTART HOME");
	reload_mapping = true;
	if (init_waypoints() > 0)
	{
		printf("Mission loaded!\n");
		use_planner = true;

		// if disarmed start with takeoff WP
		if (!takeoff_detected)
		{
			wps.next_wp_idx = -1;
		}
		else
		{
			///////////////////////////////////////
			// ELSE it will continue on 0 for loading new points in flight
			///////////////////////////////////////
			wps.next_wp_idx = 1;
		}

		printf("==>Running Mission starting at %d\n", wps.next_wp_idx);

	}

	if (running)
	{
		mavlink_message_t msg;
		char msg_string[48];
		strncpy(msg_string, "VFC MISSION", 48);

		// This will produce message on GCS and announce with audio
		mavlink_msg_statustext_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
									MAV_SEVERITY_CRITICAL, msg_string, 0, 0);
		(void) mavlink_io_send_msg_to_gcs(&msg);
	}

	// Start first point
	printf("NAV ACC: %f\n",(double)default_mpc_nav_acc_rad);

	_update_setpoint_to_waypoint();

	double takeoff_ramp_vel = 0;
	double takeoff_ramp_time = 0;
	bool delay_takeoff = true;
	bool delay_next_wp = false;
	int  fix_yaw_stage = 0;
	double wp_vel = 0;


	double interval = 0;
	double target_dv =  (double) wps_vfc_mission_cruise_speed / (BRAKE_TIME * RATE);
	cur_position_pos_yaw_des = 0;

	printf("STARTING MISSION NOW WP: %d of %d\n", wps.next_wp_idx, wps.size);

	// now we have broken out of the previous loop, we are in offboard mode!
	while (running)
	{
		// return to home position if px4 falls out of offboard mode or disarms
		if ((autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD))
		{
			mavlink_message_t msgc;
			char msg_string[48];
			strncpy(msg_string, "MISSION CANCEL", 48);
			mavlink_msg_statustext_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msgc, \
										MAV_SEVERITY_CRITICAL, msg_string, 0, 0);
			(void) mavlink_io_send_msg_to_gcs(&msgc);

			printf("Offboard interrupted, going to current hold mode\n");
			use_planner = false;
			goto HOME;
		}

		// Reuse for velocity
		double dist_to_wp = 0;

		if (use_planner)
		{
			double dist_to_wp_x = fabsf(setpoint.x - (float) cur_position_pos_vel.x);
	        double dist_to_wp_y = fabsf(setpoint.y - (float) cur_position_pos_vel.y);
	        double dist_to_wp_z = fabsf(setpoint.z - (float) cur_position_pos_vel.z);
	        // dist needs to reused for holding vs PX4
	        dist_to_wp = sqrt(dist_to_wp_x*dist_to_wp_x + dist_to_wp_y*dist_to_wp_y + dist_to_wp_z*dist_to_wp_z);

	        if (en_debug)
	        {
	        	printf("(%d) Distance to WP  %f (%f %f %f)   --   (%f %f %f)  PHOTO? %d\n",
	        			wps.next_wp_idx,
						dist_to_wp,
	        			(double)setpoint.x,
						(double)setpoint.y,
						(double)setpoint.z,
						(double)cur_position_pos_vel.x,
						(double)cur_position_pos_vel.y,
						(double)cur_position_pos_vel.z,
						wps.photo[wps.next_wp_idx] );
	        }


			if (dist_to_wp <= (double)default_mpc_nav_acc_rad)
			{

				if (wps.photo[wps.next_wp_idx] == 1)
				{
					printf("TRIGGER PHOTO\n");
					trigger_photo();
				}

				// pause
				double timeout_start =  my_time_monotonic_ns();

				if (wps_timeout > 0)
				{
					while (((my_time_monotonic_ns() - timeout_start)*1e-9) < (double)wps_timeout)
					{
						//_update_setpoint_to_current_position();
						//_send_setpoint_vfc(dist_to_wp, dist_to_wp);
						_send_setpoint_vfc(0, false, false);
						my_loop_sleep(RATE, &next_time);
					}
				}

				// critical here
				_update_last_setpoint();

				wps.next_wp_idx++;

				if (!wps_vfc_mission_loop)
				{
					if (wps.next_wp_idx >= wps.size)
					{
				        _update_setpoint_to_current_position();
						use_planner = false;
					}
					else
					{
						_update_setpoint_to_waypoint();
					}

				}
				else
				{
					wps.next_wp_idx = wps.next_wp_idx % wps.size;

					// IF I'm repeating the mission, skip takeoff as idx = 0
					if (takeoff_detected && wps.next_wp_idx == 0)
						wps.next_wp_idx = 1;

					if (en_debug)
					{
						printf("INFO: next waypoint %d of %d\n", wps.next_wp_idx+1, wps.size);
					}

					_update_setpoint_to_waypoint();
				}

				takeoff_detected = true;
				delay_next_wp  = true;
				fix_yaw_stage = 0;

				printf("[INFO] (%f) Going to Next WP %d \t(%f %f %f)\n",
						dist_to_wp,
						wps.next_wp_idx,
						(double)setpoint.x,
						(double)setpoint.y,
						(double)setpoint.z);

			}
			else
				_update_setpoint_to_waypoint();
		}

		if (wps.next_wp_idx <= 0)  // first WP is always takeoff point
		{
		    vfc_setpoint.yaw_des = 0;  // for now

			// in case the op moves the drone while it is disarmed!
			if (wps.next_wp_idx == -1)
				wps.next_wp_idx = 0;

			if (print_once)
			{
				print_once = false;
				printf("[INFO] TAKEOFF speed %f(%f) vs %f to (%f %f %f)\n",
						takeoff_ramp_vel,
						(double) wps_vfc_mission_to_ramp,
						(double)default_mpc_tko_v,
						(double)setpoint.x,
						(double)setpoint.y,
						(double)setpoint.z);
			}

			// DELAY to allow motors to reach min RPM
			if (delay_takeoff)
			{
			    // COMMENT IF BLOCK IF HAND TESTING MODE
				if (!autopilot_monitor_is_armed())
				{
					interval =  my_time_monotonic_ns();
				}
			    // COMMENT IF BLOCK IF HAND TESTING MODE

				if (((my_time_monotonic_ns() - interval)*1e-9) > RAMP_TIME)
				{
					delay_takeoff = false;
				}

				takeoff_ramp_vel = 0;
			}
			else
			{
				if ((-1.0 * cur_position_pos_vel.z) <= (default_mpc_tko_v / 2))
				{
					if (takeoff_ramp_vel < 0.98)
					{
						double ramp_within = wps_vfc_mission_to_ramp;  // ramp within 3 seconds
						double max_y = 1.0; // throttle 100%
						double a = -4.0 * max_y / (ramp_within * ramp_within);
						double h = ramp_within / 2.0;
						double k = max_y;

						takeoff_ramp_vel = a * (takeoff_ramp_time - h) * (takeoff_ramp_time - h) + k;

						takeoff_ramp_time += (double)wps_vfc_mission_to_kp;

//						printf("[INFO] TAKEOFF next %f (%f / %f / %f)\n",
//								takeoff_ramp_vel, takeoff_ramp_time, wps_vfc_mission_to_ramp, wps_vfc_mission_to_kp);

						if (takeoff_ramp_vel < 0)
							takeoff_ramp_vel = 0;

					}
					else
					{
						takeoff_ramp_vel = 1.0;
					}

					interval =  my_time_monotonic_ns();
				}
			}

			printf("[INFO] TAKEOFF next %f (%f vs %f) \n",
					takeoff_ramp_vel, cur_position_pos_vel.z*-1.0, default_mpc_tko_v);

			// NED
			_send_setpoint_vfc(takeoff_ramp_vel, true, false);
		}
		else
		{
			static int cd = 0;

			bool slow_z = false;

			// ramp up
			if (delay_next_wp)
			{
				if (fix_yaw_stage == 0)
				{
				    double yaw_rads =  calculate_heading_angle(last_setpoint.x, last_setpoint.y,
				    		setpoint.x, setpoint.y);
				    yaw_rads = round(yaw_rads * 10000.0) / 10000.0;
				    setpoint.yaw = yaw_rads;
					fix_yaw_stage = 1;
				}

				double yaw_delta = setpoint.yaw - cur_position_pos_yaw;
				if (fix_yaw_stage == 1 && fabs(yaw_delta) >= 0.1)
				{
					printf("\t\tYAW DIFF: %f \t\tyawDest: %f <-- yawOrig: %f\n", yaw_delta, setpoint.yaw, cur_position_pos_yaw);
					wp_vel = 0;

					if (yaw_delta < 0)
						cur_yaw_direction = -1;
					else
						cur_yaw_direction = 1;

					cur_position_pos_yaw_des += cur_yaw_direction * 0.125;

				}
				else
				{
					fix_yaw_stage = 2;
//					cur_position_pos_yaw_des = 0;
				}

				if (fix_yaw_stage == 2)
				{
					if (wp_vel < (double)wps_vfc_mission_cruise_speed)
					{
						wp_vel += target_dv;
					}
					else
					{
						delay_next_wp = false;
						wp_vel = wps_vfc_mission_cruise_speed;
					}
				}

				slow_z = true;

				printf("[INFO] WP Velocity %f - %f\n",
										wp_vel, vfc_setpoint.yaw_des);

			}
			else
			{
				if (fabs(dist_to_wp) < ((double)wps_vfc_mission_cruise_speed * BRAKE_TIME))
				{
					wp_vel = dist_to_wp;
					if (wp_vel < 0.05)
						wp_vel = 0.05;

					printf("[INFO] WP Velocity %f - %f\n",
						wp_vel, dist_to_wp);

				}
				else
				{
					if (wp_vel > (double)wps_vfc_mission_cruise_speed)
						wp_vel = wps_vfc_mission_cruise_speed;

					if ((cd++ % RATE) == 0)
						printf("[INFO] WP Velocity %f - %f\n",
							wp_vel, dist_to_wp);
				}

				slow_z = false;
			}

			_send_setpoint_vfc(wp_vel, false, slow_z);

		}

		my_loop_sleep(RATE, &next_time);

	}
	printf("exiting offboard wps thread\n");
	return NULL ;
}



int offboard_wps_init(void)
{
	running = 1;
	printf("STARTING WPS module\n");
	setpoint.time_boot_ms = 0;
	setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;
	//setpoint.coordinate_frame = MAV_FRAME_LOCAL_FRD;
	setpoint.type_mask =   
		    POSITION_TARGET_TYPEMASK_VX_IGNORE |
		    POSITION_TARGET_TYPEMASK_VY_IGNORE |
		    POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            POSITION_TARGET_TYPEMASK_AX_IGNORE |
            POSITION_TARGET_TYPEMASK_AY_IGNORE |
            POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
	setpoint.x = 0.0f;
	setpoint.y = 0.0f;
	setpoint.z = 0.0f;
    setpoint.vx = 0.0f;
    setpoint.vy = 0.0f;
    setpoint.vz = 0.0f;
    setpoint.afx = 0.0f;
    setpoint.afy = 0.0f;
    setpoint.afz = 0.0f;
    setpoint.yaw = 0.0f;
    setpoint.yaw_rate = 0.0f;
    setpoint.target_system = 0; // will reset later when sending
    setpoint.target_component = AUTOPILOT_COMPID;

    if (wps_vfc_mission)
    {
    	printf("WPS Mode Active, using VFC Mission MODE as Offboard!\n");
    	pipe_pthread_create(&offboard_wps_thread_id, _offboard_wps_thread_vfc_func,
							NULL, OFFBOARD_THREAD_PRIORITY);
    }
    else
    {
    	printf("WPS Mode Active, using PX4-MPC Mission MODE!\n");
    	pipe_pthread_create(&offboard_wps_thread_id, _offboard_wps_thread_func,
    						NULL, OFFBOARD_THREAD_PRIORITY);
    }

	return 0;
}

int offboard_wps_stop(int blocking)
{
	if (running == 0)
		return 0;
	running = 0;
	if (blocking)
	{
		pthread_join(offboard_wps_thread_id, NULL);
	}
	return 0;
}

void offboard_wps_en_print_debug(int debug)
{
		if (debug)
			en_debug = 1;
}
