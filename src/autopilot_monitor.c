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
#include <pthread.h>

#include <c_library_v2/development/mavlink.h> // include before modal_pipe !!
#include <modal_pipe_server.h>
#include <modal_pipe_interfaces.h>

#include "voxl_vision_hub.h"
#include "config_file.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "autopilot_monitor.h"
#include "pipe_channels.h"
#include "horizon_cal.h"
#include "offboard_wps.h"
#include "misc.h"
#include "geometry.h"
#include "macros.h"

// only restart px4 if its version is older than 1.13
// this only applies to apq8096
#define EKF2_ERROR_RESTART_VERSION_CUTTOFF ((1<<24)+(13<<16))

// connected flag set to 1 by the scrape_data function and set to 0 by the uart
// mavlink module when mavparser times out
static int is_connected=0;

// save sysid of the autopilot
// start with default of 1 that mode APs default to
static uint8_t current_sysid = 1;
static int has_sysid_been_announced = 0;

// autopilot version, e.g. "1.14.0"
static int is_autopilot_version_known = 0;
static uint32_t autopilot_version; // see _handle_autopilot_version for how to decode this
static uint8_t autopilot_version_major; // unused for now, convenience variables
static uint8_t autopilot_version_minor;
static uint8_t autopilot_version_patch;

// RC flight mode selection. Initialize with invalid values
static int is_flight_mode_configuration_known = 0;
#define INVALID_FLIGHT_MODE -2
static int flight_mode_1 = INVALID_FLIGHT_MODE;
static int flight_mode_2 = INVALID_FLIGHT_MODE;
static int flight_mode_3 = INVALID_FLIGHT_MODE;
static int flight_mode_4 = INVALID_FLIGHT_MODE;
static int flight_mode_5 = INVALID_FLIGHT_MODE;
static int flight_mode_6 = INVALID_FLIGHT_MODE;
#define INVALID_RC_CHAN -1
static int flight_mode_rc_chan = INVALID_RC_CHAN;

// Telemetry data that needs to be pulled from the relevant mavlink messages
static uint8_t  heartbeat_base_mode         = 0;
static uint32_t heartbeat_custom_mode       = 0;
static uint8_t  heartbeat_system_status     = 0;

// voltages already converted to volts/percent when read from message
static double   sys_status_battery_volts    = 0.0;
static double   sys_status_battery_percent  = 0.0;

// Flag to indicate if RC system is healthy
static bool rc_healthy = false;

// local position data
static float local_position_z = 0.0;
static float local_position_vz = 0.0;

// protect multi-byte states such as the attitude struct with this mutex
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

// some packets we keep intact for full reference
static mavlink_attitude_t attitude;
static mavlink_odometry_t odometry;
static mavlink_optical_flow_rad_t optical_flow_rad;
static mavlink_attitude_target_t attitude_target;
static mavlink_attitude_quaternion_t attitude_quaternion;

// on qrb5165 this remains 0
static int64_t px4_nanos_ahead_of_voxl = 0;

// Mavlink message is hardcoded to have 18 channels but doesn't define
// a constant for that number
#define MAX_MAVLINK_RC_CHANNELS 18
static int raw_chan_val[MAX_MAVLINK_RC_CHANNELS];

static void _request_autopilot_version()
{
	printf("requesting autopilot_version\n");
	mavlink_message_t msg;

	// send message to reboot autopilot
	mavlink_msg_command_long_pack(  autopilot_monitor_get_sysid(), VOXL_COMPID, &msg,\
									autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1,\
									MAV_CMD_REQUEST_MESSAGE,\
									0, MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0);
	mavlink_io_send_msg_to_ap(&msg);
	return;
}


// always monitor the autopilot sysid in case it changes which may happen
// during setup and config
// This isn't really used other than for debug
// low overhead so keep in case it's useful later
static void _check_sysid(mavlink_message_t* msg)
{
	// even though this is called on messages coming from the autopilot, the AP
	// may be forwarding messages from other components or even the GCS
	// so we run some overly conservative checks to validate this is actually
	// from the AP. Logic is borrowed from pymavlink:
	// https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py#L352
	if(msg->compid == MAV_COMP_ID_GIMBAL) return;

	uint8_t type = mavlink_msg_heartbeat_get_type(msg);
	if( type == MAV_TYPE_GCS	|| \
		type == MAV_TYPE_GIMBAL	|| \
		type == MAV_TYPE_ADSB	|| \
		type == MAV_TYPE_ONBOARD_CONTROLLER)
	{
		return;
	}

	uint8_t autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	if(autopilot == MAV_AUTOPILOT_INVALID) return;

	// okay, if we got to this point we are reasonably confident the message
	// was actually originating from the autopilot. announce if it changed
	// or is the first time detecting it
	if(msg->sysid != current_sysid || !has_sysid_been_announced){
		printf("Detected Autopilot Mavlink SYSID %d\n", msg->sysid);
		has_sysid_been_announced = 1;
	}

	current_sysid = msg->sysid;

	return;
}

// grab modes from the heartbeat and flag as connected
static void _handle_heartbeat(mavlink_message_t* msg)
{
	_check_sysid(msg);

	pthread_mutex_lock(&data_mutex);
	heartbeat_system_status = mavlink_msg_heartbeat_get_system_status(msg);
	heartbeat_base_mode     = mavlink_msg_heartbeat_get_base_mode(msg);
	heartbeat_custom_mode   = mavlink_msg_heartbeat_get_custom_mode(msg);

	// printf("received heartbeat (status, base, custom): %d, %d, %d\n", heartbeat_system_status, heartbeat_base_mode, heartbeat_custom_mode);
	// we just got a heartbeat from the autopilot over uart so flag as connected
	if(!is_connected){
		is_connected = 1;
		printf("Successfully connected to the autopilot through voxl-mavlink-server\n");
	}
	if(!is_autopilot_version_known){
		_request_autopilot_version();
	}
	// Wait until after autopilot version is received so that we know that
	// the autopilot is ready to process these parameter requests.
	else if(!is_flight_mode_configuration_known){
		mavlink_message_t msg;
		if (flight_mode_1 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE1", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_2 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE2", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_3 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE3", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_4 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE4", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_5 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE5", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_6 == INVALID_FLIGHT_MODE){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"COM_FLTMODE6", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if (flight_mode_rc_chan == INVALID_RC_CHAN){
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(),
					VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID,
					"RC_MAP_FLTMODE", -1);
			mavlink_io_send_msg_to_ap(&msg);
		}
		if ((flight_mode_1 != INVALID_FLIGHT_MODE) &&
			(flight_mode_2 != INVALID_FLIGHT_MODE) &&
			(flight_mode_3 != INVALID_FLIGHT_MODE) &&
			(flight_mode_4 != INVALID_FLIGHT_MODE) &&
			(flight_mode_5 != INVALID_FLIGHT_MODE) &&
			(flight_mode_6 != INVALID_FLIGHT_MODE) &&
			(flight_mode_rc_chan != INVALID_RC_CHAN)){
			is_flight_mode_configuration_known = 1;
		}
	}
	pthread_mutex_unlock(&data_mutex);
	return;
}


// copy out useful battery values
static void _handle_sys_status(mavlink_message_t* msg)
{
	// keep local copy of the battery state for future use
	pthread_mutex_lock(&data_mutex);
	sys_status_battery_volts = ((double) mavlink_msg_sys_status_get_voltage_battery(msg)) / 1000.0;
	sys_status_battery_percent = ((double) mavlink_msg_sys_status_get_battery_remaining(msg)) / 100.0;
	rc_healthy = (mavlink_msg_sys_status_get_onboard_control_sensors_health(msg) & MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
	pthread_mutex_unlock(&data_mutex);

	return;
}


// just copy attitude out and send to auto horizon if that's enabled
static void _handle_attitude(mavlink_message_t* msg)
{
	mavlink_attitude_t new_attitude;
	mavlink_msg_attitude_decode(msg, &new_attitude);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	attitude = new_attitude;
	pthread_mutex_unlock(&data_mutex);

	// send to auto horizon module
	horizon_cal_add_attitude(new_attitude);

	// use message timestamp if timestamp sync is working, otherwise just assume
	// current time
	int64_t ts_ns;
	if(px4_nanos_ahead_of_voxl!=0){
		ts_ns = (int64_t)new_attitude.time_boot_ms*1000000 - px4_nanos_ahead_of_voxl;
	}
	else{
		// subtract a millisecond for transport latency (just a guess)
		ts_ns = my_time_monotonic_ns() - 1000000;
	}

	// send to geometry module
	geometry_add_px4_attitude(ts_ns, new_attitude);

	// send out the pipe for normal clients
	static pose_vel_6dof_t mpa_attitude;
	mpa_attitude.magic_number = POSE_VEL_6DOF_MAGIC_NUMBER;
	mpa_attitude.timestamp_ns = ts_ns;

	rc_matrix_t R_att = RC_MATRIX_INITIALIZER;
	rc_rotation_matrix_from_tait_bryan(new_attitude.roll, new_attitude.pitch, new_attitude.yaw, &R_att);
	matrix_to_float(R_att, mpa_attitude.R_child_to_parent);

	mpa_attitude.w_child_wrt_child[0] = new_attitude.rollspeed;
	mpa_attitude.w_child_wrt_child[1] = new_attitude.pitchspeed;
	mpa_attitude.w_child_wrt_child[2] = new_attitude.yawspeed;

	return;
}

static void _handle_attitude_target(mavlink_message_t* msg)
{
	mavlink_attitude_target_t new_attitude_target;
	mavlink_msg_attitude_target_decode(msg, &new_attitude_target);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	attitude_target = new_attitude_target;
	pthread_mutex_unlock(&data_mutex);

	return;
}

static void _handle_attitude_quaternion(mavlink_message_t* msg)
{
	mavlink_attitude_quaternion_t new_attitude_quaternion;
	mavlink_msg_attitude_quaternion_decode(msg, &new_attitude_quaternion);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	attitude_quaternion = new_attitude_quaternion;
	pthread_mutex_unlock(&data_mutex);

	return;
}


// just copy odometry out
static void _handle_odometry(mavlink_message_t* msg)
{
	mavlink_odometry_t new_odometry;
	mavlink_msg_odometry_decode(msg, &new_odometry);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	odometry = new_odometry;
	pthread_mutex_unlock(&data_mutex);

	return;
}



// return 0 if all is well and vvpx4 should pass the message along to UDP
// return 1 if the message should be blocked
static int _handle_param_value(mavlink_message_t* msg)
{
	char param_id[20];
	memset(param_id, 0, 20);
	mavlink_msg_param_value_get_param_id(msg, param_id);


	// check for MAV_1_MODE param if we are enabling the force mav1mode param
	if(en_force_onboard_mav1_mode && strcmp(param_id, "MAV_1_MODE")==0){

		// do the nonsense bitwise conversion from float to int
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		int32_t val_int = *(int32_t*)val_ptr;

		// excellent! mav_1_mode is set to onboard mode
		// return 0 indicating we can pass this message through
		if(val_int==2){
			return 0;
		}

		fprintf(stderr, "WARNING, PX4 MAV_1_MODE not set to onboard! curently: %d\n", val_int);
		fprintf(stderr, "Setting to onboard mode\n");

		// do the nonsense bitwise conversion from int to float
		val_int = 2;
		val_ptr = (void*)&val_int;
		val_f   = *(float*)val_ptr;

		mavlink_message_t msg_set;
		mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg_set, \
				msg->sysid, msg->compid, "MAV_1_MODE", val_f, MAV_PARAM_TYPE_INT32);
		mavlink_io_send_msg_to_ap(&msg_set);

		// return 1 indicating we should NOT pass this message through
		return 1;
	}

	// check for board offset (level) param
	if(strcmp(param_id, "SENS_BOARD_X_OFF")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);
		horizon_cal_add_level_param_x(val);
		return 0;
	}

	// check for board offset (level) param if we enabled the auto leveling feature
	if(strcmp(param_id, "SENS_BOARD_Y_OFF")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);
		horizon_cal_add_level_param_y(val);
		return 0;
	}

	// MPC reuse values for Missions
	if(strcmp(param_id, "MPC_XY_VEL_MAX")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);
		set_mpc_vel_param_v(val);
		return 0;
	}
	if(strcmp(param_id, "MPC_XY_CRUISE")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);	
		set_mpc_xy_cruise_param_v(val);
		return 0;
	}
	if(strcmp(param_id, "MPC_VEL_MANUAL")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);	
		set_mpc_vel_man_param_v(val);
		return 0;
	}
	if(strcmp(param_id, "NAV_ACC_RAD")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);	
		set_mpc_nav_acc_rad(val);
		return 0;
	}
	if(strcmp(param_id, "MPC_TKO_SPEED")==0){
		float val = mavlink_msg_param_value_get_param_value(msg);
		set_mpc_tko_speed(val);
		return 0;
	}

	// Look for RC flight mode configuration
	if (strcmp(param_id, "COM_FLTMODE1")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_1 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "COM_FLTMODE2")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_2 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "COM_FLTMODE3")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_3 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "COM_FLTMODE4")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_4 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "COM_FLTMODE5")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_5 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "COM_FLTMODE6")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_6 = *(int32_t*)val_ptr;
	}
	else if (strcmp(param_id, "RC_MAP_FLTMODE")==0){
		float val_f = mavlink_msg_param_value_get_param_value(msg);
		void* val_ptr = (void*)&val_f;
		flight_mode_rc_chan = *(int32_t*)val_ptr;
	}

	// no special handling was done, return 0 indicating to pass this message
	// through to QGC
	return 0;
}

static void _handle_local_position_ned(mavlink_message_t* msg)
{
	local_position_z = mavlink_msg_local_position_ned_get_z(msg);
	local_position_vz = mavlink_msg_local_position_ned_get_vz(msg);
}

static void _handle_optical_flow_rad(mavlink_message_t* msg)
{
	mavlink_optical_flow_rad_t new_optical_flow_rad;
	mavlink_msg_optical_flow_rad_decode(msg, &new_optical_flow_rad);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	optical_flow_rad = new_optical_flow_rad;
	pthread_mutex_unlock(&data_mutex);
}

// TODO probably need to request this but not needed yet
static void _handle_autopilot_version(mavlink_message_t* msg)
{
	is_autopilot_version_known = 1;
	mavlink_autopilot_version_t new;
	mavlink_msg_autopilot_version_decode(msg, &new);

	uint32_t new_sw = new.flight_sw_version;

	// save local copy
	pthread_mutex_lock(&data_mutex);

	if(new_sw != autopilot_version){
		autopilot_version = new_sw;
		autopilot_version_major =  (new_sw >> (8*3)) & 0xFF;
		autopilot_version_minor =  (new_sw >> (8*2)) & 0xFF;
		autopilot_version_patch =  (new_sw >> (8*1)) & 0xFF;
		is_autopilot_version_known = 1;
		printf("Detected autopilot version: %d.%d.%d\n", autopilot_version_major, autopilot_version_minor, autopilot_version_patch);
	}

	pthread_mutex_unlock(&data_mutex);
}

static void _handle_rc_channels(mavlink_message_t* msg)
{
	mavlink_rc_channels_t rc_chan;
	mavlink_msg_rc_channels_decode(msg, &rc_chan);

	raw_chan_val[0] = rc_chan.chan1_raw;
	raw_chan_val[1] = rc_chan.chan2_raw;
	raw_chan_val[2] = rc_chan.chan3_raw;
	raw_chan_val[3] = rc_chan.chan4_raw;
	raw_chan_val[4] = rc_chan.chan5_raw;
	raw_chan_val[5] = rc_chan.chan6_raw;
	raw_chan_val[6] = rc_chan.chan7_raw;
	raw_chan_val[7] = rc_chan.chan8_raw;
	raw_chan_val[8] = rc_chan.chan9_raw;
	raw_chan_val[9] = rc_chan.chan10_raw;
	raw_chan_val[10] = rc_chan.chan11_raw;
	raw_chan_val[11] = rc_chan.chan12_raw;
	raw_chan_val[12] = rc_chan.chan13_raw;
	raw_chan_val[13] = rc_chan.chan14_raw;
	raw_chan_val[14] = rc_chan.chan15_raw;
	raw_chan_val[15] = rc_chan.chan16_raw;
	raw_chan_val[16] = rc_chan.chan17_raw;
	raw_chan_val[17] = rc_chan.chan18_raw;

	for (int i = rc_chan.chancount; i < MAX_MAVLINK_RC_CHANNELS; i++) {
		raw_chan_val[i] = -1;
	}
}


// functions only for apq8096 with external PX4 flight controller
#ifdef PLATFORM_APQ8096

static void _reboot_px4()
{
	// qrb5165 doesn't support reboot right now, only reboot on apq8096
	mavlink_message_t msg;

	// send message to reboot autopilot
	mavlink_msg_command_long_pack(  autopilot_monitor_get_sysid(), VOXL_COMPID, &msg,\
									autopilot_monitor_get_sysid(), MAV_COMP_ID_AUTOPILOT1,\
									MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,\
									0, 1, 0, 0, 0, 0, 0, 0);
	mavlink_io_send_msg_to_ap(&msg);
	return;
}


// return 0 if all is well and vvpx4 should pass the message along to UDP
// return 1 if the message should be blocked
static int _handle_statustext(mavlink_message_t* msg)
{
	// reset is disabled, do nothing.
	if(! en_reset_px4_on_error){
		return 0;
	}

	// reset is autopilot version dependent, don't do anything if we don't know yet
	if(!is_autopilot_version_known){
		return 0;
	}

	// if PX4 is new enough, no need to restart it anymore
	if(autopilot_version >= EKF2_ERROR_RESTART_VERSION_CUTTOFF){
		return 0;
	}

	char* text = (char*)&_MAV_PAYLOAD(msg)[1]; // pointer to text in the mavlink msg

	// check if the message matches the accel bias error
	if(strcmp(text,"Preflight Fail: High Accelerometer Bias") ==0 ||
	   strcmp(text,"Preflight Fail: Yaw estimate error")      ==0 ||
	   strcmp(text,"Critical failure detected: lockdown")     ==0 ){

		// debug print
		printf("PX4 reported error: %s\n", text);

		// send the accel bias message to UDP
		mavlink_for_ros_send_msg(msg);

		// alert QGC that we are rebooting px4
		mavlink_io_send_text_to_gcs("VOXL is restarting PX4");
		printf("rebooting px4 now\n");
		_reboot_px4();
	}

	// return 0 indicating to forward the original message along
	return 0;
}

#endif


// return 0 for normal messages
// return nonzero to indicate to mavlink-io.c not to forward this msg to ROS
int autopilot_monitor_scrape_data(mavlink_message_t* msg)
{
	switch(msg->msgid){

		// messages handled by both qrb5165 and apq8096
		case MAVLINK_MSG_ID_HEARTBEAT:
			_handle_heartbeat(msg);
			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			_handle_sys_status(msg);
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			_handle_attitude(msg);
			break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
			_handle_attitude_quaternion(msg);
			break;

		case MAVLINK_MSG_ID_ODOMETRY:
			_handle_odometry(msg);
			break;

		case MAVLINK_MSG_ID_PARAM_VALUE:
			return _handle_param_value(msg); // only block in some cases
			break;

		case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
			_handle_autopilot_version(msg);
			break;

		case MAVLINK_MSG_ID_RC_CHANNELS:
			_handle_rc_channels(msg);
			break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			_handle_local_position_ned(msg);
			break;

		case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
			_handle_optical_flow_rad(msg);
			break;

		case MAVLINK_MSG_ID_ATTITUDE_TARGET:
			_handle_attitude_target(msg);
			break;

#ifdef PLATFORM_APQ8096

		case MAVLINK_MSG_ID_STATUSTEXT:
			return _handle_statustext(msg); // only block in some cases
			break;
#endif

		default:
			break;
	}

	return 0; // most messages don't block
}


int autopilot_monitor_is_connected(void)
{
	return is_connected;
}

int autopilot_monitor_is_flight_mode_configuration_known(void)
{
	return is_flight_mode_configuration_known;
}


uint8_t autopilot_monitor_get_sysid(void)
{
	return current_sysid;
}

int autopilot_monitor_is_armed(void)
{
	if(heartbeat_base_mode & MAV_MODE_FLAG_SAFETY_ARMED){
		return 1;
	}
	return 0;
}


px4_main_mode autopilot_monitor_get_main_mode(void)
{
	return (heartbeat_custom_mode&0x00FF0000)>>16;
}


px4_main_mode autopilot_monitor_get_flight_mode_configuration(void)
{
	int raw_channel_value = -1;

	if (autopilot_monitor_is_flight_mode_configuration_known()) {
		int chan_val = autopilot_monitor_get_rc_value(flight_mode_rc_chan);
		if (chan_val > -1) {
			if ((chan_val >= 1000) && (chan_val < 1160)) {
				raw_channel_value = flight_mode_1;
			}
			else if ((chan_val >= 1160) && (chan_val < 1320)) {
				raw_channel_value = flight_mode_2;
			}
			else if ((chan_val >= 1320) && (chan_val < 1480)) {
				raw_channel_value = flight_mode_3;
			}
			else if ((chan_val >= 1480) && (chan_val < 1640)) {
				raw_channel_value = flight_mode_4;
			}
			else if ((chan_val >= 1640) && (chan_val < 1800)) {
				raw_channel_value = flight_mode_5;
			}
			else if ((chan_val >= 1800) && (chan_val <= 2000)) {
				raw_channel_value = flight_mode_6;
			}
		}
	}

	px4_main_mode desired_mode = PX4_MAIN_MODE_UNKNOWN;

	switch (raw_channel_value) {
	case 0:
		desired_mode = PX4_MAIN_MODE_MANUAL;
		break;
	case 1:
		desired_mode = PX4_MAIN_MODE_ALTCTL;
		break;
	case 6:
		desired_mode = PX4_MAIN_MODE_ACRO;
		break;
	case 7:
		desired_mode = PX4_MAIN_MODE_OFFBOARD;
		break;
	default:
		break;
	}

	return desired_mode;
}

px4_sub_mode autopilot_monitor_get_sub_mode(void)
{
	return (heartbeat_custom_mode&0xFF000000)>>24;
}


double autopilot_monitor_get_bat_voltage(void)
{
	return sys_status_battery_volts;
}


double autopilot_monitor_get_bat_percentage(void)
{
	return sys_status_battery_percent;
}

float autopilot_monitor_get_local_position_z(void)
{
	return local_position_z;
}

float autopilot_monitor_get_local_position_vz(void)
{
	return local_position_vz;
}

// TODO Handle Ardupilot modes too
void autopilot_monitor_print_main_mode(void)
{
	px4_main_mode mode = autopilot_monitor_get_main_mode();
	switch(mode){
		case PX4_MAIN_MODE_UNKNOWN :
			printf("UNKNOWN");
			break;
		case PX4_MAIN_MODE_MANUAL :
			printf("MANUAL");
			break;
		case PX4_MAIN_MODE_ALTCTL :
			printf("ALTITUDE");
			break;
		case PX4_MAIN_MODE_POSCTL :
			printf("POSITION");
			break;
		case PX4_MAIN_MODE_AUTO :
			printf("AUTO");
			break;
		case PX4_MAIN_MODE_ACRO :
			printf("ACRO");
			break;
		case PX4_MAIN_MODE_OFFBOARD :
			printf("OFFBOARD");
			break;
		case PX4_MAIN_MODE_STABILIZED :
			printf("STABILIZED");
			break;
		case PX4_MAIN_MODE_RATTITUDE:
			printf("RATTITUDE");
			break;
		default:
			printf("unknown main flight mode");
	}
	return;
}


// used by the offboard mode controllers to see if PX4 is currenly obeying
// offboard commands. returns 1 if armed and in offboard mode, otherwise 0
int autopilot_monitor_is_armed_and_in_offboard_mode(void)
{
	// yes I could make this one big if statement but we may add extra
	// conditions in the future and this is easier to read if it gets longer.
	if(!is_connected) return 0;
	if(!autopilot_monitor_is_armed()) return 0;
	if(autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD) return 0;
	return 1;
}


mavlink_attitude_t autopilot_monitor_get_attitude(void)
{
	mavlink_attitude_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = attitude;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

mavlink_attitude_target_t autopilot_monitor_get_attitude_target(void)
{
	mavlink_attitude_target_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = attitude_target;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

mavlink_attitude_quaternion_t autopilot_monitor_get_attitude_quaternion(void)
{
	mavlink_attitude_quaternion_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = attitude_quaternion;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

mavlink_odometry_t autopilot_monitor_get_odometry(void)
{
	mavlink_odometry_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = odometry;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

mavlink_optical_flow_rad_t autopilot_monitor_get_optical_flow_rad(void)
{
	mavlink_optical_flow_rad_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = optical_flow_rad;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

int autopilot_monitor_get_rpy(float* roll, float* pitch, float* yaw)
{
	if(!is_connected){
		return -1;
	}

	pthread_mutex_lock(&data_mutex);
	*roll = attitude.roll;
	*pitch = attitude.pitch;
	*yaw = attitude.yaw;
	pthread_mutex_unlock(&data_mutex);
	return 0;
}


bool autopilot_monitor_get_rc_state(void)
{
	return rc_healthy;
}


int autopilot_monitor_get_rc_value(int chan)
{
	if ((chan > 0) && (chan <= MAX_MAVLINK_RC_CHANNELS)) {
		return raw_chan_val[chan - 1];
	}

	return -1;
}


void autopilot_monitor_set_connected_flag(int flag)
{
	is_connected = flag;
	return;
}


int autopilot_monitor_init(void)
{
	// nothing to do anymore
	return 0;
}


int autopilot_monitor_stop(void)
{
	// nothing to do anymore
	return 0;
}
