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
#include <unistd.h> // usleep

#include "voxl_vision_hub.h"
#include "pipe_channels.h"
#include "config_file.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "macros.h"
#include "horizon_cal.h"
#include "horizon_cal_file.h"
#include "autopilot_monitor.h"

#include <rc_math/ring_buffer.h> // for ring buffer
#include <modal_pipe.h>

// comment out these #defines to allow calibrating on the bench for development
#define ENABLE_ARMED_AND_POSITION_CHECK
#define ENABLE_STATIONARY_CHECK

// uncomment this to debug raw data coming in from VIO and PX4
//#define DEBUG_PRINT_RAW

#define PIPE_SIZE		(64*1024)	// 64k is a typical linux default pipe size
#define READ_BUF_SIZE	(4*1024)	// tidy 1k read buffer

#ifdef PLATFORM_APQ8096
	#define ATTITUDE_MAV_RATE_HZ	100 // 100hz attitude messages in mavlink onboard mode over uart
#else
	#define ATTITUDE_MAV_RATE_HZ	10 // 10hz attitude messages in mavlink onboard mode for QRB5165
#endif

#define PERIOD_S 15 // seconds to collect data for
#define PX4_RINGBUF_LEN				(ATTITUDE_MAV_RATE_HZ*PERIOD_S)
#define PX4_RINGBUF_SAMPLES_TO_USE	((PX4_RINGBUF_LEN*2)/3)
#define VIO_RATE_HZ					30
#define VIO_RINGBUF_LEN				(VIO_RATE_HZ*PERIOD_S)
#define VIO_RINGBUF_SAMPLES_TO_USE	((VIO_RINGBUF_LEN*2)/3)

// if roll/pitch stddev is below this we are likely sitting, not flying
// Seeker reads 0.05 in perfect conditions, VIO Indoors
#define LOWER_TOLERANCE_DEG		0.02

static pthread_t thread;
static int running = 0;
static int print_param = 1;

// calibrate vio horizon at the same time if it happens to be running
// set to 1 at the beginning if vio data comes through
static int cal_vio_too = 0;

// ringbuffers for roll/pitch data and mutex to protect them
static pthread_mutex_t ringbuf_mutex;
static rc_ringbuf_t px4_roll_buf, px4_pitch_buf;
static int px4_buffer_count = 0;
static rc_ringbuf_t vio_roll_buf, vio_pitch_buf;
static int vio_buffer_count = 0;


// px4 param monitoring
static float current_roll_param, current_pitch_param;
static int has_latest_roll_param, has_latest_pitch_param;



static double _rc_ringbuf_mean(rc_ringbuf_t buf, int samples_to_use)
{
	// sanity checks
	if(!buf.initialized){
		fprintf(stderr,"ERROR in rc_ringbuf_mean, ringbuf not initialized yet\n");
		return -1.0f;
	}

	// only average the latest values, not everything
	double mean = 0.0;
	for(int i=0; i<samples_to_use; i++){
		mean += rc_ringbuf_get_value(&buf, i);
	}
	return mean/(double)samples_to_use;
}





static void data_cb(__attribute__((unused)) int ch, char* data, int bytes, \
											__attribute__((unused)) void* context)
{
	// catch possible read errors
	if(bytes<=0) return;

	// remove the trailing newline from echo
	if(bytes>1 && data[bytes-1]=='\n'){
		data[bytes-1]=0;
	}

	if(strncmp(data, CONTROL_COMMAND_START_HORIZON_CALIBRATION, \
						strlen(CONTROL_COMMAND_START_HORIZON_CALIBRATION))==0){
		printf("received command to start horizon calibration\n");
		horizon_cal_init();
		return;
	}

	printf("WARNING: received unknown command through control input pipe!\n");
	printf("got %d bytes. Command is: %s\n", bytes, data);
	return;
}


static void* _thread_func(__attribute__((unused)) void* arg)
{
	mavlink_message_t msg;
	char str[512];

	pipe_server_write_string(HORIZON_CAL_IO_CH, "horizon calibration starting\n");
	print_param = 1;

	while(running){
		usleep(500000);

		// nothing to do untill we get params from px4
		if(!has_latest_roll_param || !has_latest_pitch_param){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "waiting for params\n");

			mavlink_message_t msg;
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), \
				VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, \
				"SENS_BOARD_X_OFF", -1);
			mavlink_io_send_msg_to_ap(&msg);
			mavlink_msg_param_request_read_pack(autopilot_monitor_get_sysid(), \
				VOXL_COMPID, &msg, autopilot_monitor_get_sysid(),AUTOPILOT_COMPID, \
				"SENS_BOARD_Y_OFF", -1);
			mavlink_io_send_msg_to_ap(&msg);

			usleep(2000000);
			continue;
		}

		// done collecting params, no need to print again
		print_param = 0;

#ifdef ENABLE_ARMED_AND_POSITION_CHECK
		if(!autopilot_monitor_is_armed()){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "Please arm and take off in position mode\n");
			usleep(2000000);
			continue;
		}

		if(autopilot_monitor_get_main_mode() != PX4_MAIN_MODE_POSCTL){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "Please put PX4 in position mode\n");
			usleep(2000000);
			continue;
		}
#endif

		// wait for full buffer
		if(px4_buffer_count == 0){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "waiting for first sample\n");
			continue;
		}
		else if(px4_buffer_count < PX4_RINGBUF_LEN){
			sprintf(str, "collected %4d of %4d required PX4 samples\n", px4_buffer_count, PX4_RINGBUF_LEN);
			pipe_server_write_string(HORIZON_CAL_IO_CH, str);
			continue;
		}

		// also wait for the vio buffer if any vio data came in
		else if(vio_buffer_count < VIO_RINGBUF_LEN && vio_buffer_count>0){
			sprintf(str, "collected %4d of %4d required VIO samples\n", vio_buffer_count, VIO_RINGBUF_LEN);
			pipe_server_write_string(HORIZON_CAL_IO_CH, str);
			continue;
		}

		// check if we got any vio data
		if(vio_buffer_count>0) cal_vio_too = 1;
		else cal_vio_too = 0;

		// we have a full buffer, check for conditions met
		pthread_mutex_lock(&ringbuf_mutex);
		double roll_std_dev  = rc_ringbuf_std_dev(px4_roll_buf);
		double pitch_std_dev = rc_ringbuf_std_dev(px4_pitch_buf);
		// calculate new means
		float px4_roll_mean   = (float)_rc_ringbuf_mean(px4_roll_buf, PX4_RINGBUF_SAMPLES_TO_USE);
		float px4_pitch_mean  = (float)_rc_ringbuf_mean(px4_pitch_buf, PX4_RINGBUF_SAMPLES_TO_USE);

		double vio_roll_mean = 0;
		double vio_pitch_mean = 0;
		if(cal_vio_too){
			vio_roll_mean  = _rc_ringbuf_mean(vio_roll_buf, VIO_RINGBUF_SAMPLES_TO_USE) + vio_reported_roll_when_level_deg;
			vio_pitch_mean = _rc_ringbuf_mean(vio_pitch_buf, VIO_RINGBUF_SAMPLES_TO_USE) + vio_reported_pitch_when_level_deg;
		}
		pthread_mutex_unlock(&ringbuf_mutex);


		sprintf(str, "current px4 roll/pitch noise: %6.3f %6.3f  Tolerance:%6.3f\n", roll_std_dev, pitch_std_dev, (double)horizon_cal_tolerance);
		pipe_server_write_string(HORIZON_CAL_IO_CH, str);

		// if both are within bounds, find new offset
		if( roll_std_dev  > (double)horizon_cal_tolerance || \
			pitch_std_dev > (double)horizon_cal_tolerance){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "\nDrone too wobbly, going to keep trying\n");
			usleep(2000000);
			continue;
		}

#ifdef ENABLE_STATIONARY_CHECK
		if(	roll_std_dev  < LOWER_TOLERANCE_DEG || \
			pitch_std_dev < LOWER_TOLERANCE_DEG){
			pipe_server_write_string(HORIZON_CAL_IO_CH, "\nDrone too still, assuming it's not in flight\n");
			usleep(2000000);
			continue;
		}
#endif

		printf("horizon means: px4 R/P: %6.2f %6.2f  vio R/P %6.2f %6.2f\n", \
			(double)px4_roll_mean, (double)px4_pitch_mean, vio_roll_mean, vio_pitch_mean);

		// new parameter values adds the steady state offset to the old ones
		float new_roll_param =  current_roll_param  + px4_roll_mean;
		float new_pitch_param = current_pitch_param + px4_pitch_mean;
		sprintf(str, "\n\nData collection complete, new PX4 horizon values to be written: R:%6.2f  P:%6.2f\n", (double)new_roll_param, (double)new_pitch_param);
		pipe_server_write_string(HORIZON_CAL_IO_CH, str);

		if(cal_vio_too){
			sprintf(str, "\n\nData collection complete, new VIO values to be written: R:%6.2f  P:%6.2f\n", (double)vio_roll_mean, (double)vio_pitch_mean);
			pipe_server_write_string(HORIZON_CAL_IO_CH, str);

			// save the vio cal to disk before landing to make sure it gets saved
			// don't worry, this doesn't adjust the offsets being used while in flight
			// it only updates what's on the disk
			save_horizon_cal_file(vio_roll_mean, vio_pitch_mean);

		}
		else{
			sprintf(str, "\n\nVIO horizon was not calibrated during this routine, only PX4 horizon\n");
			pipe_server_write_string(HORIZON_CAL_IO_CH, str);
		}
		pipe_server_write_string(HORIZON_CAL_IO_CH, "Land and disarm to apply parameters\n");


		// wait for disarm
		while(autopilot_monitor_is_armed() && running){
			usleep(100000);
		}


		// don't save if the module exited while waiting to disarm
		if(!running) continue;


		usleep(100000);

		if(cal_vio_too){
			// update vio offsets
			vio_reported_roll_when_level_deg = vio_roll_mean;
			vio_reported_pitch_when_level_deg = vio_pitch_mean;
		}

		// update px4 params
		pipe_server_write_string(HORIZON_CAL_IO_CH, "\nWriting new params to PX4\n");

		mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
			autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "SENS_BOARD_X_OFF", new_roll_param, MAV_PARAM_TYPE_REAL32);
		mavlink_io_send_msg_to_ap(&msg);
		mavlink_msg_param_set_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
			autopilot_monitor_get_sysid(), AUTOPILOT_COMPID, "SENS_BOARD_Y_OFF", new_pitch_param, MAV_PARAM_TYPE_REAL32);
		mavlink_io_send_msg_to_ap(&msg);

		pipe_server_write_string(HORIZON_CAL_IO_CH, "Horizon Calibration Complete\n");

		running = 0;

	} // end of while(running)


	printf("exiting horizon_cal thread\n");
	return NULL;
}


static void _start(void)
{
	if(running){
		fprintf(stderr, "Warning, not starting horizon cal, it's already running\n");
		return;
	}

	running = 1;
	pthread_create(&thread, NULL, _thread_func, NULL);
}


static void control_pipe_handler(__attribute__((unused)) int ch, char* string, int bytes, __attribute__((unused)) void* context)
{
	// catch possible read errors
	if(bytes<=0) return;

	// remove the trailing newline from echo
	if(bytes>1 && string[bytes-1]=='\n'){
		string[bytes-1]=0;
	}

	if(strncmp(string, CONTROL_COMMAND_START_HORIZON_CALIBRATION, \
						strlen(CONTROL_COMMAND_START_HORIZON_CALIBRATION))==0){
		printf("received command to start horizon calibration\n");
		_start();
		return;
	}

	printf("WARNING: received unknown command through control input pipe!\n");
	printf("got %d bytes. Command is: %s\n", bytes, string);
	return;
}


static void connect_handler(__attribute__((unused)) int ch, __attribute__((unused)) int client_id, __attribute__((unused)) char* client_name, __attribute__((unused)) void* context)
{
	printf("client connected to horizon cal module\n");
	return;
}


static void disconnect_handler(__attribute__((unused)) int ch, __attribute__((unused)) int client_id, __attribute__((unused)) char* name, __attribute__((unused)) void* context)
{
	printf("client disconnected from horizon cal module\n");
	return;
}

int horizon_cal_init(void)
{
	if(running){
		fprintf(stderr, "Warning, not starting horizon cal, it's already running\n");
		return -1;
	}

	// make sure we start with setting the flags as not having received params yet
	has_latest_roll_param = 0;
	has_latest_pitch_param = 0;

	// allocate memory for the ringbuffers
	if(rc_ringbuf_alloc(&px4_roll_buf, PX4_RINGBUF_LEN)) return -1;
	if(rc_ringbuf_alloc(&px4_pitch_buf, PX4_RINGBUF_LEN)) return -1;
	if(rc_ringbuf_alloc(&vio_roll_buf, VIO_RINGBUF_LEN)) return -1;
	if(rc_ringbuf_alloc(&vio_pitch_buf, VIO_RINGBUF_LEN)) return -1;
	px4_buffer_count = 0;
	vio_buffer_count = 0;

	// start the server pipes to send local and fixed data out
	pipe_info_t info = { \
		.name        = HORIZON_CAL_IO_NAME,\
		.location    = HORIZON_CAL_IO_LOCATION,\
		.type        = "text",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = 64*1024};

	// configure optional callbacks
	pipe_server_set_control_cb(HORIZON_CAL_IO_CH, &control_pipe_handler, NULL);
	pipe_server_set_connect_cb(HORIZON_CAL_IO_CH, &connect_handler, NULL);
	pipe_server_set_disconnect_cb(HORIZON_CAL_IO_CH, &disconnect_handler, NULL);

	// start the IO pipe
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;
	if(pipe_server_create(HORIZON_CAL_IO_CH, info, flags)) return -1;
	return 0;
}


int horizon_cal_stop(int blocking)
{
	if(running==0) return 0;
	running = 0;
	if(blocking){
		pthread_join(thread, NULL);
	}
	pipe_server_close(HORIZON_CAL_IO_CH);

	rc_ringbuf_free(&px4_roll_buf);
	rc_ringbuf_free(&px4_pitch_buf);
	rc_ringbuf_free(&vio_roll_buf);
	rc_ringbuf_free(&vio_pitch_buf);
	return 0;
}


int horizon_cal_add_attitude(mavlink_attitude_t attitude)
{
	//don't waste time saving data if we are finished or not running
	if(!running){
		px4_buffer_count = 0;
		return 0;
	}

#ifdef ENABLE_ARMED_AND_POSITION_CHECK
	if(!autopilot_monitor_is_armed() || \
		(autopilot_monitor_get_main_mode() != PX4_MAIN_MODE_POSCTL && \
		 autopilot_monitor_get_main_mode() != PX4_MAIN_MODE_OFFBOARD))
	{
		px4_buffer_count = 0;
		return 0;
	}
#endif

	// try to lock the ringbuf mutex. It might be locked by the other thread,
	// in which case drop the attitude so we don't hold up the uart/px4 monitor thread
	if(pthread_mutex_trylock(&ringbuf_mutex)){
		return 0;
	}

	// save in degrees, not radians
	double new_roll  = (double)attitude.roll  * RAD_TO_DEG;
	double new_pitch = (double)attitude.pitch * RAD_TO_DEG;

	rc_ringbuf_insert(&px4_roll_buf,  new_roll);
	rc_ringbuf_insert(&px4_pitch_buf, new_pitch);
	px4_buffer_count++;

	pthread_mutex_unlock(&ringbuf_mutex);

	#ifdef DEBUG_PRINT_RAW
	printf("px4: %6.1f %6.1f\n", new_roll, new_pitch);
	#endif

	return 0;
}


int horizon_cal_add_level_param_x(float x)
{
	current_roll_param = x;
	has_latest_roll_param = 1;
	px4_buffer_count = 0;
	printf("horizon_cal: Received SENS_BOARD_X_OFF param: %6.3f\n", (double)x);

	if(print_param){
		char str[128];
		sprintf(str, "Current SENS_BOARD_X_OFF param: %6.3f\n", (double)x);
		pipe_server_write_string(HORIZON_CAL_IO_CH, str);
	}

	return 0;
}


int horizon_cal_add_level_param_y(float y)
{
	current_pitch_param = y;
	has_latest_pitch_param = 1;
	px4_buffer_count = 0;
	printf("horizon_cal: Received SENS_BOARD_Y_OFF param: %6.3f\n", (double)y);

	if(print_param){
		char str[128];
		sprintf(str, "Current SENS_BOARD_Y_OFF param: %6.3f\n", (double)y);
		pipe_server_write_string(HORIZON_CAL_IO_CH, str);
	}

	return 0;
}


int horizon_cal_add_vio_roll_pitch(double roll_deg, double pitch_deg)
{
	//don't waste time saving data if we are finished or not running
	if(!running){
		vio_buffer_count = 0;
		return 0;
	}

#ifdef ENABLE_ARMED_AND_POSITION_CHECK
	if(!autopilot_monitor_is_armed() || autopilot_monitor_get_main_mode() != PX4_MAIN_MODE_POSCTL){
		vio_buffer_count = 0;
		return 0;
	}
#endif

	// try to lock the ringbuf mutex. It might be locked by the other thread,
	// in which case drop the attitude so we don't hold up the vio thread
	if(pthread_mutex_trylock(&ringbuf_mutex)){
		return 0;
	}

	// save in degrees, not radians
	rc_ringbuf_insert(&vio_roll_buf,  roll_deg);
	rc_ringbuf_insert(&vio_pitch_buf, pitch_deg);
	vio_buffer_count++;

	pthread_mutex_unlock(&ringbuf_mutex);

	#ifdef DEBUG_PRINT_RAW
	printf("vio: %6.1f %6.1f\n", roll_deg, pitch_deg);
	#endif

	return 0;
}
