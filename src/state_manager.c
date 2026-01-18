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

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <rc_math.h>
#include <modal_pipe_client.h>
#include <modal_pipe_server.h>

#include "macros.h"
#include "state_manager.h"
#include "pipe_channels.h"
#include "vio_manager.h"
#include "misc.h"
#include "config_file.h"


// data from voxl-state-estimator, used for Optic flow only right now
static vio_data_t latest_state  = {0};

// straight from either internal VIO source (qvio or ov) or HITL
static vio_data_t latest_state2 = {0};

// TODO protect everything with mtx, just proving things work right now
// static pthread_mutex_t mtx[N_STATES];


// called when new valid data is received through the VIO pipe helper
void state_manager_add_new_data(vio_data_t d, int id)
{
	if (id == 1) {
		memcpy(&latest_state, &d, sizeof (vio_data_t));
	}
	else if (id == 2) {
		//printf("adding 2\n");
		memcpy(&latest_state2, &d, sizeof (vio_data_t));
	}
}

// This is the callback assigned to the client pipe interface. It checks for
// server disconnects and validates the received vio data before processing it.
// Multiple packets may come through at once although that's not typical.
// this is fetching data from the external voxl-state-estimator
static void state_pipe_helper_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// validate that the data makes sense
	int n_packets;
	vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);
	if(data_array == NULL){
		pipe_client_flush(ch);
		return;
	}

	// just grab the last state
	state_manager_add_new_data(data_array[n_packets-1], 1);

	if(n_packets>2){
		printf("state data backed up\n");
	}
	return;
}

// // this is processing data from an internal source (vio_manager.c or HITL)
// static void state2_pipe_helper_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
// {
// 	// validate that the data makes sense
// 	int n_packets;
// 	vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);
// 	if(data_array == NULL){
// 		pipe_client_flush(ch);
// 		return;
// 	}

// 	// just grab the last state
// 	state_manager_add_new_data(data_array[n_packets-1], 2);

// 	if(n_packets>2){
// 		printf("state data backed up\n");
// 	}
// 	return;
// }


int get_latest_state_data(vio_data_t *state_data_out, int id)
{
	int64_t current_time_ns = my_time_monotonic_ns();

	if (id == 1) {
		if (current_time_ns - latest_state.timestamp_ns < 1e9) {
			memcpy(state_data_out, &latest_state, sizeof (vio_data_t));
			return 0;
		}
		else {
			return -1;
		}
	}
	else if (id == 2) {
		if (current_time_ns - latest_state2.timestamp_ns < 1e9) {
			memcpy(state_data_out, &latest_state2, sizeof (vio_data_t));
			return 0;
		}
		else {
			return -1;
		}
	}

	return -1;
}


static void* hitl_vio_thread(void *arg)
{
	(void) arg;

	printf("HITL VIO thread starting\n");

	int pipe_fd = open("/run/mpa/hitl_vio", O_RDWR);
	if(pipe_fd < 0){
		fprintf(stderr, "hitl pipe missing, please start px4 hitl first, then voxl-vision-hub\n");
		return NULL;
	}

	// cool, hitl is running, stop vio manager so it doesn't conflict
	vio_manager_stop();

	// make sure the aligned output pipe exists so we can publish this data
	vio_manager_create_aligned_pipe();

	// flush out the old data, this is particularly important if the pip filled
	// up since we would then be left with a partial packet
	while(main_running){
		char buf[1024*1024];
		ssize_t bytes_read = read(pipe_fd, buf, 1024*1024);
		printf("flushed %ld bytes from the hitl pipe\n", bytes_read);
		if(bytes_read<(1024*1024)) break;
	}

	// vectors for rotating gazebo's velocity vector
	rc_vector_t v_body_wrt_body = RC_VECTOR_INITIALIZER;
	rc_vector_alloc(&v_body_wrt_body, 3);
	rc_vector_t v_body_wrt_local = RC_VECTOR_INITIALIZER;
	rc_vector_alloc(&v_body_wrt_local, 3);
	rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
	rc_matrix_alloc(&R_body_to_local, 3, 3);

	// keep reading data, TODO have this exit on sigpipe or sigint
	while(main_running){
		vio_data_t d;
		ssize_t bytes_read = read(pipe_fd, &d, sizeof (vio_data_t));
		if (0) printf("Read %ld bytes from HITL VIO pipe\n", bytes_read);

		// rotate velocity frame from body back to local
		float_to_vector(d.vel_imu_wrt_vio, &v_body_wrt_body);
		float_to_matrix(d.R_imu_to_vio, &R_body_to_local);
		rc_matrix_times_col_vec(R_body_to_local, v_body_wrt_body, &v_body_wrt_local);
		vector_to_float(v_body_wrt_local, d.vel_imu_wrt_vio);

		// all done, set and publish the new state
		latest_state2 = d;
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d, sizeof(vio_data_t));
	}

	printf("HITL VIO thread ending\n");

	return NULL;
}


int state_manager_init(void)
{
	// set up pipe callbacks
	// pipe_client_set_connect_cb(STATE_PIPE_CH, _connect_cb, NULL);
	// pipe_client_set_disconnect_cb(STATE_PIPE_CH, _disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(STATE_PIPE_CH, state_pipe_helper_cb, NULL);
	// pipe_client_set_helper_thread_priority(STATE_PIPE_CH, VIO_THREAD_PRIORITY);

	char state_pipe[32] = "state";

	pipe_client_open(STATE_PIPE_CH, state_pipe, PIPE_CLIENT_NAME,
								CLIENT_FLAG_EN_SIMPLE_HELPER,\
								VIO_RECOMMENDED_READ_BUF_SIZE);

	// // read data from a second state pipe
	// pipe_client_set_simple_helper_cb(STATE2_PIPE_CH, state2_pipe_helper_cb, NULL);

	// pipe_client_open(STATE2_PIPE_CH, vfc_vio_pipe, PIPE_CLIENT_NAME,
	// 							CLIENT_FLAG_EN_SIMPLE_HELPER,
	// 							VIO_RECOMMENDED_READ_BUF_SIZE);

	// If we are using HITL then the VIO data will come through this pipe
	if (en_hitl) {
		pthread_attr_t attr;
		pthread_attr_init(&attr);
		pthread_t thread_id;
		pthread_create(&thread_id, &attr, &hitl_vio_thread, NULL);
	}

	return 0;

}

// stop state callback immediately, then wait for manager thread to stop
void state_manager_stop(void)
{
	// close client pipes
	pipe_client_close(STATE_PIPE_CH);
	//pipe_client_close(STATE2_PIPE_CH);

	return;
}

