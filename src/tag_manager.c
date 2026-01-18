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

#include <rc_math.h>
#include <modal_pipe_client.h>
#include <modal_pipe_interfaces.h>

#include "macros.h"
#include "tag_manager.h"
#include "pipe_channels.h"
#include "geometry.h"
#include "config_file.h"
#include "offboard_follow_tag.h"

#define PIPE_NAME			"tag_detections"


static int running = 0; // local running flag separate from main_running
static int print_debug = 0;
static int print_debug_wrt_cam = 0;
static int print_debug_wrt_local = 0;


void tag_manager_en_print_debug_wrt_cam(int en_print)
{
	if(en_print){
		print_debug_wrt_cam = 1;
		print_debug = 1;
	}
	return;
}

void tag_manager_en_print_debug_wrt_local(int en_print)
{
	if(en_print){
		print_debug_wrt_local = 1;
		print_debug = 1;
	}
	return;
}


// This is the function called by the libmodal_pipe "simple helper thread"
// all tags are reported here and then we decide what to do with them
static void _data_cb(__attribute__((unused))int ch, char* data, int bytes, \
											__attribute__((unused)) void* context)
{
	static rc_matrix_t R_tag_to_cam = RC_MATRIX_INITIALIZER;
	static rc_vector_t T_tag_wrt_cam = RC_VECTOR_INITIALIZER;
	static rc_matrix_t R_tag_to_fixed = RC_MATRIX_INITIALIZER;
	static rc_vector_t T_tag_wrt_fixed = RC_VECTOR_INITIALIZER;

	// validate that the data makes sense
	int n_detections;
	tag_detection_t* data_array = pipe_validate_tag_detection_t(data, bytes, &n_detections);
	if(data_array == NULL) return;

	// process each tag
	for(int i=0; i<n_detections; i++){

		tag_detection_t d = data_array[i];

		// copy data into rc matrices
		float_to_matrix(data_array[i].R_tag_to_cam,  &R_tag_to_cam);
		float_to_vector(data_array[i].T_tag_wrt_cam, &T_tag_wrt_cam);
		float_to_matrix(data_array[i].R_tag_to_fixed,  &R_tag_to_fixed);
		float_to_vector(data_array[i].T_tag_wrt_fixed, &T_tag_wrt_fixed);

		// pass to offboard mode if enabled
		if(offboard_mode==FOLLOW_TAG && d.id==follow_tag_id){
			offboard_follow_tag_add_detection(d.timestamp_ns, R_tag_to_cam, T_tag_wrt_cam);
		}

		// passed to fixed frame relocalization if enabled and fixed
		else if(en_tag_fixed_frame && d.loc_type==TAG_LOCATION_FIXED){
			geometry_add_fixed_tag_detection(d.timestamp_ns, R_tag_to_cam, T_tag_wrt_cam,
												R_tag_to_fixed, T_tag_wrt_fixed);
		}


		// debug prints if requested
		if(print_debug_wrt_cam){
			double roll, pitch, yaw;
			rc_rotation_to_tait_bryan(R_tag_to_cam, &roll, &pitch, &yaw);
			printf("ID %d T_tag_wrt_cam %5.2f %5.2f %5.2f  Roll %5.2f Pitch %5.2f Yaw %5.2f\n",\
						d.id,T_tag_wrt_cam.d[0],T_tag_wrt_cam.d[1],T_tag_wrt_cam.d[2],roll ,pitch ,yaw);
		}
		if(print_debug_wrt_local){
			static rc_matrix_t R_tag_to_local;
			static rc_vector_t T_tag_wrt_local;
			double roll, pitch, yaw;
			// ask geometry module for the position and rotation of the tag in local
			// frame. This goes back in time and interpolates given timestamp
			int ret = geometry_calc_R_T_tag_in_local_frame(d.timestamp_ns, R_tag_to_cam, \
							T_tag_wrt_cam, &R_tag_to_local, &T_tag_wrt_local);
			if(ret==0){
				rc_rotation_to_tait_bryan(R_tag_to_local, &roll, &pitch, &yaw);

				printf("ID%3d T_tag_wrt_local %5.2f %5.2f %5.2f  Roll %5.2f Pitch %5.2f Yaw %5.2f\n",\
						d.id, T_tag_wrt_local.d[0], T_tag_wrt_local.d[1],T_tag_wrt_local.d[2],roll ,pitch ,yaw);
			}
		}
	} // end of looping through each tag
	return;
}


static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf("Connected to voxl-tag-detector\n");
	return;
}

static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf("Disconnected from voxl-tag-detector\n");
	return;
}


// start the pipe monitor thread which will open the tag pipe and helper
int tag_manager_init(void)
{
	if(running){
		fprintf(stderr, "ERROR in %s, already running\n", __FUNCTION__);
		return 0;
	}
	// start the manager thread, this will open the pipe to read tag data
	running = 1;
	pipe_client_set_connect_cb(      TAG_PIPE_CH, _connect_cb,      NULL);
	pipe_client_set_disconnect_cb(   TAG_PIPE_CH, _disconnect_cb,   NULL);
	pipe_client_set_simple_helper_cb(TAG_PIPE_CH, _data_cb, NULL);

	pipe_client_open(TAG_PIPE_CH, PIPE_NAME, PIPE_CLIENT_NAME, \
						CLIENT_FLAG_EN_SIMPLE_HELPER, \
						TAG_DETECTION_RECOMMENDED_READ_BUF_SIZE);
	return 0;
}


void tag_manager_stop(void)
{
	if(running==0){
		return;
	}
	// stop the thread, this will close the pipe
	running = 0;
	pipe_client_close(TAG_PIPE_CH);
	return;
}
