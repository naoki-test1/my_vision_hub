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
#include <modal_pipe_sink.h>

#include "fixed_pose_input.h"
#include "voxl_vision_hub.h"
#include "pipe_channels.h"
#include "geometry.h"


static void data_cb(__attribute__((unused)) int ch, char* data, int bytes, \
											__attribute__((unused)) void* context)
{
	// catch possible read errors
	if(bytes<=0) return;

	// check for 4dof pose packets
	if(bytes%sizeof(pose_4dof_t)==0){
		int n_packets;
		pose_4dof_t* pose_array = pipe_validate_pose_4dof_t(data, bytes, &n_packets);
		if(pose_array==NULL){
			fprintf(stderr, "ERROR, failed to validate pose_4dof_t packets on fixed pose input sink\n");
			return;
		}
		// loop through the array
		for(int i=0; i<n_packets; i++){
			geometry_add_fixed_frame_estimate(pose_array[i]);
		}
	}
	// check for 6dof pose packets
	else if(bytes%sizeof(pose_vel_6dof_t)==0){
		int n_packets;
		pose_vel_6dof_t* pose_array = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);
		if(pose_array==NULL){
			fprintf(stderr, "ERROR, failed to validate pose_6dof_t packets on fixed pose input sink\n");
			return;
		}
		// loop through the array
		for(int i=0; i<n_packets; i++){
			geometry_add_fixed_frame_estimate_6dof(pose_array[i]);
		}
	}
	else{
		fprintf(stderr, "WARNING in fixed_frame_pipe: uneven number of bytes read %d\n", bytes);
		return;
	}
	return;
}

int fixed_pose_input_init(void)
{
	int flags = SINK_FLAG_EN_SIMPLE_HELPER;
	int ret = pipe_sink_create(FIXED_POSE_INPUT_CH, FIXED_POSE_INPUT_LOCATION, \
								flags, POSE_4DOF_RECOMMENDED_PIPE_SIZE, \
								POSE_4DOF_RECOMMENDED_READ_BUF_SIZE);
	if(ret<0){
		return -1;
	}
	pipe_sink_set_simple_cb(FIXED_POSE_INPUT_CH, data_cb, NULL);
	return 0;
}

int fixed_pose_input_stop(void)
{
	pipe_sink_close(FIXED_POSE_INPUT_CH);
	return 0;
}
