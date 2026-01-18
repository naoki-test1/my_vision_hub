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


#ifndef PIPE_CHANNELS_H
#define PIPE_CHANNELS_H

/**
 * Consolidate some of the information about our pipe IO here
 */

// client name used when connecting to servers
#define PIPE_CLIENT_NAME	"voxl-vision-hub"
#define PIPE_SERVER_NAME	"voxl-vision-hub"

// sink channels names for these pipes are in voxl-vision-hub.h
#define FIXED_POSE_INPUT_CH		0
#define CONTROL_INPUT_CH		1

// server (output) channels, names for these pipes are in voxl-vision-hub.h
#define LOCAL_POSE_OUT_CH		0
#define FIXED_POSE_OUT_CH		1
#define VOA_PC_OUT_CH			2
#define HORIZON_CAL_IO_CH		3
#define OFFBOARD_LOG_CH			4
#define ALIGNED_VIO_CH			5

// client (input) channels
#define VIO_PIPE_CH				0	// usually voxl-qvio-server, configurable
#define SECONDARY_VIO_PIPE_CH	1	// usually voxl-openvins-server, configurable
#define TAG_PIPE_CH				2	// voxl-apriltag-server
#define TRAJECTORY_PIPE_CH		3	// voxl-mapper
#define MAVLINK_ONBOARD_CH		4	// voxl-mavlink-server
#define MAVLINK_ONBOARD_NAME	"mavlink_onboard"
#define MAVLINK_TO_GCS_CH		5	// voxl-mavlink-server
#define MAVLINK_TO_GCS_NAME		"mavlink_to_gcs"
#define IMU_PIPE_CH				6
#define STATE_PIPE_CH			7
#define STATE2_PIPE_CH			8
#define VOA_CLIENT_CH_START		10	// start at ch 10, go up to 10+MAX_VOA_INPUTS-1


#endif // end PIPE_CHANNELS_H
