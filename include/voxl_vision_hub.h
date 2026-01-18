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

/**
 * @file voxl_vision_hub.h
 *
 * This file contains definitions for interfacing with voxl-vision-px4
 * programmatically through the Modal Pipe Architecture (MPA).
 */

#ifndef VOXL_VISION_HUB_H
#define VOXL_VISION_HUB_H

#include <stdint.h>
#include <modal_pipe_common.h>


// input pipe-sink names


/**
 * The fixed frame pipe allows an external position estimator to inform
 * voxl-vision-px4 of a fixed frame pose at a given point in time. This allows
 * a position estimator such as a visual localization system (similar to our own
 * internal fixed apriltag localizer) to feed information to voxl-vision-px4
 * enabling the same fixed-frame positioning ability.
 *
 * This pipe accepts data as a pose_4dof_t struct written straight to the pipe.
 * It is implemented as a "sink" type in libmodal_pipe.
 *
 * see limbodal_pipe modal_pipe_interfaces.h
 */
#define FIXED_POSE_INPUT_NAME		"vvhub_fixed_pose_input"
#define FIXED_POSE_INPUT_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR FIXED_POSE_INPUT_NAME)


/**
 * The control input pipe uses the sink type from libmodal_pipe to accept
 * general commands. To control VVPX4's behavior while it is running.
 *
 * This is similar to the control pipes on the other outputs, but is only an
 * input with no output.
 *
 * Currently this only accepts one command to run the level horizon calibration
 * routine but it should accept more in the future.
 */
#define CONTROL_INPUT_NAME		"vvhub_control_input"
#define CONTROL_INPUT_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR CONTROL_INPUT_NAME)

// commands that can be sent to the control pipe
#define CONTROL_COMMAND_START_HORIZON_CALIBRATION "start_horizon_calibration"



// output pipe names

/**
 * These pipes present pose data similar to VIO data but in the form of a body
 * pose in either local or fixed frame. They send the similar data to the common
 * visual odometry data type used by voxl-qvio-server and voxl-vins-server:
 * Position, velocity, orientation, and angular rates.
 *
 * Data is published in the form of a pose_vel_6dof_t data struct,
 * see limbodal_pipe modal_pose_server_interface.h
 *
 * These publish data at the same rate as voxl-vision-px4 receives VIO data.
 */
#define BODY_WRT_LOCAL_POSE_NAME		"vvhub_body_wrt_local"
#define BODY_WRT_LOCAL_POSE_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR BODY_WRT_LOCAL_POSE_NAME "/")
#define BODY_WRT_FIXED_POSE_NAME 		"vvhub_body_wrt_fixed"
#define BODY_WRT_FIXED_POSE_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR BODY_WRT_FIXED_POSE_NAME "/")


/**
 * debug point cloud output from VOA
**/
#define VOA_PC_OUT_NAME		"voa_pc_out"
#define VOA_PC_OUT_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR VOA_PC_OUT_NAME "/")

/**
 * IO for doing horizon calibration procedure
**/
#define HORIZON_CAL_IO_NAME		"horizon_cal_io"
#define HORIZON_CAL_IO_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR HORIZON_CAL_IO_NAME "/")


/**
 * similar to the vvhub_body_wrt_local channel but in the more verbose vio_data_t struct format
 * This data has the center of local frame beneath the drone's center of mass at takeoff like
 * the body_wrt_local pipe. It also has the gravity vector fixed at 0,0,1 so that realignment
 * to gravity is not needed. Quality and feature count are passed through from the VIO source.
**/
#define ALIGNED_VIO_NAME		"vvhub_aligned_vio"
#define ALIGNED_VIO_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR ALIGNED_VIO_NAME "/")

#endif // VOXL_VISION_HUB_H
