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


#ifndef PX4_MAVLINK_H
#define PX4_MAVLINK_H

#include <stdint.h>
#include <c_library_v2/common/mavlink.h>

// all messages sent from VOXL are tagged with this component ID to
// differentiate their origin from packets from QGC or PX4
#define VOXL_COMPID			MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
#define AUTOPILOT_COMPID	MAV_COMP_ID_AUTOPILOT1


int mavlink_io_init(void);
int mavlink_io_stop(void);

void mavlink_io_en_print_debug_send(int en_print_debug);
void mavlink_io_en_print_debug_recv(int en_print_debug);

int mavlink_io_send_msg_to_ap(mavlink_message_t* msg);


/**
 * @brief      send a position setpoint in local frame
 *
 *             PX4 is only aware of local frame. it flies in local frame, and
 *             take setpoints in local frame. This frame is centered whever VIO started.
 *
 * @param[in]  local_sp  local setpoin
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_io_send_local_setpoint(uint8_t sysid, uint8_t compid, mavlink_set_position_target_local_ned_t local_sp);


/**
 * @brief      send a position setpoint in fixed frame
 *
 *             PX4 is only aware of local frame. it flies in local frame, and
 *             take setpoints in local frame. This function will convert a
 *             position setpoint which is in fixed frame to local frame before
 *             sending to PX4.
 *
 * @param[in]  fixed_sp  setpoint in fixed frame
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_io_send_fixed_setpoint(uint8_t sysid, uint8_t compid, mavlink_set_position_target_local_ned_t fixed_sp);

/**
 * @brief      send an attitude target to PX4
 *
 *             Specify desired attitude, angular rate, and thrust.
 *
 * @param[in]  attitude_target attitude target
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_io_send_attitude_target(uint8_t sysid, uint8_t compid, mavlink_set_attitude_target_t attitude_target);


int mavlink_io_send_set_mode(uint8_t sysid, uint8_t compid, mavlink_set_mode_t set_mode);

int mavlink_io_send_msg_to_gcs(mavlink_message_t* msg);
// used to send messages (usually warnings) to QGC
int mavlink_io_send_text_to_gcs(const char* string);

/**
 * @brief      Send an OSD string to PX4
 *
 *             Specify desired row, column, and string
 *
 * @param[in]  row, column, OSD string
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_io_send_osd(uint8_t row, uint8_t col, const char *string);

#endif // end #define PX4_MAVLINK_H
