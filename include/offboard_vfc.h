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


#ifndef OFFBOARD_VFC_H
#define OFFBOARD_VFC_H

#include <stdint.h>
#include <stdbool.h>

typedef enum offboard_vfc_submode_t{
    NONE,
    THRUST_ATTITUDE,
    ALTITUDE_ATTITUDE,
    ALTITUDE_FLOW,
    POSITION,
    TRAJ,
    VFC_BACKTRACK, // Can't conflict with other offboard backtrack mode
    VFC_MAX_SUBMODES
} offboard_vfc_submode_t;

typedef struct offboard_log_packet{
    uint8_t packet_version;
    int64_t timestamp_ns;

    int8_t submode;
    int8_t desired_submode;

    float thrust_des;
    float roll_des;
    float pitch_des;
    float yaw_des;
    float yaw_rate_des;

    float q0_des;
    float q1_des;
    float q2_des;
    float q3_des;

    float of_x;
    float of_y;
    float of_z;

    float of_x_des;
    float of_y_des;
    float of_z_des;

    float of_vx;
    float of_vy;
    float of_vz;

    float of_vx_des;
    float of_vy_des;
    float of_vz_des;

    float vio_x;
    float vio_y;
    float vio_z;

    float vio_x_des;
    float vio_y_des;
    float vio_z_des;

    float vio_vx;
    float vio_vy;
    float vio_vz;

    float vio_vx_des;
    float vio_vy_des;
    float vio_vz_des;

    uint16_t raw_rc_chans[8];

    bool altitude_ok;
    bool flow_ok;
    bool position_ok;

    bool armed;

    // Backtrack diagnostic log data
    float t0_backtrack;
    bool backtrack_desired;
    bool turtle_mode;
    bool forced_transition_to_offboard;
    int backtrack_storage_index;
    bool backtrack_wraparound;
    int backtrack_data_size;

    // vio log data
    int32_t vio_quality;
    uint16_t vio_n_feature_points;
    uint8_t vio_state;

    float loop_time; //vfc loop time in seconds

} __attribute__((packed)) offboard_log_packet;


int offboard_vfc_init(void);
int offboard_vfc_stop(int blocking);

/**
 * @brief      enable or disable the printing of debug messages
 *
 * @param[in]  debug  0 to disable, nonzero to enable
 */
void offboard_vfc_en_print_debug(int debug);

offboard_log_packet offboard_log;

#endif // end #define OFFBOARD_VFC_H