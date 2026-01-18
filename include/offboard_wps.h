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


#ifndef OFFBOARD_SQUARE_H
#define OFFBOARD_SQUARE_H


int offboard_wps_init(void);
int offboard_wps_stop(int blocking);

/**
 * @brief      enable or disable the printing of debug messages
 *
 * @param[in]  debug  0 to disable, nonzero to enable
 */
void offboard_wps_en_print_debug(int debug);

/* set timeout */
void offboard_wps_set_pause_time(double timeout);

/* set stride */
void offboard_wps_set_stride(double stride);

/* decel */
void offboard_wps_damp(double damp);

/* input */
void offboard_wps_vfc_mission(int vfc_mission);

/* input */
void offboard_wps_vfc_mission_loop(int vfc_mission_loop);

/* input */
void offboard_wps_vfc_mission_to_ramp(double vfc_mission_to_ramp);

/* input */
void offboard_wps_vfc_mission_cruise_speed(double vfc_mission_speed);

/* input */
void offboard_wps_vfc_mission_to_kp(double vfc_mission_to_kp);


/* speeds */
void set_mpc_vel_param_v(float v);
void set_mpc_xy_cruise_param_v(float v) ;
void set_mpc_vel_man_param_v(float v) ;
void set_mpc_nav_acc_rad(float v) ;
void set_mpc_tko_speed(float v) ;

static double sign(double x) {
    return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0);
}


#endif // end #define OFFBOARD_SQUARE_H
