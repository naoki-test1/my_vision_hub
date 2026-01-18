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


#ifndef AUTOPILOT_MONITOR_H
#define AUTOPILOT_MONITOR_H

#include <c_library_v2/common/mavlink.h>


// These modes are specific to PX4, Ardupilot will need its own set
typedef enum px4_main_mode{
    PX4_MAIN_MODE_UNKNOWN = 0,
    PX4_MAIN_MODE_MANUAL = 1,
    PX4_MAIN_MODE_ALTCTL = 2,
    PX4_MAIN_MODE_POSCTL = 3,
    PX4_MAIN_MODE_AUTO = 4,
    PX4_MAIN_MODE_ACRO = 5,
    PX4_MAIN_MODE_OFFBOARD = 6,
    PX4_MAIN_MODE_STABILIZED = 7,
    PX4_MAIN_MODE_RATTITUDE = 8
} px4_main_mode;

typedef enum px4_sub_mode{
    PX4_SUB_MODE_UNKNOWN = 0,
    PX4_SUB_MODE_AUTO_READY = 1,
    PX4_SUB_MODE_AUTO_TAKEOFF = 2,
    PX4_SUB_MODE_AUTO_LOITER = 3,
    PX4_SUB_MODE_AUTO_MISSION = 4,
    PX4_SUB_MODE_AUTO_RTL = 5,
    PX4_SUB_MODE_AUTO_LAND = 6,
    PX4_SUB_MODE_AUTO_RTGS = 7,
    PX4_SUB_MODE_AUTO_FOLLOW_TARGET = 8,
    PX4_SUB_MODE_AUTO_PRECLAND = 9
} px4_sub_mode;



int autopilot_monitor_init(void);
int autopilot_monitor_stop(void);



// called by px4_mavlink.c when new packets arrive from px4
// this pulls out any data that might be used locally
int autopilot_monitor_scrape_data(mavlink_message_t* msg);


// fetch locally cached px4 state
int                 autopilot_monitor_is_connected(void);
int                 autopilot_monitor_is_flight_mode_configuration_known(void);
MAV_STATE           autopilot_monitor_get_system_state(void); // usually to check for MAV_STATE_ACTIVE or MAV_STATE_STANDBY
uint8_t             autopilot_monitor_get_sysid(void);
int                 autopilot_monitor_is_armed(void);
px4_main_mode       autopilot_monitor_get_main_mode(void);
px4_main_mode       autopilot_monitor_get_flight_mode_configuration(void);
void                autopilot_monitor_print_main_mode(void);
px4_sub_mode        autopilot_monitor_get_sub_mode(void);
double              autopilot_monitor_get_bat_voltage(void);
double              autopilot_monitor_get_bat_percentage(void);
mavlink_attitude_t  autopilot_monitor_get_attitude(void);
mavlink_odometry_t  autopilot_monitor_get_odometry(void);
mavlink_optical_flow_rad_t autopilot_monitor_get_optical_flow_rad(void);
int                 autopilot_monitor_get_rpy(float* roll, float* pitch, float* yaw);
float               autopilot_monitor_get_local_position_z(void);
float               autopilot_monitor_get_local_position_vz(void);
bool                autopilot_monitor_get_rc_state(void);
int                 autopilot_monitor_get_rc_value(int chan);
mavlink_attitude_target_t  autopilot_monitor_get_attitude_target(void);
mavlink_attitude_quaternion_t autopilot_monitor_get_attitude_quaternion(void);

// used by the offboard mode controllers to see if PX4 is currenly obeying
// offboard commands. returns 1 if armed and in offboard mode, otherwise 0
int autopilot_monitor_is_armed_and_in_offboard_mode(void);

// used by px4_mavlink.c to set connection flag as disconnected when uart breaks
void autopilot_monitor_set_connected_flag(int flag);

#endif // end #define AUTOPILOT_MONITOR_H
