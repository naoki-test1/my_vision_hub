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
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <rc_math.h>
#include <modal_pipe_server.h>


#include "config_file.h"
#include "imu_manager.h"
#include "state_manager.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "macros.h"
#include "offboard_vfc.h"
#include "misc.h"
#include "pipe_channels.h"
#include <c_library_v2/common/mavlink.h>

#define MAX_TRAJ_PTS 4096
// Full minute for backtrack data
#define MAX_BACKTRACK_PTS (33 * backtrack_seconds)

static const char* submode_strings[] = {"NONE", "THRUST_ATTITUDE", "ALTITUDE_ATTITUDE", "ALTITUDE_FLOW", "POSITION", "TRAJ", "VFC_BACKTRACK"};

typedef struct pos_vel_t{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
}pos_vel_t;

typedef struct pos_yaw_t{
    float x;
    float y;
    float z;
    float yaw;
}pos_yaw_t;

typedef struct data_status_t{
    bool altitude_ok;
    bool flow_ok;
    bool position_ok;
    bool has_trajectory;
}data_status_t;

typedef struct traj_points_t{
    pos_yaw_t pos_yaw[MAX_TRAJ_PTS];
    int size;
    float dt;
}traj_points_t;

static traj_points_t traj_points;

// Circular buffer to store backtrack points
typedef struct backtrack_storage_t {
    pos_yaw_t *pos_yaw;
    int index;
    bool wraparound;
    int64_t timestamp_last_sample;
} backtrack_storage_t;

static backtrack_storage_t backtrack_storage;

// Buffer to store active backtrack points
typedef struct  backtrack_points_t {
    pos_yaw_t *pos_yaw;
    int index;
    int size;
    float dt;
} backtrack_points_t;

static backtrack_points_t backtrack_points;

static pos_vel_t optical_flow_pos_vel;
static float optical_flow_yaw = 0;

static pos_vel_t position_pos_vel;
static float position_yaw = 0;

static data_status_t data_status;

offboard_vfc_submode_t last_submode;

static int running = 0;
static pthread_t offboard_vfc_thread_id;
static int en_debug = 0;

static float yaw_des = 0;
static float x_des = 0;
static float y_des = 0;
static float z_des = 0;

static float x_des_position = 0;
static float y_des_position = 0;
static float z_des_position = 0;

// acceleration rate-limited desired lateral velocity components
static float vx_des_body_limited = 0;
static float vy_des_body_limited = 0;

// filtered desired world frame acceleration
static float ax_des_world_filt = 0;
static float ay_des_world_filt = 0;

static bool reset_a_des_filt = true;

// 1.0 means filter disabled, this will be changed during init based on conf file
static float alpha_lpf_position = 1.0f;
static float alpha_lpf_flow = 1.0f;

static pos_yaw_t pos_yaw0;
static float t0_traj = 0;
static float t0_backtrack = 0;

static rc_vector_t q_at_transition = RC_VECTOR_INITIALIZER;
static float t_transition = -1000;
static mavlink_set_attitude_target_t attitude_target_sent;

static bool roll_pitch_stick_is_still = false;

static bool backtrack_desired = false;

static bool forced_transition_to_offboard = false;

static bool is_turtle = false;

static float gravity = 9.81;

static float lerp(float a, float b, float f)
{
    return a + f * (b - a);
}

static float constrainf(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    else if (value > max) {
        return max;
    }
    else {
        return value;
    }
}

static const char* getfield(char *line, int num)
{
	const char *tok;
	for (tok = strtok(line, ","); tok && *tok; tok = strtok(NULL, ",\n"))
	{
		if (!--num)
			return tok;
	}
	return NULL ;
}

static void get_interpolated_setpoint(pos_yaw_t current, pos_yaw_t next, float fraction, pos_yaw_t *pos_yaw) {
    pos_yaw->x   = lerp(current.x,   next.x,   fraction);
    pos_yaw->y   = lerp(current.y,   next.y,   fraction);
    pos_yaw->z   = lerp(current.z,   next.z,   fraction);

    //handle potential yaw wraps in stored yaw setpoints
    float yaw_diff = next.yaw - current.yaw;
    WRAP_TO_NEGPI_TO_PI_F(yaw_diff);

    float interpolated_yaw = lerp(current.yaw, current.yaw + yaw_diff, fraction);
    WRAP_TO_NEGPI_TO_PI_F(interpolated_yaw);

    pos_yaw->yaw = interpolated_yaw;
}

static int get_traj_setpoint(float rel_time, pos_yaw_t *pos_yaw) {
    float stepf = rel_time/traj_points.dt;

    int step = (int)stepf;
    float step_part = stepf - step;

    //printf("steps: %f, %d, %f\n", (double)stepf, step, (double)step_part);

    if (traj_points.size <= 0) {
        pos_yaw->x   = 0;
        pos_yaw->y   = 0;
        pos_yaw->z   = 0;
        pos_yaw->yaw = 0;
    }
    else if ((step + 1) >= traj_points.size) {
        *pos_yaw = traj_points.pos_yaw[traj_points.size-1];
    }
    else {
        get_interpolated_setpoint(traj_points.pos_yaw[step],
                                  traj_points.pos_yaw[step+1],
                                  step_part,
                                  pos_yaw);
    }

    return 0;
}

static int get_backtrack_setpoint(float rel_time, pos_yaw_t *pos_yaw) {
    float stepf = rel_time / backtrack_points.dt;

    int step = (int)stepf;
    float step_part = stepf - step;

    //printf("steps: %f, %d, %f\n", (double)stepf, step, (double)step_part);

    if (backtrack_points.size < 1) {
        // Not enough points to run a backtrack
        return -1;
    }
    else if (backtrack_points.size < 2) {
        //only a single backtrack point so return that point
        backtrack_points.index = 1;  //so that restore_unused_backtrack_points does not restore any points
        *pos_yaw = backtrack_points.pos_yaw[0];
    }
    else if ((step + 1) >= backtrack_points.size) {
        // Past the end of our backtrack data so return the final backtrack point
        backtrack_points.index = backtrack_points.size;  //so that restore_unused_backtrack_points does not restore any points
        *pos_yaw = backtrack_points.pos_yaw[backtrack_points.size-1];
    }
    else {
        backtrack_points.index = step;

        get_interpolated_setpoint(backtrack_points.pos_yaw[step],
                                backtrack_points.pos_yaw[step+1],
                                step_part,
                                pos_yaw);

        // printf("Backtrack setpoint: %f, %f, %f, %f\n", (double) pos_yaw->x, (double) pos_yaw->y, (double) pos_yaw->z, (double) pos_yaw->yaw);
    }

    return 0;
}

static void init_backtrack_points() {
    // Hardcoded to match open vins update rate for now
    // backtrack_points.dt = 0.33;
    backtrack_points.dt = 0.05;

    backtrack_points.size = 0;
    backtrack_points.index = 0;

    printf("Initializing backtrack array. Index = %d, wraparound = %d\n",
            backtrack_storage.index, backtrack_storage.wraparound);

    if ((backtrack_storage.index > 1) || (backtrack_storage.wraparound)) {
        int start_index = backtrack_storage.index;
        int storage_index = backtrack_storage.index - 1;
        int points_index = 0;
        // If we just had a wraparound reposition storage index
        if (storage_index < 1) {
            printf("Backtrack storage had just wrapped around\n");
            storage_index = MAX_BACKTRACK_PTS - 1;
            backtrack_storage.wraparound = false;
        }
        // Copy all the way back to the beginning of the storage array
        while (storage_index >= 0) {
            backtrack_points.pos_yaw[points_index] = backtrack_storage.pos_yaw[storage_index];
            storage_index--;
            points_index++;
            backtrack_points.size++;
        }
        // If there was a wraparound continue copying all the rest of the points
        if (backtrack_storage.wraparound) {
            printf("Backtrack storage had a wrap around\n");
            storage_index = MAX_BACKTRACK_PTS - 1;
            while (storage_index >= start_index) {
                backtrack_points.pos_yaw[points_index] = backtrack_storage.pos_yaw[storage_index];
                storage_index--;
                points_index++;
                backtrack_points.size++;
            }
        }

        // Capture the backtrack start time
        t0_backtrack = my_time_monotonic_ns()/1.e9f;

        printf("Copied %d backtrack points\n", backtrack_points.size);
    }

    // Reset storage array since we have finished copying all we need out of it
    backtrack_storage.index = 0;
    backtrack_storage.wraparound = false;
}

static void store_new_backtrack_point(pos_yaw_t point, int64_t timestamp, bool restore) {
    if (((last_submode != VFC_BACKTRACK) || (restore)) && (backtrack_storage.timestamp_last_sample != timestamp)) {
        backtrack_storage.timestamp_last_sample = timestamp;
        backtrack_storage.pos_yaw[backtrack_storage.index].x = point.x;
        backtrack_storage.pos_yaw[backtrack_storage.index].y = point.y;
        backtrack_storage.pos_yaw[backtrack_storage.index].z = point.z;
        backtrack_storage.pos_yaw[backtrack_storage.index].yaw = point.yaw;
        backtrack_storage.index++;
        // Handle buffer wraparound
        if (backtrack_storage.index == MAX_BACKTRACK_PTS) {
            backtrack_storage.index = 0;
            backtrack_storage.wraparound = true;
        }

        // if (backtrack_storage.index % 10 == 0) {
        //     printf("Backtrack storing at %d: %f, %f, %f, %f\n", backtrack_storage.index, (double) point.x, (double) point.y, (double) point.z, (double) point.yaw);
        // }
    }
}

static void restore_unused_backtrack_points() {
    int64_t fake_timestamp = 0;
    printf("Restoring %d backtrack points\n", backtrack_points.size - backtrack_points.index);
    for (int i = backtrack_points.size - 1; i >= backtrack_points.index; i--) {
        store_new_backtrack_point(backtrack_points.pos_yaw[i], fake_timestamp++, true);
    }
}

static void reset_backtrack() {
    // printf("Backtrack reset\n");

    // Storage
    backtrack_storage.index = 0;
    backtrack_storage.wraparound = false;
    backtrack_storage.timestamp_last_sample = 0;

    // Points
    backtrack_points.size = 0;
    backtrack_points.index = 0;

    // RC handling
    backtrack_desired = false;
}

static mavlink_set_attitude_target_t _fill_attitude_target(float thrust, rc_vector_t q, float yaw_rate) {
    mavlink_set_attitude_target_t attitude_target;

    attitude_target.time_boot_ms = 0;
    attitude_target.target_system = 0;
    attitude_target.target_component = 0;
    attitude_target.type_mask = ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE | 
                                ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;
    attitude_target.q[0] = (float)q.d[0];
    attitude_target.q[1] = (float)q.d[1];
    attitude_target.q[2] = (float)q.d[2];
    attitude_target.q[3] = (float)q.d[3];
    attitude_target.body_roll_rate = 0;
    attitude_target.body_pitch_rate = 0;
    attitude_target.body_yaw_rate = yaw_rate;
    attitude_target.thrust_body[0] = 0;
    attitude_target.thrust_body[1] = 0;
    attitude_target.thrust_body[2] = 0;
    attitude_target.thrust = thrust;

    return attitude_target;
}

static rc_vector_t rc_quaternion_from_rpy(double roll, double pitch, double yaw) {
    rc_vector_t q = RC_VECTOR_INITIALIZER;
    rc_vector_t tb = RC_VECTOR_INITIALIZER;

    double rpy[] = {roll, pitch, yaw};
    rc_vector_from_array(&tb, rpy, 3);

    rc_quaternion_from_tb(tb, &q);

    return q;
}

static rc_vector_t rc_quaternion_from_components(float q1, float q2, float q3, float q4) {
    rc_vector_t q = RC_VECTOR_INITIALIZER;

    double qd[] = {q1, q2, q3, q4};
    rc_vector_from_array(&q, qd, 4);
    rc_quaternion_normalize(&q);

    return q;
}


static bool check_backtrack_switch(uint16_t ch_val) {

    static bool was_armed = false;

    bool is_armed = autopilot_monitor_is_armed();
    bool button_pressed = (ch_val > backtrack_rc_thresh);

    // If we were just armed and the button is pressed then
    // it is being pressed for turtle mode, not backtrack.
    if (!was_armed && is_armed && button_pressed) {
        is_turtle = true;
    }

    // If we are in turtle mode and we are no longer armed or
    // the button is no longer pressed then we have exited turtle_mode
    if (is_turtle && (!is_armed || !button_pressed)) {
        is_turtle = false;
    }

    // If we are armed, not in turtle mode, and the button is pressed
    // then we are in backtrack
    if (is_armed && !is_turtle && button_pressed) {
        backtrack_desired = true;
    }

    // If we are armed, not in turtle mode, and the button is not pressed
    // then we are not in backtrack
    if (is_armed && !is_turtle && !button_pressed) {
        backtrack_desired = false;
    }

    was_armed = is_armed;

    return backtrack_desired;
}

static void check_roll_pitch_stick_motion() {
    static float time_last_motion = -100.f;

    float time_now = my_time_monotonic_ns()/1.e9f;

    int pitch_delta = autopilot_monitor_get_rc_value(vfc_params.pitch_ch) - 1500;
    int roll_delta = autopilot_monitor_get_rc_value(vfc_params.roll_ch) - 1500;

    roll_pitch_stick_is_still = false;

    if (abs(roll_delta)>vfc_params.stick_move_threshold || abs(pitch_delta)>vfc_params.stick_move_threshold) {
        time_last_motion = time_now;
    }
    else if (time_now - time_last_motion > vfc_params.flow_transition_time) {
        roll_pitch_stick_is_still = true;
    }
}

static offboard_vfc_submode_t get_desired_submode(uint16_t ch_val) {

    // Check backtrack switch. This can be on a different RC channel
    if (check_backtrack_switch(autopilot_monitor_get_rc_value(backtrack_rc_chan))) {
        return VFC_BACKTRACK;
    }

    if (ch_val > vfc_params.traj_mode_rc_min && ch_val < vfc_params.traj_mode_rc_max) {
        return TRAJ;
    }

    if (ch_val > vfc_params.position_mode_rc_min && ch_val < vfc_params.position_mode_rc_max) {
        return POSITION;
    }

    if (ch_val > vfc_params.hybrid_flow_mode_rc_min && ch_val < vfc_params.hybrid_flow_mode_rc_max) {
        if (roll_pitch_stick_is_still) {
            return ALTITUDE_FLOW;
        }
        else {
            return ALTITUDE_ATTITUDE;
        }
    }

    if (ch_val > vfc_params.flow_mode_rc_min && ch_val < vfc_params.flow_mode_rc_max) {
        return ALTITUDE_FLOW;
    }

    if (ch_val > vfc_params.alt_mode_rc_min && ch_val < vfc_params.alt_mode_rc_max) {
        return ALTITUDE_ATTITUDE;
    }

    // If nothing was chosen then the default is manual mode (THRUST_ATTITUDE)
    return THRUST_ATTITUDE;
}

static offboard_vfc_submode_t get_fallback_submode(offboard_vfc_submode_t mode) {
    if (mode == VFC_BACKTRACK) {
        if (data_status.position_ok) {
            return VFC_BACKTRACK;
        }
        else {
            // Fallback options for backtrack mode
            return THRUST_ATTITUDE;
        }
    }

    if (mode == TRAJ) {
        if (data_status.position_ok && data_status.has_trajectory) {
            return TRAJ;
        }
        else {
            // Fallback options for trajectory mode
            return THRUST_ATTITUDE;
        }
    }

    if (mode == POSITION) {
        if (data_status.position_ok) {
            return POSITION;
        }
        else {
            // Fallback options for position mode
            return THRUST_ATTITUDE;
        }
    }

    if (mode == ALTITUDE_FLOW) {
        if (data_status.flow_ok && data_status.altitude_ok) {
            return ALTITUDE_FLOW;
        }
        else {
            // Fallback options for flow mode
            if (data_status.altitude_ok) {
                return ALTITUDE_ATTITUDE;
            }
            else {
                return THRUST_ATTITUDE;
            }
        }
    }

    if (mode == ALTITUDE_ATTITUDE) {
        if (data_status.altitude_ok) {
            return ALTITUDE_ATTITUDE;
        }
        else {
            // Fallback options for altitude mode
            return THRUST_ATTITUDE;
        }
    }

    return THRUST_ATTITUDE;
}

static float get_thrust_direct(uint16_t ch_val) {
    // The thrust stick value should be between 1000 and 2000
    float bounded_ch_val = constrainf((float)ch_val, 1000.0f, 2000.0f);

    // Normalize to 0.0 - 1.0 scaling value
    float thrust_scale = (bounded_ch_val - 1000.0f) / 1000.0f;

    // There are two thrust ranges. One between the configured minimum and
    // hover thrusts, and one between the hover and maximum thrusts.
    // We will treat the upper thrust range the same as the lower thrust range
    // to keep throttle stick feeling proportional between the lower range (e.g.
    // when the stick is below midpoint) and the higher range.
    float thrust_range = (vfc_params.thrust_hover - vfc_params.min_thrust) * 2.0f;

    // Calculate desired thrust
    float thrust_des = vfc_params.min_thrust + (thrust_range * thrust_scale);

    // Make sure resulting value is between our configured min and max values
    thrust_des = constrainf(thrust_des, vfc_params.min_thrust, vfc_params.max_thrust);

    return thrust_des;
}

static float get_thrust_altitude(float z_desired, float z_est, float vz_desired, float vz_est, float kp_z, float kd_z) {
    //TODO, compensate for tilt of vehicle

    //positive z is down
    float acc_des = kp_z * (z_desired - z_est) + kd_z * (vz_desired - vz_est);

    //positive thrust is up
    float thrust_des = vfc_params.thrust_hover * (1.0f - acc_des/gravity);

    thrust_des = constrainf(thrust_des, vfc_params.min_thrust, vfc_params.max_thrust);

    return thrust_des;
}

static float get_z_delta(float thrust_command, float kp_z)  {
    //this function calculates the required z command delta in order to achieve a desired thrust_command
    //it ignores the contribution from the kd_z term 

    if (vfc_params.thrust_hover <= 0 || kp_z < 0) {
        printf("WARNING: thrust hover and kp_z must be > 0\n");
        return 0;
    }

    float thrust_ratio = thrust_command/vfc_params.thrust_hover;
    float acc_des = gravity*(1.0f - thrust_ratio); //positive is down

    float z_delta = acc_des/kp_z;

    z_delta = constrainf(z_delta, -vfc_params.max_z_delta, vfc_params.max_z_delta);

    return z_delta;
}

static void limit_xy_mag(float *x, float *y, float max_val)
{
    float mag = sqrtf((*x)*(*x) + (*y)*(*y));

    if (mag > max_val)
    {
        float scale = max_val/mag;
        *x *= scale;
        *y *= scale;
    }

    return;
}

static float compute_alpha_lpf(float wn_hz, float dt)
{
    if (wn_hz>0)
    {
        float wn_dt = TWO_PI_F * wn_hz * dt;
        return wn_dt / (1 + wn_dt);
    }

    return 1.0f;
}

static void get_roll_pitch_des(  float x_des, float x_est,\
                                 float vx_des, float vx_est,\
                                 float y_des, float y_est, \
                                 float vy_des, float vy_est,\
                                 float yaw, float kp_xy, float kd_xy, \
                                 float alpha_lpf, \
                                 float *roll_des, float *pitch_des)
{
    float ax_des_world_raw = kp_xy * (x_des - x_est) + kd_xy * (vx_des - vx_est);
    float ay_des_world_raw = kp_xy * (y_des - y_est) + kd_xy * (vy_des - vy_est);

    if (reset_a_des_filt)
    {
        ax_des_world_filt = ax_des_world_raw;
        ay_des_world_filt = ay_des_world_raw;
        reset_a_des_filt = false;
    }
    else
    {
        // low pass filter the desired world acceleration
        ax_des_world_filt = alpha_lpf * ax_des_world_raw  + (1 - alpha_lpf) * ax_des_world_filt;
        ay_des_world_filt = alpha_lpf * ay_des_world_raw  + (1 - alpha_lpf) * ay_des_world_filt;
    }

    // yaw is the yaw angle of body w.r.t. to the x y data frame
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);

    float ax_des = ax_des_world_filt*cos_yaw + ay_des_world_filt*sin_yaw;
    float ay_des = -ax_des_world_filt*sin_yaw + ay_des_world_filt*cos_yaw;

    float a_des = sqrtf(ax_des*ax_des + ay_des*ay_des);

    if (a_des < 0.0001f) {
        *roll_des = 0;
        *pitch_des = 0;
        return;
    }  

    float a_max = gravity*tanf(vfc_params.tilt_max);

    float tilt_angle = 0;

    if (a_des < a_max) {
        tilt_angle = atanf(a_des/gravity);
    }
    else {
        tilt_angle = vfc_params.tilt_max;
    }

    *roll_des = ay_des/a_des * tilt_angle;
    *pitch_des = -ax_des/a_des * tilt_angle;

    if (en_debug) {
        printf("ax, ay, roll_des, pitch_des: %f, %f, %f, %f\n", (double)ax_des, (double)ay_des, (double)*roll_des, (double)*pitch_des);
    }
}


static float apply_rc_deadband(uint16_t ch_val, int deadband) {
    float output = 0.0;

    float diff = ch_val - 1500.0f;
    
    if (diff > deadband) {
        output = (diff - deadband) / (500.0f - deadband);
    }
    else if (diff < -deadband) {
        output = (diff + deadband) / (500.0f - deadband);
    }

    output = constrainf(output, -1.0f, 1.0f);

    return output;
}

static float get_vz_des(uint16_t ch_val) {
    //positive z id down
    float ch_val_unitless = apply_rc_deadband(ch_val, vfc_params.vz_deadband);

    return -ch_val_unitless * vfc_params.vz_max;
}

static void* _offboard_vfc_thread_func(__attribute__((unused)) void* arg)
{
    const float dt = 1.0f/vfc_params.rate;

    bool position_was_ok = false;
    bool thermal_flow_was_ok = false;
    bool print_fallback_disabled_warning = true;

	// for loop sleep
	int64_t next_time = 0;

    // Wait until flight controller is connected and has provided the
    // RC flight mode configuration
    while (!autopilot_monitor_is_flight_mode_configuration_known()) {
        usleep(10000);
    }

    // Initialize OSD for VIO and Thermal Flow
    (void) mavlink_io_send_osd(1, 1, "VIO BAD");

    (void) mavlink_io_send_osd(2, 1, "THRM BAD");

    while(running) {
        bool publish_command = true;
        bool chan_vals_ok = true;
        offboard_vfc_submode_t desired_submode = NONE;

        mavlink_set_attitude_target_t attitude_target;
        float thrust_des = 0;

        memset(&offboard_log, 0, sizeof(offboard_log));

        //GET DATA FROM AUTOPILOT
        mavlink_attitude_t attitude = autopilot_monitor_get_attitude();
        mavlink_attitude_target_t attitude_target_from_ap = autopilot_monitor_get_attitude_target();
        mavlink_attitude_quaternion_t attitude_quaternion = autopilot_monitor_get_attitude_quaternion();

        // Save previous status
        position_was_ok = data_status.position_ok;
        thermal_flow_was_ok = (data_status.flow_ok && data_status.altitude_ok);

        // get optical flow state estimate, z, and vz from mpa pipe
        data_status.flow_ok = false;
        data_status.altitude_ok = false;
        data_status.position_ok = false;

        vio_data_t state_data;
        if (get_latest_state_data(&state_data, 1) == 0) {
            // printf("pose_covariance x, y, z: %f, %f, %f\n", (double)state_data.pose_covariance[0], (double)state_data.pose_covariance[6], (double)state_data.pose_covariance[11]);

            if (state_data.pose_covariance[0] > 0 && state_data.pose_covariance[6] > 0) {
                optical_flow_pos_vel.x = state_data.T_imu_wrt_vio[0];
                optical_flow_pos_vel.y = state_data.T_imu_wrt_vio[1];

                optical_flow_pos_vel.vx = state_data.vel_imu_wrt_vio[0];
                optical_flow_pos_vel.vy = state_data.vel_imu_wrt_vio[1];

                //get the vehicle yaw angle relative to optical flow frame

                optical_flow_yaw = atan2f(state_data.R_imu_to_vio[1][0], state_data.R_imu_to_vio[0][0]);

                // printf("optical_flow yaw: %f\n", (double)optical_flow_yaw);

                data_status.flow_ok = true;
            }

            if (state_data.pose_covariance[11] > 0) {
                optical_flow_pos_vel.z = state_data.T_imu_wrt_vio[2];
                optical_flow_pos_vel.vz = state_data.vel_imu_wrt_vio[2];

                data_status.altitude_ok = true;
            }
        }

        vio_data_t position_data;
        if (get_latest_state_data(&position_data, 2) == 0) {

            offboard_log.vio_state = position_data.state;
            offboard_log.vio_quality = position_data.quality;
            offboard_log.vio_n_feature_points = position_data.n_feature_points;

            // printf("state, qual, pts: %d, %d, %d ", position_data.state, position_data.quality, position_data.n_feature_points);

            if (position_data.state == VIO_STATE_OK && position_data.quality >= vfc_params.q_min && position_data.n_feature_points >= vfc_params.points_min)
            {
                // vio_manager.c has already oriented the data to body frame
                position_pos_vel.x  = position_data.T_imu_wrt_vio[0];
                position_pos_vel.y  = position_data.T_imu_wrt_vio[1];
                position_pos_vel.z  = position_data.T_imu_wrt_vio[2];
                position_pos_vel.vx = position_data.vel_imu_wrt_vio[0];
                position_pos_vel.vy = position_data.vel_imu_wrt_vio[1];
                position_pos_vel.vz = position_data.vel_imu_wrt_vio[2];

                position_yaw = atan2f(position_data.R_imu_to_vio[1][0], position_data.R_imu_to_vio[0][0]);

                // printf("pos: (%f, %f, %f)", (double)position_data.T_imu_wrt_vio[0], (double)position_data.T_imu_wrt_vio[1], (double)position_data.T_imu_wrt_vio[2]);
                // printf("vel: (%f, %f, %f) ", (double)position_data.vel_imu_wrt_vio[0], (double)position_data.vel_imu_wrt_vio[1], (double)position_data.vel_imu_wrt_vio[2]);
                // printf("yaw: %f \n", (double)position_yaw);

                data_status.position_ok = true;

                // printf("position (x,y,z): %f, %f, %f\n", (double)position_pos_vel.x, (double)position_pos_vel.y, (double)position_pos_vel.z);

                // Update the backtrack storage if there is new position data and we are not currently
                // in a backtrack
                // pos_yaw_t new_backtrack_point = {position_pos_vel.x, position_pos_vel.y, position_pos_vel.z, position_yaw};
                pos_yaw_t new_backtrack_point = {position_pos_vel.x, position_pos_vel.y, position_pos_vel.z, attitude.yaw};
                store_new_backtrack_point(new_backtrack_point, position_data.timestamp_ns, false);
            }
            else {
                // If position data isn't valid then reset all backtrack info
                reset_backtrack();
            }
        }

        if (!position_was_ok && data_status.position_ok) {
            // Position wasn't valid and now it is. Update OSD.
            (void) mavlink_io_send_osd(1, 1, "VIO GOOD");
        } else if (position_was_ok && !data_status.position_ok) {
            // Position was valid and now it isn't. Update OSD.
            (void) mavlink_io_send_osd(1, 1, "VIO BAD");
        }

        if (!thermal_flow_was_ok && (data_status.flow_ok && data_status.altitude_ok)) {
            // Thermal flow wasn't valid and now it is. Update OSD.
            (void) mavlink_io_send_osd(2, 1, "THRM GOOD");
        } else if (thermal_flow_was_ok && !(data_status.flow_ok && data_status.altitude_ok)) {
            // Thermal flow was valid and now it isn't. Update OSD.
            (void) mavlink_io_send_osd(2, 1, "THRM BAD");
        }

        // imu_data_t imu_data;
        // if (get_latest_imu_data(&imu_data) == 0) {
        //     // printf("latest imu: %ld %f\n", imu_data.timestamp_ns, (double)imu_data.gyro_rad[0]);
        // }

        //TODO, do not send command if we fail to get data here

        for(int i=1;i<=8;i++) {
            if (autopilot_monitor_get_rc_value(i) < vfc_params.rc_chan_min ||
                autopilot_monitor_get_rc_value(i) > vfc_params.rc_chan_max ) {
                chan_vals_ok = false;
            }
        }

        if (!chan_vals_ok) {
            publish_command = false;
            if(en_debug) printf("Error: RC channel values out of range\n");
        }

        float roll_des = 0;
        float pitch_des = 0;
        float yaw_rate_des = 0;
        float vz_des = 0;

        if (!publish_command) {
            last_submode = NONE;
        }
        else {
            // Determine submode
            desired_submode = get_desired_submode(autopilot_monitor_get_rc_value(vfc_params.submode_ch));
            offboard_vfc_submode_t submode = get_fallback_submode(desired_submode);

            check_roll_pitch_stick_motion();

            if(!autopilot_monitor_is_armed_and_in_offboard_mode()) {
                //not in offboard mode

                // set the q_des from the current measured quaternion
                rc_vector_t q_des = rc_quaternion_from_components(attitude_quaternion.q1, attitude_quaternion.q2, attitude_quaternion.q3, attitude_quaternion.q4);

                //set the thrust desired as the current commanded thrust
                thrust_des = attitude_target_from_ap.thrust;

                attitude_target = _fill_attitude_target(thrust_des, q_des, 0);

                // If we were in a backtrack and the user changed flight modes on us
                // then restore the remaining backtrack points so that we can pick it
                // up again where we left off.
                if (last_submode == VFC_BACKTRACK) {
                    restore_unused_backtrack_points();
                }

                last_submode = NONE;

                print_fallback_disabled_warning = true;

                if (autopilot_monitor_is_armed() && desired_submode == VFC_BACKTRACK && data_status.position_ok) {
                    // If backtrack was activated and we are armed but not in offboard mode
                    // and we are getting valid position data then tell autopilot to go into offboard mode
                    mavlink_set_mode_t set_mode;
                    set_mode.custom_mode = PX4_MAIN_MODE_OFFBOARD << 16;
                    set_mode.target_system = autopilot_monitor_get_sysid();
                    set_mode.base_mode = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
                    mavlink_io_send_set_mode(autopilot_monitor_get_sysid(), VOXL_COMPID, set_mode);
                    forced_transition_to_offboard = true;
                }
            }
            else {
                //in offboard mode

                // If we cannot enter our desired submode and we have disabled fallback
                // modes then instruct the flight controller to exit offboard mode and
                // go to altitude hold mode
                if (submode != desired_submode && vfc_params.disable_fallback) {
                    mavlink_set_mode_t set_mode;
                    set_mode.custom_mode = PX4_MAIN_MODE_ALTCTL << 16;
                    set_mode.target_system = autopilot_monitor_get_sysid();
                    set_mode.base_mode = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
                    mavlink_io_send_set_mode(autopilot_monitor_get_sysid(), VOXL_COMPID, set_mode);
                    if (print_fallback_disabled_warning) {
                        printf("VFC Warning: Desired mode not available, transitioning to altctl\n");
                        print_fallback_disabled_warning = false;
                    }
                }

                //check if anything needs to be initialized
                if (submode != last_submode) {

                    q_at_transition = rc_quaternion_from_components(attitude_target_sent.q[0], attitude_target_sent.q[1], attitude_target_sent.q[2], attitude_target_sent.q[3]);
                    t_transition = my_time_monotonic_ns()/1.e9f;

                    reset_a_des_filt = true;

                    if (last_submode == NONE) {
                        yaw_des = attitude.yaw;
                        printf("setting yaw des to: %f\n", (double)yaw_des);
                    }

                    if (last_submode == VFC_BACKTRACK) {
                        // Don't throw away unused points, restore them and start
                        // adding new points afterwards.
                        restore_unused_backtrack_points();

                        // If we are coming out of backtrack and we had originally forced
                        // going into offboard mode out of a different flight mode then now
                        // it is necessary to force it back to the desired user selected flight mode.
                        if (forced_transition_to_offboard) {
                            mavlink_set_mode_t set_mode;
                            px4_main_mode desired_mode = autopilot_monitor_get_flight_mode_configuration();
                            // If we can't tell what the user wants then default to altitude hold mode
                            if (desired_mode == PX4_MAIN_MODE_UNKNOWN) {
                                desired_mode = PX4_MAIN_MODE_ALTCTL;
                            }
                            set_mode.custom_mode = desired_mode << 16;
                            set_mode.target_system = autopilot_monitor_get_sysid();
                            set_mode.base_mode = 1; // VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
                            mavlink_io_send_set_mode(autopilot_monitor_get_sysid(), VOXL_COMPID, set_mode);
                            printf("VFC transitioning from backtrack to flight mode %d\n", desired_mode);
                            forced_transition_to_offboard = false;
                        }
                    }

                    if (submode == ALTITUDE_ATTITUDE || submode == ALTITUDE_FLOW) {
                        if (!(last_submode == ALTITUDE_ATTITUDE || last_submode == ALTITUDE_FLOW)) {
                            float z_delta = get_z_delta(attitude_target_from_ap.thrust, vfc_params.kp_z);
                            z_des = optical_flow_pos_vel.z + z_delta;
                            printf("setting z des to: %f = %f + %f\n", (double)z_des, (double)optical_flow_pos_vel.z, (double)z_delta);
                        }
                    }

                    if (submode == ALTITUDE_FLOW) {
                        x_des = optical_flow_pos_vel.x;
                        y_des = optical_flow_pos_vel.y;
                        printf("setting x,y des to: %f, %f\n", (double)x_des, (double)y_des);
                    }

                    if (submode == POSITION || submode == TRAJ) {
                        vx_des_body_limited = 0;
                        vy_des_body_limited = 0;
                        x_des_position = position_pos_vel.x;
                        y_des_position = position_pos_vel.y;
                        z_des_position = position_pos_vel.z; //TODO: account for current commanded thrust here
                        printf("setting x,y,z des positon to: %f, %f, %f\n", (double)x_des_position, (double)y_des_position, (double)z_des_position);
                    }

                    if (submode == TRAJ) {
                        pos_yaw0.x = position_pos_vel.x;
                        pos_yaw0.y = position_pos_vel.y;
                        pos_yaw0.z = position_pos_vel.z;  //TODO: account for current commanded thrust here
                        pos_yaw0.yaw = attitude.yaw;
                        t0_traj = my_time_monotonic_ns()/1.e9f;
                    }

                    if (submode == VFC_BACKTRACK) {
                        init_backtrack_points();
                        // the desired position gets overwritten later if there is valid backtrack data
                        x_des_position = position_pos_vel.x;
                        y_des_position = position_pos_vel.y;
                        z_des_position = position_pos_vel.z; //TODO: account for current commanded thrust here
                    }
                    printf("submode: %s\n", submode_strings[submode]);

                    if (vfc_params.en_submode_announcement) {
                        mavlink_message_t msg;
                        char msg_string[48];

                        switch (submode) {
                        case ALTITUDE_ATTITUDE:
                            strncpy(msg_string, "VFC ALTITUDE", 48);
                            break;
                        case ALTITUDE_FLOW:
                            strncpy(msg_string, "VFC ALTITUDE FLOW", 48);
                            break;
                        case THRUST_ATTITUDE:
                            strncpy(msg_string, "VFC MANUAL", 48);
                            break;
                        case POSITION:
                            strncpy(msg_string, "VFC POSITION", 48);
                            break;
                        case TRAJ:
                            strncpy(msg_string, "VFC TRAJECTORY", 48);
                            break;
                        case VFC_BACKTRACK:
                            strncpy(msg_string, "VFC BACKTRACK", 48);
                            break;
                        default:
                            strncpy(msg_string, "VFC DISABLED", 48);
                            break;
                        }

                        // This will produce message on GCS and announce with audio
                        mavlink_msg_statustext_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
                                                    MAV_SEVERITY_CRITICAL, msg_string, 0, 0);
                        (void) mavlink_io_send_msg_to_gcs(&msg);
                    }
                }
                last_submode = submode;

                if (submode == VFC_BACKTRACK) {
                    float rel_time_now = my_time_monotonic_ns()/1.e9f - t0_backtrack;

                    pos_yaw_t pos_yaw_backtrack;
                    int backtrack_status = get_backtrack_setpoint(rel_time_now, &pos_yaw_backtrack);

                    if (backtrack_status == 0) {
                        x_des_position = pos_yaw_backtrack.x;
                        y_des_position = pos_yaw_backtrack.y;
                        z_des_position = pos_yaw_backtrack.z;
                        yaw_des = pos_yaw_backtrack.yaw;
                        // printf("Desired yaw: %f, current yaw: %f\n", yaw_des, position_yaw);
                    }
                }
                else if (submode == TRAJ) {
                    float rel_time_now = my_time_monotonic_ns()/1.e9f - t0_traj;

                    // printf("rel time: %f\n", (double)rel_time_now);

                    pos_yaw_t pos_yaw_traj;
                    get_traj_setpoint(rel_time_now, &pos_yaw_traj);

                    printf("setpoint: (%f, %f, %f, %f)\n", (double)pos_yaw_traj.x, (double)pos_yaw_traj.y, (double)pos_yaw_traj.z, (double)pos_yaw_traj.yaw);

                    x_des_position = pos_yaw0.x + pos_yaw_traj.x;
                    y_des_position = pos_yaw0.y + pos_yaw_traj.y;
                    z_des_position = pos_yaw0.z + pos_yaw_traj.z;
                    yaw_des = pos_yaw0.yaw + pos_yaw_traj.yaw;
                }
                else {
                    yaw_rate_des = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.yaw_ch), vfc_params.yaw_deadband)*vfc_params.yaw_rate_max;

                    yaw_des += yaw_rate_des * dt;
                }

                WRAP_TO_NEGPI_TO_PI_F(yaw_des);

                rc_vector_t q_yaw = rc_quaternion_from_rpy(0, 0, yaw_des);

                float vx_des_body, vy_des_body, cos_yaw, sin_yaw, vx_des, vy_des;
                if (submode == POSITION || submode == TRAJ || submode == VFC_BACKTRACK) {

                    float vx_ff = 0;
                    float vy_ff = 0;
                    if (submode == POSITION) {
                        vx_des_body = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.pitch_ch), vfc_params.vxy_deadband)*vfc_params.vxy_max;
                        vy_des_body = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.roll_ch), vfc_params.vxy_deadband)*vfc_params.vxy_max;

                        limit_xy_mag(&vx_des_body, &vy_des_body, vfc_params.vxy_max);

                        if (vfc_params.xy_acc_limit_vio > 0) {
                            //vx_des_body and vy_des_body are the targets
                            float delta_vx = vx_des_body - vx_des_body_limited;
                            float delta_vy = vy_des_body - vy_des_body_limited;

                            limit_xy_mag(&delta_vx, &delta_vy, dt*vfc_params.xy_acc_limit_vio);

                            vx_des_body_limited += delta_vx;
                            vy_des_body_limited += delta_vy;
                        }
                        else {
                            vx_des_body_limited = vx_des_body;
                            vy_des_body_limited = vy_des_body;
                        }

                        cos_yaw = cosf(position_yaw);
                        sin_yaw = sinf(position_yaw);

                        vx_des = vx_des_body_limited*cos_yaw - vy_des_body_limited*sin_yaw;
                        vy_des = vx_des_body_limited*sin_yaw + vy_des_body_limited*cos_yaw;

                        x_des_position += vx_des*dt;
                        y_des_position += vy_des*dt;

                        // feedforward on velocity
                        vx_ff = vx_des*vfc_params.vel_ff_factor_vio;
                        vy_ff = vy_des*vfc_params.vel_ff_factor_vio;

                        if(en_debug){
                            printf("pos_ctrl_sp: xy: %7.2f %7.2f  vxy: %7.2f %7.2f\n", x_des_position, y_des_position, vx_des, vy_des);
                        }
                    }


                    get_roll_pitch_des(x_des_position, position_pos_vel.x, vx_ff, position_pos_vel.vx,
                                        y_des_position, position_pos_vel.y, vy_ff, position_pos_vel.vy,
                                        position_yaw, vfc_params.kp_xy_vio, vfc_params.kd_xy_vio, alpha_lpf_position, &roll_des, &pitch_des);
                }
                else if (submode == ALTITUDE_FLOW) {
                    vx_des_body = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.pitch_ch), vfc_params.vxy_deadband)*vfc_params.vxy_max;
                    vy_des_body = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.roll_ch), vfc_params.vxy_deadband)*vfc_params.vxy_max;

                    limit_xy_mag(&vx_des_body, &vy_des_body, vfc_params.vxy_max);

                    cos_yaw = cosf(optical_flow_yaw);
                    sin_yaw = sinf(optical_flow_yaw);

                    vx_des = vx_des_body*cos_yaw - vy_des_body*sin_yaw;
                    vy_des = vx_des_body*sin_yaw + vy_des_body*cos_yaw;

                    x_des += vx_des*dt;
                    y_des += vy_des*dt;

                    get_roll_pitch_des(x_des, optical_flow_pos_vel.x, 0, optical_flow_pos_vel.vx, 
                                        y_des, optical_flow_pos_vel.y, 0, optical_flow_pos_vel.vy,
                                        optical_flow_yaw, vfc_params.kp_xy, vfc_params.kd_xy, alpha_lpf_flow, &roll_des, &pitch_des);
                }
                else {
                    roll_des = apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.roll_ch), 0) * vfc_params.tilt_max;
                    pitch_des = -apply_rc_deadband(autopilot_monitor_get_rc_value(vfc_params.pitch_ch), 0) * vfc_params.tilt_max;
                }

                float tilt_angle_des = sqrtf(roll_des*roll_des + pitch_des*pitch_des);

                rc_vector_t tilt_axis = RC_VECTOR_INITIALIZER;
                rc_vector_t q_tilt = RC_VECTOR_INITIALIZER;
                rc_vector_zeros(&tilt_axis, 3);

                if (tilt_angle_des > 0.0001f) {
                    tilt_axis.d[0] = roll_des/tilt_angle_des;
                    tilt_axis.d[1] = pitch_des/tilt_angle_des;
                }
                else {
                    tilt_axis.d[0] = 1.0;
                }

                if (tilt_angle_des > vfc_params.tilt_max) {
                    tilt_angle_des = vfc_params.tilt_max;
                }
                rc_axis_angle_to_quaternion(tilt_axis, tilt_angle_des, &q_tilt);

                rc_vector_t q_des = RC_VECTOR_INITIALIZER;

                // Q_BODYDES_WORLD = Q_BODYDES_YAWFRAME * Q_YAWFRAME_WORLD
                rc_quaternion_multiply(q_tilt, q_yaw, &q_des);

                if (submode == POSITION || submode == TRAJ || submode == VFC_BACKTRACK) {
                    vz_des = 0;
                    if (submode == POSITION) {
                        vz_des = get_vz_des(autopilot_monitor_get_rc_value(vfc_params.thrust_ch));

                        z_des_position += vz_des * (float)dt;
                    }
                    thrust_des = get_thrust_altitude(z_des_position, position_pos_vel.z, vz_des, position_pos_vel.vz, vfc_params.kp_z_vio, vfc_params.kd_z_vio);
                }
                else if (submode == ALTITUDE_ATTITUDE || submode == ALTITUDE_FLOW) {
                    vz_des = get_vz_des(autopilot_monitor_get_rc_value(vfc_params.thrust_ch));

                    z_des += vz_des * (float)dt;
                    thrust_des = get_thrust_altitude(z_des, optical_flow_pos_vel.z, vz_des, optical_flow_pos_vel.vz, vfc_params.kp_z, vfc_params.kd_z);
                }
                else {
                    thrust_des = get_thrust_direct(autopilot_monitor_get_rc_value(vfc_params.thrust_ch));
                }

                //check if a transition recently occurred and we need to smooth the quaternion
                float time_since_transition = my_time_monotonic_ns()/1.e9f - t_transition;
                if (time_since_transition >= 0 && time_since_transition < vfc_params.att_transition_time) {
                    float time_ratio = constrainf(time_since_transition/vfc_params.att_transition_time, 0.0f, 1.0f);

                    rc_vector_t q_des_final = RC_VECTOR_INITIALIZER;
                    rc_quaternion_slerp(q_at_transition, q_des, time_ratio, &q_des_final);

                    // printf("quat smoothing: %f\n", (double)time_ratio);
                    // rc_vector_print(q_at_transition);
                    // rc_vector_print(q_des);
                    // rc_vector_print(q_des_final);

                    q_des = q_des_final;
                }

                attitude_target = _fill_attitude_target(thrust_des, q_des, yaw_rate_des);
            }

            if(en_debug) {
                printf("mode (rc_trpy) (sp_trpy): %d (%d, %d, %d, %d) (%f, %f, %f, %f)\n", \
                            submode, \
                            autopilot_monitor_get_rc_value(3), \
                            autopilot_monitor_get_rc_value(1), \
                            autopilot_monitor_get_rc_value(2), \
                            autopilot_monitor_get_rc_value(4), \
                            (double)thrust_des, \
                            (double)roll_des, \
                            (double)pitch_des, \
                            (double)yaw_des);
            }

            mavlink_io_send_attitude_target(autopilot_monitor_get_sysid(), VOXL_COMPID, attitude_target);
            attitude_target_sent = attitude_target;
//            if(en_debug) printf("sent attitude target\n");
        }

        //fill and publish the log struct
        offboard_log.packet_version = 7;
        offboard_log.timestamp_ns = my_time_monotonic_ns();

        offboard_log.desired_submode = desired_submode;
        offboard_log.submode = last_submode;

        offboard_log.thrust_des = thrust_des;
        offboard_log.roll_des = roll_des;
        offboard_log.pitch_des = pitch_des;
        offboard_log.yaw_des = yaw_des;
        offboard_log.yaw_rate_des = yaw_rate_des;

        offboard_log.q0_des = attitude_target.q[0];
        offboard_log.q1_des = attitude_target.q[1];
        offboard_log.q2_des = attitude_target.q[2];
        offboard_log.q3_des = attitude_target.q[3];

        offboard_log.of_x = optical_flow_pos_vel.x;
        offboard_log.of_y = optical_flow_pos_vel.y;
        offboard_log.of_z = optical_flow_pos_vel.z;

        offboard_log.of_x_des = x_des;
        offboard_log.of_y_des = y_des;
        offboard_log.of_z_des = z_des;

        offboard_log.of_vx = optical_flow_pos_vel.vx;
        offboard_log.of_vy = optical_flow_pos_vel.vy;
        offboard_log.of_vz = optical_flow_pos_vel.vz;

        offboard_log.of_vx_des = 0;
        offboard_log.of_vy_des = 0;
        offboard_log.of_vz_des = vz_des;

        offboard_log.vio_x = position_pos_vel.x;
        offboard_log.vio_y = position_pos_vel.y;
        offboard_log.vio_z = position_pos_vel.z;

        offboard_log.vio_x_des = x_des_position;
        offboard_log.vio_y_des = y_des_position;
        offboard_log.vio_z_des = z_des_position;

        offboard_log.vio_vx = position_pos_vel.vx;
        offboard_log.vio_vy = position_pos_vel.vy;
        offboard_log.vio_vz = position_pos_vel.vz;

        offboard_log.vio_vx_des = 0;
        offboard_log.vio_vy_des = 0;
        offboard_log.vio_vz_des = vz_des;

        for (int idx = 0; idx<8; idx++) {
            offboard_log.raw_rc_chans[idx] = autopilot_monitor_get_rc_value(idx+1);
        }

        offboard_log.altitude_ok = data_status.altitude_ok;
        offboard_log.flow_ok = data_status.flow_ok;
        offboard_log.position_ok = data_status.position_ok;

        // Backtrack specific data logging
        offboard_log.t0_backtrack = t0_backtrack;
        offboard_log.backtrack_desired = backtrack_desired;
        offboard_log.turtle_mode = is_turtle;

        offboard_log.forced_transition_to_offboard = forced_transition_to_offboard;
        offboard_log.backtrack_storage_index = backtrack_storage.index;
        offboard_log.backtrack_wraparound = backtrack_storage.wraparound;
        offboard_log.backtrack_data_size = backtrack_points.size;
        offboard_log.armed = autopilot_monitor_is_armed();

        offboard_log.loop_time = (my_time_monotonic_ns() - next_time)/1.e9f;

        pipe_server_write(OFFBOARD_LOG_CH, &offboard_log, sizeof(offboard_log_packet));

 		if(my_loop_sleep(vfc_params.rate, &next_time)){
			fprintf(stderr, "WARNING vfc thread fell behind\n");
		}
		fflush(stdout);
    }

	printf("exiting offboard vfc thread\n");
	return NULL;
}


int offboard_vfc_init(void)
{
    alpha_lpf_position = compute_alpha_lpf(vfc_params.w_filt_xy_vio, 1.0f/vfc_params.rate);
    printf("alpha lpf position = %f\n", (double)alpha_lpf_position);

    alpha_lpf_flow = compute_alpha_lpf(vfc_params.w_filt_xy_flow, 1.0f/vfc_params.rate);
    printf("alpha lpf flow = %f\n", (double)alpha_lpf_position);

    data_status.altitude_ok = false;
    data_status.flow_ok = false;
    data_status.position_ok = false;
    data_status.has_trajectory = false;

    backtrack_storage.pos_yaw = (pos_yaw_t*) calloc(MAX_BACKTRACK_PTS, sizeof(pos_yaw_t));
    backtrack_points.pos_yaw = (pos_yaw_t*) calloc(MAX_BACKTRACK_PTS, sizeof(pos_yaw_t));

    printf("try to load VFC trajectory csv from: %s\n", vfc_traj_csv);
    traj_points.size = 0;

    FILE *stream = fopen(vfc_traj_csv, "r");

    if (stream == NULL)
    {
        printf("WARN: No trajectory csv\n");
    }
    else {
        printf("trying to read VFC trajectory csv\n");

        traj_points.dt = 0.02;

        char line[256];
        while (fgets(line, 256, stream)) {

            if (traj_points.size < MAX_TRAJ_PTS) {

                char *tmp = strdup(line);
                traj_points.pos_yaw[traj_points.size].x = atof(getfield(tmp, 1));
                free(tmp);

                tmp = strdup(line);
                traj_points.pos_yaw[traj_points.size].y = atof(getfield(tmp, 2));
                free(tmp);

                tmp = strdup(line);
                traj_points.pos_yaw[traj_points.size].z = atof(getfield(tmp, 3));
                free(tmp);

                tmp = strdup(line);
                traj_points.pos_yaw[traj_points.size].yaw = atof(getfield(tmp, 4));
                free(tmp);
            }
            else {
                printf("WARN: too many trajectory points in csv.  Max = %d\n", (int)MAX_TRAJ_PTS);
                break;
            }

            printf("cnt (x,y,z,yaw): %d, (%f, %f, %f, %f)\n", traj_points.size, (double)traj_points.pos_yaw[traj_points.size].x, (double)traj_points.pos_yaw[traj_points.size].y, (double)traj_points.pos_yaw[traj_points.size].z, (double)traj_points.pos_yaw[traj_points.size].yaw);

            traj_points.size++;
        }

        if (traj_points.size > 0 && traj_points.size < MAX_TRAJ_PTS) {
            data_status.has_trajectory = true;
        }
    }

    last_submode = NONE;

    //create to offboard log pipe
	pipe_info_t info1 = { \
		.name        = "offboard_log",\
		.location    = "offboard_log_location",\
		.type        = "offboard_log_packet",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = 1280*800*64};

	int flags1 = 0;
	if(pipe_server_create(OFFBOARD_LOG_CH, info1, flags1)) return -1;    

    running = 1;
    pipe_pthread_create(&offboard_vfc_thread_id, _offboard_vfc_thread_func, NULL, OFFBOARD_THREAD_PRIORITY);

    return 0;
}


int offboard_vfc_stop(int blocking)
{
	if(running==0) return 0;
	running = 0;
	if(blocking){
		pthread_join(offboard_vfc_thread_id, NULL);
	}

    // close server pipe
	pipe_server_close(OFFBOARD_LOG_CH);

    free(backtrack_storage.pos_yaw);
    free(backtrack_points.pos_yaw);

	return 0;
}

void offboard_vfc_en_print_debug(int debug)
{
	if(debug) en_debug = 1;
}
