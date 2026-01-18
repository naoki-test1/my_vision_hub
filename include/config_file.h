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


#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <math.h>
#include <rc_math.h>
#include <modal_pipe_common.h>

////////////////////////////////////////////////////////////////////////////////
// stuff from common config
////////////////////////////////////////////////////////////////////////////////

// from extrinsics.conf
extern double height_body_above_ground_m;
extern rc_vector_t T_stereo_wrt_body;
extern rc_matrix_t R_stereo_to_body;


/*
 * load the common extrinsics config files
 * this prints out data as it goes
 */
int load_extrinsics_file(void);


////////////////////////////////////////////////////////////////////////////////
// stuff from our own config file
////////////////////////////////////////////////////////////////////////////////

#define VOXL_VISION_OLD_CONF_FILE "/etc/modalai/voxl-vision-px4.conf"
#define VOXL_VISION_HUB_CONF_FILE "/etc/modalai/voxl-vision-hub.conf"

#define OFFBOARD_STRINGS {"off","figure_eight","follow_tag","trajectory","vfc","backtrack","wps"}
typedef enum offboard_mode_t{
	OFF,
	FIGURE_EIGHT,
	FOLLOW_TAG,
	TRAJECTORY,
	VFC,
	BACKTRACK,
	WPS
}offboard_mode_t;


#define MAX_VOA_INPUTS 6
#define VOA_FRAME_STRING_LEN 64 // matches voxl_common_config extrinsic frame len
#define VOA_INTPUT_TYPE_STRINGS {"point_cloud","tof","rangefinder"}
#define N_VOA_INPUT_TYPES 3
typedef enum voa_input_type_t{
	VOA_POINT_CLOUD,
	VOA_TOF,
	VOA_RANGEFINDER
}voa_input_type_t;

typedef struct voa_input_t{
	int enabled;
	voa_input_type_t type;
	char input_pipe[MODAL_PIPE_MAX_PATH_LEN];
	char frame[64];
	float max_depth;     // furthest distance the sensor can reliably see
	float min_depth;     // use this to cut off false positives from sensor seeing the props
	float cell_size;     // size of 3d voxel grid cells, increase for more downsampling
	int threshold;  // set to 1 to disable thresholding
	float x_fov_deg;     // FOV of the sensor in the x direction, typically width
	float y_fov_deg;     // FOV of the sensor in the y direction, typically height
	int conf_cutoff; // discard points below this confidence, only applicable to TOF
}voa_input_t;

typedef struct vfc_params_t{
    float rate;

    int rc_chan_min;
    int rc_chan_max;

    int thrust_ch;
    int roll_ch;
    int pitch_ch;
    int yaw_ch;
    int submode_ch; //channel for offboard submode

    int alt_mode_rc_min; //min rc value for altitude submode
    int alt_mode_rc_max; //max rc value for altitude submode

    int flow_mode_rc_min; //min rc value for flow submode
    int flow_mode_rc_max; //max rc value for flow submode

    int hybrid_flow_mode_rc_min; //min rc value for hybrid flow submode
    int hybrid_flow_mode_rc_max; //max rc value for hybrid flow submode

    int position_mode_rc_min; //min rc value for position submode
    int position_mode_rc_max; //max rc value for position submode

    int traj_mode_rc_min; //min rc value for traj submode
    int traj_mode_rc_max; //max rc value for traj submode

    int yaw_deadband;
    int vxy_deadband;
    int vz_deadband;

    float min_thrust;
    float max_thrust;

    float tilt_max;
    float yaw_rate_max;

    float thrust_hover; //nominal thrust required to hover is 0 to 1 thrust units

    float vz_max;
    float kp_z; //desired acceleration in m/s/s per m error
    float kd_z; //desired acceleration in m/s/s per m/s error

    float vxy_max;
    float kp_xy; //desired acceleration in m/s/s per m error
    float kd_xy; //desired acceleration in m/s/s per m/s error

    float kp_z_vio; //desired acceleration in m/s/s per m error for vio
    float kd_z_vio; //desired acceleration in m/s/s per m/s error for vio

    float kp_xy_vio; //desired acceleration in m/s/s per m error for vio
    float kd_xy_vio; //desired acceleration in m/s/s per m/s error for vio

    float w_filt_xy_vio;  //first order low pass filter in Hz applied to xy control for vio
    float w_filt_xy_flow; //first order low pass filter in Hz applied to xy control for  optic flow

    float vel_ff_factor_vio; //velocity feedforward factor for vio
    float xy_acc_limit_vio; //xy acceleration limit for vio

    float max_z_delta; //maximum z delta from current z estimated to achieve commanded thrust on transition
    float att_transition_time; //time period over which the attitude commanded is smoothed on submode transitions

    float flow_transition_time; //in hybrid flow - time period for which roll and pitch sticks must be still in order to transition to flow submode
    float stick_move_threshold; //threshold around 1500 in RC units where stick is considered still

    int q_min; //minimum quality for vio data
    int points_min; //minimum points for vio data

    int en_submode_announcement; // Enable sending submode change indications to QGC

    int disable_fallback; // Don't allow a fallback mode if desired mode is not available
}vfc_params_t;

// these are all the possible parameters from the json config file
// UDP mavlink router
extern int en_localhost_mavlink_udp;
extern int localhost_udp_port_number;
// vio
extern int en_vio;
extern char vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
extern char secondary_vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
extern char vfc_vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
extern char vfc_traj_csv[MODAL_PIPE_MAX_PATH_LEN];
extern int en_reset_vio_if_initialized_inverted;
extern float vio_warmup_s;
extern int send_odom_while_failed;
// misc features
extern float  horizon_cal_tolerance;
extern int en_hitl;
// APQ8096 only features
extern int    en_reset_px4_on_error;
extern int    en_set_clock_from_gps;
extern int    en_force_onboard_mav1_mode;
// offboard mode
extern offboard_mode_t offboard_mode;
extern int follow_tag_id;
extern int figure_eight_move_home;
extern int wps_move_home;
extern float wps_timeout;
extern float wps_stride;
extern float wps_damp;
extern int wps_vfc_mission;
extern int wps_vfc_mission_loop;
extern float wps_vfc_mission_to_ramp;
extern float wps_vfc_mission_to_kp;
extern float wps_vfc_mission_cruise_speed;
//extern int wps_vfc_mission_nav_acc;  /// TODO
extern float robot_radius;
extern double collision_sampling_dt;
extern float max_lookahead_distance;
extern int backtrack_seconds;
extern int backtrack_rc_chan;
extern int backtrack_rc_thresh;
// fixed frame
extern int en_tag_fixed_frame;
extern int fixed_frame_filter_len;
extern int en_transform_mavlink_pos_setpoints_from_fixed_frame;
//voxl flight controller params
extern vfc_params_t vfc_params;
// collision prevention (VOA)
extern int en_voa;
extern float voa_upper_bound_m;
extern float voa_lower_bound_m;
extern float voa_memory_s;
extern int   voa_max_pc_per_fusion;
extern float voa_pie_min_dist_m;
extern float voa_pie_max_dist_m;
extern float voa_pie_under_trim_m;
extern int   voa_pie_threshold;
extern float voa_send_rate_hz;
extern int   voa_pie_slices;
extern float voa_pie_bin_depth_m;
// VOA input source configuration
extern int n_voa_inputs;
extern voa_input_t voa_inputs[MAX_VOA_INPUTS];



// load only our own config file without printing the contents
int config_file_load(void);

// prints the current configuration values to the screen.
int config_file_print(void);

int extrinsics_fetch_frame_to_body(char* frame, rc_matrix_t* R, rc_vector_t* T);


#endif // end #define CONFIG_FILE_H
