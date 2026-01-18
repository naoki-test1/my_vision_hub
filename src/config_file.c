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
#include <stdlib.h> // for system()
#include <unistd.h>		// for access()
#include <modal_json.h>
#include <voxl_common_config.h>
#include "config_file.h"
#include "geometry.h"

////////////////////////////////////////////////////////////////////////////////
// stuff from common config
////////////////////////////////////////////////////////////////////////////////

double height_body_above_ground_m = 0.0;


// array of all extrinsics
static vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
static int n_extrinsics;

// defaults for TOF point cloud downsampling
#define TOF_MAX_DIST_M				6.0f
#define TOF_MIN_DIST_M				0.15f
#define TOF_DOWNSAMPLE_CELL_SIZE	0.08	// raise to reduce resolution of output
#define TOF_DOWNSAMPLE_THRESHOLD	3		// lower if small obects are being lost
#define TOF_X_FOV_DEG				106.5f
#define TOF_Y_FOV_DEG				85.1f
#define TOF_CONFIDENCE_CUTOFF		125

// defaults for stereo point cloud filtering
#define STEREO_MAX_DIST_M		8.0f
#define STEREO_MIN_DIST_M		0.3f
#define STEREO_DOWNSAMPLE_CELL_SIZE	0.08	// raise to reduce resolution of output
#define STEREO_DOWNSAMPLE_THRESHOLD	4		// lower if small obects are being lost
#define STEREO_X_FOV_DEG		68.0f
#define STEREO_Y_FOV_DEG		56.0f



int load_extrinsics_file(void)
{
	vcc_extrinsic_t tmp;

	// now load in extrinsics
	if(vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n_extrinsics, VCC_MAX_EXTRINSICS_IN_CONFIG)){
		return -1;
	}

	// pick out height above ground
	if(vcc_find_extrinsic_in_array("body", "ground", t, n_extrinsics, &tmp)){
		fprintf(stderr, "WARNING: %s missing ground to body transform\n", VCC_EXTRINSICS_PATH);
		height_body_above_ground_m = 0.0;
	}
	else{
		height_body_above_ground_m = tmp.T_child_wrt_parent[2];
	}

	return 0;
}


int extrinsics_fetch_frame_to_body(char* frame, rc_matrix_t* R, rc_vector_t* T)
{
	vcc_extrinsic_t tmp;

	// easy case where desired frame is already body, just return identity
	if(strcmp("body",frame)==0){
		if(T->initialized && T->len==3){
			T->d[0]=0.0;
			T->d[1]=0.0;
			T->d[2]=0.0;
		}
		else{
			rc_vector_zeros(T, 3);
		}
		rc_matrix_zeros(R,3,3);
		R->d[0][0]=1.0;
		R->d[1][1]=1.0;
		R->d[2][2]=1.0;
		return 0;
	}

	// pick out frame to body as is the setup in extrinsics file
	// parent is body
	// child is frame
	if(vcc_find_extrinsic_in_array("body", frame, t, n_extrinsics, &tmp)){
		fprintf(stderr, "ERROR: %s missing %s to body transform\n", VCC_EXTRINSICS_PATH, frame);
		return -1;
	}
	rc_vector_alloc(T, 3);
	rc_matrix_alloc(R,3,3);
	for(int j=0; j<3; j++){
		T->d[j] = tmp.T_child_wrt_parent[j];
		for(int k=0; k<3; k++) R->d[j][k] = tmp.R_child_to_parent[j][k];
	}
	return 0;
}



////////////////////////////////////////////////////////////////////////////////
// stuff from our own config file
////////////////////////////////////////////////////////////////////////////////

#define FILE_HEADER "\
/**\n\
 * VOXL Vision PX4 Configuration File\n\
 *\n\
 * version: don't touch this, used to keep track of changing config file formats\n\
 *\n\
 * ##############################################################################\n\
 * ## MAVROS MAVSDK\n\
 * ##############################################################################\n\
 *\n\
 * en_localhost_mavlink_udp:\n\
 *         If you are running MAVROS/MAVSDK onboard VOXL and wish to open access to\n\
 *         PX4 through a localhost UDP port simply ensure the follow feature is\n\
 *         enabled. This is set to true by default. This will allow one local process\n\
 *         to communicate with PX4 via port 14551 by default, NOT 14550 which is\n\
 *         reserved for connections outside the board. These separation prevents\n\
 *         conflicts between the two sockets. Both MAVROS and MAVSDK can be\n\
 *         configured to use this port.\n\
 *\n\
 * localhost_udp_port_number:\n\
 *         Port number for localhost UDP socket, default 14551\n\
 *\n\
 * ##############################################################################\n\
 * ## VIO\n\
 * ##############################################################################\n\
 *\n\
 * en_vio:\n\
 *         Enable processing of VIO data from MPA to be sent to PX4 as mavlink\n\
 *         odometry messages. Enabled by default.\n\
 *\n\
 * vio_pipe:\n\
 *         Primary pipe to subscribe to for VIO data. Must be a standard libmodal-pipe\n\
 *         vio_data_t type. Default is qvio. If no data is available on this pipe\n\
 *         then voxl-vision-hub will subscribe to secondary_vio_pipe instead.\n\
 *\n\
 * secondary_vio_pipe:\n\
 *         Secondary pipe to subscribe to for VIO data. Must be a standard libmodal-pipe\n\
 *         vio_data_t type. Default is ov for openvins. If no data is available on this\n\
 *         pipe then voxl-vision-hub will subscribe to the primary vio_pipe instead.\n\
 *         Set to an empty string to disable. Default: ov\n\
 *\n\
 * en_reset_vio_if_initialized_inverted:\n\
 *         For VIO algorithms like qVIO that can initialize in any orientation\n\
 *         and output their estimate of the gravity vector, we suggest leaving\n\
 *         this enabled to allow vvpx4 to automatically send the reset signal\n\
 *         back to the VIO pipe if VIO was initialized upside-down or sufficiently\n\
 *         off-level. Helpful if the user powers on a drone while carrying it to\n\
 *         the flight area and VIO starts too early.\n\
 *\n\
 * vio_warmup_s:\n\
 *         Wait this for this amount of time of having good VIO data before\n\
 *         actually starting to send to PX4. This helps stop EKF2 getting\n\
 *         confused if VIO flickers in and out while struggling to init.\n\
 *         Set to 0 to disable the feature.\n\
 *\n\
 * send_odom_while_failed:\n\
 *         On by default. Send Odometry messages to PX4 with a quality of -1 when\n\
 *         VIO indicates a failure so EKF2 can start dead reckoning.\n\
 *         This MAY need to be turned off with PX4 versions older than 1.14\n\
 *         since the quality metric was no implemented prior to PX4 1.14\n\
 *\n\
 * ##############################################################################\n\
 * ## APQ8096-only Features\n\
 * ##############################################################################\n\
 *\n\
 * en_set_clock_from_gps:\n\
 *         Enable setting the VOXL system time to GPS time if no NTP server can be\n\
 *         reached via network to set the time.\n\
 *\n\
 * en_force_onboard_mav1_mode:\n\
 *         Force PX4 to use onboard mode for mavlink 1 channel which is the channel\n\
 *         apq8096 (VOXL1) uses to communicate UART Mavlink with PX4. Not applicable\n\
 *         to qrb5165-based platforms. Sets the MAV1_MODE PX4 param.\n\
 *\n\
 * en_reset_px4_on_error:\n\
 *         Trigger a reboot of PX4 one some of PX4's unrecoverable errors,\n\
 *         Not applicable to qrb5165\n\
 *         Yaw estimate error & High Accelerometer Bias and both detected\n\
 *\n\
 * ##############################################################################\n\
 * ## Misc Features\n\
 * ##############################################################################\n\
 *\n\
 * horizon_cal_tolerance:\n\
 *         Allowable standard deviation in roll/pitch values to consider the drone\n\
 *         stable enough in flight to do a PX4 horizon calibration. Default is 0.45,\n\
 *         you can increase this slightly if flying in a small indoor area or with\n\
 *         a drone that does not hold still very well.\n\
 *         See https://docs.modalai.com/calibrate-px4-horizon/\n\
 *\n\
 * en_hitl:\n\
 *         Enable Hardware In The Loop (HITL) testing extensions. Disabled by default.\n\
 *\n\
 * ##############################################################################\n\
 * ## offboard mode config\n\
 * ##############################################################################\n\
 *\n\
 * offboard_mode: The following are valid strings\n\
 *     off: VVPX4 will not send any offboard commands to PX4\n\
 *     figure_eight: Default value, VVPX4 commands PX4 to fly a figure 8 path\n\
 *     follow_tag: Drone will follow an apriltag around. Very dangerous, not\n\
 *                 recommended for customer use, for ModalAI R&D only.\n\
 *     trajectory: VVPX4 receives polynomial trajectories by pipe and commands\n\
 *                 PX4 to follow the trajectory path. Still in development.\n\
 *     backtrack:  Drone will replay, in reverse order, the last few seconds of it's\n\
 *                 position including yaw. This is useful when the drone loses\n\
 *                 the communication link and needs to get back to a place where\n\
 *                 it is able to regain the link. This mode will notice when the\n\
 *                 RC link goes away and sends a command to px4 to enter offboard mode.\n\
 *     wps:        read waypoints in local coordinate system\n\
 *\n\
 * follow_tag_id:\n\
 *         Apriltag ID to follow in follow_tag mode\n\
 *\n\
 * figure_eight_move_home:\n\
 *         Enable by default, resets the center of the figure 8 path to wherever\n\
 *         the drone is when flipped into offboard mode. When disabled, the drone\n\
 *         will quickly fly back to the XYZ point 0,0,-1.5 in VIO frame before\n\
 *         starting the figure 8. Disabling this feature can be dangerous if VIO\n\
 *         has drifted significantly.\n\
 *\n\
 * wps_move_home:\n\
 *         Enable by default, resets the center of the wps path to wherever\n\
 *         the drone is when flipped into offboard mode. When disabled, the drone\n\
 *         will quickly fly back to the XYZ point 0,0,-1.5 in VIO frame before\n\
 *         starting the figure 8. Disabling this feature can be dangerous if VIO\n\
 *         has drifted significantly.\n\
 *\n\
 * robot_radius:\n\
 *         Robot radius to use when checking collisions within the trajectory monitor.\n\
 *         The trajectory monitor is only active when in trajectory mode\n\
 *\n\
 * collision_sampling_dt:\n\
 *         The time step to sample along the polynomials by when checking for collisions\n\
 *         in the collision monitor.\n\
 *\n\
 * max_lookahead_distance:\n\
 *         Maximum distance to look along the trajectory. Sensor data further out can be\n\
 *         unrealiable so keeping this value small reduces false positives\n\
 *\n\
 * backtrack_seconds:\n\
 *         Number of seconds worth of position data to store for replay in backtrack mode.\n\
 *\n\
 * backtrack_rc_chan:\n\
 *         RC channel to monitor for transitions into and out of backtrack mode.\n\
 *\n\
 * backtrack_rc_thresh:\n\
 *         Value above which backtrack is considered enabled on the configured RC channel.\n\
 *\n\
 * ##############################################################################\n\
 * ## Fixed Frame Tag Relocalization\n\
 * ##############################################################################\n\
 *\n\
 * en_tag_fixed_frame:\n\
 *         Enable fixed frame relocalization via voa_inputs.\n\
 *         See: https://docs.modalai.com/voxl-vision-px4-apriltag-relocalization/\n\
 *\n\
 * fixed_frame_filter_len:\n\
 *         Length of the moving average filter to use for smooth relocalization\n\
 *         when a tag is detected. Default is 5, a longer filter will result in\n\
 *         smoother behavior when a new tag comes into view. Set to 1 to do no\n\
 *         filtering at all and assume every tag detection is accurate.\n\
 *\n\
 * en_transform_mavlink_pos_setpoints_from_fixed_frame:\n\
 *         When enabled, mavlink position_target_local_ned_t commands received on\n\
 *         via UDP will be assumed to be in fixed frame and are then transformed\n\
 *         to local frame before being sent to PX4. This allows offboard mode\n\
 *         position commands from MAVROS/MAVSDK to be in fixed frame relative to\n\
 *         voa_inputs even though PX4/EKF2 operates in local frame relative to where\n\
 *         VIO initialized.\n\
 *\n\
 * ##############################################################################\n\
 * ## Collision Prevention (VOA)\n\
 * ## Settings for configuring Mavlink data sent to Autopilot for VOA\n\
 * ##############################################################################\n\
 *\n\
 * en_voa:\n\
 *         Enable processing of DFS and TOF data to be sent to PX4 as mavlink\n\
 *         obstacle_distance messages for collision prevention in position mode.\n\
 *\n\
 * voa_lower_bound_m & voa_upper_bound_m:\n\
 *         VOA ignores obstacles above and below the upper and lower bounds.\n\
 *         Remember, Z points downwards in body and NED frames, so the lower bound\n\
 *         is a positive number, and the upper bound is a negative number.\n\
 *         Defaults are lower: 0.15  upper: -0.15 Units are in meters.\n\
 *\n\
 * voa_memory_s:\n\
 *         number of seconds to keep track of sensor readings for VOA\n\
 *         default: 1.0\n\
 *\n\
 * voa_max_pc_per_fusion:\n\
 *         maximum number of sensor samples (points clouds) to fuse for every\n\
 *         mavlink transmision. Default is 100 so that voa_memory_s determines\n\
 *         when to discard old data instead. set this to 1 if you only want to use\n\
 *         the most recent sensor sample for example. If you start severly limiting\n\
 *         the number of point clouds used per fusion, you will also need to lower\n\
 *         voa_pie_threshold.\n\
 *\n\
 * voa_pie_min_dist_m:\n\
 *         minimum distance from the drone's center of mass to consider a sensor\n\
 *         sample a valid point for mavlink transmission.\n\
 *\n\
 * voa_pie_max_dist_m:\n\
 *         minimum distance from the drone's center of mass to consider a sensor\n\
 *         sample a valid point for mavlink transmission. Note this is and can be\n\
 *         different from the individual sensor limits.\n\
 *\n\
 * voa_pie_under_trim_m:\n\
 *         VOA discards points in a bubble under the drone with this radius.\n\
 *         default 1.0. This helps the drone approach an obstacle, stop\n\
 *         ascend, and continue forward smoothly over the top.\n\
 *\n\
 * voa_pie_threshold:\n\
 *         Minimum number of points that must appear in and adjacent to a pie\n\
 *         segment to consider it populated. Default 3\n\
 *\n\
 * voa_send_rate_hz:\n\
 *         Rate to send VOA mavlink data to autopilot. Independent from the\n\
 *         sensor input rates. Default 20\n\
 *\n\
 * voa_pie_slices:\n\
 *         number of slices to divide the 360 degree span around the drone into.\n\
 *         default 36 (10 degree slices)\n\
 *\n\
 * voa_pie_bin_depth_m:\n\
 *         Radial depth of each bin during the pie binning step. Default 0.15\n\
 *\n\
 * ##############################################################################\n\
 * ## Collision Prevention (VOA) Input Configuration\n\
 * ## Settings for configuring pipe data sources for VOA\n\
 * ##############################################################################\n\
 *\n\
 * voa_inputs:\n\
 *         Array of pipes to subscribe to for use with VOA, up to 10 supported\n\
 *         Each entry has 4 fields:\n\
 *\n\
 * Fields:\n\
 *    enabled:      true or false, it's safe to leave this enabled when the pipe is missing\n\
 *    type:         can be point_cloud, tof, or rangefinder\n\
 *    input_pipe:   pipe name, e.g. stereo_front_pc, rangefinders, tof, etc\n\
 *    frame:        frame of reference, should be listed in /etc/modalai/extrinsics/conf\n\
 *    max_depth:    trim away points with z greater than this\n\
 *    min_depth:    trim away points with z less than this\n\
 *    cell_size:    size of 3d voxel grid cells, increase for more downsampling\n\
 *    threshold:    num points that must exist in or adjacent to a cell to consider it\n\
 *                      populated, set to 1 to disable threasholding\n\
 *    x_fov_deg:    FOV of the sensor in the x direction, typically width\n\
 *    y_fov_deg:    FOV of the sensor in the y direction, typically height\n\
 *    conf_cutoff:  discard points below this confidence, only applicable to TOF\n\
 *\n\
 */\n"


// config file version detection
int config_file_version = 0;

// define all the "Extern" variables from config_file.h, defaults where possible

// MAVROS / MAVSDK
int en_localhost_mavlink_udp;
int localhost_udp_port_number;

// VIO
int en_vio;
char vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
char secondary_vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
char vfc_vio_pipe[MODAL_PIPE_MAX_PATH_LEN];
char vfc_traj_csv[MODAL_PIPE_MAX_PATH_LEN];
int en_send_vio_to_qgc;
int en_reset_vio_if_initialized_inverted;
float vio_warmup_s;
int send_odom_while_failed;

// apq8096-only features, make sure they are OFF for other platforms
int en_set_clock_from_gps = 0;
int en_force_onboard_mav1_mode = 0;
int en_reset_px4_on_error = 0;

// misc features
float horizon_cal_tolerance;
int en_hitl;

// offboard mode config
offboard_mode_t offboard_mode;
int follow_tag_id;
int figure_eight_move_home;
int wps_move_home;

// offboard WPS
float wps_timeout;
float wps_stride;
float wps_damp;
int wps_vfc_mission;
int wps_vfc_mission_loop;
float wps_vfc_mission_cruise_speed;
float wps_vfc_mission_to_ramp;
float wps_vfc_mission_to_kp;

float robot_radius;
double collision_sampling_dt;
float max_lookahead_distance;
int backtrack_seconds;
int backtrack_rc_chan;
int backtrack_rc_thresh;

// fixed frame
int en_tag_fixed_frame;
int fixed_frame_filter_len;
int en_transform_mavlink_pos_setpoints_from_fixed_frame;

// voxl flight controller params
vfc_params_t vfc_params;

// collision Prevention (VOA)
int   en_voa;
float voa_upper_bound_m;
float voa_lower_bound_m;
float voa_memory_s;
int   n_voa_inputs;
int   voa_max_pc_per_fusion;
float voa_pie_min_dist_m;
float voa_pie_max_dist_m;
float voa_pie_under_trim_m;
int   voa_pie_threshold;
int   voa_pie_slices;
float voa_pie_bin_depth_m;
float voa_send_rate_hz;
voa_input_t voa_inputs[MAX_VOA_INPUTS];



int config_file_print(void)
{
	const char* offboard_strings[] = OFFBOARD_STRINGS;
	const char* voa_type_strings[] = VOA_INTPUT_TYPE_STRINGS;
	printf("=================================================================");
	printf("\n");
	printf("Parameters as loaded from config file:\n");
	printf("config_file_version:        %d\n", config_file_version);
	printf("\n");
	printf("MAVROS / MAVSDK\n");
	printf("en_localhost_mavlink_udp    %d\n", en_localhost_mavlink_udp);
	printf("localhost_udp_port_number:  %d\n", localhost_udp_port_number);
	printf("\n");
	printf("VIO\n");
	printf("en_vio:                     %d\n", en_vio);
	printf("vio_pipe:                   %s\n", vio_pipe);
	printf("secondary_vio_pipe:         %s\n", secondary_vio_pipe);
	printf("en_reset_vio_if_initialized_inverted: %d\n", en_reset_vio_if_initialized_inverted);
	printf("vio_warmup_s:               %f\n", (double)vio_warmup_s);
	printf("send_odom_while_failed:     %d\n", send_odom_while_failed);
#ifdef PLATFORM_APQ8096
	printf("\n");
	printf("APQ8096-ONLY FEATURES\n");
	printf("en_set_clock_from_gps:      %d\n", en_set_clock_from_gps);
	printf("en_force_onboard_mav1_mode: %d\n", en_force_onboard_mav1_mode);
	printf("en_reset_px4_on_error:      %d\n", en_reset_px4_on_error);
#endif
	printf("\n");
	printf("MISC FEATURES\n");
	printf("horizon_cal_tolerance:      %f\n", (double)horizon_cal_tolerance);
	printf("en_hitl:                    %d\n", en_hitl);
	printf("OFFBOARD MODE\n");
	printf("offboard_mode:              %s\n", offboard_strings[offboard_mode]);
	printf("follow_tag_id:              %d\n", follow_tag_id);
	printf("figure_eight_move_home:     %d\n", figure_eight_move_home);
	printf("wps_move_home:     %d\n", wps_move_home);
	printf("wps_timeout:     %f\n", (double)wps_timeout);
	printf("wps_damp:     %f\n", (double)wps_damp);
	printf("wps_vfc_mission:     %s\n", wps_vfc_mission ? "true" : "false");
	printf("wps_vfc_mission_loop:     %s\n", wps_vfc_mission_loop ? "true" : "false");
	printf("wps_vfc_mission_to_ramp:     %f\n", (double) wps_vfc_mission_to_ramp);
	printf("wps_vfc_mission_cruise_speed:     %f\n", (double) wps_vfc_mission_cruise_speed);
	printf("wps_vfc_mission_to_kp:     %f\n", (double) wps_vfc_mission_to_kp);
	printf("robot_radius:               %f\n", (double)robot_radius);
	printf("collision_sampling_dt:      %f\n", collision_sampling_dt);
	printf("max_lookahead_distance:     %f\n", (double)max_lookahead_distance);
	printf("backtrack_seconds     :     %d\n", backtrack_seconds);
	printf("backtrack_rc_chan     :     %d\n", backtrack_rc_chan);
	printf("backtrack_rc_thresh   :     %d\n", backtrack_rc_thresh);
	printf("FIXED FRAME RELOCALIZATION\n");
	printf("en_tag_fixed_frame:         %d\n", en_tag_fixed_frame);
	printf("fixed_frame_filter_len:     %d\n", fixed_frame_filter_len);
	printf("en_transform_mavlink_pos_setpoints_from_fixed_frame:%d\n",\
							en_transform_mavlink_pos_setpoints_from_fixed_frame);
	printf("\n");
	printf("VOXL FLIGHT CONTROLLER (VFC)\n");
	printf("vfc_rate                    %f\n", (double)vfc_params.rate);
	printf("vfc_rc_chan_min             %d\n", vfc_params.rc_chan_min);
	printf("vfc_rc_chan_max             %d\n", vfc_params.rc_chan_max);
	printf("vfc_thrust_ch               %d\n", vfc_params.thrust_ch);
	printf("vfc_roll_ch                 %d\n", vfc_params.roll_ch);
	printf("vfc_pitch_ch                %d\n", vfc_params.pitch_ch);
	printf("vfc_yaw_ch                  %d\n", vfc_params.yaw_ch);
	printf("vfc_submode_ch              %d\n", vfc_params.submode_ch);
	printf("vfc_alt_mode_rc_min         %d\n", vfc_params.alt_mode_rc_min);
	printf("vfc_alt_mode_rc_max         %d\n", vfc_params.alt_mode_rc_max);
	printf("vfc_flow_mode_rc_min        %d\n", vfc_params.flow_mode_rc_min);
	printf("vfc_flow_mode_rc_max        %d\n", vfc_params.flow_mode_rc_max);
	printf("vfc_hybrid_flow_mode_rc_min %d\n", vfc_params.hybrid_flow_mode_rc_min);
	printf("vfc_hybrid_flow_mode_rc_max %d\n", vfc_params.hybrid_flow_mode_rc_max);
	printf("vfc_position_mode_rc_min    %d\n", vfc_params.position_mode_rc_min);
	printf("vfc_position_mode_rc_max    %d\n", vfc_params.position_mode_rc_max);
	printf("vfc_traj_mode_rc_min        %d\n", vfc_params.traj_mode_rc_min);
	printf("vfc_traj_mode_rc_max        %d\n", vfc_params.traj_mode_rc_max);
	printf("vfc_yaw_deadband            %d\n", vfc_params.yaw_deadband);
	printf("vfc_vxy_deadband            %d\n", vfc_params.vxy_deadband);
	printf("vfc_vz_deadband             %d\n", vfc_params.vz_deadband);
	printf("vfc_min_thrust              %f\n", (double)vfc_params.min_thrust);
	printf("vfc_max_thrust              %f\n", (double)vfc_params.max_thrust);
	printf("vfc_tilt_max                %f\n", (double)vfc_params.tilt_max);
	printf("vfc_yaw_rate_max            %f\n", (double)vfc_params.yaw_rate_max);
	printf("vfc_thrust_hover            %f\n", (double)vfc_params.thrust_hover);
	printf("vfc_vz_max                  %f\n", (double)vfc_params.vz_max);
	printf("vfc_kp_z                    %f\n", (double)vfc_params.kp_z);
	printf("vfc_kd_z                    %f\n", (double)vfc_params.kd_z);
	printf("vfc_vxy_max                 %f\n", (double)vfc_params.vxy_max);
	printf("vfc_kp_xy                   %f\n", (double)vfc_params.kp_xy);
	printf("vfc_kd_xy                   %f\n", (double)vfc_params.kd_xy);
	printf("vfc_kp_z_vio                %f\n", (double)vfc_params.kp_z_vio);
	printf("vfc_kd_z_vio                %f\n", (double)vfc_params.kd_z_vio);
	printf("vfc_kp_xy_vio               %f\n", (double)vfc_params.kp_xy_vio);
	printf("vfc_kd_xy_vio               %f\n", (double)vfc_params.kd_xy_vio);
	printf("vfc_w_filt_xy_vio           %f\n", (double)vfc_params.w_filt_xy_vio);
	printf("vfc_w_filt_xy_flow          %f\n", (double)vfc_params.w_filt_xy_flow);
	printf("vfc_vel_ff_factor_vio       %f\n", (double)vfc_params.vel_ff_factor_vio);
	printf("vfc_xy_acc_limit_vio        %f\n", (double)vfc_params.xy_acc_limit_vio);
	printf("vfc_max_z_delta             %f\n", (double)vfc_params.max_z_delta);
	printf("vfc_att_transition_time     %f\n", (double)vfc_params.att_transition_time);
	printf("vfc_stick_move_threshold    %f\n", (double)vfc_params.stick_move_threshold);
	printf("vfc_flow_transition_time    %f\n", (double)vfc_params.flow_transition_time);
	printf("vfc_q_min                   %d\n", vfc_params.q_min);
	printf("vfc_points_min              %d\n", vfc_params.points_min);
	printf("vfc_en_submode_announcement %d\n", vfc_params.en_submode_announcement);
	printf("vfc_disable_fallback        %d\n", vfc_params.disable_fallback);

	printf("\n");
	printf("COLLISION PREVENTION (VOA)\n");
	printf("en_voa:                     %d\n", en_voa);
	printf("voa_upper_bound_m:          %f\n", (double)voa_upper_bound_m);
	printf("voa_lower_bound_m:          %f\n", (double)voa_lower_bound_m);
	printf("voa_memory_s:               %f\n", (double)voa_memory_s);
	printf("voa_max_pc_per_fusion:      %d\n", voa_max_pc_per_fusion);
	printf("voa_pie_min_dist_m:         %f\n", (double)voa_pie_min_dist_m);
	printf("voa_pie_max_dist_m:         %f\n", (double)voa_pie_max_dist_m);
	printf("voa_pie_under_trim_m:       %f\n", (double)voa_pie_under_trim_m);
	printf("voa_pie_threshold:          %d\n", voa_pie_threshold);
	printf("voa_pie_slices:             %d\n", voa_pie_slices);
	printf("voa_pie_bin_depth_m:        %f\n", (double)voa_pie_bin_depth_m);
	printf("voa_send_rate_hz:           %f\n", (double)voa_send_rate_hz);
	printf("\n");
	for(int i=0; i<n_voa_inputs; i++){
		printf("voa_input #%d\n", i);
		printf("    enabled:            %d\n", voa_inputs[i].enabled);
		printf("    type:               %s\n", voa_type_strings[voa_inputs[i].type]);
		printf("    input_pipe:         %s\n", voa_inputs[i].input_pipe);
		printf("    frame:              %s\n", voa_inputs[i].frame);
		printf("    max_depth:          %f\n", (double)voa_inputs[i].max_depth);
		printf("    min_depth:          %f\n", (double)voa_inputs[i].min_depth);
		printf("    cell_size:          %f\n", (double)voa_inputs[i].cell_size);
		printf("    threshold:          %d\n", voa_inputs[i].threshold);
		printf("    x_fov_deg:          %f\n", (double)voa_inputs[i].x_fov_deg);
		printf("    y_fov_deg:          %f\n", (double)voa_inputs[i].y_fov_deg);
		printf("    conf_cutoff:        %d\n", voa_inputs[i].conf_cutoff);
	}
	printf("=================================================================");
	printf("\n");
	return 0;
}


static void _move_old_file_if_necessary(void)
{
	int new_exists = 1;
	int old_exists = 1;

	if(access(VOXL_VISION_HUB_CONF_FILE, F_OK) == -1) new_exists = 0;
	if(access(VOXL_VISION_OLD_CONF_FILE, F_OK) == -1) old_exists = 0;

	// move old file to new location
	if(!new_exists && old_exists){
		printf("Automatically migrating old config file %s to new location: %s\n",\
							VOXL_VISION_OLD_CONF_FILE, VOXL_VISION_HUB_CONF_FILE);
		rename(VOXL_VISION_OLD_CONF_FILE, VOXL_VISION_HUB_CONF_FILE);
	}
	else if(new_exists && old_exists){
		printf("Automatically deleting old config file %s \n", VOXL_VISION_OLD_CONF_FILE);
		remove(VOXL_VISION_OLD_CONF_FILE);
	}
	return;

}


int config_file_load(void)
{
	int i, ret;
	const char* offboard_strings[] = OFFBOARD_STRINGS;
	const int n_modes = sizeof(offboard_strings)/sizeof(offboard_strings[0]);
	const char* voa_type_strings[] = VOA_INTPUT_TYPE_STRINGS;

	// some defaults that are used in multiple places
	const float default_voa_upper_bound_m = -0.15f;
	const float default_voa_lower_bound_m =  0.15f;

	// support transition during voxl-vision-px4 to voxl-vision-hub rename
	_move_old_file_if_necessary();

	// check if the file exists and make a new one if not
	ret = json_make_empty_file_with_header_if_missing(VOXL_VISION_HUB_CONF_FILE,\
																		FILE_HEADER);
	if(ret < 0) return -1;
	else if(ret>0) fprintf(stderr, "Created new json file: %s\n", VOXL_VISION_HUB_CONF_FILE);

	// read the data in
	cJSON* parent = json_read_file(VOXL_VISION_HUB_CONF_FILE);
	if(parent==NULL) return -1;
	cJSON* item;

	// check config file version
	json_fetch_int_with_default(    parent, "config_file_version", &config_file_version, 1);

	// parse each item
	// MAVROS / MAVSDK
	json_fetch_bool_with_default(   parent, "en_localhost_mavlink_udp", &en_localhost_mavlink_udp, 0);
	json_fetch_int_with_default(    parent, "localhost_udp_port_number", &localhost_udp_port_number, 14551);

	// VIO
	json_fetch_bool_with_default(   parent, "en_vio", &en_vio, 1);
	json_fetch_string_with_default( parent, "vio_pipe", vio_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "qvio");
	json_fetch_string_with_default( parent, "secondary_vio_pipe", secondary_vio_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "ov");
	json_fetch_string_with_default( parent, "vfc_vio_pipe", vfc_vio_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "ov");
	json_fetch_bool_with_default(   parent, "en_reset_vio_if_initialized_inverted", &en_reset_vio_if_initialized_inverted, 1);
	json_fetch_float_with_default(  parent, "vio_warmup_s", &vio_warmup_s, 3.0f);
	json_fetch_bool_with_default(   parent, "send_odom_while_failed", &send_odom_while_failed, 1);
	// APQ8096-only features
#ifdef PLATFORM_APQ8096
	json_fetch_bool_with_default(   parent, "en_set_clock_from_gps", &en_set_clock_from_gps, 1);
	json_fetch_bool_with_default(   parent, "en_force_onboard_mav1_mode", &en_force_onboard_mav1_mode, 1);
	json_fetch_bool_with_default(   parent, "en_reset_px4_on_error", &en_reset_px4_on_error, 1);
#endif
	// misc features
	json_fetch_float_with_default(  parent, "horizon_cal_tolerance", &horizon_cal_tolerance, 0.50f);
	json_fetch_bool_with_default(   parent, "en_hitl", &en_hitl, 0);
	// offboard mode
	json_fetch_enum_with_default(   parent, "offboard_mode", (int*)&offboard_mode, offboard_strings, n_modes, 1);
	json_fetch_int_with_default(    parent, "follow_tag_id", &follow_tag_id, 0);
	json_fetch_bool_with_default(   parent, "figure_eight_move_home", &figure_eight_move_home, 1);
	json_fetch_float_with_default(  parent, "robot_radius", &robot_radius, 0.3);
	json_fetch_double_with_default( parent, "collision_sampling_dt", &collision_sampling_dt, 0.1);
	json_fetch_float_with_default(  parent, "max_lookahead_distance", &max_lookahead_distance, 1.0);
	json_fetch_int_with_default(    parent, "backtrack_seconds", &backtrack_seconds, 60);
	json_fetch_int_with_default(    parent, "backtrack_rc_chan", &backtrack_rc_chan, 8);
	json_fetch_int_with_default(    parent, "backtrack_rc_thresh", &backtrack_rc_thresh, 1500);

	json_fetch_bool_with_default(   parent, "wps_move_home", &wps_move_home, 1);
	json_fetch_float_with_default(  parent, "wps_stride", &wps_stride, 0.0);
	json_fetch_float_with_default(  parent, "wps_timeout", &wps_timeout, 0.0);
	json_fetch_float_with_default(  parent, "wps_damp", &wps_damp, 1.0);
	json_fetch_bool_with_default(   parent, "wps_vfc_mission", &wps_vfc_mission, 1);
	json_fetch_bool_with_default(   parent, "wps_vfc_mission_loop", &wps_vfc_mission_loop, 0);
	json_fetch_float_with_default(  parent, "wps_vfc_mission_to_ramp", &wps_vfc_mission_to_ramp, 25);
	json_fetch_float_with_default(  parent, "wps_vfc_mission_to_kp", &wps_vfc_mission_to_kp, 0.1);
	json_fetch_float_with_default(  parent, "wps_vfc_mission_cruise_speed", &wps_vfc_mission_cruise_speed, 1.0);

	// fixed frame
	json_fetch_bool_with_default(   parent, "en_tag_fixed_frame", &en_tag_fixed_frame, 0);
	json_fetch_int_with_default(    parent, "fixed_frame_filter_len", &fixed_frame_filter_len, 5);
	json_fetch_bool_with_default(   parent, "en_transform_mavlink_pos_setpoints_from_fixed_frame", &en_transform_mavlink_pos_setpoints_from_fixed_frame, 0);
	// voxl flight controller (vfc)
	json_fetch_float_with_default(  parent, "vfc_rate", &vfc_params.rate, 100.0);
	json_fetch_int_with_default(    parent, "vfc_rc_chan_min", &vfc_params.rc_chan_min, 980);
	json_fetch_int_with_default(    parent, "vfc_rc_chan_max", &vfc_params.rc_chan_max, 2020);
	json_fetch_int_with_default(    parent, "vfc_thrust_ch", &vfc_params.thrust_ch, 3);
	json_fetch_int_with_default(    parent, "vfc_roll_ch", &vfc_params.roll_ch, 1);
	json_fetch_int_with_default(    parent, "vfc_pitch_ch", &vfc_params.pitch_ch, 2);
	json_fetch_int_with_default(    parent, "vfc_yaw_ch", &vfc_params.yaw_ch, 4);
	json_fetch_int_with_default(    parent, "vfc_submode_ch", &vfc_params.submode_ch, 6);
	json_fetch_int_with_default(    parent, "vfc_alt_mode_rc_min", &vfc_params.alt_mode_rc_min, 0);
	json_fetch_int_with_default(    parent, "vfc_alt_mode_rc_max", &vfc_params.alt_mode_rc_max, 0);
	json_fetch_int_with_default(    parent, "vfc_flow_mode_rc_min", &vfc_params.flow_mode_rc_min, 0);
	json_fetch_int_with_default(    parent, "vfc_flow_mode_rc_max", &vfc_params.flow_mode_rc_max, 0);
	json_fetch_int_with_default(    parent, "vfc_hybrid_flow_mode_rc_min", &vfc_params.hybrid_flow_mode_rc_min, 0);
	json_fetch_int_with_default(    parent, "vfc_hybrid_flow_mode_rc_max", &vfc_params.hybrid_flow_mode_rc_max, 0);
	json_fetch_int_with_default(    parent, "vfc_position_mode_rc_min", &vfc_params.position_mode_rc_min, 0);
	json_fetch_int_with_default(    parent, "vfc_position_mode_rc_max", &vfc_params.position_mode_rc_max, 2100);
	json_fetch_int_with_default(    parent, "vfc_traj_mode_rc_min", &vfc_params.traj_mode_rc_min, 0);
	json_fetch_int_with_default(    parent, "vfc_traj_mode_rc_max", &vfc_params.traj_mode_rc_max, 0);
	json_fetch_int_with_default(    parent, "vfc_yaw_deadband", &vfc_params.yaw_deadband, 30);
	json_fetch_int_with_default(    parent, "vfc_vxy_deadband", &vfc_params.vxy_deadband, 50);
	json_fetch_int_with_default(    parent, "vfc_vz_deadband", &vfc_params.vz_deadband, 150);
	json_fetch_float_with_default(  parent, "vfc_min_thrust", &vfc_params.min_thrust, 0.0);
	json_fetch_float_with_default(  parent, "vfc_max_thrust", &vfc_params.max_thrust, 0.8);
	json_fetch_float_with_default(  parent, "vfc_tilt_max", &vfc_params.tilt_max, 0.436);
	json_fetch_float_with_default(  parent, "vfc_yaw_rate_max", &vfc_params.yaw_rate_max, 3.0);
	json_fetch_float_with_default(  parent, "vfc_thrust_hover", &vfc_params.thrust_hover, 0.5);
	json_fetch_float_with_default(  parent, "vfc_vz_max", &vfc_params.vz_max, 1.0);
	json_fetch_float_with_default(  parent, "vfc_kp_z", &vfc_params.kp_z, 5.29);
	json_fetch_float_with_default(  parent, "vfc_kd_z", &vfc_params.kd_z, 5.98);
	json_fetch_float_with_default(  parent, "vfc_vxy_max", &vfc_params.vxy_max, 3.0);
	json_fetch_float_with_default(  parent, "vfc_kp_xy", &vfc_params.kp_xy, 0.64);
	json_fetch_float_with_default(  parent, "vfc_kd_xy", &vfc_params.kd_xy, 2.56);
	json_fetch_float_with_default(  parent, "vfc_kp_z_vio", &vfc_params.kp_z_vio, 5.29);
	json_fetch_float_with_default(  parent, "vfc_kd_z_vio", &vfc_params.kd_z_vio, 5.98);
	json_fetch_float_with_default(  parent, "vfc_kp_xy_vio", &vfc_params.kp_xy_vio, 3.24);
	json_fetch_float_with_default(  parent, "vfc_kd_xy_vio", &vfc_params.kd_xy_vio, 3.96);
	json_fetch_float_with_default(  parent, "vfc_w_filt_xy_vio", &vfc_params.w_filt_xy_vio, 10.0);
	json_fetch_float_with_default(  parent, "vfc_w_filt_xy_flow", &vfc_params.w_filt_xy_flow, 3.0);
	json_fetch_float_with_default(  parent, "vfc_vel_ff_factor_vio", &vfc_params.vel_ff_factor_vio, 0.9);
	json_fetch_float_with_default(  parent, "vfc_xy_acc_limit_vio", &vfc_params.xy_acc_limit_vio, 2.5);
	json_fetch_float_with_default(  parent, "vfc_max_z_delta", &vfc_params.max_z_delta, 3.0);
	json_fetch_float_with_default(  parent, "vfc_att_transition_time", &vfc_params.att_transition_time, 0.5);
	json_fetch_float_with_default(  parent, "vfc_stick_move_threshold", &vfc_params.stick_move_threshold, 30);
	json_fetch_float_with_default(  parent, "vfc_flow_transition_time", &vfc_params.flow_transition_time, 1);
	json_fetch_int_with_default(    parent, "vfc_q_min", &vfc_params.q_min, 10);
	json_fetch_int_with_default(    parent, "vfc_points_min", &vfc_params.points_min, 7);
	json_fetch_int_with_default(    parent, "vfc_en_submode_announcement", &vfc_params.en_submode_announcement, 1);
	json_fetch_bool_with_default(   parent, "vfc_disable_fallback", &vfc_params.disable_fallback, 0);
	json_fetch_string_with_default( parent, "vfc_traj_csv", vfc_traj_csv, MODAL_PIPE_MAX_PATH_LEN-1, "/data/voxl-vision-hub/traj.csv");

	// collision prevention (voa)
	json_fetch_bool_with_default(   parent, "en_voa", &en_voa, 1);
	json_fetch_float_with_default(  parent, "voa_upper_bound_m", &voa_upper_bound_m, default_voa_upper_bound_m);
	json_fetch_float_with_default(  parent, "voa_lower_bound_m", &voa_lower_bound_m, default_voa_lower_bound_m);
	json_fetch_float_with_default(  parent, "voa_voa_memory_s", &voa_memory_s, 1.0);
	json_fetch_int_with_default(    parent, "voa_max_pc_per_fusion", &voa_max_pc_per_fusion, 100);
	json_fetch_float_with_default(  parent, "voa_pie_max_dist_m", &voa_pie_max_dist_m, 20.0);
	json_fetch_float_with_default(  parent, "voa_pie_min_dist_m", &voa_pie_min_dist_m, 0.25);
	json_fetch_float_with_default(  parent, "voa_pie_under_trim_m", &voa_pie_under_trim_m, 1.0);
	json_fetch_int_with_default(    parent, "voa_pie_threshold", &voa_pie_threshold, 3);
	json_fetch_float_with_default(  parent, "voa_send_rate_hz", &voa_send_rate_hz, 20.0);
	json_fetch_int_with_default(    parent, "voa_pie_slices", &voa_pie_slices, 36);
	json_fetch_float_with_default(  parent, "voa_pie_bin_depth_m", &voa_pie_bin_depth_m, 0.15);


	// VOA input sources
	cJSON* voa_inputs_json = json_fetch_array_and_add_if_missing(parent, "voa_inputs", &n_voa_inputs);
	if(n_voa_inputs > MAX_VOA_INPUTS){
		fprintf(stderr, "array of voa_inputs should be no more than %d long\n", MAX_VOA_INPUTS);
		return -1;
	}

	// if array is missing, add defaults in
	if(n_voa_inputs == 0){ // add two empty objects to the array. Defaults will be added by json_fetch_xxx helpers next
		n_voa_inputs = 5;

		// voxl1 dfs pipe
		i=0;
		item = cJSON_CreateObject();
		cJSON_AddItemToArray(voa_inputs_json, item);
		json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
		json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_POINT_CLOUD);
		json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "dfs_point_cloud");
		json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "stereo_l");

		json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, STEREO_MAX_DIST_M);
		json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, STEREO_MIN_DIST_M);
		json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, STEREO_DOWNSAMPLE_CELL_SIZE);
		json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, STEREO_DOWNSAMPLE_THRESHOLD);
		json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, STEREO_X_FOV_DEG);
		json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, STEREO_Y_FOV_DEG);
		json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, 0);

		// voxl2/rb5 front/rear pipes
		i=1;
		item = cJSON_CreateObject();
		cJSON_AddItemToArray(voa_inputs_json, item);
		json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
		json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_POINT_CLOUD);
		json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "stereo_front_pc");
		json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "stereo_front_l");

		json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, STEREO_MAX_DIST_M);
		json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, STEREO_MIN_DIST_M);
		json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, STEREO_DOWNSAMPLE_CELL_SIZE);
		json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, STEREO_DOWNSAMPLE_THRESHOLD);
		json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, STEREO_X_FOV_DEG);
		json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, STEREO_Y_FOV_DEG);
		json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, 0);

		i=2;
		item = cJSON_CreateObject();
		cJSON_AddItemToArray(voa_inputs_json, item);
		json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
		json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_POINT_CLOUD);
		json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "stereo_rear_pc");
		json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "stereo_rear_l");

		json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, STEREO_MAX_DIST_M);
		json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, STEREO_MIN_DIST_M);
		json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, STEREO_DOWNSAMPLE_CELL_SIZE);
		json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, STEREO_DOWNSAMPLE_THRESHOLD);
		json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, STEREO_X_FOV_DEG);
		json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, STEREO_Y_FOV_DEG);
		json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, 0);

		i=3;
		item = cJSON_CreateObject();
		cJSON_AddItemToArray(voa_inputs_json, item);
		json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
		json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_TOF);
		json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "tof");
		json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "tof");

		json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, TOF_MAX_DIST_M);
		json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, TOF_MIN_DIST_M);
		json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, TOF_DOWNSAMPLE_CELL_SIZE);
		json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, TOF_DOWNSAMPLE_THRESHOLD);
		json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, TOF_X_FOV_DEG);
		json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, TOF_Y_FOV_DEG);
		json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, TOF_CONFIDENCE_CUTOFF);

		i=4;
		item = cJSON_CreateObject();
		cJSON_AddItemToArray(voa_inputs_json, item);
		json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
		json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_RANGEFINDER);
		json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "rangefinders");
		json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "body");

		json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, STEREO_MAX_DIST_M);
		json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, STEREO_MIN_DIST_M);
		json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, STEREO_DOWNSAMPLE_CELL_SIZE);
		json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, STEREO_DOWNSAMPLE_THRESHOLD);
		json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, STEREO_X_FOV_DEG);
		json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, STEREO_Y_FOV_DEG);
		json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, 0);

		json_set_modified_flag(1); // log that we modified the parent manually
	}
	else{
		// copy out each item in the array
		for(i=0; i<n_voa_inputs; i++){
			cJSON* item = cJSON_GetArrayItem(voa_inputs_json, i);
			json_fetch_bool_with_default(item, "enabled", &voa_inputs[i].enabled, 1);
			json_fetch_enum_with_default(item, "type", (int*)&voa_inputs[i].type, voa_type_strings, N_VOA_INPUT_TYPES, VOA_POINT_CLOUD);
			json_fetch_string_with_default(item, "input_pipe", voa_inputs[i].input_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "PUT_YOUR_PIPE_NAME_HERE");
			json_fetch_string_with_default(item, "frame", voa_inputs[i].frame, 64, "PUT_YOUR_FRAME_NAME_HERE");

			if(voa_inputs[i].type == VOA_POINT_CLOUD){
				json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, STEREO_MAX_DIST_M);
				json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, STEREO_MIN_DIST_M);
				json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, STEREO_DOWNSAMPLE_CELL_SIZE);
				json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, STEREO_DOWNSAMPLE_THRESHOLD);
				json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, STEREO_X_FOV_DEG);
				json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, STEREO_Y_FOV_DEG);
				json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, 0);
			}else{
				json_fetch_float_with_default(item, "max_depth",  &voa_inputs[i].max_depth, TOF_MAX_DIST_M);
				json_fetch_float_with_default(item, "min_depth",  &voa_inputs[i].min_depth, TOF_MIN_DIST_M);
				json_fetch_float_with_default(item, "cell_size",  &voa_inputs[i].cell_size, TOF_DOWNSAMPLE_CELL_SIZE);
				json_fetch_int_with_default(item,   "threshold",  &voa_inputs[i].threshold, TOF_DOWNSAMPLE_THRESHOLD);
				json_fetch_float_with_default(item, "x_fov_deg",  &voa_inputs[i].x_fov_deg, TOF_X_FOV_DEG);
				json_fetch_float_with_default(item, "y_fov_deg",  &voa_inputs[i].y_fov_deg, TOF_Y_FOV_DEG);
				json_fetch_int_with_default(item,   "conf_cutoff",&voa_inputs[i].conf_cutoff, TOF_CONFIDENCE_CUTOFF);
			}
		}
	}

	// remove old fields to keep config file clean
	json_remove_if_present(parent, "en_auto_level_horizon");
	json_remove_if_present(parent, "horizon_cal_tol");
	json_remove_if_present(parent, "en_adsb");
	json_remove_if_present(parent, "adsb_uart_bus");
	json_remove_if_present(parent, "adsb_uart_baudrate");
	json_remove_if_present(parent, "px4_uart_bus");
	json_remove_if_present(parent, "px4_uart_baudrate");
	json_remove_if_present(parent, "qvio_auto_reset_quality");
	json_remove_if_present(parent, "en_secondary_qgc");
	json_remove_if_present(parent, "qgc_ip");
	json_remove_if_present(parent, "secondary_qgc_ip");
	json_remove_if_present(parent, "qgc_udp_port_number");
	json_remove_if_present(parent, "udp_mtu");
	json_remove_if_present(parent, "en_send_vio_to_qgc");
	json_remove_if_present(parent, "en_send_voa_to_qgc");


	// remove apq8096-only params on other platforms
#ifndef PLATFORM_APQ8096
	json_remove_if_present(parent, "en_set_clock_from_gps");
	json_remove_if_present(parent, "en_force_onboard_mav1_mode");
	json_remove_if_present(parent, "en_reset_px4_on_error");
#endif

	// check if we got any errors in that process
	if(json_get_parse_error_flag()){
		fprintf(stderr, "failed to parse data in %s\n", VOXL_VISION_HUB_CONF_FILE);
		cJSON_Delete(parent);
		return -1;
	}

	// check for automatically required config upgrades
	if( voa_lower_bound_m<1.251f && voa_lower_bound_m>1.249f && \
		voa_upper_bound_m<1.251f && voa_upper_bound_m>1.249f){
		fprintf(stderr, "WARNING: detected old incompatible settings for VOA upper/lower bounds\n");
		fprintf(stderr, "resetting back to defaults automatically\n");
		item = cJSON_GetObjectItem(parent, "voa_upper_bound_m");
		item->valuedouble = default_voa_upper_bound_m;
		voa_upper_bound_m = default_voa_upper_bound_m;
		item = cJSON_GetObjectItem(parent, "voa_lower_bound_m");
		item->valuedouble = default_voa_lower_bound_m;
		voa_lower_bound_m = default_voa_lower_bound_m;
		json_set_modified_flag(1);
	}

	// swap back to normal vio pipe if we were using the old extended packet
	if(strcmp(vio_pipe, "qvio_extended")==0){
		json_remove_if_present(parent, "vio_pipe");
		json_fetch_string_with_default( parent, "vio_pipe", vio_pipe, MODAL_PIPE_MAX_PATH_LEN-1, "qvio");
		json_set_modified_flag(1);
	}

	// write modified data to disk if neccessary
	if(json_get_modified_flag()){
		printf("The JSON config file data was modified during parsing, saving the changes to disk\n");
		json_write_to_file_with_header(VOXL_VISION_HUB_CONF_FILE, parent, FILE_HEADER);
	}
	cJSON_Delete(parent);


	// do some validation
	ret = 0;
	if(voa_lower_bound_m <= voa_upper_bound_m){
		fprintf(stderr, "ERROR parsing config file:\n");
		fprintf(stderr, "voa_lower_bound_m should be > voa_upper_bound_m\n");
		fprintf(stderr, "defaults are upper_bound=%0.2f lower_bound=%0.2f\n", (double)default_voa_upper_bound_m, (double)default_voa_lower_bound_m);
		ret = -1;
	}

	if(fixed_frame_filter_len<1){
		fprintf(stderr, "ERROR parsing config file:\n");
		fprintf(stderr, "param fixed_frame_filter_len in config file should be >=1\n");
		ret = -1;
	}

	if(voa_memory_s<0.05f){
		fprintf(stderr, "ERROR parsing config file:\n");
		fprintf(stderr, "param voa_memory_s should be >=0.05\n");
		ret = -1;
	}

	if(vio_warmup_s<0.0f){
		fprintf(stderr, "ERROR parsing config file\n");
		fprintf(stderr, "vio_warmup_s must be >=0\n");
		ret = -1;
	}

	for(i=0; i<n_voa_inputs; i++){
		if(voa_inputs[i].x_fov_deg <= 0 || voa_inputs[i].x_fov_deg > 170){
			fprintf(stderr, "ERROR parsing config file:\n");
			fprintf(stderr, "x_fov_deg must be >0 && <=170\n");
			ret = -1;
		}
		if(voa_inputs[i].y_fov_deg <= 0 || voa_inputs[i].y_fov_deg > 170){
			fprintf(stderr, "ERROR parsing config file:\n");
			fprintf(stderr, "y_fov_deg must be >0 && <=170\n");
			ret = -1;
		}
	}

	return ret;
}

