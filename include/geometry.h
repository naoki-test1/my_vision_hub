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


#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <rc_math.h>
#include <modal_pipe_interfaces.h>

#include "c_library_v2/common/mavlink.h"
#include "voxl_vision_hub.h"

/**
 * SPACIAL REFERENCE FRAME DESCRIPTIONS
 *
 * PX4 uses FRD (X Forward, Y Right and Z Down) for the local body frame. The
 * primary IMU on VOXL as mounted on the M500 and Flight Deck has X right, Y
 * forward, Z up. ModalAI Vision Lib VIO frame inherits this. This geometry
 * module serves to keep track of, convert betwee, and find the relation
 * between, the following spacial refrence frames.
 *
 * The order of these frames as described here can be considered "top to bottom"
 * where each frame is a child to the parent frame above it. Most of the time,
 * the geometry module records the rotation and translation from a child to its
 * parent so that it is computationally efficient to fetch the position of a
 * child to its parent. Most frequently, the location of the body with respect
 * to its great grandparent, the local frame. Where necessary, it additionally
 * records the inverse in memory to accelerate specific lookups.
 *
 *
 * FIXED FRAME: The top level parent frame. Like local frame (FRD) but aligned
 * to known fiducial markers (apriltags). The config file allows up to 64
 * apriltags to be configured, each with a known position and orientation in the
 * fixed frame. These are defined in the fixed_apriltags array in config.h. This
 * remains aligned to local frame until a tag is detected. desired setpoints in
 * fixed frame must be converted to local frame before being sent to PX4.
 *
 * LOCAL FRAME: (FRD) aligned with wherever drone was facing when VIO started.
 * It is centered on the ground directly underneath the center of mass. This
 * corresponds to px4/mavlink MAV_FRAME_LOCAL_FRD. Visual odometry is sent to
 * PX4 as position of body with respect to this frame. Position setpoints are
 * also sent to PX4 in local frame.
 *
 * VIO GRAVITY ALIGNED FRAME: Centered wherever VIO started with yaw aligned to
 * IMU but roll/pitch aligned with gravity. VIO raw output does not align with Z
 * along gravity but with wherever the IMU was on initialization which is not
 * particularly useful.
 *
 * VIO FRAME: Centered wherever VIO started and aligned with wherever IMU was
 * VIO started. This is NOT aligned with gravity and must be corrected to its
 * parent frame by supplying the geometry module with gravity vector estimates.
 *
 * IMU FRAME: Centered about IMU and aligned with IMU. Raw VIO output is the
 * position and rotation of this IMU frame to its parent (VIO frame).
 *
 * BODY FRAME: Centered at drone's center-of-mass and aligned with drone body
 * Forward-Right-Down. PX4 expects vision estimated velocity in this frame. PX4
 * expects vision estimated position as position of body wrt local. Note: PX4
 * allows configuration of offset between this frame and drone COM
 * (EKF2_EV_POS_X, Y, Z) but this should be set to 0 in PX4 as voxl-vision-px4
 * handles this offset from IMU to body frame.
 *
 * CAMERA FRAME: Standard open-cv camera frame with X right, Y down, and Z out
 * the lens. This is defined relative to the IMU by the tbc and ombc parameters
 * in the ModalAI vision lib config. This relation is optimized further in real
 * time by the VIO algorithm and passed into the geometry module with each new
 * VIO measurement.
 *
 * TAG FRAME: If you print out an apriltag on a piece of paper and put it on a
 * wall like a poster, X will point out the right of the paper, Y will point
 * down to the ground, and Z will point into the wall. If a camera is looking
 * straight at the apriltag, then the tag frame and camera frame will be
 * aligned. Apriltags are often placed on the ground, in which case the tag
 * frame WILL NOT align with local or fixed frame. great care must be taken when
 * describing the relation from tag to fixed frame in the config file.
 */


/**
 * Initialize the geometry module, call this before any of the other functions
 * This allocates memory for local transforms and pull in information from the
 * config file.
 *
 * @return     0 on success, -1 on failure
 */
int geometry_init(void);

/**
 * @brief      enable or disable the printing of debug messages regarding the
 *             calculation of fixed frame offsets from apriltag detections
 *
 * @param[in]  debug  0 to disable, nonzero to enable
 */
void geometry_en_print_fixed_frame_debug(int debug);

/**
 * @brief      Fetch the latest body rotation in fixed frame
 *
 *             This functions is not current used, just here in case. The out
 *             matrix will be initialized and memory allocated for if necessary.
 *
 * @param      out   result
 *
 * @return     0 on success, -1 on failure
 */
int geometry_get_R_body_to_fixed(rc_matrix_t* out);

/**
 * @brief      Fetch the latest body position in fixed frame
 *
 *             This functions is not current used, just here in case. The out
 *             vector will be initialized and memory allocated for if necessary.
 *
 * @param      out   result
 *
 * @return     0 on success, -1 on failure
 */
int geometry_get_T_body_wrt_fixed(rc_vector_t* out);

/**
 * @brief      Fetch the latest body rotation in local frame
 *
 *  This is the rotation PX4 wants for Visual Odometry
 *
 * @param      out   result
 *
 * @return     0 on success, -1 on failure
 */
int geometry_get_R_body_to_local(rc_matrix_t* out);

/**
 * @brief      get the drone's body position with respect to the local frame
 *
 * This is the position PX4 wants for Visual odometry
 *
 * @param      out   result
 *
 * @return     0 on success, -1 on failure
 */
int geometry_get_T_body_wrt_local(rc_vector_t* out);

/**
 * @brief      fetch current tait bryan angles of body in local frame
 *
 * @param[out] roll   The roll
 * @param[out] pitch  The pitch
 * @param[out] yaw    The yaw
 *
 * @return     0 on success, -1 on failure
 */
int geometry_get_tait_bryan_body_wrt_local(double* roll, double* pitch, double* yaw);

/**
 * @brief      transform a vector from fixed frame to local frame
 *
 *             used to translate position setpoints from fixed to local frame
 *             before sending to px4
 *
 * @param[in]  p_fixed  input vector (position) in fixed frame
 * @param      p_local  output position in local frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_transform_vec_from_fixed_to_local_frame(rc_vector_t p_fixed, rc_vector_t* p_local);

/**
 * @brief      rotate a vector from fixed to local frame without translation
 *
 * used to translate velocity setpoints from fixed to local frame
 *
 * @param[in]  v_fixed  a vector (e.g. velocity) in fixed frame
 * @param[out] v_local  a vector (e.g. velocity) in local frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_rotate_vec_from_fixed_to_local_frame(rc_vector_t v_fixed, rc_vector_t* v_local);


/**
 * @brief      rotate a vector from local to fixed frame without translation
 *
 * Used to translate a vector such as current velocity from local to fixed frame
 *
 * @param[in]  v_local  a vector (e.g. velocity) in local frame
 * @param[out] v_fixed  a vector (e.g. velocity) in fixed frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_rotate_vec_from_local_to_fixed_frame(rc_vector_t v_local, rc_vector_t* v_fixed);


/**
 * @brief      calculate velocity in body fram given velocity of imu in vio
 *             frame and angular rate around imu
 *
 *             Used to calculate velocity of the center of mass of the drone
 *             body before sending to PX4
 *
 * @param[in]  v_vio           velcotiy of IMU in VIO frame
 * @param[in]  w_body_wrt_imu  angular rate of body about imu, rad/s
 * @param      v_body          resulting velocity in body frame for PX4
 *
 * @return     0 on success, -1 on failure
 */
int geometry_calc_velocity_in_body_frame(rc_vector_t v_vio, rc_vector_t w_body_wrt_imu, rc_vector_t* v_body);


// Used to rotate vio velocity output in vio frame to body frame
int geometry_calc_velocity_in_local_frame(rc_vector_t v_imu_wrt_vio, \
						rc_vector_t w_imu_wrt_imu, rc_vector_t* v_body_wrt_local);
/**
 * @brief      Rotate a vector from vio to body frame
 *
 *
 * @param[in]  v_vio   original vector in VIO frame
 * @param      v_body  New vector in body frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_rotate_vec_from_vio_to_body_frame(rc_vector_t v_vio, rc_vector_t* v_body);

/**
 * @brief      rotate a vector from imu frame to body frame
 *
 *             Used to rotate gyro (angular rate) values from imu frame to body
 *             frame. The output vector v_body will be initialized and memory
 *             allocated for if necessary.
 *
 * @param[in]  v_imu   original vector in imu frame
 * @param      v_body  output vector in body frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_rotate_vec_from_imu_to_body_frame(rc_vector_t v_imu, rc_vector_t* v_body);

/**
 * @brief      Rotate a vector from VIO to local frame
 *
 *             Used to rotate covariance values from vio to local frame. The
 *             output vector v_fixed will be initialized and memory allocated
 *             for if necessary.
 *
 * @param[in]  v_vio    original vector in VIO frame
 * @param      v_local  output vector in fixed frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_rotate_vec_from_vio_to_local_frame(rc_vector_t v_vio, rc_vector_t* v_local);

/**
 * @brief      pass the geometry module an updated estimate of the gravity
 *             vector in VIO frame.
 *
 *             The geometry module will use this to update the rotation from VIO
 *             frame to gravity-corrected VIO frame. This will also serve to
 *             make sure VIO was not intialized upside down or too far off from
 *             level.
 *
 * @param      grav  gravity vector in VIO frame
 *
 * @return     returns 0 on success -1 if data is not ready for gravity
 *             correction -2 if orientation is so far off VIO should be reset
 */
int geometry_update_gravity_offset(float grav[3]);

/**
 * @brief      input new VIO odometry data
 *
 *             Call this when new VIO data is received, it copies new data into
 *             geometry module and also stores this data in a ring buffer for
 *             lookups. This is the primary VIO odometry data relating the
 *             center of the IMU to VIO's spacial frame centered whever VIO
 *             initialized.
 *
 * @param[in]  timestamp_ns   timestamp in nanoseconds
 * @param[in]  new_R_imu_to_vio   3x3 Rotation matrix from IMU to VIO spacial frame
 * @param[in]  new_T_imu_wrt_vio  position of IMU with respect to VIO spacial frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_update_imu_to_vio(int64_t timestamp_ns, float T[3], float R[3][3]);

/**
 * @brief      input new spacial relation between camera and imu
 *
 *             call this when new VIO data is received it copies new data into
 *             local record of frames. This could be set just once but VIO
 *             should continuously be optimizing the relation between camera and
 *             IMU so you can keep updating this is each new IMU frame to make
 *             apriltag detection more accurate if desired.
 *
 * @param[in]  new_R_cam_to_imu   3x3 rotation matrix from camera to imu
 * @param[in]  new_T_cam_wrt_imu  xyz position of camera with respect to imu
 *
 * @return     0 on success, -1 on failure
 */
int geometry_update_cam_to_imu(float T[3], float R[3][3]);

/*
 * @brief      This updates the relation between local and fixed frame.
 *
 *             call this when an apriltag is detected. tag_index is NOT the id
 *             of the tag detected, it's the index in the fixed_apriltag[] array
 *             from config.h corresponding to the detected tag. The index is
 *             used by this function to look up the expected position and
 *             rotation
 *
 * @param[in]  frame_timestamp_ns    frame timestamp of apriltag image in
 *                                   nanoseconds
 * @param[in]  new_R_tag_to_camera   Rotation of tag to camera frame
 * @param[in]  new_T_tag_wrt_camera  Translation of tag with respect to camera
 * @param[in]  R_tag_to_fixed        known rotation of tag to fixed frame
 * @param[in]  T_tag_wrt_fixed       location of tag in fixed frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_add_fixed_tag_detection(int64_t frame_timestamp_ns,
            rc_matrix_t R_tag_to_cam, rc_vector_t T_tag_wrt_cam,
            rc_matrix_t R_tag_to_fixed, rc_vector_t T_tag_wrt_fixed);


/**
 * @brief      find position of detected tag in local frame given detection in
 *             camera frame
 *
 *             Used for positioning the drone relative to apriltag. This
 *             interpolates between previous VIO data points based on provided
 *             timestamp. WIll return -1 if not enough previous vio data is
 *             available.
 *
 * @param[in]  frame_timestamp_ns  The frame timestamp in ns
 * @param[in]  R_tag_to_cam        R from detected tag to camera frame
 * @param[in]  T_tag_wrt_cam       location of detected tag in camera frame
 * @param[out] R_tag_to_local      Resulting rotation of tag to local
 * @param[out] T_tag_wrt_local     location of tag with respect to local frame
 *
 * @return     0 on success, -1 on failure
 */
int geometry_calc_R_T_tag_in_local_frame(int64_t frame_timestamp_ns,
                    rc_matrix_t R_tag_to_cam, rc_vector_t T_tag_wrt_cam,
                    rc_matrix_t* R_tag_to_local, rc_vector_t* T_tag_wrt_local);

/**
 * @brief      gets the current reset counter
 *
 *             The reset counter keeps track of how many times VIO has "jumped"
 *             due to vio reset, loop closure, or apriltag detection. The
 *             geometry module will bump this automatically when the fixed frame
 *             shifts relative to local frame significantly. It's not used for
 *             anything yet, just a placeholder really.
 *
 * @return     current reset counter value
 */
int geometry_get_reset_counter(void);

/**
 * @brief      Increase the reset counter by 1
 *
 *             Use this is manually bump the reset counter. For example on VIO
 *             reset.
 *
 * @return     returns the new updated reset counter
 */
int geometry_bump_reset_counter(void);

/**
 * @brief      convert a mavlink position setpoint from fixed to local frame
 *
 *             use this is you have a desired location in apriltag-corected
 *             fixed frame that you want to fly to. PX4 flies in local frame, so
 *             setpoints need to be translated to local frame first.
 *
 * @param[in]  in    input
 * @param      out   output
 *
 * @return     0 on success, -1 on failure
 */
int geometry_transform_fixed_setpoint_to_local( mavlink_set_position_target_local_ned_t in,\
                                                mavlink_set_position_target_local_ned_t* out);

/**
 * @brief      invert a rotation and translation from A-to-B to B-to-A
 *
 *             The rotation is simple, just transpose the matrix. The
 *             translation vector T_A_wrt_B is left-multiplied by -1 * R_B_to_A
 *
 * @param[in]  R_A_to_B   input Rotation from A to B
 * @param[in]  T_A_wrt_B  input translation of A with respect to B
 * @param[out] R_B_to_A   output rotation from B to A
 * @param[out] T_B_wrt_A  output translation of B with respect to A
 *
 * @return     0 on success, -1 on failure
 */
int geometry_invert_tf( rc_matrix_t  R_A_to_B, rc_vector_t  T_A_wrt_B,\
                        rc_matrix_t* R_B_to_A, rc_vector_t* T_B_wrt_A);


/**
 * @brief      fetch the most recent transform from stereo camera frame to the
 *             current body frame assuming the body has been leveled out.
 *
 *             This is used to transform obstacle points to the body frame while
 *             compensaing for time delay and the roll/pitch of the airframe.
 *
 * @param[in]  frame_timestamp_ns  The frame timestamp in ns (apps proc monotonic time)
 * @param      R_stereo_to_level   Rotation from stereo to level
 *
 * @return     0 on success, -1 on failure
 */
/*
int geometry_get_RT_stereo_to_level(int64_t frame_timestamp_ns,
										rc_matrix_t* R_stereo_to_level,
										rc_vector_t* T_stereo_wrt_level);
*/

int geometry_get_RT_frame_in_past_to_level(char* frame, int64_t past_ts_ns,\
							int64_t new_ts_ns, rc_matrix_t* R, rc_vector_t* T);


/**
 * @brief      add an estimate of the drone's body frame with respect to fixed frame
 *
 * yaw must be in +-PI
 *
 * @param[in]  pose_body_wrt_fixed  timestamped pose
 *
 * @return     0 on success, -1 on failure
 */
int geometry_add_fixed_frame_estimate(pose_4dof_t pose_body_wrt_fixed);


/**
 * @brief      add an estimate of the drone's body frame with respect to fixed frame
 *
 * velocity components are ignored
 *
 * @param[in]  pose_body_wrt_fixed  timestamped pose
 *
 * @return     0 on success, -1 on failure
 */
int geometry_add_fixed_frame_estimate_6dof(pose_vel_6dof_t pose_body_wrt_fixed);

/**
 * @brief      load in translations and rotations from extrinsic config file
 *             relating to the imu location.
 *
 *             This is called when connecting to a vio server which indicates
 *             which imu it is using for VIO. Since the vio data describes the
 *             position of the imu, we need to load in the correct extrinsics
 *             value relating the imu to body.
 *
 * @param[in]  imu_string  The imu string typically imu1 or imu0
 *
 * @return     0 on success, -1 on failure
 */
int geometry_set_imu(const char* imu_string);

int geometry_get_RT_fixed_to_level_at_time(int64_t ts_ns, rc_matrix_t *R, rc_vector_t *T);

// just for debugging
int geometry_get_T_body_in_past(int64_t past_ts_ns, float* xyz);


// put data into the px4 attitude ringbuffer
int geometry_add_px4_attitude(int64_t ts_ns, mavlink_attitude_t new_attitude);

// used for leveling out obstacle avoidance point clouds with PX4 attitude data when
// vio is not available.
int geometry_get_RT_frame_in_past_to_level_from_px4_attitude(char* frame, \
				int64_t past_ts_ns, int64_t new_ts_ns, rc_matrix_t* R, rc_vector_t* T);

#endif // GEOMETRY_H
