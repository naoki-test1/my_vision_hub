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
#include <rc_math.h>
#include <modal_pipe_client.h>
#include <modal_pipe_server.h>

#include "macros.h"
#include "vio_manager.h"
#include "pipe_channels.h"
#include "geometry.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "autopilot_monitor.h"
#include "config_file.h"
#include "misc.h"
#include "horizon_cal.h"
#include "state_manager.h"


// debug modes set by setter functions
static int print_debug_local;
static int print_debug_fixed;


// consider VIO offline after 150ms
#define VIO_TIMEOUT_NS 150000000
static int64_t last_valid_vio_ts_ns = 0;


void vio_manager_en_print_debug_local(int en_print)
{
	print_debug_local = en_print;
	return;
}

void vio_manager_en_print_debug_fixed(int en_print)
{
	print_debug_fixed = en_print;
	return;
}

static int _rotate_covariance(float* vio_pose_cov, float* vio_vel_cov, float* mav_pose_cov, float* mav_vel_cov)
{
	memset(mav_pose_cov, 0, sizeof(float)*21);
	memset(mav_vel_cov,  0, sizeof(float)*21);

	// Odometry message contains the upper right triangle of full 6x6 pose
	// covariance. EKF2 just takes the max value in the diagonal for attitude
	// and position so we only copy and transform the diagonal elements here.
	static rc_vector_t rotation_cov_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t position_cov_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t velocity_cov_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t rotation_cov_wrt_local = RC_VECTOR_INITIALIZER;
	static rc_vector_t position_cov_wrt_local = RC_VECTOR_INITIALIZER;
	static rc_vector_t velocity_cov_wrt_local = RC_VECTOR_INITIALIZER;

	/*
	from px4's vehicle_odometry.msg
	uint8 COVARIANCE_MATRIX_X_VARIANCE=0
	uint8 COVARIANCE_MATRIX_Y_VARIANCE=6
	uint8 COVARIANCE_MATRIX_Z_VARIANCE=11
	uint8 COVARIANCE_MATRIX_ROLL_VARIANCE=15
	uint8 COVARIANCE_MATRIX_PITCH_VARIANCE=18
	uint8 COVARIANCE_MATRIX_YAW_VARIANCE=20
	uint8 COVARIANCE_MATRIX_VX_VARIANCE=0
	uint8 COVARIANCE_MATRIX_VY_VARIANCE=6
	uint8 COVARIANCE_MATRIX_VZ_VARIANCE=11
	uint8 COVARIANCE_MATRIX_ROLLRATE_VARIANCE=15
	uint8 COVARIANCE_MATRIX_PITCHRATE_VARIANCE=18
	uint8 COVARIANCE_MATRIX_YAWRATE_VARIANCE=20
	*/

	// vectors to transform from vio to local frame
	rc_vector_alloc(&rotation_cov_wrt_vio,3);
	rc_vector_alloc(&position_cov_wrt_vio,3);
	rc_vector_alloc(&velocity_cov_wrt_vio,3);

	// pull out diagonal entries for rotation
	position_cov_wrt_vio.d[0] = vio_pose_cov[0];
	position_cov_wrt_vio.d[1] = vio_pose_cov[6];
	position_cov_wrt_vio.d[2] = vio_pose_cov[11];
	rotation_cov_wrt_vio.d[0] = vio_pose_cov[15];
	rotation_cov_wrt_vio.d[1] = vio_pose_cov[18];
	rotation_cov_wrt_vio.d[2] = vio_pose_cov[20];
	velocity_cov_wrt_vio.d[0] = vio_vel_cov[0];
	velocity_cov_wrt_vio.d[1] = vio_vel_cov[6];
	velocity_cov_wrt_vio.d[2] = vio_vel_cov[11];

	// rotate
	geometry_rotate_vec_from_vio_to_local_frame(rotation_cov_wrt_vio, &rotation_cov_wrt_local);
	geometry_rotate_vec_from_vio_to_local_frame(position_cov_wrt_vio, &position_cov_wrt_local);
	geometry_rotate_vec_from_vio_to_local_frame(velocity_cov_wrt_vio, &velocity_cov_wrt_local);


	// position
	mav_pose_cov[0]=fabsf(position_cov_wrt_local.d[0]);
	mav_pose_cov[6]=fabsf(position_cov_wrt_local.d[1]);
	mav_pose_cov[11]=fabsf(position_cov_wrt_local.d[2]);
	// angle
	mav_pose_cov[15]=fabsf(rotation_cov_wrt_local.d[0]);
	mav_pose_cov[18]=fabsf(rotation_cov_wrt_local.d[1]);
	mav_pose_cov[20]=fabsf(rotation_cov_wrt_local.d[2]);
	// velocity
	mav_vel_cov[0]=fabsf(velocity_cov_wrt_local.d[0]);
	mav_vel_cov[6]=fabsf(velocity_cov_wrt_local.d[1]);
	mav_vel_cov[11]=fabsf(velocity_cov_wrt_local.d[2]);
	// QVIO doesn't give us covariance for angular rates so duplicate
	// the cov for angle for completeness. ekf2 doesn't currently use it
	mav_vel_cov[15]=fabsf(rotation_cov_wrt_local.d[0]);
	mav_vel_cov[18]=fabsf(rotation_cov_wrt_local.d[1]);
	mav_vel_cov[20]=fabsf(rotation_cov_wrt_local.d[2]);

	return 0;
}

static void _send_failed_odom_msg(void)
{
	if(!en_vio) return;

	mavlink_message_t msg;

	uint64_t usec = my_time_monotonic_ns()/1000;
	uint8_t frame_id = MAV_FRAME_LOCAL_FRD;
	uint8_t child_frame_id = MAV_FRAME_BODY_FRD;

	float T_body_wrt_local[3]={0,0,0};
	float q_for_px4[4] = {1,0,0,0};
	float v_body_wrt_body[3]={0,0,0};
	float w_body_wrt_body[3]={0,0,0};

	float mav_pose_cov[21];
	float mav_vel_cov[21];
	for(int i=0;i<21;i++){
		mav_pose_cov[i] = NAN;
		mav_vel_cov[i] = NAN;
	}
	// indicate a failed state
	int quality = -1;

	mavlink_msg_odometry_pack(  autopilot_monitor_get_sysid(), \
								VOXL_COMPID, \
								&msg, \
								usec, \
								frame_id, \
								child_frame_id, \
								T_body_wrt_local[0], \
								T_body_wrt_local[1], \
								T_body_wrt_local[2], \
								q_for_px4, \
								v_body_wrt_body[0], \
								v_body_wrt_body[1], \
								v_body_wrt_body[2], \
								w_body_wrt_body[0], \
								w_body_wrt_body[1], \
								w_body_wrt_body[2], \
								mav_pose_cov, \
								mav_vel_cov, \
								geometry_get_reset_counter(), \
								MAV_ESTIMATOR_TYPE_VIO, \
								quality);

	// send to autopilot
	mavlink_io_send_msg_to_ap(&msg);

	if(print_debug_local){
		printf("sent odometry with quality = -1 to indicate VIO failure\n");
	}
	return;
}


// called when new valid data is received through the VIO pipe helper
// the purpose of this function is to clean up raw vio outputs and make it suitable
// for use by VFC, PX4, and Ardupilot. This in primarily invlolves rotating and
// shifting the data from respresinting the IMU location in VIO frame to the
// body location in local frame. e.g. it corrects for the position and orientation
// of the IMU relative to the drone's center of mass.
static void process_new_vio_data(vio_data_t d)
{
	int i,j;
	mavlink_message_t msg;
	static uint8_t last_vio_state = VIO_STATE_FAILED;
	static int64_t last_timestamp = 0; // keep track of timestamp to catch duplicates
	static int64_t first_good_timestamp_after_reset = 0;
	static int has_alerted_vio_has_started = 0;
	static int has_alerted_vio_has_failed = 0;

	// create an aligned vio struct along the way for publishing and use by VSE
	// Start it in a failed state since the first few logic statements are going
	// to quit early and publish when vio is unhappy for a multitude of reasons.
	vio_data_t d_aligned;
	memset(&d_aligned, 0, sizeof(vio_data_t));
	d_aligned.magic_number = VIO_MAGIC_NUMBER;
	d_aligned.quality = -1;
	d_aligned.state = VIO_STATE_FAILED;
	d_aligned.timestamp_ns = d.timestamp_ns;
	d_aligned.error_code = d.error_code;


	// warn if VIO failed and print the error
	// also send a failed odom msg so EKF2 is aware
	if(d.state == VIO_STATE_FAILED && last_vio_state != VIO_STATE_FAILED){
		last_vio_state = d.state;
		has_alerted_vio_has_started = 0;
		if(!has_alerted_vio_has_failed){
			has_alerted_vio_has_failed = 1;
			if(print_debug_local){
				fprintf(stderr, "Vision Failure\n");
			}
			mavlink_io_send_text_to_gcs("Vision Failure");
		}
		if(send_odom_while_failed) _send_failed_odom_msg();
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}

	// if first ever packet, record this time
	// This is important if VVPX4 starts after vio has initialized
	if(d.state == VIO_STATE_OK && first_good_timestamp_after_reset == 0){
		first_good_timestamp_after_reset = d.timestamp_ns;
	}

	// if we just went from a failed or initializing state to OK
	// then record this time
	if(d.state == VIO_STATE_OK && last_vio_state != VIO_STATE_OK){
		first_good_timestamp_after_reset = d.timestamp_ns;
	}

	// keep track of last state to detection transitions
	last_vio_state = d.state;

	// just return if vio is still initializing or still in failed state
	if(d.state != VIO_STATE_OK){
		if(send_odom_while_failed) _send_failed_odom_msg();
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}

	// discard duplicate timestamps which can occur when vio is starved of imu
	if(d.timestamp_ns==last_timestamp){
		if(send_odom_while_failed) _send_failed_odom_msg();
		return;
	}
	else{
		last_timestamp = d.timestamp_ns;
	}

	// also discard data during a 1 second warmup period after VIO reports
	// its okay. This helps prevent failed starts from triggering EKF2 to start
	// using VIO when VIO is just going to fail again a few samples later.
	// It also serves to slow down VIO resets when it's being reset by the 
	// upside-down gravity vector check.
	float t_since_start = (float)(d.timestamp_ns - first_good_timestamp_after_reset)/1000000000.0f;
	if(t_since_start < vio_warmup_s){
		if(send_odom_while_failed) _send_failed_odom_msg();
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}

	// check for time mismatch
	int64_t current_time_ns = my_time_monotonic_ns();

	// invalid if more than 0.1s in the future
	if(	d.timestamp_ns > current_time_ns+100000000){
		fprintf(stderr, "WARNING: VIO time %7.1fs in the future, dropping packet\n", (double)(d.timestamp_ns-current_time_ns)/1000000000.0);
		mavlink_io_send_text_to_gcs("Vision Data in the Future");
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}

	// invalid if more than 0.3s old
	if(	d.timestamp_ns < current_time_ns-300000000){
		fprintf(stderr, "WARNING: VIO time %7.1fs too old, dropping packet\n", (double)(current_time_ns - d.timestamp_ns)/1000000000.0);
		mavlink_io_send_text_to_gcs("Vision Data Too Old");
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}

	// gravity update function also serves to see if vio was initialized
	// upside down or really off-level.
	int ret = geometry_update_gravity_offset(d.gravity_vector);

	// if not successful, reset and try again
	if(ret==-2){
		if(en_reset_vio_if_initialized_inverted){
			fprintf(stderr, "Sending hard reset command to VIO\n");
			pipe_client_send_control_cmd(VIO_PIPE_CH, RESET_VIO_HARD);
			mavlink_io_send_text_to_gcs("Vision Upside Down");
			has_alerted_vio_has_failed = 1;
		}
		else{
			if(print_debug_local){
				fprintf(stderr, "You must fix VIO's gravity orientation\n");
			}
			mavlink_io_send_text_to_gcs("Vision Initialized Upside Down");
		}
	}
	if(ret==-3){
		if(en_reset_vio_if_initialized_inverted){
			fprintf(stderr, "Sending hard reset command to VIO\n");
			pipe_client_send_control_cmd(VIO_PIPE_CH, RESET_VIO_HARD);
			mavlink_io_send_text_to_gcs("Vision Not Level Enough");
			has_alerted_vio_has_failed = 1;
		}
		else{
			if(print_debug_local){
				fprintf(stderr, "You must fix VIO's gravity orientation\n");
			}
			mavlink_io_send_text_to_gcs("Vision Not Level Enough");
		}
	}
	// some other failure, don't continue. This should never happen!
	if(ret<0){
		if(send_odom_while_failed) _send_failed_odom_msg();
		pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));
		return;
	}


	// all checks passed, nothing should fail after this point
	// alert the user the warmup has completed and VIO has started
	if(!has_alerted_vio_has_started){
		has_alerted_vio_has_started = 1;
		has_alerted_vio_has_failed = 0;
		geometry_bump_reset_counter();
		if(print_debug_local){
			fprintf(stderr, "Vision Started\n");
			printf("bumped reset counter to %d\n", geometry_get_reset_counter());
		}
		mavlink_io_send_text_to_gcs("Vision Started");
	}


	// copy out position & rotation, camera and imu relation, send into geometry module
	geometry_update_imu_to_vio(d.timestamp_ns, d.T_imu_wrt_vio, d.R_imu_to_vio);
	geometry_update_cam_to_imu(d.T_cam_wrt_imu, d.R_cam_to_imu);

	// Fetch converted rotation and position of the body in local frame
	static rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
	static rc_vector_t T_body_wrt_local = RC_VECTOR_INITIALIZER;
	geometry_get_R_body_to_local(&R_body_to_local);
	geometry_get_T_body_wrt_local(&T_body_wrt_local);
	double roll, pitch, yaw;
	rc_rotation_to_tait_bryan(R_body_to_local, &roll, &pitch, &yaw);

	// angular rates copy and transform
	// this isn't used by ekf2 but send anyway for completeness
	static rc_vector_t w_imu_wrt_imu = RC_VECTOR_INITIALIZER;
	static rc_vector_t w_body_wrt_body = RC_VECTOR_INITIALIZER;
	float_to_vector(d.imu_angular_vel, &w_imu_wrt_imu);
	geometry_rotate_vec_from_imu_to_body_frame(w_imu_wrt_imu, &w_body_wrt_body);

	// velocity copy and transform
	// mavlink odometry message expects velocity in body frame even though
	// position is in local frame so after converting from vio frame to local
	// frame we need to further rotate into body frame. This will be undone in
	// px4 and rotated back to local ned frame before going into ekf2
	static rc_vector_t v_imu_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t v_body_wrt_body = RC_VECTOR_INITIALIZER;
	float_to_vector(d.vel_imu_wrt_vio, &v_imu_wrt_vio);
	geometry_calc_velocity_in_body_frame(v_imu_wrt_vio, w_imu_wrt_imu, &v_body_wrt_body);

	// rotate covariance into correct frame
	float mav_pose_cov[21];
	float mav_vel_cov[21];
	_rotate_covariance(d.pose_covariance, d.velocity_covariance, mav_pose_cov, mav_vel_cov);

	// if we got here, all was valid, record this time
	last_valid_vio_ts_ns = d.timestamp_ns;


	// if we enabled sending vio data to px4, do so
	if(en_vio){
		// VIO rotation is sent in as body to local frame
		// VIO position is sent as body position with respect to local frame
		// VIO velocity is sent in aligned with body frame
		// vio angular rates are sent in aligmed with body frame
		uint8_t frame_id = MAV_FRAME_LOCAL_FRD;
		uint8_t child_frame_id = MAV_FRAME_BODY_FRD;

		// convert to quaternion for px4. px4 wants q from local to body
		// so invert the quaternion
		float q_for_px4[4];
		RC_VECTOR_ON_STACK(q_body_to_local,4);
		rc_rotation_to_quaternion(R_body_to_local, &q_body_to_local);
		for(i=0;i<4;i++) q_for_px4[i] = q_body_to_local.d[i];
		for(i=1;i<4;i++) q_for_px4[i] *= -1.0f; // invert q

		// Send timestamp as-is. This timestamp comes from clock-monotonic
		// PX4 will do its own timesyncing since we respond to its timesync requests.
		uint64_t usec = d.timestamp_ns/1000;

		// send data if PX4 is online and we have registered its sysid
		mavlink_msg_odometry_pack(  autopilot_monitor_get_sysid(), \
									VOXL_COMPID, \
									&msg, \
									usec, \
									frame_id, \
									child_frame_id, \
									T_body_wrt_local.d[0], \
									T_body_wrt_local.d[1], \
									T_body_wrt_local.d[2], \
									q_for_px4, \
									v_body_wrt_body.d[0], \
									v_body_wrt_body.d[1], \
									v_body_wrt_body.d[2], \
									w_body_wrt_body.d[0], \
									w_body_wrt_body.d[1], \
									w_body_wrt_body.d[2], \
									mav_pose_cov, \
									mav_vel_cov, \
									geometry_get_reset_counter(), \
									MAV_ESTIMATOR_TYPE_VIO, \
									d.quality);
		// send to Autopilot
		mavlink_io_send_msg_to_ap(&msg);
	}

	// now populate our aligned vio packet with the same data as we sent to EKF2
	// including pulling in the state and quality
	d_aligned = d;
	vector_to_float(T_body_wrt_local, d_aligned.T_imu_wrt_vio);
	for(i=0;i<3;i++){
		for(j=0;j<3;j++) d_aligned.R_imu_to_vio[i][j] = R_body_to_local.d[i][j];
	}
	memcpy(d.pose_covariance, mav_pose_cov, 21*sizeof(float));
	static rc_vector_t v_body_wrt_local = RC_VECTOR_INITIALIZER;
	geometry_calc_velocity_in_local_frame(v_imu_wrt_vio, w_imu_wrt_imu, &v_body_wrt_local);
	vector_to_float(v_body_wrt_local, d_aligned.vel_imu_wrt_vio);
	memcpy(d.velocity_covariance, mav_vel_cov, 21*sizeof(float));
	vector_to_float(w_body_wrt_body, d_aligned.imu_angular_vel);
	d_aligned.gravity_vector[0] = 0.0f;
	d_aligned.gravity_vector[1] = 0.0f;
	d_aligned.gravity_vector[2] = 1.0f;

	// set this new date locally and publish
	state_manager_add_new_data(d_aligned, 2);
	pipe_server_write(ALIGNED_VIO_CH, (char*)&d_aligned, sizeof(vio_data_t));

	// if anyone is subscribed to the local pose pipe, write latest data to it
	if(pipe_server_get_num_clients(LOCAL_POSE_OUT_CH)>0){
		pose_vel_6dof_t p;
		p.magic_number = POSE_VEL_6DOF_MAGIC_NUMBER;
		p.timestamp_ns = d.timestamp_ns;
		vector_to_float(T_body_wrt_local, p.T_child_wrt_parent);
		matrix_to_float(R_body_to_local, p.R_child_to_parent);
		vector_to_float(v_body_wrt_local, p.v_child_wrt_parent);
		vector_to_float(w_body_wrt_body, p.w_child_wrt_child);
		pipe_server_write(LOCAL_POSE_OUT_CH, (char*)&p, sizeof(p));
	}

	// if anyone is subscribed to the fixed pose pipe, write latest data to it
	if(pipe_server_get_num_clients(FIXED_POSE_OUT_CH)>0){
		pose_vel_6dof_t p;
		p.magic_number = POSE_VEL_6DOF_MAGIC_NUMBER;
		p.timestamp_ns = d.timestamp_ns;
		static rc_vector_t T_body_wrt_fixed = RC_VECTOR_INITIALIZER;
		static rc_matrix_t R_body_to_fixed = RC_MATRIX_INITIALIZER;
		geometry_get_T_body_wrt_fixed(&T_body_wrt_fixed);
		geometry_get_R_body_to_fixed(&R_body_to_fixed);
		vector_to_float(T_body_wrt_fixed, p.T_child_wrt_parent);
		matrix_to_float(R_body_to_fixed, p.R_child_to_parent);
		RC_VECTOR_ON_STACK(v_body_wrt_fixed,3);
		geometry_rotate_vec_from_local_to_fixed_frame(v_body_wrt_local, &v_body_wrt_fixed);
		vector_to_float(v_body_wrt_fixed, p.v_child_wrt_parent);
		vector_to_float(w_body_wrt_body, p.w_child_wrt_child);
		pipe_server_write(FIXED_POSE_OUT_CH, (char*)&p, sizeof(p));
	}


	// send to horizon cal module
	horizon_cal_add_vio_roll_pitch(roll*RAD_TO_DEG, pitch*RAD_TO_DEG);

	// print local odometry and body velocity as it was sent to px4
	if(print_debug_local)
	{
		printf("T_body_wrt_local: %5.2f %5.2f %5.2f  RPY: %5.2f %5.2f %5.2f  velocity: %5.2f %5.2f %5.2f\n", \
			T_body_wrt_local.d[0], T_body_wrt_local.d[1], T_body_wrt_local.d[2],\
			roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI,\
			v_body_wrt_body.d[0],  v_body_wrt_body.d[1],  v_body_wrt_body.d[2]);
	}

	// print odometry with respect to fixed frame. This is NOT sent to px4,
	// this is just a debug tool.
	if(print_debug_fixed){
		double roll, pitch, yaw;
		static rc_matrix_t R_body_to_fixed = RC_MATRIX_INITIALIZER;
		static rc_vector_t T_body_wrt_fixed = RC_VECTOR_INITIALIZER;
		geometry_get_R_body_to_fixed(&R_body_to_fixed);
		geometry_get_T_body_wrt_fixed(&T_body_wrt_fixed);
		rc_rotation_to_tait_bryan(R_body_to_fixed, &roll, &pitch, &yaw);
		printf("T_body_wrt_fixed: %5.2f %5.2f %5.2f  RPY: %5.2f %5.2f %5.2f\n", \
			T_body_wrt_fixed.d[0], T_body_wrt_fixed.d[1], T_body_wrt_fixed.d[2],\
			roll, pitch, yaw);
	}

	return;
}


int vio_manager_is_active(void)
{
	return (last_valid_vio_ts_ns > (my_time_monotonic_ns()-VIO_TIMEOUT_NS));
}


// This is the callback assigned to the client pipe interface. It checks for
// server disconnects and validates the received vio data before processing it.
// Multiple packets may come through at once although that's not typical.
static void vio_pipe_helper_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// validate that the data makes sense
	int n_packets, i;
	vio_data_t* data_array = pipe_validate_vio_data_t(data, bytes, &n_packets);
	if(data_array == NULL){
		pipe_client_flush(ch);
		return;
	}
	for(i=0;i<n_packets;i++) process_new_vio_data(data_array[i]);

	// if(n_packets>2){
	// 	mavlink_io_send_text_to_gcs("Vision Data Backed Up");
	// }
	return;
}


// We don't know which imu the VIO algorithm is using until we connect to it
// This is called to fetch the location of the imu to the center of mass
// from the extrinsics file when we connect to a new VIO server.
static void _update_imu(int ch)
{
	cJSON* json = pipe_client_get_info_json(ch);
	if(json == NULL){
		fprintf(stderr, "WARNING: failed to get info json from qvio server\n");
		return;
	}

	// fetch the imu pipe name from the qvio server, returns something like "/run/mpa/imu0/"
	char buf[64];
	memset(buf,0,64);
	int ret = json_fetch_string(json, "imu", buf, 64);
	cJSON_Delete(json);

	if(ret<0){
		// config.c already set this up for imu1 so just leave it as is
		fprintf(stderr, "WARNING: json info from vio server doesn't contain imu field, using imu1\n");
		return;
	}

	// trim the string down to just the end.
	int len = strlen(buf);
	if(len<1){
		fprintf(stderr, "ERROR invalid imu name in vio pipe info\n");
		return;
	}

	// trim the trailing '/' if it exists
	if(buf[len-1]=='/'){
		buf[len-1]=0;
		len--;
	}

	// find the new start
	char* start = buf;
	for(int i=len; i>0; i--){
		if(buf[i-1]=='/'){
			start = buf + i;
			break;
		}
	}

	printf("Geometry module updating to use imu: %s for VIO\n", start);
	geometry_set_imu(start);
	return;
}


// called when connecting to the VIO pipe
static void _connect_cb(int ch, __attribute__((unused)) void* context)
{
	char* str = vio_pipe;
	if(ch==SECONDARY_VIO_PIPE_CH) str=secondary_vio_pipe;

	printf("Connected to VIO pipe: %s\n", str);

	// close the other channel to prevent conflicts.
	// could potentially support hot swapping between sources but that sounds
	// like it could go very wrong very quickly. Just use one source for now.
	pipe_client_close((ch==VIO_PIPE_CH) ? SECONDARY_VIO_PIPE_CH:VIO_PIPE_CH);

	// update IMU source from the new pipe
	_update_imu(ch);

	// NOTE: simple helper callback won't be called until this callback returns
	return;
}


// called when disconecting from the VIO pipe, usually due to
// VIO server crashing or resetting
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf("Disconnected from VIO pipe: %s\n", vio_pipe);

	// Run the packet handler with a vio packet indicating an error
	// This will trigger the same behaviors as if VIO reported an
	// Error itself such as warning the user and resetting timers
	vio_data_t d;
	d.magic_number = VIO_MAGIC_NUMBER;
	d.timestamp_ns = my_time_monotonic_ns();
	d.error_code = ERROR_CODE_UNKNOWN;
	d.state = VIO_STATE_FAILED;
	d.quality = -1.0f;
	process_new_vio_data(d);

	return;
}

// separate function so state manager can call it
// two modules may call this so ensure it only creates the pipe once
// libmodal_pipe won't break but it will warn you if that happens
void vio_manager_create_aligned_pipe(void)
{
	// TODO checking if a channel is open should really be a libmodal_pipe function
	static int is_opened = 0;
	if(is_opened) return;

	pipe_info_t info3 = { \
		.name        = ALIGNED_VIO_NAME,\
		.location    = ALIGNED_VIO_LOCATION,\
		.type        = "vio_data_t",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = VIO_RECOMMENDED_PIPE_SIZE};

	int flags3 = 0;
	if(pipe_server_create(ALIGNED_VIO_CH, info3, flags3)) return;
	is_opened = 1;
	return;
}


int vio_manager_init(void)
{
	// set up pipe callbacks
	pipe_client_set_connect_cb(VIO_PIPE_CH, _connect_cb, NULL);
	pipe_client_set_connect_cb(SECONDARY_VIO_PIPE_CH, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(VIO_PIPE_CH, _disconnect_cb, NULL);
	pipe_client_set_disconnect_cb(SECONDARY_VIO_PIPE_CH, _disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(VIO_PIPE_CH, vio_pipe_helper_cb, NULL);
	pipe_client_set_simple_helper_cb(SECONDARY_VIO_PIPE_CH, vio_pipe_helper_cb, NULL);
	pipe_client_set_helper_thread_priority(VIO_PIPE_CH, VIO_THREAD_PRIORITY);
	pipe_client_set_helper_thread_priority(SECONDARY_VIO_PIPE_CH, VIO_THREAD_PRIORITY);

	// always open primary
	pipe_client_open(VIO_PIPE_CH, vio_pipe, PIPE_CLIENT_NAME,
								CLIENT_FLAG_EN_SIMPLE_HELPER,\
								VIO_RECOMMENDED_READ_BUF_SIZE);

	// also open secondary if set
	if(strlen(secondary_vio_pipe)>0){
		pipe_client_open(SECONDARY_VIO_PIPE_CH, secondary_vio_pipe, PIPE_CLIENT_NAME,
								CLIENT_FLAG_EN_SIMPLE_HELPER,\
								VIO_RECOMMENDED_READ_BUF_SIZE);
	}


	// start the server pipes to send local and fixed data out
	pipe_info_t info1 = { \
		.name        = BODY_WRT_LOCAL_POSE_NAME,\
		.location    = BODY_WRT_LOCAL_POSE_LOCATION,\
		.type        = "pose_vel_6dof_t",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = POSE_6DOF_RECOMMENDED_PIPE_SIZE};

	int flags1 = 0;
	if(pipe_server_create(LOCAL_POSE_OUT_CH, info1, flags1)) return -1;

	pipe_info_t info2 = { \
		.name        = BODY_WRT_FIXED_POSE_NAME,\
		.location    = BODY_WRT_FIXED_POSE_LOCATION,\
		.type        = "pose_vel_6dof_t",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = POSE_6DOF_RECOMMENDED_PIPE_SIZE};

	int flags2 = 0;
	if(pipe_server_create(FIXED_POSE_OUT_CH, info2, flags2)) return -1;

	vio_manager_create_aligned_pipe();

	return 0;
}


// stop VIO callback immediately, then wait for manager thread to stop
void vio_manager_stop(void)
{
	// close client pipes
	pipe_client_close(VIO_PIPE_CH);

	// close server pipes
	pipe_server_close(LOCAL_POSE_OUT_CH);
	pipe_server_close(FIXED_POSE_OUT_CH);

	// DON'T CLOSE ALIGNED OUTPUT PIPE YET
	// state manager may still be publishing
	// pipe_server_close(ALIGNED_VIO_CH);
	return;
}

