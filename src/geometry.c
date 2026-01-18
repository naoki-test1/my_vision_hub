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
#include <math.h>
#include <rc_math.h>
#include <pthread.h>
#include <voxl_common_config.h>

#include "geometry.h"
#include "config_file.h"
#include "transform_ringbuf.h"
#include "pose_filter.h"
#include "macros.h"
#include "autopilot_monitor.h"
#include "misc.h"
#include "horizon_cal_file.h"

// leave this off for now, not convinced it helps yet
//#define USE_VIO_HORIZON_CAL

// store lots of past vio data for use in VOA and relocalization
#define VIO_RINGBUF_SIZE 100
#define PX4_ATTITUDE_RINGBUF_SIZE 200

////////////////////////////////////////////////////////////////////////////////
// these are dynamically changing transforms and should always be read/write
// protected by the tf_mutex
////////////////////////////////////////////////////////////////////////////////
static pthread_mutex_t tf_mutex = PTHREAD_MUTEX_INITIALIZER;

static rc_matrix_t R_local_to_fixed = RC_MATRIX_INITIALIZER;
static rc_matrix_t R_fixed_to_local = RC_MATRIX_INITIALIZER;
static double yaw_local_to_fixed = 0.0;
static rc_vector_t T_local_wrt_fixed = RC_VECTOR_INITIALIZER;
static rc_vector_t T_fixed_wrt_local = RC_VECTOR_INITIALIZER;
static rc_pose_filter_t filter_local_to_fixed = RC_POSE_FILTER_INITIALIZER;

// these two are both updated when a new gravity vector is provided
static rc_matrix_t R_vio_to_vio_ga = RC_MATRIX_INITIALIZER;
static rc_matrix_t R_vio_to_local = RC_MATRIX_INITIALIZER;

// these are based on IMU location to center of mass
static rc_matrix_t R_imu_to_body = RC_MATRIX_INITIALIZER;
static rc_vector_t T_imu_wrt_body = RC_VECTOR_INITIALIZER;

// keep forward and back copies of imu_to_vio since body are used
// this is always the latest from vio. For old transforms use vio_ringbuf
static rc_matrix_t R_imu_to_vio = RC_MATRIX_INITIALIZER;
static rc_vector_t T_imu_wrt_vio = RC_VECTOR_INITIALIZER;
static rc_matrix_t R_vio_to_imu = RC_MATRIX_INITIALIZER;
static rc_vector_t T_vio_wrt_imu = RC_VECTOR_INITIALIZER;
static rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
static rc_vector_t T_body_wrt_local = RC_VECTOR_INITIALIZER;
static rc_matrix_t R_cam_to_imu = RC_MATRIX_INITIALIZER;
static rc_vector_t T_cam_wrt_imu = RC_VECTOR_INITIALIZER;
////////////////////////////////////////////////////////////////////////////////
// end of tf_mutex protected dynamic transforms
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// these are static transforms set up by config file or geometry_initialize
////////////////////////////////////////////////////////////////////////////////
// R_imu_to_body // this is in config.h
// T_imu_wrt_body // this is in config.h
static rc_matrix_t R_body_to_imu = RC_MATRIX_INITIALIZER;
static rc_vector_t T_body_wrt_imu = RC_VECTOR_INITIALIZER;

// local is under body, on the ground, via_ga is where the imu is when the body
// is on the ground. So via_ga_wrt_local is almost the inverse of body_wrt_imu.
static rc_matrix_t R_vio_ga_to_local = RC_MATRIX_INITIALIZER;
static rc_vector_t T_vio_ga_wrt_local = RC_VECTOR_INITIALIZER;

// T_stereo_wrt_body // this is in config.h
// R_stereo_to_body  // this is in config.h
////////////////////////////////////////////////////////////////////////////////
// end of static transforms
////////////////////////////////////////////////////////////////////////////////

// ring buffer of imu_wrt_vio data from VIO, this has its own mutex
static rc_tf_ringbuf_t vio_ringbuf = RC_TF_RINGBUF_INITIALIZER;
static rc_tf_ringbuf_t body_local_ringbuf = RC_TF_RINGBUF_INITIALIZER;

// ring buffer of px4 attitude
static rc_tf_ringbuf_t px4_attitude_ringbuf = RC_TF_RINGBUF_INITIALIZER;

// set to 1 after geometry_init indicating all memory is allocated
static int initialized = 0;
// set to 1 if a big jump is detected from apriltag detection
static int reset_counter = 0;
static int fixed_frame_debug = 0;

int geometry_invert_tf(rc_matrix_t R_A_to_B, rc_vector_t T_A_wrt_B,
					   rc_matrix_t *R_B_to_A, rc_vector_t *T_B_wrt_A)
{
	rc_matrix_transpose(R_A_to_B, R_B_to_A);
	rc_matrix_times_col_vec(*R_B_to_A, T_A_wrt_B, T_B_wrt_A);
	rc_vector_times_scalar(T_B_wrt_A, -1.0);
	return 0;
}

static void _populate_transforms(void)
{
	// This is all done AFTER T_imu_wrt_body and R_imu_to_body are set

	// set imu_to_body as 0/identity for now
	rc_vector_alloc(&T_vio_ga_wrt_local,3);
	T_vio_ga_wrt_local.d[0] = T_imu_wrt_body.d[0];
	T_vio_ga_wrt_local.d[1] = T_imu_wrt_body.d[1];
	T_vio_ga_wrt_local.d[2] = T_imu_wrt_body.d[2] - height_body_above_ground_m;

	// identity rotation from local to fixed until a tag is detected
	rc_matrix_identity(&R_local_to_fixed, 3);
	rc_matrix_identity(&R_fixed_to_local, 3);
	rc_vector_zeros(&T_local_wrt_fixed, 3);
	rc_vector_zeros(&T_fixed_wrt_local, 3);

	// from config file
	rc_matrix_duplicate(R_imu_to_body, &R_vio_ga_to_local);
	rc_matrix_duplicate(R_imu_to_body, &R_vio_to_local);

	// identity until gravity correction
	rc_matrix_identity(&R_vio_to_vio_ga, 3);

	// set when VIO data comes in
	rc_matrix_identity(&R_imu_to_vio, 3);
	rc_vector_zeros(&T_imu_wrt_vio, 3);
	rc_matrix_identity(&R_vio_to_imu, 3);
	rc_vector_zeros(&T_body_wrt_local, 3);
	rc_matrix_identity(&R_body_to_local, 3);

	// imu to body is from config file, just need to find its inverse
	geometry_invert_tf(R_imu_to_body, T_imu_wrt_body, &R_body_to_imu, &T_body_wrt_imu);

	// from tbc/ombc from vio data, start as identity, will get loaded on first vio
	rc_matrix_identity(&R_cam_to_imu, 3);
	rc_vector_zeros(&T_cam_wrt_imu, 3);

	return;
}

int geometry_set_imu(const char *imu_string)
{
	int n, j, k;
	vcc_extrinsic_t t[VCC_MAX_EXTRINSICS_IN_CONFIG];
	vcc_extrinsic_t tmp;

	// now load in extrinsics
	if (vcc_read_extrinsic_conf_file(VCC_EXTRINSICS_PATH, t, &n, VCC_MAX_EXTRINSICS_IN_CONFIG))
	{
		return -1;
	}

	// Pick out IMU to Body. config.c already set this up for imu1 so just leave it as is
	if (vcc_find_extrinsic_in_array("body", imu_string, t, n, &tmp))
	{
		fprintf(stderr, "ERROR: %s missing body to %s, sticking with identity for now\n", VCC_EXTRINSICS_PATH, imu_string);
		return -1;
	}

	pthread_mutex_lock(&tf_mutex);

	rc_vector_from_array(&T_imu_wrt_body, tmp.T_child_wrt_parent, 3);
	rc_matrix_alloc(&R_imu_to_body, 3, 3);
	for (j = 0; j < 3; j++){
		for (k = 0; k < 3; k++)
			R_imu_to_body.d[j][k] = tmp.R_child_to_parent[j][k];
	}
	printf("new T imu wrt body:\n");
	rc_vector_print(T_imu_wrt_body);
	printf("new R imu to body:\n");
	rc_matrix_print(R_imu_to_body);

	// set other transforms that depends on IMU
	_populate_transforms();

	pthread_mutex_unlock(&tf_mutex);

	printf("done updating transforms to use imu: %s\n", imu_string);
	return 0;
}


int geometry_init(void)
{
	// start with a base config assuming IMU is at the COG and oriented the same
	// this will get set more correctly when we get the first VIO data that says
	// exactly which imu is being used
	pthread_mutex_lock(&tf_mutex);
	rc_vector_zeros(&T_imu_wrt_body, 3);
	rc_matrix_identity(&R_imu_to_body, 3);
	_populate_transforms();
	pthread_mutex_unlock(&tf_mutex);

	// allocate filters and ringbuffers for apriltag detection
	rc_tf_ringbuf_alloc(&vio_ringbuf, VIO_RINGBUF_SIZE);
	rc_tf_ringbuf_alloc(&body_local_ringbuf, VIO_RINGBUF_SIZE);
	rc_tf_ringbuf_alloc(&px4_attitude_ringbuf, PX4_ATTITUDE_RINGBUF_SIZE);

	// filter for offset between local and fixed frames
	if (fixed_frame_filter_len < 1)
	{
		fprintf(stderr, "WARNING: param fixed_frame_filter_len in config file should be >=1\n");
		fixed_frame_filter_len = 5;
	}
	rc_pose_filter_alloc(&filter_local_to_fixed, fixed_frame_filter_len);
	initialized = 1;
	return 0;
}

void geometry_en_print_fixed_frame_debug(int debug)
{
	if (debug)
		fixed_frame_debug = 1;
}

// This is the rotation PX4 wants for Visual odometry
int geometry_get_R_body_to_local(rc_matrix_t *out)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_duplicate(R_body_to_local, out);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// This is the position PX4 wants for Visual odometry
int geometry_get_T_body_wrt_local(rc_vector_t *out)
{
	pthread_mutex_lock(&tf_mutex);
	rc_vector_duplicate(T_body_wrt_local, out);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

int geometry_get_tait_bryan_body_wrt_local(double *roll, double *pitch, double *yaw)
{
	rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
	geometry_get_R_body_to_local(&R_body_to_local);
	rc_rotation_to_tait_bryan(R_body_to_local, roll, pitch, yaw);
	rc_matrix_free(&R_body_to_local);
	return 0;
}

int geometry_get_R_body_to_fixed(rc_matrix_t *out)
{
	// get most of the way there with R from body to local
	geometry_get_R_body_to_local(out);
	// then just rotate into fixed frame
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_left_multiply_inplace(R_local_to_fixed, out);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

int geometry_get_T_body_wrt_fixed(rc_vector_t *out)
{
	// get most of the way there with body to local.
	geometry_get_T_body_wrt_local(out);
	pthread_mutex_lock(&tf_mutex);
	// rotate into fixed frame
	rc_matrix_times_col_vec_inplace(R_local_to_fixed, out);
	// add offset of local frame in fixed frame
	rc_vector_sum_inplace(out, T_local_wrt_fixed);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// used to translate position setpoints from fixed to local frame
int geometry_transform_vec_from_fixed_to_local_frame(rc_vector_t p_fixed, rc_vector_t *p_local)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_fixed_to_local, p_fixed, p_local);
	rc_vector_sum_inplace(p_local, T_fixed_wrt_local);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// used to translate velocity setpoints from fixed to local frame
int geometry_rotate_vec_from_fixed_to_local_frame(rc_vector_t v_fixed, rc_vector_t *v_local)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_fixed_to_local, v_fixed, v_local);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// used to translate velocity vectors from local to fixed frame
int geometry_rotate_vec_from_local_to_fixed_frame(rc_vector_t v_local, rc_vector_t *v_fixed)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_local_to_fixed, v_local, v_fixed);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// Used to rotate vio velocity output in vio frame to body frame
int geometry_calc_velocity_in_body_frame(rc_vector_t v_vio, rc_vector_t w_body_wrt_imu, rc_vector_t *v_body)
{
	// find velocity in imu_frame of body induced from angular rate
	RC_VECTOR_ON_STACK(tmp, 3);
	rc_vector_cross_product(w_body_wrt_imu, T_body_wrt_imu, &tmp);
	rc_matrix_times_col_vec_inplace(R_imu_to_body, &tmp);

	// find velocity from vio in body frame
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_vio_to_imu, v_vio, v_body);
	pthread_mutex_unlock(&tf_mutex);
	rc_matrix_times_col_vec_inplace(R_imu_to_body, v_body);

	// sum the two together
	rc_vector_sum_inplace(v_body, tmp);
	return 0;
}

// Used to rotate vio velocity output in vio frame to body frame
int geometry_calc_velocity_in_local_frame(rc_vector_t v_imu_wrt_vio,
										  rc_vector_t w_imu_wrt_imu, rc_vector_t *v_body_wrt_local)
{
	// find velocity of body in imu frame induced from angular rate
	RC_VECTOR_ON_STACK(cross, 3);
	rc_vector_cross_product(w_imu_wrt_imu, T_body_wrt_imu, &cross);

	// then rotate that to a velocity in local frame
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec_inplace(R_imu_to_vio, &cross);
	rc_matrix_times_col_vec_inplace(R_vio_to_local, &cross);

	// rotate VIO's reported velocity (v_imu_wrt_vio) to local frame
	// with the assumption that the IMU and Body are rigidly attached
	rc_matrix_times_col_vec(R_vio_to_local, v_imu_wrt_vio, v_body_wrt_local);
	pthread_mutex_unlock(&tf_mutex);

	// sum the two velocities together
	rc_vector_sum_inplace(v_body_wrt_local, cross);
	return 0;
}

int geometry_rotate_vec_from_vio_to_body_frame(rc_vector_t v_vio, rc_vector_t *v_body)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_vio_to_imu, v_vio, v_body);
	pthread_mutex_unlock(&tf_mutex);
	rc_matrix_times_col_vec_inplace(R_imu_to_body, v_body);
	return 0;
}

int geometry_rotate_vec_from_imu_to_body_frame(rc_vector_t v_imu, rc_vector_t *v_body)
{
	// R_imu_to_body is static, no need to lock mutex
	// TODO check if this is identity (which it often is) and skip the multiplication
	rc_matrix_times_col_vec(R_imu_to_body, v_imu, v_body);
	return 0;
}

int geometry_rotate_vec_from_vio_to_local_frame(rc_vector_t v_vio, rc_vector_t *v_local)
{
	pthread_mutex_lock(&tf_mutex);
	rc_matrix_times_col_vec(R_vio_to_local, v_vio, v_local);
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

/*
 * Updates local estimate from VIO to gravity corrected vio frame
 *
 * returns 0 on success
 * -1 if data is not ready for gravity correction
 * -2 if orientation is so far off VIO should be reset
 */
int geometry_update_gravity_offset(float *grav)
{
	static rc_vector_t grav_wrt_local = RC_VECTOR_INITIALIZER;
	static rc_vector_t grav_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t down_wrt_local = RC_VECTOR_INITIALIZER;
	static rc_vector_t down_wrt_vio = RC_VECTOR_INITIALIZER;
	static rc_vector_t cross = RC_VECTOR_INITIALIZER;

	// downward facing vector where gravity should point
	rc_vector_alloc(&down_wrt_local, 3);
	down_wrt_local.d[0] = 0.0;
	down_wrt_local.d[1] = 0.0;
	down_wrt_local.d[2] = 1.0;

	// gravity vector in VIO frame as reported by vio
	rc_vector_alloc(&grav_wrt_vio, 3);
	grav_wrt_vio.d[0] = grav[0];
	grav_wrt_vio.d[1] = grav[1];
	grav_wrt_vio.d[2] = grav[2];

	// rotate grav vector from vio to local frame
	rc_matrix_times_col_vec(R_vio_ga_to_local, grav_wrt_vio, &grav_wrt_local);

	// rotate down vector from local to vio frame
	rc_matrix_times_col_vec(R_body_to_imu, down_wrt_local, &down_wrt_vio);

	// check for flip
	if (grav_wrt_local.d[2] < 0.0)
	{
		fprintf(stderr, "WARNING can't initialize VIO gravity correction, drone upside down\n");
		fprintf(stderr, "If this message keeps appearing when the drone is righted, the extrinsics\n");
		fprintf(stderr, "config file /etc/modalai/extrinsics.conf is likely setup incorrectly.\n");
		fprintf(stderr, "run voxl-configure-extrinsics to reset this file to one of several default configs\n");
		return -2;
	}

	// normalize for magnitude and find cross product
	double grav_magnitude = rc_vector_norm(grav_wrt_vio, 2.0);
	if(grav_magnitude < 0.1){
		fprintf(stderr, "ERROR in %s magnitude of gravity vector too small\n", __FUNCTION__);
		return -1;
	}

	grav_wrt_vio.d[0] = grav_wrt_vio.d[0] / grav_magnitude;
	grav_wrt_vio.d[1] = grav_wrt_vio.d[1] / grav_magnitude;
	grav_wrt_vio.d[2] = grav_wrt_vio.d[2] / grav_magnitude;
	rc_vector_cross_product(grav_wrt_vio, down_wrt_vio, &cross);

	// find angle of rotation
	// a x b = ||a|| * ||b|| * sin(a) * n where n is a unit vector in direction
	// off the cross product
	// we already normalized a and b so ||a x b||=1
	double cross_magnitude = rc_vector_norm(cross, 2.0);
	double angle = asin(cross_magnitude);
	double angle_deg = (angle / 6.28318530718) * 360.0;

	// now calculate the rotation matrix for the gravity vector
	static rc_matrix_t R_grav_vector = RC_MATRIX_INITIALIZER;
	if(fabs(angle_deg) > 35){
		fprintf(stderr, "WARNING initializing VIO gravity correction, system not level enough\n");
		return -3;
	}
	else if(fabs(angle)<0.001){
		//printf("Angle less than 0.001 %f\n", angle);
		rc_matrix_identity(&R_vio_to_vio_ga, 3);
		rc_matrix_identity(&R_grav_vector, 3); // needed initialization to identity
	}
	else if(rc_axis_angle_to_rotation_matrix(cross, angle, &R_grav_vector)){
		fprintf(stderr, "ERROR in %s calling rc_axis_angle_to_rotation_matrix\n", __FUNCTION__);
		return -1;
	}

	// and calculate the additional offset from horizon calibration
	static rc_matrix_t R_horizon_cal = RC_MATRIX_INITIALIZER;
	#ifdef USE_VIO_HORIZON_CAL
	if(rc_rotation_matrix_from_tait_bryan(	-vio_reported_roll_when_level_deg * DEG_TO_RAD,\
											-vio_reported_pitch_when_level_deg * DEG_TO_RAD,\
											0.0, &R_horizon_cal)){
		fprintf(stderr, "ERROR in %s calculating vio horizon cal correction\n", __FUNCTION__);
		return -1;
	}
	#else
	rc_matrix_identity(&R_horizon_cal, 3);
	#endif

	// lock mutex before writing out results
	pthread_mutex_lock(&tf_mutex);

	// calculate new gravity correction rotation from both gravity vector and horizon cal
	rc_matrix_multiply(R_horizon_cal, R_grav_vector, &R_vio_to_vio_ga);

	// also save VIO to local to save time in the future
	// R_vio_to_local is what's used when we process new vio pose data
	rc_matrix_multiply(R_vio_ga_to_local, R_vio_to_vio_ga, &R_vio_to_local);

	pthread_mutex_unlock(&tf_mutex);

	return 0;
}

// call this when new VIO data is received
// it copies new data into local record of frames
int geometry_update_imu_to_vio(int64_t timestamp_ns, float T[3], float R[3][3])
{
	int i, j;
	if (!initialized)
	{
		fprintf(stderr, "ERROR in geometry_update_imu_to_vio, call geometry_init() first\n");
		return -1;
	}

	pthread_mutex_lock(&tf_mutex);
	for (i = 0; i < 3; i++)
		T_imu_wrt_vio.d[i] = T[i];
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
			R_imu_to_vio.d[i][j] = R[i][j];
	}
	// also store the inverse
	geometry_invert_tf(R_imu_to_vio, T_imu_wrt_vio,
					   &R_vio_to_imu, &T_vio_wrt_imu);

	// calc body to local to save in the ringbuf
	rc_matrix_multiply(R_imu_to_vio, R_body_to_imu, &R_body_to_local);
	rc_matrix_left_multiply_inplace(R_vio_to_local, &R_body_to_local);

	// start with location of body wrt imu and go up the chain to fixed frame
	// first rotate vector from imu to body into vio frame
	rc_matrix_times_col_vec(R_imu_to_vio, T_body_wrt_imu, &T_body_wrt_local);
	// now we are in vio frame, can add offset of imu in vio frame
	rc_vector_sum_inplace(&T_body_wrt_local, T_imu_wrt_vio);
	// rotate into vio_ga frame and then into local frame in one step using the
	// R_vio_to_local shortcut calculated when grav vector was updated
	rc_matrix_times_col_vec_inplace(R_vio_to_local, &T_body_wrt_local);
	// add offset of vio_ga in local frame
	rc_vector_sum_inplace(&T_body_wrt_local, T_vio_ga_wrt_local);

	// add data to ringbuf
	rc_tf_ringbuf_insert(&vio_ringbuf, timestamp_ns, R_imu_to_vio, T_imu_wrt_vio);
	rc_tf_ringbuf_insert(&body_local_ringbuf, timestamp_ns, R_body_to_local, T_body_wrt_local);

	pthread_mutex_unlock(&tf_mutex);

	return 0;
}

// call this when new VIO data is received
// it copies new data into local record of frames
int geometry_update_cam_to_imu(float T[3], float R[3][3])
{
	int i, j;
	if (!initialized)
	{
		fprintf(stderr, "ERROR in geometry_update_cam_to_imu, call geometry_init() first\n");
		return -1;
	}

	pthread_mutex_lock(&tf_mutex);
	for (i = 0; i < 3; i++)
		T_cam_wrt_imu.d[i] = T[i];
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
			R_cam_to_imu.d[i][j] = R[i][j];
	}
	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

int geometry_add_fixed_tag_detection(int64_t frame_timestamp_ns,
									 rc_matrix_t R_tag_to_cam, rc_vector_t T_tag_wrt_cam,
									 rc_matrix_t R_tag_to_fixed, rc_vector_t T_tag_wrt_fixed)
{
	static rc_matrix_t R_tag_to_local;
	static rc_vector_t T_tag_wrt_local;
	static rc_matrix_t R_fixed_to_tag;
	static rc_vector_t tmp;
	static rc_matrix_t new_R_fixed_to_local = RC_MATRIX_INITIALIZER;
	static rc_matrix_t new_R_local_to_fixed = RC_MATRIX_INITIALIZER;
	static rc_vector_t new_T_fixed_wrt_local = RC_VECTOR_INITIALIZER;
	static rc_vector_t new_T_local_wrt_fixed = RC_VECTOR_INITIALIZER;
	double roll, pitch, yaw;

	/*
	// this was a debug print to confirm there is no phase offset between the tag
	// detection and corrected interpolated VIO position. It was good!!!
	printf("ts: %lld tag_wrt_cam: %5.3f %5.3f %5.3f  correct_imu_wrt_vio: %5.3f %5.3f %5.3f\n",\
	frame_timestamp_ns,\
	new_T_tag_wrt_cam.d[0], new_T_tag_wrt_cam.d[1],new_T_tag_wrt_cam.d[2],\
	correct_T_imu_wrt_vio.d[0], correct_T_imu_wrt_vio.d[1],correct_T_imu_wrt_vio.d[2]);
	*/

	// fetch position and rotation of the tag in local frame.
	// This goes back in time and interpolates given timestamp
	if (geometry_calc_R_T_tag_in_local_frame(frame_timestamp_ns, R_tag_to_cam,
											 T_tag_wrt_cam, &R_tag_to_local, &T_tag_wrt_local))
	{
		return -1;
	}

	rc_matrix_transpose(R_tag_to_fixed, &R_fixed_to_tag);

	// calc new yaw local to fixed and reduce rotation to pure yaw
	// roll/pitch values are still used to check for error
	rc_matrix_multiply(R_tag_to_local,
					   R_fixed_to_tag,
					   &new_R_fixed_to_local);
	rc_rotation_to_tait_bryan(new_R_fixed_to_local, &roll, &pitch, &yaw);
	// overwrite new_R_fixed_to_local with pure yaw
	rc_rotation_matrix_from_yaw(yaw, &new_R_fixed_to_local);

	// ignore bad detections that are >10 degrees off (.174 radians)
	if (fabs(roll) > 0.174 || fabs(pitch) > 0.174)
	{
		fprintf(stderr, "WARNING, apriltag roll/pitch out of bounds\n");
		return -1;
	}

	// calc new T_fixed_wrt_local
	// T_tl = (R_fl*T_tf)+T_fl  -->  T_fl = T_tl - (R_fl*T_tl)
	rc_matrix_times_col_vec(new_R_fixed_to_local,
							T_tag_wrt_fixed, &tmp);
	rc_vector_subtract(T_tag_wrt_local, tmp, &new_T_fixed_wrt_local);

	// find the inverse of R&T, this is what will actually be saved
	geometry_invert_tf(new_R_fixed_to_local, new_T_fixed_wrt_local,
					   &new_R_local_to_fixed, &new_T_local_wrt_fixed);

	// debug check of new tanslation
	if (fixed_frame_debug)
	{
		printf("T_fixed_wrt_local: %5.2f %5.2f %5.2f T_local_wrt_fixed: %5.2f %5.2f %5.2f yaw_local_to_fixed: %5.2f\n",
			   new_T_fixed_wrt_local.d[0], new_T_fixed_wrt_local.d[1], new_T_fixed_wrt_local.d[2],
			   new_T_local_wrt_fixed.d[0], new_T_local_wrt_fixed.d[1], new_T_local_wrt_fixed.d[2],
			   yaw);
	}

	// add new values into filter
	rc_pose_filter_march(&filter_local_to_fixed, new_T_local_wrt_fixed, yaw);

	// update global variables and set rotation from yaw
	pthread_mutex_lock(&tf_mutex);
	rc_pose_filter_fetch(&filter_local_to_fixed, &T_local_wrt_fixed, &yaw_local_to_fixed);
	rc_rotation_matrix_from_yaw(-yaw_local_to_fixed, &R_local_to_fixed);

	// also calculate the inverse translation
	geometry_invert_tf(R_local_to_fixed, T_local_wrt_fixed,
					   &R_fixed_to_local, &T_fixed_wrt_local);
	pthread_mutex_unlock(&tf_mutex);

	// if this is the first detection let the user know
	if (filter_local_to_fixed.step == 1)
	{
		printf("first tag detected: new yaw: %0.2f, T_local_wrt_fixed: %5.2f %5.2f %5.2f\n",
			   yaw_local_to_fixed,
			   T_local_wrt_fixed.d[0], T_local_wrt_fixed.d[1], T_local_wrt_fixed.d[2]);
	}

	return 0;
}

int geometry_calc_R_T_tag_in_local_frame(int64_t frame_timestamp_ns,
										 rc_matrix_t R_tag_to_cam, rc_vector_t T_tag_wrt_cam,
										 rc_matrix_t *R_tag_to_local, rc_vector_t *T_tag_wrt_local)
{
	// these "correct" values are from the past aligning to the timestamp
	static rc_matrix_t correct_R_imu_to_vio = RC_MATRIX_INITIALIZER;
	static rc_vector_t correct_T_imu_wrt_vio = RC_VECTOR_INITIALIZER;

	int ret = rc_tf_ringbuf_get_tf_at_time(&vio_ringbuf,
										   frame_timestamp_ns,
										   &correct_R_imu_to_vio,
										   &correct_T_imu_wrt_vio);

	// fail silently, on catastrophic failure the previous function would have
	// printed a message. If ret==-2 that's a soft failure meaning we just don't
	// have enough vio data yet, so also return silently.
	if (ret < 0)
		return -1;

	pthread_mutex_lock(&tf_mutex);

	// calculate position of tag wrt local
	rc_matrix_times_col_vec(R_cam_to_imu, T_tag_wrt_cam, T_tag_wrt_local);
	rc_vector_sum_inplace(T_tag_wrt_local, T_cam_wrt_imu);
	rc_matrix_times_col_vec_inplace(correct_R_imu_to_vio, T_tag_wrt_local);
	rc_vector_sum_inplace(T_tag_wrt_local, correct_T_imu_wrt_vio);
	rc_matrix_times_col_vec_inplace(R_vio_to_local, T_tag_wrt_local);
	rc_vector_sum_inplace(T_tag_wrt_local, T_vio_ga_wrt_local);

	// calculate rotation tag to local
	rc_matrix_multiply(R_cam_to_imu, R_tag_to_cam, R_tag_to_local);
	rc_matrix_left_multiply_inplace(correct_R_imu_to_vio, R_tag_to_local);
	rc_matrix_left_multiply_inplace(R_vio_to_local, R_tag_to_local);

	pthread_mutex_unlock(&tf_mutex);
	return 0;
}

// reset counter isn't used yet, just here incase
int geometry_get_reset_counter(void)
{
	return reset_counter;
}

int geometry_bump_reset_counter(void)
{
	reset_counter++;
	return reset_counter;
}

int geometry_transform_fixed_setpoint_to_local(mavlink_set_position_target_local_ned_t in,
											   mavlink_set_position_target_local_ned_t *out)
{
	rc_vector_t tmp1 = RC_VECTOR_INITIALIZER;
	rc_vector_t tmp2 = RC_VECTOR_INITIALIZER;
	rc_vector_alloc(&tmp1, 3);
	rc_vector_alloc(&tmp2, 3);

	// start by copying everything to make sure nothing is missed
	memcpy(out, &in, sizeof(mavlink_set_position_target_local_ned_t));

	tmp1.d[0] = in.x;
	tmp1.d[1] = in.y;
	tmp1.d[2] = in.z;
	geometry_transform_vec_from_fixed_to_local_frame(tmp1, &tmp2);
	out->x = tmp2.d[0];
	out->y = tmp2.d[1];
	out->z = tmp2.d[2];

	if (!(in.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE))
	{
		tmp1.d[0] = in.vx;
		tmp1.d[1] = in.vy;
		tmp1.d[2] = in.vz;
		geometry_rotate_vec_from_fixed_to_local_frame(tmp1, &tmp2);
		out->vx = tmp2.d[0];
		out->vy = tmp2.d[1];
		out->vz = tmp2.d[2];
	}

	if (!(in.type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE))
	{
		tmp1.d[0] = in.afx;
		tmp1.d[1] = in.afy;
		tmp1.d[2] = in.afz;
		geometry_rotate_vec_from_fixed_to_local_frame(tmp1, &tmp2);
		out->afx = tmp2.d[0];
		out->afy = tmp2.d[1];
		out->afz = tmp2.d[2];
	}

	double new_yaw = (double)in.yaw + yaw_local_to_fixed;
	WRAP_TO_NEGPI_TO_PI(new_yaw);
	out->yaw = (float)new_yaw;

	/*
		printf("old setpoint: pos: %0.1f %0.1f %0.1f yaw: %d\n",\
			in.x,in.y,in.z, (int)(in.yaw*360.0/(2.0*M_PI)));
		printf("new setpoint: pos: %0.1f %0.1f %0.1f yaw: %d\n",\
			out->x,out->y,out->z, (int)(out->yaw*360.0/(2.0*M_PI)));
	*/

	rc_vector_free(&tmp1);
	rc_vector_free(&tmp2);
	return 0;
}

int geometry_get_T_body_in_past(int64_t past_ts_ns, float *xyz)
{
	// these "correct" values are from the past aligning to the timestamp
	static rc_matrix_t past_R_body_to_local = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(past_T_body_wrt_local, 3);

	int ret = rc_tf_ringbuf_get_tf_at_time(&body_local_ringbuf,
										   past_ts_ns,
										   &past_R_body_to_local,
										   &past_T_body_wrt_local);
	rc_matrix_free(&past_R_body_to_local);

	if (ret)
		return -1;

	xyz[0] = past_T_body_wrt_local.d[0];
	xyz[1] = past_T_body_wrt_local.d[1];
	xyz[2] = past_T_body_wrt_local.d[2];
	return 0;
}

int geometry_get_RT_frame_in_past_to_level(char *frame, int64_t past_ts_ns,
										   int64_t new_ts_ns, rc_matrix_t *R, rc_vector_t *T)
{
	int ret = 0;

	// these "correct" values are from the past aligning to the timestamp
	static rc_matrix_t past_R_body_to_local = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(past_T_body_wrt_local, 3);
	static rc_matrix_t new_R_body_to_local = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(new_T_body_wrt_local, 3);
	static rc_matrix_t new_R_local_to_body = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(new_T_local_wrt_body, 3);
	static rc_matrix_t R_frame_to_body = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(T_frame_wrt_body, 3);

	// fetch RT from desired frame to body
	if (extrinsics_fetch_frame_to_body(frame, &R_frame_to_body, &T_frame_wrt_body))
		return -1;

	pthread_mutex_lock(&tf_mutex);

	int ret1 = rc_tf_ringbuf_get_tf_at_time(&body_local_ringbuf,
										   past_ts_ns,
										   &past_R_body_to_local,
										   &past_T_body_wrt_local);

	int ret2 = rc_tf_ringbuf_get_tf_at_time(&body_local_ringbuf,
											new_ts_ns,
											&new_R_body_to_local,
											&new_T_body_wrt_local);

	// if we failed to get a transform from the ringbuf, VIO data is not yet available
	if(ret1 < 0 || ret2){
		rc_matrix_identity(&past_R_body_to_local, 3);
		memset(past_T_body_wrt_local.d, 0, 3*sizeof(double));
		rc_matrix_identity(&new_R_body_to_local, 3);
		memset(new_T_body_wrt_local.d, 0, 3*sizeof(double));
		ret = -2;
	}

	// also get inverse
	geometry_invert_tf(new_R_body_to_local, new_T_body_wrt_local,
					   &new_R_local_to_body, &new_T_local_wrt_body);

	// If we got here then VIO is valid we we have good interpolated data from
	// the VIO ringbuffer. Use this data to do proper angle and translation
	// compensation since the depth-from-stereo data may be quite delayed.

	// find the rotation from body to body_level by looking at the cross product
	// of a known down vector with the vector pointing out the bottom of the drone
	// start with a reference downward facing vector
	RC_VECTOR_ON_STACK(vec_down, 3);
	vec_down.d[0] = 0;
	vec_down.d[1] = 0;
	vec_down.d[2] = 1.0;

	// now find the vector pointing out bottom of body frame
	RC_VECTOR_ON_STACK(vec_out_bottom, 3);
	rc_matrix_times_col_vec(new_R_local_to_body, vec_down, &vec_out_bottom);

	// check for flip
	static int has_warned = 0;
	if(vec_out_bottom.d[2] < 0.0){
		if(!has_warned){
			fprintf(stderr, "WARNING in %s, drone upside down\n", __FUNCTION__);
			has_warned = 1;
		}
		pthread_mutex_unlock(&tf_mutex);
		return -2;
	}
	has_warned = 0;

	// find cross product, no need to normalize as we started with unit vectors
	RC_VECTOR_ON_STACK(cross, 3);
	rc_vector_cross_product(vec_out_bottom, vec_down, &cross);

	// find angle of rotation
	// a x b = ||a|| * ||b|| * sin(a) * n where n is a unit vector in direction
	// off the cross product
	// we already normalized a and b so ||a x b||=1
	double cross_magnitude = rc_vector_norm(cross, 2.0);
	double angle = asin(cross_magnitude);

	static rc_matrix_t new_R_body_to_level_at_t = RC_MATRIX_INITIALIZER;
	if(fabs(angle)<0.001){
		rc_matrix_identity(&new_R_body_to_level_at_t, 3);
	}
	else if(rc_axis_angle_to_rotation_matrix(cross, angle, &new_R_body_to_level_at_t)){
		fprintf(stderr, "ERROR in %s axis_angle_to_rotation_matrix\n", __FUNCTION__);
		return -5;
	}

	// find rotation R_frame_to_level
	rc_matrix_multiply(past_R_body_to_local, R_frame_to_body, R);
	rc_matrix_left_multiply_inplace(new_R_local_to_body, R);
	rc_matrix_left_multiply_inplace(new_R_body_to_level_at_t, R);

	// find translation
	rc_matrix_times_col_vec(past_R_body_to_local, T_frame_wrt_body, T);
	rc_vector_sum_inplace(T, past_T_body_wrt_local);
	rc_matrix_times_col_vec_inplace(new_R_local_to_body, T);
	rc_vector_sum_inplace(T, new_T_local_wrt_body);
	rc_matrix_times_col_vec_inplace(new_R_body_to_level_at_t, T);

	pthread_mutex_unlock(&tf_mutex);
	return ret;
}

int geometry_add_fixed_frame_estimate(pose_4dof_t pose_body_wrt_fixed)
{
	RC_VECTOR_ON_STACK(tmp, 3);
	static rc_matrix_t new_R_fixed_to_local = RC_MATRIX_INITIALIZER;
	static rc_matrix_t new_R_local_to_fixed = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(new_T_fixed_wrt_local, 3);
	RC_VECTOR_ON_STACK(new_T_local_wrt_fixed, 3);

	// these are the values that come in from the pose, put them into vectors
	RC_VECTOR_ON_STACK(T_body_wrt_fixed, 3);
	T_body_wrt_fixed.d = pose_body_wrt_fixed.p;
	double yaw_fixed_to_body = pose_body_wrt_fixed.yaw;

	if (yaw_fixed_to_body > M_PI || yaw_fixed_to_body < -M_PI)
	{
		fprintf(stderr, "ERROR adding fixed_frame_estimate, yaw out of bounds\n");
		return -1;
	}

	// these "correct" values are from the past aligning to the timestamp
	static rc_matrix_t correct_R_imu_to_vio = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(correct_T_imu_wrt_vio, 3);

	int ret = rc_tf_ringbuf_get_tf_at_time(&vio_ringbuf,
										   pose_body_wrt_fixed.timestamp_ns,
										   &correct_R_imu_to_vio,
										   &correct_T_imu_wrt_vio);

	// fail silently, on catastrophic failure the previous function would have
	// printed a message. If ret==-2 that's a soft failure meaning we just don't
	// have enough vio data yet, so also return silently.
	if (ret < 0)
		return -1;

	// calculate body to local at that timestamp
	pthread_mutex_lock(&tf_mutex);
	static rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
	static rc_vector_t T_body_wrt_local = RC_VECTOR_INITIALIZER;
	rc_matrix_multiply(correct_R_imu_to_vio, R_body_to_imu, &R_body_to_local);
	rc_matrix_left_multiply_inplace(R_vio_to_local, &R_body_to_local);

	// start with location of body wrt imu and go up the chain to fixed frame
	// first rotate vector from imu to body into vio frame
	rc_matrix_times_col_vec(correct_R_imu_to_vio, T_body_wrt_imu, &T_body_wrt_local);
	rc_vector_sum_inplace(&T_body_wrt_local, correct_T_imu_wrt_vio);
	rc_matrix_times_col_vec_inplace(R_vio_to_local, &T_body_wrt_local);
	rc_vector_sum_inplace(&T_body_wrt_local, T_vio_ga_wrt_local);
	pthread_mutex_unlock(&tf_mutex);

	// calc new yaw local to fixed and reduce rotation to pure yaw
	// roll/pitch values are still used to check for error
	double roll, pitch, yaw_local_to_body;
	rc_rotation_to_tait_bryan(R_body_to_local, &roll, &pitch, &yaw_local_to_body);

	double new_yaw_local_to_fixed = yaw_local_to_body - yaw_fixed_to_body;
	rc_rotation_matrix_from_yaw(new_yaw_local_to_fixed, &new_R_fixed_to_local);

	// calc new T_fixed_wrt_local
	// T_tl = (R_fl*T_tf)+T_fl  -->  T_fl = T_tl - (R_fl*T_tl)
	rc_matrix_times_col_vec(new_R_fixed_to_local,
							T_body_wrt_fixed, &tmp);
	rc_vector_subtract(T_body_wrt_local, tmp, &new_T_fixed_wrt_local);

	// find the inverse of R&T, this is what will actually be saved
	geometry_invert_tf(new_R_fixed_to_local, new_T_fixed_wrt_local,
					   &new_R_local_to_fixed, &new_T_local_wrt_fixed);

	// debug check of new tanslation
	if (fixed_frame_debug)
	{
		printf("T_fixed_wrt_local: %5.2f %5.2f %5.2f T_local_wrt_fixed: %5.2f %5.2f %5.2f yaw_local_to_fixed: %5.2f\n",
			   new_T_fixed_wrt_local.d[0], new_T_fixed_wrt_local.d[1], new_T_fixed_wrt_local.d[2],
			   new_T_local_wrt_fixed.d[0], new_T_local_wrt_fixed.d[1], new_T_local_wrt_fixed.d[2],
			   new_yaw_local_to_fixed);
	}

	// add new values into filter
	rc_pose_filter_march(&filter_local_to_fixed, new_T_local_wrt_fixed, new_yaw_local_to_fixed);

	// update global variables and set rotation from yaw
	pthread_mutex_lock(&tf_mutex);
	rc_pose_filter_fetch(&filter_local_to_fixed, &T_local_wrt_fixed, &yaw_local_to_fixed);
	rc_rotation_matrix_from_yaw(-yaw_local_to_fixed, &R_local_to_fixed);

	// also calculate the inverse translation
	geometry_invert_tf(R_local_to_fixed, T_local_wrt_fixed,
					   &R_fixed_to_local, &T_fixed_wrt_local);
	pthread_mutex_unlock(&tf_mutex);

	// if this is the first detection let the user know
	if (filter_local_to_fixed.step == 1)
	{
		printf("first fixed_frame input: new yaw: %0.2f, T_local_wrt_fixed: %5.2f %5.2f %5.2f\n",
			   yaw_local_to_fixed,
			   T_local_wrt_fixed.d[0], T_local_wrt_fixed.d[1], T_local_wrt_fixed.d[2]);
	}

	return 0;
}

int geometry_add_fixed_frame_estimate_6dof(pose_vel_6dof_t pose_body_wrt_fixed)
{
	RC_VECTOR_ON_STACK(tmp, 3);

	// these are the values that come in from the pose, put them into vectors
	static rc_matrix_t R_body_to_fixed;
	rc_matrix_alloc(&R_body_to_fixed, 3, 3);
	RC_VECTOR_ON_STACK(T_body_wrt_fixed, 3);
	for (int i = 0; i < 3; i++)
	{
		T_body_wrt_fixed.d[i] = pose_body_wrt_fixed.T_child_wrt_parent[i];
		for (int j = 0; j < 3; j++)
		{
			R_body_to_fixed.d[i][j] = pose_body_wrt_fixed.R_child_to_parent[i][j];
		}
	}

	// these "correct" values are from VIO in the past aligning to the timestamp
	static rc_matrix_t correct_R_imu_to_vio = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(correct_T_imu_wrt_vio, 3);

	int ret = rc_tf_ringbuf_get_tf_at_time(&vio_ringbuf,
										   pose_body_wrt_fixed.timestamp_ns,
										   &correct_R_imu_to_vio,
										   &correct_T_imu_wrt_vio);

	// fail silently, on catastrophic failure the previous function would have
	// printed a message. If ret==-2 that's a soft failure meaning we just don't
	// have enough vio data yet, so also return silently.
	if (ret < 0)
		return -1;

	// calculate body to local at that timestamp
	pthread_mutex_lock(&tf_mutex);
	static rc_matrix_t R_body_to_local = RC_MATRIX_INITIALIZER;
	static rc_vector_t T_body_wrt_local = RC_VECTOR_INITIALIZER;
	rc_matrix_multiply(correct_R_imu_to_vio, R_body_to_imu, &R_body_to_local);
	rc_matrix_left_multiply_inplace(R_vio_to_local, &R_body_to_local);

	// start with location of body wrt imu and go up the chain to fixed frame
	// first rotate vector from imu to body into vio frame
	rc_matrix_times_col_vec(correct_R_imu_to_vio, T_body_wrt_imu, &T_body_wrt_local);
	rc_vector_sum_inplace(&T_body_wrt_local, correct_T_imu_wrt_vio);
	rc_matrix_times_col_vec_inplace(R_vio_to_local, &T_body_wrt_local);
	rc_vector_sum_inplace(&T_body_wrt_local, T_vio_ga_wrt_local);

	// CALC new_R_fixed_to_local here!
	// have R_body_to_fixed & R_body_to_local, want R_fixed_to_local
	static rc_matrix_t R_fixed_to_body = RC_MATRIX_INITIALIZER;
	rc_matrix_transpose(R_body_to_fixed, &R_fixed_to_body);
	rc_matrix_multiply(R_body_to_local, R_fixed_to_body, &R_fixed_to_local);

	// calc new T_fixed_wrt_local
	// T_tl = (R_fl*T_tf)+T_fl  -->  T_fl = T_tl - (R_fl*T_tl)
	rc_matrix_times_col_vec(R_fixed_to_local,
							T_body_wrt_fixed, &tmp);
	rc_vector_subtract(T_body_wrt_local, tmp, &T_fixed_wrt_local);

	// find the inverse of R&T
	geometry_invert_tf(R_fixed_to_local, T_fixed_wrt_local,
					   &R_local_to_fixed, &T_local_wrt_fixed);

	// find the new local to fixed yaw which is used late rin offboard mode
	// to update the yaw orientation of the drone
	double roll, pitch, yaw;
	rc_rotation_to_tait_bryan(R_fixed_to_local, &roll, &pitch, &yaw);
	yaw_local_to_fixed = yaw;

	pthread_mutex_unlock(&tf_mutex);

	// debug check of new tanslation
	if (fixed_frame_debug)
	{
		printf("T_local_wrt_fixed: %5.2f %5.2f %5.2f  RPY l_to_f deg %5.1f %5.1f %5.1f\n",
			   T_local_wrt_fixed.d[0], T_local_wrt_fixed.d[1], T_local_wrt_fixed.d[2],
			   roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);
	}

	return 0;
}


int geometry_add_px4_attitude(int64_t ts_ns, mavlink_attitude_t a)
{
	static rc_matrix_t R_body_to_local;
	static rc_vector_t T_body_wrt_local;

	rc_matrix_alloc(&R_body_to_local, 3, 3);
	if (!T_body_wrt_local.initialized)
	{
		rc_vector_zeros(&T_body_wrt_local, 3);
	}

	rc_rotation_matrix_from_tait_bryan(a.roll, a.pitch, a.yaw, &R_body_to_local);

	int ret = rc_tf_ringbuf_insert(&px4_attitude_ringbuf, ts_ns, R_body_to_local, T_body_wrt_local);
	if (ret)
	{
		fprintf(stderr, "ERROR in %s adding attitude to ringbuf\n", __FUNCTION__);
		return ret;
	}

	return 0;
}


// return value of -2 means we couldn't find PX4 attitude data but provided identity anyway
int geometry_get_RT_frame_in_past_to_level_from_px4_attitude(char *frame,
															 int64_t past_ts_ns, int64_t new_ts_ns, rc_matrix_t *R, rc_vector_t *T)
{
	int ret = 0;

	// sensor frame to body from extrinsics file
	static rc_matrix_t R_frame_to_body = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(T_frame_wrt_body, 3);

	// body to local frame from past from ringbuffer. In this case translation
	// will be 0 since we are only using the px4 attitude buffer
	static rc_matrix_t past_R_body_to_local = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(past_T_body_wrt_local, 3);

	// body to local frame at the new timestamp from forward interpolation
	// also its inverse
	static rc_matrix_t new_R_body_to_local = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(new_T_body_wrt_local, 3);
	static rc_matrix_t new_R_local_to_body = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(new_T_local_wrt_body, 3);

	// fetch RT from desired frame to body
	if (extrinsics_fetch_frame_to_body(frame, &R_frame_to_body, &T_frame_wrt_body))
	{
		return -5;
	}

	// fetch the old attitude
	int ret1 = rc_tf_ringbuf_get_tf_at_time(&px4_attitude_ringbuf,
										   past_ts_ns,
										   &past_R_body_to_local,
										   &past_T_body_wrt_local);

	// fetch where the body is in local frame at the desired timestamp
	int ret2 = rc_tf_ringbuf_get_tf_at_time(&px4_attitude_ringbuf,
											new_ts_ns,
											&new_R_body_to_local,
											&new_T_body_wrt_local);

	// if we failed to get a transform from the ringbuf, attitude data is not
	// coming from px4
	if(ret1 < 0 || ret2){
		// fprintf(stderr, "failed to find px4 attitude\n");
		rc_matrix_identity(&past_R_body_to_local, 3);
		memset(past_T_body_wrt_local.d, 0, 3*sizeof(double));
		rc_matrix_identity(&new_R_body_to_local, 3);
		memset(new_T_body_wrt_local.d, 0, 3*sizeof(double));
		ret = -2;
	}


	// also get inverse
	geometry_invert_tf(new_R_body_to_local, new_T_body_wrt_local,
					   &new_R_local_to_body, &new_T_local_wrt_body);

	// If we got here then px4 attitude data is valid we we have good interpolated data from
	// the ringbuffer. Use this data to do proper angle and translation
	// compensation since the depth-from-stereo data may be quite delayed.

	// find the rotation from body to body_level by looking at the cross product
	// of a known down vector with the vector pointing out the bottom of the drone
	// start with a reference downward facing vector
	RC_VECTOR_ON_STACK(vec_down, 3);
	vec_down.d[0] = 0;
	vec_down.d[1] = 0;
	vec_down.d[2] = 1.0;

	// now find the vector pointing out bottom of body frame
	RC_VECTOR_ON_STACK(vec_out_bottom, 3);
	rc_matrix_times_col_vec(new_R_local_to_body, vec_down, &vec_out_bottom);

	// check for flip
	if (vec_out_bottom.d[2] < 0.0)
	{
		fprintf(stderr, "ERROR in %s, drone upside down\n", __FUNCTION__);
		return -4;
	}

	// find cross product, no need to normalize as we started with unit vectors
	RC_VECTOR_ON_STACK(cross, 3);
	rc_vector_cross_product(vec_out_bottom, vec_down, &cross);
	// rc_vector_print(vec_out_bottom);
	// rc_vector_print(vec_down);
	// rc_vector_print(cross);


	// find angle of rotation
	// a x b = ||a|| * ||b|| * sin(a) * n where n is a unit vector in direction
	// off the cross product
	// we already normalized a and b so ||a x b||=1
	double cross_magnitude = rc_vector_norm(cross, 2.0);
	double angle = asin(cross_magnitude);
	// printf("angle: %f\n", angle);

	static rc_matrix_t new_R_body_to_level = RC_MATRIX_INITIALIZER;
	if(fabs(angle)<0.001){
		rc_matrix_identity(&new_R_body_to_level, 3);
	}
	else if(rc_axis_angle_to_rotation_matrix(cross, angle, &new_R_body_to_level)){
		fprintf(stderr, "ERROR in %s axis_angle_to_rotation_matrix\n", __FUNCTION__);
		return -5;
	}

	// find rotation R_frame_to_level
	rc_matrix_multiply(past_R_body_to_local, R_frame_to_body, R);
	rc_matrix_left_multiply_inplace(new_R_local_to_body, R);
	rc_matrix_left_multiply_inplace(new_R_body_to_level, R);

	// find translation
	rc_matrix_times_col_vec(past_R_body_to_local, T_frame_wrt_body, T);
	rc_matrix_times_col_vec_inplace(new_R_local_to_body, T);
	rc_matrix_times_col_vec_inplace(new_R_body_to_level, T);

	return ret;
}

int geometry_get_RT_fixed_to_level_at_time(int64_t ts_ns,
										   rc_matrix_t *R,
										   rc_vector_t *T)
{
	// Setup the transform matrices
	static rc_matrix_t R_body_to_local_at_t = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(T_body_wrt_local_at_t, 3);
	static rc_matrix_t R_body_to_fixed_at_t = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(T_body_wrt_fixed_at_t, 3);
	static rc_matrix_t R_fixed_to_body_at_t = RC_MATRIX_INITIALIZER;
	RC_VECTOR_ON_STACK(T_fixed_wrt_body_at_t, 3);

	pthread_mutex_lock(&tf_mutex);

	int ret = rc_tf_ringbuf_get_tf_at_time(&body_local_ringbuf,
										   ts_ns,
										   &R_body_to_local_at_t,
										   &T_body_wrt_local_at_t);

	// This would occur if VIO is not present. This function was written for use in the trajectory monitor
	// and we should never run trajectories without VIO running
	if (ret < 0)
	{
		pthread_mutex_unlock(&tf_mutex);
		return -1;
	}

	// Get rotation of body to fixed
	rc_matrix_left_multiply_inplace(R_local_to_fixed, &R_body_to_local_at_t);
	rc_matrix_multiply(R_local_to_fixed, R_body_to_local_at_t, &R_body_to_fixed_at_t);

	// Get translation of body wrt to fixed
	rc_matrix_times_col_vec(R_local_to_fixed, T_body_wrt_local_at_t, &T_body_wrt_fixed_at_t);

	// Get the inverted transforms
	geometry_invert_tf(R_body_to_fixed_at_t, T_body_wrt_fixed_at_t, &R_fixed_to_body_at_t, &T_fixed_wrt_body_at_t);

	// find the rotation from body to body_level by looking at the cross product
	// of a known down vector with the vector pointing out the bottom of the drone
	// start with a reference downward facing vector
	RC_VECTOR_ON_STACK(vec_down, 3);
	vec_down.d[0] = 0;
	vec_down.d[1] = 0;
	vec_down.d[2] = 1.0;

	// now find the vector pointing out bottom of body frame
	RC_VECTOR_ON_STACK(vec_out_bottom, 3);
	rc_matrix_times_col_vec(R_fixed_to_body_at_t, vec_down, &vec_out_bottom);

	// check for flip
	if (vec_out_bottom.d[2] < 0.0)
	{
		fprintf(stderr, "WARNING in %s, drone upside down\n", __FUNCTION__);
		pthread_mutex_unlock(&tf_mutex);
		return -2;
	}

	// find cross product, no need to normalize as we started with unit vectors
	RC_VECTOR_ON_STACK(cross, 3);
	rc_vector_cross_product(vec_out_bottom, vec_down, &cross);

	// find angle of rotation
	// a x b = ||a|| * ||b|| * sin(a) * n where n is a unit vector in direction
	// off the cross product
	// we already normalized a and b so ||a x b||=1
	double cross_magnitude = rc_vector_norm(cross, 2.0);
	double angle = asin(cross_magnitude);

	static rc_matrix_t R_body_to_level_at_t = RC_MATRIX_INITIALIZER;
	if(fabs(angle)<0.001){
		rc_matrix_identity(&R_body_to_level_at_t, 3);
	}
	else if(rc_axis_angle_to_rotation_matrix(cross, angle, &R_body_to_level_at_t))
	{
		fprintf(stderr, "ERROR in %s axis_angle_to_rotation_matrix\n", __FUNCTION__);
		pthread_mutex_unlock(&tf_mutex);
		return -1;
	}

	// find rotation from fixed to level
	rc_matrix_multiply(R_body_to_level_at_t, R_fixed_to_body_at_t, R);

	// find translation
	rc_matrix_times_col_vec(R_body_to_level_at_t, T_fixed_wrt_body_at_t, T);

	pthread_mutex_unlock(&tf_mutex);
	return 0;
}
