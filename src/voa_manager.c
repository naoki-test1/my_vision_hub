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
#include <modal_pipe_client.h>
#include <modal_pipe_server.h>
#include <rc_math.h>
#include <modalcv/point_cloud.h>
#include <modalcv/common.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

#include "voa_manager.h"
#include "vio_manager.h"
#include "obs_pc_filter.h"
#include "pipe_channels.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "autopilot_monitor.h"
#include "config_file.h"
#include "macros.h"
#include "geometry.h"
#include "misc.h"
#include "voxl_rangefinder_interface.h"


// big read buffer for point clouds, auto-expanded by helper if needed
#define PIPE_READ_BUF_SIZE	(12*320*240)

// local variables
static int running = 0; // local running flag separate from main_running
static int en_linescan = 0;
static int en_timing = 0;
static int en_debug = 0;
static pthread_t sender_thread_id;
static pie_binner_t pie_binner	= PIE_BINNER_INITIALIZER;
static rc_filter_t delay_filter	= RC_FILTER_INITIALIZER;
static mcv_pc_t combined_pc = MCV_PC_INITIALIZER;



void voa_manager_en_print_linescan(int en)
{
	en_linescan = en;
	return;
}


void voa_manager_en_timing(int en)
{
	en_timing = en;
	obs_pc_filter_en_timing(en);
	return;
}

void voa_manager_en_debug(int en)
{
	en_debug = en;
	obs_pc_filter_en_debug(en);
	mcv_en_debug_prints(en);
	return;
}

static int _voa_input_idx_to_mpa_ch(int i)
{
	int ch = VOA_CLIENT_CH_START+i;
	return ch;
}

static int _mpa_ch_to_voa_input_idx(int ch)
{
	int i = ch - VOA_CLIENT_CH_START;
	return i;
}

static mcv_pc_downsample2_config _get_downsample_config(int ch)
{
	int i = _mpa_ch_to_voa_input_idx(ch);
	voa_input_t* v = &voa_inputs[i];
	mcv_pc_downsample2_config config;

	config.max_depth    = v->max_depth;
	config.min_depth    = v->min_depth;
	config.cell_size    = v->cell_size;
	config.threshold    = v->threshold;
	config.x_fov_deg    = v->x_fov_deg;
	config.y_fov_deg    = v->y_fov_deg;
	config.conf_cutoff  = v->conf_cutoff;

	return config;
}


// thread to send data to PX4 asynchronously from the sensor data when
// Its job is to align the whole rinbuf of sensor data given
// the previous history of VIO data into one merged point cloud, then filter
// that and send it into PX4 after filtering.
// If VIO is not running then just use the last sensor sample.
static void* _sender_thread_func(__attribute__((unused)) void* arg)
{
	int i,j,p,ret;
	uint32_t n;
	int has_warned_no_attitude = 0;
	static int64_t hb_last_time = 0;

	while(running){

		// try to maintain constant loop rate
		static int64_t next_time = 0;
		if(my_loop_sleep(voa_send_rate_hz, &next_time)){
			fprintf(stderr, "WARNING VOA sender thread fell behind\n");
		}

		// lock pie binner mutex since other threads will be inserting data in
		pthread_mutex_lock(&pie_binner.mutex);

		// cleanup old ringbuf data
		obs_pc_ringbuf_cleanup_old(&pie_binner.ringbuf, my_time_monotonic_ns() - (int64_t)(1000000000.0f*voa_memory_s));

		// make sure we clean out the pie binner before adding fresh ones in
		pie_binner_zero_out(&pie_binner);
		int64_t t_start = my_time_monotonic_ns();

		// based on previous measurements, predict a time in the future when we
		// we predict we will finish the calculations
		double filtered_delay_s = delay_filter.newest_output;
		int64_t new_ts_ns = t_start + (int64_t)(filtered_delay_s*1000000000.0);

		// combined point cloud for debug pipe
		combined_pc.n = 0; // reset counter to 0 for this pass
		point_cloud_metadata_t meta;
		meta.timestamp_ns = new_ts_ns;
		int send_combined_pc = 0;
		if(pipe_server_get_num_clients(VOA_PC_OUT_CH)>0 || en_debug){
			send_combined_pc = 1;
		}

		// for every point cloud in our buffer, rotate and translate into the new
		// level-body frame of the drone, then add to the pie binner
		for(p=0; p<pie_binner.ringbuf.items_in_buf; p++){

			// stop integrating if we've reached the limit
			if(p>=voa_max_pc_per_fusion) break;

			// start from the newest point cloud at position 0 since we will
			// stop after that first one if no VIO is available
			mcv_pc_t pc;
			if(obs_pc_ringbuf_get_pc_at_pos(&pie_binner.ringbuf, p, &pc)) continue;
			if(!pc.initialized) continue; // skip empty point clouds

			if(en_debug){
				printf("VOA starting to process point cloud #%2d of %2d\n", p+1, pie_binner.ringbuf.valid_items);
				printf("pc timestamp is %5.2fs older than now\n", (double)(new_ts_ns-pc.ts_ns)/1000000000.0);
			}

			static rc_vector_t T_frame_wrt_level = RC_VECTOR_INITIALIZER;
			static rc_matrix_t R_frame_to_level  = RC_MATRIX_INITIALIZER;

			// if VIO is enabled, get the transform from vio rinbuffers
			if(vio_manager_is_active()){
				if(en_debug) printf("VOA using VIO attitude and position correction\n");
				ret = geometry_get_RT_frame_in_past_to_level(pc.frame, pc.ts_ns,\
							new_ts_ns, &R_frame_to_level, &T_frame_wrt_level);
			}
			else{
				if(en_debug) printf("VOA using PX4 attitude correction\n");
				ret = geometry_get_RT_frame_in_past_to_level_from_px4_attitude(\
					pc.frame, pc.ts_ns, new_ts_ns, &R_frame_to_level, &T_frame_wrt_level);

				if(ret == -2 && !has_warned_no_attitude){
					fprintf(stderr, "WARNING in VOA manager, no attitude data from px4\n");
					has_warned_no_attitude = 1;
				}
			}

			if(ret!=0 && ret!=-2){
				if(en_debug){
					fprintf(stderr, "problem in %s getting frame to level ret = %d\n", __FUNCTION__, ret);
				}
				pthread_mutex_unlock(&pie_binner.mutex);
				continue;
			}

			// convert to float for faster math
			float T[3];
			float R[3][3];
			for(i=0;i<3;i++){
				T[i] = (float)T_frame_wrt_level.d[i];
				for(j=0;j<3;j++){
					R[i][j] = (float)R_frame_to_level.d[i][j];
				}
			}

			/*
			// debug transforms
			double roll, pitch, yaw;
			rc_rotation_to_tait_bryan(R_frame_to_level, &roll, &pitch, &yaw);
			printf("T_frame_wrt_level: %5.2f %5.2f %5.2f  RPY: %5.2f %5.2f %5.2f\n", \
			T_frame_wrt_level.d[0], T_frame_wrt_level.d[1], T_frame_wrt_level.d[2],\
			roll*180.0/3.14159, pitch*180.0/3.14159, yaw*180.0/3.14159);
			rc_vector_print(T_frame_wrt_level);
			rc_matrix_print(R_frame_to_level);
			*/

			// now rotate every point
			if(en_debug){
				printf("VOA transforming point cloud\n");
			}

			float new_pc[pc.n*3];
			for(n=0; n<pc.n; n++){

				float old[3];
				float new[3];
				old[0] = pc.d[(n*3)];
				old[1] = pc.d[(n*3)+1];
				old[2] = pc.d[(n*3)+2];

				for(i=0;i<3;i++){
					new[i] = 0.0f;
					for(j=0;j<3;j++){
						new[i] += R[i][j] * old[j];
					}
					new[i] += T[i];
				}
				// copy into our new array
				for(i=0;i<3;i++) new_pc[(n*3)+i]=new[i];
			}

			// add in the rotated point to the pie binner
			if(en_debug) printf("adding points to binner\n");
			for(n=0; n<pc.n; n++){
				pie_binner_add_point(&pie_binner, &new_pc[n*3]);
			}

			// if there is a subscriber to the debug pipe, form a combined pc
			// from each of the indiviual point clouds
			if(send_combined_pc){
				if(en_debug) printf("merging pc\n");
				mcv_pc_add_points(&combined_pc, new_pc, pc.n);
			}

		}

		// send the debug pc, no need to free the combined_and_downsampled pc, it can be reused
		if(send_combined_pc && combined_pc.n>0){

			// downsample since this can get quite large
			mcv_pc_t combined_and_downsampled = MCV_PC_INITIALIZER;
			if(mcv_pc_downsample(combined_pc.d, combined_pc.n, \
									0.15, &combined_and_downsampled)){
				fprintf(stderr, "ERROR, failed to downsample combined VOA point cloud\n");
			}
			else{
				meta.n_points = combined_and_downsampled.n;
				meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;
				if(en_debug){
					printf("downsampled combined PC from %5d to %5d points\n", \
								combined_pc.n, combined_and_downsampled.n);
				}
				pipe_server_write_point_cloud(VOA_PC_OUT_CH, meta, combined_and_downsampled.d);
			}
			// meta.n_points = combined_pc.n;
			// meta.format = POINT_CLOUD_FORMAT_FLOAT_XYZ;
			// pipe_server_write_point_cloud(VOA_PC_OUT_CH, meta, combined_pc.d);
		}

		mavlink_message_t msg;

		if(en_debug){
			printf("VOA filtering bins to mavlink message\n");
		}
		ret = pie_binner_filter_to_mavlink_msg(&pie_binner, &msg);
		pthread_mutex_unlock(&pie_binner.mutex);
		if(ret){
			fprintf(stderr, "pie binner failed\n");
			continue;
		}

		// add this calculation time to the filter
		int64_t t_after = my_time_monotonic_ns();
		double new_delay_s = (t_after-t_start)/1000000000.0;
		rc_filter_march(&delay_filter, new_delay_s);

		if(en_timing){
			printf("VOA time to bin:   %6.2fms filtered: %6.2fms\n", new_delay_s*1000.0, filtered_delay_s*1000.0);
		}

		// send sensor object distance data
		mavlink_io_send_msg_to_ap(&msg);

		// PX4 collision avoidance heartbeat must be sent to enable within
		// a hardcoded 5s window 
		// Let's go with 1s heartbeats
		if (t_after - hb_last_time > 1e9)
		{
			// send heartbeat to enable OBS on PX4
			// this is separate from the usual heartbeat we send from mavlink-server
			mavlink_message_t msg_obs;
			mavlink_msg_heartbeat_pack(\
							autopilot_monitor_get_sysid(), \
							MAV_COMP_ID_OBSTACLE_AVOIDANCE, \
							&msg_obs, \
							MAV_TYPE_ONBOARD_CONTROLLER,\
							MAV_AUTOPILOT_INVALID,\
							0, 0, MAV_STATE_ACTIVE);
			mavlink_io_send_msg_to_ap(&msg_obs);
			hb_last_time = t_after;
		}

		if(en_linescan){
			uint16_t distances[72];
			mavlink_msg_obstacle_distance_get_distances(&msg, distances);
			// i logically going from backwards, clockwise all the way around
			// j is the corresponding index in the distance array
			for(i=0;i<voa_pie_slices;i++){
				j = i - (voa_pie_slices/2);
				if(j<0) j+=voa_pie_slices;
				if(i>=72){
					fprintf(stderr, "logic error in linescan print\n");
					continue;
				}
				printf("slice %2d  dist", i);
				double dist_m = distances[j]/100.0;
				if(dist_m>(double)voa_pie_max_dist_m) printf(" none");
				else printf("%5.2fm", dist_m);
				if(j==0) printf(" forward\n");
				else if (i==0) printf(" back\n");
				else if (i==(voa_pie_slices/4)) printf(" left\n");
				else if (i==(voa_pie_slices*3/4)) printf(" right\n");
				else printf("\n");
			}
			printf("\n");
		}
	}

	printf("exiting VOA thread\n");
	return NULL;
}

// This is the callback assigned to the client pipe interface.
// Multiple packets may come through at once although that's not typical.
//
// TODO: currently the camera and point cloud helpers don't provide a way to
// detect a disconnect, only the simple helper which VIO and Apriltag managers
// use. Add disconnect detection to libmodal_pipe and this in the future
static void _tof_helper_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	int ret;
	int n_packets;

	tof2_data_t* tof_data_ptr = pipe_validate_tof2_data_t(data, bytes, &n_packets);
	if(tof_data_ptr==NULL){
		pipe_client_flush(ch);
		return;
	}

	// tof delay measurement testing, looks good!
	// int midpoint = ((MPA_TOF_WIDTH*MPA_TOF_HEIGHT)/2) + MPA_TOF_WIDTH/2;
	// float dist = tof_data_ptr[0].points[midpoint][2];
	// float vio_xyz[3];
	// geometry_get_T_body_in_past(tof_data_ptr[0].timestamp_ns, vio_xyz);
	// double timestamp_ms = tof_data_ptr[0].timestamp_ns/1000000.0;
	// printf("%0.3f,%0.3f,%0.3f\n", (double)vio_xyz[0], (double)dist, timestamp_ms);

	mcv_pc_t new_pc = MCV_PC_INITIALIZER;

	// lock the binner as multiple sensor callbacks may use it
	pthread_mutex_lock(&pie_binner.mutex);

	// downsample each packet
	for(int i=0; i<n_packets; i++){

		if(en_debug) printf("VOA downsampling new tof point cloud\n");
		new_pc = mcv_pc_empty();
		float* xyz    = (float*)tof_data_ptr[i].points;
		uint8_t* confidence = tof_data_ptr[i].confidences;
		mcv_pc_downsample2_config config = _get_downsample_config(ch);
		int n_pts = tof_data_ptr[i].width * tof_data_ptr[i].height;
		ret = mcv_pc_downsample2(xyz, n_pts, confidence, &config, &new_pc);
		if(ret || new_pc.n<=0) continue;
		new_pc.ts_ns = tof_data_ptr[i].timestamp_ns;
		strcpy(new_pc.frame, voa_inputs[ch-VOA_CLIENT_CH_START].frame);

		// add the filtered point cloud to the ringbuffer this will take control of
		// the allocated memory, we don't need to free it ourselves
		obs_pc_ringbuf_insert_pc(&pie_binner.ringbuf, &new_pc);
	}

	// unlock mutex
	pthread_mutex_unlock(&pie_binner.mutex);

	return;
}

// This is the callback assigned to the client pipe interface.
// Multiple packets may come through at once although that's not typical.
//
// TODO: currently the camera and point cloud helpers don't provide a way to
// detect a disconnect, only the simple helper which VIO and Apriltag managers
// use. Add disconnect detection to libmodal_pipe and this in the future
static void pc_helper_cb(int ch, point_cloud_metadata_t meta, void* data,
									__attribute__((unused)) void* context)
{
	int ret;

	if(meta.format != POINT_CLOUD_FORMAT_FLOAT_XYZ){
		fprintf(stderr, "ERROR in voa_manager, unsupported point cloud format.\n");
		fprintf(stderr, "disconnecting from channel %d\n", ch);
		pipe_client_close(ch);
	}

	// downsample and filter the raw point cloud into a new one
	if(en_debug) printf("VOA downsampling new stereo point cloud\n");
	mcv_pc_t new_pc = MCV_PC_INITIALIZER;
	mcv_pc_downsample2_config config = _get_downsample_config(ch);
	ret = mcv_pc_downsample2((const float*)data, meta.n_points, NULL, &config, &new_pc);
	if(ret || new_pc.n<=0) return;
	new_pc.ts_ns = meta.timestamp_ns;
	strcpy(new_pc.frame, voa_inputs[ch-VOA_CLIENT_CH_START].frame);

	// lock the binner as multiple sensor callbacks may use it
	pthread_mutex_lock(&pie_binner.mutex);

	// add the filtered point cloud to the ringbuffer this will take control of
	// the allocated memory, we don't need to free it ourselves
	obs_pc_ringbuf_insert_pc(&pie_binner.ringbuf, &new_pc);
	pthread_mutex_unlock(&pie_binner.mutex);

	return;
}

// short for rangefinder points per sensors. each rangefinder adds 5 points,
// one in the middle, and 4 for each corner of a square FOV
#define RF_PPS 5

// process one block of rangefinder samples with the same timestamp
static int _process_rangefinders(rangefinder_data_t* data, int n)
{
	static int has_warned_direction_vector = 0;

	// sanity checks
	if(data==NULL){
		fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(n<1){
		fprintf(stderr, "ERROR in %s, number of packets should be >=1\n", __FUNCTION__);
		return -1;
	}

	// validate the timestamps all match and count the number of positive ranges
	int64_t ts = data[0].timestamp_ns;
	int i,j;
	int n_valid = 0;
	for(i=0; i<n; i++){
		if(data[i].timestamp_ns != ts){
			fprintf(stderr, "ERROR in %s, timestamp not consistent\n", __FUNCTION__);
			return -1;
		}
		if(data[i].distance_m>0.0f) n_valid++;
	}

	// no good data, return;
	if(n_valid<1) return 0;

	if(en_debug) printf("VOA processing rangefinder data\n");

	// make new point cloud to put points in
	mcv_pc_t new_pc = MCV_PC_INITIALIZER;

	// This will be freed by the pc ringbuffer later when we are done with it
	if(mcv_pc_alloc(&new_pc, n_valid*RF_PPS)) return -1;

	// unlike DFS and TOF, rangefinders are always in body frame and report
	// their own extrinsics in the pipe data
	static const char* frame_string = "body";
	strcpy(new_pc.frame, frame_string);
	new_pc.ts_ns = ts;

	// some rangefinder readings may report no reading, keep track of the ones
	// to actually project into a point cloud
	int valid_range_ctr = 0;

	// add each rangefinder reading into point cloud
	for(i=0; i<n; i++){

		// skip invalid sensor readings
		if(data[i].distance_m<=0.0f) continue;

		// get direction vector and its length
		float* d = data[i].direction_wrt_body;
		float d_len = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
		if(d_len<0.0001f){
			if(!has_warned_direction_vector){
				fprintf(stderr, "WARNING in %s, rangefinder data has direction vector with 0 length\n", __FUNCTION__);
			}
			has_warned_direction_vector = 1;
			mcv_pc_free(&new_pc);
			return -1;
		}
		RC_VECTOR_ON_STACK(d_norm, 3);
		for(j=0;j<3;j++) d_norm.d[j]=d[j]/d_len;

		// construct 5 points per sensor in sensor reference frame
		double p[3*RF_PPS];
		double dist_z = data[i].distance_m;
		double dist_xy = dist_z * (double)data[i].fov_deg * (3.14/180.0) * 0.5 * 0.707;
		for(j=0;j<RF_PPS;j++) p[(j*3)+2] = dist_z;
		p[(1*3)+0] =  dist_xy;
		p[(1*3)+1] =  dist_xy;
		p[(2*3)+0] = -dist_xy;
		p[(2*3)+1] =  dist_xy;
		p[(3*3)+0] =  dist_xy;
		p[(3*3)+1] = -dist_xy;
		p[(4*3)+0] = -dist_xy;
		p[(4*3)+1] = -dist_xy;

		// find rotation matrix from sensor frame to body frame
		static rc_matrix_t R_sensor_to_body = RC_MATRIX_INITIALIZER;

		// case where sensor already aligns with body frame
		if(d[0]==0.0f && d[1]==0.0f){
			rc_matrix_identity(&R_sensor_to_body, 3);
			if(d[2]<0){
				R_sensor_to_body.d[1][1] = -1.0;
				R_sensor_to_body.d[2][2] = -1.0;
			}
		}
		// all other cases
		else{
			// start with cross product of unit vectors
			RC_VECTOR_ON_STACK(cross, 3);
			RC_VECTOR_ON_STACK(along_z, 3);
			along_z.d[0] = 0.0;
			along_z.d[1] = 0.0;
			along_z.d[2] = 1.0;
			rc_vector_cross_product(along_z, d_norm, &cross);

			// find angle of rotation
			// a x b = ||a|| * ||b|| * sin(a) * n where n is a unit vector in direction
			// off the cross product
			// we already normalized a and b so ||a x b||=1
			double cross_magnitude = rc_vector_norm(cross, 2.0);
			if(cross_magnitude>1.0) cross_magnitude = 1.0; // bound any rounding errors
			if(cross_magnitude<-1.0) cross_magnitude = -1.0;
			double angle = asin(cross_magnitude);

			/*
			printf("i=%d angle = %f magnitude = %f\n", i, angle *180.0/3.14159, cross_magnitude);
			printf("along_z %6.3f %6.3f %6.3f  d_norm %6.3f %6.3f %6.3f cross %6.3f %6.3f %6.3f\n", \
											along_z.d[0], along_z.d[1], along_z.d[2],\
											d_norm.d[0], d_norm.d[1], d_norm.d[2],\
											cross.d[0], cross.d[1], cross.d[2]);
			*/
			if(rc_axis_angle_to_rotation_matrix(cross,angle, &R_sensor_to_body)){
				fprintf(stderr, "ERROR in %s axis_angle_to_rotation_matrix\n", __FUNCTION__);
				mcv_pc_free(&new_pc);
				return -1;
			}
		}

		// rotate each of the 5 points and insert into point cloud
		int base = valid_range_ctr*3*RF_PPS;
		valid_range_ctr++;
		for(j=0;j<RF_PPS;j++){

			RC_VECTOR_ON_STACK(rotated, 3);
			RC_VECTOR_ON_STACK(start, 3);
			start.d[0] = p[(j*3)+0];
			start.d[1] = p[(j*3)+1];
			start.d[2] = p[(j*3)+2];
			rc_matrix_times_col_vec(R_sensor_to_body, start, &rotated);

			for(int k=0;k<3;k++) new_pc.d[base+(j*3)+k] = (float)rotated.d[k] + data[i].location_wrt_body[k];
		}

		/*
		// Simple method with just one point in the middle of each sensor FOV
		// vector from body frame to center of rangefinder FOV
		float v[3];
		float dist = data[i].distance_m / d_len;
		for(j=0;j<3;j++){
			v[j] = data[i].location_wrt_body[j] + (dist * d[j]);
		}

		// insert into point cloud
		for(j=0;j<3;j++){
			new_pc.d[(i*3)+j] = v[j];
		}
		*/
	}

	// lock the binner as multiple sensor callbacks may use it
	pthread_mutex_lock(&pie_binner.mutex);

	// add the filtered point cloud to the ringbuffer this will take control of
	// the allocated memory, we don't need to free it ourselves
	obs_pc_ringbuf_insert_pc(&pie_binner.ringbuf, &new_pc);
	pthread_mutex_unlock(&pie_binner.mutex);

	return 0;
}


// This is the callback assigned to the client pipe interface.
static void _rangefinder_helper_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	int n_packets;

	rangefinder_data_t* rangefinder_data_ptr = voxl_rangefinder_validate_pipe_data(data, bytes, &n_packets);
	if(rangefinder_data_ptr==NULL){
		pipe_client_flush(ch);
		return;
	}

	// data comes in as groups of rangefinder_data_t structs, one for each sensor.
	// however these are often taken at the same time, We may get a bunch of
	// readings at the same timestamp, followed by another bunch at a later timestamp.
	// go through and bundle together all the data at a single timestamp to rotate
	// and process together.
	int64_t current_ts = rangefinder_data_ptr[0].timestamp_ns;
	int n_this_group=1;
	int beginning_of_group=0;

	for(int i=0;i<=n_packets;i++){
		// Found the end of a block of equal timestamps
		if(i==(n_packets-1) || rangefinder_data_ptr[i].timestamp_ns != current_ts){

			// process the block
			_process_rangefinders(&rangefinder_data_ptr[beginning_of_group], n_this_group);

			// if we've not reached the end, reset the counter and start positions
			if(i<(n_packets-1)){
				beginning_of_group=i;
				current_ts=rangefinder_data_ptr[i].timestamp_ns;
				n_this_group=1; // reset counter
			}
		}
		// not at the end of a block, increment counter
		else{
			n_this_group++;
		}

	}

	return;
}


static void _connect_cb(int ch, __attribute__((unused)) void* context)
{
	printf("Connected to VOA input pipe: %s\n", voa_inputs[ch-VOA_CLIENT_CH_START].input_pipe);
	return;
}

static void _disconnect_cb(int ch, __attribute__((unused)) void* context)
{
	printf("Disconnected from pipe: %s\n", voa_inputs[ch-VOA_CLIENT_CH_START].input_pipe);
	return;
}


int voa_manager_init(void)
{
	if(running){
		fprintf(stderr, "ERROR in %s, already running\n", __FUNCTION__);
		return 0;
	}

	/*
	// don't bother running these tests anymore, it was useful in development
	if(en_debug){
		printf("==============================================================\n");
		printf("testing angle binner\n");
		tan_bin_table_test(voa_pie_slices);
		printf("testing distance binner\n");
		distance_bin_table_test(voa_pie_max_dist_m/voa_pie_bin_depth_m, MAX_DIST_M);
		printf("==============================================================\n");
	}
	*/

	// create a pie binner with our config
	pie_binner.ringbuf_size = voa_max_pc_per_fusion + 2; // extra buffer
	pie_binner.n_angle_bins = voa_pie_slices;
	pie_binner.n_distance_bins = voa_pie_max_dist_m/voa_pie_bin_depth_m;
	pie_binner.pie_threshold = voa_pie_threshold;
	pie_binner.z_max = voa_lower_bound_m;
	pie_binner.z_min = voa_upper_bound_m;
	pie_binner.dist_max = voa_pie_max_dist_m;
	pie_binner.dist_min = voa_pie_min_dist_m;
	pie_binner.under_trim_dist = voa_pie_under_trim_m;
	if(pie_binner_init(&pie_binner)){
		fprintf(stderr, "ERROR: Failed to init pie binner, can't start VOA\n");
		return -1;
	}

	// allocate memory for the combined pc
	mcv_pc_alloc(&combined_pc, voa_max_pc_per_fusion);

	// set up filter to monitor the delay in pie-binner processing
	rc_filter_first_order_lowpass(&delay_filter, (1.0/(double)voa_send_rate_hz), 0.5);
	rc_filter_enable_saturation(&delay_filter, 0.0, 0.030);
	rc_filter_prefill_inputs(&delay_filter, 0.015);

	// start the server pipe to output debug point cloud
	pipe_info_t info = { \
		.name        = VOA_PC_OUT_NAME,\
		.location    = VOA_PC_OUT_LOCATION,\
		.type        = "point_cloud_metadata_t",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = 8*1024*1024};

	int flags = 0;
	if(pipe_server_create(VOA_PC_OUT_CH, info, flags)) return -1;

	// open each enabled voa input pipe
	for(int i=0;i<MAX_VOA_INPUTS;i++){
		if(voa_inputs[i].enabled==0) continue;
		int ch = _voa_input_idx_to_mpa_ch(i);
		pipe_client_set_connect_cb(ch,		_connect_cb,	NULL);
		pipe_client_set_disconnect_cb(ch,	_disconnect_cb,	NULL);
		switch(voa_inputs[i].type){
			case VOA_POINT_CLOUD:
				pipe_client_set_point_cloud_helper_cb(ch, pc_helper_cb, NULL);
				pipe_client_open(ch, voa_inputs[i].input_pipe, PIPE_CLIENT_NAME, \
						CLIENT_FLAG_EN_POINT_CLOUD_HELPER, PIPE_READ_BUF_SIZE);
				break;

			case VOA_TOF:
				pipe_client_set_simple_helper_cb(ch, _tof_helper_cb, NULL);
				pipe_client_open(ch, voa_inputs[i].input_pipe, PIPE_CLIENT_NAME, \
						CLIENT_FLAG_EN_SIMPLE_HELPER, TOF2_RECOMMENDED_READ_BUF_SIZE);
				break;

			case VOA_RANGEFINDER:
				pipe_client_set_simple_helper_cb(ch, _rangefinder_helper_cb, NULL);
				pipe_client_open(ch, voa_inputs[i].input_pipe, PIPE_CLIENT_NAME, \
							CLIENT_FLAG_EN_SIMPLE_HELPER, RANGEFINDER_RECOMMENDED_READ_BUF_SIZE);
				break;

			default:
				fprintf(stderr, "ERROR in %s, invalid voa_input_type_t enum: %d\n", __FUNCTION__, (int)voa_inputs[i].type);
				return -1;
		}
	}

	running = 1;
	pthread_create(&sender_thread_id, NULL, _sender_thread_func, NULL);


	//mcv_pc_en_debug(1);


	return 0;
}


void voa_manager_stop(void)
{
	if(running==0) return;
	running = 0;
	for(int i=0;i<MAX_VOA_INPUTS;i++){
		if(voa_inputs[i].enabled==0) continue;
		pipe_client_close(i+VOA_CLIENT_CH_START);
	}
	pthread_join(sender_thread_id, NULL);
	pie_binner_free(&pie_binner);
	rc_filter_free(&delay_filter);
	pipe_server_close(VOA_PC_OUT_CH);
	mcv_pc_free(&combined_pc);
	return;
}
