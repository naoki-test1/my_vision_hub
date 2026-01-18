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
#include <unistd.h>	// for usleep()
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include <modalcv/point_cloud.h>
#include <modalcv/common.h>

#include "obs_pc_filter.h"
#include "macros.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "misc.h"

#define POINT_BYTES (3*sizeof(float))

static int en_debug = 0;
static int en_timing = 0;


void obs_pc_filter_en_debug(int en)
{
	en_debug = en;
	mcv_en_debug_prints(en);
}


void obs_pc_filter_en_timing(int en)
{
	en_timing = en;
	mcv_en_timing_prints(en);
}




int obs_pc_ringbuf_alloc(obs_pc_ringbuf_t* buf, int size)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(size<2)){
		fprintf(stderr,"ERROR in %s, size must be >=2\n", __FUNCTION__);
		return -1;
	}

	// if it's already allocated, nothing to do
	if(buf->initialized && buf->size==size && buf->d!=NULL) return 0;

	// make sure it's zero'd out
	buf->size = 0;
	buf->index = 0;
	buf->items_in_buf=0;
	buf->initialized = 0;

	// allocate mem for array
	buf->d = (mcv_pc_t*)malloc(size*sizeof(mcv_pc_t));
	if(buf->d==NULL){
		fprintf(stderr,"ERROR in %s, failed to allocate memory\n", __FUNCTION__);
		return -1;
	}
	for(int i=0;i<size;i++) buf->d[i] = mcv_pc_empty();

	// write out other details
	buf->size = size;
	buf->initialized = 1;
	return 0;
}


int obs_pc_ringbuf_free(obs_pc_ringbuf_t* buf)
{
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(!buf->initialized){
		return 0;
	}

	// free each pc
	for(int i=0;i<buf->size;i++){
		mcv_pc_free(&buf->d[i]);
	}
	// put buffer back to default
	*buf = obs_pc_ringbuf_empty();

	return 0;
}


obs_pc_ringbuf_t obs_pc_ringbuf_empty(void)
{
	obs_pc_ringbuf_t new = OBS_PC_RINGBUF_INITIALIZER;
	return new;
}



int obs_pc_ringbuf_insert_pc(obs_pc_ringbuf_t* buf, mcv_pc_t* new)
{
	int new_index;

	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(new==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}

	// got an empty or uninitialized point cloud, just return
	if(new->initialized==0 || new->n <= 0){
		return 0;
	}

	// we are about to interact with the ringbuf, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// more sanity checks
	if(unlikely(!buf->initialized)){
		pthread_mutex_unlock(&buf->mutex);
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// if this is the first thing to be entered make sure to start at zero
	if(buf->items_in_buf==0){
		new_index = 0;
	}
	else{
		// increment index and check for loop-around
		new_index = buf->index+1;
		if(new_index >= buf->size) new_index = 0;
	}

	// check if we need to flush out an old point cloud, then copy new data in
	if(buf->items_in_buf == buf->size){
		//printf("free one full\n");
		mcv_pc_free(&buf->d[new_index]);
	}
	buf->d[new_index] = *new;

	// bump index and increment number of items if necessary
	buf->index = new_index;
	if(buf->items_in_buf < buf->size){
		buf->items_in_buf++;
	}
	buf->valid_items++;

	// all done, save the timestamp and unlock mutex
	if(new->ts_ns > buf->latest_ts){
		buf->latest_ts = new->ts_ns;
	}
	pthread_mutex_unlock(&buf->mutex);

	// we copied over the mcv_pc_t struct including the pointer to its dynamically
	// allocated memory. Wipe the old copy of this struct now that we have control
	// over it so the user doesn't wipe the memory accidentally.
	*new = mcv_pc_empty();

	return 0;
}



int obs_pc_ringbuf_get_pc_at_pos(obs_pc_ringbuf_t* buf, int position, mcv_pc_t* result)
{
	// sanity checks
	if(unlikely(buf==NULL || result==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(position<0)){
		fprintf(stderr,"ERROR in %s, position must be >= 0\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// about to start reading the buffer, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// silently return if user requested a position beyond buffer size
	if(position >= buf->size){
		pthread_mutex_unlock(&buf->mutex);
		return -3;
	}
	// silently return if user requested an item that hasn't been added yet
	if(position >= buf->items_in_buf){
		pthread_mutex_unlock(&buf->mutex);
		return -2;
	}

	// return index is just latest index minus position due to the order we keep
	// data (populated from left to right)
	int return_index = buf->index - position;

	// check for looparound
	if(return_index < 0){
		return_index += buf->size;
	}

	// write out data
	*result = buf->d[return_index];

	// all done, unlock mutex
	pthread_mutex_unlock(&buf->mutex);

	return 0;
}


int obs_pc_ringbuf_cleanup_old(obs_pc_ringbuf_t* buf, int64_t ts_ns)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in %s, ringbuf uninitialized\n", __FUNCTION__);
		return -1;
	}

	// about to start reading the buffer, lock the mutex
	pthread_mutex_lock(&buf->mutex);

	// scan through all
	for(int i=0;i<buf->size;i++){

		// free the old ones until we find a new ish one
		if(buf->d[i].ts_ns<ts_ns && buf->d[i].initialized){
			//printf("free one old\n");
			mcv_pc_free(&buf->d[i]);
			buf->valid_items--;
		}
		// keep going through all since they may be out of order
	}

	// all done, unlock mutex
	pthread_mutex_unlock(&buf->mutex);
	return 0;
}












int tan_bin_table_init(tan_bin_table_t* t, int n_bins)
{
	// sanity checks
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(n_bins%2 || n_bins<4)){
		fprintf(stderr,"ERROR in %s, bins must be even and >=4\n", __FUNCTION__);
		return -1;
	}

	// table needs to be large enough for the right half plus the 2 limits
	// on either size
	t->table_size = (n_bins/2)+2;
	t->d = (float*)malloc(t->table_size*sizeof(float));
	t->deg_per_bin = 360.0f / n_bins;
	t->n_bins = n_bins;
	float rad_per_bin = TWO_PI / n_bins;

	// convenience value
	float half_bin_rad = PI / n_bins;

	// first and last values are +- inf
	t->d[0]               =  1.0f / 0.0f;
	t->d[t->table_size-1] = -1.0f / 0.0f;

	for(int i=1;i<(t->table_size-1);i++){
		float lower_bound_of_bin_rad = (i*rad_per_bin) - half_bin_rad - (float)PI_2;
		t->d[i] = tan(-lower_bound_of_bin_rad);
		if(en_debug){
			printf("for bin %d, lower bound angle is %5.1fdeg, tan is %5.2f\n", \
					i, (double)lower_bound_of_bin_rad*RAD_TO_DEG, (double)t->d[i]);
		}
	}

	t->initialized = 1;
	return 0;
}


int tan_bin_table_free(tan_bin_table_t* t)
{
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(!t->initialized){
		return 0;
	}

	free(t->d);
	// put buffer back to default
	*t = tan_bin_table_empty();

	return 0;
}


tan_bin_table_t tan_bin_table_empty(void)
{
	tan_bin_table_t new = TAN_BIN_TABLE_INITIALIZER;
	return new;
}


int tan_bin_table_xy_to_bin(tan_bin_table_t* t, float x, float y)
{
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!t->initialized)){
		fprintf(stderr,"ERROR in %s, table_unitialized\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(x==0.0f && y==0.0f)){
		fprintf(stderr,"ERROR in %s, received 0,0 point at origin\n", __FUNCTION__);
		return -1;
	}

	// check for easy forward/back conditions
	if(y==0.0f){
		if(x>0.0f) return 0;
		else return t->n_bins/2;
	}

	// xy in body frame: X is forward, Y is to the right
	// the map only covers the right half of the drone, so if y is negative,
	// mirror it over
	int is_left = 0;
	if(y<0){
		is_left = 1;
		y*=-1.0f;
	}

	// now binary search through the table until we find the right bin
	// use a safety counter in case the table is malformed so we don't get stuck
	int ctr = 0;

	// tangent is x/y. This can be +- inf which is fine
	float goal = x/y;
	int first = 0;
	int last  = t->table_size - 2;
	int middle = (first+last)/2;

	// note the table is monotonically DECREASING
	while(1){

		// see if we need to go up the table
		if(t->d[middle] < goal){
			last = middle - 1;
		}
		// see if we found it
		else if(t->d[middle]>=goal && t->d[middle+1]<=goal){
			break;
		}
		// otherwise move towards the beginning of the table
		else{
			first = middle + 1;
		}
		middle = (first + last)/2;

		// safety check
		ctr++;
		if(unlikely(ctr >= t->table_size)){
			fprintf(stderr,"ERROR in %s, binary search failed for x=%f y=%f\n", __FUNCTION__, (double)x, (double)y);
			return -1;
		}
	}

	// if we were on the right side, all done
	if(!is_left) return middle;

	// for left side points, go back from the end with wraparound
	int ret = t->n_bins-middle;
	if(ret>(t->n_bins-1)) ret = 0;
	return ret;
}


int tan_bin_table_test(int n_bins)
{
	int i;
	tan_bin_table_t t;

	tan_bin_table_init(&t, n_bins);

	printf("n_bins: %2d deg per bin: %4.1f table size: %2d\n", t.n_bins, (double)t.deg_per_bin, t.table_size);
	for(i=0;i<t.table_size; i++){
		printf("i: %2d %5.2f\n", i, (double)t.d[i]);
	}

	int counter[100];
	memset(counter, 0, 100*sizeof(int));

	for(i=0;i<360; i++){

		// this is the angle to the right of forward
		float angle_rad = i*DEG_TO_RAD;
		float y = sin(angle_rad);
		float x = cos(angle_rad);
		int bin = tan_bin_table_xy_to_bin(&t, x,y);
		printf("deg %3d x %5.2f y %5.2f bin %2d\n", i, (double)x, (double)y, bin);
		counter[bin]++;
	}

	for(i=0;i<t.n_bins;i++){
		printf("bin %2d pts %3d\n", i, counter[i]);
	}


	tan_bin_table_free(&t);
	return 0;
}



int distance_bin_table_init(distance_bin_table_t* t, int n_bins, float dist_max)
{
	// sanity checks
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(n_bins<2)){
		fprintf(stderr,"ERROR in %s, bins must >=2\n", __FUNCTION__);
		return -1;
	}

	t->d    = (float*)malloc((n_bins+1)*sizeof(float));
	t->d_sq = (float*)malloc((n_bins+1)*sizeof(float));
	t->dist_max = dist_max;
	t->n_bins = n_bins;
	t->m_per_bin = dist_max/(n_bins-1);

	t->d[0]    = 0.0f;
	t->d_sq[0] = 0.0f;
	for(int i=1;i<n_bins;i++){
		t->d[i]    = (i * t->m_per_bin);
		t->d_sq[i] = t->d[i] * t->d[i];
	}

	t->initialized = 1;
	return 0;
}


int distance_bin_table_free(distance_bin_table_t* t)
{
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(!t->initialized){
		return 0;
	}

	free(t->d);
	free(t->d_sq);
	// put buffer back to default
	*t = distance_bin_table_empty();

	return 0;
}


distance_bin_table_t distance_bin_table_empty(void)
{
	distance_bin_table_t new = DISTANCE_BIN_TABLE_INITIALIZER;
	return new;
}


int distance_bin_table_xy_to_bin(distance_bin_table_t* t, float x, float y)
{
	float dist_squared = (x*x) + (y*y);
	return distance_bin_table_dist_squared_to_bin(t, dist_squared);
}


int distance_bin_table_dist_squared_to_bin(distance_bin_table_t* t, float dist_squared)
{
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!t->initialized)){
		fprintf(stderr,"ERROR in %s, table_unitialized\n", __FUNCTION__);
		return -1;
	}

	// check for anything out of bounds
	if(dist_squared >= t->d_sq[t->n_bins-1]) return t->n_bins-1;


	// now binary search through the table until we find the right bin
	// use a safety counter in case the table is malformed so we don't get stuck
	int ctr = 0;

	int first  = 0;
	int last   = t->n_bins-1;
	int middle = (first+last)/2;


	// note the table is monotonically DECREASING
	while(1){

		// see if we need to go up the table
		if(t->d_sq[middle+1] < dist_squared){
			first = middle + 1;
		}
		// see if we found it
		else if(t->d_sq[middle]<=dist_squared && t->d_sq[middle+1]>=dist_squared){
			break;
		}
		// otherwise move towards the beginning of the table
		else{
			last = middle - 1;
		}
		middle = (first + last)/2;

		// safety check
		ctr++;
		if(unlikely(ctr >= t->n_bins)){
			fprintf(stderr,"ERROR in %s, binary search failed for dist_squared: %6.2f\n", __FUNCTION__, (double)dist_squared);
			return -1;
		}
	}

	return middle;
}


float distance_bin_to_distance(distance_bin_table_t* t, int bin)
{
	if(unlikely(t==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1.0f;
	}
	if(unlikely(!t->initialized)){
		fprintf(stderr,"ERROR in %s, table_unitialized\n", __FUNCTION__);
		return -1.0f;
	}
	if(unlikely(bin<0 || bin>=t->n_bins)){
		fprintf(stderr,"ERROR in %s, bin out of bounds: %d\n", __FUNCTION__, bin);
		return -1.0f;
	}
	return t->d[bin];
}


int distance_bin_table_test(int n_bins, float dist_max)
{
	int i;
	distance_bin_table_t t;

	distance_bin_table_init(&t, n_bins, dist_max);

	printf("n_bins: %2d dist_max %4.1f\n", t.n_bins, (double)dist_max);


	// print the bins
	for(i=0;i<(n_bins-1); i++){
		printf("bin %2d  from %6.2f to %6.2f\n", i, (double)t.d[i], (double)t.d[i+1]);
	}
	printf("bin %2d  everything >%6.2f\n", n_bins-1, (double)t.d[n_bins-1]);



	int counter[n_bins];
	memset(counter, 0, n_bins*sizeof(int));

	for(i=0;i<((n_bins+1)*4); i++){

		float dist = (i/4.0f)*(dist_max/n_bins);
		float y = sqrtf(2.0f)*dist/2.0f;
		float x = sqrtf(2.0f)*dist/2.0f;
		int bin = distance_bin_table_xy_to_bin(&t, x,y);

		printf("dist %5.2f x %5.2f y %5.2f bin %2d\n", (double)dist, (double)x, (double)y, bin);
		if(bin>=0){
			counter[bin]++;
		}
	}

	// print binning final results
	for(i=0;i<n_bins;i++){
		printf("bin %2d pts %3d\n", i, counter[i]);
	}

	distance_bin_table_free(&t);
	return 0;
}





#define EXTRA_BINS_TO_CHECK 4

int pie_binner_init(pie_binner_t* b)
{
	// sanity checks
	if(unlikely(b==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(b->n_angle_bins<4 || b->n_distance_bins<4)){
		fprintf(stderr,"ERROR in %s, must set bin fields first\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(b->z_max <= b->z_min)){
		fprintf(stderr,"ERROR in %s, z_max must be greater than z_min\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(b->pie_threshold<1)){
		fprintf(stderr,"ERROR in %s, pie threshold must be >=1\n", __FUNCTION__);
		return -1;
	}

	pie_binner_free(b);

	if(obs_pc_ringbuf_alloc(&b->ringbuf, b->ringbuf_size)){
		fprintf(stderr,"ERROR in %s, failed to initialize ringbuf\n", __FUNCTION__);
		return -1;
	}
	if(tan_bin_table_init(&b->tan_table, b->n_angle_bins)){
		fprintf(stderr,"ERROR in %s, failed to initialize tan bin table\n", __FUNCTION__);
		return -1;
	}
	if(distance_bin_table_init(&b->dist_table, b->n_distance_bins, b->dist_max)){
		fprintf(stderr,"ERROR in %s, failed to initialize distance bin table\n", __FUNCTION__);
		return -1;
	}

	// set up remaining values in the binner struct
	b->rows = b->n_angle_bins;
	b->cols = b->n_distance_bins + EXTRA_BINS_TO_CHECK; // extra distance to make filtering easier later
	b->dist_min_squared = b->dist_min * b->dist_min;
	b->under_ratio = b->z_max / b->under_trim_dist;

	// shortcuts for code readability
	int rows = b->rows;
	int cols = b->cols;

	// allocate contiguous memory for the major(row) pointers
	b->map_points = (int**)malloc(rows*sizeof(int*));
	b->map_dist = (float**)malloc(rows*sizeof(float*));
	if(unlikely(b->map_points==NULL || b->map_dist==NULL)){
		perror("ERROR in pie_binner_init");
		fprintf(stderr, "tried allocating a %dx%d matrix\n", rows,cols);
		return -1;
	}

	// allocate contiguous memory for the actual data
	void* ptr1 = calloc(rows*cols, sizeof(int));
	void* ptr2 = calloc(rows*cols, sizeof(float));
	if(unlikely(ptr1==NULL || ptr2==NULL)){
		perror("ERROR in pie_binner_init");
		fprintf(stderr, "tried allocating a %dx%d matrix\n", rows,cols);
		free(b->map_points);
		free(b->map_dist);
		return -1;
	}

	// manually fill in the pointer to each row
	for(int i=0;i<rows;i++){
		b->map_points[i] =(int*)   (((char*)ptr1) + (i*cols*sizeof(int)));
		b->map_dist[i]   =(float*) (((char*)ptr2) + (i*cols*sizeof(float)));
	}

	b->initialized = 1;
	return 0;
}




int pie_binner_free(pie_binner_t* b)
{
	if(unlikely(b==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(!b->initialized){
		return 0;
	}

	obs_pc_ringbuf_free(&b->ringbuf);
	tan_bin_table_free(&b->tan_table);
	distance_bin_table_free(&b->dist_table);

	// free memory allocated for the data then the major array
	if(b->map_points != NULL) free(b->map_points[0]);
	free(b->map_points);
	b->map_points = NULL;
	if(b->map_dist != NULL) free(b->map_dist[0]);
	free(b->map_dist);
	b->map_dist = NULL;

	// leave config fields alone!!

	return 0;

}


pie_binner_t pie_binner_empty(void)
{
	pie_binner_t new = PIE_BINNER_INITIALIZER;
	return new;
}


int pie_binner_zero_out(pie_binner_t* b)
{
	if(unlikely(b==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!b->initialized)){
		fprintf(stderr,"ERROR in %s, binner not initialized\n", __FUNCTION__);
		return -1;
	}

	memset(b->map_points[0], 0, b->rows*b->cols*(sizeof(int)));
	memset(b->map_dist[0],   0, b->rows*b->cols*(sizeof(float)));
	return 0;
}


int pie_binner_add_point(pie_binner_t* b, float* p)
{
	if(unlikely(b==NULL || p==NULL)){
		fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!b->initialized)){
		fprintf(stderr,"ERROR in %s, binner not initialized\n", __FUNCTION__);
		return -1;
	}

	float x = p[0];
	float y = p[1];
	float z = p[2];

	// throw out points too high/low
	if(z<b->z_min || z>b->z_max) return 0;

	// find angular bin
	int angle_bin = tan_bin_table_xy_to_bin(&b->tan_table, x, y);
	if(angle_bin<0) return -1;

	// find distance bin
	float dist_squared = x*x + y*y;
	if(dist_squared < b->dist_min_squared) return 0;
	int dist_bin = distance_bin_table_dist_squared_to_bin(&b->dist_table, dist_squared);
	if(dist_bin<0) return -1;
	float dist = sqrt(dist_squared);

	// check for points under the drone
	if(dist < b->under_trim_dist){
		float cut = (dist*b->under_ratio)-0.07f; // lower bubble 7cm below drone COG
		if(z>cut) return 0;
	}

	// keep a running average of the distance in this bin
	b->map_points[angle_bin][dist_bin]++;
	b->map_dist[angle_bin][dist_bin] += dist;

	// if(en_debug){
	// 	printf("xyz: %5.1f %5.1f %5.1f binned to: ", (double)x, (double)y, (double)z);
	// 	printf("anglebin:%2d distbin:%2d dist: %5.1fm\n", angle_bin, dist_bin, (double)dist);
	// }

	return 0;
}


int pie_binner_filter_to_mavlink_msg(pie_binner_t* b, mavlink_message_t* msg)
{
	int i,j;

	// final distance data for px4
	uint16_t distances[72];
	uint16_t dist_max_cm = b->dist_table.dist_max * 100.0f;

	// now set the bins to max_distance +1 means no obstacle is present.
	for(i=0;i<72;i++){
		distances[i] = dist_max_cm + 1;
	}

	if(b->ringbuf.items_in_buf>0){
		// scan bins for any that meet the criteria
		// for each angle....
		for(i=0; i<b->rows; i++){

			// scan the distances from inside out, remember we have one extra column
			// than we do distance bins
			for(j=0; j<(b->cols-EXTRA_BINS_TO_CHECK); j++){

				// skip empty bins
				if(b->map_points[i][j] <= 0) continue;

				// sum up points in this bin, the two on either side
				// and the three behind it
				int sum = 0;
				for(int k=-1; k<=1; k++){
					// row (angle) to inspect with wraparound check
					int ii = i-k;
					if(ii<0) ii+=b->rows;
					else if(ii>=b->rows) ii-=b->rows;

					// sum points in this bin plus extras
					for(int m=0; m<=EXTRA_BINS_TO_CHECK; m++){
						sum += b->map_points[ii][j+m];
					}
				}

				// check the sum of points in all the neighbors
				if(sum >= b->pie_threshold){
					float dist_m = b->map_dist[i][j] / b->map_points[i][j];
					// if(dist_m<0.1f) continue; // error check, not needed?
					uint16_t dist_cm = dist_m * 100.0f;
					distances[i] = dist_cm;
					//printf("added a distance! i: %d j:%d dist: %5.1fm\n", i, j, (double)dist_m);
					break; // move onto next angle
				}
			}
		}
	}


	// TODO investigate if min distance changes px4 behavior or not
	#define MIN_DISTANCE_CM 0

	// set other params for mavlink message
	uint64_t time_usec = 0;
	uint8_t increment_int = 0; // use the float field instead
	float increment_f = 360.0f / b->tan_table.n_bins;
	float angle_offset = 0.0f; // 0 means bin 0 is straight ahead along X

	mavlink_msg_obstacle_distance_pack(autopilot_monitor_get_sysid(), VOXL_COMPID,\
		msg, time_usec, MAV_DISTANCE_SENSOR_UNKNOWN, distances, increment_int,\
		MIN_DISTANCE_CM, dist_max_cm, increment_f, angle_offset, MAV_FRAME_BODY_FRD);

	return 0;
}


