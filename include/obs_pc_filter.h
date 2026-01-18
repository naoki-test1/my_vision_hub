
#ifndef OBS_PC_FILTER_H
#define OBS_PC_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>
#include <c_library_v2/common/mavlink.h>
#include <modalcv/point_cloud.h>


void obs_pc_filter_en_debug(int en);
void obs_pc_filter_en_timing(int en);


#define COMBINED_PC_MAX_POINTS 100000



/**
 * used to hold the downsampled point clouds from sensors for the last second or
 * so. The ringbuffer can be cleaned out which removes old data. It shouldn't
 * fill up in practice. Point clouds can be added out of order.
 */
typedef struct obs_pc_ringbuf_t {
	mcv_pc_t* d;			///< pointer to dynamically allocated data
	int size;				///< number of elements the buffer can hold
	int index;				///< index of the most recently added value
	int items_in_buf;		///< number of items in the buffer, between 0 and size
	int valid_items;		///< <= items_in_buf, number of populated items
	int initialized;		///< flag indicating if memory has been allocated for the buffer
	int64_t latest_ts;		///< latest timestamp added to buffer
	pthread_mutex_t mutex;	///< mutex to protect the buffer during read/write operations
} obs_pc_ringbuf_t;

#define OBS_PC_RINGBUF_INITIALIZER {\
	.d = NULL,\
	.size = 0,\
	.index = 0,\
	.items_in_buf = 0,\
	.valid_items = 0,\
	.initialized = 0,\
	.latest_ts = 0,\
	.mutex = PTHREAD_MUTEX_INITIALIZER\
}

int obs_pc_ringbuf_alloc(obs_pc_ringbuf_t* buf, int size);
int obs_pc_ringbuf_free(obs_pc_ringbuf_t* buf);
obs_pc_ringbuf_t obs_pc_ringbuf_empty(void);
int obs_pc_ringbuf_insert_pc(obs_pc_ringbuf_t* buf, mcv_pc_t* new);
int obs_pc_ringbuf_get_pc_at_pos(obs_pc_ringbuf_t* buf, int pos, mcv_pc_t* result);
int obs_pc_ringbuf_cleanup_old(obs_pc_ringbuf_t* buf, int64_t ts_ns);





/**
 * Table to speed up the angular part of of binning process
 */
typedef struct tan_bin_table_t {
	int initialized;	///< flag indicating if memory has been allocated for the buffer
	float* d;			///< pointer to dynamically allocated data
	int table_size;
	int n_bins;
	float deg_per_bin;
} tan_bin_table_t;

#define TAN_BIN_TABLE_INITIALIZER {\
	.initialized = 0,\
	.d = NULL,\
	.table_size = 0,\
	.n_bins = 0,\
	.deg_per_bin = 0.0f\
}

int tan_bin_table_init(tan_bin_table_t* t, int n_bins);
int tan_bin_table_free(tan_bin_table_t* t);
tan_bin_table_t tan_bin_table_empty(void);
int tan_bin_table_xy_to_bin(tan_bin_table_t* t, float x, float y);
int tan_bin_table_test(int n_bins);





/**
 * table to speed up the distance portion of the binning process
 */
typedef struct distance_bin_table_t {
	int initialized;	///< flag indicating if memory has been allocated for the buffer
	float* d;			///< pointer to dynamically allocated data
	float* d_sq;			///< pointer to dynamically allocated data
	int table_size;
	int n_bins;
	float m_per_bin;
	float dist_max;
} distance_bin_table_t;



#define DISTANCE_BIN_TABLE_INITIALIZER {\
	.initialized = 0,\
	.d = NULL,\
	.d_sq = NULL,\
	.table_size = 0,\
	.n_bins = 0,\
	.m_per_bin = 0.0f,\
	.dist_max = 0.0f\
}


int distance_bin_table_init(distance_bin_table_t* t, int n_bins, float dist_max);
int distance_bin_table_free(distance_bin_table_t* t);
distance_bin_table_t distance_bin_table_empty(void);
int distance_bin_table_xy_to_bin(distance_bin_table_t* t, float x, float y);
int distance_bin_table_dist_squared_to_bin(distance_bin_table_t* t, float dist_squared);
float distance_bin_to_distance(distance_bin_table_t* t, int bin);
int distance_bin_table_test(int n_bins, float dist_max);




/**
 * Binner takes xyz points and bins them into
 */
typedef struct pie_binner_t {
	int initialized;	///< flag indicating if memory has been allocated for the buffer
	obs_pc_ringbuf_t ringbuf;
	tan_bin_table_t tan_table;
	distance_bin_table_t dist_table;

	// config fields must be set up before initializing!
	int ringbuf_size;
	int n_angle_bins;
	int n_distance_bins;
	int pie_threshold;
	float z_max;
	float z_min;
	float dist_max;
	float dist_min;
	float under_trim_dist;

	// variables used later by the pie binner
	float dist_min_squared;
	float under_ratio;
	int rows;
	int cols;
	int** map_points;	// 2d matrix tracking points in each bin
	float** map_dist;	// 2d matrix tracking average distance of points in each bin
	pthread_mutex_t mutex;
} pie_binner_t;


#define PIE_BINNER_INITIALIZER {\
	.initialized = 0,\
	.ringbuf = OBS_PC_RINGBUF_INITIALIZER,\
	.tan_table = TAN_BIN_TABLE_INITIALIZER,\
	.dist_table = DISTANCE_BIN_TABLE_INITIALIZER,\
	.pie_threshold = 0,\
	.z_max = 0.0f,\
	.z_min = 0.0f,\
	.rows = 0,\
	.cols = 0,\
	.map_points = NULL,\
	.map_dist = NULL,\
	.mutex = PTHREAD_MUTEX_INITIALIZER\
}

int pie_binner_init(pie_binner_t* b);
int pie_binner_free(pie_binner_t* b);
pie_binner_t pie_binner_empty(void);
int pie_binner_zero_out(pie_binner_t* b);
int pie_binner_add_point(pie_binner_t* b, float* p);
int pie_binner_filter_to_mavlink_msg(pie_binner_t* b, mavlink_message_t* msg);


#ifdef __cplusplus
}
#endif

#endif // OBS_PC_FILTER_H
