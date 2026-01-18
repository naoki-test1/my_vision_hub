/**
 * "ring_buf.h"
 *
 * @brief      ring buffer implementation for arbitrary datatypes
 *
 * Ring buffers are FIFO (first in first out) buffers of fixed length which
 * efficiently boot out the oldest value when full. They are particularly well
 * suited for storing the last n values in a discrete time filter.
 *
 * The user creates their own instance of a buffer and passes a pointer to the
 * these functions to perform normal operations.
 *
 * @author     James Strawson
 * @date       2021
 *
 */


#ifndef RC_POSE_FILTER_H
#define RC_POSE_FILTER_H

#include <rc_math.h>
#include <pthread.h>

typedef struct rc_pose_filter_t {
	rc_ringbuf_t bx;
	rc_ringbuf_t by;
	rc_ringbuf_t bz;
	rc_ringbuf_t byaw;
	double x;
	double y;
	double z;
	double yaw;
	int len;
	int initialized;
	int step;
	pthread_mutex_t mutex;
} rc_pose_filter_t;


#define RC_POSE_FILTER_INITIALIZER {\
	.bx = RC_RINGBUF_INITIALIZER,\
	.by = RC_RINGBUF_INITIALIZER,\
	.bz = RC_RINGBUF_INITIALIZER,\
	.byaw = RC_RINGBUF_INITIALIZER,\
	.x=0.0,\
	.y=0.0,\
	.z=0.0,\
	.yaw=0.0,\
	.len = 0,\
	.initialized = 0,\
	.step = 0,\
	.mutex = PTHREAD_MUTEX_INITIALIZER}




int rc_pose_filter_alloc(rc_pose_filter_t* f, int len);



int rc_pose_filter_free(rc_pose_filter_t* f);


rc_pose_filter_t rc_pose_filter_empty(void);


int rc_pose_filter_march(rc_pose_filter_t* f, rc_vector_t xyz, double yaw);

int rc_pose_filter_fetch(rc_pose_filter_t* f, rc_vector_t* xyz, double* yaw);

#endif // end #define RC_POSE_FILTER_H