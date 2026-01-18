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

#include <stdio.h>
#include <math.h>
#include "pose_filter.h"
#include "macros.h"


int rc_pose_filter_alloc(rc_pose_filter_t* f, int len)
{
	// sanity checks
	if(len<1){
		fprintf(stderr,"ERROR in rc_pose_filter_alloc, len must be >=1\n");
		return -1;
	}
	pthread_mutex_lock(&f->mutex);

	if(len>1){
		if(rc_ringbuf_alloc(&f->bx,len)){
			fprintf(stderr,"ERROR in rc_pose_filter_alloc, failed to alloc ringbuf\n");
			return -1;
		}
		if(rc_ringbuf_alloc(&f->by,len)){
			fprintf(stderr,"ERROR in rc_pose_filter_alloc, failed to alloc ringbuf\n");
			return -1;
		}
		if(rc_ringbuf_alloc(&f->bz,len)){
			fprintf(stderr,"ERROR in rc_pose_filter_alloc, failed to alloc ringbuf\n");
			return -1;
		}
		if(rc_ringbuf_alloc(&f->byaw,len)){
			fprintf(stderr,"ERROR in rc_pose_filter_alloc, failed to alloc ringbuf\n");
			return -1;
		}
	}

	f->len=len;
	f->x=0.0;
	f->y=0.0;
	f->z=0.0;
	f->yaw=0.0;
	f->step=0;
	f->initialized=1;
	pthread_mutex_unlock(&f->mutex);
	return 0;
}



int rc_pose_filter_free(rc_pose_filter_t* f)
{
	if(!f->initialized) return 0;
	pthread_mutex_lock(&f->mutex);
	rc_ringbuf_free(&f->bx);
	rc_ringbuf_free(&f->by);
	rc_ringbuf_free(&f->bz);
	rc_ringbuf_free(&f->byaw);
	pthread_mutex_unlock(&f->mutex);
	*f=rc_pose_filter_empty();
	return 0;
}


rc_pose_filter_t rc_pose_filter_empty(void)
{
	rc_pose_filter_t ret = RC_POSE_FILTER_INITIALIZER;
	return ret;
}


int rc_pose_filter_march(rc_pose_filter_t* f, rc_vector_t xyz, double yaw)
{
	int i;
	double sum;

	// sanity checks
	if(unlikely(f==NULL)){
		fprintf(stderr,"ERROR in rc_pose_filter_march, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_pose_filter_march, filter uninitialized\n");
		return -1;
	}
	if(unlikely(!xyz.initialized)){
		fprintf(stderr,"ERROR in rc_pose_filter_march, vector xyz uninitialized\n");
		return -1;
	}
	if(unlikely(xyz.len!=3)){
		fprintf(stderr,"ERROR in rc_pose_filter_march, vector xyz should be of length 3\n");
		return -1;
	}

	pthread_mutex_lock(&f->mutex);

	// easy case where filter is of length 1, just copy values in
	if(f->len==1){
		f->yaw = yaw;
		f->x = xyz.d[0];
		f->y = xyz.d[1];
		f->z = xyz.d[2];
		f->step++;
	}
	else{
		// for yaw, check for wrap first, on first step xyzyaw[3]==0 so
		// this wrap shouldn't occure on first step
		WRAP_TO_NEGPI_TO_PI(yaw);
		if(yaw>(f->yaw+PI)){
			for(i=0;i<f->len;i++) f->byaw.d[i]+=TWO_PI;
		}
		else if(yaw<(f->yaw-PI)){
			for(i=0;i<f->len;i++) f->byaw.d[i]-=TWO_PI;
		}
		rc_ringbuf_insert(&f->byaw, yaw);
		rc_ringbuf_insert(&f->bx, xyz.d[0]);
		rc_ringbuf_insert(&f->by, xyz.d[1]);
		rc_ringbuf_insert(&f->bz, xyz.d[2]);
		f->step++;

		// calculate new outputs
		int n=MIN(f->step,f->len);
		sum = 0.0;
		for(i=0;i<n;i++) sum+=rc_ringbuf_get_value(&f->bx,i);
		f->x=sum/n;
		sum = 0.0;
		for(i=0;i<n;i++) sum+=rc_ringbuf_get_value(&f->by,i);
		f->y=sum/n;
		sum = 0.0;
		for(i=0;i<n;i++) sum+=rc_ringbuf_get_value(&f->bz,i);
		f->z=sum/n;
		sum = 0.0;
		for(i=0;i<n;i++) sum+=rc_ringbuf_get_value(&f->byaw,i);
		f->yaw=sum/n;

		// make sure the new output didn't tip around the wrap point
		// this should hapen almost never
		if(f->yaw>PI){
			for(i=0;i<f->len;i++) f->byaw.d[i]-=TWO_PI;
			f->yaw-=TWO_PI;
		}
		else if(f->yaw<-PI){
			for(i=0;i<f->len;i++) f->byaw.d[i]+=TWO_PI;
			f->yaw+=TWO_PI;
		}
	}

	pthread_mutex_unlock(&f->mutex);
	return 0;
}

int rc_pose_filter_fetch(rc_pose_filter_t* f, rc_vector_t* xyz, double* yaw)
{
	// sanity checks
	if(unlikely(f==NULL)){
		fprintf(stderr,"ERROR in rc_pose_filter_fetch, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!f->initialized)){
		fprintf(stderr,"ERROR in rc_pose_filter_fetch, filter uninitialized\n");
		return -1;
	}
	if(unlikely(!xyz->initialized)){
		fprintf(stderr,"ERROR in rc_pose_filter_fetch, vector xyz uninitialized\n");
		return -1;
	}
	if(unlikely(xyz->len!=3)){
		fprintf(stderr,"ERROR in rc_pose_filter_fetch, vector xyz should be of length 3\n");
		return -1;
	}
	pthread_mutex_lock(&f->mutex);
	xyz->d[0]=f->x;
	xyz->d[1]=f->y;
	xyz->d[2]=f->z;
	*yaw=f->yaw;
	pthread_mutex_unlock(&f->mutex);
	return 0;
}

