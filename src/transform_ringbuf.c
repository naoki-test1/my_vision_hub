/**
 * "transform_ringbuf.h"
 *
 *
 * @author     james@modalai.com
 * @date       2020
 *
 */


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <rc_math.h>
#include "transform_ringbuf.h"


rc_tf_t rc_tf_empty(void)
{
	rc_tf_t new = RC_TF_INITIALIZER;
	return new;
}

/**
 * @brief      Allocates memory for a ring buffer and initializes an
 * rc_tf_ringbuf_t struct.
 *
 * If buf is already the right size then it is left untouched. Otherwise any
 * existing memory allocated for buf is freed to avoid memory leaks and new
 * memory is allocated.
 *
 * @param      buf   Pointer to user's buffer
 * @param[in]  size  Number of elements to allocate space for
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_tf_ringbuf_alloc(rc_tf_ringbuf_t* buf, int size)
{
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_alloc, received NULL pointer\n");
		return -1;
	}
	if(unlikely(size<2)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_alloc, size must be >=2\n");
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
	buf->d = (rc_tf_t*)calloc(size,sizeof(rc_tf_t));
	if(buf->d==NULL){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_alloc, failed to allocate memory\n");
		return -1;
	}
	int i;
	for(i=0;i<size; i++){
		buf->d[i]=rc_tf_empty();
		rc_matrix_alloc(&buf->d[i].R, 3,3);
		rc_vector_alloc(&buf->d[i].T,3);
	}
	// write out other details
	buf->size = size;
	buf->initialized = 1;
	return 0;
}

/**
 * @brief      Frees the memory allocated for buffer buf.
 *
 * Also set the initialized flag to 0 so other functions don't try to access
 * unallocated memory.
 *
 * @param      buf   Pointer to user's buffer
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_tf_ringbuf_free(rc_tf_ringbuf_t* buf)
{
	rc_tf_ringbuf_t new = RC_RINGBUF_INITIALIZER;
	if(unlikely(buf==NULL)){
		fprintf(stderr, "ERROR in rc_tf_ringbuf_free, received NULL pointer\n");
		return -1;
	}
	if(buf->initialized){
		int i;
		for(i=0;i<buf->size; i++){
			rc_matrix_free(&buf->d[i].R);
			rc_vector_free(&buf->d[i].T);
		}
		free(buf->d);
		// put buffer back to default
		*buf = new;
	}
	return 0;
}


/**
 * @brief      Puts a new float into the ring buffer and updates the index
 * accordingly.
 *
 * If the buffer was full then the oldest value in the buffer is automatically
 * removed.
 *
 * @param      buf   Pointer to user's buffer
 * @param[in]  val   The value to be inserted
 *
 * @return     Returns 0 on success or -1 on failure.
 */
int rc_tf_ringbuf_insert(rc_tf_ringbuf_t* buf, int64_t ts, rc_matrix_t R, rc_vector_t T)
{
	int new_index;

	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_insert, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_insert, ringbuf uninitialized\n");
		return -1;
	}

	pthread_mutex_lock(&buf->mutex);

	// increment index and check for loop-around
	// if this is the first thing to be entered make sure to start at zero
	if(buf->items_in_buf==0){
		//fprintf(stderr,"inserting first value\n");
		new_index=0;
	}
	else{
		new_index=buf->index+1;
		if(new_index>=buf->size) new_index=0;
	}

	// write out new value
	buf->d[new_index].ts=ts;
	rc_matrix_duplicate(R, &buf->d[new_index].R);
	rc_vector_duplicate(T, &buf->d[new_index].T);
	// bump index
	buf->index=new_index;
	// increment number of items if necessary
	if(buf->items_in_buf<buf->size) buf->items_in_buf++;

	pthread_mutex_unlock(&buf->mutex);

	return 0;
}


int rc_tf_ringbuf_get_entry_at_pos(rc_tf_ringbuf_t* buf, int position, rc_tf_t* result)
{
	int return_index;
	// sanity checks
	if(unlikely(buf==NULL || result==NULL)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_entry_at_pos, received NULL pointer\n");
		return -1;
	}
	if(unlikely(position<0 || (position>buf->size-1))){
		fprintf(stderr,"ERROR in rc_ringbuf_get_entry_at_pos, position out of bounds\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_entry_at_pos, ringbuf uninitialized\n");
		return -1;
	}
	// silently return if user requested an item that hasn't been added yet
	if(position>=buf->items_in_buf){
		//fprintf(stderr,"ERROR in rc_ringbuf_get_entry_at_pos %d, not enough entries\n", position);
		return -2;
	}

	pthread_mutex_lock(&buf->mutex);
	return_index=buf->index-position;

	// check for looparound
	if(return_index<0) return_index+=buf->size;
	*result = buf->d[return_index];

	pthread_mutex_unlock(&buf->mutex);
	return 0;
}

/**
 * @brief      Fetches the timestamp which is 'position' steps behind the last
 *             value added to the buffer.
 *
 *             If 'position' is given as 0 then the most recent entry is
 *             returned. The position obviously can't be larger than size-1.
 *             This will also check and return -2 if the buffer hasn't been
 *             filled up enough to go back that far in time.
 *
 *             don't lock the mutex in here! this function is used locally by
 *             get_tf_at_time which calls this many times while the mutex is
 *             locked already
 *
 * @param[in]  buf       Pointer to user's buffer
 * @param[in]  position  steps back in the buffer to fetch the entry from
 *
 * @return     requested timestamp on success, -2 if buffer doesn't contain
 *             enough entries to go back to requested position, -1 on other
 *             error
 */
static int64_t __get_ts_at_pos(rc_tf_ringbuf_t* buf, int position)
{
	int return_index;
	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_timestamp_at_pos, received NULL pointer\n");
		return -1;
	}
	if(unlikely(position<0 || (position>buf->size-1))){
		fprintf(stderr,"ERROR in rc_ringbuf_get_timestamp_at_pos, position out of bounds\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_ringbuf_get_timestamp_at_pos, ringbuf uninitialized\n");
		return -1;
	}
	// silently return if user requested an item that hasn't been added yet
	if(position>=buf->items_in_buf){
		//fprintf(stderr,"ERROR in rc_ringbuf_get_timestamp_at_pos %d, not enough entries\n", position);
		return -2;
	}

	return_index=buf->index-position;
	// check for looparound
	if(return_index<0) return_index+=buf->size;
	int64_t ret = buf->d[return_index].ts;

	return ret;
}


int rc_tf_ringbuf_get_tf_at_time(rc_tf_ringbuf_t* buf, int64_t ts, rc_matrix_t* R, rc_vector_t* T)
{
	int i;
	int found = 0;
	static rc_tf_t tf_before = RC_TF_INITIALIZER;
	static rc_tf_t tf_after = RC_TF_INITIALIZER;
	static rc_vector_t q_before = RC_VECTOR_INITIALIZER;
	static rc_vector_t q_after = RC_VECTOR_INITIALIZER;
	static rc_vector_t q_new = RC_VECTOR_INITIALIZER;
	int64_t tmp;

	// sanity checks
	if(unlikely(buf==NULL)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_get_value, received NULL pointer\n");
		return -1;
	}
	if(unlikely(!buf->initialized)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_get_value, ringbuf uninitialized\n");
		return -1;
	}
	if(ts<=0){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_get_value, requested timestamp must be >0\n");
		return -1;
	}
	if(buf->items_in_buf < 2){
		// fprintf(stderr,"ERROR in rc_tf_ringbuf_get_value, not enough data in buffer yet\n");
		return -2;
	}
	// allow timestamps up to 0.2s newer than our last position record
	if(ts > (__get_ts_at_pos(buf,0)+200000000)){
		fprintf(stderr,"ERROR in rc_tf_ringbuf_get_value, timestamp too new\n");
		fprintf(stderr,"Requested time %7.2fs newer than latest data\n", (double)(ts-__get_ts_at_pos(buf,0))/1000000000.0);
		return -3;
	}

	// check for timestamp newer than we have record of, if so, extrapolate given
	// the two most recent records
	if(ts > __get_ts_at_pos(buf,0)){
		if(unlikely(rc_tf_ringbuf_get_entry_at_pos(buf, 1,   &tf_before))){
			fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to fetch entry 0\n");
			return -1;
		}
		if(unlikely(rc_tf_ringbuf_get_entry_at_pos(buf, 0, &tf_after))){
			fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to fetch entry 1\n");
			return -1;
		}
		found = 1;
	}
	else{
		// now go searching through the buffer to find which two entries to interpolate between
		for(i=0;i<buf->items_in_buf;i++){
			tmp = __get_ts_at_pos(buf,i);

			// check for error
			if(tmp<=0){
				fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, found unpopulated entry at position%d\n", i);
				return -1;
			}

			// found the right value! no interpolation needed
			if(tmp == ts){
				if(unlikely(rc_tf_ringbuf_get_entry_at_pos(buf, i, &tf_before))){
					fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to fetch entry at ts\n");
					return -1;
				}
				rc_matrix_duplicate(tf_before.R, R);
				rc_vector_duplicate(tf_before.T, T);
				return 0;
			}

			// once we get a timestamp older than requested ts, we have found the
			// right interval, grab the appropriate transforms
			if(tmp<ts){
				// if this condition is met on the newest entry in the buffer where
				// position=0 then the user is asking for data that's in the future
				// or at least newer than the buffer if aware of. I suppose we could
				// interpolate into the future here but this condition shouldn't be
				// met in practice since the apriltag detection is much slower than
				// VIO is publishing
				if(i==0){
					fprintf(stderr,"WARNING: rc_tf_ringbuf_get_tf_at_time, requested timestamp is newer than buffer's newest entry\n");
					return -1;
				}

				// This step should be older than requested ts, make sure the next
				// entry is actually newer, it should be in data went into the
				// buffer monotonically
				if(__get_ts_at_pos(buf,i-1)<ts){
					fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, bad timestamp found\n");
					return -1;
				}

				// these fetches shouldn't fail since we already got the timestamp
				// at this position.
				if(unlikely(rc_tf_ringbuf_get_entry_at_pos(buf, i,   &tf_before))){
					fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to fetch entry before ts\n");
					return -1;
				}
				if(unlikely(rc_tf_ringbuf_get_entry_at_pos(buf, i-1, &tf_after))){
					fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to fetch entry after ts\n");
					return -1;
				}
				// break and start interpolation
				found = 1;
				break;
			}
		} // end loop through buffer
	}

	// unsuccessful in finding
	if(!found) return -2;

#ifdef DEBUG
	printf("in rc_tf_ringbuf_get_tf_at_time found entries with timestamps:\n");
	printf("%lld %lld on either side of %lld\n", tf_before.ts, tf_after.ts, ts);
	printf("interpolating between:\n");
	printf("R1:");
	rc_matrix_print(tf_before.R);
	printf("R2:");
	rc_matrix_print(tf_after.R);
	printf("T1:");
	rc_vector_print(tf_before.T);
	printf("T2:");
	rc_vector_print(tf_after.T);
#endif

	// calculate interpolation constant t which is between 0 and 1.
	// 0 would be right at tf_before, 1 would be right at tf_after.
	double t = (ts-tf_before.ts) / (tf_after.ts-tf_before.ts);

	// interpolate position linearly, this is the easy bit
	if(rc_vector_lin_interpolate(tf_before.T, tf_after.T, t, T)){
		fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to linearly interpolate T\n");
		return -1;
	}

	// to interpolate the rotations we convert to quaternions and use SLERP
	if(rc_rotation_to_quaternion(tf_before.R, &q_before)){
		fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to convert rotation matrix to quaternion\n");
		return -1;
	}
	if(rc_rotation_to_quaternion(tf_after.R, &q_after)){
		fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to convert rotation matrix to quaternion\n");
		return -1;
	}
	if(rc_quaternion_slerp(q_before, q_after, t, &q_new)){
		fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to interpolate quaternions\n");
		return -1;
	}
	if(rc_quaternion_to_rotation_matrix(q_new, R)){
		fprintf(stderr,"ERROR: rc_tf_ringbuf_get_tf_at_time, failed to convert quaternion to final rotation matrix\n");
		return -1;
	}

	return 0;
}

