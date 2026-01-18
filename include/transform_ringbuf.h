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


#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif

#include <stdlib.h>
#include <pthread.h>
#include <rc_math.h>


/**
 * each item in the ringbuffer contains a 3x3 rotation matrix, translation, and
 * associated timestamp.
 */
typedef struct rc_tf_t {
	int64_t ts;
	rc_matrix_t R;
	rc_vector_t T;
} rc_tf_t;

/**
 * initializer for the rc_tf_ringbuf_t to make sure it starts zero'd out
 */
#define RC_TF_INITIALIZER {\
	.ts = -1,\
	.R = RC_MATRIX_INITIALIZER,\
	.T = RC_VECTOR_INITIALIZER}


/**
 * @brief      Struct containing state of a ringbuffer and pointer to
 * dynamically allocated memory.
 */
typedef struct rc_tf_ringbuf_t {
	rc_tf_t* d;	///< pointer to dynamically allocated data
	int size;	///< number of elements the buffer can hold
	int index;	///< index of the most recently added value
	int items_in_buf; ///< number of items in the buffer, between 0 and size
	int initialized;///< flag indicating if memory has been allocated for the buffer
	pthread_mutex_t mutex;
} rc_tf_ringbuf_t;


/**
 * initializer for the rc_tf_ringbuf_t to make sure it starts zero'd out
 */
#define RC_TF_RINGBUF_INITIALIZER {\
	.d = NULL,\
	.size = 0,\
	.index = 0,\
	.items_in_buf = 0,\
	.initialized = 0,\
	.mutex = PTHREAD_MUTEX_INITIALIZER}



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
int rc_tf_ringbuf_alloc(rc_tf_ringbuf_t* buf, int size);


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
int rc_tf_ringbuf_free(rc_tf_ringbuf_t* buf);

/**
 * @brief      returns an empty rc_tf_t transform
 *
 */
rc_tf_t rc_tf_empty(void);


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
int rc_tf_ringbuf_insert(rc_tf_ringbuf_t* buf, int64_t ts, rc_matrix_t R, rc_vector_t T);


/**
 * @brief      Fetches the entry which is 'position' steps behind the last value
 *             added to the buffer.
 *
 *             If 'position' is given as 0 then the most recent entry is
 *             returned. The position obviously can't be larger than size-1.
 *             This will also check and return -2 if the buffer hasn't been
 *             filled up enough to go back that far in time.
 *
 * @param      buf       Pointer to user's buffer
 * @param[in]  position  steps back in the buffer to fetch the entry from
 * @param      result    pointer to write the result into.
 *
 * @return     0 on success, -2 if buffer hasn't been populated yet, -3 if the
 *             timestamp is too new, and -1 on other error
 */
int rc_tf_ringbuf_get_entry_at_pos(rc_tf_ringbuf_t* buf, int position, rc_tf_t* result);


/**
 * @brief      fetches transform at timestamp ts
 *
 *             Interpolates between nearest two recorded transforms. If the
 *             buffer happens to contain an exact match at the requested
 *             timestamp then that tranform will be returned. If interpolation
 *             is necessary, then linear interpolation is used for the
 *             translation and SLERP is used to interpolate between rotations.
 *
 * @param[in]  buf   Pointer to user's buffer
 * @param[in]  ts    requested timestamp
 * @param[out] R     estimated rotation matrix at requested timestamp
 * @param[out] T     estimated translation at requested timestamp
 *
 * @return     Returns 0 if a valid value was found, -1 on error. -2 if there
 *             simply wasn't sufficient data in the buffer, such as when waiting
 *             for VIO to initialize or if the requested timestamp is too old
 */
int rc_tf_ringbuf_get_tf_at_time(rc_tf_ringbuf_t* buf, int64_t ts, rc_matrix_t* R, rc_vector_t* T);