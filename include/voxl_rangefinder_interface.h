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


#ifndef VOXL_RANGEFINDER_SERVER_PIPE_INTERFACE_H
#define VOXL_RANGEFINDER_SERVER_PIPE_INTERFACE_H


#include <modal_pipe_interfaces.h>
#include <modal_pipe_common.h> // for MODAL_PIPE_DEFAULT_BASE_DIR


#define RANGEFINDER_PIPE_NAME		"rangefinders"
#define RANGEFINDER_PIPE_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR RANGEFINDER_PIPE_NAME "/")

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define RANGEFINDER_MAGIC_NUMBER (0x564F584C)


#define RANGEFINDER_TYPE_STRINGS {"unknown", "TOF_VL53L1X","Chirp_CH201"}
#define N_RANGEFINDER_TYPES	3

// types for the 'type' fields in rangefinder_data_t
#define RANGEFINDER_TYPE_UNKNOWN		0
#define RANGEFINDER_TYPE_TOF_VL53L1X	1 // supports 27, 20, and 15 degree FOV
#define RANGEFINDER_TYPE_CHIRP_CH201	2

/*
 * data structure containing detailed data about a rangefinder measurement.
 *
 * It is perhaps a bit overkill but pipes are fast and rangefinders run at slow sample rates so this makes it easy from a software standpoint to have detailed information about the rangefinder itself available.
 *
 * totals 68 bytes
 */
typedef struct rangefinder_data_t{
	uint32_t magic_number;      ///< Unique 32-bit number used to signal the beginning of a struct
	int64_t timestamp_ns;       ///< Timestamp in clock_monotonic system time
	uint32_t sample_id;         ///< When multiple sensors are sampled at the same time they should share the same sample_id

	int sensor_id;              ///< unique id identifying the sensor from which the sample originated.
	float distance_m;           ///< distance in meters
	float uncertainty_m;        ///< uncertainty in meters. Set negative if unknown

	float fov_deg;              ///< field of view of the sensor in degrees
	float location_wrt_body[3]; ///< location of the rangefinder with respect to body frame.
	float direction_wrt_body[3];///< direction vector of the rangefinder with respect to body frame

	float range_max_m;          ///< Maximum range of the sensor in meters
	int type;                   ///< Rangefinder type, e.g. RANGEFINDER_TPYE_VL53L1X
	int reserved;               ///<  reserved for future use
} __attribute__((packed)) rangefinder_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 120 packets which is
 * perhaps more than necessary but take just under 2 pages of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define RANGEFINDER_RECOMMENDED_READ_BUF_SIZE	(sizeof(rangefinder_data_t) * 120)


// we recommend a 128k buffer that can hold 1927 rangefinder packets
#define RANGEFINDER_RECOMMENDED_PIPE_SIZE		(128 * 1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a rangefinder_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a rangefinder_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() of the data pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an rangefinder_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
static inline rangefinder_data_t* voxl_rangefinder_validate_pipe_data(char* data, int bytes, int* n_packets)
{
	// cast raw data from buffer to an vio_data_t array so we can read data
	// without memcpy. Also write out packets read as 0 until we validate data.
	rangefinder_data_t* new_ptr = (rangefinder_data_t*) data;
	*n_packets = 0;

	// basic sanity checks
	if(bytes<0){
		fprintf(stderr, "ERROR validating VIO data received through pipe: number of bytes = %d\n", bytes);
		return NULL;
	}
	if(data==NULL){
		fprintf(stderr, "ERROR validating VIO data received through pipe: got NULL data pointer\n");
		return NULL;
	}
	if(bytes%sizeof(rangefinder_data_t)){
		fprintf(stderr, "ERROR validating VIO data received through pipe: read partial packet\n");
		fprintf(stderr, "read %d bytes, but it should be a multiple of %ld\n", bytes, sizeof(rangefinder_data_t));
		return NULL;
	}

	// calculate number of packets locally until we validate each packet
	int n_packets_tmp = bytes/sizeof(rangefinder_data_t);

	// check if any packets failed the magic number check
	int i, n_failed = 0;
	for(i=0;i<n_packets_tmp;i++){
		if(new_ptr[i].magic_number != RANGEFINDER_MAGIC_NUMBER){
			n_failed++;
			fprintf(stderr, "ERROR got magic number %d, expected %d\n", new_ptr[i].magic_number, RANGEFINDER_MAGIC_NUMBER);
		}
	}
	if(n_failed>0){
		fprintf(stderr, "ERROR validating rangefinder data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
		return NULL;
	}

	// if we get here, all good. Write out the number of packets read and return
	// the new cast pointer. It's the same pointer the user provided but cast to
	// the right type for simplicity and easy of use.
	*n_packets = n_packets_tmp;
	return new_ptr;
}



#endif // VOXL_RANGEFINDER_SERVER_PIPE_INTERFACE_H
