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

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <modal_pipe_client.h>

#include "imu_manager.h"
#include "pipe_channels.h"

#define IMU_BUFFER_SIZE 500

static imu_data_t imu_data[IMU_BUFFER_SIZE];
static int imu_data_cnt = -1;
static pthread_mutex_t buf_mutex = PTHREAD_MUTEX_INITIALIZER;

static void add_imu_data_to_buffer(imu_data_t imu_data_in) {
    pthread_mutex_lock(&buf_mutex);
    imu_data_cnt++;

    if (imu_data_cnt >= IMU_BUFFER_SIZE) {
        imu_data_cnt = 0;
    }

    memcpy(&imu_data[imu_data_cnt], &imu_data_in, sizeof (imu_data_t));
    pthread_mutex_unlock(&buf_mutex);

    // for (int i = 0; i < IMU_BUFFER_SIZE; i++) {
    //     printf("%d: imu: %ld %f\n", i, imu_data[i].timestamp_ns, (double)imu_data[i].gyro_rad[0]);
    // }
}

int get_latest_imu_data(imu_data_t *imu_data_out){
    pthread_mutex_lock(&buf_mutex);
    if (imu_data_cnt > 0) {
        memcpy(imu_data_out, &imu_data[imu_data_cnt], sizeof (imu_data_t));
        pthread_mutex_unlock(&buf_mutex);
        return 0;
    }
    else {
        pthread_mutex_unlock(&buf_mutex);
        return -1;
    }
}


static void new_imu_data_handler(__attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context){

	int n_packets;
	imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);

	if (data_array == NULL) return;
	if (n_packets <= 0) return;

	//if(1) printf("got %3d imu packets\n", n_packets);

	for (int i = 0; i < n_packets; i++) {
		// uint32_t magic_number;  ///< Set to IMU_IMAGE_MAGIC_NUMBER for frame syncing
		// float accl_ms2[3];      ///< XYZ acceleration in m/s^2
		// float gyro_rad[3];      ///< XYZ gyro rotation in rad/s
		// float temp_c;           ///< temp in C, IMU_INVALID_TEMPERATURE_VALUE if no thermometer present
		// uint64_t timestamp_ns;  ///< timestamp in nanoseconds, uses system clock_monotonic
		// printf("imu: %ld %f\n",data_array[i].timestamp_ns, (double)data_array[i].gyro_rad[0]);

        add_imu_data_to_buffer(data_array[i]);
	}
}


int imu_manager_init(){
	pipe_client_set_simple_helper_cb(IMU_PIPE_CH, new_imu_data_handler, NULL);

	// connect to imu - hardcoded for now, TODO
	char full_pipe[32] = "imu_apps";

	int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;
	if (pipe_client_open(IMU_PIPE_CH, full_pipe, PIPE_CLIENT_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE) != 0) {
		printf("FAILED TO OPEN IMU PIPE\n");
		return -1;
	}
	return 0;
}


void imu_manager_stop(void)
{
	// close client pipes
	pipe_client_close(IMU_PIPE_CH);

	return;
}
