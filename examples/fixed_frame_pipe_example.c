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

/**
 * @file fixed_frame_pipe_example
 *
 * This is not compiled as part of this project, it's just an example of how to
 * send data into the fixed frame pipe.
 */


#include <stdio.h>
#include <fcntl.h>	// for O_WRONLY & O_NONBLOCK
#include <errno.h>	// for perror
#include <unistd.h>	// for write and close
#include <time.h>
#include <voxl_vision_px4.h>

int main()
{
	// open the pipe
	int fd;
	fd = open(FIXED_FRAME_PIPE_PATH, O_WRONLY|O_NONBLOCK);
	if(fd<0){
		perror("couldn't open fixed frame pipe:");
		fprintf(stderr, "make sure voxl-vision-px4 service is running and\n");
		fprintf(stderr, "en_fixed_frame_pipe is set to true in\n");
		return -1;
	}

	// grab current time to use for timestamp
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	int64_t time_ns = (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;

	// example pose, zero rotation, 1m above ground
	pose_4dof_t pose;
	pose.timestamp_ns = time_ns;
	pose.p[0] =  0.0; // x
	pose.p[1] =  0.0; // y
	pose.p[2] = -1.0; // z
	pose.yaw  =  0.0;

	// write the raw struct and check for error
	int bytes = write(fd, &pose, sizeof(pose_4dof_t));
	if(bytes < (int)sizeof(pose_4dof_t)){
		perror("error writing to pipe:");
		close(fd);
		return -1;
	}

	// close and quit
	printf("closing %s\n", FIXED_FRAME_PIPE_PATH);
	close(fd);
	return 0;
}