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
#include <modal_pipe_sink.h>

#include "control_input.h"
#include "voxl_vision_hub.h"
#include "pipe_channels.h"
#include "horizon_cal.h"

#define PIPE_SIZE		(64*1024)	// 64k is a typical linux default pipe size
#define READ_BUF_SIZE	(4*1024)	// tidy 1k read buffer


static void data_cb(__attribute__((unused)) int ch, char* data, int bytes, \
											__attribute__((unused)) void* context)
{
	// catch possible read errors
	if(bytes<=0) return;

	// remove the trailing newline from echo
	if(bytes>1 && data[bytes-1]=='\n'){
		data[bytes-1]=0;
	}


	printf("WARNING: received unknown command through control input pipe!\n");
	printf("got %d bytes. Command is: %s\n", bytes, data);
	return;
}

int control_input_init(void)
{
	pipe_sink_set_simple_cb(CONTROL_INPUT_CH, data_cb, NULL);

	int flags = SINK_FLAG_EN_SIMPLE_HELPER;
	return pipe_sink_create(CONTROL_INPUT_CH, CONTROL_INPUT_LOCATION, \
								flags, PIPE_SIZE, READ_BUF_SIZE);
}

int control_input_stop(void)
{
	pipe_sink_close(CONTROL_INPUT_CH);
	return 0;
}
