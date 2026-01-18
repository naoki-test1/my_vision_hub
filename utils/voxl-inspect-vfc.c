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
#include <signal.h>
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>

#include <modal_pipe_client.h>
#include <modal_start_stop.h>

#include "offboard_vfc.h"

#define CLIENT_NAME		"voxl-inspect-vfc"

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN] = "offboard_log";

static const char* submode_strings[VFC_MAX_SUBMODES] = {"      NONE       ",
														" THRUST_ATTITUDE ",
														"ALTITUDE_ATTITUDE",
														"  ALTITUDE_FLOW  ",
														"     POSITION    ",
														"       TRAJ      ",
														"  VFC_BACKTRACK  "};

#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define RESET_FONT			"\x1b[0m"	// undo any font/color settings
#define FONT_BOLD			"\033[1m"	// bold font
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place

static void _print_usage(void)
{
	printf("\n\
typical usage\n\
/# voxl-inspect-vfc\n\
\n");
	return;
}

// called whenever we connect or reconnect to the server
static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf(FONT_BOLD);
	printf("\n");
	printf(" dt(ms) |");
	printf(" flow |");
	printf(" vio  |");
	printf(" altitude |");
	printf("   desired mode   |");
	printf("   actual mode    |");
	printf("\n");
	printf(RESET_FONT);
	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "\nvfc server disconnected\n");
	return;
}


static void _print_data(offboard_log_packet d)
{
	// keep track of time between samples
	static int64_t t_last = 0;
	double dt_ms;
	if(t_last == 0) dt_ms = 0.0;
	else dt_ms = (double)(d.timestamp_ns-t_last)/1000000.0;
	t_last = d.timestamp_ns;

	printf("\r" CLEAR_LINE);

	printf("%7.1f |", dt_ms);

	if (d.flow_ok) {
		printf("  OK  |");
	} else {
		printf(" FAIL |");
	}

	if (d.position_ok) {
		printf("  OK  |");
	} else {
		printf(" FAIL |");
	}

	if (d.altitude_ok) {
		printf("    OK    |");
	} else {
		printf("   FAIL   |");
	}

	printf(" %s |", submode_strings[d.desired_submode]);
	printf(" %s | ", submode_strings[d.submode]);

	fflush(stdout);
	return;
}


static void _helper_cb( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	int packet_size = sizeof(offboard_log_packet);
	// validate that the data makes sense
	int n_packets = 0;
	if ((! bytes) || (bytes % packet_size)) {
		printf("Invalid data size: %d", bytes);
	}
	else {
		n_packets = bytes / packet_size;
		for (int i = 0; i < n_packets; i++) {
			_print_data((offboard_log_packet) *((offboard_log_packet*) (data + (packet_size * i))));
		}
	}

	return;
}

int main()
{
	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// prints can be quite long, disable terminal wrapping
	printf(DISABLE_WRAP);

	// set up all our MPA callbacks
	pipe_client_set_simple_helper_cb(0, _helper_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// request a new pipe from the server
	printf("waiting for server\n");
	int ret = pipe_client_open(0, pipe_path, CLIENT_NAME, \
				EN_PIPE_CLIENT_SIMPLE_HELPER, \
				sizeof(offboard_log_packet) * 10);

	// check for MPA errors
	if(ret<0){
		pipe_print_error(ret);
		printf(ENABLE_WRAP);
		return -1;
	}

	// keep going until signal handler sets the running flag to 0
	while(main_running) usleep(200000);

	// all done, signal pipe read threads to stop
	printf("\nclosing and exiting\n");
	pipe_client_close_all();
	printf(ENABLE_WRAP);

	return 0;
}
