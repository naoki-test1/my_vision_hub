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

#include <stdio.h>	// for fprintf
#include <unistd.h>	// for read() & write()
#include <errno.h>	// to check error in read() and write()
#include <fcntl.h>	// for O_WRONLY & O_RDONLY
#include <string.h>	// for strlen()
#include <stdlib.h>	// for exit()
#include <pthread.h>

#include <voxl_vision_hub.h>
#include <modal_pipe_client.h>
#include <modal_start_stop.h>


#define CLIENT_NAME "voxl-calibrate-px4-horizon"
#define PIPE_READ_BUF_SIZE 1024

/**
 * This is a blocking function which returns 0 if the user presses ENTER.
 * If ctrl-C is pressed it will quit the program
 */
static int continue_or_quit(void)
{
	// set stdin to non-canonical raw mode to capture all button presses
	fflush(stdin);
	if(system("stty raw")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty raw\n");
		return -1;
	}

	int ret;

	while(1){
		int c = getchar();
		// break if we read ctrl-c
		if(c==3 || c==24 || c==26){
			ret=-1;
			break;
		}
		if(c=='\r' || c=='\n'){
			ret = 0;
			break;
		}
	}

	fflush(stdin);

	// put stdin back to normal canonical mode
	if(system("stty cooked")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty cooked\n");
		return -1;
	}

	return ret;
}


// called whenever we connect or reconnect to the server
static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	int ret;

	// now we are connected and before we read data,
	// check that the type is correct!!!
	if(!pipe_is_type(HORIZON_CAL_IO_NAME, "text")){
		fprintf(stderr, "ERROR, pipe is not of type \"text\"\n");
		main_running = 0;
		return;
	}

	printf("\n\n\n");
	printf("-------------------------------------------------------------------------\n");
	printf("\n");
	printf("This calibration process is an in-flight alternative to the level horizon\n");
	printf("calibration method in QGC. By doing this in-flight, airframe asymmetries\n");
	printf("are taken into account, and you do not need a perfectly level surface and\n");
	printf("perfectly level landing gear.\n");
	printf("\n");
	printf("For this you should take off indoors in position mode with VIO enabled in\n");
	printf("a large enough room to avoid excessive turbulence. Make sure airflow in\n");
	printf("the room is minimized, including turning off fans and AC units.\n");
	printf("\n");
	printf("When you are ready, take off in position mode and fly around for 20-30\n");
	printf("seconds to allow VIO to stabilize and for EKF2 to converge on its IMU bias.\n");
	printf("Then navigate the drone to the middle of the room and leave it still.\n");
	printf("\n");
	printf("voxl-vision-px4 will monitor the roll/pitch values reported by PX4 until\n");
	printf("the drone remains still enough for 10 seconds to be confident that the\n");
	printf("measurement is correct. If the condition is met, it will write new\n");
	printf("SENS_BOARD_X_OFF & SENS_BOARD_Y_OFF parameters to PX4 once disarmed.\n");
	printf("\n");
	printf("If you have a particularly wobbly drone or a lot of turbulence, then the\n");
	printf("stationary condition may not ever be met. You can increase the allowable\n");
	printf("noise tolerance in /etc/modalai/voxl-vision-px4.conf with the field\n");
	printf("horizon_cal_tolerance. Note that increasing this will potentially\n");
	printf("reduce the accuracy of the calibration. Allow the drone to hover for at\n");
	printf("least 2 minutes without success before increasing the allowable tolerance.\n");
	printf("\n");
	printf("\n");
	printf("Press ENTER to start the calibration process or Ctrl-C to quit\n");
	printf("\n");

	if(continue_or_quit()){
		main_running = 0;
		return;
	}

	printf("\nsending command to start calibration\n");
	ret = pipe_client_send_control_cmd(0, CONTROL_COMMAND_START_HORIZON_CALIBRATION);
	if(ret<0){
		fprintf(stderr, "failed to send control command to voxl-vision-px4\n");
		pipe_print_error(ret);
		main_running = 0;
	}



	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "disconnected from voxl-vision-px4\n");
	return;
}


// print out anything vvpx4 sends us
static void _simple_cb(__attribute__((unused))int ch, char* data, __attribute__((unused))int bytes, __attribute__((unused)) void* context)
{
	int i = 0;

	// print each string received in this callback
	while(i<bytes){

		// print from start of next string
		char* start = data+i;
		printf("%s",start);

		// check if we are done
		if(strcmp(start,"Horizon Calibration Complete\n")==0){
			main_running = 0;
		}

		// increment to the start of the next string
		i += strlen(start)+1;
	}

	return;
}




int main()
{
	int ret;

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// for this test we will use the simple helper with optional debug mode
	int flags = CLIENT_FLAG_EN_SIMPLE_HELPER;

	// in auto-reconnect mode, tell the user we are waiting
	printf("trying to connect to voxl-vision-px4\n");

	// assign callabcks for data, connection, and disconnect. the "NULL" arg
	// here can be an optional void* context pointer passed back to the callbacks
	pipe_client_set_simple_helper_cb(0, _simple_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);

	// init connection to server. In auto-reconnect mode this will "succeed"
	// even if the server is offline, but it will connect later on automatically
	ret = pipe_client_open(0, HORIZON_CAL_IO_NAME, CLIENT_NAME, flags, PIPE_READ_BUF_SIZE);

	// check for success
	if(ret){
		fprintf(stderr, "ERROR opening channel:\n");
		pipe_print_error(ret);
		if(ret==PIPE_ERROR_SERVER_NOT_AVAILABLE){
			fprintf(stderr, "make sure to start modal-hello-server first!\n");
		}
		return -1;
	}

	// keep going until signal handler sets the main_running flag to 0
	while(main_running){
		usleep(100000);
	}

	// all done, signal pipe read threads to stop
	fflush(stdout);
	pipe_client_close_all();

	return 0;
}
