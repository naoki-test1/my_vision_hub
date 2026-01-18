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
#include <unistd.h>
#include <math.h>
#include <getopt.h>
#include <stdlib.h> // for exit()
#include <modal_pipe.h>

#include "config_file.h"
#include "geometry.h"
#include "mavlink_io.h"
#include "mavlink_for_ros.h"
#include "vio_manager.h"
#include "tag_manager.h"
#include "voa_manager.h"
#include "autopilot_monitor.h"
#include "offboard_mode.h"
#include "fixed_pose_input.h"
#include "horizon_cal.h"
#include "horizon_cal_file.h"
#include "control_input.h"
#include "pipe_channels.h"
#include "imu_manager.h"
#include "state_manager.h"


#define PROCESS_NAME "voxl-vision-hub" // to name PID file

static int override_en_tag=0;
static int override_en_voa=0;
static int override_en_vio=0;
static int override_en_tag_fixed_frame=0;


static void _print_usage(void)
{
	printf("\n\
voxl-vision-hub usually runs as a systemd background service. However, for debug\n\
purposes it can be started from the command line manually with any of the following\n\
debug options. When started from the command line, voxl-vision-hub will automatically\n\
stop the background service so you don't have to stop it manually\n\
\n\
-b, --debug_mav_send        show debug info on mavlink packets going to autopilot\n\
-c, --load_config_only      Load the config file and then exit right away.\n\
                              This also adds new defaults if necessary. This is used\n\
                              by voxl-configure-vision-px4 to make sure the config file\n\
                              is up to date without actually starting this service.\n\
-d, --debug_uart_recv       show debug info on mavlink packets coming from autopilot\n\
-g, --debug_fixed_frame     print debug info regarding the calculation of fixed frame\n\
                              relative to local frame as set by tags.\n\
-h, --help                  print this help message\n\
-l, --debug_tag_local       print location and rotation of each tag in local frame\n\
                              note that tag frame of reference is not the same\n\
                              as local frame so interpreting roll/pitch/yaw requires\n\
                              some thought\n\
-m, --debug_tag_cam         print location and rotation of each tag detection wrt cam\n\
-o, --debug_odometry        print the odometry of body in local frame each time\n\
                              a VIO packet is sent to autopilot\n\
-p, --debug_odometry_fixed  print the odometry of body in fixed frame. Note this\n\
                              is not the data being sent to autopilot. This is to help\n\
                              ensure fixed tags are set correctly in config file\n\
-r, --debug_voa_filter      print VOA point cloud filtering info\n\
-s, --debug_voa_linescan    print detected obstacles as linescan points\n\
-t, --debug_voa_timing      print timing data about VOA point cloud calcs\n\
-u, --debug_offboard        print debug info for whichever offboard mode is active\n\
\n");
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"debug_mav_send",        no_argument,       0, 'b'},
		{"load_config_only",      no_argument,       0, 'c'},
		{"debug_mav_recv",        no_argument,       0, 'd'},
		{"debug_fixed_frame",     no_argument,       0, 'g'},
		{"help",                  no_argument,       0, 'h'},
		{"debug_tag_local",       no_argument,       0, 'l'},
		{"debug_tag_cam",         no_argument,       0, 'm'},
		{"debug_odometry",        no_argument,       0, 'o'},
		{"debug_odometry_fixed",  no_argument,       0, 'p'},
		{"debug_voa_filter",      no_argument,       0, 'r'},
		{"debug_voa_linescan",    no_argument,       0, 's'},
		{"debug_voa_timing",      no_argument,       0, 't'},
		{"debug_offboard",        no_argument,       0, 'u'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "bcdghlmoprstu",
							long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'b':
			mavlink_io_en_print_debug_send(1);
			mavlink_for_ros_en_print_debug_send(1);
			break;

		case 'c':
			config_file_load();
			exit(0);
			break;

		case 'd':
			mavlink_io_en_print_debug_recv(1);
			mavlink_for_ros_en_print_debug_recv(1);
			break;

		case 'g':
			geometry_en_print_fixed_frame_debug(1);
			override_en_tag_fixed_frame = 1;
			break;

		case 'h':
			_print_usage();
			return -1;

		case 'l':
			tag_manager_en_print_debug_wrt_local(1);
			override_en_tag=1;
			break;

		case 'm':
			tag_manager_en_print_debug_wrt_cam(1);
			override_en_tag=1;
			break;

		case 'o':
			vio_manager_en_print_debug_local(1);
			override_en_vio=1;
			break;

		case 'p':
			vio_manager_en_print_debug_fixed(1);
			override_en_vio=1;
			override_en_tag_fixed_frame = 1;
			break;

		case 'r':
			override_en_voa=1;
			voa_manager_en_debug(1);
			break;

		case 's':
			override_en_voa=1;
			voa_manager_en_print_linescan(1);
			break;

		case 't':
			override_en_voa=1;
			voa_manager_en_timing(1);
			break;

		case 'u':
			offboard_mode_en_print_debug(1);
			break;

		default:
			_print_usage();
			return -1;
		}
	}

	return 0;
}

static void _quit(int ret)
{
	// we don't want user inputs coming in while we are shutting down, so stop
	// the control input first
	// currently not in use
	// printf("stopping control input\n");
	// control_input_stop();

	// then stop non-critical optional threads that may still be dependent on
	// more critical features like UDP and UART ports
	if(ret==0){
		offboard_mode_stop(1); // stop in blocking mode, waits until thread joins
	}
	else{
		offboard_mode_stop(0); // non-blocking mode
	}

	printf("stopping horizon_cal\n");
	horizon_cal_stop(0);

	// stop all the vision stuff before closing comms ports since these all
	// write out mavlink
	printf("stopping voa manager\n");
	voa_manager_stop();
	printf("stopping tag manager\n");
	tag_manager_stop();
	printf("stopping vio manager\n");
	vio_manager_stop();
	printf("stopping fixed pose in module\n");
	fixed_pose_input_stop();
	printf("stopping imu manager\n");
	imu_manager_stop();
	printf("stopping state manager\n");
	state_manager_stop();

	// now stop the critical UDP and UART mavlink interfaces
	printf("Stopping mavlink for ros module\n");
	mavlink_for_ros_stop();
	printf("Stopping mavlink io module\n");
	mavlink_io_stop();
	printf("Stopping autopilot monitor\n");
	autopilot_monitor_stop();

	// each module should ahve cleaned up its own pipes, but to be safe we
	// make are everything is closed up here
	printf("closing remaining client pipes\n");
	pipe_client_close_all();
	printf("closing remaining server pipes\n");
	pipe_server_close_all();

	// clean up our PID file is all was successful
	printf("Removing PID file\n");
	remove_pid_file(PROCESS_NAME);
	printf("exiting\n");
	exit(ret);
	return;
}


// initializes everything then waits on signal handler to close
int main(int argc, char* argv[])
{
	if(_parse_opts(argc, argv)) return -1;

	printf("loading our own config file\n");
	if(config_file_load()) return -1;
	config_file_print();

	printf("loading extrinsics config file\n");
	if(load_extrinsics_file()) return -1;

	printf("loading horizon cal file\n");
	if(load_horizon_cal_file()) return -1;

	// after loading config file, apply overrides that came from arguments
	if(override_en_vio) en_vio=1;
	if(override_en_voa) en_voa=1;
	if(override_en_tag_fixed_frame) en_tag_fixed_frame = 1;

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(kill_existing_process(PROCESS_NAME, 2.0)<-2) return -1;

	// start signal manager so we can exit cleanly
	if(enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal manager\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PROCESS_NAME);

	// Set this before initializing any other modules so that they can
	// use it.
	main_running=1;

	// start the critical modules other things depend on
	printf("starting geometry module\n");
	if(geometry_init()){
		_quit(-1);
	}

	// start monitor before mavlink-io
	printf("starting autopilot monitor\n");
	if(autopilot_monitor_init()){
		_quit(-1);
	}

	// most things depends on mavlink-io so start early
	printf("starting mavlink IO\n");
	if(mavlink_io_init()){
		_quit(-1);
	}

	if(en_localhost_mavlink_udp){
		printf("starting mavlink for ros\n");
		if(mavlink_for_ros_init()){
			_quit(-1);
		}
	}

	printf("starting fixed pose input\n");
	if(fixed_pose_input_init()){
		_quit(-1);
	}

	// start vio manager even is "en_vio" is disabled since VFC will use it
	printf("starting vio manager\n");
	if(vio_manager_init()){
		_quit(-1);
	}

	printf("starting tag manager\n");
	if(tag_manager_init()){
		_quit(-1);
	}

	if(en_voa && n_voa_inputs>0){
		printf("starting voa manager\n");
		if(voa_manager_init()){
			_quit(-1);
		}
	}

	printf("starting horizon cal module\n");
	if(horizon_cal_init()){
		_quit(-1);
	}

	printf("starting imu manager\n");
	if(imu_manager_init()){
		_quit(-1);
	}

	printf("starting state manager\n");
	if(state_manager_init()){
		_quit(-1);
	}

	if(offboard_mode_init()){
		_quit(-1);
	}

	// currently not in use
	// printf("starting control input pipe\n");
	// if(control_input_init()){
	// 	_quit(-1);
	// }


////////////////////////////////////////////////////////////////////////////////
// all threads started, wait for signal manager to stop it
////////////////////////////////////////////////////////////////////////////////

	printf("Init complete\n");
	usleep(500000);

	// warn user if PX4 still hasn't connected yet
	if(main_running && !autopilot_monitor_is_connected()){
		if(!pipe_client_is_connected(MAVLINK_ONBOARD_CH)){
			fprintf(stderr, "WARNING: voxl-mavlink-server does not appear to be running\n");
			fprintf(stderr, "make sure it is enabled: systemctl enable voxl-mavlink-server\n");
		}
		else{
			fprintf(stderr, "WARNING: connected to voxl-mavlink-server but no messages from autopilot\n");
		}
	}


	while(main_running) usleep(500000);
	printf("Starting shutdown sequence\n");


////////////////////////////////////////////////////////////////////////////////
// Stop all the threads
////////////////////////////////////////////////////////////////////////////////

	_quit(0);
	return 0;
}
