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

#include "config_file.h"
#include "offboard_mode.h"
#include "offboard_figure_eight.h"
#include "offboard_wps.h"
#include "offboard_follow_tag.h"
#include "offboard_trajectory.h"
#include "offboard_vfc.h"
#include "offboard_backtrack.h"


int offboard_mode_init(void)
{
	// start appropriate offboard mode thread
	if(offboard_mode == FIGURE_EIGHT){
		printf("starting offboard figure eight\n");
		return offboard_figure_eight_init();
	}
	else if(offboard_mode == FOLLOW_TAG){
		printf("starting offboard follow tag\n");
		return offboard_follow_tag_init();
	}
	else if(offboard_mode == TRAJECTORY){
		printf("starting offboard trajectory\n");
		return offboard_trajectory_init();
	}
	else if(offboard_mode == VFC){
		printf("starting offboard vfc\n");
		return offboard_vfc_init();
	}
	else if(offboard_mode == BACKTRACK){
		printf("starting offboard backtrack\n");
		return offboard_backtrack_init();
	}
	else if(offboard_mode == WPS){
		printf("starting offboard waypoints with timeout %f and stride %f\n",
				(double)wps_timeout, (double)wps_stride);
		offboard_wps_set_pause_time(wps_timeout);
		offboard_wps_vfc_mission(wps_vfc_mission);
		offboard_wps_vfc_mission_loop(wps_vfc_mission_loop);
		offboard_wps_vfc_mission_to_ramp(wps_vfc_mission_to_ramp);
		offboard_wps_vfc_mission_to_kp(wps_vfc_mission_to_kp);
		offboard_wps_vfc_mission_cruise_speed(wps_vfc_mission_cruise_speed);
		offboard_wps_set_stride(wps_stride);
		offboard_wps_damp(wps_damp);

		return offboard_wps_init();
	}
	return 0;
}


int offboard_mode_stop(int blocking)
{
	// start appropriate offboard mode thread
	if(offboard_mode == FIGURE_EIGHT){
		printf("stopping offboard figure eight\n");
		return offboard_figure_eight_stop(blocking);
	}
	else if(offboard_mode == FOLLOW_TAG){
		printf("stopping offboard follow tag\n");
		return offboard_follow_tag_stop(blocking);
	}
	else if(offboard_mode == TRAJECTORY){
		printf("stopping offboard trajectory\n");
		return offboard_trajectory_stop(blocking);
	}
	else if(offboard_mode == VFC){
		printf("stopping offboard vfc\n");
		return offboard_vfc_stop(blocking);
	}
	else if(offboard_mode == BACKTRACK){
		printf("stopping offboard backtrack\n");
		return offboard_backtrack_stop(blocking);
	}
	else if(offboard_mode == WPS){
		printf("stopping offboard wps\n");
		return offboard_wps_stop(blocking);
	}
	return 0;
}

void offboard_mode_en_print_debug(int debug)
{
	offboard_figure_eight_en_print_debug(debug);
	offboard_follow_tag_en_print_debug(debug);
	offboard_trajectory_en_print_debug(debug);
	offboard_vfc_en_print_debug(debug);
	offboard_backtrack_en_print_debug(debug);
	offboard_wps_en_print_debug(debug);
	return;
}
