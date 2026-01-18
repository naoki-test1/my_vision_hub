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
#include <modal_json.h>
#include <unistd.h>
#include "horizon_cal_file.h"


#define FILE_HEADER "\
/**\n\
 * VOXL Vision PX4 VIO Horizon Calibration file\n\
 *\n\
 * version: don't touch this, used to keep track of changing config file formats\n\
 *\n\
 */\n"


#define CURRENT_VERSION 1

// file version detection
int horizon_cal_file_version = CURRENT_VERSION;

double vio_reported_roll_when_level_deg = 0.0;
double vio_reported_pitch_when_level_deg = 0.0;



int load_horizon_cal_file(void)
{
	// return if there is no calibration file
	if(access(HORIZON_CAL_FILE_PATH, F_OK) == -1){
		return 0;
	}

	// read the data in
	cJSON* parent = json_read_file(HORIZON_CAL_FILE_PATH);
	if(parent==NULL) return -1;

	// check config file version
	json_fetch_int_with_default(    parent, "horizon_cal_file_version", &horizon_cal_file_version, CURRENT_VERSION);

	json_fetch_double_with_default(  parent, "vio_reported_roll_when_level_deg", &vio_reported_roll_when_level_deg, 0.0);
	json_fetch_double_with_default(  parent, "vio_reported_pitch_when_level_deg", &vio_reported_pitch_when_level_deg, 0.0);

	// write modified data to disk if neccessary
	if(json_get_modified_flag()){
		printf("The vio horizon cal file was modified during parsing, saving the changes to disk\n");
		json_write_to_file_with_header(HORIZON_CAL_FILE_PATH, parent, FILE_HEADER);
	}
	cJSON_Delete(parent);

	return 0;
}


int save_horizon_cal_file(double roll, double pitch)
{
	// new json object to construct from raw data
	cJSON *parent = cJSON_CreateObject();

	cJSON_AddNumberToObject(parent,	"horizon_cal_file_version",	CURRENT_VERSION);
	cJSON_AddNumberToObject(parent,	"vio_reported_roll_when_level_deg",		roll);
	cJSON_AddNumberToObject(parent,	"vio_reported_pitch_when_level_deg",	pitch);


	if(json_write_to_file_with_header(HORIZON_CAL_FILE_PATH, parent, FILE_HEADER)){
		fprintf(stderr, "ERROR failed to write calibration file %s to disk\n", HORIZON_CAL_FILE_PATH);
		return -1;
	}
	cJSON_Delete(parent);

	return 0;
}
