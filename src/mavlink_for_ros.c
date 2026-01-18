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
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>	// for  close
#include <errno.h>

#include "mavlink_for_ros.h"
#include "mavlink_io.h"
#include "config_file.h"
// #include "geometry.h"
#include "offboard_mode.h"
#include "autopilot_monitor.h"

#define UDP_READ_BUF_LEN	(32*1024)
#define MAV_CHAN			0
#define MAX_CONNECTIONS		16
#define LOCAL_ADDRESS_INT	2130706433


static int running;
static int sockfd_local;
static struct sockaddr_in si_me_local;  // my sockaddr for talking to localhost
static struct timeval tv; // for recv timeout
static pthread_t recv_thread_id_local;
static int print_debug_send;
static int print_debug_recv;


// this function is called once a complete message is parsed from either udp port
static void _message_handler(mavlink_message_t* msg)
{
	// send signal to stop our own offboard mode if something else is trying to
	// take over
	if( msg->msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED ||\
		msg->msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT){
		offboard_mode_stop(0); // 0 == nonblocking mode!
	}

	// if enabled, translate position setpoints to local frame
	if(msg->msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED && \
		en_transform_mavlink_pos_setpoints_from_fixed_frame){
		//printf("converting setpoint to local frame, sys %d comp: %d\n", msg->sysid, msg->compid);
		mavlink_set_position_target_local_ned_t sp;
		uint8_t sysid=msg->sysid;
		uint8_t compid=msg->compid;
		mavlink_msg_set_position_target_local_ned_decode(msg, &sp);
		mavlink_io_send_fixed_setpoint(sysid,compid,sp);
		return;
	}

	// every other message, just send to autopilot
	mavlink_io_send_msg_to_ap(msg);
	return;
}


void mavlink_for_ros_en_print_debug_send(int en_print_debug)
{
	printf("Enabling send debugging for localhost UDP port to ROS\n");
	print_debug_send = en_print_debug;
	return;
}

void mavlink_for_ros_en_print_debug_recv(int en_print_debug)
{
	printf("Enabling recv debugging for localhost UDP port from ROS\n");
	print_debug_recv = en_print_debug;
	return;
}


int mavlink_for_ros_send_msg(mavlink_message_t* msg)
{
	// nothing to do if shutting down
	if(!running) return -1;

	// basic debugging help
	if(print_debug_send){
		printf("to ROS: msg id: %3d\n", msg->msgid);
	}

	// unpack message into a buffer ready to send
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int new_msg_len = mavlink_msg_to_send_buffer(buf, msg);

	// mavros doesn't play nice with multiple mavlink packets in one UDP packet
	// so for localhost port just send one packet per mavlink message.
	sendto(sockfd_local, buf, new_msg_len, MSG_CONFIRM, \
					(const struct sockaddr*) &si_me_local, sizeof(si_me_local));

	return 0;
}


// thread to read local port localhost_udp_port_number
static void* __recv_local_thread_func(__attribute__((unused)) void *vargp)
{
	// general local variables
	int i, bytes_read;
	mavlink_message_t msg;
	mavlink_status_t status;
	int msg_received = 0;
	char buf[UDP_READ_BUF_LEN];
	struct sockaddr_in si_other; // don't look at this since we expect to just read from localhost
	socklen_t slen = sizeof(si_other);

	// keep going until mavlink_for_ros_stop sets running to 0
	while(running){

		// Receive UDP message from ground control, this is blocking with timeout
		errno = 0;
		bytes_read = recvfrom(sockfd_local, buf, UDP_READ_BUF_LEN, MSG_WAITALL,\
								(struct sockaddr*)&si_other, &slen);
		// ignore EAGAIN error, that just means the timeout worked
		if(bytes_read<0 && errno!=EAGAIN){
			perror("ERROR: UDP recvfrom local had a problem");
			usleep(1000000);
			continue;
		}

		// do the mavlink byte-by-byte parsing
		for(i=0; i<bytes_read; i++){
			msg_received = mavlink_parse_char(MAV_CHAN, buf[i], &msg, &status);

			// check for dropped packets
			if(status.packet_rx_drop_count != 0){
				fprintf(stderr,"WARNING: ROS UDP local listener dropped %d packets\n", status.packet_rx_drop_count);
			}

			// msg_received indicates this byte was the end of a complete packet
			if(msg_received){
				if(print_debug_recv){
					printf("From ROS msg ID: %3d sysid:%3d from port:%6d IP: %s \n",\
							msg.msgid, msg.sysid, ntohs(si_other.sin_port), \
							inet_ntoa(si_other.sin_addr));
				}
				_message_handler(&msg);
			}
		}
	}

	printf("exiting mavlink for ros localhost udp listener thread\n");
	return NULL;
}


int mavlink_for_ros_init(void)
{
	// set up new socket
	sockfd_local = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(sockfd_local < 0) {
		perror("ERROR: local UDP socket creation failed");
		return -1;
	}

	// set timeout for the socket
	tv.tv_sec = 0;
	tv.tv_usec = 500000;
	setsockopt(sockfd_local, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

	// set up my own sockaddr
	si_me_local.sin_family = AF_INET; // IPv4
	si_me_local.sin_port = htons(localhost_udp_port_number);
	si_me_local.sin_addr.s_addr = inet_addr("127.0.0.1");

	running = 1;
	pthread_create(&recv_thread_id_local, NULL, __recv_local_thread_func, NULL);

	return 0;
}


int mavlink_for_ros_stop(void)
{
	if(running==0){
		return 0;
	}

	running=0;
	pthread_join(recv_thread_id_local, NULL);
	close(sockfd_local);

	printf("udp mavlink for ros stopped\n");
	return 0;
}
