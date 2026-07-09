/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <mavlink.h>
#include "FcLink.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace gz_hitl;

TEST(FcLink, UdpRoundtripAndLinkOk)
{
	// fake FC: bind to 127.0.0.1:24580
	int fc_sock = socket(AF_INET, SOCK_DGRAM, 0);
	ASSERT_GE(fc_sock, 0);
	sockaddr_in fc_addr{};
	fc_addr.sin_family = AF_INET;
	fc_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	fc_addr.sin_port = htons(24580);
	ASSERT_EQ(bind(fc_sock, (sockaddr *)&fc_addr, sizeof(fc_addr)), 0);

	FcLink link;
	ASSERT_TRUE(link.openUdp("127.0.0.1", 24580, 24540));
	std::atomic<int> got_heartbeat{0};
	link.setMessageHandler([&](const mavlink_message_t &m) {
		if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) { got_heartbeat++; }
	});
	link.start();

	// bridge->FC: send one HIL_SENSOR, verify the fake FC receives it
	mavlink_message_t msg;
	mavlink_hil_sensor_t hs{};
	hs.time_usec = 42;
	mavlink_msg_hil_sensor_encode(1, 200, &msg, &hs);
	link.sendMessage(msg);

	uint8_t buf[2048];
	sockaddr_in src{}; socklen_t slen = sizeof(src);
	struct timeval tv {2, 0};
	setsockopt(fc_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	ssize_t n = recvfrom(fc_sock, buf, sizeof(buf), 0, (sockaddr *)&src, &slen);
	ASSERT_GT(n, 0);

	// FC->bridge: reply with a heartbeat, verify the handler fires and linkOk() is true
	mavlink_message_t hb;
	mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0, 0);
	uint8_t out[2048];
	uint16_t len = mavlink_msg_to_send_buffer(out, &hb);
	sendto(fc_sock, out, len, 0, (sockaddr *)&src, slen);

	std::this_thread::sleep_for(std::chrono::milliseconds(300));
	EXPECT_GE(got_heartbeat.load(), 1);
	EXPECT_TRUE(link.linkOk());

	link.stop();
	close(fc_sock);
}
