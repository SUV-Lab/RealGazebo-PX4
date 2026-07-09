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

#include "GcsRelay.hpp"
#include <mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

namespace gz_hitl
{

GcsRelay::~GcsRelay()
{
	stop();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

bool GcsRelay::open(const std::string &qgc_host, int qgc_port)
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		fprintf(stderr, "gz-hitl-bridge: warn: GcsRelay: failed to create socket -> no QGC relay\n");
		return false;
	}

	// 200ms receive timeout so the RX thread never blocks forever (lets stop() return promptly)
	struct timeval tv {0, 200000};
	setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	// Local bind is skipped (ephemeral port) -- sendto() to QGC lets the kernel assign a source
	// port, QGC replies to that port, and rxLoop() learns _remote from it so the round trip continues.

	const in_addr_t addr = inet_addr(qgc_host.c_str());

	if (addr == INADDR_NONE) {
		fprintf(stderr, "gz-hitl-bridge: warn: GcsRelay: invalid qgc host '%s' -> no QGC relay\n",
			qgc_host.c_str());
		::close(_fd);
		_fd = -1;
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(_remote_mutex);
		std::memset(&_remote, 0, sizeof(_remote));
		_remote.sin_family = AF_INET;
		_remote.sin_addr.s_addr = addr;
		_remote.sin_port = htons(qgc_port);
	}

	printf("gz-hitl-bridge: GcsRelay: relaying to QGC at %s:%d\n", qgc_host.c_str(), qgc_port);
	return true;
}

void GcsRelay::forwardToGcs(const mavlink_message_t &msg)
{
	if (_fd < 0) { return; }   // open() failed/was not called -> silently drop, no relay (non-fatal)

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	sockaddr_in remote_copy;
	{
		std::lock_guard<std::mutex> lock(_remote_mutex);
		remote_copy = _remote;
	}

	// sendto() failure is ignored -- has no effect on the HIL path (FC<->gz), non-fatal
	(void)!::sendto(_fd, buf, len, 0, (sockaddr *)&remote_copy, sizeof(remote_copy));
}

void GcsRelay::setToFcHandler(std::function<void(const mavlink_message_t &)> h)
{
	_handler = std::move(h);
}

void GcsRelay::rxLoop()
{
	uint8_t buf[2048];
	mavlink_message_t msg;
	mavlink_status_t status;

	while (_running) {
		sockaddr_in src{}; socklen_t slen = sizeof(src);
		ssize_t n = ::recvfrom(_fd, buf, sizeof(buf), 0, (sockaddr *)&src, &slen);

		if (n > 0) {
			std::lock_guard<std::mutex> lock(_remote_mutex);
			_remote = src;   // learn QGC's address (reflects its reply source port)
		}

		if (n <= 0) { usleep(2000); continue; }

		// Uses a channel (MAVLINK_COMM_1) distinct from FcLink::rxLoop()'s as a defensive
		// choice. mavlink_parse_char() is generated as "static inline", so each translation
		// unit (this file vs FcLink.cpp) already gets its own copy of the per-channel parser
		// state; the separate channel is extra insurance, not the actual reason the two RX
		// threads don't interfere with each other.
		for (ssize_t i = 0; i < n; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status)) {
				if (_handler) { _handler(msg); }
			}
		}
	}
}

void GcsRelay::start()
{
	if (_fd < 0) {
		fprintf(stderr, "gz-hitl-bridge: warn: GcsRelay: not started (open() failed/not called) -> no QGC relay\n");
		return;
	}

	if (_running.exchange(true)) { return; }   // already running -> ignore (prevents duplicate start)

	_rx_thread = std::thread(&GcsRelay::rxLoop, this);
}

void GcsRelay::stop()
{
	_running = false;

	if (_rx_thread.joinable()) { _rx_thread.join(); }
}

} // namespace gz_hitl
