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

#include "FcLink.hpp"
#include <mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <mutex>

namespace gz_hitl
{

namespace
{

// Maps a baud rate to a termios speed_t constant. Returns false for unsupported rates instead
// of silently falling back to a default, so a typo'd --baud value fails loudly at openSerial()
// time rather than quietly talking to the FC at the wrong rate.
bool baudToSpeed(int baud, speed_t &out)
{
	switch (baud) {
	case 57600:   out = B57600;   return true;

	case 115200:  out = B115200;  return true;

	case 230400:  out = B230400;  return true;

	case 460800:  out = B460800;  return true;

	case 921600:  out = B921600;  return true;

	default:      return false;
	}
}

} // namespace

FcLink::~FcLink()
{
	stop();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

bool FcLink::openUdp(const std::string &fc_host, int fc_port, int local_port)
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) { return false; }

	// 200ms receive timeout so the RX thread never blocks forever (lets stop() return promptly)
	struct timeval tv {0, 200000};
	setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = INADDR_ANY;
	local.sin_port = htons(local_port);

	if (bind(_fd, (sockaddr *)&local, sizeof(local)) < 0) { return false; }

	{
		std::lock_guard<std::mutex> lock(_remote_mutex);
		std::memset(&_remote, 0, sizeof(_remote));
		_remote.sin_family = AF_INET;
		_remote.sin_addr.s_addr = inet_addr(fc_host.c_str());
		_remote.sin_port = htons(fc_port);
	}

	_is_serial = false;
	return true;
}

bool FcLink::openSerial(const std::string &device, int baud)
{
	speed_t sp;

	if (!baudToSpeed(baud, sp)) {
		fprintf(stderr,
			"gz-hitl-bridge: error: unsupported baud rate %d "
			"(supported: 57600/115200/230400/460800/921600)\n", baud);
		return false;
	}

	_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) { return false; }

	termios tio{};
	tcgetattr(_fd, &tio);
	cfmakeraw(&tio);
	cfsetispeed(&tio, sp);
	cfsetospeed(&tio, sp);
	tio.c_cflag |= (CLOCAL | CREAD);

	if (tcsetattr(_fd, TCSANOW, &tio) != 0) { return false; }

	_is_serial = true;
	return true;
}

void FcLink::sendMessage(const mavlink_message_t &msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	if (_is_serial) {
		(void)!::write(_fd, buf, len);

	} else {
		sockaddr_in remote_copy;
		{
			std::lock_guard<std::mutex> lock(_remote_mutex);
			remote_copy = _remote;
		}
		(void)!::sendto(_fd, buf, len, 0, (sockaddr *)&remote_copy, sizeof(remote_copy));
	}
}

void FcLink::setMessageHandler(std::function<void(const mavlink_message_t &)> h)
{
	_handler = std::move(h);
}

void FcLink::rxLoop()
{
	uint8_t buf[2048];
	mavlink_message_t msg;
	mavlink_status_t status;

	while (_running) {
		ssize_t n;

		if (_is_serial) {
			n = ::read(_fd, buf, sizeof(buf));

		} else {
			sockaddr_in src{}; socklen_t slen = sizeof(src);
			n = ::recvfrom(_fd, buf, sizeof(buf), 0, (sockaddr *)&src, &slen);

			if (n > 0) {
				std::lock_guard<std::mutex> lock(_remote_mutex);
				_remote = src;   // learn the FC's address (handles reboot/port changes)
			}
		}

		if (n <= 0) { usleep(2000); continue; }

		_last_rx.store(std::chrono::steady_clock::now());

		for (ssize_t i = 0; i < n; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
				if (_handler) { _handler(msg); }
			}
		}
	}
}

void FcLink::heartbeatLoop()
{
	while (_running) {
		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack_chan(1, 200, MAVLINK_COMM_0, &msg,
						MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, 0);
		sendMessage(msg);

		for (int i = 0; i < 10 && _running; i++) { usleep(100000); }
	}
}

void FcLink::start()
{
	if (_running.exchange(true)) { return; }   // already running -> ignore (prevents duplicate start)

	_rx_thread = std::thread(&FcLink::rxLoop, this);
	_hb_thread = std::thread(&FcLink::heartbeatLoop, this);
}

void FcLink::stop()
{
	_running = false;

	if (_rx_thread.joinable()) { _rx_thread.join(); }

	if (_hb_thread.joinable()) { _hb_thread.join(); }
}

bool FcLink::linkOk() const
{
	return (std::chrono::steady_clock::now() - _last_rx.load()) < std::chrono::seconds(3);
}

} // namespace gz_hitl
