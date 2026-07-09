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

#pragma once

#include <mavlink.h>
#include <netinet/in.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace gz_hitl
{

// MAVLink link to the FC (flight controller): UDP or serial, framing, 1Hz heartbeat.
class FcLink
{
public:
	FcLink() = default;
	~FcLink();

	FcLink(const FcLink &) = delete;
	FcLink &operator=(const FcLink &) = delete;

	bool openUdp(const std::string &fc_host, int fc_port, int local_port);
	bool openSerial(const std::string &device, int baud);

	void sendMessage(const mavlink_message_t &msg);

	// Invoked from the RX thread for every complete message.
	// Must be called before start() -- the handler is not synchronized against a running RX thread.
	void setMessageHandler(std::function<void(const mavlink_message_t &)> h);

	void start();   // starts the RX thread + 1Hz heartbeat (sysid=1, compid=200)
	void stop();

	bool linkOk() const;   // true if there has been reception from the FC within the last 3 seconds

private:
	void rxLoop();
	void heartbeatLoop();

	std::atomic<bool> _running{false};
	std::thread _rx_thread;
	std::thread _hb_thread;

	std::atomic<std::chrono::steady_clock::time_point> _last_rx{std::chrono::steady_clock::time_point{}};

	int _fd{-1};
	bool _is_serial{false};
	sockaddr_in _remote{};
	std::mutex _remote_mutex;   // _remote: written by rxLoop, read by sendMessage() -- prevents a race

	std::function<void(const mavlink_message_t &)> _handler;
};

} // namespace gz_hitl
