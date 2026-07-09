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
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace gz_hitl
{

// Bidirectional FC<->QGC MAVLink UDP relay (default port 14550). Follows the same pattern as
// FcLink (one socket, 200ms SO_RCVTIMEO, one RX thread, mutex-protected remote address, stop()
// from the destructor) -- except the relay is passive, so there is no heartbeat thread (FC and
// QGC exchange their own heartbeats directly).
// All errors are non-fatal: a failed open() only logs a warning and the bridge keeps running
// without the GCS relay; a failed sendto() is ignored -- neither affects the HIL path (FC<->gz).
class GcsRelay
{
public:
	GcsRelay() = default;
	~GcsRelay();

	GcsRelay(const GcsRelay &) = delete;
	GcsRelay &operator=(const GcsRelay &) = delete;

	bool open(const std::string &qgc_host, int qgc_port);   // default 127.0.0.1:14550

	void forwardToGcs(const mavlink_message_t &msg);        // FC -> QGC

	// Invoked from the RX thread for every complete message from QGC (e.g. fclink.sendMessage)
	void setToFcHandler(std::function<void(const mavlink_message_t &)> h);

	void start();   // starts the RX thread (no-op if open() failed or was not called)
	void stop();

private:
	void rxLoop();

	std::atomic<bool> _running{false};
	std::thread _rx_thread;

	int _fd{-1};
	sockaddr_in _remote{};
	std::mutex _remote_mutex;   // _remote: written by the RX thread, read by forwardToGcs() from another thread

	std::function<void(const mavlink_message_t &)> _handler;
};

} // namespace gz_hitl
