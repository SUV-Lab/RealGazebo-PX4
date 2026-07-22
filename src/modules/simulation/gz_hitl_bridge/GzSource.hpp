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

#include "HilCodec.hpp"

#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace gz_hitl
{

// gz-transport sensor source: subscribes to 5 sensor topics (airspeed is optional) plus one
// motor-command publisher. SensorState is protected by stateMutex() (the caller locks it);
// GpsSample is protected by an internal mutex (gpsPop() consumes it atomically).
class GzSource
{
public:
	GzSource(std::string world, std::string model);
	~GzSource();

	// Starts subscriptions/publisher. Subscribe() succeeds even if the gz topic doesn't exist
	// yet; once a publisher appears, callbacks start automatically (no retry logic needed).
	// on_imu(time_usec) is invoked from the IMU callback context (a gz thread) and triggers
	// sending the HIL_SENSOR/HIL_GPS messages.
	bool init(std::function<void(uint64_t)> on_imu, unsigned num_servos = 0);

	// Unsubscribes and tears down the motor publisher (destroying the Node unsubscribes it
	// under gz-transport's internal mutex, so no callback can fire afterward). Idempotent.
	// Also called from the destructor, but call it explicitly (e.g. from main()) while the
	// other members (_mutex/_state/_gps_mutex/_on_imu) are still alive, to avoid a
	// callback-vs-destruction race.
	void stop();

	SensorState &state() { return _state; }        // access while holding stateMutex()
	std::mutex &stateMutex() { return _mutex; }

	bool gpsPop(GpsSample &out);   // true if a new GPS sample is available (consumed once)

	// Silently a no-op if called after stop() -- the publisher has already been torn down.
	void publishMotors(const MotorCommand &cmd);
	void publishMotorsZero(unsigned count);

	// Control surfaces. init() advertises one /model/<model>/servo_N topic per
	// servo, the same names PX4's own gz SITL bridge uses
	// (GZMixingInterfaceServo), so an existing model's JointPositionController
	// blocks are driven unchanged.
	void publishServos(const ServoCommand &cmd);

	bool imuAlive() const;   // true if an IMU callback has fired within the last second

private:
	void imuCallback(const gz::msgs::IMU &msg);
	void magCallback(const gz::msgs::Magnetometer &msg);
	void baroCallback(const gz::msgs::FluidPressure &msg);
	void airspeedCallback(const gz::msgs::AirSpeed &msg);
	void navsatCallback(const gz::msgs::NavSat &msg);

	std::string _world;
	std::string _model;

	// Owned via unique_ptr: stop() calls reset() explicitly, which unsubscribes under
	// gz-transport's internal mutex so no callback fires afterward. Even though this member
	// is declared first (and would otherwise be destroyed last), calling stop() explicitly
	// means cleanup does not depend on destruction order.
	std::unique_ptr<gz::transport::Node> _node;
	gz::transport::Node::Publisher _motor_pub;
	std::vector<gz::transport::Node::Publisher> _servo_pubs;

	SensorState _state;
	std::mutex _mutex;    // protects _state

	std::function<void(uint64_t)> _on_imu;
	std::atomic<std::chrono::steady_clock::time_point> _last_imu{std::chrono::steady_clock::time_point{}};

	GpsSample _gps_latest{};
	bool _gps_fresh{false};
	std::mutex _gps_mutex;   // protects _gps_latest/_gps_fresh
};

} // namespace gz_hitl
