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

#include "GzSource.hpp"

#include <cstdio>

namespace gz_hitl
{

GzSource::GzSource(std::string world, std::string model) :
	_world(std::move(world)),
	_model(std::move(model))
{
}

GzSource::~GzSource()
{
	stop();
}

bool GzSource::init(std::function<void(uint64_t)> on_imu, unsigned num_servos)
{
	_on_imu = std::move(on_imu);

	_node = std::make_unique<gz::transport::Node>();

	const std::string sensor_base = "/world/" + _world + "/model/" + _model + "/link/base_link/sensor/";

	const std::string imu_topic = sensor_base + "imu_sensor/imu";

	if (!_node->Subscribe(imu_topic, &GzSource::imuCallback, this)) {
		fprintf(stderr, "gz-hitl-bridge: warn: failed to subscribe %s\n", imu_topic.c_str());
	}

	const std::string mag_topic = sensor_base + "magnetometer_sensor/magnetometer";

	if (!_node->Subscribe(mag_topic, &GzSource::magCallback, this)) {
		fprintf(stderr, "gz-hitl-bridge: warn: failed to subscribe %s\n", mag_topic.c_str());
	}

	const std::string baro_topic = sensor_base + "air_pressure_sensor/air_pressure";

	if (!_node->Subscribe(baro_topic, &GzSource::baroCallback, this)) {
		fprintf(stderr, "gz-hitl-bridge: warn: failed to subscribe %s\n", baro_topic.c_str());
	}

	const std::string gps_topic = sensor_base + "navsat_sensor/navsat";

	if (!_node->Subscribe(gps_topic, &GzSource::navsatCallback, this)) {
		fprintf(stderr, "gz-hitl-bridge: warn: failed to subscribe %s\n", gps_topic.c_str());
	}

	// optional: failure only warns (fixed-wing/no-airspeed models legitimately lack this topic)
	const std::string airspeed_topic = "/world/" + _world + "/model/" + _model +
					   "/link/airspeed_link/sensor/air_speed/air_speed";

	if (!_node->Subscribe(airspeed_topic, &GzSource::airspeedCallback, this)) {
		fprintf(stderr, "gz-hitl-bridge: warn: failed to subscribe %s (optional)\n", airspeed_topic.c_str());
	}

	const std::string motor_topic = "/" + _model + "/command/motor_speed";
	_motor_pub = _node->Advertise<gz::msgs::Actuators>(motor_topic);

	for (unsigned i = 0; i < num_servos; i++) {
		// matches PX4's gz SITL bridge: /model/<model>/servo_N carrying a Double angle
		const std::string servo_topic = "/model/" + _model + "/servo_" + std::to_string(i);
		_servo_pubs.push_back(_node->Advertise<gz::msgs::Double>(servo_topic));

		if (!_servo_pubs.back().Valid()) {
			fprintf(stderr, "gz-hitl-bridge: error: failed to advertise %s\n", servo_topic.c_str());
			return false;
		}
	}

	if (!_motor_pub.Valid()) {
		fprintf(stderr, "gz-hitl-bridge: error: failed to advertise %s\n", motor_topic.c_str());
		return false;
	}

	return true;
}

void GzSource::stop()
{
	// Invalidate the publisher first -> publishMotors() silently becomes a no-op from here on.
	_motor_pub = gz::transport::Node::Publisher();
	_servo_pubs.clear();

	// Destroying the Node unsubscribes it under gz-transport's internal mutex, so no more
	// imu/mag/baro/navsat callbacks can fire for this object after this call returns
	// (gz-transport does not spawn a per-Node thread to join here -- the guarantee comes from
	// the mutex-protected unsubscribe, not a thread join). That makes it safe for this
	// function's caller to destroy the other members (_mutex/_state/_gps_mutex/_on_imu)
	// once this function returns.
	_node.reset();
}

void GzSource::imuCallback(const gz::msgs::IMU &msg)
{
	const uint64_t time_usec = msg.header().stamp().sec() * 1000000ULL + msg.header().stamp().nsec() / 1000ULL;

	const double lin_acc[3] = {msg.linear_acceleration().x(), msg.linear_acceleration().y(), msg.linear_acceleration().z()};
	const double ang_vel[3] = {msg.angular_velocity().x(), msg.angular_velocity().y(), msg.angular_velocity().z()};

	{
		std::lock_guard<std::mutex> lock(_mutex);
		imuToState(lin_acc, ang_vel, _state);
	}

	_last_imu.store(std::chrono::steady_clock::now());

	if (_on_imu) {
		_on_imu(time_usec);
	}
}

void GzSource::magCallback(const gz::msgs::Magnetometer &msg)
{
	const double field_tesla[3] = {msg.field_tesla().x(), msg.field_tesla().y(), msg.field_tesla().z()};

	std::lock_guard<std::mutex> lock(_mutex);
	magToState(field_tesla, _state);
}

void GzSource::baroCallback(const gz::msgs::FluidPressure &msg)
{
	std::lock_guard<std::mutex> lock(_mutex);
	baroToState(msg.pressure(), _state);
}

void GzSource::airspeedCallback(const gz::msgs::AirSpeed &msg)
{
	std::lock_guard<std::mutex> lock(_mutex);
	airspeedToState(msg.diff_pressure(), msg.temperature(), _state);
}

void GzSource::navsatCallback(const gz::msgs::NavSat &msg)
{
	GpsSample g;
	g.lat_deg = msg.latitude_deg();
	g.lon_deg = msg.longitude_deg();
	g.alt_m = msg.altitude();
	g.vel_n = msg.velocity_north();
	g.vel_e = msg.velocity_east();
	g.vel_d = -msg.velocity_up();

	std::lock_guard<std::mutex> lock(_gps_mutex);
	_gps_latest = g;
	_gps_fresh = true;
}

bool GzSource::gpsPop(GpsSample &out)
{
	std::lock_guard<std::mutex> lock(_gps_mutex);

	if (_gps_fresh) {
		out = _gps_latest;
		_gps_fresh = false;
		return true;
	}

	return false;
}

void GzSource::publishMotors(const MotorCommand &cmd)
{
	if (!_motor_pub.Valid()) {
		return;
	}

	gz::msgs::Actuators msg;
	auto *vel = msg.mutable_velocity();
	vel->Reserve(static_cast<int>(cmd.count));

	for (unsigned i = 0; i < cmd.count; i++) {
		vel->Add(cmd.velocity_rad_s[i]);
	}

	_motor_pub.Publish(msg);
}

void GzSource::publishServos(const ServoCommand &cmd)
{
	const unsigned n = (cmd.count < _servo_pubs.size()) ? cmd.count
			   : static_cast<unsigned>(_servo_pubs.size());

	for (unsigned i = 0; i < n; i++) {
		if (!_servo_pubs[i].Valid()) { continue; }

		gz::msgs::Double msg;
		msg.set_data(cmd.angle_rad[i]);
		_servo_pubs[i].Publish(msg);
	}
}

void GzSource::publishMotorsZero(unsigned count)
{
	MotorCommand cmd;
	cmd.armed = false;
	cmd.count = (count > 16u) ? 16u : count;
	// velocity_rad_s is already zero-initialized by MotorCommand's default constructor
	publishMotors(cmd);
}

bool GzSource::imuAlive() const
{
	return (std::chrono::steady_clock::now() - _last_imu.load()) < std::chrono::seconds(1);
}

} // namespace gz_hitl
