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

#include "HilCodec.hpp"
#include <mavlink.h>
#include <cmath>
#include <cstring>

namespace gz_hitl
{

void imuToState(const double a[3], const double w[3], SensorState &s)
{
	// FLU -> FRD: 180° about X
	s.acc[0] = a[0]; s.acc[1] = -a[1]; s.acc[2] = -a[2];
	s.gyro[0] = w[0]; s.gyro[1] = -w[1]; s.gyro[2] = -w[2];
	s.imu_updated = true;
}

void magToState(const double t[3], SensorState &s)
{
	// FIXME: once we're on jetty or later
	// The magnetometer plugin publishes in units of gauss (despite the field_tesla name) and in a
	// weird left-handed coordinate system.
	// https://github.com/gazebosim/gz-sim/pull/2460
	s.mag[0] = -t[1]; s.mag[1] = -t[0]; s.mag[2] = t[2];
	s.mag_updated = true;
}

void baroToState(double p_pa, SensorState &s)
{
	s.abs_pressure_hpa = p_pa / 100.0;
	s.pressure_alt_m = 44330.0 * (1.0 - std::pow(p_pa / 101325.0, 0.190295));
	s.baro_updated = true;
}

void airspeedToState(double dp_pa, double temp_k, SensorState &s)
{
	s.diff_pressure_hpa = dp_pa / 100.0;
	s.temperature_c = temp_k - 273.15;
	s.diff_updated = true;
}

mavlink_hil_sensor_t buildHilSensor(uint64_t time_usec, SensorState &s)
{
	mavlink_hil_sensor_t m;
	std::memset(&m, 0, sizeof(m));
	m.time_usec = time_usec;
	m.id = 0;

	if (s.imu_updated) {
		m.xacc = s.acc[0]; m.yacc = s.acc[1]; m.zacc = s.acc[2];
		m.xgyro = s.gyro[0]; m.ygyro = s.gyro[1]; m.zgyro = s.gyro[2];
		m.fields_updated |= 0x0007u | 0x0038u;
		s.imu_updated = false;
	}

	if (s.mag_updated) {
		m.xmag = s.mag[0]; m.ymag = s.mag[1]; m.zmag = s.mag[2];
		m.fields_updated |= 0x01C0u;
		s.mag_updated = false;
	}

	if (s.baro_updated) {
		m.abs_pressure = s.abs_pressure_hpa;
		m.pressure_alt = s.pressure_alt_m;
		m.temperature = s.temperature_c;
		m.fields_updated |= 0x0200u | 0x0800u | 0x1000u;
		s.baro_updated = false;
	}

	if (s.diff_updated) {
		m.diff_pressure = s.diff_pressure_hpa;
		m.fields_updated |= 0x0400u;
		s.diff_updated = false;
	}

	return m;
}

mavlink_hil_gps_t buildHilGps(uint64_t time_usec, const GpsSample &g)
{
	mavlink_hil_gps_t m;
	std::memset(&m, 0, sizeof(m));
	m.time_usec = time_usec;
	m.lat = static_cast<int32_t>(std::llround(g.lat_deg * 1e7));
	m.lon = static_cast<int32_t>(std::llround(g.lon_deg * 1e7));
	m.alt = static_cast<int32_t>(std::llround(g.alt_m * 1000.0));
	m.eph = 100; m.epv = 178;
	m.vn = static_cast<int16_t>(std::llround(g.vel_n * 100.0));
	m.ve = static_cast<int16_t>(std::llround(g.vel_e * 100.0));
	m.vd = static_cast<int16_t>(std::llround(g.vel_d * 100.0));
	m.vel = static_cast<uint16_t>(std::llround(std::sqrt(g.vel_n * g.vel_n + g.vel_e * g.vel_e) * 100.0));
	double cog = std::atan2(g.vel_e, g.vel_n) * 180.0 / M_PI;

	if (cog < 0) { cog += 360.0; }

	m.cog = static_cast<uint16_t>(std::llround(cog * 100.0));
	m.fix_type = 3;
	m.satellites_visible = 10;
	m.id = 0;
	return m;
}

MotorCommand decodeActuators(const mavlink_hil_actuator_controls_t &m,
			     unsigned num_motors, double max_vel_rad_s)
{
	MotorCommand c;
	c.armed = (m.mode & 128u) != 0;       // MAV_MODE_FLAG_SAFETY_ARMED
	c.count = (num_motors > 16u) ? 16u : num_motors;

	for (unsigned i = 0; i < c.count; i++) {
		double v = c.armed ? static_cast<double>(m.controls[i]) * max_vel_rad_s : 0.0;
		c.velocity_rad_s[i] = (v > 0.0) ? v : 0.0;
	}

	return c;
}

} // namespace gz_hitl
