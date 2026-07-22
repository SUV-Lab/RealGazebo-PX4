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
#include <cstdint>

namespace gz_hitl
{
struct SensorState {
	double acc[3] {};  double gyro[3] {}; bool imu_updated{false}; // FRD, SI
	double mag[3] {};  bool mag_updated{false};                    // FRD, gauss
	double abs_pressure_hpa{0}; double pressure_alt_m{0}; bool baro_updated{false};
	double diff_pressure_hpa{0}; bool diff_updated{false};
	double temperature_c{25.0};
};
struct GpsSample { double lat_deg, lon_deg, alt_m; double vel_n, vel_e, vel_d; };

// Converts raw gz sensor values to SensorState (frame/unit conversions).
void imuToState(const double lin_acc_flu[3], const double ang_vel_flu[3], SensorState &s);
void magToState(const double field_tesla_gz[3], SensorState &s);
void baroToState(double pressure_pa, SensorState &s);
void airspeedToState(double diff_pressure_pa, double temperature_k, SensorState &s);

// Packing into MAVLink messages (consumes the updated flags -- false after the call).
mavlink_hil_sensor_t buildHilSensor(uint64_t time_usec, SensorState &s);
mavlink_hil_gps_t    buildHilGps(uint64_t time_usec, const GpsSample &g);

// Actuator decoding (HIL_ACTUATOR_CONTROLS -> motor angular velocities)
struct MotorCommand { bool armed{false}; double velocity_rad_s[16] {}; unsigned count{0}; };

// Control-surface deflections, in radians, for the servo_N gz topics.
// PX4 normalises non-motor outputs to [-1, 1] (pwm_out_sim/PWMSim.cpp), and
// gz's JointPositionController takes a target angle, so the value is scaled
// by max_angle_rad.
struct ServoCommand { double angle_rad[16] {}; unsigned count{0}; };

// HIL channel layout: <motorJoint> entries first, then <moveableLink> ones.
// The manager derives both counts from the model SDF and the paired
// RealGazebo HITL airframe mirrors them in HIL_ACT_FUNC (motors 101..,
// servos 201..), so the two always describe the same channels.
MotorCommand decodeActuators(const mavlink_hil_actuator_controls_t &m,
			     unsigned num_motors, double max_vel_rad_s);
ServoCommand decodeServos(const mavlink_hil_actuator_controls_t &m,
			  unsigned num_motors, unsigned num_servos,
			  double max_angle_rad);
} // namespace gz_hitl
