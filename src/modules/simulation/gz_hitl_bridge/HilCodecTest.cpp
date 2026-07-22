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
#include "HilCodec.hpp"

using namespace gz_hitl;

TEST(HilCodec, ImuFluToFrd)
{
	SensorState s;
	const double acc[3] = {1.0, 2.0, -9.8};
	const double gyro[3] = {0.1, 0.2, 0.3};
	imuToState(acc, gyro, s);
	EXPECT_DOUBLE_EQ(s.acc[0], 1.0);
	EXPECT_DOUBLE_EQ(s.acc[1], -2.0);
	EXPECT_DOUBLE_EQ(s.acc[2], 9.8);
	EXPECT_DOUBLE_EQ(s.gyro[1], -0.2);
	EXPECT_TRUE(s.imu_updated);
}

TEST(HilCodec, MagGzAxesToFrd)
{
	SensorState s;
	// gz's magnetometer plugin already publishes in gauss (despite the field_tesla name) using a
	// left-handed coordinate system; only the axis remap applies here: HIL axes = (-y, -x, z).
	const double gauss[3] = {1e-5, 2e-5, 3e-5};
	magToState(gauss, s);
	EXPECT_NEAR(s.mag[0], -2e-5, 1e-9);
	EXPECT_NEAR(s.mag[1], -1e-5, 1e-9);
	EXPECT_NEAR(s.mag[2],  3e-5, 1e-9);
}

TEST(HilCodec, BaroPaToHpaAndIsaAlt)
{
	SensorState s;
	baroToState(101325.0, s);              // standard sea-level pressure
	EXPECT_NEAR(s.abs_pressure_hpa, 1013.25, 0.01);
	EXPECT_NEAR(s.pressure_alt_m, 0.0, 0.5);
	baroToState(89874.6, s);               // ISA ~1000 m
	EXPECT_NEAR(s.pressure_alt_m, 1000.0, 10.0);
}

TEST(HilCodec, AirspeedPaToHpaTempKtoC)
{
	SensorState s;
	airspeedToState(200.0, 298.15, s);
	EXPECT_NEAR(s.diff_pressure_hpa, 2.0, 1e-9);
	EXPECT_NEAR(s.temperature_c, 25.0, 1e-6);
}

TEST(HilCodec, BuildHilSensorBitmaskAndConsume)
{
	SensorState s;
	const double acc[3] = {0, 0, -9.8}; const double gyro[3] = {0, 0, 0};
	imuToState(acc, gyro, s);
	baroToState(101325.0, s);
	mavlink_hil_sensor_t m = buildHilSensor(123456, s);
	EXPECT_EQ(m.time_usec, 123456u);
	EXPECT_EQ(m.fields_updated & 0x0007u, 0x0007u);   // ACCEL
	EXPECT_EQ(m.fields_updated & 0x0038u, 0x0038u);   // GYRO
	EXPECT_EQ(m.fields_updated & 0x0A00u, 0x0A00u);   // ABS_PRESSURE|PRESSURE_ALT
	EXPECT_EQ(m.fields_updated & 0x01C0u, 0u);        // MAG not updated
	EXPECT_FLOAT_EQ(m.zacc, 9.8f);                    // FRD
	// verify flags are consumed
	mavlink_hil_sensor_t m2 = buildHilSensor(123457, s);
	EXPECT_EQ(m2.fields_updated, 0u);
}

TEST(HilCodec, BuildHilGpsUnits)
{
	GpsSample g{47.397742, 8.545594, 488.0, 1.0, 2.0, -0.5};
	mavlink_hil_gps_t m = buildHilGps(1000, g);
	EXPECT_EQ(m.lat, 473977420);
	EXPECT_EQ(m.lon, 85455940);
	EXPECT_EQ(m.alt, 488000);
	EXPECT_EQ(m.vn, 100); EXPECT_EQ(m.ve, 200); EXPECT_EQ(m.vd, -50);
	EXPECT_EQ(m.fix_type, 3);
	EXPECT_EQ(m.eph, 100); EXPECT_EQ(m.epv, 178);
	EXPECT_EQ(m.satellites_visible, 10);
	// cog: atan2(ve,vn)=63.43° → cdeg
	EXPECT_NEAR(m.cog, 6343, 2);
}

TEST(HilCodec, DecodeActuatorsArmed)
{
	mavlink_hil_actuator_controls_t in{};
	in.mode = 128 | 4;                    // MAV_MODE_FLAG_SAFETY_ARMED | other bits

	for (int i = 0; i < 4; i++) { in.controls[i] = 0.7f; }

	MotorCommand c = decodeActuators(in, 4, 1000.0);
	EXPECT_TRUE(c.armed);
	EXPECT_EQ(c.count, 4u);
	// mavlink controls[] is float, so 0.7f is itself already an approximation of 700.0
	// (0.7f == 0.699999988...) -- the 1e-3 tolerance only absorbs that float32 rounding
	// error, and still catches a logic bug (e.g. a wrong scale constant).
	EXPECT_NEAR(c.velocity_rad_s[0], 700.0, 1e-3);
	EXPECT_NEAR(c.velocity_rad_s[3], 700.0, 1e-3);
}

TEST(HilCodec, DecodeActuatorsDisarmedZero)
{
	mavlink_hil_actuator_controls_t in{};
	in.mode = 4;                          // not armed
	in.controls[0] = 0.9f;
	MotorCommand c = decodeActuators(in, 4, 1000.0);
	EXPECT_FALSE(c.armed);
	EXPECT_DOUBLE_EQ(c.velocity_rad_s[0], 0.0);
}

TEST(HilCodec, DecodeActuatorsClampNegative)
{
	mavlink_hil_actuator_controls_t in{};
	in.mode = 128;
	in.controls[0] = -0.3f;               // motors are [0,1] -- guard against negative values
	MotorCommand c = decodeActuators(in, 1, 1000.0);
	EXPECT_DOUBLE_EQ(c.velocity_rad_s[0], 0.0);
}

// -- control surfaces -----------------------------------------------------

TEST(HilCodec, decodeServosTakesTheChannelsAfterTheMotors)
{
	// lc_62 layout: 8 motors on channels 0..7, 5 control surfaces on 8..12
	mavlink_hil_actuator_controls_t in{};
	in.mode = 128;   // armed

	for (int i = 0; i < 8; i++) { in.controls[i] = 0.5f; }

	in.controls[8]  = 1.0f;    // full positive deflection
	in.controls[9]  = -1.0f;   // full negative
	in.controls[10] = 0.0f;    // centred
	in.controls[11] = 0.5f;
	in.controls[12] = -0.25f;

	ServoCommand c = decodeServos(in, 8, 5, 0.4);
	ASSERT_EQ(c.count, 5u);
	EXPECT_DOUBLE_EQ(c.angle_rad[0], 0.4);
	EXPECT_DOUBLE_EQ(c.angle_rad[1], -0.4);
	EXPECT_DOUBLE_EQ(c.angle_rad[2], 0.0);
	EXPECT_DOUBLE_EQ(c.angle_rad[3], 0.2);
	EXPECT_DOUBLE_EQ(c.angle_rad[4], -0.1);
}

TEST(HilCodec, decodeServosIsEmptyForAMultirotor)
{
	mavlink_hil_actuator_controls_t in{};
	in.mode = 128;
	EXPECT_EQ(decodeServos(in, 4, 0, 0.4).count, 0u);
}

TEST(HilCodec, decodeServosClampsToTheSixteenHilChannels)
{
	mavlink_hil_actuator_controls_t in{};
	in.mode = 128;
	// 12 motors leave only 4 channels, however many servos are claimed
	EXPECT_EQ(decodeServos(in, 12, 8, 0.4).count, 4u);
}

TEST(HilCodec, decodeServosDeflectsWhileDisarmed)
{
	// unlike motors, surfaces are not a hazard: the FC still owns them
	mavlink_hil_actuator_controls_t in{};
	in.mode = 0;   // disarmed
	in.controls[4] = 1.0f;
	ServoCommand c = decodeServos(in, 4, 1, 0.4);
	ASSERT_EQ(c.count, 1u);
	EXPECT_DOUBLE_EQ(c.angle_rad[0], 0.4);
}
