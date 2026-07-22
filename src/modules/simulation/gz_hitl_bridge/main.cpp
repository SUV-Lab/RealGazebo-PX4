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

// gz-hitl-bridge: standalone daemon. Forwards gz-transport sensor data to the FC as MAVLink
// HIL_SENSOR/HIL_GPS, and publishes the FC's HIL_ACTUATOR_CONTROLS to gz motor topics.

#include "FcLink.hpp"
#include "GcsRelay.hpp"
#include "GzSource.hpp"
#include "HilCodec.hpp"

#include <mavlink.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

using namespace gz_hitl;

namespace
{

struct Options {
	std::string model;
	std::string world{"default"};

	bool use_udp{false};
	std::string udp_host;
	int udp_port{0};

	std::string device;
	int baud{921600};

	int local_port{14540};

	unsigned motors{8};
	double motor_max_vel{1000.0};

	// Control surfaces following the motors on the HIL channels. The manager
	// counts <moveableLink> in the model SDF; 0 for a plain multirotor.
	unsigned servos{0};
	double servo_max_angle{0.5};   // rad at full [-1,1] deflection

	// MAVLink system id stamped on everything the bridge sends to the FC.
	// The manager passes vehicle_id + 1 (PX4's own convention: SITL's rcS
	// sets MAV_SYS_ID = instance + 1). Without a distinct id per vehicle,
	// every bridge in a multi-HITL fleet transmits as system 1 and a GCS
	// merges them into a single vehicle.
	int sysid{1};

	// The FC<->QGC relay is OPT-IN: it is the only way QGC can see a
	// serial-linked FC, but it is pure duplication when the FC reaches QGC
	// on its own (ethernet FC with a separate GCS MAVLink instance), where
	// it costs bridge CPU and shows QGC a second link to the same vehicle.
	bool qgc_relay{false};
	std::string qgc_host;
	int qgc_port{0};
};

std::atomic<bool> g_run{true};

// Set by the RX thread when the FC announces a system id different from the
// one we were configured with (--sysid). Holds the FC's id; 0 = no mismatch.
// A mismatch is a misconfiguration, not a transient: QGC would list the
// bridge and the FC as two systems, and on the MAV_USEHILGPS path PX4
// silently drops HIL_GPS whose sysid is not its own. The main loop turns
// this into a loud exit rather than letting the run continue half-broken.
std::atomic<int> g_fc_sysid_mismatch{0};

void handleSigint(int /*sig*/)
{
	g_run = false;
}

void printUsage(const char *prog)
{
	fprintf(stderr,
		"usage: %s --model <name> [--world default] [--udp <host:port> | --device </dev/ttyACM0>]\n"
		"       [--baud 921600] [--local-port 14540] [--motors 8] [--motor-max-vel 1000.0]\n"
		"       [--sysid 1] [--servos 0] [--servo-max-angle 0.5]\n"
		"       [--qgc <host:port>]   (omit to disable the QGC relay)\n",
		prog);
}

bool splitHostPort(const std::string &s, std::string &host, int &port)
{
	const size_t p = s.find(':');

	if (p == std::string::npos || p == 0 || p + 1 >= s.size()) {
		return false;
	}

	host = s.substr(0, p);
	port = std::atoi(s.substr(p + 1).c_str());
	return port > 0;
}

// Strict numeric parsing for CLI arguments: unlike atoi/atof, rejects empty or partially-numeric
// input (e.g. "abc", "12x") instead of silently returning 0.
bool parseInt(const std::string &s, int &out)
{
	if (s.empty()) { return false; }

	char *end = nullptr;
	const long v = std::strtol(s.c_str(), &end, 10);

	if (end != s.c_str() + s.size()) { return false; }

	out = static_cast<int>(v);
	return true;
}

bool parseDouble(const std::string &s, double &out)
{
	if (s.empty()) { return false; }

	char *end = nullptr;
	const double v = std::strtod(s.c_str(), &end);

	if (end != s.c_str() + s.size()) { return false; }

	out = v;
	return true;
}

bool parseArgs(int argc, char **argv, Options &o)
{
	for (int i = 1; i < argc; i++) {
		const std::string arg = argv[i];

		auto next = [&]() -> std::string {
			return (i + 1 < argc) ? std::string(argv[++i]) : std::string();
		};

		if (arg == "--model") {
			o.model = next();

		} else if (arg == "--world") {
			o.world = next();

		} else if (arg == "--udp") {
			if (!splitHostPort(next(), o.udp_host, o.udp_port)) {
				return false;
			}

			o.use_udp = true;

		} else if (arg == "--device") {
			o.device = next();

		} else if (arg == "--baud") {
			if (!parseInt(next(), o.baud)) { return false; }

		} else if (arg == "--local-port") {
			if (!parseInt(next(), o.local_port)) { return false; }

		} else if (arg == "--motors") {
			int motors;

			if (!parseInt(next(), motors) || motors < 1 || motors > 16) { return false; }

			o.motors = static_cast<unsigned>(motors);

		} else if (arg == "--motor-max-vel") {
			if (!parseDouble(next(), o.motor_max_vel)) { return false; }

		} else if (arg == "--servos") {
			int v = 0;

			if (!parseInt(next(), v) || v < 0 || v > 16) { return false; }

			o.servos = static_cast<unsigned>(v);

		} else if (arg == "--servo-max-angle") {
			if (!parseDouble(next(), o.servo_max_angle) || o.servo_max_angle <= 0.0) {
				return false;
			}

		} else if (arg == "--sysid") {
			if (!parseInt(next(), o.sysid) || o.sysid < 1 || o.sysid > 255) {
				return false;
			}

		} else if (arg == "--qgc") {
			if (!splitHostPort(next(), o.qgc_host, o.qgc_port)) {
				return false;
			}

			o.qgc_relay = true;

		} else {
			return false;
		}
	}

	if (o.model.empty()) {
		return false;
	}

	return o.use_udp || !o.device.empty();
}

} // namespace

int main(int argc, char **argv)
{
	Options o;

	if (!parseArgs(argc, argv, o)) {
		printUsage(argv[0]);
		return 1;
	}

	FcLink fclink;
	const bool link_opened = o.use_udp
				 ? fclink.openUdp(o.udp_host, o.udp_port, o.local_port)
				 : fclink.openSerial(o.device, o.baud);

	if (!link_opened) {
		fprintf(stderr, "gz-hitl-bridge: error: failed to open FC link\n");
		return 1;
	}

	GzSource gz(o.world, o.model);

	const unsigned servos = o.servos;
	const double servo_max_angle = o.servo_max_angle;

	// One system id for every bridge-originated message (HIL encodes below
	// and FcLink's heartbeat), so the FC and any GCS see a single component.
	const uint8_t sysid = static_cast<uint8_t>(o.sysid);
	fclink.setSysId(sysid);

	auto on_imu = [&fclink, &gz, sysid](uint64_t time_usec) {
		mavlink_hil_sensor_t hil_sensor;
		{
			std::lock_guard<std::mutex> lock(gz.stateMutex());
			hil_sensor = buildHilSensor(time_usec, gz.state());
		}

		mavlink_message_t msg;
		{
			// pack against FcLink's shared tx status: the chan helpers keep a
			// per-translation-unit counter, which would interleave with the
			// heartbeat's and flood the FC's per-component 'lost' stats
			std::lock_guard<std::mutex> lock(fclink.txMutex());
			mavlink_msg_hil_sensor_encode_status(sysid, 200, fclink.txStatus(), &msg, &hil_sensor);
		}
		fclink.sendMessage(msg);

		GpsSample gps;

		if (gz.gpsPop(gps)) {
			mavlink_hil_gps_t hil_gps = buildHilGps(time_usec, gps);
			mavlink_message_t gps_msg;
			{
				std::lock_guard<std::mutex> lock(fclink.txMutex());
				mavlink_msg_hil_gps_encode_status(sysid, 200, fclink.txStatus(), &gps_msg, &hil_gps);
			}
			fclink.sendMessage(gps_msg);
		}
	};

	if (!gz.init(on_imu, servos)) {
		fprintf(stderr, "gz-hitl-bridge: error: failed to init gz source\n");
		return 1;
	}

	const unsigned motors = o.motors;
	const double motor_max_vel = o.motor_max_vel;

	// GcsRelay: bidirectional FC<->QGC relay. A failed open() is non-fatal -- it only logs a
	// warning (from inside GcsRelay::open()) and the bridge keeps running without the relay.
	GcsRelay relay;

	if (o.qgc_relay) {
		relay.open(o.qgc_host, o.qgc_port);
		relay.setToFcHandler([&fclink](const mavlink_message_t &m) {
			fclink.sendMessage(m);
		});

	} else {
		printf("gz-hitl-bridge: QGC relay disabled (no --qgc); QGC must reach the FC directly\n");
	}

	// setMessageHandler() is called before fclink.start() (required by FcLink's thread-safety contract)
	fclink.setMessageHandler([&gz, &relay, motors, motor_max_vel, servos, servo_max_angle,
							 sysid](const mavlink_message_t &m) {
		// The FC's own heartbeat carries its MAV_SYS_ID. Only an autopilot's
		// heartbeat counts: a GCS heartbeat forwarded over this link (when the
		// FC instance has MAV_x_FORWARD on) legitimately carries another id.
		if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			mavlink_heartbeat_t hb;
			mavlink_msg_heartbeat_decode(&m, &hb);

			if (hb.autopilot != MAV_AUTOPILOT_INVALID && m.sysid != sysid) {
				g_fc_sysid_mismatch.store(m.sysid);
			}
		}

		if (m.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
			mavlink_hil_actuator_controls_t hil_actuator_controls;
			mavlink_msg_hil_actuator_controls_decode(&m, &hil_actuator_controls);
			const MotorCommand cmd = decodeActuators(hil_actuator_controls, motors, motor_max_vel);
			gz.publishMotors(cmd);

			if (servos > 0) {
				gz.publishServos(decodeServos(hil_actuator_controls, motors, servos, servo_max_angle));
			}
		}

		// Every message from the FC (including HIL_ACTUATOR_CONTROLS) is relayed to QGC -- the
		// HIL_SENSOR/HIL_GPS messages the bridge sends to the FC never come through this
		// handler path, so no filtering is needed here.
		relay.forwardToGcs(m);
	});

	// relay.start() runs before fclink.start(); shutdown below tears them down in reverse order.
	// Skipped entirely when the relay is disabled (start() would only warn about the unopened fd).
	if (o.qgc_relay) {
		relay.start();
	}

	fclink.start();

	std::signal(SIGINT, handleSigint);

	printf("gz-hitl-bridge: running (model=%s world=%s)\n", o.model.c_str(), o.world.c_str());

	bool link_was_ok = false;   // no FC reception yet at startup (matches linkOk()'s real initial value)
	bool imu_was_alive = false; // no gz IMU callback yet at startup (matches imuAlive()'s real initial value)

	int exit_code = 0;

	while (g_run) {
		const int fc_sysid = g_fc_sysid_mismatch.load();

		if (fc_sysid != 0) {
			fprintf(stderr,
				"gz-hitl-bridge: error: sysid mismatch -- configured --sysid %d but the FC "
				"announces MAV_SYS_ID %d. Set the FC's MAV_SYS_ID to %d (or pass sys_id: %d "
				"in the vehicle's YAML entry) and restart.\n",
				sysid, fc_sysid, sysid, fc_sysid);
			exit_code = 1;
			break;
		}

		const bool link_ok = fclink.linkOk();

		if (!link_ok && link_was_ok) {
			gz.publishMotorsZero(motors);
			fprintf(stderr, "gz-hitl-bridge: warn: FC link lost -> motors zero\n");

		} else if (link_ok && !link_was_ok) {
			printf("gz-hitl-bridge: FC link OK\n");
		}

		link_was_ok = link_ok;

		const bool imu_ok = gz.imuAlive();

		if (!imu_ok && imu_was_alive) {
			fprintf(stderr, "gz-hitl-bridge: warn: gz IMU silent (HIL_SENSOR stalled)\n");

		} else if (imu_ok && !imu_was_alive) {
			printf("gz-hitl-bridge: gz IMU OK\n");
		}

		imu_was_alive = imu_ok;

		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// Fixed shutdown order: 1. stop fclink's handler (no more publishMotors/relay.forwardToGcs
	// calls after this) -> 2. stop relay (joins its QGC RX thread -- after this point,
	// relay.setToFcHandler -> fclink.sendMessage is never called) -> 3. publish zero motor
	// commands (while the publisher is still valid) -> 4. tear down gz subscriptions/publisher
	// (callbacks fully stopped) -> only now is it safe to destroy GzSource's members
	// (mutex/state/callback).
	fclink.stop();
	relay.stop();
	gz.publishMotorsZero(motors);
	gz.stop();

	printf("gz-hitl-bridge: stopped\n");
	return exit_code;
}
