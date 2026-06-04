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

#include "can_attitude.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
#include <drivers/uavcan/uavcan_main.hpp>
#include <uavcan/equipment/ahrs/CustomAttitude.hpp>

CanAttitudeAppState CanAttitude::appState;

CanAttitude::CanAttitude()
{
}

CanAttitude::~CanAttitude()
{
}

int CanAttitude::main()
{
	appState.setRunning(true);

	PX4_INFO("Initializing DroneCAN custom attitude transmitter...");

	UavcanNode *uavcan_node = UavcanNode::instance();
	if (uavcan_node == nullptr) {
		PX4_ERR("UavcanNode is not running. Please start DroneCAN ('uavcan start') first.");
		appState.setRunning(false);
		return -1;
	}

	// We must lock the node mutex when interacting with uavcan node (e.g. creating a publisher)
	uavcan_node->lock();
	uavcan::Publisher<uavcan::equipment::ahrs::CustomAttitude> ahrs_pub(uavcan_node->get_node());
	ahrs_pub.setPriority(uavcan::TransferPriority::Default);
	uavcan_node->unlock();

	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	if (attitude_sub < 0) {
		PX4_ERR("Failed to subscribe to vehicle_attitude");
		appState.setRunning(false);
		return -1;
	}

	PX4_INFO("DroneCAN custom attitude transmitter successfully started");

	while (!appState.exitRequested()) {
		// Run loop at 200 Hz
		px4_usleep(5000);

		bool updated = false;
		orb_check(attitude_sub, &updated);

		if (updated) {
			vehicle_attitude_s att;
			if (orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &att) == PX4_OK) {
				matrix::Quatf q(att.q);
				matrix::Eulerf euler(q);

				uavcan::equipment::ahrs::CustomAttitude msg;

				msg.roll_pitch_yaw[0] = euler.phi();   // Roll (rad)
				msg.roll_pitch_yaw[1] = euler.theta(); // Pitch (rad)
				msg.roll_pitch_yaw[2] = euler.psi();   // Yaw (rad)

				// Lock the node mutex during broadcast to ensure thread safety
				uavcan_node->lock();
				int res = ahrs_pub.broadcast(msg);
				uavcan_node->unlock();

				if (res < 0) {
					static uint64_t last_err_time = 0;
					uint64_t now = hrt_absolute_time();
					if (now - last_err_time >= 1000000ULL) {
						PX4_ERR("DroneCAN broadcast failed: %d", res);
						last_err_time = now;
					}
				}

				static uint64_t last_log_time = 0;
				static uint32_t sent_count = 0;
				sent_count++;

				uint64_t now = hrt_absolute_time();
				if (now - last_log_time >= 1000000ULL) { // 1 second
					last_log_time = now;
					PX4_INFO("Sent %u attitude frames. Last broadcast res: %d. RPY: [%.3f, %.3f, %.3f]",
						(unsigned)sent_count, res,
						(double)msg.roll_pitch_yaw[0],
						(double)msg.roll_pitch_yaw[1],
						(double)msg.roll_pitch_yaw[2]
					);
				}
			}
		}
	}

	PX4_INFO("Stopping DroneCAN custom attitude transmitter...");
	orb_unsubscribe(attitude_sub);

	appState.setRunning(false);
	return 0;
}
