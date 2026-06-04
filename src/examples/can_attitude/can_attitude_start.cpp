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
#include <px4_platform_common/tasks.h>
#include <string.h>

static int daemon_task;

extern "C" __EXPORT int can_attitude_main(int argc, char *argv[]);

int can_attitude_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: can_attitude {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (CanAttitude::appState.isRunning()) {
			PX4_INFO("can_attitude is already running");
			return 0;
		}

		daemon_task = px4_task_spawn_cmd("can_attitude",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2500,
						 PX4_MAIN,
						 (char *const *)nullptr);

		if (daemon_task < 0) {
			PX4_ERR("Failed to spawn can_attitude task");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (!CanAttitude::appState.isRunning()) {
			PX4_INFO("can_attitude is not running");
			return 0;
		}

		CanAttitude::appState.requestExit();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (CanAttitude::appState.isRunning()) {
			PX4_INFO("can_attitude is running");
		} else {
			PX4_INFO("can_attitude is not running");
		}
		return 0;
	}

	PX4_WARN("usage: can_attitude {start|stop|status}");
	return 1;
}
