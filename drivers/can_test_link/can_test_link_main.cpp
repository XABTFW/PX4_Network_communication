/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file can_test_link_main.cpp
 * Minimal CAN test module for basic CAN link verification between two flight controllers.
 *
 * @author PX4 Development Team
 */

#include "can_test_link.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

static CanTestLink *g_can_test_link = nullptr;

extern "C" __EXPORT int can_test_link_main(int argc, char *argv[]);

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal CAN test module for basic CAN link verification between two flight controllers.

This module supports two modes:
- TX mode: Periodically sends fixed CAN frames
- RX mode: Receives and prints all incoming CAN frames

### Examples
Start in RX mode on can0:
$ can_test_link start -m rx -d can0

Start in TX mode on can0, sending ID 0x123 at 20Hz:
$ can_test_link start -m tx -d can0 -i 0x123 -r 20

Check status:
$ can_test_link status

Stop the module:
$ can_test_link stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("can_test_link", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
	PRINT_MODULE_USAGE_PARAM_STRING('m', "rx", "tx|rx", "Mode: tx or rx", false);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "can0", nullptr, "CAN device name", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0x123, 0, 0x7FF, "CAN ID (TX mode only)", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 10, 1, 1000, "TX rate in Hz (TX mode only)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the module");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Send a single CAN frame: send <id> <hex bytes...>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
}

int can_test_link_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	const char *verb = argv[1];

	if (strcmp(verb, "start") == 0) {
		if (g_can_test_link != nullptr && g_can_test_link->is_running()) {
			PX4_WARN("already running, stop first");
			return 1;
		}

		// Parse arguments
		const char *mode = "rx";
		const char *device = "can0";
		uint32_t can_id = 0x123;
		uint32_t rate_hz = 10;

		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "m:d:i:r:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'm':
				mode = myoptarg;
				break;

			case 'd':
				device = myoptarg;
				break;

			case 'i':
				can_id = strtoul(myoptarg, nullptr, 0);
				break;

			case 'r':
				rate_hz = strtoul(myoptarg, nullptr, 10);
				break;

			default:
				usage();
				return 1;
			}
		}

		// Validate parameters
		if (strcmp(mode, "tx") != 0 && strcmp(mode, "rx") != 0) {
			PX4_ERR("Invalid mode: %s (must be 'tx' or 'rx')", mode);
			usage();
			return 1;
		}

		if (can_id > 0x7FF) {
			PX4_ERR("Invalid CAN ID: 0x%lX (must be <= 0x7FF for standard frame)", (unsigned long)can_id);
			return 1;
		}

		if (rate_hz < 1 || rate_hz > 1000) {
			PX4_ERR("Invalid rate: %lu Hz (must be 1-1000)", (unsigned long)rate_hz);
			return 1;
		}

		// Reuse existing object (keeps socket open) or create new one
		if (g_can_test_link == nullptr) {
			g_can_test_link = new CanTestLink();

			if (g_can_test_link == nullptr) {
				PX4_ERR("Failed to allocate CanTestLink");
				return 1;
			}
		}

		if (g_can_test_link->start(mode, device, can_id, rate_hz) != 0) {
			delete g_can_test_link;
			g_can_test_link = nullptr;
			PX4_ERR("Failed to start");
			return 1;
		}

		return 0;

	} else if (strcmp(verb, "stop") == 0) {
		if (g_can_test_link == nullptr || !g_can_test_link->is_running()) {
			PX4_WARN("not running");
			return 1;
		}

		g_can_test_link->stop();
		// Object is kept alive so the socket can be reused on next start

		return 0;

	} else if (strcmp(verb, "destroy") == 0) {
		if (g_can_test_link == nullptr) {
			PX4_WARN("not created");
			return 1;
		}

		delete g_can_test_link;
		g_can_test_link = nullptr;
		PX4_INFO("Module destroyed");

		return 0;

	} else if (strcmp(verb, "send") == 0) {
		// Usage: can_test_link send <id> <hex_byte1> <hex_byte2> ...
		if (argc < 4) {
			PX4_ERR("Usage: can_test_link send <id> <hex bytes...>");
			return 1;
		}

		if (g_can_test_link == nullptr) {
			g_can_test_link = new CanTestLink();

			if (g_can_test_link == nullptr) {
				PX4_ERR("Failed to allocate CanTestLink");
				return 1;
			}
		}

		uint32_t can_id = strtoul(argv[2], nullptr, 0);

		if (can_id > 0x7FF) {
			PX4_ERR("Invalid CAN ID: 0x%lX (must be <= 0x7FF)", (unsigned long)can_id);
			return 1;
		}

		uint8_t data[8] = {};
		int dlc = argc - 3;

		if (dlc > 8) { dlc = 8; }

		for (int i = 0; i < dlc; i++) {
			data[i] = (uint8_t)strtoul(argv[3 + i], nullptr, 16);
		}

		return g_can_test_link->send_single_frame(can_id, data, dlc);

	} else if (strcmp(verb, "status") == 0) {
		if (g_can_test_link == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		g_can_test_link->print_status();

		return 0;

	} else {
		usage();
		return 1;
	}
}
