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

#include "can_test_link.hpp"

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <net/if.h>
#include <netpacket/can.h>
#include <nuttx/can.h>

CanTestLink::CanTestLink() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_perf_tx_success = perf_alloc(PC_COUNT, MODULE_NAME": tx_success");
	_perf_tx_fail = perf_alloc(PC_COUNT, MODULE_NAME": tx_fail");
	_perf_rx_success = perf_alloc(PC_COUNT, MODULE_NAME": rx_success");
	_perf_rx_timeout = perf_alloc(PC_COUNT, MODULE_NAME": rx_timeout");
}

CanTestLink::~CanTestLink()
{
	stop();
	close_can_socket();

	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
		_mavlink_log_pub = nullptr;
	}

	perf_free(_perf_tx_success);
	perf_free(_perf_tx_fail);
	perf_free(_perf_rx_success);
	perf_free(_perf_rx_timeout);
}

int CanTestLink::start(const char *mode, const char *device, uint32_t can_id, uint32_t rate_hz)
{
	if (_running) {
		PX4_WARN("already running");
		return -1;
	}

	// Parse mode
	if (strcmp(mode, "tx") == 0) {
		_mode = Mode::TX;
		_tx_can_id = can_id;
		_tx_rate_hz = rate_hz;
		PX4_INFO("Starting in TX mode: device=%s, ID=0x%03lX, rate=%lu Hz", device, (unsigned long)can_id,
			 (unsigned long)rate_hz);

	} else if (strcmp(mode, "rx") == 0) {
		_mode = Mode::RX;
		PX4_INFO("Starting in RX mode: device=%s", device);

	} else {
		PX4_ERR("Invalid mode: %s (must be 'tx' or 'rx')", mode);
		return -1;
	}

	const bool device_changed = (strncmp(_device_name, device, sizeof(_device_name)) != 0);

	if (device_changed && _can_fd >= 0) {
		PX4_INFO("CAN device changed from %s to %s, reopening socket", _device_name, device);
		close_can_socket();
	}

	// Store device name
	strncpy(_device_name, device, sizeof(_device_name) - 1);
	_device_name[sizeof(_device_name) - 1] = '\0';

	// Reset counters for the new session
	_tx_count = 0;
	_rx_count = 0;
	_last_tx_time = 0;
	_last_rx_time = 0;
	_rx_timeout_count = 0;

	_running = true;

	// Schedule work item
	if (_mode == Mode::TX) {
		uint32_t interval_us = 1000000 / _tx_rate_hz;
		ScheduleOnInterval(interval_us);
	} else {
		// RX mode: run at 100Hz to poll for incoming frames
		ScheduleOnInterval(10000); // 10ms = 100Hz
	}

	PX4_INFO("Module started successfully");
	return 0;
}

void CanTestLink::stop()
{
	if (!_running) {
		return;
	}

	_running = false;
	ScheduleClear();
	// Socket is intentionally kept open so that the next start() can reuse
	// it without going through ifdown/ifup which leaves the FDCAN RX path
	// in a broken state on STM32H7.

	PX4_INFO("Module stopped (socket kept open for reuse)");
}

int CanTestLink::init_can_socket()
{
	int fd = -1;
	int ifindex = -1;

	if (init_can_socket_for_device(_device_name, fd, &ifindex) != 0) {
		return -1;
	}

	_can_fd = fd;
	_ifindex = ifindex;
	return 0;
}

int CanTestLink::init_can_socket_for_device(const char *device_name, int &fd, int *ifindex_out)
{
	fd = -1;

	if (ifindex_out != nullptr) {
		*ifindex_out = -1;
	}

	fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

	if (fd < 0) {
		PX4_ERR("socket() failed: %d (%s)", errno, strerror(errno));
		return -1;
	}

	PX4_INFO("Socket created: fd=%d", fd);

	const int ifindex = if_nametoindex(device_name);

	if (ifindex == 0) {
		PX4_ERR("if_nametoindex(%s) failed: %d (%s)", device_name, errno, strerror(errno));
		close(fd);
		fd = -1;
		return -1;
	}

	PX4_INFO("Interface index: %d", ifindex);

	int loopback = 0;
	int ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

	if (ret < 0) {
		PX4_WARN("setsockopt(CAN_RAW_LOOPBACK) failed: %d (%s)", errno, strerror(errno));
	} else {
		PX4_INFO("CAN_RAW_LOOPBACK disabled");
	}

	int recv_own_msgs = 0;
	ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

	if (ret < 0) {
		PX4_WARN("setsockopt(CAN_RAW_RECV_OWN_MSGS) failed: %d (%s)", errno, strerror(errno));
	} else {
		PX4_INFO("CAN_RAW_RECV_OWN_MSGS disabled");
	}

	struct can_filter filter = {};
	filter.can_id = 0;
	filter.can_mask = 0;

	ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

	if (ret < 0) {
		PX4_ERR("setsockopt(CAN_RAW_FILTER) failed: %d (%s)", errno, strerror(errno));
		close(fd);
		fd = -1;
		return -1;
	}

	PX4_INFO("CAN filter set to receive all frames");

	struct sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifindex;

	ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));

	if (ret < 0) {
		PX4_ERR("bind() failed: %d (%s)", errno, strerror(errno));
		close(fd);
		fd = -1;
		return -1;
	}

	PX4_INFO("Socket bound successfully to %s (ifindex=%d)", device_name, ifindex);

	if (ifindex_out != nullptr) {
		*ifindex_out = ifindex;
	}

	return 0;
}

void CanTestLink::close_can_socket()
{
	if (_can_fd >= 0) {
		close(_can_fd);
		_can_fd = -1;
		_ifindex = -1;
		PX4_INFO("Socket closed");
	}
}

void CanTestLink::Run()
{
	if (!_running) {
		return;
	}

	// Open the CAN socket from the work queue context that will use it.
	// On NuttX, opening the fd in the shell task and then using it from a
	// ScheduledWorkItem can lead to EBADF on send()/read().
	if (_can_fd < 0) {
		if (init_can_socket() != 0) {
			PX4_ERR("Failed to initialize CAN socket in Run()");
			return;
		}
	}

	if (_mode == Mode::TX) {
		run_tx_mode();
	} else if (_mode == Mode::RX) {
		run_rx_mode();
	}
}

void CanTestLink::run_tx_mode()
{
	if (_can_fd < 0) {
		PX4_WARN("TX skipped: CAN socket not ready");
		return;
	}

	struct can_frame frame = {};
	frame.can_id = _tx_can_id & CAN_SFF_MASK; // Standard frame
	frame.can_dlc = 8;
	frame.data[0] = 0x11;
	frame.data[1] = 0x22;
	frame.data[2] = 0x33;
	frame.data[3] = 0x44;
	frame.data[4] = 0x55;
	frame.data[5] = 0x66;
	frame.data[6] = 0x77;
	frame.data[7] = 0x88;

	ssize_t nbytes = send(_can_fd, &frame, sizeof(frame), 0);

	if (nbytes == sizeof(frame)) {
		_tx_count++;
		_last_tx_time = hrt_absolute_time();
		perf_count(_perf_tx_success);

		PX4_INFO("TX[%llu]: ID=0x%03lX DLC=%u Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
			 _tx_count,
			 (unsigned long)(frame.can_id),
			 (unsigned int)frame.can_dlc,
			 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
			 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

	} else {
		perf_count(_perf_tx_fail);
		PX4_ERR("TX failed: nbytes=%d, errno=%d (%s)", (int)nbytes, errno, strerror(errno));
	}
}

void CanTestLink::run_rx_mode()
{
	if (_can_fd < 0) {
		PX4_WARN("RX skipped: CAN socket not ready");
		return;
	}

	// Do not rely on poll() notifications here. On this NuttX/FDCAN setup the
	// socket can receive data while poll() never reports readable events.
	bool received_any = false;
	int reads = 0;

	while (reads < 20) {
		struct can_frame frame = {};
		ssize_t nbytes = read(_can_fd, &frame, sizeof(frame));

		if (nbytes < (ssize_t)sizeof(frame)) {
			if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_ERR("read() failed: nbytes=%d, errno=%d (%s)", (int)nbytes, errno, strerror(errno));
			}

			break;
		}

		reads++;
		received_any = true;
		_rx_count++;
		_last_rx_time = hrt_absolute_time();
		_rx_timeout_count = 0;
		perf_count(_perf_rx_success);

		publish_rx_frame_to_gcs(frame);
	}

	if (!received_any) {
		hrt_abstime now = hrt_absolute_time();

		if (_last_rx_time == 0) {
			if (_rx_timeout_count % 100 == 0) {
				PX4_INFO("Poll timeout: no frame received yet (waiting for first frame)");
			}

			_rx_timeout_count++;

		} else {
			hrt_abstime elapsed_us = now - _last_rx_time;

			if (elapsed_us > 1000000) {
				if (_rx_timeout_count % 100 == 0) {
					PX4_WARN("Poll timeout: no frame received for %.2f seconds",
						 (double)elapsed_us / 1000000.0);
				}

				_rx_timeout_count++;
				perf_count(_perf_rx_timeout);
			}
		}
	}
}

int CanTestLink::send_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
	int local_fd = -1;

	if (init_can_socket_for_device(_device_name, local_fd) != 0) {
		PX4_ERR("Failed to initialize temporary CAN socket for send");
		return -1;
	}

	struct can_frame frame = {};
	frame.can_id = can_id & CAN_SFF_MASK;
	frame.can_dlc = (dlc > 8) ? 8 : dlc;
	memcpy(frame.data, data, frame.can_dlc);

	ssize_t nbytes = send(local_fd, &frame, sizeof(frame), 0);
	close(local_fd);

	if (nbytes == sizeof(frame)) {
		_tx_count++;
		_last_tx_time = hrt_absolute_time();
		perf_count(_perf_tx_success);

		PX4_INFO("TX[%llu]: ID=0x%03lX DLC=%u Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
			 _tx_count,
			 (unsigned long)(frame.can_id),
			 (unsigned int)frame.can_dlc,
			 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
			 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
		return 0;

	} else {
		perf_count(_perf_tx_fail);
		PX4_ERR("TX failed: nbytes=%d, errno=%d (%s)", (int)nbytes, errno, strerror(errno));
		return -1;
	}
}

void CanTestLink::publish_rx_frame_to_gcs(const struct can_frame &frame)
{
	mavlink_log_s mavlink_log{};
	mavlink_log.timestamp = hrt_absolute_time();
	mavlink_log.severity = 6; // MAV_SEVERITY_INFO

	snprintf(mavlink_log.text, sizeof(mavlink_log.text),
		 "RX[%llu] %03lX:%02X%02X%02X%02X%02X%02X%02X%02X",
		 _rx_count,
		 (unsigned long)(frame.can_id & CAN_SFF_MASK),
		 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
		 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
	mavlink_log.text[sizeof(mavlink_log.text) - 1] = '\0';

	if (_mavlink_log_pub == nullptr) {
		// This PX4 branch does not expose orb_advertise_queue() through the
		// common uORB API used by modules. A normal advertise still uses the
		// topic metadata, and mavlink_log declares ORB_QUEUE_LENGTH = 8.
		_mavlink_log_pub = orb_advertise(ORB_ID(mavlink_log), &mavlink_log);

		if (_mavlink_log_pub == nullptr) {
			PX4_ERR("orb_advertise(mavlink_log) failed");
		}

		return;
	}

	if (orb_publish(ORB_ID(mavlink_log), _mavlink_log_pub, &mavlink_log) != PX4_OK) {
		PX4_WARN("orb_publish(mavlink_log) failed");
	}
}

void CanTestLink::print_status()
{
	PX4_INFO("=== CAN Test Link Status ===");
	PX4_INFO("Running: %s", _running ? "YES" : "NO");

	if (_mode == Mode::TX) {
		PX4_INFO("Mode: TX");
	} else if (_mode == Mode::RX) {
		PX4_INFO("Mode: RX");
	} else {
		PX4_INFO("Mode: NONE");
	}

	PX4_INFO("Device: %s", _device_name);
	PX4_INFO("Socket FD: %d", _can_fd);
	PX4_INFO("Interface Index: %d", _ifindex);

	if (_mode == Mode::TX) {
		PX4_INFO("TX CAN ID: 0x%03lX", (unsigned long)_tx_can_id);
		PX4_INFO("TX Rate: %lu Hz", (unsigned long)_tx_rate_hz);
		PX4_INFO("TX Count: %llu", _tx_count);

		if (_last_tx_time > 0) {
			hrt_abstime now = hrt_absolute_time();
			hrt_abstime elapsed_us = now - _last_tx_time;
			PX4_INFO("Last TX: %.3f seconds ago", (double)elapsed_us / 1000000.0);
		} else {
			PX4_INFO("Last TX: never");
		}

		perf_print_counter(_perf_tx_success);
		perf_print_counter(_perf_tx_fail);

	} else if (_mode == Mode::RX) {
		PX4_INFO("RX Count: %llu", _rx_count);

		if (_last_rx_time > 0) {
			hrt_abstime now = hrt_absolute_time();
			hrt_abstime elapsed_us = now - _last_rx_time;
			PX4_INFO("Last RX: %.3f seconds ago", (double)elapsed_us / 1000000.0);
		} else {
			PX4_INFO("Last RX: never (waiting for first frame)");
		}

		PX4_INFO("RX Timeout Count: %lu", (unsigned long)_rx_timeout_count);

		perf_print_counter(_perf_rx_success);
		perf_print_counter(_perf_rx_timeout);
	}

	PX4_INFO("============================");
}
