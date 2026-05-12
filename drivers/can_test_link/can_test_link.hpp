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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/mavlink_log.h>

#include <nuttx/can.h>
#include <netpacket/can.h>
#include <net/if.h>

class CanTestLink : public px4::ScheduledWorkItem
{
public:
	CanTestLink();
	~CanTestLink() override;

	/**
	 * @brief Start the module
	 * @param mode "tx" or "rx"
	 * @param device CAN device name (e.g., "can0")
	 * @param can_id CAN ID for TX mode (default 0x123)
	 * @param rate_hz TX rate in Hz (default 10)
	 * @return 0 on success, -1 on failure
	 */
	int start(const char *mode, const char *device, uint32_t can_id, uint32_t rate_hz);

	/**
	 * @brief Stop the module
	 */
	void stop();

	/**
	 * @brief Print status information
	 */
	void print_status();

	/**
	 * @brief Check if module is running
	 */
	bool is_running() const { return _running; }

private:
	void Run() override;

	/**
	 * @brief Initialize CAN socket
	 * @return 0 on success, -1 on failure
	 */
	int init_can_socket();

	/**
	 * @brief Initialize a CAN socket bound to the given device
	 * @return 0 on success, -1 on failure
	 */
	int init_can_socket_for_device(const char *device_name, int &fd, int *ifindex_out = nullptr);

	/**
	 * @brief Close CAN socket
	 */
	void close_can_socket();

	/**
	 * @brief RX mode: receive and print CAN frames
	 */
	void run_rx_mode();

	/**
	 * @brief TX mode: send periodic CAN frames
	 */
	void run_tx_mode();

	/**
	 * @brief Publish a compact RX line to mavlink_log using a queued topic
	 */
	void publish_rx_frame_to_gcs(const struct can_frame &frame);

public:
	/**
	 * @brief Send a single CAN frame with specified ID and data
	 * @return 0 on success, -1 on failure
	 */
	int send_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

private:
	enum class Mode {
		NONE,
		TX,
		RX
	};

	Mode _mode{Mode::NONE};
	char _device_name[16]{"can0"};
	int _can_fd{-1};
	int _ifindex{-1};
	bool _running{false};

	// TX mode parameters
	uint32_t _tx_can_id{0x123};
	uint32_t _tx_rate_hz{10};
	uint64_t _tx_count{0};
	hrt_abstime _last_tx_time{0};

	// RX mode parameters
	uint64_t _rx_count{0};
	hrt_abstime _last_rx_time{0};
	uint32_t _rx_timeout_count{0};

	// Performance counters
	perf_counter_t _perf_tx_success{nullptr};
	perf_counter_t _perf_tx_fail{nullptr};
	perf_counter_t _perf_rx_success{nullptr};
	perf_counter_t _perf_rx_timeout{nullptr};

	// MAVLink log publisher handle. The mavlink_log topic itself declares a
	// queue, so a normal advertise keeps the topic's buffered behavior.
	orb_advert_t _mavlink_log_pub{nullptr};
};
