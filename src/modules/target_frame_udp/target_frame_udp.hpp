#pragma once

#include "target_frame_codec.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/follower_info.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <netinet/in.h>

using namespace time_literals;

class TargetFrameUdp : public ModuleBase<TargetFrameUdp>, public px4::ScheduledWorkItem
{
public:
	struct Options {
		char peer_ip[INET_ADDRSTRLEN]{"127.0.0.1"};
		uint16_t local_port{50000};
		uint16_t remote_port{50000};
		float rate_hz{10.f};
		target_frame_udp::CrcMode crc_mode{target_frame_udp::CrcMode::Modbus};
		uint8_t track_type{0x20};
		uint8_t target_attribute{0x22};
	};

	explicit TargetFrameUdp(const Options &options);
	~TargetFrameUdp() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	bool open_socket();
	void close_socket();
	void update_ownship();
	void send_ownship(hrt_abstime now);
	bool send_targets(const target_frame_udp::Target *targets, size_t target_count, bool force_dump);
	bool send_raw_frame(const uint8_t *buffer, size_t length, bool force_dump);
	void receive_frames();
	void publish_target(const target_frame_udp::Target &target);
	void dump_frame(const char *direction, const uint8_t *buffer, size_t length) const;
	void dump_target(const char *direction, const target_frame_udp::Target &target) const;
	bool command_send_decimal(int argc, char *argv[]);
	bool command_send_hex(int argc, char *argv[]);
	bool ownship_valid(hrt_abstime now) const;
	static bool parse_crc_mode(const char *name, target_frame_udp::CrcMode &mode);
	static const char *crc_mode_name(target_frame_udp::CrcMode mode);
	static bool codec_self_test();

	Options _options{};
	int _socket_fd{-1};
	struct sockaddr_in _peer_address {};
	uint32_t _vehicle_id{0};
	px4::atomic<uint8_t> _tx_sequence{0};
	px4::atomic<bool> _dump_enabled{false};
	px4::atomic<bool> _auto_send_enabled{true};
	uint8_t _last_rx_sequence{0};
	bool _rx_sequence_initialized{false};
	hrt_abstime _last_tx_time{0};
	hrt_abstime _tx_interval{100_ms};

	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};

	uint64_t _tx_packets{0};
	uint64_t _rx_packets{0};
	uint64_t _rx_targets{0};
	uint64_t _rx_dropped{0};
	uint64_t _rx_sequence_errors{0};
	uint64_t _socket_errors{0};

	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Publication<follower_info_s> _follower_info_pub{ORB_ID(follower_info)};
};
