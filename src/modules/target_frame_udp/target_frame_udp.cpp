#include "target_frame_udp.hpp"

#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <inttypes.h>
#include <initializer_list>
#include <limits>
#include <unistd.h>

using target_frame_udp::CrcMode;

TargetFrameUdp::TargetFrameUdp(const Options &options) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_options(options)
{
	_tx_interval = static_cast<hrt_abstime>(1_s / _options.rate_hz);
}

TargetFrameUdp::~TargetFrameUdp()
{
	close_socket();
}

bool TargetFrameUdp::init()
{
	int32_t system_id = 0;
	param_t id_handle = param_find("MAV_SYS_ID");

	if (id_handle == PARAM_INVALID || param_get(id_handle, &system_id) != PX4_OK || system_id <= 0 || system_id > UINT16_MAX) {
		PX4_ERR("invalid MAV_SYS_ID");
		return false;
	}

	_vehicle_id = static_cast<uint32_t>(system_id);

	if (!open_socket()) {
		return false;
	}

	ScheduleOnInterval(20_ms);
	return true;
}

bool TargetFrameUdp::open_socket()
{
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		PX4_ERR("socket: %s", strerror(errno));
		return false;
	}

	int reuse = 1;
	setsockopt(_socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	const int flags = fcntl(_socket_fd, F_GETFL, 0);

	if (flags < 0 || fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		PX4_ERR("nonblocking socket: %s", strerror(errno));
		close_socket();
		return false;
	}

	struct sockaddr_in local_address {};
	local_address.sin_family = AF_INET;
	local_address.sin_addr.s_addr = htonl(INADDR_ANY);
	local_address.sin_port = htons(_options.local_port);

	if (bind(_socket_fd, reinterpret_cast<struct sockaddr *>(&local_address), sizeof(local_address)) < 0) {
		PX4_ERR("bind UDP %u: %s", static_cast<unsigned>(_options.local_port), strerror(errno));
		close_socket();
		return false;
	}

	_peer_address.sin_family = AF_INET;
	_peer_address.sin_port = htons(_options.remote_port);

	if (inet_pton(AF_INET, _options.peer_ip, &_peer_address.sin_addr) != 1) {
		PX4_ERR("invalid peer IPv4: %s", _options.peer_ip);
		close_socket();
		return false;
	}

	return true;
}

void TargetFrameUdp::close_socket()
{
	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}
}

void TargetFrameUdp::update_ownship()
{
	_global_position_sub.update(&_global_position);
	_local_position_sub.update(&_local_position);
}

bool TargetFrameUdp::ownship_valid(hrt_abstime now) const
{
	return _global_position.timestamp != 0 && now - _global_position.timestamp < 1_s &&
	       _global_position.lat_lon_valid && _global_position.alt_valid &&
	       PX4_ISFINITE(_global_position.lat) && PX4_ISFINITE(_global_position.lon) && PX4_ISFINITE(_global_position.alt) &&
	       _global_position.lat >= -90.0 && _global_position.lat <= 90.0 &&
	       _global_position.lon >= -180.0 && _global_position.lon <= 180.0 &&
	       _local_position.timestamp != 0 && now - _local_position.timestamp < 1_s;
}

void TargetFrameUdp::send_ownship(hrt_abstime now)
{
	if (_socket_fd < 0 || (now - _last_tx_time) < _tx_interval || !ownship_valid(now)) {
		return;
	}

	_last_tx_time = now;
	target_frame_udp::Target target{};
	target.longitude_e7 = static_cast<int32_t>(llround(_global_position.lon * 1e7));
	target.latitude_e7 = static_cast<int32_t>(llround(_global_position.lat * 1e7));

	const double altitude_mm = static_cast<double>(_global_position.alt) * 1000.0;
	target.altitude_mm = static_cast<int32_t>(llround(fmax(static_cast<double>(INT32_MIN),
			     fmin(static_cast<double>(INT32_MAX), altitude_mm))));
	target.target_id = static_cast<uint16_t>(_vehicle_id);
	target.altitude_m = _global_position.alt;
	target.track_type = _options.track_type;
	target.target_attribute = _options.target_attribute;

	if (_local_position.v_xy_valid && _local_position.v_z_valid &&
	    PX4_ISFINITE(_local_position.vx) && PX4_ISFINITE(_local_position.vy) && PX4_ISFINITE(_local_position.vz)) {
		target.vx_m_s = _local_position.vx;
		target.vy_m_s = _local_position.vy;
		target.vz_m_s = _local_position.vz;
		target.speed_m_s = sqrtf(target.vx_m_s * target.vx_m_s + target.vy_m_s * target.vy_m_s +
					target.vz_m_s * target.vz_m_s);
	}

	uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};
	size_t encoded_length = 0;

	if (!target_frame_udp::encode(&target, 1, _tx_sequence++, static_cast<uint32_t>(now / 1000),
				      _options.crc_mode, buffer, sizeof(buffer), encoded_length)) {
		++_socket_errors;
		return;
	}

	const ssize_t sent = sendto(_socket_fd, buffer, encoded_length, 0,
				    reinterpret_cast<const struct sockaddr *>(&_peer_address), sizeof(_peer_address));

	if (sent == static_cast<ssize_t>(encoded_length)) {
		++_tx_packets;

	} else {
		++_socket_errors;
		PX4_WARN("UDP send failed: %s", sent < 0 ? strerror(errno) : "short write");
	}
}

void TargetFrameUdp::publish_target(const target_frame_udp::Target &target)
{
	if (target.target_id == 0 || target.target_id == _vehicle_id || target.delete_flag != 0 ||
	    target.latitude_e7 < -900000000 || target.latitude_e7 > 900000000 ||
	    target.longitude_e7 < -1800000000 || target.longitude_e7 > 1800000000 ||
	    !PX4_ISFINITE(target.altitude_m) || !PX4_ISFINITE(target.vx_m_s) ||
	    !PX4_ISFINITE(target.vy_m_s) || !PX4_ISFINITE(target.vz_m_s)) {
		return;
	}

	follower_info_s info{};
	info.timestamp = hrt_absolute_time();
	info.mavid = target.target_id;
	info.lat = static_cast<double>(target.latitude_e7) * 1e-7;
	info.lon = static_cast<double>(target.longitude_e7) * 1e-7;
	info.alt = target.altitude_m;
	info.vx = target.vx_m_s;
	info.vy = target.vy_m_s;
	info.vz = target.vz_m_s;
	info.yaw = NAN;
	info.yawspeed = NAN;
	info.land = 0;
	info.at_target = 0;
	_follower_info_pub.publish(info);
	++_rx_targets;
}

void TargetFrameUdp::receive_frames()
{
	uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};

	while (_socket_fd >= 0) {
		struct sockaddr_in source_address {};
		socklen_t source_length = sizeof(source_address);
		const ssize_t received = recvfrom(_socket_fd, buffer, sizeof(buffer), 0,
					  reinterpret_cast<struct sockaddr *>(&source_address), &source_length);

		if (received < 0) {
			if (errno != EAGAIN
#if EWOULDBLOCK != EAGAIN
			    && errno != EWOULDBLOCK
#endif
			   ) {
				++_socket_errors;
			}

			break;
		}

		if (received == 0) {
			break;
		}

		if (source_address.sin_addr.s_addr != _peer_address.sin_addr.s_addr ||
		    source_address.sin_port != _peer_address.sin_port) {
			++_rx_dropped;
			continue;
		}

		target_frame_udp::Target targets[target_frame_udp::MAX_TARGETS] {};
		target_frame_udp::FrameInfo frame_info{};
		const auto result = target_frame_udp::decode(buffer, static_cast<size_t>(received), _options.crc_mode,
				targets, target_frame_udp::MAX_TARGETS, frame_info);

		if (result != target_frame_udp::DecodeResult::Ok) {
			++_rx_dropped;

			if ((_rx_dropped % 100) == 1) {
				PX4_WARN("drop target frame: %s", target_frame_udp::decode_result_string(result));
			}

			continue;
		}

		++_rx_packets;

		if (_rx_sequence_initialized && frame_info.sequence != static_cast<uint8_t>(_last_rx_sequence + 1)) {
			++_rx_sequence_errors;
		}

		_last_rx_sequence = frame_info.sequence;
		_rx_sequence_initialized = true;

		for (size_t i = 0; i < frame_info.target_count; ++i) {
			publish_target(targets[i]);
		}
	}
}

void TargetFrameUdp::Run()
{
	if (should_exit()) {
		ScheduleClear();
		close_socket();
		exit_and_cleanup();
		return;
	}
	update_ownship();
	receive_frames();
	send_ownship(hrt_absolute_time());
}

bool TargetFrameUdp::parse_crc_mode(const char *name, CrcMode &mode)
{
	if (strcmp(name, "none") == 0) { mode = CrcMode::None; }
	else if (strcmp(name, "modbus") == 0) { mode = CrcMode::Modbus; }
	else if (strcmp(name, "ccitt") == 0 || strcmp(name, "ccitt-false") == 0) { mode = CrcMode::CcittFalse; }
	else if (strcmp(name, "x25") == 0) { mode = CrcMode::X25; }
	else { return false; }

	return true;
}

const char *TargetFrameUdp::crc_mode_name(CrcMode mode)
{
	switch (mode) {
	case CrcMode::None: return "none";
	case CrcMode::Modbus: return "modbus";
	case CrcMode::CcittFalse: return "ccitt-false";
	case CrcMode::X25: return "x25";
	}

	return "unknown";
}

bool TargetFrameUdp::codec_self_test()
{
	static constexpr uint8_t crc_check[] {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

	if (target_frame_udp::crc16(CrcMode::Modbus, crc_check, sizeof(crc_check)) != 0x4B37 ||
	    target_frame_udp::crc16(CrcMode::CcittFalse, crc_check, sizeof(crc_check)) != 0x29B1 ||
	    target_frame_udp::crc16(CrcMode::X25, crc_check, sizeof(crc_check)) != 0x906E) {
		return false;
	}

	target_frame_udp::Target input{};
	input.longitude_e7 = 1163912345;
	input.latitude_e7 = 399071234;
	input.altitude_mm = 123450;
	input.target_id = 2;
	input.speed_m_s = 12.5f;
	input.altitude_m = 123.45f;
	input.vx_m_s = 1.25f;
	input.vy_m_s = -2.5f;
	input.vz_m_s = 0.75f;

	uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};
	size_t encoded_length = 0;

	for (CrcMode mode : {CrcMode::None, CrcMode::Modbus, CrcMode::CcittFalse, CrcMode::X25}) {
		if (!target_frame_udp::encode(&input, 1, 254, 123456, mode, buffer, sizeof(buffer), encoded_length) ||
		    encoded_length != 67) {
			return false;
		}

		target_frame_udp::Target output{};
		target_frame_udp::FrameInfo info{};

		if (target_frame_udp::decode(buffer, encoded_length, mode, &output, 1, info) !=
		    target_frame_udp::DecodeResult::Ok || info.sequence != 254 || info.time_ms != 123456 ||
		    output.longitude_e7 != input.longitude_e7 || output.latitude_e7 != input.latitude_e7 ||
		    output.altitude_mm != input.altitude_mm || output.target_id != input.target_id ||
		    memcmp(&output.vy_m_s, &input.vy_m_s, sizeof(float)) != 0) {
			return false;
		}

		if (mode != CrcMode::None) {
			buffer[20] ^= 1;

			if (target_frame_udp::decode(buffer, encoded_length, mode, &output, 1, info) !=
			    target_frame_udp::DecodeResult::BadCrc) {
				return false;
			}
		}
	}

	target_frame_udp::Target multi_input[2] {input, input};
	multi_input[1].target_id = 3;
	multi_input[1].longitude_e7 = -1220840575;
	multi_input[1].latitude_e7 = 374219999;

	if (!target_frame_udp::encode(multi_input, 2, 7, 42, CrcMode::Modbus, buffer, sizeof(buffer), encoded_length) ||
	    encoded_length != 117) {
		return false;
	}

	target_frame_udp::Target multi_output[2] {};
	target_frame_udp::FrameInfo multi_info{};

	if (target_frame_udp::decode(buffer, encoded_length, CrcMode::Modbus, multi_output, 2, multi_info) !=
	    target_frame_udp::DecodeResult::Ok || multi_info.target_count != 2 ||
	    multi_output[1].target_id != multi_input[1].target_id ||
	    multi_output[1].longitude_e7 != multi_input[1].longitude_e7) {
		return false;
	}

	if (target_frame_udp::decode(buffer, encoded_length - 1, CrcMode::Modbus, multi_output, 2, multi_info) !=
	    target_frame_udp::DecodeResult::BadLength) {
		return false;
	}

	buffer[0] = 0;

	if (target_frame_udp::decode(buffer, encoded_length, CrcMode::Modbus, multi_output, 2, multi_info) !=
	    target_frame_udp::DecodeResult::BadHeader) {
		return false;
	}

	return true;
}

int TargetFrameUdp::task_spawn(int argc, char *argv[])
{
	Options options{};
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	while ((ch = px4_getopt(argc, argv, "t:l:o:r:c:y:a:h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			strncpy(options.peer_ip, myoptarg, sizeof(options.peer_ip) - 1);
			options.peer_ip[sizeof(options.peer_ip) - 1] = '\0';
			break;

		case 'l': options.local_port = static_cast<uint16_t>(strtoul(myoptarg, nullptr, 10)); break;
		case 'o': options.remote_port = static_cast<uint16_t>(strtoul(myoptarg, nullptr, 10)); break;
		case 'r': options.rate_hz = math::constrain(strtof(myoptarg, nullptr), 1.f, 50.f); break;

		case 'c':
			if (!parse_crc_mode(myoptarg, options.crc_mode)) { return print_usage("invalid CRC mode"); }
			break;

		case 'y': options.track_type = static_cast<uint8_t>(strtoul(myoptarg, nullptr, 0)); break;
		case 'a': options.target_attribute = static_cast<uint8_t>(strtoul(myoptarg, nullptr, 0)); break;
		case 'h':
		default: return print_usage();
		}
	}

	if (options.local_port == 0 || options.remote_port == 0) {
		return print_usage("UDP ports must be non-zero");
	}

	auto *instance = new TargetFrameUdp(options);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int TargetFrameUdp::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp(argv[0], "test") == 0) {
		const bool passed = codec_self_test();
		PX4_INFO("target frame codec self-test: %s", passed ? "PASS" : "FAIL");
		return passed ? PX4_OK : PX4_ERROR;
	}

	return print_usage("unknown command");
}

int TargetFrameUdp::print_status()
{
	PX4_INFO("peer=%s:%u local=%u rate=%.1fHz crc=%s sysid=%" PRIu32,
		 _options.peer_ip, static_cast<unsigned>(_options.remote_port), static_cast<unsigned>(_options.local_port),
		 static_cast<double>(_options.rate_hz), crc_mode_name(_options.crc_mode), _vehicle_id);
	PX4_INFO("tx=%" PRIu64 " rx=%" PRIu64 " targets=%" PRIu64 " dropped=%" PRIu64
		 " sequence_errors=%" PRIu64 " socket_errors=%" PRIu64,
		 _tx_packets, _rx_packets, _rx_targets, _rx_dropped, _rx_sequence_errors, _socket_errors);
	return PX4_OK;
}

int TargetFrameUdp::print_usage(const char *reason)
{
	if (reason != nullptr) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Bidirectional UDP transport for the radar/display target-frame protocol.

The module sends this vehicle as one target and publishes valid received targets
to follower_info. CRC is configurable because the source protocol does not name
a CRC-16 profile; MODBUS is only the temporary default.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("target_frame_udp", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", "<IPv4>", "Peer IPv4 address", true);
	PRINT_MODULE_USAGE_PARAM_INT('l', 50000, 1, 65535, "Local UDP port", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 50000, 1, 65535, "Remote UDP port", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 10.f, 1.f, 50.f, "Transmit rate", true);
	PRINT_MODULE_USAGE_PARAM_STRING('c', "modbus", "none|modbus|ccitt|x25", "CRC-16 profile", true);
	PRINT_MODULE_USAGE_PARAM_INT('y', 0x20, 0, 255, "Track type", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', 0x22, 0, 255, "Target attribute", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Run protocol codec self-test");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int target_frame_udp_main(int argc, char *argv[])
{
	return TargetFrameUdp::main(argc, argv);
}
