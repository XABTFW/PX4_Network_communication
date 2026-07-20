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
#include <stdio.h>
#include <unistd.h>

using target_frame_udp::CrcMode;

namespace
{

int hex_nibble(char value)
{
	if (value >= '0' && value <= '9') { return value - '0'; }
	if (value >= 'a' && value <= 'f') { return value - 'a' + 10; }
	if (value >= 'A' && value <= 'F') { return value - 'A' + 10; }
	return -1;
}

bool parse_double_value(const char *text, double &value)
{
	char *end = nullptr;
	errno = 0;
	value = strtod(text, &end);
	return errno == 0 && end != text && *end == '\0' && std::isfinite(value);
}

bool parse_float_value(const char *text, float &value)
{
	char *end = nullptr;
	errno = 0;
	value = strtof(text, &end);
	return errno == 0 && end != text && *end == '\0' && PX4_ISFINITE(value);
}

bool parse_target_id(const char *text, uint16_t &value)
{
	char *end = nullptr;
	errno = 0;
	const unsigned long parsed = strtoul(text, &end, 0);

	if (errno != 0 || end == text || *end != '\0' || parsed == 0 || parsed > UINT16_MAX) {
		return false;
	}

	value = static_cast<uint16_t>(parsed);
	return true;
}

} // namespace

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
	if (!_auto_send_enabled.load() || _socket_fd < 0 || (now - _last_tx_time) < _tx_interval || !ownship_valid(now)) {
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

	send_targets(&target, 1, false);
}

bool TargetFrameUdp::send_targets(const target_frame_udp::Target *targets, size_t target_count, bool force_dump)
{
	uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};
	size_t encoded_length = 0;
	const uint8_t sequence = _tx_sequence.fetch_add(1);

	if (!target_frame_udp::encode(targets, target_count, sequence, static_cast<uint32_t>(hrt_absolute_time() / 1000),
				      _options.crc_mode, buffer, sizeof(buffer), encoded_length)) {
		++_socket_errors;
		return false;
	}

	if (force_dump || _dump_enabled.load()) {
		dump_frame("TX", buffer, encoded_length);

		for (size_t i = 0; i < target_count; ++i) {
			dump_target("TX", targets[i]);
		}
	}

	return send_raw_frame(buffer, encoded_length, false);
}

bool TargetFrameUdp::send_raw_frame(const uint8_t *buffer, size_t length, bool force_dump)
{
	if (_socket_fd < 0 || buffer == nullptr || length == 0) {
		return false;
	}

	if (force_dump) {
		dump_frame("TX", buffer, length);
	}

	const ssize_t sent = sendto(_socket_fd, buffer, length, 0,
				    reinterpret_cast<const struct sockaddr *>(&_peer_address), sizeof(_peer_address));

	if (sent == static_cast<ssize_t>(length)) {
		++_tx_packets;
		return true;

	} else {
		++_socket_errors;
		PX4_WARN("UDP send failed: %s", sent < 0 ? strerror(errno) : "short write");
		return false;
	}
}

void TargetFrameUdp::dump_frame(const char *direction, const uint8_t *buffer, size_t length) const
{
	static constexpr size_t bytes_per_line = 16;
	PX4_INFO("%s HEX len=%u", direction, static_cast<unsigned>(length));

	for (size_t offset = 0; offset < length; offset += bytes_per_line) {
		char text[bytes_per_line * 3 + 1] {};
		size_t text_index = 0;
		const size_t line_end = math::min(offset + bytes_per_line, length);

		for (size_t i = offset; i < line_end; ++i) {
			const int written = snprintf(&text[text_index], sizeof(text) - text_index, "%02X%s", buffer[i],
						     i + 1 < line_end ? " " : "");

			if (written <= 0) {
				break;
			}

			text_index += static_cast<size_t>(written);
		}

		PX4_INFO("%s HEX +%03u: %s", direction, static_cast<unsigned>(offset), text);
	}
}

void TargetFrameUdp::dump_target(const char *direction, const target_frame_udp::Target &target) const
{
	PX4_INFO("%s DEC id=%u lon=%.7f lat=%.7f alt=%.3f speed=%.3f", direction,
		 static_cast<unsigned>(target.target_id),
		 static_cast<double>(target.longitude_e7) * 1e-7,
		 static_cast<double>(target.latitude_e7) * 1e-7,
		 static_cast<double>(target.altitude_m), static_cast<double>(target.speed_m_s));
	PX4_INFO("%s DEC vx=%.3f vy=%.3f vz=%.3f del=%u type=0x%02x attr=0x%02x",
		 direction,
		 static_cast<double>(target.vx_m_s), static_cast<double>(target.vy_m_s),
		 static_cast<double>(target.vz_m_s), static_cast<unsigned>(target.delete_flag),
		 static_cast<unsigned>(target.track_type), static_cast<unsigned>(target.target_attribute));
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

		if (_dump_enabled.load()) {
			dump_frame("RX", buffer, static_cast<size_t>(received));
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
			if (_dump_enabled.load()) {
				dump_target("RX", targets[i]);
			}

			publish_target(targets[i]);
		}
	}
}

void TargetFrameUdp::encode_follower_info()
{
	// Take target data delivered through MAVLink (UAV_INFO -> follower_info),
	// encode it into the radar/display protocol frame and print HEX + decimal.
	// This makes the "GCS -> MAVLink -> PX4 -> protocol HEX" path observable.
	if (!_follower_dump_enabled.load()) {
		return;
	}

	follower_info_s info{};

	while (_follower_info_sub.update(&info)) {
		if (info.mavid == 0 || info.mavid == _vehicle_id || !PX4_ISFINITE(info.lat) ||
		    !PX4_ISFINITE(info.lon) || !PX4_ISFINITE(info.alt) ||
		    info.lat < -90.0 || info.lat > 90.0 || info.lon < -180.0 || info.lon > 180.0) {
			continue;
		}

		target_frame_udp::Target target{};
		target.target_id = static_cast<uint16_t>(info.mavid);
		target.longitude_e7 = static_cast<int32_t>(llround(info.lon * 1e7));
		target.latitude_e7 = static_cast<int32_t>(llround(info.lat * 1e7));

		const double altitude_mm = info.alt * 1000.0;
		target.altitude_mm = static_cast<int32_t>(llround(fmax(static_cast<double>(INT32_MIN),
				     fmin(static_cast<double>(INT32_MAX), altitude_mm))));
		target.altitude_m = static_cast<float>(info.alt);
		target.vx_m_s = static_cast<float>(info.vx);
		target.vy_m_s = static_cast<float>(info.vy);
		target.vz_m_s = static_cast<float>(info.vz);
		target.speed_m_s = sqrtf(target.vx_m_s * target.vx_m_s + target.vy_m_s * target.vy_m_s +
					 target.vz_m_s * target.vz_m_s);
		target.track_type = _options.track_type;
		target.target_attribute = _options.target_attribute;

		uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};
		size_t encoded_length = 0;
		const uint8_t sequence = _tx_sequence.fetch_add(1);

		if (target_frame_udp::encode(&target, 1, sequence,
					     static_cast<uint32_t>(hrt_absolute_time() / 1000),
					     _options.crc_mode, buffer, sizeof(buffer), encoded_length)) {
			dump_frame("MAV", buffer, encoded_length);
			dump_target("MAV", target);
			++_follower_frames;
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
	encode_follower_info();
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

bool TargetFrameUdp::command_send_decimal(int argc, char *argv[])
{
	if (argc != 8) {
		PX4_ERR("usage: target_frame_udp senddec <id> <lon> <lat> <alt> <vx> <vy> <vz>");
		return false;
	}

	target_frame_udp::Target target{};
	double longitude = 0.0;
	double latitude = 0.0;
	float altitude = 0.f;

	if (!parse_target_id(argv[1], target.target_id) ||
	    !parse_double_value(argv[2], longitude) || longitude < -180.0 || longitude > 180.0 ||
	    !parse_double_value(argv[3], latitude) || latitude < -90.0 || latitude > 90.0 ||
	    !parse_float_value(argv[4], altitude) ||
	    !parse_float_value(argv[5], target.vx_m_s) ||
	    !parse_float_value(argv[6], target.vy_m_s) ||
	    !parse_float_value(argv[7], target.vz_m_s)) {
		PX4_ERR("invalid senddec value");
		return false;
	}

	const double altitude_mm = static_cast<double>(altitude) * 1000.0;

	if (altitude_mm < static_cast<double>(INT32_MIN) || altitude_mm > static_cast<double>(INT32_MAX)) {
		PX4_ERR("altitude outside int32 millimetre range");
		return false;
	}

	target.longitude_e7 = static_cast<int32_t>(llround(longitude * 1e7));
	target.latitude_e7 = static_cast<int32_t>(llround(latitude * 1e7));
	target.altitude_mm = static_cast<int32_t>(llround(altitude_mm));
	target.altitude_m = altitude;
	target.speed_m_s = sqrtf(target.vx_m_s * target.vx_m_s + target.vy_m_s * target.vy_m_s +
				target.vz_m_s * target.vz_m_s);
	target.track_type = _options.track_type;
	target.target_attribute = _options.target_attribute;

	if (!send_targets(&target, 1, true)) {
		PX4_ERR("senddec failed");
		return false;
	}

	PX4_INFO("senddec sent 67-byte target frame");
	return true;
}

bool TargetFrameUdp::command_send_hex(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("usage: target_frame_udp sendhex <hex bytes>");
		return false;
	}

	uint8_t buffer[target_frame_udp::MAX_DATAGRAM_SIZE] {};
	size_t length = 0;
	int high_nibble = -1;

	for (int argument = 1; argument < argc; ++argument) {
		const char *text = argv[argument];

		for (size_t index = 0; text[index] != '\0'; ++index) {
			const char value = text[index];

			if ((value == '0') && (text[index + 1] == 'x' || text[index + 1] == 'X') && high_nibble < 0) {
				++index;
				continue;
			}

			if (value == ':' || value == '-' || value == '_' || value == ',') {
				continue;
			}

			const int nibble = hex_nibble(value);

			if (nibble < 0) {
				PX4_ERR("invalid hex character: %c", value);
				return false;
			}

			if (high_nibble < 0) {
				high_nibble = nibble;

			} else {
				if (length >= sizeof(buffer)) {
					PX4_ERR("hex frame exceeds %u bytes", static_cast<unsigned>(sizeof(buffer)));
					return false;
				}

				buffer[length++] = static_cast<uint8_t>((high_nibble << 4) | nibble);
				high_nibble = -1;
			}
		}
	}

	if (high_nibble >= 0 || length == 0) {
		PX4_ERR("hex input must contain complete bytes");
		return false;
	}

	target_frame_udp::Target targets[target_frame_udp::MAX_TARGETS] {};
	target_frame_udp::FrameInfo frame_info{};
	const auto result = target_frame_udp::decode(buffer, length, _options.crc_mode,
			    targets, target_frame_udp::MAX_TARGETS, frame_info);

	if (result != target_frame_udp::DecodeResult::Ok) {
		PX4_ERR("sendhex rejected: %s", target_frame_udp::decode_result_string(result));
		return false;
	}

	for (size_t i = 0; i < frame_info.target_count; ++i) {
		dump_target("HEX", targets[i]);
	}

	if (!send_raw_frame(buffer, length, true)) {
		PX4_ERR("sendhex failed");
		return false;
	}

	_tx_sequence.store(static_cast<uint8_t>(frame_info.sequence + 1));
	PX4_INFO("sendhex sent %u-byte frame with %u target(s)", static_cast<unsigned>(length),
		 static_cast<unsigned>(frame_info.target_count));
	return true;
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

	if (!is_running()) {
		return print_usage("target_frame_udp not running");
	}

	TargetFrameUdp *instance = get_instance();

	if (argc == 2 && strcmp(argv[0], "dump") == 0) {
		if (strcmp(argv[1], "on") == 0) {
			instance->_dump_enabled.store(true);
			PX4_INFO("frame dump enabled");
			return PX4_OK;
		}

		if (strcmp(argv[1], "off") == 0) {
			instance->_dump_enabled.store(false);
			PX4_INFO("frame dump disabled");
			return PX4_OK;
		}
	}

	if (argc == 2 && strcmp(argv[0], "mavdump") == 0) {
		if (strcmp(argv[1], "on") == 0) {
			instance->_follower_dump_enabled.store(true);
			PX4_INFO("follower_info -> protocol HEX dump enabled");
			return PX4_OK;
		}

		if (strcmp(argv[1], "off") == 0) {
			instance->_follower_dump_enabled.store(false);
			PX4_INFO("follower_info -> protocol HEX dump disabled");
			return PX4_OK;
		}
	}

	if (argc == 2 && strcmp(argv[0], "auto") == 0) {
		if (strcmp(argv[1], "on") == 0) {
			instance->_auto_send_enabled.store(true);
			PX4_INFO("automatic ownship transmission enabled");
			return PX4_OK;
		}

		if (strcmp(argv[1], "off") == 0) {
			instance->_auto_send_enabled.store(false);
			PX4_INFO("automatic ownship transmission disabled");
			return PX4_OK;
		}
	}

	if (argc > 0 && strcmp(argv[0], "senddec") == 0) {
		return instance->command_send_decimal(argc, argv) ? PX4_OK : PX4_ERROR;
	}

	if (argc > 0 && strcmp(argv[0], "sendhex") == 0) {
		return instance->command_send_hex(argc, argv) ? PX4_OK : PX4_ERROR;
	}

	return print_usage("unknown command");
}

int TargetFrameUdp::print_status()
{
	PX4_INFO("peer=%s:%u local=%u rate=%.1fHz crc=%s sysid=%" PRIu32 " auto=%d dump=%d mavdump=%d",
		 _options.peer_ip, static_cast<unsigned>(_options.remote_port), static_cast<unsigned>(_options.local_port),
		 static_cast<double>(_options.rate_hz), crc_mode_name(_options.crc_mode), _vehicle_id,
		 static_cast<int>(_auto_send_enabled.load()), static_cast<int>(_dump_enabled.load()),
		 static_cast<int>(_follower_dump_enabled.load()));
	PX4_INFO("tx=%" PRIu64 " rx=%" PRIu64 " targets=%" PRIu64 " dropped=%" PRIu64
		 " sequence_errors=%" PRIu64 " socket_errors=%" PRIu64 " mav_frames=%" PRIu64,
		 _tx_packets, _rx_packets, _rx_targets, _rx_dropped, _rx_sequence_errors, _socket_errors,
		 _follower_frames);
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
	PX4_INFO_RAW("     dump on|off                         Print live TX/RX HEX and decoded decimal data\n");
	PX4_INFO_RAW("     mavdump on|off                      Encode follower_info (from MAVLink UAV_INFO) to protocol HEX and print\n");
	PX4_INFO_RAW("     auto on|off                         Enable or pause automatic ownship frames\n");
	PX4_INFO_RAW("     senddec <id> <lon> <lat> <alt> <vx> <vy> <vz>\n");
	PX4_INFO_RAW("                                         Encode decimal values and send one frame\n");
	PX4_INFO_RAW("     sendhex <FD10...CRC>                 Validate and send one raw HEX frame\n");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int target_frame_udp_main(int argc, char *argv[])
{
	return TargetFrameUdp::main(argc, argv);
}
