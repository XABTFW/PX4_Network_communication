#include "target_frame_codec.hpp"

#include <cstring>

namespace target_frame_udp
{
namespace
{

void put_u16(uint8_t *buffer, size_t offset, uint16_t value)
{
	buffer[offset] = static_cast<uint8_t>(value);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t *buffer, size_t offset, uint32_t value)
{
	buffer[offset] = static_cast<uint8_t>(value);
	buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
	buffer[offset + 2] = static_cast<uint8_t>(value >> 16);
	buffer[offset + 3] = static_cast<uint8_t>(value >> 24);
}

void put_u64(uint8_t *buffer, size_t offset, uint64_t value)
{
	for (size_t i = 0; i < 8; ++i) {
		buffer[offset + i] = static_cast<uint8_t>(value >> (8 * i));
	}
}

void put_float(uint8_t *buffer, size_t offset, float value)
{
	uint32_t bits = 0;
	static_assert(sizeof(bits) == sizeof(value), "float must be IEEE-754 binary32");
	memcpy(&bits, &value, sizeof(bits));
	put_u32(buffer, offset, bits);
}

uint16_t get_u16(const uint8_t *buffer, size_t offset)
{
	return static_cast<uint16_t>(buffer[offset]) |
	       static_cast<uint16_t>(static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

uint32_t get_u32(const uint8_t *buffer, size_t offset)
{
	return static_cast<uint32_t>(buffer[offset]) |
	       (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
	       (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
	       (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

uint64_t get_u64(const uint8_t *buffer, size_t offset)
{
	uint64_t value = 0;

	for (size_t i = 0; i < 8; ++i) {
		value |= static_cast<uint64_t>(buffer[offset + i]) << (8 * i);
	}

	return value;
}

float get_float(const uint8_t *buffer, size_t offset)
{
	const uint32_t bits = get_u32(buffer, offset);
	float value = 0.f;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

uint16_t crc_modbus(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];

		for (unsigned bit = 0; bit < 8; ++bit) {
			crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
		}
	}

	return crc;
}

uint16_t crc_ccitt_false(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < length; ++i) {
		crc ^= static_cast<uint16_t>(data[i] << 8);

		for (unsigned bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
		}
	}

	return crc;
}

uint16_t crc_x25(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];

		for (unsigned bit = 0; bit < 8; ++bit) {
			crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
		}
	}

	return static_cast<uint16_t>(crc ^ 0xFFFF);
}

void encode_target(const Target &target, uint8_t *buffer)
{
	put_u32(buffer, 0, static_cast<uint32_t>(target.longitude_e7));
	put_u32(buffer, 4, static_cast<uint32_t>(target.latitude_e7));
	put_u32(buffer, 8, static_cast<uint32_t>(target.altitude_mm));
	put_u16(buffer, 12, target.target_id);
	put_u16(buffer, 14, target.delete_flag);
	put_float(buffer, 16, target.speed_m_s);
	put_float(buffer, 20, target.distance_m);
	put_float(buffer, 24, target.azimuth_rad);
	put_float(buffer, 28, target.elevation_rad);
	put_float(buffer, 32, target.altitude_m);
	buffer[36] = target.track_type;
	buffer[37] = target.target_attribute;
	put_float(buffer, 38, target.vx_m_s);
	put_float(buffer, 42, target.vy_m_s);
	put_float(buffer, 46, target.vz_m_s);
}

void decode_target(const uint8_t *buffer, Target &target)
{
	target.longitude_e7 = static_cast<int32_t>(get_u32(buffer, 0));
	target.latitude_e7 = static_cast<int32_t>(get_u32(buffer, 4));
	target.altitude_mm = static_cast<int32_t>(get_u32(buffer, 8));
	target.target_id = get_u16(buffer, 12);
	target.delete_flag = get_u16(buffer, 14);
	target.speed_m_s = get_float(buffer, 16);
	target.distance_m = get_float(buffer, 20);
	target.azimuth_rad = get_float(buffer, 24);
	target.elevation_rad = get_float(buffer, 28);
	target.altitude_m = get_float(buffer, 32);
	target.track_type = buffer[36];
	target.target_attribute = buffer[37];
	target.vx_m_s = get_float(buffer, 38);
	target.vy_m_s = get_float(buffer, 42);
	target.vz_m_s = get_float(buffer, 46);
}

} // namespace

size_t frame_size(size_t target_count)
{
	return HEADER_SIZE + target_count * TARGET_SIZE + CRC_SIZE;
}

uint16_t crc16(CrcMode mode, const uint8_t *data, size_t length)
{
	switch (mode) {
	case CrcMode::None: return 0;
	case CrcMode::Modbus: return crc_modbus(data, length);
	case CrcMode::CcittFalse: return crc_ccitt_false(data, length);
	case CrcMode::X25: return crc_x25(data, length);
	}

	return 0;
}

bool encode(const Target *targets, size_t target_count, uint8_t sequence, uint64_t time_ms, CrcMode crc_mode,
	    uint8_t *buffer, size_t capacity, size_t &encoded_length)
{
	encoded_length = frame_size(target_count);

	if (targets == nullptr || buffer == nullptr || target_count == 0 || target_count > MAX_TARGETS ||
	    capacity < encoded_length || encoded_length > UINT16_MAX) {
		encoded_length = 0;
		return false;
	}

	buffer[0] = FRAME_HEADER;
	buffer[1] = PAYLOAD_LENGTH_FIELD;
	buffer[2] = sequence;
	buffer[3] = MESSAGE_TYPE;
	buffer[4] = RESERVED;
	buffer[5] = 0xC5;
	buffer[6] = 0xCE;
	buffer[7] = 0xC2;
	put_u64(buffer, 8, time_ms);
	put_u16(buffer, 16, static_cast<uint16_t>(encoded_length));
	buffer[18] = static_cast<uint8_t>(target_count);

	for (size_t i = 0; i < target_count; ++i) {
		encode_target(targets[i], &buffer[HEADER_SIZE + i * TARGET_SIZE]);
	}

	put_u16(buffer, encoded_length - CRC_SIZE, crc16(crc_mode, buffer, encoded_length - CRC_SIZE));
	return true;
}

DecodeResult decode(const uint8_t *buffer, size_t length, CrcMode crc_mode, Target *targets, size_t target_capacity,
		    FrameInfo &frame_info)
{
	if (buffer == nullptr || length < frame_size(1)) { return DecodeResult::TooShort; }
	if (buffer[0] != FRAME_HEADER || buffer[3] != MESSAGE_TYPE || buffer[4] != RESERVED) { return DecodeResult::BadHeader; }
	if (buffer[5] != 0xC5 || buffer[6] != 0xCE || buffer[7] != 0xC2) { return DecodeResult::BadIdentifier; }
	if (buffer[1] != PAYLOAD_LENGTH_FIELD) { return DecodeResult::BadPayloadLength; }

	const size_t target_count = buffer[18];

	if (target_count == 0 || target_count > MAX_TARGETS || targets == nullptr || target_count > target_capacity) {
		return DecodeResult::BadTargetCount;
	}

	const size_t expected_length = frame_size(target_count);

	if (length != expected_length || get_u16(buffer, 16) != expected_length) { return DecodeResult::BadLength; }

	if (crc_mode != CrcMode::None &&
	    get_u16(buffer, length - CRC_SIZE) != crc16(crc_mode, buffer, length - CRC_SIZE)) {
		return DecodeResult::BadCrc;
	}

	frame_info.sequence = buffer[2];
	frame_info.time_ms = get_u64(buffer, 8);
	frame_info.target_count = static_cast<uint8_t>(target_count);

	for (size_t i = 0; i < target_count; ++i) {
		decode_target(&buffer[HEADER_SIZE + i * TARGET_SIZE], targets[i]);
	}

	return DecodeResult::Ok;
}

const char *decode_result_string(DecodeResult result)
{
	switch (result) {
	case DecodeResult::Ok: return "ok";
	case DecodeResult::TooShort: return "too short";
	case DecodeResult::BadHeader: return "bad header";
	case DecodeResult::BadIdentifier: return "bad identifier";
	case DecodeResult::BadPayloadLength: return "bad payload length";
	case DecodeResult::BadTargetCount: return "bad target count";
	case DecodeResult::BadLength: return "bad length";
	case DecodeResult::BadCrc: return "bad CRC";
	}

	return "unknown";
}

} // namespace target_frame_udp
