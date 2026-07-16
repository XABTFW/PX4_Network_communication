#pragma once

#include <cstddef>
#include <cstdint>

namespace target_frame_udp
{

static constexpr uint8_t FRAME_HEADER = 0xFD;
static constexpr uint8_t PAYLOAD_LENGTH_FIELD = 0x10;
static constexpr uint8_t MESSAGE_TYPE = 0x01;
static constexpr uint8_t RESERVED = 0x01;
static constexpr size_t HEADER_SIZE = 15;
static constexpr size_t TARGET_SIZE = 50;
static constexpr size_t CRC_SIZE = 2;
static constexpr size_t MAX_DATAGRAM_SIZE = 1472;
static constexpr size_t MAX_TARGETS = (MAX_DATAGRAM_SIZE - HEADER_SIZE - CRC_SIZE) / TARGET_SIZE;

enum class CrcMode : uint8_t {
	None = 0,
	Modbus,
	CcittFalse,
	X25
};

enum class DecodeResult : uint8_t {
	Ok = 0,
	TooShort,
	BadHeader,
	BadIdentifier,
	BadPayloadLength,
	BadTargetCount,
	BadLength,
	BadCrc
};

struct Target {
	int32_t longitude_e7{0};
	int32_t latitude_e7{0};
	int32_t altitude_mm{0};
	uint16_t target_id{0};
	uint16_t delete_flag{0};
	float speed_m_s{0.f};
	float distance_m{0.f};
	float azimuth_rad{0.f};
	float elevation_rad{0.f};
	float altitude_m{0.f};
	uint8_t track_type{0x20};
	uint8_t target_attribute{0x22};
	float vx_m_s{0.f};
	float vy_m_s{0.f};
	float vz_m_s{0.f};
};

struct FrameInfo {
	uint8_t sequence{0};
	uint32_t time_ms{0};
	uint8_t target_count{0};
};

size_t frame_size(size_t target_count);
uint16_t crc16(CrcMode mode, const uint8_t *data, size_t length);

bool encode(const Target *targets, size_t target_count, uint8_t sequence, uint32_t time_ms, CrcMode crc_mode,
	    uint8_t *buffer, size_t capacity, size_t &encoded_length);

DecodeResult decode(const uint8_t *buffer, size_t length, CrcMode crc_mode, Target *targets, size_t target_capacity,
		    FrameInfo &frame_info);

const char *decode_result_string(DecodeResult result);

} // namespace target_frame_udp
