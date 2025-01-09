#pragma once

#include <vector>

namespace JETI {

struct Header {
	uint8_t H0;
	uint8_t H1;
	uint8_t len;
	uint8_t Packet_ID;
	uint8_t Data_ID;
	uint8_t Channels;

	friend bool operator==(Header const &lhs, Header const &rhs);
	// this Requires CPP-20
	//  friend bool operator<=>(Header const &lhs, Header const &rhs) =
	//  default;
};


struct CRC {
	uint8_t crc0;
	uint8_t crc1;

	friend bool operator==(CRC const &lhs, CRC const &rhs);
};

union raw_channel_t {
	uint16_t data;
	uint8_t raw[2];
};

bool operator==(Header const &lhs, Header const &rhs);

bool operator==(CRC const &lhs, CRC const &rhs);

bool IsChannels(const JETI::Header head);

auto GetHeader(const uint8_t data[], size_t len) -> JETI::Header;

auto GetChannel(const uint8_t data[], const size_t len, const uint8_t idx) -> float;

auto ExtractCrcValues(const uint8_t data[], const size_t len) -> JETI::CRC;

uint16_t GetCRC(const uint8_t data[], const size_t len);

uint16_t crc16_update( uint16_t crc, uint8_t data );

uint16_t Get_crc16z(const uint8_t *p, uint16_t len);

bool ValidateMsg(const uint8_t *p, const uint8_t data[], const size_t len);

std::vector<float> GetChannelValues(const uint8_t data[], const size_t len);

bool CheckChannelOverreach(int askedChannels, const uint8_t data[], const size_t len);
}
