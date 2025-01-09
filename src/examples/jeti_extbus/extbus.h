#pragma once

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


}
