/****************************************************************************
 *
 *   Copyright (C) 2019-2024 PX4 Development Team. All rights reserved.
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

#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>

/**
 * Basic unit tests to drive the core functionality
 * of the JETI ext protokol reception side
 */

/**
 * A sample msg from data sheet, page 6.
 *
 * Macket From Maset
 *
 * 0x3E 0x03 0x28 0x06 0x31 0x20 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82
 * 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F
 * 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x4F 0xE2
 *
 * 0x3E 0x03 – Packet header that forbids answering
 * 0x28 – Message length (40)
 * 0x06 – Packet ID 0x31 – The data identifier – channel values
 * 0x20 – Length of data blocks (32) - 16 channels x 2B
 * 0x1F82 – Value of the 1st channel (8066)/8'000'000 = 1,00825ms
 *
 * 0xE24F - CRC16-CCITT
 *
 */

class JETIChannelData : public testing::Test {
      protected:
	JETIChannelData() {}

	static const size_t data_len{40u};
	const uint8_t raw_data[data_len] = {
	    0x3E, 0x03, 0x28, 0x06, 0x31, 0x20, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x4F, 0xE2,
	};
	const uint8_t* data_pointer = raw_data;
};

/** TODO
 * + [x] Parse out header data to Struct
 * + [ ] Calc CRC
 * + [ ] Validate Msg via CRC
 * + [ ] Get individual channel values
 * + [ ] Signal problem of channel overreaching - requesting wrong number
 */

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

bool operator==(Header const &lhs, Header const &rhs) {
	if (lhs.H0 != rhs.H0) {
		return false;
	}
	if (lhs.H1 != rhs.H1) {
		return false;
	}
	if (lhs.len != rhs.len) {
		return false;
	}
	if (lhs.Packet_ID != rhs.Packet_ID) {
		return false;
	}
	if (lhs.Data_ID != rhs.Data_ID) {
		return false;
	}
	if (lhs.Channels != rhs.Channels) {
		return false;
	}
	return true;
}

bool operator==(CRC const &lhs, CRC const &rhs){
	if (lhs.crc0 != rhs.crc0) {
		return false;
	}
	if (lhs.crc1 != rhs.crc1) {
		return false;
	}
	return true;
}

bool IsChannels(const JETI::Header head) {
	if (head.H0 != 0x3E) {
		return false;
	}
	if (not(head.H1 == 0x03 || head.H1 == 0x01)) {
		return false;
	}

	return (head.Data_ID == 0x31) ? (true) : (false);
}

auto GetHeader(const uint8_t data[], size_t len) -> JETI::Header {
	// Checksum check
	auto head = JETI::Header{};

	head.H0 = data[0];
	head.H1 = data[1];
	head.len = data[2];
	head.Packet_ID = data[3];
	head.Data_ID = data[4];
	head.Channels = data[5] >> 1;
	return head;
}


union raw_channel_t {
	uint16_t data;
	uint8_t raw[2];
};

// Introduce a Strong Type here for CH Index
// Pretest if enough channel is avaiable or not!
auto GetChannel(const uint8_t data[], const size_t len,
		const uint8_t idx) -> float {
	raw_channel_t raw_channel = {.data = 0};
	if (idx == 0) {
		raw_channel.raw[0] = data[6];
		raw_channel.raw[1] = data[7];
	}
	return static_cast<float>(raw_channel.data) / 8'000;
}

auto ExtractCrcValues(const uint8_t data[], const size_t len) {
	auto crc = JETI::CRC{};

	crc.crc0 = data[len-2];
	crc.crc1 = data[len-1];

	return crc;
}

uint16_t GetCRC(const uint8_t data[], const size_t len) {
	JETI::CRC crcExtracted = JETI::ExtractCrcValues(data, len);

	return (crcExtracted.crc1*0x100)^crcExtracted.crc0;
}

uint64_t crc16_update_operation8(uint16_t crc, uint16_t data){
	return ((uint32_t)(data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ (data << 3);
}

uint64_t crc16_update( uint64_t crc, uint16_t data )  // called crc_ccitt_update before
{
uint64_t ret_val;
data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
data = (data ^ (data << 4));
ret_val = ((uint32_t)(data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ (data << 3);
return ret_val;
}

uint64_t get_crc16z(const uint8_t *p, uint16_t len)
{
uint64_t crc16_data=0;
while(len--) { crc16_data=crc16_update(crc16_data, p[0]); p++; }
return(crc16_data);
}

} // namespace JETI

TEST_F(JETIChannelData, ParseHeader) {
	const JETI::Header header = {
	    .H0 = 0x3E,
	    .H1 = 0x03,
	    .len = 0x28,
	    .Packet_ID = 0x06,
	    .Data_ID = 0x31,
	    .Channels = 0x10,
	};

	EXPECT_EQ(header, JETI::GetHeader(raw_data, data_len));
}

TEST_F(JETIChannelData, RecognizeHeader) {
	const auto head = JETI::GetHeader(raw_data, data_len);

	EXPECT_TRUE(JETI::IsChannels(head));
}

TEST_F(JETIChannelData, CalculateFirstChannelValue) {
	const auto ch_id{0};
	auto channel = JETI::GetChannel(raw_data, data_len, ch_id);
	EXPECT_LE(1.00825f - channel, 0.000000001f);
}

TEST_F(JETIChannelData, ExtractCrcValues) {
	const JETI::CRC crc = {
		.crc0 = 0x4F,
		.crc1 = 0xE2,
	};

	EXPECT_EQ(crc, JETI::ExtractCrcValues(raw_data, data_len));
}

TEST_F(JETIChannelData, GetCRC) {

	EXPECT_EQ(0xE24F, JETI::GetCRC(raw_data, data_len));
}

TEST_F(JETIChannelData, ChecksumCheckOperation) {
	EXPECT_EQ(0x03C0CD, JETI::crc16_update_operation8(0x00, 0x3DE));
}

TEST_F(JETIChannelData, GetCRC16Update) {

 	EXPECT_EQ(0x03C0CD, JETI::crc16_update(0x00, 0x3E));
}

TEST_F(JETIChannelData, GetCRCwithChecksum) {

 	EXPECT_EQ(0xE24F, JETI::get_crc16z(data_pointer, data_len));
}
