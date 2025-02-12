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

#include "extbus.h"
#include "exbus_decode.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <iostream>
#include <utility>

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
 * 0x06 – Packet ID
 * 0x31 – The data identifier – channel values
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
	uint8_t raw_data[data_len] = {
	    0x3E, 0x03, 0x28, 0x06, 0x31, 0x20, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x4F, 0xE2,
	};
	uint8_t *data_pointer = raw_data;
};

/** TODO
 * + [x] Parse out header data to Struct
 * + [ ] Calc CRC
 * + [ ] Validate Msg via CRC
 * + [ ] Get individual channel values
 * + [ ] Signal problem of channel overreaching - requesting wrong number
 */

namespace JETI {

} // namespace JETI

TEST_F(JETIChannelData, ParseHeader) {
	const JETI::Header header = {
	    .H0 = JETI::consts::head_h_channel_data,
	    .H1 = JETI::consts::head_l_without_scope,
	    .len = 0x28,
	    .Packet_ID = 0x06,
	    .Data_ID = 0x31,
	    .Channels = 0x10,
	};

	const JETI::Header testHeader = JETI::GetHeader(raw_data);

	EXPECT_EQ(header, JETI::GetHeader(raw_data));
	EXPECT_EQ(header, testHeader);
	EXPECT_EQ(0x10, testHeader.Channels);
}

TEST_F(JETIChannelData, RecognizeHeader) {
	const auto head = JETI::GetHeader(raw_data);

	EXPECT_TRUE(JETI::IsChannels(head));
}

TEST_F(JETIChannelData, CalculateFirstChannelValue) {
	const auto ch_id{0};
	auto channel = JETI::GetChannel(raw_data, data_len, ch_id);
	EXPECT_LE(1.00825f - channel, 0.000000001f);
}

TEST_F(JETIChannelData, CalculateSecondChannelValue) {
	const auto ch_id{1};
	auto channel = JETI::GetChannel(raw_data, data_len, ch_id);
	EXPECT_LE(1.00825f - channel, 0.000000001f);
}

TEST_F(JETIChannelData, ExtractCrcValues) {
	const JETI::CRC crc = {
	    .crc0 = 0x4F,
	    .crc1 = 0xE2,
	};
	JETI::Header testHeader = JETI::GetHeader(raw_data);

	EXPECT_EQ(crc, JETI::ExtractCrcValues(raw_data, testHeader));
}

TEST_F(JETIChannelData, GetCRC) {

	JETI::Header testHeader = JETI::GetHeader(raw_data);
	EXPECT_EQ(0xE24F, JETI::GetCRC(raw_data, testHeader));
}

TEST_F(JETIChannelData, GetCRC16Update) {

	EXPECT_EQ(0xD8FD, JETI::crc16_update(0x00, JETI::consts::head_h_channel_data));
}

TEST_F(JETIChannelData, GetCRCwithChecksum) {

	JETI::Header testHeader = JETI::GetHeader(raw_data);
	EXPECT_EQ(0xE24F, JETI::Get_crc16z(data_pointer, testHeader));
}

TEST_F(JETIChannelData, ValidateChecksum) {

	JETI::Header testHeader = JETI::GetHeader(raw_data);
	EXPECT_TRUE(JETI::ValidateMsg(data_pointer, raw_data, testHeader));
}

TEST_F(JETIChannelData, CheckIfChannelIsOverreached) {

	EXPECT_FALSE(JETI::CheckChannelOverreach(5, raw_data));
	EXPECT_FALSE(JETI::CheckChannelOverreach(16, raw_data));
	EXPECT_TRUE(JETI::CheckChannelOverreach(28, raw_data));
}

TEST_F(JETIChannelData, jetiClassDecode){
	JETI::DecodeVariable decode;

	//Check whether updateCurrentByte() works
	decode.updateCurrentByte(JETI::consts::head_l_without_scope);
	EXPECT_EQ(JETI::consts::head_l_without_scope, decode.current_byte);

        //Check whether we reach GOT_HEADER_BYTE_HIGH
	decode.updateCurrentByte(JETI::consts::head_h_channel_data);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH, decode.current_state);

        //Check whether we reach GOT_HEADER_BYTE_LOW
	decode.updateCurrentByte(JETI::consts::head_l_without_scope);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW, decode.current_state);

        //Check whether we reach GOT_LEN
	decode.updateCurrentByte(0x28);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_LEN, decode.current_state);

        //Check whether we reach GOT_PACKET_ID
	decode.updateCurrentByte(0x06);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_PACKET_ID, decode.current_state);

        //Check whether we reach GOT_DATA_TYPES
	decode.updateCurrentByte(0x31);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_TYPES, decode.current_state);

        //Check whether a too large data length will pass
	decode.updateCurrentByte(0x31);
	EXPECT_FALSE(decode.checkValidDataLength());

	//Check whether a too short data lenth will pass
	decode.updateCurrentByte(0x08);
	EXPECT_FALSE(decode.checkValidDataLength());

	//Check whether we reach GOT_HEADER_BYTE_LOW
        decode.updateCurrentByte(0x20);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_LEN, decode.current_state);

        //Check whether we reach GOT_DATA_CHANNEL_LOW
	decode.updateCurrentByte(0x82);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW, decode.current_state);

        //Check whether we reach GOT_DATA_CHANNEL_HIGH
	decode.updateCurrentByte(0x1F);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH, decode.current_state);


	for(int i = 0; i < 15; i++){
	decode.updateCurrentByte(0x82);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW, decode.current_state);
	decode.updateCurrentByte(0x1F);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH, decode.current_state);
	}

        //Check whether we reach GOT_CRC16_BYTE_LOW
	decode.updateCurrentByte(0x4F);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW, decode.current_state);
}

TEST_F(JETIChannelData, jetiDecodeDefault){
	JETI::DecodeVariable decode;
	decode.updateCurrentByte(JETI::consts::head_h_channel_data);
	decode.decodePackage();
	decode.updateCurrentByte(JETI::consts::head_l_without_scope);
	decode.decodePackage();
	decode.updateCurrentByte(0x02);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::UNSYNCED, decode.current_state);
}

TEST_F(JETIChannelData, jetiDecodeRequestMenu){
	JETI::DecodeVariable decode;
	decode.updateCurrentByte(JETI::consts::head_h_request);
	decode.decodePackage();
	decode.updateCurrentByte(JETI::consts::head_l_with_scope);
	decode.decodePackage();
	decode.updateCurrentByte(0x09);
	decode.decodePackage();
	decode.updateCurrentByte(0x88);
	decode.decodePackage();
	decode.updateCurrentByte(0x3B);
	decode.decodePackage();
	decode.updateCurrentByte(0x01);
	decode.decodePackage();
	decode.updateCurrentByte(0xF0);
	decode.decodePackage();
	decode.updateCurrentByte(0xA3);
	decode.decodePackage();
	decode.updateCurrentByte(0x24);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_CRC16_BYTE_HIGH, decode.current_state);
}

TEST_F(JETIChannelData, jetiDecodeRequestTelemetry){
	JETI::DecodeVariable decode;
	decode.updateCurrentByte(JETI::consts::head_h_request);
	decode.decodePackage();
	decode.updateCurrentByte(JETI::consts::head_l_with_scope);
	decode.decodePackage();
	decode.updateCurrentByte(0x08);
	decode.decodePackage();
	decode.updateCurrentByte(0x06);
	decode.decodePackage();
	decode.updateCurrentByte(0x3A);
	decode.decodePackage();
	decode.updateCurrentByte(0x00);
	decode.decodePackage();
	decode.updateCurrentByte(0x98);
	decode.decodePackage();
	decode.updateCurrentByte(0x81);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_CRC16_BYTE_HIGH, decode.current_state);
}

