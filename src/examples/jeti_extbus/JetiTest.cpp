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
	const uint8_t raw_data[data_len] = {
	    0x3E, 0x03, 0x28, 0x06, 0x31, 0x20, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
	    0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x4F, 0xE2,
	};
	const uint8_t *data_pointer = raw_data;
};

/** TODO
 * + [x] Parse out header data to Struct
 * + [ ] Calc CRC
 * + [ ] Validate Msg via CRC
 * + [ ] Get individual channel values
 * + [ ] Signal problem of channel overreaching - requesting wrong number
 */

namespace JETI {

auto jeti_decode_channel_high(uint16_t channel, const uint8_t value)
    -> std::pair<JETI::DECODE_STATE, uint16_t> {
	channel += value << 8;
	return std::make_pair(DECODE_STATE::GOT_DATA_CHANNEL_LOW, channel);
}

auto jetiDecode(const uint8_t byte, enum JETI::DECODE_STATE decode_state)
    -> std::pair<DECODE_STATE, uint16_t> {

	static uint16_t channel_value{0};

	using namespace JETI;

	switch (decode_state) {
	case DECODE_STATE::UNSYNCED:
		if (byte == consts::head_h) {
			return std::make_pair(DECODE_STATE::GOT_HEADER_BYTE_HIGH,
					      0u);
		}
		break;

	case JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH:
		if (byte == consts::head_l) {
			return std::make_pair(DECODE_STATE::GOT_HEADER_BYTE_LOW,
					      0u);
		}
		break;

	case JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW:
		return std::make_pair(DECODE_STATE::GOT_LEN, 0u);
		break;

	case JETI::DECODE_STATE::GOT_LEN:
		if (byte == 0x06)
			return std::make_pair(DECODE_STATE::GOT_PACKET_ID, 0u);
		break;

	case JETI::DECODE_STATE::GOT_PACKET_ID:
		if (byte == 0x31)
			return std::make_pair(DECODE_STATE::GOT_DATA_LEN, 0u);
		break;

	case JETI::DECODE_STATE::GOT_DATA_LEN:
		if (byte == 0x00)
			return std::make_pair(DECODE_STATE::UNSYNCED, 0u);

		return std::make_pair(DECODE_STATE::GOT_DATA_CHANNEL_LOW, byte);
		break;

	case JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW:
		channel_value = byte;
		return std::make_pair(DECODE_STATE::GOT_DATA_CHANNEL_HIGH, 0);

	case JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH:
		return jeti_decode_channel_high(channel_value, byte);
		break;

	default:
		return std::make_pair(DECODE_STATE::UNSYNCED, 0u);
		break;
	}

	return std::make_pair(DECODE_STATE::UNSYNCED, channel_value);
}


class DecodeVariable{
public:

	JETI::DECODE_STATE current_state;
	uint8_t current_byte;
	uint8_t data_package[40];
	int data_channel_count;

	DecodeVariable() : current_state(JETI::DECODE_STATE::UNSYNCED), current_byte(0), data_channel_count(0){};

	void decodePackage(){
		switch(current_state) {
		case JETI::DECODE_STATE::UNSYNCED:
		    if(checkValidHeaderHigh()){
		  	updateCurrentState(JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH);
			updateDataPackage(0, current_byte);
		    }
		    break;

		case JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH:
		    if(checkValidHeaderLow()){
			updateCurrentState(JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW);
			updateDataPackage(1, current_byte);
		    } else {
			returnToUnsyncState();
		    }
		    break;

		case JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW:
		    if(checkValidLength()){
			updateCurrentState(JETI::DECODE_STATE::GOT_LEN);
			updateDataPackage(2, current_byte);
		    } else {
			returnToUnsyncState();
		    }
		    break;

		case JETI::DECODE_STATE::GOT_LEN:
		    updateCurrentState(JETI::DECODE_STATE::GOT_PACKET_ID);
		    updateDataPackage(3, current_byte);
		    break;

		case JETI::DECODE_STATE::GOT_PACKET_ID:
		    updateCurrentState(JETI::DECODE_STATE::GOT_DATA_TYPES);
		    updateDataPackage(4, current_byte);
		    break;

		case JETI::DECODE_STATE::GOT_DATA_TYPES:
		    if(checkValidDataLength()){
			updateCurrentState(JETI::DECODE_STATE::GOT_DATA_LEN);
			updateDataPackage(5, current_byte);
		    } else {
			returnToUnsyncState();
		    }
		    break;

		case JETI::DECODE_STATE::GOT_DATA_LEN:
		    updateCurrentState(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW);
		    updateDataChannel();
		    break;

		case JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW:
		    updateCurrentState(JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH);
		    updateDataChannel();
		    break;

		case JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH:
		    if(checkLastDataChannel()){
			updateCurrentState(JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW);
		        updateDataChannel();
			break;
		    } else {
			updateCurrentState(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW);
		        updateDataChannel();
			break;
		    }
		    break;

		case JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW:
		    updateCurrentState(JETI::DECODE_STATE::GOT_CRC16_BYTE_HIGH);
		    updateDataChannel();
		    break;

                default:
		    current_state = JETI::DECODE_STATE::UNSYNCED;
		    break;
		}
	};

	void updateCurrentByte(uint8_t new_byte){
		current_byte = new_byte;
	};

	void updateCurrentState(JETI::DECODE_STATE new_state){
		current_state = new_state;
	}

	bool checkValidHeaderHigh(){
		return current_byte == consts::head_h;
	};

	bool checkValidHeaderLow(){
		return current_byte == consts::head_l;
	}

	bool checkValidLength(){
		return current_byte > 9 && current_byte%2 == 0; //smallest package contains at least ten bytes and is an even number
	}

	bool checkValidDataLength(){
		return current_byte == data_package[2] - 8;
	}

	void returnToUnsyncState(){
		current_state = JETI::DECODE_STATE::UNSYNCED;
	}

	bool checkLastDataChannel(){
		return data_channel_count == data_package[5];
	}

	void updateDataChannel(){
		if(!outOfBounds()){
			updateDataPackage((6+data_channel_count), current_byte);
			data_channel_count += 1;
		} else {
			printf("Data is out of bounds!\n\n");
		}
	}

	void updateDataPackage(int entry_position, uint8_t entry_val){
		data_package[entry_position] = entry_val;
	}

	bool outOfBounds(){
		return data_channel_count > data_package[5]+2;
	}
};

} // namespace JETI

TEST_F(JETIChannelData, ParseHeader) {
	const JETI::Header header = {
	    .H0 = JETI::consts::head_h,
	    .H1 = JETI::consts::head_l,
	    .len = 0x28,
	    .Packet_ID = 0x06,
	    .Data_ID = 0x31,
	    .Channels = 0x10,
	};

	const JETI::Header testHeader = JETI::GetHeader(raw_data, data_len);

	EXPECT_EQ(header, JETI::GetHeader(raw_data, data_len));
	EXPECT_EQ(header, testHeader);
	EXPECT_EQ(0x10, testHeader.Channels);
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

	EXPECT_EQ(crc, JETI::ExtractCrcValues(raw_data, data_len));
}

TEST_F(JETIChannelData, GetCRC) {

	EXPECT_EQ(0xE24F, JETI::GetCRC(raw_data, data_len));
}

TEST_F(JETIChannelData, GetCRC16Update) {

	EXPECT_EQ(0xD8FD, JETI::crc16_update(0x00, JETI::consts::head_h));
}

TEST_F(JETIChannelData, GetCRCwithChecksum) {

	EXPECT_EQ(0xE24F, JETI::Get_crc16z(data_pointer, data_len));
}

TEST_F(JETIChannelData, ValidateChecksum) {

	EXPECT_TRUE(JETI::ValidateMsg(data_pointer, raw_data, data_len));
}

TEST_F(JETIChannelData, CheckIfChannelIsOverreached) {

	EXPECT_FALSE(JETI::CheckChannelOverreach(5, raw_data, data_len));
	EXPECT_FALSE(JETI::CheckChannelOverreach(16, raw_data, data_len));
	EXPECT_TRUE(JETI::CheckChannelOverreach(28, raw_data, data_len));
}

TEST_F(JETIChannelData, jetiDecode) {

	/*
	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH,
		  JETI::jetiDecode(JETI::consts::head_h,
				   JETI::DECODE_STATE::UNSYNCED));

	EXPECT_EQ(JETI::DECODE_STATE::UNSYNCED,
		  JETI::jetiDecode(0x00, JETI::DECODE_STATE::UNSYNCED));

	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW,
		  JETI::jetiDecode(JETI::consts::head_l,
				   JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH));

	EXPECT_EQ(
	    JETI::DECODE_STATE::UNSYNCED,
	    JETI::jetiDecode(0x00, JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH));

	EXPECT_EQ(
	    JETI::DECODE_STATE::GOT_LEN,
	    JETI::jetiDecode(0x28, JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW));

	// How to handle the different message lenght!!!
	// EXPECT_EQ(JETI::DECODE_STATE::GOT_LEN, JETI::jetiDecode(0x0,
	// JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW));

	EXPECT_EQ(JETI::DECODE_STATE::GOT_PACKET_ID,
		  JETI::jetiDecode(0x06, JETI::DECODE_STATE::GOT_LEN));

	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_LEN,
		  JETI::jetiDecode(0x31, JETI::DECODE_STATE::GOT_PACKET_ID));

	EXPECT_EQ(JETI::DECODE_STATE::UNSYNCED,
		  JETI::jetiDecode(0x00, JETI::DECODE_STATE::GOT_DATA_LEN));

		  */
	auto result = JETI::jetiDecode(0x20, JETI::DECODE_STATE::GOT_DATA_LEN);

	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW, result.first);
	EXPECT_EQ(32, result.second);

	/**
	 * + TODO:
	 * + How are we sure that we reached the next state?
	 * + How to ensure Packge length and data lenght are mathcing?
	 * + How to signal, that the current/next byte is already crc?
	 * + How to handle 4,6,10,12 channels?
	 * + How to handle incorrect channel value like not even value?  like 1,
	 * 5?
	 * + Separate SM to standlone cpp/h files
	 * + Convert SM to a class to help pass variables
	 * +....
	 */

	// result = JETI::jetiDecode(0x82, result.first);
	result = JETI::jetiDecode(0x82, JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW);

	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH, result.first);
	EXPECT_EQ(0, result.second);

	result = JETI::jetiDecode(0x1F, JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH);

	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW, result.first);
	EXPECT_EQ(0x1F82U, result.second);
}

TEST_F(JETIChannelData, jetiClassDecode){
	JETI::DecodeVariable decode;

	decode.updateCurrentByte(JETI::consts::head_l);
	EXPECT_EQ(JETI::consts::head_l, decode.current_byte);

	decode.updateCurrentByte(JETI::consts::head_h);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_HIGH, decode.current_state);

	decode.updateCurrentByte(JETI::consts::head_l);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_HEADER_BYTE_LOW, decode.current_state);

	decode.updateCurrentByte(0x28);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_LEN, decode.current_state);

	decode.updateCurrentByte(0x06);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_PACKET_ID, decode.current_state);

	decode.updateCurrentByte(0x31);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_TYPES, decode.current_state);


	decode.updateCurrentByte(0x31);
	EXPECT_FALSE(decode.checkValidDataLength());
	decode.updateCurrentByte(0x08);
	EXPECT_FALSE(decode.checkValidDataLength());
        decode.updateCurrentByte(0x20);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_LEN, decode.current_state);


	decode.updateCurrentByte(0x82);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW, decode.current_state);

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

	decode.updateCurrentByte(0x4F);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW, decode.current_state);
}

TEST_F(JETIChannelData, jetiDecodeDefault){
	JETI::DecodeVariable decode;
	decode.updateCurrentByte(JETI::consts::head_h);
	decode.decodePackage();
	decode.updateCurrentByte(JETI::consts::head_l);
	decode.decodePackage();
	decode.updateCurrentByte(0x02);
	decode.decodePackage();
	EXPECT_EQ(JETI::DECODE_STATE::UNSYNCED, decode.current_state);
}



