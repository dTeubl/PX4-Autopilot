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
#include <iostream>
#include "extbus.h"


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


uint8_t* createDataArray(int size) {
    uint8_t* arr = new uint8_t[size];
    arr[0] = 0x3E;
    arr[1] = 0x03;
    arr[2] = (uint8_t)size;
    return arr;
}

int jetiDecode(uint8_t byte, enum JETI_DECODE_STATE decode_state) {

    int ret = 0;

    switch(decode_state) {
        case JETI_DECODE_STATE_UNSYNCED:
            if (byte == 0x3E) {
	        decode_state = JETI_DECODE_STATE_GOT_HEADER_BYTE_1;
	        return 1;
            }
            return 0;
	    break;

        case JETI_DECODE_STATE_GOT_HEADER_BYTE_1:
            if (byte == 0x03) {
		decode_state = JETI_DECODE_STATE_GOT_HEADER_BYTE_2;
	        return 2;
            }
            return 0;
	    break;

	case JETI_DECODE_STATE_GOT_HEADER_BYTE_2:
	    decode_state = JETI_DECODE_STATE_GOT_LEN;
	    ret = (int)byte;
	    break;

	case JETI_DECODE_STATE_GOT_LEN:
	    decode_state = JETI_DECODE_STATE_GOT_ID;
	    ret = (int)byte;
	    break;

	case JETI_DECODE_STATE_GOT_ID:
            decode_state = JETI_DECODE_STATE_GOT_DATA_ID;
	    ret = (int)byte;
	    break;

	case JETI_DECODE_STATE_GOT_DATA_ID:
	    decode_state = JETI_DECODE_STATE_GOT_DATA_LEN;
	    ret = (int)byte;
	    break;

        default:
            ret = 0;
	    break;
    }

    return ret; // Return a value indicating successful processing
}

uint8_t* recreateData(JETI::Header header, JETI::CRC crc) {
    uint8_t* arr = new uint8_t[(int)header.len];
    arr[0] = header.H0;
    arr[1] = header.H1;
    arr[2] = header.len;
    arr[3] = header.Packet_ID;
    arr[4] = header.Data_ID;

    for (int i = 0; i < ((int)header.Channels*2); i++) {
	//fill array with correct channel value
    }

    arr[(int)header.len-2] = crc.crc0;
    arr[(int)header.len-1] = crc.crc1;

    return arr;
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

         EXPECT_EQ(0xD8FD, JETI::crc16_update(0x00, 0x3E));
}

TEST_F(JETIChannelData, GetCRCwithChecksum) {

         EXPECT_EQ(0xE24F, JETI::Get_crc16z(data_pointer, data_len));
}

TEST_F(JETIChannelData, ValidateChecksum) {

         EXPECT_TRUE(JETI::ValidateMsg(data_pointer, raw_data, data_len));
}

TEST_F(JETIChannelData, CheckIfChannelIsOverreached) {

        EXPECT_FALSE(JETI::CheckChannelOverreach(5,raw_data,data_len));
        EXPECT_FALSE(JETI::CheckChannelOverreach(16,raw_data,data_len));
        EXPECT_TRUE(JETI::CheckChannelOverreach(28,raw_data,data_len));

}

TEST_F(JETIChannelData, jetiDecode) {
	EXPECT_EQ(1, JETI::jetiDecode(0x3E, JETI::JETI_DECODE_STATE_UNSYNCED));
	EXPECT_EQ(2, JETI::jetiDecode(0x03, JETI::JETI_DECODE_STATE_GOT_HEADER_BYTE_1));
	EXPECT_EQ(40, JETI::jetiDecode(0x28, JETI::JETI_DECODE_STATE_GOT_HEADER_BYTE_2));
        EXPECT_EQ(6, JETI::jetiDecode(0x06, JETI::JETI_DECODE_STATE_GOT_LEN));
}

