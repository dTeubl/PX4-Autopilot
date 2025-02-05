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

#pragma once

#include <cstdint>

namespace JETI {

namespace consts {
constexpr std::uint8_t head_h_channel_data{0x3EU};
constexpr std::uint8_t head_h_request{0x3DU};
constexpr std::uint8_t head_l_with_scope{0x01};
constexpr std::uint8_t head_l_without_scope{0x03U};
} // namespace consts

enum DECODE_STATE {
	UNSYNCED = 0,
	GOT_HEADER_BYTE_HIGH,
	GOT_HEADER_BYTE_LOW,
	GOT_LEN,
	GOT_PACKET_ID,
	GOT_DATA_TYPES,
	GOT_DATA_LEN,
	GOT_DATA_CHANNEL_HIGH,
	GOT_DATA_CHANNEL_LOW,
	GOT_CRC,
	GOT_CRC16_BYTE_HIGH,
	GOT_CRC16_BYTE_LOW,
};

class DecodeVariable{
public:

	JETI::DECODE_STATE current_state;
	uint8_t current_byte;
	uint8_t data_package[40];
	int data_channel_count;

	DecodeVariable();

	void decodePackage();

	void updateCurrentByte(uint8_t new_byte);

	void updateCurrentState(JETI::DECODE_STATE new_state);

	bool checkValidHeaderHigh();

	bool checkValidHeaderLow();

	bool checkValidLength();

	bool checkValidDataLength();

	void returnToUnsyncState();

	bool checkLastDataChannel();

	void updateDataChannel();

	void updateDataPackage(int entry_position, uint8_t entry_val);

	bool outOfBounds();

	bool dataLenZero();
};

}
