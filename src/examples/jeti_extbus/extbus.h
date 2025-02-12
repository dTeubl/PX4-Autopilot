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

auto GetHeader(uint8_t data[], size_t len) -> JETI::Header;

bool CheckChannelOverreach(int askedChannels, uint8_t data[],
			   const size_t len);

auto GetChannel(uint8_t data[], const size_t len, const uint8_t idx)
    -> float;

uint16_t GetCRC(uint8_t data[], const size_t len);

auto ExtractCrcValues(uint8_t data[], const size_t len) -> JETI::CRC;

uint16_t Get_crc16z(uint8_t *p, uint16_t len);

uint16_t crc16_update(uint16_t crc, uint8_t data);

bool ValidateMsg(uint8_t *p, uint8_t data[], const size_t len);
} // namespace JETI
