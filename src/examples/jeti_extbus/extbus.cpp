#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>
#include "extbus.h"

bool JETI::operator==(Header const &lhs, Header const &rhs) {
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


bool JETI::operator==(CRC const &lhs, CRC const &rhs){
	if (lhs.crc0 != rhs.crc0) {
		return false;
	}
	if (lhs.crc1 != rhs.crc1) {
		return false;
	}
	return true;
}


bool JETI::IsChannels(const JETI::Header head) {
	if (head.H0 != 0x3E) {
		return false;
	}
	if (not(head.H1 == 0x03 || head.H1 == 0x01)) {
		return false;
	}

	return (head.Data_ID == 0x31) ? (true) : (false);
}


auto JETI::GetHeader(const uint8_t data[], size_t len) -> JETI::Header {
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


// Introduce a Strong Type here for CH Index
// Pretest if enough channel is avaiable or not!
auto JETI::GetChannel(const uint8_t data[], const size_t len,
		const uint8_t idx) -> float {
	raw_channel_t raw_channel = {.data = 0};
	if (idx == 0) {
		raw_channel.raw[0] = data[6];
		raw_channel.raw[1] = data[7];
	}else{
		raw_channel.raw[0] = data[6+2*idx];
		raw_channel.raw[1] = data[7+2*idx];
	}
	return static_cast<float>((raw_channel.raw[1]*0x100)^raw_channel.raw[0]) / 8'000;
	// return static_cast<float>(raw_channel.data) / 8'000;
}


auto JETI::ExtractCrcValues(const uint8_t data[], const size_t len) -> JETI::CRC {
	auto crc = JETI::CRC{};

	crc.crc0 = data[len-2];
	crc.crc1 = data[len-1];

	return crc;
}


uint16_t JETI::GetCRC(const uint8_t data[], const size_t len) {
	JETI::CRC crcExtracted = JETI::ExtractCrcValues(data, len);

	return (crcExtracted.crc1*0x100)^crcExtracted.crc0;
}


uint16_t JETI::crc16_update( uint16_t crc, uint8_t data ) {
	uint16_t ret_val;
	data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
	data ^= data << 4;
	ret_val = ((uint16_t)(data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
	return ret_val;
}


uint16_t JETI::Get_crc16z(const uint8_t *p, uint16_t len) {
	uint16_t crc16_data=0;
	while(len-- > 2) {crc16_data=crc16_update(crc16_data,p[0]); p++;}
	return(crc16_data);
}


bool JETI::ValidateMsg(const uint8_t *p, const uint8_t data[], const size_t len){
	if(Get_crc16z(p, len) == GetCRC(data, len)){
		return true;
	}
	return false;
}

std::vector<float> JETI::GetChannelValues(const uint8_t data[], const size_t len) {
	const JETI::Header header = GetHeader(data, len);
	int iteration = header.Channels;
	std::vector<float> ChannelValues(iteration);
	for (int i = 0; i < iteration; i++) {
		ChannelValues[i] = GetChannel(data,len,i);
		}
        return ChannelValues;
}

bool JETI::CheckChannelOverreach(int askedChannels, const uint8_t data[], const size_t len){
	const JETI::Header header = GetHeader(data, len);
	if(askedChannels > header.Channels){
		return true;
	}
	return false;
}
