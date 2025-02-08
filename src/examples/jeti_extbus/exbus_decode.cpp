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
#include <iostream>
#include "exbus_decode.h"

JETI::DecodeVariable::DecodeVariable() : current_state(JETI::DECODE_STATE::UNSYNCED), current_byte(0), data_channel_count(0){};

void JETI::DecodeVariable::decodePackage(){
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
		    if(dataLenZero()){
			updateCurrentState(JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW);
		        updateDataChannel();
			break;
		    } else {
			updateCurrentState(JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW);
		        updateDataChannel();
			break;
		    }
		    break;

		case JETI::DECODE_STATE::GOT_DATA_CHANNEL_LOW:
		    if(checkLastDataChannel()){
			updateCurrentState(JETI::DECODE_STATE::GOT_CRC16_BYTE_LOW);
		        updateDataChannel();
			break;
		    } else {
			updateCurrentState(JETI::DECODE_STATE::GOT_DATA_CHANNEL_HIGH);
		        updateDataChannel();
			break;
		    }
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

	case JETI::DECODE_STATE::GOT_CRC16_BYTE_HIGH:
	    returnToUnsyncState();
	    break;

        default:
	    current_state = JETI::DECODE_STATE::UNSYNCED;
	    break;
	}
};

void JETI::DecodeVariable::updateCurrentByte(uint8_t new_byte){
	current_byte = new_byte;
};

void JETI::DecodeVariable::updateCurrentState(JETI::DECODE_STATE new_state){
	current_state = new_state;
}

bool JETI::DecodeVariable::checkValidHeaderHigh(){
	return current_byte == JETI::consts::head_h_channel_data || current_byte == JETI::consts::head_h_request;
};

bool JETI::DecodeVariable::checkValidHeaderLow(){
	return current_byte == JETI::consts::head_l_with_scope || current_byte == JETI::consts::head_l_without_scope;
}

bool JETI::DecodeVariable::checkValidLength(){
	return current_byte > 7 && current_byte <= 40;
}

bool JETI::DecodeVariable::checkValidDataLength(){
	return current_byte == data_package[2] - 8;
}

void JETI::DecodeVariable::returnToUnsyncState(){
	current_state = JETI::DECODE_STATE::UNSYNCED;
}

bool JETI::DecodeVariable::checkLastDataChannel(){
	return data_channel_count == data_package[5];
}

void JETI::DecodeVariable::updateDataChannel(){
	if(!outOfBounds()){
		updateDataPackage((6+data_channel_count), current_byte);
		data_channel_count += 1;
	} else {
		printf("Data is out of bounds!\n\n");
	}
}

void JETI::DecodeVariable::updateDataPackage(int entry_position, uint8_t entry_val){
	data_package[entry_position] = entry_val;
}

bool JETI::DecodeVariable::outOfBounds(){
	return data_channel_count > data_package[5]+2;
}

bool JETI::DecodeVariable::dataLenZero(){
	if(data_package[5] == 0){
		return true;
	} else {
		return false;
	}
}
