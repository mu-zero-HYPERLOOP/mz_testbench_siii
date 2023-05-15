/*
 * bms44_receiver.cpp
 *
 *  Created on: May 15, 2023
 *      Author: OfficeLaptop
 */
#include "bms44_receiver.hpp"
#include "canzero.hpp"
#include <cinttypes>
#include <stm32f4xx_hal_can.h>


namespace bms44 {

struct bms_frame {
	uint8_t m_crcLow;
	uint8_t m_crcHigh;
	float m_temperature;
	float m_voltage;
	float m_current;
	float m_average_power_10sec;
	float m_remaining_capacity_wh;
	float m_full_charge_capacity_wh;
	float m_hours_to_full_charge;
	uint8_t m_reset[50];
};

bms_frame m_currentFrame;

struct bms_frame_fragment {
	explicit bms_frame_fragment(RxMessage& raw){
		uint32_t size = raw.rxHeader.DLC;
		m_size = size - 1;
		for(size_t i=0;i<m_size;i++){
			m_payload[i] = raw.rxBuf[i];
		}
		uint8_t tailByte = raw.rxBuf[m_size];
		m_sofFlag = tailByte & SOF_MASK;
		m_eofFlag = tailByte & EOF_MASK;
		m_toggleFlag = tailByte & TOGGLE_MASK;
		m_transferId = tailByte & TRANSFER_MASK;
	}

	uint32_t size() const{
		return m_size;
	}

	uint8_t operator[](uint32_t idx) const{
		return m_payload[idx];
	}

	bool sofFlag() const{
		return m_sofFlag;
	}

	bool eofFlag() const {
		return m_eofFlag;
	}

	bool toggleFlag()const {
		return m_toggleFlag;
	}

private:
	static constexpr uint8_t SOF_MASK = 0x1 << 7;
	static constexpr uint8_t EOF_MASK = 0x1 << 6;
	static constexpr uint8_t TOGGLE_MASK = 0x1 << 5;
	static constexpr uint8_t TRANSFER_MASK = 0x1F;
	size_t m_size;
	uint8_t m_payload[7];
	bool m_sofFlag;
	bool m_eofFlag;
	bool m_toggleFlag;
	uint8_t m_transferId;

};

struct bms_frame_builder {
	void append_fragment(const bms_frame_fragment& fragment){
		if(m_empty){ //expect sof.
			if(not fragment.sofFlag()){
				printf("ERROR parsing BMS Frame: unexpected SOF Flag!\n");
				m_error = true;
			}else{
				m_empty = false;
				m_error = false;
				m_toggle = false;
				m_end = 0;
			}
		}
		if(fragment.toggleFlag() != m_toggle){
			printf("ERROR parsing BMS Frame: INVALID TOGGLE FLAG\n");
			m_error = true;
		}

		//copy payload.
		for(size_t i=0;i<fragment.size();i++){
			m_buffer[m_end++] = fragment[i];
		}

		if(fragment.eofFlag()){
			m_complete = true;
		}
		m_toggle = !m_toggle;
	}
	bool isComplete(){
		return m_complete;
	}

	bool reset(){
		m_empty = true;
		m_complete = false;
		m_end = 0;
		m_error = false;
		m_toggle = false;
	}

	float parsef16(uint8_t b1, uint8_t b2){
		uint16_t data = b1 | b2 << sizeof(8);

		//represents 16 bit float
		uint16_t sign = (data & (0x1 << 15)) >> 15;
		int16_t exponent = (data & 0x7C00) >> 10;
		int16_t mantisse = (data & 0x3FF);

		int8_t u8_mantisse = exponent & 0xF;
		if(exponent & (0x1 << 4)){
			sum -= (0x1 << 4);
		}

		int16_t mantisse = exponent & 0x1FF;
		if(mantisse & (0x1 << 9)){
			mantisse -= (0x1 << 9);
		}

		//convert to 32 bit float.


	}

	bms_frame build(){
		m_currentFrame.m_crcLow = m_buffer[0];
		m_currentFrame.m_crcHigh = m_buffer[1];
		m_currentFrame.m_temperature = parsef16(m_buffer[1], m_buffer[1]);

		reset();
		return frame;
	}
private:
	uint32_t m_end = 0;
	bool m_empty = true;
	bool m_complete = false;
	bool m_error = false;
	bool m_toggle = false;
	uint8_t m_transferId;
	uint8_t m_buffer[64];

};

bms_frame_builder frame_builder;

void bmsFrameReceiver(RxMessage& raw){
	bms_frame_fragment fragment(raw);
	frame_builder.append_fragment(fragment);
	if(frame_builder.isComplete()){
		m_currentFrame = frame_builder.build();
	}
}

void init(){
	can::registerMessageReceiver<can::messages::BMS_TX_Status>(bmsFrameReceiver);
}

void update(){
	printf("temperature = %u\n", m_currentFrame.m_temperature);
}

}



