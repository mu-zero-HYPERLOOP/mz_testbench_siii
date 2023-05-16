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

	void reset(){
		m_empty = true;
		m_complete = false;
		m_end = 0;
		m_error = false;
		m_toggle = false;
	}

	float parsef16(uint16_t data){
		//represents 16 bit float
		uint16_t sign = (data & (0x1 << 15)) >> 15;
		int16_t exponent = (data & 0x7C00) >> 10;
		int16_t mantisse = (data & 0x3FF);

		uint8_t u8_exponent = exponent & 0x1F;
		u8_exponent -= (0x1 << 4) - 1;
		u8_exponent += (0x1 << 7) - 1;

		int32_t u23_mantisse = (mantisse & 0x3FF) << 13;

		uint32_t f32u32 = (sign << 31) | (u8_exponent << 23) | u23_mantisse;
		float * f32p = (float*)(&f32u32);
		float f32 = *f32p;
		return f32;
	}

	bms_frame build(){
		bms_frame frame;
		frame.m_crcLow = m_buffer[0];
		frame.m_crcHigh = m_buffer[1];
		frame.m_temperature = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[2])));
		frame.m_voltage = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[4])));
		frame.m_current = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[6])));
		frame.m_average_power_10sec = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[8])));
		frame.m_remaining_capacity_wh = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[10])));
		frame.m_full_charge_capacity_wh = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[12])));
		frame.m_hours_to_full_charge = parsef16(*reinterpret_cast<uint16_t*>(&(m_buffer[14])));

		float temperature = *reinterpret_cast<float*>(&(m_buffer[2]));
		printf("other temperature = %f\n", temperature);

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
	printf("======= FRAME ======\n");
	printf("temperature          = %f\n", m_currentFrame.m_temperature);
	printf("voltage              = %f\n", m_currentFrame.m_voltage);
	printf("current              = %f\n", m_currentFrame.m_current);
	printf("average pow          = %f\n", m_currentFrame.m_average_power_10sec);
	printf("remaining capacity   = %f\n", m_currentFrame.m_remaining_capacity_wh);
	printf("full charge cap      = %f\n", m_currentFrame.m_full_charge_capacity_wh);
	printf("hours to full charge = %f\n", m_currentFrame.m_hours_to_full_charge);


}

}



