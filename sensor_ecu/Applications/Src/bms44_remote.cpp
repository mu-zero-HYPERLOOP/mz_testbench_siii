/*
 * bms44_receiver.cpp
 *
 *  Created on: May 15, 2023
 *      Author: OfficeLaptop
 */
#include <bms44_remote.hpp>
#include "canzero.hpp"
#include <cinttypes>
#include <stm32f4xx_hal_can.h>

namespace bms44 {

struct bms_frame {
	volatile uint8_t m_crcLow;
	volatile uint8_t m_crcHigh;
	volatile int16_t m_manufacturId;
	volatile int16_t m_skuId;
	volatile uint16_t m_voltage;
	volatile int16_t m_current; // in 10mA.
	volatile int16_t m_temperature;
	volatile uint16_t m_remainingCapacityProzent;
	volatile uint16_t m_cycleLife;
	volatile int16_t m_health;
	volatile uint16_t m_cell1Voltage;
	volatile uint16_t m_cell2Voltage;
	volatile uint16_t m_cell3Voltage;
	volatile uint16_t m_cell4Voltage;
	volatile uint16_t m_cell5Voltage;
	volatile uint16_t m_cell6Voltage;
	volatile uint16_t m_cell7Voltage;
	volatile uint16_t m_cell8Voltage;
	volatile uint16_t m_cell9Voltage;
	volatile uint16_t m_cell10Voltage;
	volatile uint16_t m_cell11Voltage;
	volatile uint16_t m_cell12Voltage;
	volatile uint16_t m_standardCapacity;
	volatile uint16_t m_remainingCapacity_mAh;
	volatile uint16_t m_errorFlags1;
	volatile uint16_t m_errorFlags2;

};

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

struct bms_frame_fragment {
	explicit bms_frame_fragment(RxMessage &raw) {
		uint32_t size = raw.rxHeader.DLC;
		m_size = size - 1;
		for (size_t i = 0; i < m_size; i++) {
			m_payload[i] = raw.rxBuf[i];
		}
		uint8_t tailByte = raw.rxBuf[m_size];
		m_sofFlag = tailByte & SOF_MASK;
		m_eofFlag = tailByte & EOF_MASK;
		m_toggleFlag = tailByte & TOGGLE_MASK;
		m_transferId = tailByte & TRANSFER_MASK;
	}

	uint32_t size() const {
		return m_size;
	}

	uint8_t operator[](uint32_t idx) const {
		return m_payload[idx];
	}

	bool sofFlag() const {
		return m_sofFlag;
	}

	bool eofFlag() const {
		return m_eofFlag;
	}

	bool toggleFlag() const {
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
	void append_fragment(const bms_frame_fragment &fragment) {
		if (m_empty) { //expect sof.
			if (not fragment.sofFlag()) {
				printf("ERROR parsing BMS Frame: expected SOF Flag!\n");
				m_error = true;
			} else {
				m_empty = false;
				m_error = false;
				m_toggle = false;
				m_end = 0;
			}
		}
		if (fragment.toggleFlag() != m_toggle) {
			printf("ERROR parsing BMS Frame: INVALID TOGGLE FLAG\n");
			m_error = true;
		}

		//copy payload.
		for (size_t i = 0; i < fragment.size(); i++) {
			m_buffer[m_end++] = fragment[i];
		}

		if (fragment.eofFlag()) {
			m_complete = true;
		}
		m_toggle = !m_toggle;
	}
	bool isComplete() {
		return m_complete;
	}

	void reset() {
		m_empty = true;
		m_complete = false;
		m_end = 0;
		m_error = false;
		m_toggle = false;
	}

	float parsef16(uint16_t data) {
		//represents 16 bit float
		uint16_t sign = (data & (0x1 << 15)) >> 15;
		int16_t exponent = (data & 0x7C00) >> 10;
		int16_t mantisse = (data & 0x3FF);

		uint8_t u8_exponent = exponent & 0x1F;
		u8_exponent -= (0x1 << 4) - 1;
		u8_exponent += (0x1 << 7) - 1;

		int32_t u23_mantisse = (mantisse & 0x3FF) << 13;

		uint32_t f32u32 = (sign << 31) | (u8_exponent << 23) | u23_mantisse;
		float *f32p = (float*) (&f32u32);
		float f32 = *f32p;
		return f32;
	}

	bms_frame build() {
		//log binary dump.
		bms_frame frame = *reinterpret_cast<bms_frame*>(m_buffer);

		/*
		printf("NEW  BMS44-FRAME\n");
		printf("manufacturer Id = %u\n", frame.m_manufacturId);
		printf("temperature = %u\n", frame.m_temperature);
		printf("remainingCap = %u\n", frame.m_remainingCapacityProzent);
		printf("remainingCap = %u\n", frame.m_remainingCapacity_mAh);
		printf("health status = %u\n", frame.m_health);
		printf("cycle life = %u\n", frame.m_cycleLife);
		printf("current = %d\n", frame.m_current);
		printf("cells voltage = %u\n", frame.m_voltage);
		printf("cells voltage 1 = %u\n", frame.m_cell1Voltage);
		printf("cells voltage 2 = %u\n", frame.m_cell2Voltage);
		printf("cells voltage 3 = %u\n", frame.m_cell3Voltage);
		printf("cells voltage 4 = %u\n", frame.m_cell4Voltage);
		printf("cells voltage 5 = %u\n", frame.m_cell5Voltage);
		printf("cells voltage 6 = %u\n", frame.m_cell6Voltage);
		printf("cells voltage 7 = %u\n", frame.m_cell7Voltage);
		printf("cells voltage 8 = %u\n", frame.m_cell8Voltage);
		printf("cells voltage 9 = %u\n", frame.m_cell9Voltage);
		printf("cells voltage 10 = %u\n", frame.m_cell10Voltage);
		printf("cells voltage 11 = %u\n", frame.m_cell11Voltage);
		printf("cells voltage 12 = %u\n", frame.m_cell12Voltage);
		printf("error bits 1 = %u\n", frame.m_errorFlags1);
		printf("error bits 2 = %u\n", frame.m_errorFlags2);
		*/

		uint16_t total_cell_voltage = 0;
		volatile uint16_t *cellVoltagePtr = &frame.m_cell1Voltage;
		for (size_t i = 0; i < 12; i++) {
			total_cell_voltage += *cellVoltagePtr;
			cellVoltagePtr++;
		}
		printf("total cell voltage = %u\n", total_cell_voltage);

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

bms_frame_builder frame_builder1;
bms_frame_builder frame_builder2;

bms_frame m_bms44_1_state;
bms_frame m_bms44_2_state;

void bms1FrameReceiver(RxMessage &raw) {
	bms_frame_fragment fragment(raw);
	frame_builder1.append_fragment(fragment);
	if (frame_builder1.isComplete()) {
		m_bms44_1_state = frame_builder1.build();
	}
}

void bms2FrameReceiver(RxMessage &raw) {
	bms_frame_fragment fragment(raw);
	frame_builder2.append_fragment(fragment);
	if (frame_builder2.isComplete()) {
		m_bms44_2_state = frame_builder2.build();
	}
}

void init() {
	can::registerMessageReceiver<can::messages::BMS44_1_Frame>(
			bms1FrameReceiver);
	can::registerMessageReceiver<can::messages::BMS44_2_Frame>(
			bms1FrameReceiver);
}

void update() {

	/*
	 printf("======= FRAME ======\n");
	 printf("temperature          = %f\n", m_currentFrame.m_temperature);
	 printf("voltage              = %f\n", m_currentFrame.m_voltage);
	 printf("current              = %f\n", m_currentFrame.m_current);
	 printf("average pow          = %f\n", m_currentFrame.m_average_power_10sec);
	 printf("remaining capacity   = %f\n", m_currentFrame.m_remaining_capacity_wh);
	 printf("full charge cap      = %f\n", m_currentFrame.m_full_charge_capacity_wh);
	 printf("hours to full charge = %f\n", m_currentFrame.m_hours_to_full_charge);
	 */

}

}

