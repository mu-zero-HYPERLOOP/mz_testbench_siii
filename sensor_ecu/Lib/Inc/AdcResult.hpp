/*
 * MultiChannelAdcResult.hpp
 *
 *  Created on: Apr 15, 2023
 *      Author: karl
 */

#ifndef INC_MULTICHANNELADCRESULT_HPP_
#define INC_MULTICHANNELADCRESULT_HPP_

#include <cstdint>

template<typename uintx_t = uint16_t>
class AdcResult {
public:
	explicit AdcResult(uintx_t *buffer, size_t bufferSize,
			unsigned int resolution) :
			m_buffer(buffer), m_bufferSize(bufferSize), m_resolution(resolution) {
	}

	const uintx_t& operator[](size_t rank) {
		if (rank > m_bufferSize)
			Error_Handler();
		return m_buffer[rank];
	}

	float asVoltageF(size_t rank) {
		uint32_t v = static_cast<uint32_t>((*this)[rank]);
		return v * 3.3 / 4095.0;
	}

	//returns the voltage as a u32 in mico-Volt.
	uint32_t asVoltageU32(size_t rank) {
		uint64_t v = static_cast<uint32_t>((*this)[rank]);
		v *= 3300000;
		v /= 4095; //rounding is much slower compared to flooring, thats why we floor.
		return (uint32_t)v;
	}

private:
	uintx_t *m_buffer;
	size_t m_bufferSize;
	unsigned int m_resolution;
};

#endif /* INC_MULTICHANNELADCRESULT_HPP_ */
