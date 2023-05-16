/*
 * SerialPacketProtocol.hpp
 *
 *  Created on: Mar 19, 2021
 *      Author: Flo Keck
 *
 *  Serial Packet Protocol (SPP)
 *  ============================
 *
 *  A simple protocol to transmit and receive packages over a UART connection.
 *  Used in Season 2020/21 to transmit CAN packages over UART to avoid having to use the Vector CAN Interfaces.
 *
 *  A UART packet follows this format
 *  	Three bytes start of packet (0xAA 0xAA 0xAA)
 *  	One byte packet length (Length of data, can be between 1 and 255 bytes)
 *  	Data bytes
 *  	One byte checksum (simple XOR over all data bytes)
 *  	One byte stop of packet (0x55)
 *
 *  Details
 *  	* Byte stuffing:
 *  		* To avoid a false detection of the start of packet, byte stuffing is used.
 *  		* The sender inserts a stuff byte (0x55) after it transmitted two 0xAA data bytes
 *  		* The receiver discards the following byte after it received two 0xAA data bytes
 *  	* Length shift:
 *  		* If the length of data to transmit is lower or equal to 0xAA, the transmitted length is substracted by one.
 *  		* This is to avoid that the transmitted length field is 0xAA which could lead to a false detection of the packet start.
 *  		* Therefore, packet lengths of 0 bytes are not possible
 *  	* It is implemented as a header only library, otherwise using templates for the buffer length would not work.
 *  	* The other side (e.g. on your laptop) is implemented as a python skript.
 *  	* Technical explanation: https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 *  	* Where I stole the DMA receive code from: https://www.bepat.de/2020/12/02/stm32f103c8-uart-with-dma-buffer-and-idle-detection/
 */

#ifndef CANZERO_INCLUDE_SERIALPACKETPROTOCOL_HPP_
#define CANZERO_INCLUDE_SERIALPACKETPROTOCOL_HPP_

#include "stm32f4xx_hal.h"
#include "typedefinitions.h"
#include "FreeRTOS.h"
#include "usart.h"

// Different states of packet reception
enum class SppRxState {
	WAIT_FOR_START, RECEIVE_LENGTH, READING_DATA, RECEIVE_CRC, RECEIVE_STOP
};

// MAX_PACKET_LEN is the maximum data length of a packet
// RX_BUFFER_SIZE is the size of the RX buffer (needs to include packet overhead and stuff bytes)
template<uint16_t MAX_PACKET_LEN, uint16_t DMA_RX_BUFFER_SIZE>
class SerialPacketProtocol {
private:
	// Constants declarations
	static const uint8_t START_BYTE = 0xAA;
	static const uint8_t STUFF_BYTE = 0x55;
	static const uint8_t STOP_BYTE = 0x55;
	const uint16_t DMA_TX_BUFFER_SIZE = 6 + MAX_PACKET_LEN + MAX_PACKET_LEN / 10;// 6 bytes overhead per packet and up to 10% stuff bytes

	// Member variables
	UART_HandleTypeDef *m_huart;
	void (*m_packetCallback)(uint8_t*, uint16_t);

	// Transmission
	uint8_t m_dmaTxBuffer[6 + MAX_PACKET_LEN + MAX_PACKET_LEN / 10];	// DMA_TX_BUFFER_SIZE

	// Reception
	SppRxState m_rxState = SppRxState::WAIT_FOR_START;
	uint8_t m_lastByte = 0;
	uint8_t m_penultimateByte = 0;
	uint16_t m_rxPacketLength = 0;
	uint16_t m_receivedBytes = 0;
	uint8_t m_rxPacketBuffer[MAX_PACKET_LEN];
	// Reception DMA
	uint8_t m_rxRollover = 0;
	uint8_t m_rxCounter = 0;
	uint8_t m_rxBfrPos = 0;
	uint8_t m_dmaRxBuffer[DMA_RX_BUFFER_SIZE];

	uint8_t m_enabled = 0;

public:
	/**
	 * Constructor, you need to supply the handle to the UART module
	 */
	SerialPacketProtocol(UART_HandleTypeDef *huart, void (*callback)(uint8_t*, uint16_t)) :
			m_huart { huart }, m_packetCallback { callback } {
		static_assert(DMA_RX_BUFFER_SIZE >= 2 * (6 + MAX_PACKET_LEN + MAX_PACKET_LEN / 10), "RX DMABuffer needs to hold at least two packets!");
	}

	virtual ~SerialPacketProtocol() {
	}


	/**
	 * Call this to enable receiving data
	 */
	void startReceiving() {
		//MX_USART2_UART_Init();
		// Enable Idle line interrupt and start receiving DMA
		__HAL_UART_ENABLE_IT(m_huart, UART_IT_IDLE);
		m_huart->hdmarx->Instance->NDTR = DMA_RX_BUFFER_SIZE;
		HAL_UART_Receive_DMA(m_huart, m_dmaRxBuffer, DMA_RX_BUFFER_SIZE);
		m_enabled = 1;
	}

	/**
	 * This function shall be called from the receive ISR of the UART.
	 */
	void UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		if (huart == m_huart && m_enabled) {
			// UART Rx Complete Callback;
			// Rx Complete is called by: DMA (automatically), if it rolls over
			// and when an IDLE Interrupt occurs
			// DMA Interrupt allays occurs BEFORE the idle interrupt can be fired because
			// idle detection needs at least one UART clock to detect the bus is idle. So
			// in the case, that the transmission length is one full buffer length
			// and the start buffer pointer is at 0, it will be also 0 at the end of the
			// transmission. In this case the DMA rollover will increment the RxRollover
			// variable first and len will not be zero.
			if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_IDLE)) {// Check if it is an "Idle Interrupt"
				__HAL_UART_CLEAR_IDLEFLAG(m_huart);		// clear the interrupt
				m_rxCounter++;						// increment the Rx Counter

				uint16_t start = m_rxBfrPos;// Rx bytes start position (=last buffer position)
				m_rxBfrPos = DMA_RX_BUFFER_SIZE - (uint16_t) m_huart->hdmarx->Instance->NDTR;// determine actual buffer position
				uint16_t len = DMA_RX_BUFFER_SIZE;			// init len with max. size

				if (m_rxRollover < 2) {
					if (m_rxRollover) {						// rolled over once
						if (m_rxBfrPos <= start)
							len = m_rxBfrPos + DMA_RX_BUFFER_SIZE - start;// no bytes overwritten
						else
							len = DMA_RX_BUFFER_SIZE + 1;	// bytes overwritten error
					} else {
						len = m_rxBfrPos - start;			// no bytes overwritten
					}
				} else {
					len = DMA_RX_BUFFER_SIZE + 2;				// dual rollover error
				}

				if (len && (len <= DMA_RX_BUFFER_SIZE)) {
					dataReceived(m_dmaRxBuffer, len, start, DMA_RX_BUFFER_SIZE);
				} else {
					// buffer overflow error:
					printDebugISR("NAK RX BUFFER OVERFLOW ERROR %d\r\n", (len - DMA_RX_BUFFER_SIZE));
				}

				m_rxRollover = 0;					// reset the Rollover variable
			} else if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_ORE)) {
				// An overrun error might occur at the start when messages are already sent but the softwaer is not ready yet
				// Read the SR and DR register, this should clear the overrun error
				(void)m_huart->Instance->SR;
				(void)m_huart->Instance->DR;
			} else {
				// no idle flag? --> DMA rollover occurred
				m_rxRollover++;		// increment Rollover Counter
			}
		}
	}

	/**
	 * Call this whenever you received data
	 *     pData: uint8_t data pointer
	 *     size: number of received bytes
	 */
	void dataReceived(uint8_t *pData, uint16_t size, uint16_t start = 0, uint16_t rollover = 65535) {
		for (uint16_t i = 0; i < size; i++) {
			if (m_rxState == SppRxState::WAIT_FOR_START) {
				if (m_lastByte == START_BYTE && m_penultimateByte == START_BYTE
						&& pData[(start + i) % rollover] == START_BYTE) {
					// We found the start of a new packet
					m_rxState = SppRxState::RECEIVE_LENGTH;
				}
			} else if (m_rxState == SppRxState::RECEIVE_LENGTH) {
				m_rxPacketLength = pData[(start + i) % rollover];
				if (m_rxPacketLength < START_BYTE) {
					m_rxPacketLength++;
				}

				m_receivedBytes = 0;
				m_rxState = SppRxState::READING_DATA;
			} else if (m_rxState == SppRxState::READING_DATA) {
				if (m_lastByte == START_BYTE
						&& m_penultimateByte == START_BYTE) {
					// Stuff byte -> skip it
				} else {
					m_rxPacketBuffer[m_receivedBytes++] = pData[(start + i) % rollover];

					if (m_receivedBytes == m_rxPacketLength) {
						// Packet is fully here
						m_rxState = SppRxState::RECEIVE_CRC;
					}
				}
			} else if (m_rxState == SppRxState::RECEIVE_CRC) {
				uint8_t crcCalculated = 0;
				for (int j = 0; j < m_rxPacketLength; j++) {
					crcCalculated = crcCalculated ^ m_rxPacketBuffer[j];
				}
				if (crcCalculated != pData[(start + i) % rollover]) { // CRC wrong
					printDebugISR("CRC wrong!\n")
					m_rxState = SppRxState::WAIT_FOR_START;
				} else {	// CRC correct
					m_rxState = SppRxState::RECEIVE_STOP;
				}
			} else if (m_rxState == SppRxState::RECEIVE_STOP) {
				if (pData[(start + i) % rollover] != STOP_BYTE) {
					printDebugISR("Stop Byte wrong!\n");
				} else {
					// Got a full packet. Packet handler will be called at the end of this function
					m_packetCallback(m_rxPacketBuffer, m_rxPacketLength);
				}
				m_rxState = SppRxState::WAIT_FOR_START;
			}
			m_penultimateByte = m_lastByte;
			m_lastByte = pData[(start + i) % rollover];
		}
	}

	/**
	 * Send a packet
	 *     pData: uint8_t buffer of data to send
	 *     size: number of bytes to send
	 */
	HAL_StatusTypeDef send(uint8_t *pData, uint16_t size) {
		// Wait until last transmission is finished
		while (m_huart->gState != HAL_UART_STATE_READY) {
		}

		// Check that size is valid
		if (size <= 0 || size > 255 || size > MAX_PACKET_LEN) {
			return HAL_ERROR;
		}

		// Variables needed
		uint8_t checksum = 0;
		uint16_t bufferPos = 0;

		// Start of packet
		m_dmaTxBuffer[bufferPos++] = START_BYTE;
		m_dmaTxBuffer[bufferPos++] = START_BYTE;
		m_dmaTxBuffer[bufferPos++] = START_BYTE;

		// Length. Shift by one if smaller or equal to START_BYTE to avoid a false detection of start of packet
		if (size <= START_BYTE) {
			m_dmaTxBuffer[bufferPos++] = size - 1;
		} else {
			m_dmaTxBuffer[bufferPos++] = size;
		}

		// Loop over data bytes
		for (uint8_t i = 0; i < size; i++) {
			m_dmaTxBuffer[bufferPos++] = pData[i];
			checksum = checksum ^ pData[i];

			// Byte stuffing. If the last two data bytes were a START_BYTE, insert a STUFF_BYTE
			if (i >= 1) {
				if (m_dmaTxBuffer[bufferPos - 1] == START_BYTE
						&& m_dmaTxBuffer[bufferPos - 2] == START_BYTE) {
					m_dmaTxBuffer[bufferPos++] = STUFF_BYTE;
				}
			}
		}

		m_dmaTxBuffer[bufferPos++] = checksum;
		m_dmaTxBuffer[bufferPos++] = STOP_BYTE;

		return HAL_UART_Transmit_DMA(m_huart, m_dmaTxBuffer, bufferPos);
	}

	/**
	 * Send a CAN message
	 *     txMsg: CAN Message to send.
	 */
	HAL_StatusTypeDef sendCANMessage(const TxMessage &txMsg, uint8_t bus = 0) {
		uint8_t canBuffer[10];
		canBuffer[0] = txMsg.txHeader.StdId & 0xFF;
		canBuffer[1] = ((txMsg.txHeader.StdId >> 8) & 0xF)
				+ ((bus << 4) & 0xF0);

		for (uint8_t i = 0; i < txMsg.txHeader.DLC; i++) {
			canBuffer[i + 2] = txMsg.txBuf[i];
		}

		return send(canBuffer, 2 + txMsg.txHeader.DLC);
	}
};

#endif /* CANZERO_INCLUDE_SERIALPACKETPROTOCOL_HPP_ */
