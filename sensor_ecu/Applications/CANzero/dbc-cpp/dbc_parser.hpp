/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY DBC2CPP V1.7.7.
 * 
 * This header file was generated from 'pod2023_gen.dbc' on 20:21:27 05.07.2023.
 * It contains all messages and signals as well as value tables and attributes of the DBC file.
 * Only messages and signals received or sent from node 'SensorF' were parsed.
 * The STM32 template was used to generate code for STM32 microcontrollers.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef DBCPARSER_DBC_PARSER_HPP
#define DBCPARSER_DBC_PARSER_HPP

#pragma once

#include <stdint.h>
#include <cmath>
#include "stm32f4xx_hal.h"
#include "typedefinitions.h"
#include "log.h"
extern osMessageQueueId_t czSendQueue;
extern osMessageQueueId_t czReceiveQueue;


// This is needed for the Windows State Machine Simulation Framework (SMSF)
// std::round() is not a constexpr function and the windows compiler throws an error
// Further investigation needed, why ARM-GCC has no problem with it?!
#ifndef _WIN32
#define STD_ROUND std::round
#else
constexpr float custom_round(float x) {
    if(x > 0.0) {
        return x + 0.5;
    } else {
        return x - 0.5;
    }
}
#define STD_ROUND custom_round
#endif


namespace can {

    /**************************************************************************
    * Calculated mask and id for receive fiter.                               *
    ***************************************************************************/
    namespace filters {
        constexpr uint8_t num_ext = 2;      // Number of used receive filters for extended (29-bit) ID messages
        constexpr uint32_t mask_ext[2] = {   // Filter mask for extended (29-bit) ID messages
            0x1FFFFFF9,            0x1EFF8109 
        };
        constexpr uint32_t id_ext[2] = {     // Filter ID for extended (29-bit) ID messages
            0x1FFFFFF8,            0x108000 
        };

        constexpr uint8_t num_std = 24;      // Number of used receive filters for standard (11-bit) ID messages
        constexpr uint32_t mask_std[24] = {   // Filter mask for standard (11-bit) ID messages
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FE,            0x7FD,            0x6FF, 
            0x7FB,            0x77F,            0x3FF,            0x6FF 
        };
        constexpr uint32_t id_std[24] = {     // Filter ID for standard (11-bit) ID messages
            0x430,            0x234,            0x250,            0x411, 
            0x20B,            0x002,            0x341,            0x781, 
            0x191,            0x711,            0x19A,            0x21A, 
            0x51A,            0x55A,            0x71A,            0x722, 
            0x714,            0x004,            0x001,            0x200, 
            0x100,            0x241,            0x1C1,            0x601 
        };
    }

    /**************************************************************************
    * Buses used to send a CAN message on a specific bus.                     *
    * We use the upper 16-bits of the RTR field of CAN_TxHeaderTypeDef to     *
    * transmit the information on which bus a message should be send.         *
    ***************************************************************************/
    typedef uint32_t CAN_BusTypeDef;
    namespace buses {
        constexpr uint32_t mask = 0xFFFF0000;
        constexpr uint32_t shift = 16;

        // Let the CANzero interface decide on which bus this message will be transmitted
        constexpr CAN_BusTypeDef ANY = (0 << shift);

        // Send message on CAN1
        constexpr CAN_BusTypeDef BUS1 = (1 << shift);

        // Send message on CAN2
        constexpr CAN_BusTypeDef BUS2 = (2 << shift);
    }

    /**************************************************************************
    * Function to check which message was received.                           *
    * It supports standard (11-bit) and extended (29-bit) IDs.                *
    ***************************************************************************/
    template <class MESSAGE>
    inline bool checkRxMessage(const RxMessage& message) {
        if (message.rxHeader.IDE == CAN_ID_STD) {
            return (MESSAGE::isExtendedId == false) && (message.rxHeader.StdId == MESSAGE::id);
        } else {
            return (MESSAGE::isExtendedId == true) && (message.rxHeader.ExtId == MESSAGE::id);
        }
    }

        
    /**********************************************************************************************
    * MessageInfo struct, Message class and help functions.                                       *
    ***********************************************************************************************/
    // Function to check if a signal belongs to a message.
    constexpr bool checkIfSignalBelongsToMessage(uint32_t msgId, const uint32_t sigIds[], uint8_t numIds) {
        if (numIds == 0) {
            return true;
        }
        for (int i = 0; i < numIds; i++) {
            if(sigIds[i] == msgId) {
                return true;
            }
        }
        return false;
    }

    // MessageInfo struct holds information about a message.
    /*
    class MessageInfo {
        public:
        const uint32_t id;
        const uint8_t dlc;

        ~MessageInfo() = default;
        constexpr MessageInfo() : id {0}, dlc {0} {}
        constexpr MessageInfo(uint32_t _id, uint8_t _dlc) : id {_id}, dlc{_dlc} {}
    };
    */

    // Using a base class that holds all common function reduces flash usage
    // Otherwise, each function would be in the flash once per used message type, because class templates are used
    class MessageBase {
    public:
        const uint32_t id;
        uint8_t dlc;
        const bool isExtendedId;
        uint64_t intel;
        uint64_t motorola;

        MessageBase(uint32_t _id, uint8_t _dlc, bool _isExt) : id{_id}, dlc{_dlc}, isExtendedId{_isExt}, intel{0}, motorola{0} {}

        MessageBase(uint32_t _id, uint8_t _dlc, bool _isExt, const uint8_t rxBuf[8]) noexcept : id{_id}, dlc{_dlc}, isExtendedId{_isExt} {
            // Using bitshift instead of memcpy because in this way the code does not depend on the endianess of the used microcontroller
            intel = static_cast<uint32_t>(rxBuf[0]) + (static_cast<uint32_t>(rxBuf[1]) << 8) + (static_cast<uint32_t>(rxBuf[2]) << 16) + (static_cast<uint32_t>(rxBuf[3]) << 24);
            intel += (static_cast<uint64_t>(rxBuf[4]) << 32) + (static_cast<uint64_t>(rxBuf[5]) << 40);
            intel += (static_cast<uint64_t>(rxBuf[6]) << 48) + (static_cast<uint64_t>(rxBuf[7]) << 56);
            motorola = static_cast<uint32_t>(rxBuf[7]) + (static_cast<uint32_t>(rxBuf[6]) << 8) + (static_cast<uint32_t>(rxBuf[5]) << 16) + (static_cast<uint32_t>(rxBuf[4]) << 24);
            motorola += (static_cast<uint64_t>(rxBuf[3]) << 32) + (static_cast<uint64_t>(rxBuf[2]) << 40);
            motorola += (static_cast<uint64_t>(rxBuf[1]) << 48) + (static_cast<uint64_t>(rxBuf[0]) << 56);
        };
        ~MessageBase() = default;

        // Reset internal buffers
        void clear() noexcept {
            intel = 0;
            motorola = 0;
        }

        // Convert message to data buffer
        void toBuf(uint8_t txBuf[8]) const noexcept {
            txBuf[0] = (intel & 0xFF) | ((motorola >> 56) & 0xFF);
            txBuf[1] = ((intel >> 8) & 0xFF) | ((motorola >> 48) & 0xFF);
            txBuf[2] = ((intel >> 16) & 0xFF) | ((motorola >> 40) & 0xFF);
            txBuf[3] = ((intel >> 24) & 0xFF) | ((motorola >> 32) & 0xFF);
            txBuf[4] = ((intel >> 32) & 0xFF) | ((motorola >> 24) & 0xFF);
            txBuf[5] = ((intel >> 40) & 0xFF) | ((motorola >> 16) & 0xFF);
            txBuf[6] = ((intel >> 48) & 0xFF) | ((motorola >> 8) & 0xFF);
            txBuf[7] = ((intel >> 56) & 0xFF) | (motorola & 0xFF);
        }


        // Convert message to STM32 CAN_TxHeaderTypeDef and txBuf
        void toBuf(CAN_TxHeaderTypeDef& txHeader, uint8_t txBuf[8], CAN_BusTypeDef bus = buses::ANY) const noexcept {
            if(isExtendedId) {
                txHeader.StdId = 0;
                txHeader.ExtId = id;
                txHeader.IDE = CAN_ID_EXT;
            } else {
                txHeader.StdId = id;
                txHeader.ExtId = 0;
                txHeader.IDE = CAN_ID_STD;
            }
            txHeader.RTR = bus + CAN_RTR_DATA; // Upper 16-bit contain bus. Lower 16-bit contain RTR
            txHeader.DLC = dlc;
            txHeader.TransmitGlobalTime = DISABLE;
            txBuf[0] = (intel & 0xFF) | ((motorola >> 56) & 0xFF);
            txBuf[1] = ((intel >> 8) & 0xFF) | ((motorola >> 48) & 0xFF);
            txBuf[2] = ((intel >> 16) & 0xFF) | ((motorola >> 40) & 0xFF);
            txBuf[3] = ((intel >> 24) & 0xFF) | ((motorola >> 32) & 0xFF);
            txBuf[4] = ((intel >> 32) & 0xFF) | ((motorola >> 24) & 0xFF);
            txBuf[5] = ((intel >> 40) & 0xFF) | ((motorola >> 16) & 0xFF);
            txBuf[6] = ((intel >> 48) & 0xFF) | ((motorola >> 8) & 0xFF);
            txBuf[7] = ((intel >> 56) & 0xFF) | (motorola & 0xFF);
        }

        // Convert message to CANzero TxMessage by reference
        void toBuf(TxMessage& txMsg, CAN_BusTypeDef bus = buses::ANY) const noexcept {
            toBuf(txMsg.txHeader, txMsg.txBuf, bus);
        }

        // Convert message to CANzero TxMessage and return created struct
        TxMessage getTxMessage(CAN_BusTypeDef bus = buses::ANY) const noexcept {
            TxMessage txMsg;
            toBuf(txMsg.txHeader, txMsg.txBuf, bus);
            return txMsg;
        }

        // Use this function to send a message over the queue to the CAN bus
        void send(CAN_BusTypeDef bus = buses::ANY) const noexcept {
            TxMessage sendTxMessage = getTxMessage(bus);
            if(osMessageQueuePut(czSendQueue, &sendTxMessage, 0, 0) != osOK) {
                printDebug("Failed sending message %lu because queue is full!\n", sendTxMessage.txHeader.StdId);
            }
        }

        // Send message and loop it back to receive queue to receive it from another thread in the same ECU
        void sendAndLoopback(CAN_BusTypeDef bus = buses::ANY) const noexcept {
            // Normal send over CAN
            TxMessage sendTxMessage = getTxMessage(bus);
            if(osMessageQueuePut(czSendQueue, &sendTxMessage, 0, 0) != osOK) {
                printDebug("Failed sending message %lu because queue is full!\n", sendTxMessage.txHeader.StdId);
            }

            // Loop message back to receive queue
            RxMessage sendRxMessage;
            sendRxMessage.rxHeader.StdId = sendTxMessage.txHeader.StdId;
            sendRxMessage.rxHeader.ExtId = sendTxMessage.txHeader.ExtId;
            sendRxMessage.rxHeader.IDE = sendTxMessage.txHeader.IDE;
            sendRxMessage.rxHeader.RTR = sendTxMessage.txHeader.RTR;
            sendRxMessage.rxHeader.DLC = sendTxMessage.txHeader.DLC;
            sendRxMessage.rxHeader.Timestamp = 0;
            sendRxMessage.rxHeader.FilterMatchIndex = 0;
            sendRxMessage.rxBuf[0] = sendTxMessage.txBuf[0];
            sendRxMessage.rxBuf[1] = sendTxMessage.txBuf[1];
            sendRxMessage.rxBuf[2] = sendTxMessage.txBuf[2];
            sendRxMessage.rxBuf[3] = sendTxMessage.txBuf[3];
            sendRxMessage.rxBuf[4] = sendTxMessage.txBuf[4];
            sendRxMessage.rxBuf[5] = sendTxMessage.txBuf[5];
            sendRxMessage.rxBuf[6] = sendTxMessage.txBuf[6];
            sendRxMessage.rxBuf[7] = sendTxMessage.txBuf[7];
            osMessageQueuePut(czReceiveQueue, &sendRxMessage, 0, 0);
        }
    };

    // Message class, container for getting and setting signals.
    template <class MESSAGE_T>
    class Message: public MessageBase {
    public:

        ~Message() noexcept = default;

        // Constructor for sending a message
        constexpr Message() : MessageBase{MESSAGE_T::id, MESSAGE_T::dlc, MESSAGE_T::isExtendedId} {}

        // Constructor for receiving a message with a buffer
        constexpr Message(const uint8_t rxBuf[8]) : MessageBase{MESSAGE_T::id, MESSAGE_T::dlc, MESSAGE_T::isExtendedId, rxBuf} {}

        // Constructor for receiving a message with STM CAN_RxHeaderTypeDef and buffer
        constexpr Message(const CAN_RxHeaderTypeDef& rxHeader, const uint8_t rxBuf[8]) noexcept : MessageBase{MESSAGE_T::id, MESSAGE_T::dlc, MESSAGE_T::isExtendedId, rxBuf} {
            // Check if ID matches
            if(MESSAGE_T::isExtendedId) {
                if (rxHeader.ExtId != MESSAGE_T::id) {
                    while(1);
                }
            } else {
                if (rxHeader.StdId != MESSAGE_T::id) {
                    while(1);
                }
            }
        };

        // Constructor for receiving a message with CANzero RxMessage
        constexpr Message(const RxMessage& rxMsg) noexcept : Message{rxMsg.rxHeader, rxMsg.rxBuf} {}

        // Templated function to set a signal to a message
        template <class T>
        void set(typename T::dataType value) {
            static_assert(checkIfSignalBelongsToMessage(MESSAGE_T::id, T::ids, T::numIds), "Cannot set signal because it is not part of the message!");
            T::set(intel, motorola, dlc, value);
        }

        // Templated function to get a signal from a message
        template <class T>
        typename T::dataType get() {
            static_assert(checkIfSignalBelongsToMessage(MESSAGE_T::id, T::ids, T::numIds), "Cannot get signal because it is not part of the message!");
            return T::get(intel, motorola);
        }
    };

    /**********************************************************************************************
    * Signed converters for converting signed signals.                                            *
    ***********************************************************************************************/
    struct SignedConverter8Bits {
        int8_t value : 8;
    };
    struct SignedConverter16Bits {
        int16_t value : 16;
    };
    struct SignedConverter18Bits {
        int32_t value : 18;
    };
    struct SignedConverter32Bits {
        int32_t value : 32;
    };

    /**********************************************************************************************
    * Network nodes with attributes                                                               *
    ***********************************************************************************************/
    namespace nodes {
        namespace Bat1 {
            constexpr char comment[] = "";
        }
        namespace OpticalSensor {
            constexpr char comment[] = "Kistler SFP-II optical Sensor.";
        }
        namespace EMUS_BMS {
            constexpr char comment[] = "HV BMS ";
        }
        namespace SCIMO_PE {
            constexpr char comment[] = "";
        }
        namespace TelemetryNode {
            constexpr char comment[] = "Gateway between Pod and Telemetry Node-ID 0x22Gateway between Pod and Telemetry Node-ID 0x22";

            // Attributes of node 'TelemetryNode'
            constexpr uint8_t CANzero_NodeID = 34;
        }
        namespace Master {
            constexpr char comment[] = "CANzero NMT Master Node.";
        }
        namespace SensorF {
            constexpr char comment[] = "SensorECU Node-ID 0x1";

            // Attributes of node 'SensorF'
            constexpr uint8_t CANzero_NodeID = 1;
        }
        namespace SensorR {
            constexpr char comment[] = "SensorECUR Node-ID 0x2";

            // Attributes of node 'SensorR'
            constexpr uint8_t CANzero_NodeID = 2;
        }
        namespace BrakeF {
            constexpr char comment[] = "BrakeECU Front Node-ID 0x11";

            // Attributes of node 'BrakeF'
            constexpr uint8_t CANzero_NodeID = 17;
        }
        namespace BrakeR {
            constexpr char comment[] = "BrakeECUR Node-ID 0x12";

            // Attributes of node 'BrakeR'
            constexpr uint8_t CANzero_NodeID = 18;
        }
        namespace PDU {
            constexpr char comment[] = "PowerD Node-ID 0x1A";

            // Attributes of node 'PDU'
            constexpr uint8_t CANzero_NodeID = 26;
        }
        namespace HVCU {
            constexpr char comment[] = "HVController Node-ID 0xA";

            // Attributes of node 'HVCU'
            constexpr uint8_t CANzero_NodeID = 10;
        }
        namespace HVTU {
            constexpr char comment[] = "High Voltage CAN Translation Unit between BMS and Pod Node-ID 0x19";

            // Attributes of node 'HVTU'
            constexpr uint8_t CANzero_NodeID = 25;
        }
        namespace TestBench {
            constexpr char comment[] = "";
        }
        namespace MDB1 {
            constexpr char comment[] = "ModularLevitationUnit-1 Node-ID 0x21";

            // Attributes of node 'MDB1'
            constexpr uint8_t CANzero_NodeID = 33;
        }
        namespace MDB2 {
            constexpr char comment[] = "ModularLevitationUnit-2 Node-ID 0x23";

            // Attributes of node 'MDB2'
            constexpr uint8_t CANzero_NodeID = 35;
        }
        namespace MDB3 {
            constexpr char comment[] = "ModularLevitationUnit-3 Node-ID 0x27";

            // Attributes of node 'MDB3'
            constexpr uint8_t CANzero_NodeID = 39;
        }
        namespace MDB4 {
            constexpr char comment[] = "ModularLevitationUnit-4 Node-ID 0x28";

            // Attributes of node 'MDB4'
            constexpr uint8_t CANzero_NodeID = 40;
        }
        namespace MDB5 {
            constexpr char comment[] = "ModularLevitationUnit-5 Node-ID 0x29";

            // Attributes of node 'MDB5'
            constexpr uint8_t CANzero_NodeID = 41;
        }
        namespace MDB6 {
            constexpr char comment[] = "ModularLevitationUnit-6 Node-ID 0x31";

            // Attributes of node 'MDB6'
            constexpr uint8_t CANzero_NodeID = 49;
        }
        namespace Track {
            constexpr char comment[] = "TrackECU Node-ID 0x32";

            // Attributes of node 'Track'
            constexpr uint8_t CANzero_NodeID = 50;
        }
        namespace CLU {
            constexpr char comment[] = "central levitation unit Node-ID 0x14";

            // Attributes of node 'CLU'
            constexpr uint8_t CANzero_NodeID = 20;
        }
    }
    
    /**********************************************************************************************
    * Attribute definitions (only ENUM attributes need extra definition)                          *
    ***********************************************************************************************/
    enum class GenSigSendType_t {
        CYCLIC,
        ONWRITE,
        ONWRITEWITHREPETITION,
        ONCHANGE,
        ONCHANGEWITHREPETITION,
        IFACTIVE,
        IFACTIVEWITHREPETITION,
        NOSIGSENDTYPE
    };
    enum class GenMsgSendType_t {
        CYCLIC,
        NOT_USED,
        NOT_USED_01,
        NOT_USED_02,
        NOT_USED_03,
        CYCLIC_01,
        NOT_USED_04,
        IFACTIVE,
        NOMSGSENDTYPE
    };
    enum class CANzero_SDO_AccessIfOperational_t {
        NO,
        YES
    };
    enum class CANzero_SDO_AccessType_t {
        READ_WRITE,
        READ_ONLY,
        WRITE_ONLY
    };


    /**********************************************************************************************
    * Network attributes                                                                          *
    ***********************************************************************************************/
    constexpr char CANzero_NMTMasterName[] = "Master";
    constexpr char CANzero_SDOClientName[] = "TelemetryNode";
    constexpr uint32_t CANzero_DBCVersion = 256;
    constexpr char CANzero_ProtocolVersion[] = "V1.0";
    constexpr char BusType[] = "CAN";
    constexpr char DBName[] = "pod2022";
    
    /**********************************************************************************************
    * Namespace containing all signals with their value tables and attributes                     *
    ***********************************************************************************************/
    namespace signals {
        class SensorF_IgnoreLevel {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x430 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class SensorF_EmergencyLevel {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x430 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xFF00ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00ull) >> 8);
                return value;
            }
        };
        class PDU_TX_ErrorPrio {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x5 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class CLU_TX_ErrorPrio {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class BrakeF_TX_ErrorPrio {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x3 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class CLU_RX_ActionRequest {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x234 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class BMS44_2_FramePayload {
            public:
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1109216 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFFFFFFFFFFFFFull;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFFFFFFull));
                return value;
            }
        };
        class CLU_RX_ErrorStatus {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x700 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }
        };
        class CLU_LevitationState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x250 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'CLU_LevitationState'
            constexpr static uint8_t OFF = 0;
            constexpr static uint8_t IDLE = 1;
            constexpr static uint8_t PRECHARGE = 2;
            constexpr static uint8_t READY = 3;
            constexpr static uint8_t LEVI_START = 4;
            constexpr static uint8_t LEVI = 5;
            constexpr static uint8_t ERROR = 6;
            constexpr static uint8_t INCONSISTANT = 7;
        };
        class CLU_RequiresCooling {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x300 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }
        };
        class MDB1_Temperature {
            public:
            using dataType = double;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x104 };
            constexpr static double min = static_cast<double>(-214748.3648);
            constexpr static double max = static_cast<double>(214748.3647);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, double value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int32_t rawValue = static_cast<int32_t>(STD_ROUND((value) / (0.0001)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFFFFFull;
            }
            constexpr static inline double get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int32_t value = static_cast<int32_t>((intel & 0xFFFFFFFFull));
                // Convert raw bits to signed value
                SignedConverter32Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.0001);
            }
        };
        class BrakeF_TX_Pressure_Reservoir {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x411 };
            constexpr static float min = static_cast<float>(-2);
            constexpr static float max = static_cast<float>(18.475);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-2.0f)) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.005f) + (-2.0f);
            }

            // Attributes of signal 'BrakeF_TX_Pressure_Reservoir'
            constexpr static float GenSigStartValue = 400.0f;
        };
        class PDU_RX_LPCh4_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x20B };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }
        };
        class PDU_RX_LPCh5_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x20B };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }
        };
        class PDU_RX_LPCh6_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x20B };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }
        };
        class PDU_RX_LPCh7_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x20B };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }
        };
        class BMS44_1_FramePayload {
            public:
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x10ECE0 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFFFFFFFFFFFFFull;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFFFFFFull));
                return value;
            }
        };
        class Track_TX_Response {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class Track_RX_Command {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x0 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class TEST_GROUND_STATION_COMMAND {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x200 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }
        };
        class MDB1_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x100 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'MDB1_State'
            constexpr static uint8_t INIT = 0;
            constexpr static uint8_t IDLE = 1;
            constexpr static uint8_t PRECHARGE = 2;
            constexpr static uint8_t READY = 3;
            constexpr static uint8_t LEVI_START = 4;
            constexpr static uint8_t LEVI_RUN = 5;
            constexpr static uint8_t LEVI_END = 6;
            constexpr static uint8_t LEVI_UNSTABLE = 7;
            constexpr static uint8_t LEVI_AIR_GAP_CHANGE = 8;
            constexpr static uint8_t ERROR = 10;
            constexpr static uint8_t ERROR_OVERCURRENT = 11;
            constexpr static uint8_t ERROR_OVERVOLT = 12;
            constexpr static uint8_t ERROR_OVERTEMP = 13;
            constexpr static uint8_t ERROR_AIRGAP = 14;
            constexpr static uint8_t ERROR_AIRGAP_SEN = 15;
            constexpr static uint8_t ERROR_CAN = 16;
            constexpr static uint8_t ERROR_CURRE_CALIB = 17;
            constexpr static uint8_t ERROR_CHIPTEMP = 18;
            constexpr static uint8_t ERROR_CALCTIME = 19;
        };
        class OpticalSensor_TX_Timestamp {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFull;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFFull));
                return value;
            }
        };
        class OpticalSensor_TX_Vel {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(400);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.036f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.036f);
            }
        };
        class OpticalSensor_TX_Distance {
            public:
            using dataType = double;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFA };
            constexpr static double min = static_cast<double>(0);
            constexpr static double max = static_cast<double>(4294970);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, double value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint32_t rawValue = static_cast<uint32_t>(STD_ROUND((value) / (0.001)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFFFFFFFF00000000ull;
            }
            constexpr static inline double get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF00000000ull) >> 32);
                return value * (0.001);
            }
        };
        class OpticalSensor_TX_SerialNo {
            public:
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFFFull;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFull));
                return value;
            }
        };
        class OpticalSensor_TX_SensorNo {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFF000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF000000ull) >> 24);
                return value;
            }
        };
        class OpticalSensor_TX_Temp {
            public:
            using dataType = int8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, int8_t value) noexcept {
                int8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFF00000000ull;
            }
            constexpr static inline int8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int8_t value = static_cast<int8_t>((intel & 0xFF00000000ull) >> 32);
                // Convert raw bits to signed value
                SignedConverter8Bits signedConverter{value};
                value = signedConverter.value;
                return value;
            }
        };
        class OpticalSensor_TX_LEDCurrent {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(2.55);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 40) & 0xFF0000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000000000ull) >> 40);
                return value * (0.01f);
            }
        };
        class OpticalSensor_TX_STST {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0x1000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000000000000ull) >> 48);
                return value;
            }
        };
        class OpticalSensor_TX_SelfTest {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 49) & 0x2000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000000000000ull) >> 49);
                return value;
            }
        };
        class OpticalSensor_TX_SensorOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 50) & 0x4000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000000000000ull) >> 50);
                return value;
            }
        };
        class OpticalSensor_TX_TempOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 51) & 0x8000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000000000000ull) >> 51);
                return value;
            }
        };
        class OpticalSensor_TX_OpticOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 52) & 0x10000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000000000000ull) >> 52);
                return value;
            }
        };
        class OpticalSensor_TX_CurrentLowHigh {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 53) & 0x20000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20000000000000ull) >> 53);
                return value;
            }
        };
        class OpticalSensor_TX_CurrentOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 54) & 0x40000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40000000000000ull) >> 54);
                return value;
            }
        };
        class OpticalSensor_TX_PowerOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 55) & 0x80000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x80000000000000ull) >> 55);
                return value;
            }
        };
        class OpticalSensor_TX_LED_Cal {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 56) & 0x100000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x100000000000000ull) >> 56);
                return value;
            }
        };
        class OpticalSensor_TX_LEDStatus {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 57) & 0x600000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x600000000000000ull) >> 57);
                return value;
            }

            // Value table of signal 'OpticalSensor_TX_LEDStatus'
            constexpr static uint8_t OFF = 0;
            constexpr static uint8_t ON = 1;
            constexpr static uint8_t FLASHING = 2;
        };
        class OpticalSensor_TX_V25OK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 59) & 0x800000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x800000000000000ull) >> 59);
                return value;
            }
        };
        class OpticalSensor_TX_V33OK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 60) & 0x1000000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000000000000000ull) >> 60);
                return value;
            }
        };
        class OpticalSensor_TX_VeeOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 61) & 0x2000000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000000000000000ull) >> 61);
                return value;
            }
        };
        class OpticalSensor_TX_VddOK {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1FFFFFFC };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 62) & 0x4000000000000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000000000000000ull) >> 62);
                return value;
            }
        };
        class CANzero_NMT_State {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFull;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFFull));
                return value;
            }

            // Value table of signal 'CANzero_NMT_State'
            constexpr static uint16_t START_REMOTE_NODE = 1;
            constexpr static uint16_t STOP_REMOTE_NODE = 2;
            constexpr static uint16_t ENTER_PREOPERATIONAL = 128;
            constexpr static uint16_t RESET = 129;
        };
        class CANzero_NMT_Node {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2 };
            constexpr static uint16_t min = static_cast<uint16_t>(0);
            constexpr static uint16_t max = static_cast<uint16_t>(63);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'CANzero_NMT_Node'
            constexpr static uint16_t ALL = 0;
            constexpr static uint16_t SENSORF = 1;
            constexpr static uint16_t SENSORR = 2;
            constexpr static uint16_t HVCU = 10;
            constexpr static uint16_t BRAKEF = 17;
            constexpr static uint16_t BRAKER = 18;
            constexpr static uint16_t CLU = 20;
            constexpr static uint16_t HVTU = 25;
            constexpr static uint16_t PDU = 26;
            constexpr static uint16_t MDB1 = 33;
            constexpr static uint16_t TELEMETRYNODE = 34;
            constexpr static uint16_t MDB2 = 35;
            constexpr static uint16_t MDB3 = 39;
            constexpr static uint16_t MDB4 = 40;
            constexpr static uint16_t MDB5 = 41;
            constexpr static uint16_t MDB6 = 49;
            constexpr static uint16_t TRACK = 50;
        };
        class SensorF_W0_OtherWarning {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'SensorF_W0_OtherWarning'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class SensorF_W1_StateMTransitionW {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'SensorF_W1_StateMTransitionW'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class SensorF_W2_encoderOORWarning {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'SensorF_W2_encoderOORWarning'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class SensorF_E0_OtherError {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0x1000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000ull) >> 12);
                return value;
            }

            // Value table of signal 'SensorF_E0_OtherError'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E1_CPUOverTemp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 13) & 0x2000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000ull) >> 13);
                return value;
            }

            // Value table of signal 'SensorF_E1_CPUOverTemp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E2_OverVolt {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 14) & 0x4000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000ull) >> 14);
                return value;
            }

            // Value table of signal 'SensorF_E2_OverVolt'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E3_UnderVolt {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 15) & 0x8000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000ull) >> 15);
                return value;
            }

            // Value table of signal 'SensorF_E3_UnderVolt'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E4_InvalidPosition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x10000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_E4_InvalidPosition'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E5_ReservoirOverTemp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 17) & 0x20000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20000ull) >> 17);
                return value;
            }

            // Value table of signal 'SensorF_E5_ReservoirOverTemp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E6_CLUHeartbeatMiss {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 18) & 0x40000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40000ull) >> 18);
                return value;
            }

            // Value table of signal 'SensorF_E6_CLUHeartbeatMiss'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E7_BECUHeartbeatMiss {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 19) & 0x80000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x80000ull) >> 19);
                return value;
            }

            // Value table of signal 'SensorF_E7_BECUHeartbeatMiss'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E8_PDUHeartbeatMiss {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 20) & 0x100000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x100000ull) >> 20);
                return value;
            }

            // Value table of signal 'SensorF_E8_PDUHeartbeatMiss'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E9_TelemetryHeartbeatMiss {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 21) & 0x200000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x200000ull) >> 21);
                return value;
            }

            // Value table of signal 'SensorF_E9_TelemetryHeartbeatMiss'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;

            // Attributes of signal 'SensorF_E9_TelemetryHeartbeatMiss'
            constexpr static char SystemSignalLongSymbol[] = "SensorF_E9_TelemetryHeartbeatMiss";
        };
        class SensorF_E10_TitanOverTemp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 22) & 0x400000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x400000ull) >> 22);
                return value;
            }

            // Value table of signal 'SensorF_E10_TitanOverTemp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E11_HyperionOverTemp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 23) & 0x800000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x800000ull) >> 23);
                return value;
            }

            // Value table of signal 'SensorF_E11_HyperionOverTemp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E12_TitanLowHp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0x1000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000000ull) >> 24);
                return value;
            }

            // Value table of signal 'SensorF_E12_TitanLowHp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E13_HyperionLowHp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 25) & 0x2000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000000ull) >> 25);
                return value;
            }

            // Value table of signal 'SensorF_E13_HyperionLowHp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E14_TitanLowCap {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 26) & 0x4000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000000ull) >> 26);
                return value;
            }

            // Value table of signal 'SensorF_E14_TitanLowCap'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E15_HyperionLowCap {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 27) & 0x8000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000000ull) >> 27);
                return value;
            }

            // Value table of signal 'SensorF_E15_HyperionLowCap'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_E16_EboxOverTemp {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x81 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 28) & 0x10000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000000ull) >> 28);
                return value;
            }

            // Value table of signal 'SensorF_E16_EboxOverTemp'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class SensorF_TX_PodState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x181 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'SensorF_TX_PodState'
            constexpr static uint8_t POD_OFF = 0;
            constexpr static uint8_t POD_IDLE = 1;
            constexpr static uint8_t POD_LAUNCH_PREPARATION = 2;
            constexpr static uint8_t POD_READY_TO_LAUNCH = 3;
            constexpr static uint8_t POD_START_LEVITATION = 4;
            constexpr static uint8_t POD_STABLE_LEVITATION = 5;
            constexpr static uint8_t POD_CRUSING = 6;
            constexpr static uint8_t POD_DISENGAGE_BRAKES = 7;
            constexpr static uint8_t POD_STOP_LEVITATION = 8;
            constexpr static uint8_t POD_ROLLING = 9;
            constexpr static uint8_t POD_ENGAGE_BRAKES = 10;
            constexpr static uint8_t POD_END_OF_RUN = 11;
            constexpr static uint8_t POD_SAFE_TO_APPROCH = 12;
            constexpr static uint8_t POD_PUSHABLE = 13;
            constexpr static uint8_t POD_STARTUP = 14;
        };
        class SensorF_TX_BrakesTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'SensorF_TX_BrakesTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnableTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnableTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_ErrorResetTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'SensorF_TX_ErrorResetTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_HWEnableTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }

            // Value table of signal 'SensorF_TX_HWEnableTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_LaunchSetupTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0x10ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10ull) >> 4);
                return value;
            }

            // Value table of signal 'SensorF_TX_LaunchSetupTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_LaunchStartTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 5) & 0x20ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20ull) >> 5);
                return value;
            }

            // Value table of signal 'SensorF_TX_LaunchStartTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 6) & 0x40ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40ull) >> 6);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_SetHVTransition {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 7) & 0x80ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x80ull) >> 7);
                return value;
            }

            // Value table of signal 'SensorF_TX_SetHVTransition'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnableBrakeFT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0x100ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x100ull) >> 8);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnableBrakeFT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnableBrakeRT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 9) & 0x200ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x200ull) >> 9);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnableBrakeRT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnableHVCUT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 10) & 0x400ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x400ull) >> 10);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnableHVCUT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnablePDUT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 11) & 0x800ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x800ull) >> 11);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnablePDUT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_EnableSensorRT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0x1000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000ull) >> 12);
                return value;
            }

            // Value table of signal 'SensorF_TX_EnableSensorRT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopSensorRT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 13) & 0x2000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000ull) >> 13);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopSensorRT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopBrakeFT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 14) & 0x4000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000ull) >> 14);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopBrakeFT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopBrakeRT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 15) & 0x8000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000ull) >> 15);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopBrakeRT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopPDUT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x10000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopPDUT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_TX_PreopHVCUT {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x201 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0x1000000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000000ull) >> 24);
                return value;
            }

            // Value table of signal 'SensorF_TX_PreopHVCUT'
            constexpr static bool NO_TRANSITIONS = 0;
            constexpr static bool TRANSITION = 1;
        };
        class SensorF_Pos_EncoderFront {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x401 };
            constexpr static float min = static_cast<float>(-655.36);
            constexpr static float max = static_cast<float>(655.355);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int32_t rawValue = static_cast<int32_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0x3FFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int32_t value = static_cast<int32_t>((intel & 0x3FFFFull));
                // Convert raw bits to signed value
                SignedConverter18Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.005f);
            }
        };
        class SensorF_Vel_EncoderFront {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x401 };
            constexpr static float min = static_cast<float>(-65.536);
            constexpr static float max = static_cast<float>(65.534);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFFFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF000000ull) >> 24);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.002f);
            }
        };
        class SensorF_GyroFront_X {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x401 };
            constexpr static float min = static_cast<float>(-557.056);
            constexpr static float max = static_cast<float>(557.039);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.017f)));
                intel |= (static_cast<uint64_t>(rawValue) << 40) & 0xFFFF0000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF0000000000ull) >> 40);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.017f);
            }
        };
        class SensorF_TX_BatteryTemp {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x441 };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(174.75);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.05f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.05f) + (-30.0f);
            }

            // Attributes of signal 'SensorF_TX_BatteryTemp'
            constexpr static float GenSigStartValue = 600.0f;
        };
        class SensorF_LIMT_Stator_1 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x481 };
            constexpr static float min = static_cast<float>(-20);
            constexpr static float max = static_cast<float>(287.125);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-20.0f)) / (0.075f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.075f) + (-20.0f);
            }

            // Attributes of signal 'SensorF_LIMT_Stator_1'
            constexpr static float GenSigStartValue = 266.666666666667f;
        };
        class SensorF_LIMT_Stator_2 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x481 };
            constexpr static float min = static_cast<float>(-20);
            constexpr static float max = static_cast<float>(287.125);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-20.0f)) / (0.075f)));
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xFFF000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000ull) >> 12);
                return value * (0.075f) + (-20.0f);
            }

            // Attributes of signal 'SensorF_LIMT_Stator_2'
            constexpr static float GenSigStartValue = 266.666666666667f;
        };
        class SensorF_LIMT_Stator_3 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x481 };
            constexpr static float min = static_cast<float>(-20);
            constexpr static float max = static_cast<float>(287.125);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-20.0f)) / (0.075f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000ull) >> 24);
                return value * (0.075f) + (-20.0f);
            }

            // Attributes of signal 'SensorF_LIMT_Stator_3'
            constexpr static float GenSigStartValue = 266.666666666667f;
        };
        class SensorF_LIMT_Stator_4 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x481 };
            constexpr static float min = static_cast<float>(-20);
            constexpr static float max = static_cast<float>(287.125);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-20.0f)) / (0.075f)));
                intel |= (static_cast<uint64_t>(rawValue) << 36) & 0xFFF000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000ull) >> 36);
                return value * (0.075f) + (-20.0f);
            }

            // Attributes of signal 'SensorF_LIMT_Stator_4'
            constexpr static float GenSigStartValue = 266.666666666667f;
        };
        class SensorF_Cooling_Temp {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x481 };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(174.75);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.05f)));
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0xFFF000000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000000ull) >> 48);
                return value * (0.05f) + (-30.0f);
            }

            // Attributes of signal 'SensorF_Cooling_Temp'
            constexpr static float GenSigStartValue = 600.0f;
        };
        class SensorF_AccFront_X {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4C1 };
            constexpr static float min = static_cast<float>(-40.96);
            constexpr static float max = static_cast<float>(40.95875);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.00125f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFFull));
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.00125f);
            }
        };
        class SensorF_AccFront_Y {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4C1 };
            constexpr static float min = static_cast<float>(-40.96);
            constexpr static float max = static_cast<float>(40.95875);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.00125f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF0000ull) >> 16);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.00125f);
            }
        };
        class SensorF_AccFront_Z {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4C1 };
            constexpr static float min = static_cast<float>(-40.96);
            constexpr static float max = static_cast<float>(40.95875);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.00125f)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFFFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF00000000ull) >> 32);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.00125f);
            }
        };
        class SensorF_GyroFront_Z {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4C1 };
            constexpr static float min = static_cast<float>(-557.056);
            constexpr static float max = static_cast<float>(557.039);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int16_t rawValue = static_cast<int16_t>(STD_ROUND((value) / (0.017f)));
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0xFFFF000000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF000000000000ull) >> 48);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.017f);
            }
        };
        class SensorF_SDO_ID {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 3;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1, 0x601 };
            constexpr static uint16_t min = static_cast<uint16_t>(0);
            constexpr static uint16_t max = static_cast<uint16_t>(4095);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value;
            }

            // Value table of signal 'SensorF_SDO_ID'
            constexpr static uint16_t RESERVED = 0;
            constexpr static uint16_t NODEID = 1;
            constexpr static uint16_t NODESTATUS = 2;
            constexpr static uint16_t PROTOCOLVERSION = 3;
            constexpr static uint16_t STACKVERSION = 4;
            constexpr static uint16_t DBCVERSION = 5;
            constexpr static uint16_t HEARTBEATINTERVAL = 16;
            constexpr static uint16_t SENDODONBOOTUP = 32;
            constexpr static uint16_t ODENTRYSENDINTERVAL = 33;
            constexpr static uint16_t HYPERIONTEMPERATURE = 512;
            constexpr static uint16_t HYPERIONVOLTAGE = 513;
            constexpr static uint16_t HYPERIONCURRENT = 514;
            constexpr static uint16_t HYPERIONREMAININGCAPACITY = 515;
            constexpr static uint16_t HYPERIONLIFECYCLE = 516;
            constexpr static uint16_t HYPERIONHEALTHSTATUS = 517;
            constexpr static uint16_t HYPERIONCELL1VOLTAGE = 518;
            constexpr static uint16_t HYPERIONCELL2VOLTAGE = 519;
            constexpr static uint16_t HYPERIONCELL3VOLTAGE = 520;
            constexpr static uint16_t HYPERIONCELL4VOLTAGE = 521;
            constexpr static uint16_t HYPERIONCELL5VOLTAGE = 522;
            constexpr static uint16_t HYPERIONCELL6VOLTAGE = 523;
            constexpr static uint16_t HYPERIONCELL7VOLTAGE = 524;
            constexpr static uint16_t HYPERIONCELL8VOLTAGE = 525;
            constexpr static uint16_t HYPERIONCELL9VOLTAGE = 526;
            constexpr static uint16_t HYPERIONCELL10VOLTAGE = 527;
            constexpr static uint16_t HYPERIONCELL11VOLTAGE = 528;
            constexpr static uint16_t HYPERIONCELL12VOLTAGE = 529;
            constexpr static uint16_t TITANTEMPERATURE = 768;
            constexpr static uint16_t TITANVOLTAGE = 769;
            constexpr static uint16_t TITANCURRENT = 770;
            constexpr static uint16_t TITANREMAININGCAPACITY = 771;
            constexpr static uint16_t TITANLIFECYCLE = 772;
            constexpr static uint16_t TITANHEALTHSTATUS = 773;
            constexpr static uint16_t TITANCELL1VOLTAGE = 774;
            constexpr static uint16_t TITANCELL2VOLTAGE = 775;
            constexpr static uint16_t TITANCELL3VOLTAGE = 776;
            constexpr static uint16_t TITANCELL4VOLTAGE = 777;
            constexpr static uint16_t TITANCELL5VOLTAGE = 778;
            constexpr static uint16_t TITANCELL6VOLTAGE = 779;
            constexpr static uint16_t TITANCELL7VOLTAGE = 780;
            constexpr static uint16_t TITANCELL8VOLTAGE = 781;
            constexpr static uint16_t TITANCELL9VOLTAGE = 782;
            constexpr static uint16_t TITANCELL10VOLTAGE = 783;
            constexpr static uint16_t TITANCELL11VOLTAGE = 784;
            constexpr static uint16_t TITANCELL12VOLTAGE = 785;
            constexpr static uint16_t CPUUSAGE = 1040;
            constexpr static uint16_t MEMFREE = 1041;
            constexpr static uint16_t BOARDTEMP = 1042;
            constexpr static uint16_t INPUTVOLTAGE = 1043;
            constexpr static uint16_t RUNTIME = 1044;
            constexpr static uint16_t SDCIN = 1045;
            constexpr static uint16_t SDCOUT = 1046;
            constexpr static uint16_t CHIPUID1 = 1056;
            constexpr static uint16_t CHIPUID2 = 1057;
            constexpr static uint16_t BUILDDATE = 1072;
            constexpr static uint16_t BUILDTIME = 1073;
            constexpr static uint16_t CAN1_TXERRCNT = 1104;
            constexpr static uint16_t CAN1_RXERRCNT = 1105;
            constexpr static uint16_t CAN1_LASTERRORCODE = 1106;
            constexpr static uint16_t CAN1_AUTOERRORRESET = 1107;
            constexpr static uint16_t CAN1_BAUDRATE = 1108;
            constexpr static uint16_t CAN1_STATUS = 1110;
            constexpr static uint16_t CAN1_DISCARDEDTXMESSAGES = 1111;
            constexpr static uint16_t CAN1_ERRORSTATUS = 1112;
            constexpr static uint16_t CAN1_DELAYEDTXMESSAGES = 1113;
            constexpr static uint16_t CAN2_TXERRCNT = 1120;
            constexpr static uint16_t CAN2_RXERRCNT = 1121;
            constexpr static uint16_t CAN2_LASTERRORCODE = 1122;
            constexpr static uint16_t CAN2_AUTOERRORRESET = 1123;
            constexpr static uint16_t CAN2_BAUDRATE = 1124;
            constexpr static uint16_t CAN2_STATUS = 1126;
            constexpr static uint16_t CAN2_DISCARDEDTXMESSAGES = 1127;
            constexpr static uint16_t CAN2_ERRORSTATUS = 1128;
            constexpr static uint16_t CAN2_DELAYEDTXMESSAGES = 1129;
            constexpr static uint16_t SAMPLINGINTERVAL = 2048;
            constexpr static uint16_t TELEMETRYCOMMANDS = 2304;
            constexpr static uint16_t STATEMACHINEINTERVAL = 2305;
            constexpr static uint16_t STATEMACHINEACTIVATE = 2306;
            constexpr static uint16_t HVBATTERYMODE = 2307;
            constexpr static uint16_t ENCODERWHEELDIAMETER = 2308;
            constexpr static uint16_t ENCODERRESETPOSITION = 2309;
            constexpr static uint16_t SETRESET = 2320;
            constexpr static uint16_t IMU_NUMBER = 2592;
            constexpr static uint16_t IMU1_TEMPERATURE = 2597;
            constexpr static uint16_t IMU2_TEMPERATURE = 2598;
            constexpr static uint16_t IMU3_TEMPERATURE = 2599;
            constexpr static uint16_t IMU_ACCELX = 2600;
            constexpr static uint16_t IMU_ACCELY = 2601;
            constexpr static uint16_t IMU_ACCELZ = 2608;
            constexpr static uint16_t IMU_GYROX = 2609;
            constexpr static uint16_t IMU_GYROY = 2610;
            constexpr static uint16_t IMU_GYROZ = 2611;
            constexpr static uint16_t COOLINGPRESSURE = 2816;
            constexpr static uint16_t RESERVOIRTEMPERATURE = 2817;
            constexpr static uint16_t EBOXTEMPERATURE = 2818;
            constexpr static uint16_t STRIPECOUNT = 3072;
            constexpr static uint16_t POSITION = 3328;
            constexpr static uint16_t VELOCITY = 3329;
        };
        class SensorF_SDO_RespCode {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x581 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }

            // Value table of signal 'SensorF_SDO_RespCode'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t ERR_NON_EXISTING_OBJECT = 1;
            constexpr static uint8_t ERR_WRITE_ONLY_OBJECT = 2;
            constexpr static uint8_t ERR_READ_ONLY_OBJECT = 3;
            constexpr static uint8_t ERR_NO_ACCESS_IN_THIS_STATE = 4;
            constexpr static uint8_t ERR_OUT_OF_RANGE = 5;
        };
        class SensorF_OD_TitanCell12Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 785            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 785);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 785) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell12Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell11Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 784            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 784);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 784) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell11Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell10Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 783            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 783);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 783) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell10Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell9Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 782            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 782);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 782) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell9Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell8Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 781            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 781);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 781) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell8Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell7Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 780            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 780);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 780) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell7Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell6Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 779            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 779);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 779) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell6Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell5Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 778            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 778);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 778) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell5Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell4Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 777            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 777);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 777) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell4Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell3Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 776            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 776);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 776) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell3Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell2Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 775            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 775);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 775) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell2Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanCell1Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 774            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 774);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 774) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanCell1Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanHealthStatus {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 773            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 773);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 773) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_TitanHealthStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanLifeCycle {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 772            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 772);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 772) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_TitanLifeCycle'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanRemainingCapacity {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 771            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 771);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 771) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_TitanRemainingCapacity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_TitanRemainingCapacity";
        };
        class SensorF_OD_TitanCurrent {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 770            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 770);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 770) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_TitanCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanVoltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 769            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 769);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 769) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_TitanVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_TitanTemperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 768            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 768);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 768) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorF_OD_TitanTemperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell12Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 529            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 529);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 529) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell12Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell11Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 528            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 528);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 528) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell11Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell10Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 527            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 527);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 527) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell10Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell9Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 526            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 526);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 526) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell9Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell8Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 525            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 525);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 525) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell8Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell7Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 524            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 524);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 524) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell7Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell6Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 523            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 523);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 523) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell6Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell5Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 522            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 522);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 522) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell5Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell4Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 521            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 521);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 521) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell4Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell3Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 520            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 520);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 520) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell3Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell2Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 519            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 519);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 519) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell2Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionCell1Voltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 518            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 518);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 518) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCell1Voltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionHealthStatus {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 517            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 517);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 517) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_HyperionHealthStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionLifeCycle {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 516            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 516);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 516) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_HyperionLifeCycle'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionRemainingCapacity {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 515            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 515);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 515) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorF_OD_HyperionRemainingCapacity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_HyperionRemainingCapacity";
        };
        class SensorF_OD_HyperionCurrent {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 514            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 514);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 514) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_HyperionCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionVoltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 513            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 513);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 513) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_HyperionVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HyperionTemperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 512            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 512);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 512) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorF_OD_HyperionTemperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_SetReset {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2320            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2320);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2320) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_SetReset'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t RESET = 1;

            // Attributes of signal 'SensorF_OD_SetReset'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::WRITE_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_Velocity {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 3329            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 3329);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 3329) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_Velocity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_Position {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 3328            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 3328);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 3328) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_Position'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_StripeCount {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 3072            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 3072);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 3072) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_StripeCount'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_EboxTemperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2818            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2818);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2818) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_EboxTemperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 18000.0f;
            constexpr static float CANzero_SDO_Default = 80.0f;
        };
        class SensorF_OD_ReservoirTemperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2817            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2817);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2817) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_ReservoirTemperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 18000.0f;
            constexpr static float CANzero_SDO_Default = 80.0f;
        };
        class SensorF_OD_CoolingPressure {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2816            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2816);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2816) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_CoolingPressure'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 9810.0f;
            constexpr static float CANzero_SDO_Default = -1.9f;
        };
        class SensorF_OD_IMU_GyroZ {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2611            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2611);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2611) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_GyroZ'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_GyroY {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2610            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2610);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2610) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_GyroY'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_GyroX {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2609            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2609);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2609) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_GyroX'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_AccelZ {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2608            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2608);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2608) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_AccelZ'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_AccelY {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2601            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2601);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2601) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_AccelY'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_AccelX {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2600            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2600);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2600) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU_AccelX'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU3_Temperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2599            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2599);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2599) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU3_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU2_Temperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2598            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2598);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2598) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU2_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU1_Temperature {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2597            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2597);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2597) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorF_OD_IMU1_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_IMU_number {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2592            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2592);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2592) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_IMU_number'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_EncoderResetPosition {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2309            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2309);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2309) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_EncoderResetPosition'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t RESET = 1;

            // Attributes of signal 'SensorF_OD_EncoderResetPosition'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::WRITE_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_EncoderWheelDiameter {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2308            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(1);
            constexpr static float max = static_cast<float>(300);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2308);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2308) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.005f);
            }

            // Attributes of signal 'SensorF_OD_EncoderWheelDiameter'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 15000.0f;
            constexpr static float CANzero_SDO_Default = 75.0f;
        };
        class SensorF_OD_HVBatteryMode {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2307            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2307);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2307) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_HVBatteryMode'
            constexpr static uint8_t DISCHARGING = 0;
            constexpr static uint8_t CHARGING = 1;

            // Attributes of signal 'SensorF_OD_HVBatteryMode'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_StateMachineActivate {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2306            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2306);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2306) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_StateMachineActivate'
            constexpr static uint8_t ACTIVATE = 0;
            constexpr static uint8_t DEACTIVATE = 1;

            // Attributes of signal 'SensorF_OD_StateMachineActivate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_StateMachineInterval {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2305            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint8_t min = static_cast<uint8_t>(5);
            constexpr static uint8_t max = static_cast<uint8_t>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2305);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2305) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_StateMachineInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10.0f;
            constexpr static float CANzero_SDO_Default = 10.0f;
        };
        class SensorF_OD_TelemetryCommands {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2304            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2304);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2304) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_TelemetryCommands'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t LAUNCH_PREPARATION = 1;
            constexpr static uint8_t HV_ACTIVATION = 2;
            constexpr static uint8_t PUSHING_START = 3;
            constexpr static uint8_t PUSHING_END = 4;
            constexpr static uint8_t LAUNCH_START = 5;
            constexpr static uint8_t LAUNCH_ABORT = 6;
            constexpr static uint8_t IDLE = 7;
            constexpr static uint8_t ERROR_RESOLVED = 8;
            constexpr static uint8_t EMERGENCY = 9;

            // Attributes of signal 'SensorF_OD_TelemetryCommands'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_samplingInterval {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2048            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0.01);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 2048);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2048) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorF_OD_samplingInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 10.0f;
        };
        class SensorF_OD_CAN2_DelayedTxMessages {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1129            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1129);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1129) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN2_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_CAN2_DelayedTxMessages";
        };
        class SensorF_OD_CAN2_ErrorStatus {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1128            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1128);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1128) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN2_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'SensorF_OD_CAN2_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN2_DiscardedTxMessages {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1127            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1127);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1127) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN2_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_CAN2_DiscardedTxMessages";
        };
        class SensorF_OD_CAN2_Status {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1126            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1126);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1126) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN2_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'SensorF_OD_CAN2_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN2_Baudrate {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1124            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1124);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1124) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN2_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorF_OD_CAN2_autoErrorReset {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1123            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1123);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1123) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN2_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorF_OD_CAN2_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorF_OD_CAN2_lastErrorCode {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1122            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1122);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1122) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN2_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'SensorF_OD_CAN2_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN2_RxErrCnt {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1121            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1121);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1121) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN2_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN2_TxErrCnt {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1120            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1120);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1120) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN2_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN1_DelayedTxMessages {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1113            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1113);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1113) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN1_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_CAN1_DelayedTxMessages";
        };
        class SensorF_OD_CAN1_ErrorStatus {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1112            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1112);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1112) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN1_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'SensorF_OD_CAN1_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN1_DiscardedTxMessages {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1111            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1111);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1111) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN1_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorF_OD_CAN1_DiscardedTxMessages";
        };
        class SensorF_OD_CAN1_Status {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1110            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1110);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1110) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN1_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'SensorF_OD_CAN1_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN1_Baudrate {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1108            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1108);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1108) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN1_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorF_OD_CAN1_autoErrorReset {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1107            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1107);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1107) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN1_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorF_OD_CAN1_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorF_OD_CAN1_lastErrorCode {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1106            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1106);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1106) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_CAN1_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'SensorF_OD_CAN1_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN1_RxErrCnt {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1105            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1105);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1105) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN1_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CAN1_TxErrCnt {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1104            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1104);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1104) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_CAN1_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_BuildTime {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1073            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1073);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1073) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_BuildTime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_BuildDate {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1072            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1072);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1072) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_BuildDate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_ChipUID2 {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1057            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1057);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1057) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_ChipUID2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_ChipUID1 {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1056            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1056);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1056) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_ChipUID1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_SdcOut {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1046            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1046);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1046) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_SdcOut'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_SdcIn {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1045            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1045);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1045) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_SdcIn'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_runtime {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1044            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1044);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1044) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_runtime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_InputVoltage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1043            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1043);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1043) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorF_OD_InputVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_BoardTemp {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1042            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(625.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1042);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1042) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-30.0f);
            }

            // Attributes of signal 'SensorF_OD_BoardTemp'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 3000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_MemFree {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1041            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(262140);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1041);
                uint32_t rawValue = static_cast<uint32_t>((value) / (4));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1041) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (4);
            }

            // Attributes of signal 'SensorF_OD_MemFree'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_CpuUsage {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1040            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorF_SDO_ID::set(intel, motorola, dlc, 1040);
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1040) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'SensorF_OD_CpuUsage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_OdEntrySendInterval {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 33            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 33);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 33) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_OdEntrySendInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class SensorF_OD_SendOdOnBootup {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 32            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 32);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 32) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_SendOdOnBootup'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorF_OD_SendOdOnBootup'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_HeartbeatInterval {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 16            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 16);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 16) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_HeartbeatInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 250.0f;
            constexpr static float CANzero_SDO_Default = 250.0f;
        };
        class SensorF_OD_DbcVersion {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 5            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 5);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 5) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_DbcVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_StackVersion {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 4            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 4);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 4) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_StackVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_ProtocolVersion {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 3            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 3);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 3) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_ProtocolVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorF_OD_NodeStatus {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 2            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 2);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 2) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorF_OD_NodeStatus'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
            constexpr static uint8_t RESET = 128;

            // Attributes of signal 'SensorF_OD_NodeStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_OD_NodeID {
            public:
            // This signal is multiplexed by SensorF_SDO_ID == 1            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x581, 0x5C1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorF_SDO_ID::set(intel, motorola, dlc, 1);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorF_SDO_ID::get(intel, motorola) != 1) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorF_OD_NodeID'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorF_NodeState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x701 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'SensorF_NodeState'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
        };
        class SensorR_RX_LaunchComm {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2C2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0x3ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x3ull));
                return value;
            }

            // Value table of signal 'SensorR_RX_LaunchComm'
            constexpr static uint8_t START_LAUNCH = 0;
            constexpr static uint8_t ABORT_LAUNCH = 1;
            constexpr static uint8_t NO_START = 2;
        };
        class SensorR_RX_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2C2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'SensorR_RX_Enable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class SensorR_RX_ErrorReset {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2C2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }

            // Value table of signal 'SensorR_RX_ErrorReset'
            constexpr static bool NO_RESET = 0;
            constexpr static bool RESET = 1;
        };
        class BrakeF_TX_Status {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0x3ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x3ull));
                return value;
            }

            // Value table of signal 'BrakeF_TX_Status'
            constexpr static uint8_t DISENGAGED = 0;
            constexpr static uint8_t ENGAGEDEMERGENCY = 1;
            constexpr static uint8_t ENGAGEDSERVICE = 2;
        };
        class BrakeF_TX_Enabled {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'BrakeF_TX_Enabled'
            constexpr static bool DISABLED = 0;
            constexpr static bool ENABLED = 1;
        };
        class BrakeF_TX_ErrorFlag {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }

            // Value table of signal 'BrakeF_TX_ErrorFlag'
            constexpr static bool NO_ERROR = 0;
            constexpr static bool ERROR = 1;
        };
        class BrakeF_TX_SDC_Input {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0x10ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10ull) >> 4);
                return value;
            }

            // Value table of signal 'BrakeF_TX_SDC_Input'
            constexpr static bool OPEN = 0;
            constexpr static bool CLOSED = 1;
        };
        class BrakeF_TX_DeltaTime_Control {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xFF00ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00ull) >> 8);
                return value;
            }
        };
        class BrakeF_TX_MaxDeltaTime_Control {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x191 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }
        };
        class BrakeF_RX_ErrorReset {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'BrakeF_RX_ErrorReset'
            constexpr static bool NO_RESET = 0;
            constexpr static bool RESET = 1;
        };
        class BrakeF_RX_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'BrakeF_RX_Enable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class BrakeF_RX_Engage {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0xCull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xCull) >> 2);
                return value;
            }

            // Value table of signal 'BrakeF_RX_Engage'
            constexpr static uint8_t DISENGAGE = 0;
            constexpr static uint8_t ENGAGEEMERGENCY = 1;
            constexpr static uint8_t ENGAGESERVICE = 2;
        };
        class BrakeF_NodeState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x711 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'BrakeF_NodeState'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
        };
        class BrakeR_RX_ErrorReset {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'BrakeR_RX_ErrorReset'
            constexpr static bool NO_RESET = 0;
            constexpr static bool RESET = 1;
        };
        class BrakeR_RX_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'BrakeR_RX_Enable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class BrakeR_RX_Engage {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1D2 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0xCull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xCull) >> 2);
                return value;
            }

            // Value table of signal 'BrakeR_RX_Engage'
            constexpr static uint8_t DISENGAGE = 0;
            constexpr static uint8_t ENGAGEEMERGENCY = 1;
            constexpr static uint8_t ENGAGESERVICE = 2;
        };
        class PDU_TX_Enabled {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x19A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'PDU_TX_Enabled'
            constexpr static bool DISABLED = 0;
            constexpr static bool ENABLED = 1;
        };
        class PDU_TX_ErrorFlag {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x19A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'PDU_TX_ErrorFlag'
            constexpr static bool NO_ERROR = 0;
            constexpr static bool ERROR = 1;
        };
        class PDU_TX_PEHWEnabled {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x19A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'PDU_TX_PEHWEnabled'
            constexpr static bool DISABLED = 0;
            constexpr static bool ENABLED = 1;
        };
        class PDU_RX_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'PDU_RX_Enable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class PDU_RX_ErrorReset {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'PDU_RX_ErrorReset'
            constexpr static bool NO_RESET = 0;
            constexpr static bool RESET = 1;
        };
        class PDU_RX_PEHWEnable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'PDU_RX_PEHWEnable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class PDU_HPCh1_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x21A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(16.38);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.004f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.004f);
            }
        };
        class PDU_HPCh2_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x21A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(16.38);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.004f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF0000ull) >> 16);
                return value * (0.004f);
            }
        };
        class PDU_HPCh3_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x21A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(16.38);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.004f)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF00000000ull) >> 32);
                return value * (0.004f);
            }
        };
        class PDU_HPCh4_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x21A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(16.38);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.004f)));
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0xFFF000000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000000ull) >> 48);
                return value * (0.004f);
            }
        };
        class PDU_LPCh1_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh1_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh10_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xFF00ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00ull) >> 8);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh10_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh2_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh2_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh3_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF000000ull) >> 24);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh3_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh8_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00000000ull) >> 32);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh8_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh9_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x25A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 40) & 0xFF0000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000000000ull) >> 40);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_LPCh9_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_HPCh1_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_HPCh1_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_HPCh2_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xFF00ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00ull) >> 8);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_HPCh2_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_HPCh3_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_HPCh3_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_HPCh4_Dutycycle {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x2DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF000000ull) >> 24);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_HPCh4_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh1_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x51A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.002f);
            }
        };
        class PDU_LPCh2_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x51A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xFFF000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000ull) >> 12);
                return value * (0.002f);
            }
        };
        class PDU_LPCh3_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x51A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000ull) >> 24);
                return value * (0.002f);
            }
        };
        class PDU_LPCh4_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x51A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 36) & 0xFFF000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000ull) >> 36);
                return value * (0.002f);
            }
        };
        class PDU_LPCh5_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x51A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0xFFF000000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000000ull) >> 48);
                return value * (0.002f);
            }
        };
        class PDU_LPCh10_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x55A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.002f);
            }
        };
        class PDU_LPCh6_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x55A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xFFF000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000ull) >> 12);
                return value * (0.002f);
            }
        };
        class PDU_LPCh7_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x55A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFFF000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000ull) >> 24);
                return value * (0.002f);
            }
        };
        class PDU_LPCh8_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x55A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 36) & 0xFFF000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000ull) >> 36);
                return value * (0.002f);
            }
        };
        class PDU_LPCh9_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x55A };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(8.19);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.002f)));
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0xFFF000000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000000000000ull) >> 48);
                return value * (0.002f);
            }
        };
        class PDU_NodeState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x71A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'PDU_NodeState'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
        };
        class HVCU_RX_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1CA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'HVCU_RX_Enable'
            constexpr static bool DISABLE = 0;
            constexpr static bool ENABLE = 1;
        };
        class HVCU_RX_Activate {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1CA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'HVCU_RX_Activate'
            constexpr static bool DEACTIVATE = 0;
            constexpr static bool ACTIVATE = 1;
        };
        class HVCU_RX_ErrorReset {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1CA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'HVCU_RX_ErrorReset'
            constexpr static bool NO_RESET = 0;
            constexpr static bool RESET = 1;
        };
        class HVCU_RX_Charging {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x1CA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }

            // Value table of signal 'HVCU_RX_Charging'
            constexpr static bool NOT_CHARGING = 0;
            constexpr static bool CHARGING = 1;
        };
        class TelemetryNode_NodeState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x722 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'TelemetryNode_NodeState'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
        };
        class CLU_NodeState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x714 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFFull));
                return value;
            }

            // Value table of signal 'CLU_NodeState'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
        };
    }

    /**********************************************************************************************
    * Namespace containing all messages                                                           *
    ***********************************************************************************************/
    namespace messages {
        class SensorF_RX_IgnoreLevel {
            public:
            constexpr static uint32_t id = 0x430;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_IgnoreLevel = signals::SensorF_IgnoreLevel;
            using SensorF_EmergencyLevel = signals::SensorF_EmergencyLevel;

            // Attributes of message 'SensorF_RX_IgnoreLevel'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_Error {
            public:
            constexpr static uint32_t id = 0x5;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_TX_ErrorPrio = signals::PDU_TX_ErrorPrio;

            // Attributes of message 'PDU_TX_Error'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_TX_Error {
            public:
            constexpr static uint32_t id = 0x4;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_TX_ErrorPrio = signals::CLU_TX_ErrorPrio;

            // Attributes of message 'CLU_TX_Error'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_Error {
            public:
            constexpr static uint32_t id = 0x3;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_TX_ErrorPrio = signals::BrakeF_TX_ErrorPrio;

            // Attributes of message 'BrakeF_TX_Error'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_RX_ActionRequest {
            public:
            constexpr static uint32_t id = 0x234;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_RX_ActionRequest_ = signals::CLU_RX_ActionRequest;

            // Attributes of message 'CLU_RX_ActionRequest'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BMS44_2_Frame {
            public:
            constexpr static uint32_t id = 0x1109216;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = true;

            // Signals
            using BMS44_2_FramePayload = signals::BMS44_2_FramePayload;

            // Attributes of message 'BMS44_2_Frame'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_RX_StatusLedControll {
            public:
            constexpr static uint32_t id = 0x700;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_RX_ErrorStatus = signals::CLU_RX_ErrorStatus;

            // Attributes of message 'CLU_RX_StatusLedControll'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_TX_LevitationState {
            public:
            constexpr static uint32_t id = 0x250;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_LevitationState = signals::CLU_LevitationState;

            // Attributes of message 'CLU_TX_LevitationState'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_TX_CoolingState {
            public:
            constexpr static uint32_t id = 0x300;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_RequiresCooling = signals::CLU_RequiresCooling;

            // Attributes of message 'CLU_TX_CoolingState'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class MDB1_TX_Temperature {
            public:
            constexpr static uint32_t id = 0x104;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using MDB1_Temperature = signals::MDB1_Temperature;

            // Attributes of message 'MDB1_TX_Temperature'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PressureCooling {
            public:
            constexpr static uint32_t id = 0x411;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_TX_Pressure_Reservoir = signals::BrakeF_TX_Pressure_Reservoir;

            // Attributes of message 'BrakeF_TX_PressureCooling'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_LP_Enable {
            public:
            constexpr static uint32_t id = 0x20B;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_RX_LPCh4_Enable = signals::PDU_RX_LPCh4_Enable;
            using PDU_RX_LPCh5_Enable = signals::PDU_RX_LPCh5_Enable;
            using PDU_RX_LPCh6_Enable = signals::PDU_RX_LPCh6_Enable;
            using PDU_RX_LPCh7_Enable = signals::PDU_RX_LPCh7_Enable;

            // Attributes of message 'PDU_RX_LP_Enable'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BMS44_1_Frame {
            public:
            constexpr static uint32_t id = 0x10ECE0;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = true;

            // Signals
            using BMS44_1_FramePayload = signals::BMS44_1_FramePayload;

            // Attributes of message 'BMS44_1_Frame'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class Track_TX_Respond {
            public:
            constexpr static uint32_t id = 0x1;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using Track_TX_Response = signals::Track_TX_Response;

            // Attributes of message 'Track_TX_Respond'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class Track_RX_Controll {
            public:
            constexpr static uint32_t id = 0x0;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using Track_RX_Command = signals::Track_RX_Command;

            // Attributes of message 'Track_RX_Controll'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class TEST_GROUND_STATION_CONTROLL {
            public:
            constexpr static uint32_t id = 0x200;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using TEST_GROUND_STATION_COMMAND = signals::TEST_GROUND_STATION_COMMAND;

            // Attributes of message 'TEST_GROUND_STATION_CONTROLL'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class MDB1_TX_State {
            public:
            constexpr static uint32_t id = 0x100;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using MDB1_State = signals::MDB1_State;

            // Attributes of message 'MDB1_TX_State'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class OpticalSensor_TX_MainData {
            public:
            constexpr static uint32_t id = 0x1FFFFFFA;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = true;

            // Signals
            using OpticalSensor_TX_Timestamp = signals::OpticalSensor_TX_Timestamp;
            using OpticalSensor_TX_Vel = signals::OpticalSensor_TX_Vel;
            using OpticalSensor_TX_Distance = signals::OpticalSensor_TX_Distance;

            // Attributes of message 'OpticalSensor_TX_MainData'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class OpticalSensor_TX_Status {
            public:
            constexpr static uint32_t id = 0x1FFFFFFC;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = true;

            // Signals
            using OpticalSensor_TX_SerialNo = signals::OpticalSensor_TX_SerialNo;
            using OpticalSensor_TX_SensorNo = signals::OpticalSensor_TX_SensorNo;
            using OpticalSensor_TX_Temp = signals::OpticalSensor_TX_Temp;
            using OpticalSensor_TX_LEDCurrent = signals::OpticalSensor_TX_LEDCurrent;
            using OpticalSensor_TX_STST = signals::OpticalSensor_TX_STST;
            using OpticalSensor_TX_SelfTest = signals::OpticalSensor_TX_SelfTest;
            using OpticalSensor_TX_SensorOK = signals::OpticalSensor_TX_SensorOK;
            using OpticalSensor_TX_TempOK = signals::OpticalSensor_TX_TempOK;
            using OpticalSensor_TX_OpticOK = signals::OpticalSensor_TX_OpticOK;
            using OpticalSensor_TX_CurrentLowHigh = signals::OpticalSensor_TX_CurrentLowHigh;
            using OpticalSensor_TX_CurrentOK = signals::OpticalSensor_TX_CurrentOK;
            using OpticalSensor_TX_PowerOK = signals::OpticalSensor_TX_PowerOK;
            using OpticalSensor_TX_LED_Cal = signals::OpticalSensor_TX_LED_Cal;
            using OpticalSensor_TX_LEDStatus = signals::OpticalSensor_TX_LEDStatus;
            using OpticalSensor_TX_V25OK = signals::OpticalSensor_TX_V25OK;
            using OpticalSensor_TX_V33OK = signals::OpticalSensor_TX_V33OK;
            using OpticalSensor_TX_VeeOK = signals::OpticalSensor_TX_VeeOK;
            using OpticalSensor_TX_VddOK = signals::OpticalSensor_TX_VddOK;

            // Attributes of message 'OpticalSensor_TX_Status'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CANzero_NMT {
            public:
            constexpr static uint32_t id = 0x2;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using CANzero_NMT_State = signals::CANzero_NMT_State;
            using CANzero_NMT_Node = signals::CANzero_NMT_Node;

            // Attributes of message 'CANzero_NMT'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_EMCY {
            public:
            constexpr static uint32_t id = 0x81;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_W0_OtherWarning = signals::SensorF_W0_OtherWarning;
            using SensorF_W1_StateMTransitionW = signals::SensorF_W1_StateMTransitionW;
            using SensorF_W2_encoderOORWarning = signals::SensorF_W2_encoderOORWarning;
            using SensorF_E0_OtherError = signals::SensorF_E0_OtherError;
            using SensorF_E1_CPUOverTemp = signals::SensorF_E1_CPUOverTemp;
            using SensorF_E2_OverVolt = signals::SensorF_E2_OverVolt;
            using SensorF_E3_UnderVolt = signals::SensorF_E3_UnderVolt;
            using SensorF_E4_InvalidPosition = signals::SensorF_E4_InvalidPosition;
            using SensorF_E5_ReservoirOverTemp = signals::SensorF_E5_ReservoirOverTemp;
            using SensorF_E6_CLUHeartbeatMiss = signals::SensorF_E6_CLUHeartbeatMiss;
            using SensorF_E7_BECUHeartbeatMiss = signals::SensorF_E7_BECUHeartbeatMiss;
            using SensorF_E8_PDUHeartbeatMiss = signals::SensorF_E8_PDUHeartbeatMiss;
            using SensorF_E9_TelemetryHeartbeatMiss = signals::SensorF_E9_TelemetryHeartbeatMiss;
            using SensorF_E10_TitanOverTemp = signals::SensorF_E10_TitanOverTemp;
            using SensorF_E11_HyperionOverTemp = signals::SensorF_E11_HyperionOverTemp;
            using SensorF_E12_TitanLowHp = signals::SensorF_E12_TitanLowHp;
            using SensorF_E13_HyperionLowHp = signals::SensorF_E13_HyperionLowHp;
            using SensorF_E14_TitanLowCap = signals::SensorF_E14_TitanLowCap;
            using SensorF_E15_HyperionLowCap = signals::SensorF_E15_HyperionLowCap;
            using SensorF_E16_EboxOverTemp = signals::SensorF_E16_EboxOverTemp;

            // Attributes of message 'SensorF_EMCY'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_StatePod {
            public:
            constexpr static uint32_t id = 0x181;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_TX_PodState = signals::SensorF_TX_PodState;

            // Attributes of message 'SensorF_TX_StatePod'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_RX_PDO1 {
            public:
            constexpr static uint32_t id = 0x1C1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_RX_PDO1'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_FSMTransitions {
            public:
            constexpr static uint32_t id = 0x201;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_TX_BrakesTransition = signals::SensorF_TX_BrakesTransition;
            using SensorF_TX_EnableTransition = signals::SensorF_TX_EnableTransition;
            using SensorF_TX_ErrorResetTransition = signals::SensorF_TX_ErrorResetTransition;
            using SensorF_TX_HWEnableTransition = signals::SensorF_TX_HWEnableTransition;
            using SensorF_TX_LaunchSetupTransition = signals::SensorF_TX_LaunchSetupTransition;
            using SensorF_TX_LaunchStartTransition = signals::SensorF_TX_LaunchStartTransition;
            using SensorF_TX_PreopTransition = signals::SensorF_TX_PreopTransition;
            using SensorF_TX_SetHVTransition = signals::SensorF_TX_SetHVTransition;
            using SensorF_TX_EnableBrakeFT = signals::SensorF_TX_EnableBrakeFT;
            using SensorF_TX_EnableBrakeRT = signals::SensorF_TX_EnableBrakeRT;
            using SensorF_TX_EnableHVCUT = signals::SensorF_TX_EnableHVCUT;
            using SensorF_TX_EnablePDUT = signals::SensorF_TX_EnablePDUT;
            using SensorF_TX_EnableSensorRT = signals::SensorF_TX_EnableSensorRT;
            using SensorF_TX_PreopSensorRT = signals::SensorF_TX_PreopSensorRT;
            using SensorF_TX_PreopBrakeFT = signals::SensorF_TX_PreopBrakeFT;
            using SensorF_TX_PreopBrakeRT = signals::SensorF_TX_PreopBrakeRT;
            using SensorF_TX_PreopPDUT = signals::SensorF_TX_PreopPDUT;
            using SensorF_TX_PreopHVCUT = signals::SensorF_TX_PreopHVCUT;

            // Attributes of message 'SensorF_TX_FSMTransitions'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_RX_PDO2 {
            public:
            constexpr static uint32_t id = 0x241;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_RX_PDO2'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO3 {
            public:
            constexpr static uint32_t id = 0x281;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO3'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_RX_PDO3 {
            public:
            constexpr static uint32_t id = 0x2C1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_RX_PDO3'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO4 {
            public:
            constexpr static uint32_t id = 0x301;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO4'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_RX_PDO4 {
            public:
            constexpr static uint32_t id = 0x341;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_RX_PDO4'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO5 {
            public:
            constexpr static uint32_t id = 0x381;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO5'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO6 {
            public:
            constexpr static uint32_t id = 0x3C1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO6'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_EncoderFront {
            public:
            constexpr static uint32_t id = 0x401;
            constexpr static uint8_t dlc = 7;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_Pos_EncoderFront = signals::SensorF_Pos_EncoderFront;
            using SensorF_Vel_EncoderFront = signals::SensorF_Vel_EncoderFront;
            using SensorF_GyroFront_X = signals::SensorF_GyroFront_X;

            // Attributes of message 'SensorF_TX_EncoderFront'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_BMS {
            public:
            constexpr static uint32_t id = 0x441;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_TX_BatteryTemp = signals::SensorF_TX_BatteryTemp;

            // Attributes of message 'SensorF_TX_BMS'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_Temperature {
            public:
            constexpr static uint32_t id = 0x481;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_LIMT_Stator_1 = signals::SensorF_LIMT_Stator_1;
            using SensorF_LIMT_Stator_2 = signals::SensorF_LIMT_Stator_2;
            using SensorF_LIMT_Stator_3 = signals::SensorF_LIMT_Stator_3;
            using SensorF_LIMT_Stator_4 = signals::SensorF_LIMT_Stator_4;
            using SensorF_Cooling_Temp = signals::SensorF_Cooling_Temp;

            // Attributes of message 'SensorF_TX_Temperature'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_AccFront {
            public:
            constexpr static uint32_t id = 0x4C1;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_AccFront_X = signals::SensorF_AccFront_X;
            using SensorF_AccFront_Y = signals::SensorF_AccFront_Y;
            using SensorF_AccFront_Z = signals::SensorF_AccFront_Z;
            using SensorF_GyroFront_Z = signals::SensorF_GyroFront_Z;

            // Attributes of message 'SensorF_TX_AccFront'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO11 {
            public:
            constexpr static uint32_t id = 0x501;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO11'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_TX_PDO12 {
            public:
            constexpr static uint32_t id = 0x541;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_TX_PDO12'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_SDO_Resp {
            public:
            constexpr static uint32_t id = 0x581;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_SDO_ID = signals::SensorF_SDO_ID;
            using SensorF_SDO_RespCode = signals::SensorF_SDO_RespCode;
            using SensorF_OD_TitanCell12Voltage = signals::SensorF_OD_TitanCell12Voltage;
            using SensorF_OD_TitanCell11Voltage = signals::SensorF_OD_TitanCell11Voltage;
            using SensorF_OD_TitanCell10Voltage = signals::SensorF_OD_TitanCell10Voltage;
            using SensorF_OD_TitanCell9Voltage = signals::SensorF_OD_TitanCell9Voltage;
            using SensorF_OD_TitanCell8Voltage = signals::SensorF_OD_TitanCell8Voltage;
            using SensorF_OD_TitanCell7Voltage = signals::SensorF_OD_TitanCell7Voltage;
            using SensorF_OD_TitanCell6Voltage = signals::SensorF_OD_TitanCell6Voltage;
            using SensorF_OD_TitanCell5Voltage = signals::SensorF_OD_TitanCell5Voltage;
            using SensorF_OD_TitanCell4Voltage = signals::SensorF_OD_TitanCell4Voltage;
            using SensorF_OD_TitanCell3Voltage = signals::SensorF_OD_TitanCell3Voltage;
            using SensorF_OD_TitanCell2Voltage = signals::SensorF_OD_TitanCell2Voltage;
            using SensorF_OD_TitanCell1Voltage = signals::SensorF_OD_TitanCell1Voltage;
            using SensorF_OD_TitanHealthStatus = signals::SensorF_OD_TitanHealthStatus;
            using SensorF_OD_TitanLifeCycle = signals::SensorF_OD_TitanLifeCycle;
            using SensorF_OD_TitanRemainingCapacity = signals::SensorF_OD_TitanRemainingCapacity;
            using SensorF_OD_TitanCurrent = signals::SensorF_OD_TitanCurrent;
            using SensorF_OD_TitanVoltage = signals::SensorF_OD_TitanVoltage;
            using SensorF_OD_TitanTemperature = signals::SensorF_OD_TitanTemperature;
            using SensorF_OD_HyperionCell12Voltage = signals::SensorF_OD_HyperionCell12Voltage;
            using SensorF_OD_HyperionCell11Voltage = signals::SensorF_OD_HyperionCell11Voltage;
            using SensorF_OD_HyperionCell10Voltage = signals::SensorF_OD_HyperionCell10Voltage;
            using SensorF_OD_HyperionCell9Voltage = signals::SensorF_OD_HyperionCell9Voltage;
            using SensorF_OD_HyperionCell8Voltage = signals::SensorF_OD_HyperionCell8Voltage;
            using SensorF_OD_HyperionCell7Voltage = signals::SensorF_OD_HyperionCell7Voltage;
            using SensorF_OD_HyperionCell6Voltage = signals::SensorF_OD_HyperionCell6Voltage;
            using SensorF_OD_HyperionCell5Voltage = signals::SensorF_OD_HyperionCell5Voltage;
            using SensorF_OD_HyperionCell4Voltage = signals::SensorF_OD_HyperionCell4Voltage;
            using SensorF_OD_HyperionCell3Voltage = signals::SensorF_OD_HyperionCell3Voltage;
            using SensorF_OD_HyperionCell2Voltage = signals::SensorF_OD_HyperionCell2Voltage;
            using SensorF_OD_HyperionCell1Voltage = signals::SensorF_OD_HyperionCell1Voltage;
            using SensorF_OD_HyperionHealthStatus = signals::SensorF_OD_HyperionHealthStatus;
            using SensorF_OD_HyperionLifeCycle = signals::SensorF_OD_HyperionLifeCycle;
            using SensorF_OD_HyperionRemainingCapacity = signals::SensorF_OD_HyperionRemainingCapacity;
            using SensorF_OD_HyperionCurrent = signals::SensorF_OD_HyperionCurrent;
            using SensorF_OD_HyperionVoltage = signals::SensorF_OD_HyperionVoltage;
            using SensorF_OD_HyperionTemperature = signals::SensorF_OD_HyperionTemperature;
            using SensorF_OD_SetReset = signals::SensorF_OD_SetReset;
            using SensorF_OD_Velocity = signals::SensorF_OD_Velocity;
            using SensorF_OD_Position = signals::SensorF_OD_Position;
            using SensorF_OD_StripeCount = signals::SensorF_OD_StripeCount;
            using SensorF_OD_EboxTemperature = signals::SensorF_OD_EboxTemperature;
            using SensorF_OD_ReservoirTemperature = signals::SensorF_OD_ReservoirTemperature;
            using SensorF_OD_CoolingPressure = signals::SensorF_OD_CoolingPressure;
            using SensorF_OD_IMU_GyroZ = signals::SensorF_OD_IMU_GyroZ;
            using SensorF_OD_IMU_GyroY = signals::SensorF_OD_IMU_GyroY;
            using SensorF_OD_IMU_GyroX = signals::SensorF_OD_IMU_GyroX;
            using SensorF_OD_IMU_AccelZ = signals::SensorF_OD_IMU_AccelZ;
            using SensorF_OD_IMU_AccelY = signals::SensorF_OD_IMU_AccelY;
            using SensorF_OD_IMU_AccelX = signals::SensorF_OD_IMU_AccelX;
            using SensorF_OD_IMU3_Temperature = signals::SensorF_OD_IMU3_Temperature;
            using SensorF_OD_IMU2_Temperature = signals::SensorF_OD_IMU2_Temperature;
            using SensorF_OD_IMU1_Temperature = signals::SensorF_OD_IMU1_Temperature;
            using SensorF_OD_IMU_number = signals::SensorF_OD_IMU_number;
            using SensorF_OD_EncoderResetPosition = signals::SensorF_OD_EncoderResetPosition;
            using SensorF_OD_EncoderWheelDiameter = signals::SensorF_OD_EncoderWheelDiameter;
            using SensorF_OD_HVBatteryMode = signals::SensorF_OD_HVBatteryMode;
            using SensorF_OD_StateMachineActivate = signals::SensorF_OD_StateMachineActivate;
            using SensorF_OD_StateMachineInterval = signals::SensorF_OD_StateMachineInterval;
            using SensorF_OD_TelemetryCommands = signals::SensorF_OD_TelemetryCommands;
            using SensorF_OD_samplingInterval = signals::SensorF_OD_samplingInterval;
            using SensorF_OD_CAN2_DelayedTxMessages = signals::SensorF_OD_CAN2_DelayedTxMessages;
            using SensorF_OD_CAN2_ErrorStatus = signals::SensorF_OD_CAN2_ErrorStatus;
            using SensorF_OD_CAN2_DiscardedTxMessages = signals::SensorF_OD_CAN2_DiscardedTxMessages;
            using SensorF_OD_CAN2_Status = signals::SensorF_OD_CAN2_Status;
            using SensorF_OD_CAN2_Baudrate = signals::SensorF_OD_CAN2_Baudrate;
            using SensorF_OD_CAN2_autoErrorReset = signals::SensorF_OD_CAN2_autoErrorReset;
            using SensorF_OD_CAN2_lastErrorCode = signals::SensorF_OD_CAN2_lastErrorCode;
            using SensorF_OD_CAN2_RxErrCnt = signals::SensorF_OD_CAN2_RxErrCnt;
            using SensorF_OD_CAN2_TxErrCnt = signals::SensorF_OD_CAN2_TxErrCnt;
            using SensorF_OD_CAN1_DelayedTxMessages = signals::SensorF_OD_CAN1_DelayedTxMessages;
            using SensorF_OD_CAN1_ErrorStatus = signals::SensorF_OD_CAN1_ErrorStatus;
            using SensorF_OD_CAN1_DiscardedTxMessages = signals::SensorF_OD_CAN1_DiscardedTxMessages;
            using SensorF_OD_CAN1_Status = signals::SensorF_OD_CAN1_Status;
            using SensorF_OD_CAN1_Baudrate = signals::SensorF_OD_CAN1_Baudrate;
            using SensorF_OD_CAN1_autoErrorReset = signals::SensorF_OD_CAN1_autoErrorReset;
            using SensorF_OD_CAN1_lastErrorCode = signals::SensorF_OD_CAN1_lastErrorCode;
            using SensorF_OD_CAN1_RxErrCnt = signals::SensorF_OD_CAN1_RxErrCnt;
            using SensorF_OD_CAN1_TxErrCnt = signals::SensorF_OD_CAN1_TxErrCnt;
            using SensorF_OD_BuildTime = signals::SensorF_OD_BuildTime;
            using SensorF_OD_BuildDate = signals::SensorF_OD_BuildDate;
            using SensorF_OD_ChipUID2 = signals::SensorF_OD_ChipUID2;
            using SensorF_OD_ChipUID1 = signals::SensorF_OD_ChipUID1;
            using SensorF_OD_SdcOut = signals::SensorF_OD_SdcOut;
            using SensorF_OD_SdcIn = signals::SensorF_OD_SdcIn;
            using SensorF_OD_runtime = signals::SensorF_OD_runtime;
            using SensorF_OD_InputVoltage = signals::SensorF_OD_InputVoltage;
            using SensorF_OD_BoardTemp = signals::SensorF_OD_BoardTemp;
            using SensorF_OD_MemFree = signals::SensorF_OD_MemFree;
            using SensorF_OD_CpuUsage = signals::SensorF_OD_CpuUsage;
            using SensorF_OD_OdEntrySendInterval = signals::SensorF_OD_OdEntrySendInterval;
            using SensorF_OD_SendOdOnBootup = signals::SensorF_OD_SendOdOnBootup;
            using SensorF_OD_HeartbeatInterval = signals::SensorF_OD_HeartbeatInterval;
            using SensorF_OD_DbcVersion = signals::SensorF_OD_DbcVersion;
            using SensorF_OD_StackVersion = signals::SensorF_OD_StackVersion;
            using SensorF_OD_ProtocolVersion = signals::SensorF_OD_ProtocolVersion;
            using SensorF_OD_NodeStatus = signals::SensorF_OD_NodeStatus;
            using SensorF_OD_NodeID = signals::SensorF_OD_NodeID;

            // Attributes of message 'SensorF_SDO_Resp'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_SDO_Req_Up {
            public:
            constexpr static uint32_t id = 0x5C1;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_SDO_ID = signals::SensorF_SDO_ID;
            using SensorF_OD_TitanCell12Voltage = signals::SensorF_OD_TitanCell12Voltage;
            using SensorF_OD_TitanCell11Voltage = signals::SensorF_OD_TitanCell11Voltage;
            using SensorF_OD_TitanCell10Voltage = signals::SensorF_OD_TitanCell10Voltage;
            using SensorF_OD_TitanCell9Voltage = signals::SensorF_OD_TitanCell9Voltage;
            using SensorF_OD_TitanCell8Voltage = signals::SensorF_OD_TitanCell8Voltage;
            using SensorF_OD_TitanCell7Voltage = signals::SensorF_OD_TitanCell7Voltage;
            using SensorF_OD_TitanCell6Voltage = signals::SensorF_OD_TitanCell6Voltage;
            using SensorF_OD_TitanCell5Voltage = signals::SensorF_OD_TitanCell5Voltage;
            using SensorF_OD_TitanCell4Voltage = signals::SensorF_OD_TitanCell4Voltage;
            using SensorF_OD_TitanCell3Voltage = signals::SensorF_OD_TitanCell3Voltage;
            using SensorF_OD_TitanCell2Voltage = signals::SensorF_OD_TitanCell2Voltage;
            using SensorF_OD_TitanCell1Voltage = signals::SensorF_OD_TitanCell1Voltage;
            using SensorF_OD_TitanHealthStatus = signals::SensorF_OD_TitanHealthStatus;
            using SensorF_OD_TitanLifeCycle = signals::SensorF_OD_TitanLifeCycle;
            using SensorF_OD_TitanRemainingCapacity = signals::SensorF_OD_TitanRemainingCapacity;
            using SensorF_OD_TitanCurrent = signals::SensorF_OD_TitanCurrent;
            using SensorF_OD_TitanVoltage = signals::SensorF_OD_TitanVoltage;
            using SensorF_OD_TitanTemperature = signals::SensorF_OD_TitanTemperature;
            using SensorF_OD_HyperionCell12Voltage = signals::SensorF_OD_HyperionCell12Voltage;
            using SensorF_OD_HyperionCell11Voltage = signals::SensorF_OD_HyperionCell11Voltage;
            using SensorF_OD_HyperionCell10Voltage = signals::SensorF_OD_HyperionCell10Voltage;
            using SensorF_OD_HyperionCell9Voltage = signals::SensorF_OD_HyperionCell9Voltage;
            using SensorF_OD_HyperionCell8Voltage = signals::SensorF_OD_HyperionCell8Voltage;
            using SensorF_OD_HyperionCell7Voltage = signals::SensorF_OD_HyperionCell7Voltage;
            using SensorF_OD_HyperionCell6Voltage = signals::SensorF_OD_HyperionCell6Voltage;
            using SensorF_OD_HyperionCell5Voltage = signals::SensorF_OD_HyperionCell5Voltage;
            using SensorF_OD_HyperionCell4Voltage = signals::SensorF_OD_HyperionCell4Voltage;
            using SensorF_OD_HyperionCell3Voltage = signals::SensorF_OD_HyperionCell3Voltage;
            using SensorF_OD_HyperionCell2Voltage = signals::SensorF_OD_HyperionCell2Voltage;
            using SensorF_OD_HyperionCell1Voltage = signals::SensorF_OD_HyperionCell1Voltage;
            using SensorF_OD_HyperionHealthStatus = signals::SensorF_OD_HyperionHealthStatus;
            using SensorF_OD_HyperionLifeCycle = signals::SensorF_OD_HyperionLifeCycle;
            using SensorF_OD_HyperionRemainingCapacity = signals::SensorF_OD_HyperionRemainingCapacity;
            using SensorF_OD_HyperionCurrent = signals::SensorF_OD_HyperionCurrent;
            using SensorF_OD_HyperionVoltage = signals::SensorF_OD_HyperionVoltage;
            using SensorF_OD_HyperionTemperature = signals::SensorF_OD_HyperionTemperature;
            using SensorF_OD_SetReset = signals::SensorF_OD_SetReset;
            using SensorF_OD_Velocity = signals::SensorF_OD_Velocity;
            using SensorF_OD_Position = signals::SensorF_OD_Position;
            using SensorF_OD_StripeCount = signals::SensorF_OD_StripeCount;
            using SensorF_OD_EboxTemperature = signals::SensorF_OD_EboxTemperature;
            using SensorF_OD_ReservoirTemperature = signals::SensorF_OD_ReservoirTemperature;
            using SensorF_OD_CoolingPressure = signals::SensorF_OD_CoolingPressure;
            using SensorF_OD_IMU_GyroZ = signals::SensorF_OD_IMU_GyroZ;
            using SensorF_OD_IMU_GyroY = signals::SensorF_OD_IMU_GyroY;
            using SensorF_OD_IMU_GyroX = signals::SensorF_OD_IMU_GyroX;
            using SensorF_OD_IMU_AccelZ = signals::SensorF_OD_IMU_AccelZ;
            using SensorF_OD_IMU_AccelY = signals::SensorF_OD_IMU_AccelY;
            using SensorF_OD_IMU_AccelX = signals::SensorF_OD_IMU_AccelX;
            using SensorF_OD_IMU3_Temperature = signals::SensorF_OD_IMU3_Temperature;
            using SensorF_OD_IMU2_Temperature = signals::SensorF_OD_IMU2_Temperature;
            using SensorF_OD_IMU1_Temperature = signals::SensorF_OD_IMU1_Temperature;
            using SensorF_OD_IMU_number = signals::SensorF_OD_IMU_number;
            using SensorF_OD_EncoderResetPosition = signals::SensorF_OD_EncoderResetPosition;
            using SensorF_OD_EncoderWheelDiameter = signals::SensorF_OD_EncoderWheelDiameter;
            using SensorF_OD_HVBatteryMode = signals::SensorF_OD_HVBatteryMode;
            using SensorF_OD_StateMachineActivate = signals::SensorF_OD_StateMachineActivate;
            using SensorF_OD_StateMachineInterval = signals::SensorF_OD_StateMachineInterval;
            using SensorF_OD_TelemetryCommands = signals::SensorF_OD_TelemetryCommands;
            using SensorF_OD_samplingInterval = signals::SensorF_OD_samplingInterval;
            using SensorF_OD_CAN2_DelayedTxMessages = signals::SensorF_OD_CAN2_DelayedTxMessages;
            using SensorF_OD_CAN2_ErrorStatus = signals::SensorF_OD_CAN2_ErrorStatus;
            using SensorF_OD_CAN2_DiscardedTxMessages = signals::SensorF_OD_CAN2_DiscardedTxMessages;
            using SensorF_OD_CAN2_Status = signals::SensorF_OD_CAN2_Status;
            using SensorF_OD_CAN2_Baudrate = signals::SensorF_OD_CAN2_Baudrate;
            using SensorF_OD_CAN2_autoErrorReset = signals::SensorF_OD_CAN2_autoErrorReset;
            using SensorF_OD_CAN2_lastErrorCode = signals::SensorF_OD_CAN2_lastErrorCode;
            using SensorF_OD_CAN2_RxErrCnt = signals::SensorF_OD_CAN2_RxErrCnt;
            using SensorF_OD_CAN2_TxErrCnt = signals::SensorF_OD_CAN2_TxErrCnt;
            using SensorF_OD_CAN1_DelayedTxMessages = signals::SensorF_OD_CAN1_DelayedTxMessages;
            using SensorF_OD_CAN1_ErrorStatus = signals::SensorF_OD_CAN1_ErrorStatus;
            using SensorF_OD_CAN1_DiscardedTxMessages = signals::SensorF_OD_CAN1_DiscardedTxMessages;
            using SensorF_OD_CAN1_Status = signals::SensorF_OD_CAN1_Status;
            using SensorF_OD_CAN1_Baudrate = signals::SensorF_OD_CAN1_Baudrate;
            using SensorF_OD_CAN1_autoErrorReset = signals::SensorF_OD_CAN1_autoErrorReset;
            using SensorF_OD_CAN1_lastErrorCode = signals::SensorF_OD_CAN1_lastErrorCode;
            using SensorF_OD_CAN1_RxErrCnt = signals::SensorF_OD_CAN1_RxErrCnt;
            using SensorF_OD_CAN1_TxErrCnt = signals::SensorF_OD_CAN1_TxErrCnt;
            using SensorF_OD_BuildTime = signals::SensorF_OD_BuildTime;
            using SensorF_OD_BuildDate = signals::SensorF_OD_BuildDate;
            using SensorF_OD_ChipUID2 = signals::SensorF_OD_ChipUID2;
            using SensorF_OD_ChipUID1 = signals::SensorF_OD_ChipUID1;
            using SensorF_OD_SdcOut = signals::SensorF_OD_SdcOut;
            using SensorF_OD_SdcIn = signals::SensorF_OD_SdcIn;
            using SensorF_OD_runtime = signals::SensorF_OD_runtime;
            using SensorF_OD_InputVoltage = signals::SensorF_OD_InputVoltage;
            using SensorF_OD_BoardTemp = signals::SensorF_OD_BoardTemp;
            using SensorF_OD_MemFree = signals::SensorF_OD_MemFree;
            using SensorF_OD_CpuUsage = signals::SensorF_OD_CpuUsage;
            using SensorF_OD_OdEntrySendInterval = signals::SensorF_OD_OdEntrySendInterval;
            using SensorF_OD_SendOdOnBootup = signals::SensorF_OD_SendOdOnBootup;
            using SensorF_OD_HeartbeatInterval = signals::SensorF_OD_HeartbeatInterval;
            using SensorF_OD_DbcVersion = signals::SensorF_OD_DbcVersion;
            using SensorF_OD_StackVersion = signals::SensorF_OD_StackVersion;
            using SensorF_OD_ProtocolVersion = signals::SensorF_OD_ProtocolVersion;
            using SensorF_OD_NodeStatus = signals::SensorF_OD_NodeStatus;
            using SensorF_OD_NodeID = signals::SensorF_OD_NodeID;

            // Attributes of message 'SensorF_SDO_Req_Up'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_SDO_Req_Down {
            public:
            constexpr static uint32_t id = 0x601;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_SDO_ID = signals::SensorF_SDO_ID;

            // Attributes of message 'SensorF_SDO_Req_Down'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_Heartbeat {
            public:
            constexpr static uint32_t id = 0x701;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_NodeState = signals::SensorF_NodeState;

            // Attributes of message 'SensorF_Heartbeat'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_BTL_TX {
            public:
            constexpr static uint32_t id = 0x741;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_BTL_TX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorF_BTL_RX {
            public:
            constexpr static uint32_t id = 0x781;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'SensorF_BTL_RX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorR_RX_RunControl {
            public:
            constexpr static uint32_t id = 0x2C2;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorR_RX_LaunchComm = signals::SensorR_RX_LaunchComm;
            using SensorR_RX_Enable = signals::SensorR_RX_Enable;
            using SensorR_RX_ErrorReset = signals::SensorR_RX_ErrorReset;

            // Attributes of message 'SensorR_RX_RunControl'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_Status {
            public:
            constexpr static uint32_t id = 0x191;
            constexpr static uint8_t dlc = 3;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_TX_Status_ = signals::BrakeF_TX_Status;
            using BrakeF_TX_Enabled = signals::BrakeF_TX_Enabled;
            using BrakeF_TX_ErrorFlag = signals::BrakeF_TX_ErrorFlag;
            using BrakeF_TX_SDC_Input = signals::BrakeF_TX_SDC_Input;
            using BrakeF_TX_DeltaTime_Control = signals::BrakeF_TX_DeltaTime_Control;
            using BrakeF_TX_MaxDeltaTime_Control = signals::BrakeF_TX_MaxDeltaTime_Control;

            // Attributes of message 'BrakeF_TX_Status'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_RX_Control {
            public:
            constexpr static uint32_t id = 0x1D1;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_RX_ErrorReset = signals::BrakeF_RX_ErrorReset;
            using BrakeF_RX_Enable = signals::BrakeF_RX_Enable;
            using BrakeF_RX_Engage = signals::BrakeF_RX_Engage;

            // Attributes of message 'BrakeF_RX_Control'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_Heartbeat {
            public:
            constexpr static uint32_t id = 0x711;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_NodeState = signals::BrakeF_NodeState;

            // Attributes of message 'BrakeF_Heartbeat'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeR_RX_Control {
            public:
            constexpr static uint32_t id = 0x1D2;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeR_RX_ErrorReset = signals::BrakeR_RX_ErrorReset;
            using BrakeR_RX_Enable = signals::BrakeR_RX_Enable;
            using BrakeR_RX_Engage = signals::BrakeR_RX_Engage;

            // Attributes of message 'BrakeR_RX_Control'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_Status {
            public:
            constexpr static uint32_t id = 0x19A;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_TX_Enabled = signals::PDU_TX_Enabled;
            using PDU_TX_ErrorFlag = signals::PDU_TX_ErrorFlag;
            using PDU_TX_PEHWEnabled = signals::PDU_TX_PEHWEnabled;

            // Attributes of message 'PDU_TX_Status'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_Control {
            public:
            constexpr static uint32_t id = 0x1DA;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_RX_Enable = signals::PDU_RX_Enable;
            using PDU_RX_ErrorReset = signals::PDU_RX_ErrorReset;
            using PDU_RX_PEHWEnable = signals::PDU_RX_PEHWEnable;

            // Attributes of message 'PDU_RX_Control'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_HP_Current {
            public:
            constexpr static uint32_t id = 0x21A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_HPCh1_Current = signals::PDU_HPCh1_Current;
            using PDU_HPCh2_Current = signals::PDU_HPCh2_Current;
            using PDU_HPCh3_Current = signals::PDU_HPCh3_Current;
            using PDU_HPCh4_Current = signals::PDU_HPCh4_Current;

            // Attributes of message 'PDU_TX_HP_Current'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_LP_Dutycycle {
            public:
            constexpr static uint32_t id = 0x25A;
            constexpr static uint8_t dlc = 6;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LPCh1_Dutycycle = signals::PDU_LPCh1_Dutycycle;
            using PDU_LPCh10_Dutycycle = signals::PDU_LPCh10_Dutycycle;
            using PDU_LPCh2_Dutycycle = signals::PDU_LPCh2_Dutycycle;
            using PDU_LPCh3_Dutycycle = signals::PDU_LPCh3_Dutycycle;
            using PDU_LPCh8_Dutycycle = signals::PDU_LPCh8_Dutycycle;
            using PDU_LPCh9_Dutycycle = signals::PDU_LPCh9_Dutycycle;

            // Attributes of message 'PDU_RX_LP_Dutycycle'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_HP_Dutycycle {
            public:
            constexpr static uint32_t id = 0x2DA;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_HPCh1_Dutycycle = signals::PDU_HPCh1_Dutycycle;
            using PDU_HPCh2_Dutycycle = signals::PDU_HPCh2_Dutycycle;
            using PDU_HPCh3_Dutycycle = signals::PDU_HPCh3_Dutycycle;
            using PDU_HPCh4_Dutycycle = signals::PDU_HPCh4_Dutycycle;

            // Attributes of message 'PDU_RX_HP_Dutycycle'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_LP_Current1 {
            public:
            constexpr static uint32_t id = 0x51A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LPCh1_Current = signals::PDU_LPCh1_Current;
            using PDU_LPCh2_Current = signals::PDU_LPCh2_Current;
            using PDU_LPCh3_Current = signals::PDU_LPCh3_Current;
            using PDU_LPCh4_Current = signals::PDU_LPCh4_Current;
            using PDU_LPCh5_Current = signals::PDU_LPCh5_Current;

            // Attributes of message 'PDU_TX_LP_Current1'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_LP_Current2 {
            public:
            constexpr static uint32_t id = 0x55A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LPCh10_Current = signals::PDU_LPCh10_Current;
            using PDU_LPCh6_Current = signals::PDU_LPCh6_Current;
            using PDU_LPCh7_Current = signals::PDU_LPCh7_Current;
            using PDU_LPCh8_Current = signals::PDU_LPCh8_Current;
            using PDU_LPCh9_Current = signals::PDU_LPCh9_Current;

            // Attributes of message 'PDU_TX_LP_Current2'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_Heartbeat {
            public:
            constexpr static uint32_t id = 0x71A;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_NodeState = signals::PDU_NodeState;

            // Attributes of message 'PDU_Heartbeat'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class HVCU_RX_Control {
            public:
            constexpr static uint32_t id = 0x1CA;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using HVCU_RX_Enable = signals::HVCU_RX_Enable;
            using HVCU_RX_Activate = signals::HVCU_RX_Activate;
            using HVCU_RX_ErrorReset = signals::HVCU_RX_ErrorReset;
            using HVCU_RX_Charging = signals::HVCU_RX_Charging;

            // Attributes of message 'HVCU_RX_Control'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class TelemetryNode_Heartbeat {
            public:
            constexpr static uint32_t id = 0x722;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using TelemetryNode_NodeState = signals::TelemetryNode_NodeState;

            // Attributes of message 'TelemetryNode_Heartbeat'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class CLU_Heartbeat {
            public:
            constexpr static uint32_t id = 0x714;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using CLU_NodeState = signals::CLU_NodeState;

            // Attributes of message 'CLU_Heartbeat'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };

        // Aliases for all CANzero messages of the current node, excluding the node name in the message name
        // This allows writing node independent code by using e.g. can::messages::CANZERO_BTL_RX instead of can::messages::PDU_BTL_RX
        // Also if the user renamed a PDO (e.g. PDU_TX_PDO1 to PDU_TX_Current) the alias will still be TX_PDO1
        using CANZERO_EMCY = SensorF_EMCY;
        using CANZERO_TX_PDO1 = SensorF_TX_StatePod;
        using CANZERO_RX_PDO1 = SensorF_RX_PDO1;
        using CANZERO_TX_PDO2 = SensorF_TX_FSMTransitions;
        using CANZERO_RX_PDO2 = SensorF_RX_PDO2;
        using CANZERO_TX_PDO3 = SensorF_TX_PDO3;
        using CANZERO_RX_PDO3 = SensorF_RX_PDO3;
        using CANZERO_TX_PDO4 = SensorF_TX_PDO4;
        using CANZERO_RX_PDO4 = SensorF_RX_PDO4;
        using CANZERO_TX_PDO5 = SensorF_TX_PDO5;
        using CANZERO_TX_PDO6 = SensorF_TX_PDO6;
        using CANZERO_TX_PDO7 = SensorF_TX_EncoderFront;
        using CANZERO_TX_PDO8 = SensorF_TX_BMS;
        using CANZERO_TX_PDO9 = SensorF_TX_Temperature;
        using CANZERO_TX_PDO10 = SensorF_TX_AccFront;
        using CANZERO_TX_PDO11 = SensorF_TX_PDO11;
        using CANZERO_TX_PDO12 = SensorF_TX_PDO12;
        using CANZERO_SDO_Resp = SensorF_SDO_Resp;
        using CANZERO_SDO_Req_Up = SensorF_SDO_Req_Up;
        using CANZERO_SDO_Req_Down = SensorF_SDO_Req_Down;
        using CANZERO_Heartbeat = SensorF_Heartbeat;
        using CANZERO_BTL_TX = SensorF_BTL_TX;
        using CANZERO_BTL_RX = SensorF_BTL_RX;
        
    }
}

#endif // DBCPARSER_DBC_PARSER_HPP