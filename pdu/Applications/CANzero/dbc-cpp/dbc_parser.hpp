/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY DBC2CPP V1.7.7.
 * 
 * This header file was generated from 'pod2023_gen.dbc' on 16:26:22 15.06.2023.
 * It contains all messages and signals as well as value tables and attributes of the DBC file.
 * Only messages and signals received or sent from node 'PDU' were parsed.
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
        constexpr uint8_t num_ext = 0;      // Number of used receive filters for extended (29-bit) ID messages
        constexpr uint32_t mask_ext[1] = {   // Filter mask for extended (29-bit) ID messages
        };
        constexpr uint32_t id_ext[1] = {     // Filter ID for extended (29-bit) ID messages
        };

        constexpr uint8_t num_std = 16;      // Number of used receive filters for standard (11-bit) ID messages
        constexpr uint32_t mask_std[16] = {   // Filter mask for standard (11-bit) ID messages
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF 
        };
        constexpr uint32_t id_std[16] = {     // Filter ID for standard (11-bit) ID messages
            0x10A,            0x002,            0x181,            0x441, 
            0x481,            0x202,            0x282,            0x482, 
            0x582,            0x1DA,            0x25A,            0x2DA, 
            0x35A,            0x5DA,            0x61A,            0x79A 
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
    struct SignedConverter20Bits {
        int32_t value : 20;
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
    constexpr char BusType[] = "CAN";
    constexpr char CANzero_ProtocolVersion[] = "V1.0";
    constexpr uint32_t CANzero_DBCVersion = 205;
    constexpr char CANzero_SDOClientName[] = "TelemetryNode";
    constexpr char CANzero_NMTMasterName[] = "Master";
    constexpr char DBName[] = "pod2022";
    
    /**********************************************************************************************
    * Namespace containing all signals with their value tables and attributes                     *
    ***********************************************************************************************/
    namespace signals {
        class PDU_RX_LPCh4_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x10A };
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
            constexpr static uint32_t ids[] = { 0x10A };
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
            constexpr static uint32_t ids[] = { 0x10A };
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
            constexpr static uint32_t ids[] = { 0x10A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
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
        class SensorR_TX_RunState {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x202 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0x3ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x3ull));
                return value;
            }

            // Value table of signal 'SensorR_TX_RunState'
            constexpr static uint8_t LAUNCHING = 0;
            constexpr static uint8_t READY_TO_LAUNCH = 1;
            constexpr static uint8_t FINISHED = 2;
            constexpr static uint8_t LAUNCH_ABORT = 3;
            constexpr static uint8_t IDLE_NOT_READY = 4;
        };
        class SensorR_TX_TrajectorySet {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x202 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0x10ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10ull) >> 4);
                return value;
            }

            // Value table of signal 'SensorR_TX_TrajectorySet'
            constexpr static bool NOT_SET = 0;
            constexpr static bool SET = 1;
            constexpr static bool INVALID_SET = 2;
        };
        class SensorR_TX_Enabled {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x202 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 5) & 0x20ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20ull) >> 5);
                return value;
            }

            // Value table of signal 'SensorR_TX_Enabled'
            constexpr static bool DISABLED = 0;
            constexpr static bool ENABLED = 1;
        };
        class SensorR_TX_ErrorFlag {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x202 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 6) & 0x40ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40ull) >> 6);
                return value;
            }

            // Value table of signal 'SensorR_TX_ErrorFlag'
            constexpr static bool NO_ERROR = 0;
            constexpr static bool ERROR = 1;
        };
        class SensorR_TX_Acc {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x282 };
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
        class SensorR_TX_Vel {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x282 };
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
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int16_t value = static_cast<int16_t>((intel & 0xFFFF0000ull) >> 16);
                // Convert raw bits to signed value
                SignedConverter16Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.002f);
            }
        };
        class SensorR_TX_Pos {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x282 };
            constexpr static float min = static_cast<float>(-524.288);
            constexpr static float max = static_cast<float>(524.287);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int32_t rawValue = static_cast<int32_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFFFFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int32_t value = static_cast<int32_t>((intel & 0xFFFFF00000000ull) >> 32);
                // Convert raw bits to signed value
                SignedConverter20Bits signedConverter{value};
                value = signedConverter.value;
                return value * (0.001f);
            }
        };
        class SensorR_LIMT_Coil_1 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x482 };
            constexpr static float min = static_cast<float>(-5);
            constexpr static float max = static_cast<float>(199.7);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-5.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0x7FFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0x7FFull));
                return value * (0.1f) + (-5.0f);
            }

            // Attributes of signal 'SensorR_LIMT_Coil_1'
            constexpr static float GenSigStartValue = 50.0f;
        };
        class SensorR_LIMT_Coil_2 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x482 };
            constexpr static float min = static_cast<float>(-5);
            constexpr static float max = static_cast<float>(199.7);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-5.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x7FF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0x7FF0000ull) >> 16);
                return value * (0.1f) + (-5.0f);
            }

            // Attributes of signal 'SensorR_LIMT_Coil_2'
            constexpr static float GenSigStartValue = 50.0f;
        };
        class SensorR_LIMT_Coil_3 {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x482 };
            constexpr static float min = static_cast<float>(-5);
            constexpr static float max = static_cast<float>(199.7);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-5.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0x7FF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0x7FF00000000ull) >> 32);
                return value * (0.1f) + (-5.0f);
            }

            // Attributes of signal 'SensorR_LIMT_Coil_3'
            constexpr static float GenSigStartValue = 50.0f;
        };
        class SensorR_SDO_ID {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
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

            // Value table of signal 'SensorR_SDO_ID'
            constexpr static uint16_t RESERVED = 0;
            constexpr static uint16_t NODEID = 1;
            constexpr static uint16_t NODESTATUS = 2;
            constexpr static uint16_t PROTOCOLVERSION = 3;
            constexpr static uint16_t STACKVERSION = 4;
            constexpr static uint16_t DBCVERSION = 5;
            constexpr static uint16_t HEARTBEATINTERVAL = 16;
            constexpr static uint16_t SENDODONBOOTUP = 32;
            constexpr static uint16_t ODENTRYSENDINTERVAL = 33;
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
            constexpr static uint16_t ENCODERWHEELDIAMETER = 2576;
            constexpr static uint16_t ENCODERRESETPOSITION = 2586;
            constexpr static uint16_t IMU_NUMBER = 2592;
            constexpr static uint16_t IMU1_TEMPERATURE = 2597;
            constexpr static uint16_t IMU2_TEMPERATURE = 2598;
            constexpr static uint16_t IMU3_TEMPERATURE = 2599;
            constexpr static uint16_t EXECTIMEOVERALL = 2608;
            constexpr static uint16_t EXECTIMESTATEEST = 2609;
            constexpr static uint16_t EXECTIMECONTROL = 2610;
            constexpr static uint16_t EXECTIMEREADSENSORS = 2611;
            constexpr static uint16_t OPCKP = 2624;
            constexpr static uint16_t OPCKI = 2625;
            constexpr static uint16_t CTRLMODE = 2626;
            constexpr static uint16_t CURRENTREQ = 2627;
            constexpr static uint16_t PODMASS = 2628;
            constexpr static uint16_t FMAXCTRL = 2629;
            constexpr static uint16_t FMINCTRL = 2630;
            constexpr static uint16_t MAXCTRLERROR = 2631;
            constexpr static uint16_t MAXCTRLERRORDURATION = 2632;
            constexpr static uint16_t WARNINGTHRESHHOLDFORLIMMODELOUTOFBOUNDARIES = 2633;
            constexpr static uint16_t MAXREQUIREDVELOCITY = 2634;
            constexpr static uint16_t VDCREDUCEFACTOR = 2635;
            constexpr static uint16_t RUNMODE = 2636;
            constexpr static uint16_t SETFINISH = 2637;
            constexpr static uint16_t RUNPROFILE_T0 = 2816;
            constexpr static uint16_t RUNPROFILE_V0 = 2817;
            constexpr static uint16_t RUNPROFILE_T1 = 2818;
            constexpr static uint16_t RUNPROFILE_V1 = 2819;
            constexpr static uint16_t RUNPROFILE_T2 = 2820;
            constexpr static uint16_t RUNPROFILE_V2 = 2821;
            constexpr static uint16_t RUNPROFILE_T3 = 2822;
            constexpr static uint16_t RUNPROFILE_V3 = 2823;
            constexpr static uint16_t RUNPROFILE_T4 = 2824;
            constexpr static uint16_t RUNPROFILE_V4 = 2825;
            constexpr static uint16_t ESTIMATEDRUNLENGTH = 2826;
            constexpr static uint16_t TC_I = 3072;
            constexpr static uint16_t TC_F = 3073;
            constexpr static uint16_t VELOCITYCONTROLLER = 3074;
            constexpr static uint16_t CVOPCIMAX = 3075;
            constexpr static uint16_t CVOPCTCF = 3076;
            constexpr static uint16_t CVOPCTCI = 3077;
            constexpr static uint16_t CVOPCSMOOTHINGDELTAVELOCITY = 3078;
            constexpr static uint16_t CVOPCDELTAVELHIGHERCURRENT = 3079;
            constexpr static uint16_t CVOPCREDUCECURRENTWHILECRUISING = 3080;
            constexpr static uint16_t CVOPCHYSTERESEF = 3081;
            constexpr static uint16_t ACCATCURRENTMEASURED = 3088;
            constexpr static uint16_t CURRENTFORCONSTANTVELOCITY = 3089;
            constexpr static uint16_t CASCADEDMAXSETFORCEDELTA = 3090;
            constexpr static uint16_t CASCADEDFREQCURUPDATE = 3091;
            constexpr static uint16_t CASCADEDKPINCREASECURRENT = 3092;
            constexpr static uint16_t CASCADEDKPDECREASECURRENT = 3093;
            constexpr static uint16_t CVOPCIMIN = 3094;
            constexpr static uint16_t CVOPCTCFREQ = 3095;
            constexpr static uint16_t ADAPTMODELBYCURRENT = 3096;
            constexpr static uint16_t CURRENT4STOPPING = 3097;
            constexpr static uint16_t FINISHEDDELAY = 3104;
            constexpr static uint16_t MAXRUNDISTANCE = 3105;
            constexpr static uint16_t OPC_RATELIMITER_F = 3106;
            constexpr static uint16_t DEBUGCASCADEDIREQ = 3107;
            constexpr static uint16_t DEBUGCVOPCFREQ = 3108;
            constexpr static uint16_t OPC_BRAKEFLAG = 3109;
            constexpr static uint16_t OPC_T_RUN_MAX = 3110;
            constexpr static uint16_t OPC_F_MAX = 3111;
            constexpr static uint16_t OPC_T_ACC = 3112;
            constexpr static uint16_t OPC_V_MAX = 3113;
            constexpr static uint16_t OPC_V_THRESHOLD = 3120;
            constexpr static uint16_t OPC_F_DECREASE = 3121;
            constexpr static uint16_t OPC_LOWPASS_TCF = 3122;
            constexpr static uint16_t OPC_LOWPASS_TCI = 3123;
            constexpr static uint16_t STRIPEMODE = 3329;
            constexpr static uint16_t REFLECTORDISTANCE = 3330;
            constexpr static uint16_t POSITIONGLITCH = 3331;
            constexpr static uint16_t ENABLESIGNALGENERATION = 3332;
            constexpr static uint16_t ACCDEVIATION = 3585;
            constexpr static uint16_t VELDEVIATIONENCODER = 3586;
            constexpr static uint16_t VELDEVIATIONOPTIC = 3587;
            constexpr static uint16_t POSDEVIATIONENCODER = 3588;
            constexpr static uint16_t POSDEVIATIONOPTIC = 3589;
        };
        class SensorR_SDO_RespCode {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }

            // Value table of signal 'SensorR_SDO_RespCode'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t ERR_NON_EXISTING_OBJECT = 1;
            constexpr static uint8_t ERR_WRITE_ONLY_OBJECT = 2;
            constexpr static uint8_t ERR_READ_ONLY_OBJECT = 3;
            constexpr static uint8_t ERR_NO_ACCESS_IN_THIS_STATE = 4;
            constexpr static uint8_t ERR_OUT_OF_RANGE = 5;
        };
        class SensorR_OD_posDeviationOptic {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3589            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3589);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3589) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_posDeviationOptic'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 40.0f;
            constexpr static float CANzero_SDO_Default = 4.0f;
        };
        class SensorR_OD_posDeviationEncoder {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3588            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3588);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3588) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_posDeviationEncoder'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 40.0f;
            constexpr static float CANzero_SDO_Default = 4.0f;
        };
        class SensorR_OD_velDeviationOptic {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3587            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3587);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3587) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_velDeviationOptic'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 20.0f;
            constexpr static float CANzero_SDO_Default = 2.0f;
        };
        class SensorR_OD_velDeviationEncoder {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3586            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3586);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3586) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_velDeviationEncoder'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 20.0f;
            constexpr static float CANzero_SDO_Default = 2.0f;
        };
        class SensorR_OD_accDeviation {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3585            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3585);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3585) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_accDeviation'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 40.0f;
            constexpr static float CANzero_SDO_Default = 4.0f;
        };
        class SensorR_OD_EnableSignalGeneration {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3332            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3332);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3332) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_EnableSignalGeneration'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_EnableSignalGeneration";
        };
        class SensorR_OD_PositionGlitch {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3331            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3331);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3331) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_PositionGlitch'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 5.0f;
        };
        class SensorR_OD_ReflectorDistance {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3330            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3330);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3330) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_ReflectorDistance'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 200.0f;
            constexpr static float CANzero_SDO_Default = 20.0f;
        };
        class SensorR_OD_StripeMode {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3329            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3329);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3329) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_StripeMode'
            constexpr static uint8_t LEFT = 0;
            constexpr static uint8_t RIGHT = 1;
            constexpr static uint8_t BOTH = 2;

            // Attributes of signal 'SensorR_OD_StripeMode'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_opc_lowpass_TcI {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3123            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3123);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3123) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_opc_lowpass_TcI'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2.0f;
            constexpr static float CANzero_SDO_Default = 0.02f;
        };
        class SensorR_OD_opc_lowpass_Tcf {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3122            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3122);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3122) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_opc_lowpass_Tcf'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2.0f;
            constexpr static float CANzero_SDO_Default = 0.02f;
        };
        class SensorR_OD_opc_F_decrease {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3121            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3121);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3121) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_opc_F_decrease'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 25.0f;
            constexpr static float CANzero_SDO_Default = 0.25f;
        };
        class SensorR_OD_opc_v_threshold {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3120            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3120);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3120) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_opc_v_threshold'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 25.0f;
            constexpr static float CANzero_SDO_Default = 0.25f;
        };
        class SensorR_OD_opc_v_max {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3113            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3113);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3113) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_opc_v_max'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 200.0f;
            constexpr static float CANzero_SDO_Default = 2.0f;
        };
        class SensorR_OD_opc_T_acc {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3112            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3112);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3112) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_opc_T_acc'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 20.0f;
            constexpr static float CANzero_SDO_Default = 2.0f;
        };
        class SensorR_OD_opc_F_max {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3111            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3111);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3111) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_opc_F_max'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 90.0f;
            constexpr static float CANzero_SDO_Default = 90.0f;
        };
        class SensorR_OD_opc_t_run_max {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3110            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3110);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3110) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_opc_t_run_max'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 40.0f;
            constexpr static float CANzero_SDO_Default = 4.0f;
        };
        class SensorR_OD_opc_brakeFlag {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3109            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3109);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3109) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_opc_brakeFlag'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t BRAKED = 1;

            // Attributes of signal 'SensorR_OD_opc_brakeFlag'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_DebugCVOPCfreq {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3108            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-3200);
            constexpr static float max = static_cast<float>(3353.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3108);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-3200.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3108) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f) + (-3200.0f);
            }

            // Attributes of signal 'SensorR_OD_DebugCVOPCfreq'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_DebugCascadedIreq {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3107            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-3200);
            constexpr static float max = static_cast<float>(3353.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3107);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-3200.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3107) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f) + (-3200.0f);
            }

            // Attributes of signal 'SensorR_OD_DebugCascadedIreq'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_opc_ratelimiter_f {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3106            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-3200);
            constexpr static float max = static_cast<float>(3353.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3106);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-3200.0f)) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3106) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f) + (-3200.0f);
            }

            // Attributes of signal 'SensorR_OD_opc_ratelimiter_f'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 37000.0f;
            constexpr static float CANzero_SDO_Default = 500.0f;
        };
        class SensorR_OD_maxRunDistance {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3105            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-1600);
            constexpr static float max = static_cast<float>(1676.75);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3105);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-1600.0f)) / (0.05f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3105) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.05f) + (-1600.0f);
            }

            // Attributes of signal 'SensorR_OD_maxRunDistance'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 38000.0f;
            constexpr static float CANzero_SDO_Default = 300.0f;
        };
        class SensorR_OD_FinishedDelay {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3104            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3104);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3104) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_FinishedDelay'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2500.0f;
            constexpr static float CANzero_SDO_Default = 2500.0f;
        };
        class SensorR_OD_Current4Stopping {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3097            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3097);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3097) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_Current4Stopping'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 500.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class SensorR_OD_AdaptModelByCurrent {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3096            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3096);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3096) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_AdaptModelByCurrent'
            constexpr static uint8_t CONST = 0;
            constexpr static uint8_t USECURRENT = 1;

            // Attributes of signal 'SensorR_OD_AdaptModelByCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CVOPCTcFreq {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3095            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3095);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3095) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CVOPCTcFreq'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 100.0f;
            constexpr static float CANzero_SDO_Default = 100.0f;
        };
        class SensorR_OD_CVOPCImin {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3094            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3094);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3094) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CVOPCImin'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class SensorR_OD_CascadedKpDecreaseCurrent {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3093            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3093);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3093) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_CascadedKpDecreaseCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 400.0f;
            constexpr static float CANzero_SDO_Default = 40.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CascadedKpDecreaseCurrent";
        };
        class SensorR_OD_CascadedKpIncreaseCurrent {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3092            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3092);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3092) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_CascadedKpIncreaseCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 400.0f;
            constexpr static float CANzero_SDO_Default = 40.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CascadedKpIncreaseCurrent";
        };
        class SensorR_OD_CascadedFreqCurUpdate {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3091            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3091);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3091) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorR_OD_CascadedFreqCurUpdate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 100.0f;
            constexpr static float CANzero_SDO_Default = 0.1f;
        };
        class SensorR_OD_CascadedMaxSetForceDelta {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3090            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3090);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3090) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_CascadedMaxSetForceDelta'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 100.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CascadedMaxSetForceDelta";
        };
        class SensorR_OD_CurrentForConstantVelocity {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3089            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3089);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3089) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_CurrentForConstantVelocity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 8000.0f;
            constexpr static float CANzero_SDO_Default = 80.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CurrentForConstantVelocity";
        };
        class SensorR_OD_AccAtCurrentMeasured {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3088            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3088);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3088) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_AccAtCurrentMeasured'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 7700.0f;
            constexpr static float CANzero_SDO_Default = 77.0f;
        };
        class SensorR_OD_CVOPCHystereseF {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3081            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3081);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3081) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_CVOPCHystereseF'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 4000.0f;
            constexpr static float CANzero_SDO_Default = 400.0f;
        };
        class SensorR_OD_CVOPCReduceCurrentWhileCruising {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3080            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3080);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3080) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_CVOPCReduceCurrentWhileCruising'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 90.0f;
            constexpr static float CANzero_SDO_Default = 0.9f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CVOPCReduceCurrentWhileCruising";
        };
        class SensorR_OD_CVOPCDeltaVelHigherCurrent {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3079            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3079);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3079) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_CVOPCDeltaVelHigherCurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 30.0f;
            constexpr static float CANzero_SDO_Default = 0.3f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CVOPCDeltaVelHigherCurrent";
        };
        class SensorR_OD_CVOPCSmoothingDeltaVelocity {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3078            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 3078);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3078) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_CVOPCSmoothingDeltaVelocity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 5.0f;
            constexpr static float CANzero_SDO_Default = 0.05f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CVOPCSmoothingDeltaVelocity";
        };
        class SensorR_OD_CVOPCTcI {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3077            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3077);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3077) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CVOPCTcI'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 300.0f;
            constexpr static float CANzero_SDO_Default = 300.0f;
        };
        class SensorR_OD_CVOPCTcf {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3076            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3076);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3076) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CVOPCTcf'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorR_OD_CVOPCImax {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3075            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3075);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3075) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CVOPCImax'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 200.0f;
            constexpr static float CANzero_SDO_Default = 200.0f;
        };
        class SensorR_OD_VelocityController {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3074            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3074);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3074) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_VelocityController'
            constexpr static uint8_t CC = 0;
            constexpr static uint8_t CVFF = 1;
            constexpr static uint8_t CVFB = 2;
            constexpr static uint8_t CAS = 3;
            constexpr static uint8_t FAST = 4;

            // Attributes of signal 'SensorR_OD_VelocityController'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_TC_f {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3073            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3073);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3073) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_TC_f'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 150.0f;
            constexpr static float CANzero_SDO_Default = 150.0f;
        };
        class SensorR_OD_TC_I {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3072            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3072);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3072) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_TC_I'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 150.0f;
            constexpr static float CANzero_SDO_Default = 150.0f;
        };
        class SensorR_OD_EstimatedRunLength {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2826            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-1600);
            constexpr static float max = static_cast<float>(1676.75);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2826);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-1600.0f)) / (0.05f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2826) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.05f) + (-1600.0f);
            }

            // Attributes of signal 'SensorR_OD_EstimatedRunLength'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_RunProfile_V4 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2825            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-327);
            constexpr static float max = static_cast<float>(328.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2825);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-327.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2825) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-327.0f);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_V4'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32700.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_RunProfile_T4 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2824            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(655350);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2824);
                uint32_t rawValue = static_cast<uint32_t>((value) / (10));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2824) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (10);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_T4'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 450.0f;
            constexpr static float CANzero_SDO_Default = 4500.0f;
        };
        class SensorR_OD_RunProfile_V3 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2823            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-327);
            constexpr static float max = static_cast<float>(328.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2823);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-327.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2823) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-327.0f);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_V3'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32800.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_RunProfile_T3 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2822            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(655350);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2822);
                uint32_t rawValue = static_cast<uint32_t>((value) / (10));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2822) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (10);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_T3'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 350.0f;
            constexpr static float CANzero_SDO_Default = 3500.0f;
        };
        class SensorR_OD_RunProfile_V2 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2821            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-327);
            constexpr static float max = static_cast<float>(328.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2821);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-327.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2821) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-327.0f);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_V2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32800.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_RunProfile_T2 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2820            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(655350);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2820);
                uint32_t rawValue = static_cast<uint32_t>((value) / (10));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2820) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (10);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_T2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 300.0f;
            constexpr static float CANzero_SDO_Default = 3000.0f;
        };
        class SensorR_OD_RunProfile_V1 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2819            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-327);
            constexpr static float max = static_cast<float>(328.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2819);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-327.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2819) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-327.0f);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_V1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32705.0f;
            constexpr static float CANzero_SDO_Default = 0.05f;
        };
        class SensorR_OD_RunProfile_T1 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2818            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(655350);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2818);
                uint32_t rawValue = static_cast<uint32_t>((value) / (10));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2818) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (10);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_T1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 100.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorR_OD_RunProfile_V0 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2817            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-327);
            constexpr static float max = static_cast<float>(328.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2817);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-327.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2817) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-327.0f);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_V0'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 32700.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_RunProfile_T0 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2816            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(655350);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2816);
                uint32_t rawValue = static_cast<uint32_t>((value) / (10));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2816) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (10);
            }

            // Attributes of signal 'SensorR_OD_RunProfile_T0'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_SetFinish {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2637            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2637);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2637) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_SetFinish'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t FINISH = 1;

            // Attributes of signal 'SensorR_OD_SetFinish'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_RunMode {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2636            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2636);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2636) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_RunMode'
            constexpr static uint8_t DYNAMIC = 0;
            constexpr static uint8_t LEVITATION = 1;
            constexpr static uint8_t MANUAL = 2;

            // Attributes of signal 'SensorR_OD_RunMode'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_VdcReduceFactor {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2635            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2635);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2635) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_VdcReduceFactor'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 9.499999999999998f;
            constexpr static float CANzero_SDO_Default = 0.95f;
        };
        class SensorR_OD_MaxRequiredVelocity {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2634            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2634);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2634) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_MaxRequiredVelocity'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 190.0f;
            constexpr static float CANzero_SDO_Default = 19.0f;
        };
        class SensorR_OD_WarningThreshholdForLimModelOutOfBoundaries {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2633            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2633);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2633) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorR_OD_WarningThreshholdForLimModelOutOfBoundaries'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 900.0f;
            constexpr static float CANzero_SDO_Default = 0.9f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_WarningThreshholdForLimModelOutOfBoundaries";
        };
        class SensorR_OD_MaxCtrlErrorDuration {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2632            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2632);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2632) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_MaxCtrlErrorDuration'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 0.5f;
        };
        class SensorR_OD_MaxCtrlError {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2631            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2631);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2631) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_MaxCtrlError'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 150.0f;
            constexpr static float CANzero_SDO_Default = 1.5f;
        };
        class SensorR_OD_FminCtrl {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2630            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65535.0);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2630);
                uint16_t rawValue = static_cast<uint16_t>(value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2630) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_FminCtrl'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 922.0f;
            constexpr static float CANzero_SDO_Default = 922.0f;
        };
        class SensorR_OD_FmaxCtrl {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2629            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65535.0);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2629);
                uint16_t rawValue = static_cast<uint16_t>(value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2629) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_FmaxCtrl'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1004.0f;
            constexpr static float CANzero_SDO_Default = 1004.0f;
        };
        class SensorR_OD_PodMass {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2628            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2628);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2628) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_PodMass'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2300.0f;
            constexpr static float CANzero_SDO_Default = 230.0f;
        };
        class SensorR_OD_CurrentReq {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2627            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2627);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2627) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'SensorR_OD_CurrentReq'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1050.0f;
            constexpr static float CANzero_SDO_Default = 105.0f;
        };
        class SensorR_OD_CtrlMode {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2626            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2626);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2626) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CtrlMode'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_OpcKi {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2625            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2625);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2625) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_OpcKi'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 0.5f;
        };
        class SensorR_OD_OpcKp {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2624            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2624);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2624) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_OpcKp'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 600.0f;
            constexpr static float CANzero_SDO_Default = 6.0f;
        };
        class SensorR_OD_ExecTimeReadSensors {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2611            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2611);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2611) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_ExecTimeReadSensors'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ExecTimeControl {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2610            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2610);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2610) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_ExecTimeControl'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ExecTimeStateEst {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2609            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2609);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2609) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_ExecTimeStateEst'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ExecTimeOverall {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2608            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2608);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2608) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_ExecTimeOverall'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_IMU3_Temperature {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2599            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2599);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2599) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorR_OD_IMU3_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_IMU2_Temperature {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2598            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2598);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2598) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorR_OD_IMU2_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_IMU1_Temperature {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2597            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-100);
            constexpr static float max = static_cast<float>(555.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2597);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-100.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2597) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-100.0f);
            }

            // Attributes of signal 'SensorR_OD_IMU1_Temperature'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_IMU_number {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2592            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2592);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2592) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_IMU_number'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_EncoderResetPosition {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2586            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2586);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2586) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_EncoderResetPosition'
            constexpr static uint8_t NONE = 0;
            constexpr static uint8_t RESET = 1;

            // Attributes of signal 'SensorR_OD_EncoderResetPosition'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::WRITE_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_EncoderWheelDiameter {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2576            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(1);
            constexpr static float max = static_cast<float>(300);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2576);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2576) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.005f);
            }

            // Attributes of signal 'SensorR_OD_EncoderWheelDiameter'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 15000.0f;
            constexpr static float CANzero_SDO_Default = 75.0f;
        };
        class SensorR_OD_samplingInterval {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2048            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0.01);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 2048);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2048) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'SensorR_OD_samplingInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 10.0f;
        };
        class SensorR_OD_CAN2_DelayedTxMessages {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1129            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1129);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1129) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN2_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CAN2_DelayedTxMessages";
        };
        class SensorR_OD_CAN2_ErrorStatus {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1128            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1128);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1128) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN2_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'SensorR_OD_CAN2_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN2_DiscardedTxMessages {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1127            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1127);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1127) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN2_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CAN2_DiscardedTxMessages";
        };
        class SensorR_OD_CAN2_Status {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1126            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1126);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1126) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN2_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'SensorR_OD_CAN2_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN2_Baudrate {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1124            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1124);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1124) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN2_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorR_OD_CAN2_autoErrorReset {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1123            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1123);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1123) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN2_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorR_OD_CAN2_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_CAN2_lastErrorCode {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1122            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1122);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1122) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN2_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'SensorR_OD_CAN2_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN2_RxErrCnt {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1121            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1121);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1121) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN2_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN2_TxErrCnt {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1120            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1120);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1120) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN2_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN1_DelayedTxMessages {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1113            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1113);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1113) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN1_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CAN1_DelayedTxMessages";
        };
        class SensorR_OD_CAN1_ErrorStatus {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1112            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1112);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1112) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN1_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'SensorR_OD_CAN1_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN1_DiscardedTxMessages {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1111            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1111);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1111) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN1_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "SensorR_OD_CAN1_DiscardedTxMessages";
        };
        class SensorR_OD_CAN1_Status {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1110            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1110);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1110) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN1_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'SensorR_OD_CAN1_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN1_Baudrate {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1108            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1108);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1108) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN1_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class SensorR_OD_CAN1_autoErrorReset {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1107            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1107);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1107) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN1_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorR_OD_CAN1_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_CAN1_lastErrorCode {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1106            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1106);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1106) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_CAN1_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'SensorR_OD_CAN1_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN1_RxErrCnt {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1105            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1105);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1105) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN1_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CAN1_TxErrCnt {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1104            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1104);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1104) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_CAN1_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_BuildTime {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1073            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1073);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1073) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_BuildTime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_BuildDate {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1072            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1072);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1072) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_BuildDate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ChipUID2 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1057            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1057);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1057) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_ChipUID2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ChipUID1 {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1056            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1056);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1056) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_ChipUID1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_SdcOut {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1046            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1046);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1046) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_SdcOut'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_SdcIn {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1045            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1045);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1045) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_SdcIn'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_runtime {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1044            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1044);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1044) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_runtime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_InputVoltage {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1043            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1043);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1043) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'SensorR_OD_InputVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_BoardTemp {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1042            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(625.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1042);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1042) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-30.0f);
            }

            // Attributes of signal 'SensorR_OD_BoardTemp'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 3000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_MemFree {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1041            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(262140);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1041);
                uint32_t rawValue = static_cast<uint32_t>((value) / (4));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1041) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (4);
            }

            // Attributes of signal 'SensorR_OD_MemFree'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_CpuUsage {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1040            
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                SensorR_SDO_ID::set(intel, motorola, dlc, 1040);
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1040) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'SensorR_OD_CpuUsage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_OdEntrySendInterval {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 33            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 33);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 33) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_OdEntrySendInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class SensorR_OD_SendOdOnBootup {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 32            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 32);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 32) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_SendOdOnBootup'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'SensorR_OD_SendOdOnBootup'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_HeartbeatInterval {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 16            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 16);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 16) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_HeartbeatInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 250.0f;
            constexpr static float CANzero_SDO_Default = 250.0f;
        };
        class SensorR_OD_DbcVersion {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 5            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 5);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 5) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_DbcVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_StackVersion {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 4            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 4);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 4) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_StackVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_ProtocolVersion {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 3            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 3);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 3) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_ProtocolVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class SensorR_OD_NodeStatus {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 2            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 2);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 2) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'SensorR_OD_NodeStatus'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
            constexpr static uint8_t RESET = 128;

            // Attributes of signal 'SensorR_OD_NodeStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class SensorR_OD_NodeID {
            public:
            // This signal is multiplexed by SensorR_SDO_ID == 1            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x582 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                SensorR_SDO_ID::set(intel, motorola, dlc, 1);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (SensorR_SDO_ID::get(intel, motorola) != 1) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'SensorR_OD_NodeID'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_W0_OtherWarning {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'PDU_W0_OtherWarning'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class PDU_W1_batterVoltageLow {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'PDU_W1_batterVoltageLow'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class PDU_W2_batterTempHigh {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'PDU_W2_batterTempHigh'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class PDU_E0_OtherError {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0x1000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000ull) >> 12);
                return value;
            }

            // Value table of signal 'PDU_E0_OtherError'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class PDU_E1_batterVoltageCritical {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 13) & 0x2000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000ull) >> 13);
                return value;
            }

            // Value table of signal 'PDU_E1_batterVoltageCritical'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class PDU_E2_batteryOvercurrent {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 14) & 0x4000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000ull) >> 14);
                return value;
            }

            // Value table of signal 'PDU_E2_batteryOvercurrent'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class PDU_E3_batterTempCritical {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 15) & 0x8000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000ull) >> 15);
                return value;
            }

            // Value table of signal 'PDU_E3_batterTempCritical'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class PDU_E4_watchdogStateMachine {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x9A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x10000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_E4_watchdogStateMachine'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
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
        class PDU_HPCh1_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFull));
                return value;
            }
        };
        class PDU_HPCh1_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0x30ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x30ull) >> 4);
                return value;
            }
        };
        class PDU_HPCh2_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 6) & 0xC0ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC0ull) >> 6);
                return value;
            }
        };
        class PDU_HPCh2_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xF00ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF00ull) >> 8);
                return value;
            }
        };
        class PDU_HPCh3_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }
        };
        class PDU_HPCh3_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x30000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x30000ull) >> 16);
                return value;
            }
        };
        class PDU_HPCh4_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 18) & 0x3C0000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x3C0000ull) >> 18);
                return value;
            }
        };
        class PDU_HPCh4_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x29A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 22) & 0xC00000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC00000ull) >> 22);
                return value;
            }
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
        class PDU_D1_Dutycycle {
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

            // Attributes of signal 'PDU_D1_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_D2_Dutycycle {
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

            // Attributes of signal 'PDU_D2_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_D3_Dutycycle {
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
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF00000000ull) >> 32);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_D3_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_D4_Dutycycle {
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
                intel |= (static_cast<uint64_t>(rawValue) << 40) & 0xFF0000000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000000000ull) >> 40);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_D4_Dutycycle'
            constexpr static float GenSigStartValue = 200.0f;
        };
        class PDU_LPCh1_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }
        };
        class PDU_LPCh2_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }
        };
        class PDU_LPCh3_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }
        };
        class PDU_LPCh4_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }
        };
        class PDU_LPCh5_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0x10ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10ull) >> 4);
                return value;
            }
        };
        class PDU_LPCh6_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 5) & 0x20ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20ull) >> 5);
                return value;
            }
        };
        class PDU_LPCh7_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 6) & 0x40ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40ull) >> 6);
                return value;
            }
        };
        class PDU_LPCh8_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 7) & 0x80ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x80ull) >> 7);
                return value;
            }
        };
        class PDU_LPCh9_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0x100ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x100ull) >> 8);
                return value;
            }
        };
        class PDU_LPCh10_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 9) & 0x200ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x200ull) >> 9);
                return value;
            }
        };
        class PDU_HPCh1_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 10) & 0x400ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x400ull) >> 10);
                return value;
            }
        };
        class PDU_HPCh2_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 11) & 0x800ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x800ull) >> 11);
                return value;
            }
        };
        class PDU_HPCh3_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0x1000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000ull) >> 12);
                return value;
            }
        };
        class PDU_HPCh4_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 13) & 0x2000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000ull) >> 13);
                return value;
            }
        };
        class PDU_D1_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 14) & 0x4000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000ull) >> 14);
                return value;
            }
        };
        class PDU_D2_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 15) & 0x8000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000ull) >> 15);
                return value;
            }
        };
        class PDU_D3_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x10000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000ull) >> 16);
                return value;
            }
        };
        class PDU_D4_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 17) & 0x20000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20000ull) >> 17);
                return value;
            }
        };
        class PDU_SDC_Enable {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x35A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 18) & 0x40000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x40000ull) >> 18);
                return value;
            }
        };
        class PDU_LPCh10_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xFull));
                return value;
            }
        };
        class PDU_LPCh1_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 4) & 0xF0ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF0ull) >> 4);
                return value;
            }
        };
        class PDU_LPCh2_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0xF00ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF00ull) >> 8);
                return value;
            }
        };
        class PDU_LPCh3_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }
        };
        class PDU_LPCh4_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xF0000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF0000ull) >> 16);
                return value;
            }
        };
        class PDU_LPCh5_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 20) & 0xF00000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF00000ull) >> 20);
                return value;
            }
        };
        class PDU_LPCh6_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xF000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000000ull) >> 24);
                return value;
            }
        };
        class PDU_LPCh7_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 28) & 0xF0000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF0000000ull) >> 28);
                return value;
            }
        };
        class PDU_LPCh8_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xF00000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF00000000ull) >> 32);
                return value;
            }
        };
        class PDU_LPCh9_ShortCnt {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(15);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 36) & 0xF000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000000000ull) >> 36);
                return value;
            }
        };
        class PDU_LPCh10_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 40) & 0x30000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x30000000000ull) >> 40);
                return value;
            }
        };
        class PDU_LPCh1_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 42) & 0xC0000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC0000000000ull) >> 42);
                return value;
            }

            // Value table of signal 'PDU_LPCh1_State'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t STATUS_CHANGE_PENDING = 1;
            constexpr static uint8_t OUTPUT_SHORT_CIRCUIT = 2;
            constexpr static uint8_t EXTERNAL_VOLTAGE = 3;
        };
        class PDU_LPCh2_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 44) & 0x300000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x300000000000ull) >> 44);
                return value;
            }
        };
        class PDU_LPCh3_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 46) & 0xC00000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC00000000000ull) >> 46);
                return value;
            }
        };
        class PDU_LPCh4_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 48) & 0x3000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x3000000000000ull) >> 48);
                return value;
            }
        };
        class PDU_LPCh5_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 50) & 0xC000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC000000000000ull) >> 50);
                return value;
            }
        };
        class PDU_LPCh6_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 52) & 0x30000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x30000000000000ull) >> 52);
                return value;
            }
        };
        class PDU_LPCh7_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 54) & 0xC0000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC0000000000000ull) >> 54);
                return value;
            }
        };
        class PDU_LPCh8_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 56) & 0x300000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x300000000000000ull) >> 56);
                return value;
            }
        };
        class PDU_LPCh9_State {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x49A };
            constexpr static uint8_t min = static_cast<uint8_t>(0);
            constexpr static uint8_t max = static_cast<uint8_t>(3);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 58) & 0xC00000000000000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xC00000000000000ull) >> 58);
                return value;
            }
        };
        class PDU_LV_Current {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(40.95);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue)) & 0xFFFull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFull));
                return value * (0.01f);
            }
        };
        class PDU_LV_Voltage {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(40.95);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xFFF000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF000ull) >> 12);
                return value * (0.01f);
            }
        };
        class PDU_LV_SOC {
            public:
            using dataType = int8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x4DA };
            constexpr static int8_t min = static_cast<int8_t>(0);
            constexpr static int8_t max = static_cast<int8_t>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, int8_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                int8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 24) & 0xFF000000ull;
            }
            constexpr static inline int8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                int8_t value = static_cast<int8_t>((intel & 0xFF000000ull) >> 24);
                // Convert raw bits to signed value
                SignedConverter8Bits signedConverter{value};
                value = signedConverter.value;
                return value;
            }
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
        class PDU_SDO_ID {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 3;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA, 0x61A };
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

            // Value table of signal 'PDU_SDO_ID'
            constexpr static uint16_t RESERVED = 0;
            constexpr static uint16_t NODEID = 1;
            constexpr static uint16_t NODESTATUS = 2;
            constexpr static uint16_t PROTOCOLVERSION = 3;
            constexpr static uint16_t STACKVERSION = 4;
            constexpr static uint16_t DBCVERSION = 5;
            constexpr static uint16_t HEARTBEATINTERVAL = 16;
            constexpr static uint16_t SENDODONBOOTUP = 32;
            constexpr static uint16_t ODENTRYSENDINTERVAL = 33;
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
            constexpr static uint16_t BATTERVOLTAGELOW = 2048;
            constexpr static uint16_t BATTERVOLTAGECRITICAL = 2049;
            constexpr static uint16_t OVERTEMPWARN = 2050;
            constexpr static uint16_t OVERTEMPCRITICAL = 2051;
            constexpr static uint16_t BATTERYOVERCURRENT = 2128;
            constexpr static uint16_t CURRENTREADINTERVAL = 2304;
            constexpr static uint16_t STATUSSENDINTERVAL = 2305;
            constexpr static uint16_t WATCHDOGTIMEOUT = 2306;
            constexpr static uint16_t PROJECTXXENABLED = 2560;
            constexpr static uint16_t LEDCOMMANDS = 2562;
            constexpr static uint16_t COOLINGPUMPENABLED = 2816;
        };
        class PDU_SDO_RespCode {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x59A };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }

            // Value table of signal 'PDU_SDO_RespCode'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t ERR_NON_EXISTING_OBJECT = 1;
            constexpr static uint8_t ERR_WRITE_ONLY_OBJECT = 2;
            constexpr static uint8_t ERR_READ_ONLY_OBJECT = 3;
            constexpr static uint8_t ERR_NO_ACCESS_IN_THIS_STATE = 4;
            constexpr static uint8_t ERR_OUT_OF_RANGE = 5;
        };
        class PDU_OD_CoolingPumpEnabled {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2816            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2816);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2816) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CoolingPumpEnabled'
            constexpr static uint8_t DISABLE = 0;
            constexpr static uint8_t ENABLE = 1;

            // Attributes of signal 'PDU_OD_CoolingPumpEnabled'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_LedCommands {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2562            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2562);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2562) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_LedCommands'
            constexpr static uint16_t NONE = 0;
            constexpr static uint16_t WHIPEANIMATION = 1;
            constexpr static uint16_t MUGREENANIMATION = 2;
            constexpr static uint16_t RAINBOWANIMATION = 3;
            constexpr static uint16_t FLASHLIGHTS = 4;
            constexpr static uint16_t SNAKEANIMATION = 5;
            constexpr static uint16_t BLINKPINK = 6;
            constexpr static uint16_t MULTIWHIPE = 7;
            constexpr static uint16_t EMERGENCY = 8;

            // Attributes of signal 'PDU_OD_LedCommands'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_projectXXEnabled {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2560            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2560);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2560) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_projectXXEnabled'
            constexpr static uint8_t DISABLE = 0;
            constexpr static uint8_t ENABLE = 1;

            // Attributes of signal 'PDU_OD_projectXXEnabled'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_watchdogTimeout {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2306            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2306);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2306) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_watchdogTimeout'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 25.0f;
            constexpr static float CANzero_SDO_Default = 25.0f;
        };
        class PDU_OD_statusSendInterval {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2305            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2305);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2305) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_statusSendInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 20.0f;
            constexpr static float CANzero_SDO_Default = 20.0f;
        };
        class PDU_OD_currentReadInterval {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2304            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2304);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2304) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_currentReadInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 100.0f;
            constexpr static float CANzero_SDO_Default = 100.0f;
        };
        class PDU_OD_batteryOvercurrent {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2128            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 2128);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2128) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'PDU_OD_batteryOvercurrent'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1500.0f;
            constexpr static float CANzero_SDO_Default = 15.0f;
        };
        class PDU_OD_overTempCritical {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2051            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 2051);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2051) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'PDU_OD_overTempCritical'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 700.0f;
            constexpr static float CANzero_SDO_Default = 70.0f;
        };
        class PDU_OD_overTempWarn {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2050            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(6553.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 2050);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.1f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2050) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.1f);
            }

            // Attributes of signal 'PDU_OD_overTempWarn'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 400.0f;
            constexpr static float CANzero_SDO_Default = 40.0f;
        };
        class PDU_OD_batterVoltageCritical {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2049            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 2049);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2049) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'PDU_OD_batterVoltageCritical'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2100.0f;
            constexpr static float CANzero_SDO_Default = 21.0f;
        };
        class PDU_OD_batterVoltageLow {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2048            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(655.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 2048);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2048) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f);
            }

            // Attributes of signal 'PDU_OD_batterVoltageLow'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2220.0f;
            constexpr static float CANzero_SDO_Default = 22.2f;
        };
        class PDU_OD_CAN2_DelayedTxMessages {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1129            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1129);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1129) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN2_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_ErrorStatus {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1128            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1128);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1128) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN2_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'PDU_OD_CAN2_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_DiscardedTxMessages {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1127            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1127);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1127) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN2_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_Status {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1126            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1126);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1126) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN2_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'PDU_OD_CAN2_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_Baudrate {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1124            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1124);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1124) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN2_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class PDU_OD_CAN2_autoErrorReset {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1123            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1123);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1123) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN2_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'PDU_OD_CAN2_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_CAN2_lastErrorCode {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1122            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1122);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1122) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN2_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'PDU_OD_CAN2_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_RxErrCnt {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1121            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1121);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1121) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN2_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN2_TxErrCnt {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1120            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1120);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1120) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN2_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_DelayedTxMessages {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1113            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1113);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1113) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN1_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_ErrorStatus {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1112            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1112);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1112) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN1_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'PDU_OD_CAN1_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_DiscardedTxMessages {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1111            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1111);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1111) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN1_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_Status {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1110            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1110);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1110) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN1_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'PDU_OD_CAN1_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_Baudrate {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1108            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1108);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1108) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN1_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class PDU_OD_CAN1_autoErrorReset {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1107            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1107);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1107) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN1_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'PDU_OD_CAN1_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_CAN1_lastErrorCode {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1106            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1106);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1106) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_CAN1_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'PDU_OD_CAN1_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_RxErrCnt {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1105            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1105);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1105) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN1_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CAN1_TxErrCnt {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1104            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1104);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1104) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_CAN1_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_BuildTime {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1073            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1073);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1073) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_BuildTime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_BuildDate {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1072            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1072);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1072) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_BuildDate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_ChipUID2 {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1057            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1057);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1057) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_ChipUID2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_ChipUID1 {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1056            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1056);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1056) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_ChipUID1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_SdcOut {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1046            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1046);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1046) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_SdcOut'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_SdcIn {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1045            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1045);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1045) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_SdcIn'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_runtime {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1044            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1044);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1044) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_runtime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_InputVoltage {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1043            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1043);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1043) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'PDU_OD_InputVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_BoardTemp {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1042            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(625.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1042);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1042) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-30.0f);
            }

            // Attributes of signal 'PDU_OD_BoardTemp'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 3000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_MemFree {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1041            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(262140);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1041);
                uint32_t rawValue = static_cast<uint32_t>((value) / (4));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1041) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (4);
            }

            // Attributes of signal 'PDU_OD_MemFree'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_CpuUsage {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1040            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                PDU_SDO_ID::set(intel, motorola, dlc, 1040);
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1040) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'PDU_OD_CpuUsage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_OdEntrySendInterval {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 33            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 33);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 33) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_OdEntrySendInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class PDU_OD_SendOdOnBootup {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 32            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 32);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 32) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_SendOdOnBootup'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'PDU_OD_SendOdOnBootup'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_HeartbeatInterval {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 16            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 16);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 16) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_HeartbeatInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 250.0f;
            constexpr static float CANzero_SDO_Default = 250.0f;
        };
        class PDU_OD_DbcVersion {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 5            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 5);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 5) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_DbcVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_StackVersion {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 4            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 4);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 4) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_StackVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_ProtocolVersion {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 3            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 3);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 3) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_ProtocolVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class PDU_OD_NodeStatus {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 2            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 2);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 2) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'PDU_OD_NodeStatus'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
            constexpr static uint8_t RESET = 128;

            // Attributes of signal 'PDU_OD_NodeStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class PDU_OD_NodeID {
            public:
            // This signal is multiplexed by PDU_SDO_ID == 1            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x59A, 0x5DA };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                PDU_SDO_ID::set(intel, motorola, dlc, 1);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (PDU_SDO_ID::get(intel, motorola) != 1) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'PDU_OD_NodeID'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
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
    }

    /**********************************************************************************************
    * Namespace containing all messages                                                           *
    ***********************************************************************************************/
    namespace messages {
        class PDU_RX_LP_Enable {
            public:
            constexpr static uint32_t id = 0x10A;
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
        class SensorR_TX_RunStatus {
            public:
            constexpr static uint32_t id = 0x202;
            constexpr static uint8_t dlc = 1;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorR_TX_RunState = signals::SensorR_TX_RunState;
            using SensorR_TX_TrajectorySet = signals::SensorR_TX_TrajectorySet;
            using SensorR_TX_Enabled = signals::SensorR_TX_Enabled;
            using SensorR_TX_ErrorFlag = signals::SensorR_TX_ErrorFlag;

            // Attributes of message 'SensorR_TX_RunStatus'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorR_TX_SE {
            public:
            constexpr static uint32_t id = 0x282;
            constexpr static uint8_t dlc = 7;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorR_TX_Acc = signals::SensorR_TX_Acc;
            using SensorR_TX_Vel = signals::SensorR_TX_Vel;
            using SensorR_TX_Pos = signals::SensorR_TX_Pos;

            // Attributes of message 'SensorR_TX_SE'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorR_TX_Temperature {
            public:
            constexpr static uint32_t id = 0x482;
            constexpr static uint8_t dlc = 6;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorR_LIMT_Coil_1 = signals::SensorR_LIMT_Coil_1;
            using SensorR_LIMT_Coil_2 = signals::SensorR_LIMT_Coil_2;
            using SensorR_LIMT_Coil_3 = signals::SensorR_LIMT_Coil_3;

            // Attributes of message 'SensorR_TX_Temperature'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class SensorR_SDO_Resp {
            public:
            constexpr static uint32_t id = 0x582;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorR_SDO_ID = signals::SensorR_SDO_ID;
            using SensorR_SDO_RespCode = signals::SensorR_SDO_RespCode;
            using SensorR_OD_posDeviationOptic = signals::SensorR_OD_posDeviationOptic;
            using SensorR_OD_posDeviationEncoder = signals::SensorR_OD_posDeviationEncoder;
            using SensorR_OD_velDeviationOptic = signals::SensorR_OD_velDeviationOptic;
            using SensorR_OD_velDeviationEncoder = signals::SensorR_OD_velDeviationEncoder;
            using SensorR_OD_accDeviation = signals::SensorR_OD_accDeviation;
            using SensorR_OD_EnableSignalGeneration = signals::SensorR_OD_EnableSignalGeneration;
            using SensorR_OD_PositionGlitch = signals::SensorR_OD_PositionGlitch;
            using SensorR_OD_ReflectorDistance = signals::SensorR_OD_ReflectorDistance;
            using SensorR_OD_StripeMode = signals::SensorR_OD_StripeMode;
            using SensorR_OD_opc_lowpass_TcI = signals::SensorR_OD_opc_lowpass_TcI;
            using SensorR_OD_opc_lowpass_Tcf = signals::SensorR_OD_opc_lowpass_Tcf;
            using SensorR_OD_opc_F_decrease = signals::SensorR_OD_opc_F_decrease;
            using SensorR_OD_opc_v_threshold = signals::SensorR_OD_opc_v_threshold;
            using SensorR_OD_opc_v_max = signals::SensorR_OD_opc_v_max;
            using SensorR_OD_opc_T_acc = signals::SensorR_OD_opc_T_acc;
            using SensorR_OD_opc_F_max = signals::SensorR_OD_opc_F_max;
            using SensorR_OD_opc_t_run_max = signals::SensorR_OD_opc_t_run_max;
            using SensorR_OD_opc_brakeFlag = signals::SensorR_OD_opc_brakeFlag;
            using SensorR_OD_DebugCVOPCfreq = signals::SensorR_OD_DebugCVOPCfreq;
            using SensorR_OD_DebugCascadedIreq = signals::SensorR_OD_DebugCascadedIreq;
            using SensorR_OD_opc_ratelimiter_f = signals::SensorR_OD_opc_ratelimiter_f;
            using SensorR_OD_maxRunDistance = signals::SensorR_OD_maxRunDistance;
            using SensorR_OD_FinishedDelay = signals::SensorR_OD_FinishedDelay;
            using SensorR_OD_Current4Stopping = signals::SensorR_OD_Current4Stopping;
            using SensorR_OD_AdaptModelByCurrent = signals::SensorR_OD_AdaptModelByCurrent;
            using SensorR_OD_CVOPCTcFreq = signals::SensorR_OD_CVOPCTcFreq;
            using SensorR_OD_CVOPCImin = signals::SensorR_OD_CVOPCImin;
            using SensorR_OD_CascadedKpDecreaseCurrent = signals::SensorR_OD_CascadedKpDecreaseCurrent;
            using SensorR_OD_CascadedKpIncreaseCurrent = signals::SensorR_OD_CascadedKpIncreaseCurrent;
            using SensorR_OD_CascadedFreqCurUpdate = signals::SensorR_OD_CascadedFreqCurUpdate;
            using SensorR_OD_CascadedMaxSetForceDelta = signals::SensorR_OD_CascadedMaxSetForceDelta;
            using SensorR_OD_CurrentForConstantVelocity = signals::SensorR_OD_CurrentForConstantVelocity;
            using SensorR_OD_AccAtCurrentMeasured = signals::SensorR_OD_AccAtCurrentMeasured;
            using SensorR_OD_CVOPCHystereseF = signals::SensorR_OD_CVOPCHystereseF;
            using SensorR_OD_CVOPCReduceCurrentWhileCruising = signals::SensorR_OD_CVOPCReduceCurrentWhileCruising;
            using SensorR_OD_CVOPCDeltaVelHigherCurrent = signals::SensorR_OD_CVOPCDeltaVelHigherCurrent;
            using SensorR_OD_CVOPCSmoothingDeltaVelocity = signals::SensorR_OD_CVOPCSmoothingDeltaVelocity;
            using SensorR_OD_CVOPCTcI = signals::SensorR_OD_CVOPCTcI;
            using SensorR_OD_CVOPCTcf = signals::SensorR_OD_CVOPCTcf;
            using SensorR_OD_CVOPCImax = signals::SensorR_OD_CVOPCImax;
            using SensorR_OD_VelocityController = signals::SensorR_OD_VelocityController;
            using SensorR_OD_TC_f = signals::SensorR_OD_TC_f;
            using SensorR_OD_TC_I = signals::SensorR_OD_TC_I;
            using SensorR_OD_EstimatedRunLength = signals::SensorR_OD_EstimatedRunLength;
            using SensorR_OD_RunProfile_V4 = signals::SensorR_OD_RunProfile_V4;
            using SensorR_OD_RunProfile_T4 = signals::SensorR_OD_RunProfile_T4;
            using SensorR_OD_RunProfile_V3 = signals::SensorR_OD_RunProfile_V3;
            using SensorR_OD_RunProfile_T3 = signals::SensorR_OD_RunProfile_T3;
            using SensorR_OD_RunProfile_V2 = signals::SensorR_OD_RunProfile_V2;
            using SensorR_OD_RunProfile_T2 = signals::SensorR_OD_RunProfile_T2;
            using SensorR_OD_RunProfile_V1 = signals::SensorR_OD_RunProfile_V1;
            using SensorR_OD_RunProfile_T1 = signals::SensorR_OD_RunProfile_T1;
            using SensorR_OD_RunProfile_V0 = signals::SensorR_OD_RunProfile_V0;
            using SensorR_OD_RunProfile_T0 = signals::SensorR_OD_RunProfile_T0;
            using SensorR_OD_SetFinish = signals::SensorR_OD_SetFinish;
            using SensorR_OD_RunMode = signals::SensorR_OD_RunMode;
            using SensorR_OD_VdcReduceFactor = signals::SensorR_OD_VdcReduceFactor;
            using SensorR_OD_MaxRequiredVelocity = signals::SensorR_OD_MaxRequiredVelocity;
            using SensorR_OD_WarningThreshholdForLimModelOutOfBoundaries = signals::SensorR_OD_WarningThreshholdForLimModelOutOfBoundaries;
            using SensorR_OD_MaxCtrlErrorDuration = signals::SensorR_OD_MaxCtrlErrorDuration;
            using SensorR_OD_MaxCtrlError = signals::SensorR_OD_MaxCtrlError;
            using SensorR_OD_FminCtrl = signals::SensorR_OD_FminCtrl;
            using SensorR_OD_FmaxCtrl = signals::SensorR_OD_FmaxCtrl;
            using SensorR_OD_PodMass = signals::SensorR_OD_PodMass;
            using SensorR_OD_CurrentReq = signals::SensorR_OD_CurrentReq;
            using SensorR_OD_CtrlMode = signals::SensorR_OD_CtrlMode;
            using SensorR_OD_OpcKi = signals::SensorR_OD_OpcKi;
            using SensorR_OD_OpcKp = signals::SensorR_OD_OpcKp;
            using SensorR_OD_ExecTimeReadSensors = signals::SensorR_OD_ExecTimeReadSensors;
            using SensorR_OD_ExecTimeControl = signals::SensorR_OD_ExecTimeControl;
            using SensorR_OD_ExecTimeStateEst = signals::SensorR_OD_ExecTimeStateEst;
            using SensorR_OD_ExecTimeOverall = signals::SensorR_OD_ExecTimeOverall;
            using SensorR_OD_IMU3_Temperature = signals::SensorR_OD_IMU3_Temperature;
            using SensorR_OD_IMU2_Temperature = signals::SensorR_OD_IMU2_Temperature;
            using SensorR_OD_IMU1_Temperature = signals::SensorR_OD_IMU1_Temperature;
            using SensorR_OD_IMU_number = signals::SensorR_OD_IMU_number;
            using SensorR_OD_EncoderResetPosition = signals::SensorR_OD_EncoderResetPosition;
            using SensorR_OD_EncoderWheelDiameter = signals::SensorR_OD_EncoderWheelDiameter;
            using SensorR_OD_samplingInterval = signals::SensorR_OD_samplingInterval;
            using SensorR_OD_CAN2_DelayedTxMessages = signals::SensorR_OD_CAN2_DelayedTxMessages;
            using SensorR_OD_CAN2_ErrorStatus = signals::SensorR_OD_CAN2_ErrorStatus;
            using SensorR_OD_CAN2_DiscardedTxMessages = signals::SensorR_OD_CAN2_DiscardedTxMessages;
            using SensorR_OD_CAN2_Status = signals::SensorR_OD_CAN2_Status;
            using SensorR_OD_CAN2_Baudrate = signals::SensorR_OD_CAN2_Baudrate;
            using SensorR_OD_CAN2_autoErrorReset = signals::SensorR_OD_CAN2_autoErrorReset;
            using SensorR_OD_CAN2_lastErrorCode = signals::SensorR_OD_CAN2_lastErrorCode;
            using SensorR_OD_CAN2_RxErrCnt = signals::SensorR_OD_CAN2_RxErrCnt;
            using SensorR_OD_CAN2_TxErrCnt = signals::SensorR_OD_CAN2_TxErrCnt;
            using SensorR_OD_CAN1_DelayedTxMessages = signals::SensorR_OD_CAN1_DelayedTxMessages;
            using SensorR_OD_CAN1_ErrorStatus = signals::SensorR_OD_CAN1_ErrorStatus;
            using SensorR_OD_CAN1_DiscardedTxMessages = signals::SensorR_OD_CAN1_DiscardedTxMessages;
            using SensorR_OD_CAN1_Status = signals::SensorR_OD_CAN1_Status;
            using SensorR_OD_CAN1_Baudrate = signals::SensorR_OD_CAN1_Baudrate;
            using SensorR_OD_CAN1_autoErrorReset = signals::SensorR_OD_CAN1_autoErrorReset;
            using SensorR_OD_CAN1_lastErrorCode = signals::SensorR_OD_CAN1_lastErrorCode;
            using SensorR_OD_CAN1_RxErrCnt = signals::SensorR_OD_CAN1_RxErrCnt;
            using SensorR_OD_CAN1_TxErrCnt = signals::SensorR_OD_CAN1_TxErrCnt;
            using SensorR_OD_BuildTime = signals::SensorR_OD_BuildTime;
            using SensorR_OD_BuildDate = signals::SensorR_OD_BuildDate;
            using SensorR_OD_ChipUID2 = signals::SensorR_OD_ChipUID2;
            using SensorR_OD_ChipUID1 = signals::SensorR_OD_ChipUID1;
            using SensorR_OD_SdcOut = signals::SensorR_OD_SdcOut;
            using SensorR_OD_SdcIn = signals::SensorR_OD_SdcIn;
            using SensorR_OD_runtime = signals::SensorR_OD_runtime;
            using SensorR_OD_InputVoltage = signals::SensorR_OD_InputVoltage;
            using SensorR_OD_BoardTemp = signals::SensorR_OD_BoardTemp;
            using SensorR_OD_MemFree = signals::SensorR_OD_MemFree;
            using SensorR_OD_CpuUsage = signals::SensorR_OD_CpuUsage;
            using SensorR_OD_OdEntrySendInterval = signals::SensorR_OD_OdEntrySendInterval;
            using SensorR_OD_SendOdOnBootup = signals::SensorR_OD_SendOdOnBootup;
            using SensorR_OD_HeartbeatInterval = signals::SensorR_OD_HeartbeatInterval;
            using SensorR_OD_DbcVersion = signals::SensorR_OD_DbcVersion;
            using SensorR_OD_StackVersion = signals::SensorR_OD_StackVersion;
            using SensorR_OD_ProtocolVersion = signals::SensorR_OD_ProtocolVersion;
            using SensorR_OD_NodeStatus = signals::SensorR_OD_NodeStatus;
            using SensorR_OD_NodeID = signals::SensorR_OD_NodeID;

            // Attributes of message 'SensorR_SDO_Resp'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_EMCY {
            public:
            constexpr static uint32_t id = 0x9A;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_W0_OtherWarning = signals::PDU_W0_OtherWarning;
            using PDU_W1_batterVoltageLow = signals::PDU_W1_batterVoltageLow;
            using PDU_W2_batterTempHigh = signals::PDU_W2_batterTempHigh;
            using PDU_E0_OtherError = signals::PDU_E0_OtherError;
            using PDU_E1_batterVoltageCritical = signals::PDU_E1_batterVoltageCritical;
            using PDU_E2_batteryOvercurrent = signals::PDU_E2_batteryOvercurrent;
            using PDU_E3_batterTempCritical = signals::PDU_E3_batterTempCritical;
            using PDU_E4_watchdogStateMachine = signals::PDU_E4_watchdogStateMachine;

            // Attributes of message 'PDU_EMCY'
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
        class PDU_TX_HP_Short_Circuit_Debug {
            public:
            constexpr static uint32_t id = 0x29A;
            constexpr static uint8_t dlc = 3;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_HPCh1_ShortCnt = signals::PDU_HPCh1_ShortCnt;
            using PDU_HPCh1_State = signals::PDU_HPCh1_State;
            using PDU_HPCh2_State = signals::PDU_HPCh2_State;
            using PDU_HPCh2_ShortCnt = signals::PDU_HPCh2_ShortCnt;
            using PDU_HPCh3_ShortCnt = signals::PDU_HPCh3_ShortCnt;
            using PDU_HPCh3_State = signals::PDU_HPCh3_State;
            using PDU_HPCh4_ShortCnt = signals::PDU_HPCh4_ShortCnt;
            using PDU_HPCh4_State = signals::PDU_HPCh4_State;

            // Attributes of message 'PDU_TX_HP_Short_Circuit_Debug'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_HP_D_Dutycycle {
            public:
            constexpr static uint32_t id = 0x2DA;
            constexpr static uint8_t dlc = 6;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_HPCh1_Dutycycle = signals::PDU_HPCh1_Dutycycle;
            using PDU_HPCh2_Dutycycle = signals::PDU_HPCh2_Dutycycle;
            using PDU_D1_Dutycycle = signals::PDU_D1_Dutycycle;
            using PDU_D2_Dutycycle = signals::PDU_D2_Dutycycle;
            using PDU_D3_Dutycycle = signals::PDU_D3_Dutycycle;
            using PDU_D4_Dutycycle = signals::PDU_D4_Dutycycle;

            // Attributes of message 'PDU_RX_HP_D_Dutycycle'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_PDO4 {
            public:
            constexpr static uint32_t id = 0x31A;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_TX_PDO4'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_RX_Manual_Control {
            public:
            constexpr static uint32_t id = 0x35A;
            constexpr static uint8_t dlc = 3;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LPCh1_Enable = signals::PDU_LPCh1_Enable;
            using PDU_LPCh2_Enable = signals::PDU_LPCh2_Enable;
            using PDU_LPCh3_Enable = signals::PDU_LPCh3_Enable;
            using PDU_LPCh4_Enable = signals::PDU_LPCh4_Enable;
            using PDU_LPCh5_Enable = signals::PDU_LPCh5_Enable;
            using PDU_LPCh6_Enable = signals::PDU_LPCh6_Enable;
            using PDU_LPCh7_Enable = signals::PDU_LPCh7_Enable;
            using PDU_LPCh8_Enable = signals::PDU_LPCh8_Enable;
            using PDU_LPCh9_Enable = signals::PDU_LPCh9_Enable;
            using PDU_LPCh10_Enable = signals::PDU_LPCh10_Enable;
            using PDU_HPCh1_Enable = signals::PDU_HPCh1_Enable;
            using PDU_HPCh2_Enable = signals::PDU_HPCh2_Enable;
            using PDU_HPCh3_Enable = signals::PDU_HPCh3_Enable;
            using PDU_HPCh4_Enable = signals::PDU_HPCh4_Enable;
            using PDU_D1_Enable = signals::PDU_D1_Enable;
            using PDU_D2_Enable = signals::PDU_D2_Enable;
            using PDU_D3_Enable = signals::PDU_D3_Enable;
            using PDU_D4_Enable = signals::PDU_D4_Enable;
            using PDU_SDC_Enable = signals::PDU_SDC_Enable;

            // Attributes of message 'PDU_RX_Manual_Control'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_PDO5 {
            public:
            constexpr static uint32_t id = 0x39A;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_TX_PDO5'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_PDO6 {
            public:
            constexpr static uint32_t id = 0x3DA;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_TX_PDO6'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_PDO7 {
            public:
            constexpr static uint32_t id = 0x41A;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_TX_PDO7'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_PDO8 {
            public:
            constexpr static uint32_t id = 0x45A;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_TX_PDO8'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_LP_Short_Circuit_Debug {
            public:
            constexpr static uint32_t id = 0x49A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LPCh10_ShortCnt = signals::PDU_LPCh10_ShortCnt;
            using PDU_LPCh1_ShortCnt = signals::PDU_LPCh1_ShortCnt;
            using PDU_LPCh2_ShortCnt = signals::PDU_LPCh2_ShortCnt;
            using PDU_LPCh3_ShortCnt = signals::PDU_LPCh3_ShortCnt;
            using PDU_LPCh4_ShortCnt = signals::PDU_LPCh4_ShortCnt;
            using PDU_LPCh5_ShortCnt = signals::PDU_LPCh5_ShortCnt;
            using PDU_LPCh6_ShortCnt = signals::PDU_LPCh6_ShortCnt;
            using PDU_LPCh7_ShortCnt = signals::PDU_LPCh7_ShortCnt;
            using PDU_LPCh8_ShortCnt = signals::PDU_LPCh8_ShortCnt;
            using PDU_LPCh9_ShortCnt = signals::PDU_LPCh9_ShortCnt;
            using PDU_LPCh10_State = signals::PDU_LPCh10_State;
            using PDU_LPCh1_State = signals::PDU_LPCh1_State;
            using PDU_LPCh2_State = signals::PDU_LPCh2_State;
            using PDU_LPCh3_State = signals::PDU_LPCh3_State;
            using PDU_LPCh4_State = signals::PDU_LPCh4_State;
            using PDU_LPCh5_State = signals::PDU_LPCh5_State;
            using PDU_LPCh6_State = signals::PDU_LPCh6_State;
            using PDU_LPCh7_State = signals::PDU_LPCh7_State;
            using PDU_LPCh8_State = signals::PDU_LPCh8_State;
            using PDU_LPCh9_State = signals::PDU_LPCh9_State;

            // Attributes of message 'PDU_TX_LP_Short_Circuit_Debug'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_TX_LV_BMS {
            public:
            constexpr static uint32_t id = 0x4DA;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_LV_Current = signals::PDU_LV_Current;
            using PDU_LV_Voltage = signals::PDU_LV_Voltage;
            using PDU_LV_SOC = signals::PDU_LV_SOC;

            // Attributes of message 'PDU_TX_LV_BMS'
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
        class PDU_SDO_Resp {
            public:
            constexpr static uint32_t id = 0x59A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_SDO_ID = signals::PDU_SDO_ID;
            using PDU_SDO_RespCode = signals::PDU_SDO_RespCode;
            using PDU_OD_CoolingPumpEnabled = signals::PDU_OD_CoolingPumpEnabled;
            using PDU_OD_LedCommands = signals::PDU_OD_LedCommands;
            using PDU_OD_projectXXEnabled = signals::PDU_OD_projectXXEnabled;
            using PDU_OD_watchdogTimeout = signals::PDU_OD_watchdogTimeout;
            using PDU_OD_statusSendInterval = signals::PDU_OD_statusSendInterval;
            using PDU_OD_currentReadInterval = signals::PDU_OD_currentReadInterval;
            using PDU_OD_batteryOvercurrent = signals::PDU_OD_batteryOvercurrent;
            using PDU_OD_overTempCritical = signals::PDU_OD_overTempCritical;
            using PDU_OD_overTempWarn = signals::PDU_OD_overTempWarn;
            using PDU_OD_batterVoltageCritical = signals::PDU_OD_batterVoltageCritical;
            using PDU_OD_batterVoltageLow = signals::PDU_OD_batterVoltageLow;
            using PDU_OD_CAN2_DelayedTxMessages = signals::PDU_OD_CAN2_DelayedTxMessages;
            using PDU_OD_CAN2_ErrorStatus = signals::PDU_OD_CAN2_ErrorStatus;
            using PDU_OD_CAN2_DiscardedTxMessages = signals::PDU_OD_CAN2_DiscardedTxMessages;
            using PDU_OD_CAN2_Status = signals::PDU_OD_CAN2_Status;
            using PDU_OD_CAN2_Baudrate = signals::PDU_OD_CAN2_Baudrate;
            using PDU_OD_CAN2_autoErrorReset = signals::PDU_OD_CAN2_autoErrorReset;
            using PDU_OD_CAN2_lastErrorCode = signals::PDU_OD_CAN2_lastErrorCode;
            using PDU_OD_CAN2_RxErrCnt = signals::PDU_OD_CAN2_RxErrCnt;
            using PDU_OD_CAN2_TxErrCnt = signals::PDU_OD_CAN2_TxErrCnt;
            using PDU_OD_CAN1_DelayedTxMessages = signals::PDU_OD_CAN1_DelayedTxMessages;
            using PDU_OD_CAN1_ErrorStatus = signals::PDU_OD_CAN1_ErrorStatus;
            using PDU_OD_CAN1_DiscardedTxMessages = signals::PDU_OD_CAN1_DiscardedTxMessages;
            using PDU_OD_CAN1_Status = signals::PDU_OD_CAN1_Status;
            using PDU_OD_CAN1_Baudrate = signals::PDU_OD_CAN1_Baudrate;
            using PDU_OD_CAN1_autoErrorReset = signals::PDU_OD_CAN1_autoErrorReset;
            using PDU_OD_CAN1_lastErrorCode = signals::PDU_OD_CAN1_lastErrorCode;
            using PDU_OD_CAN1_RxErrCnt = signals::PDU_OD_CAN1_RxErrCnt;
            using PDU_OD_CAN1_TxErrCnt = signals::PDU_OD_CAN1_TxErrCnt;
            using PDU_OD_BuildTime = signals::PDU_OD_BuildTime;
            using PDU_OD_BuildDate = signals::PDU_OD_BuildDate;
            using PDU_OD_ChipUID2 = signals::PDU_OD_ChipUID2;
            using PDU_OD_ChipUID1 = signals::PDU_OD_ChipUID1;
            using PDU_OD_SdcOut = signals::PDU_OD_SdcOut;
            using PDU_OD_SdcIn = signals::PDU_OD_SdcIn;
            using PDU_OD_runtime = signals::PDU_OD_runtime;
            using PDU_OD_InputVoltage = signals::PDU_OD_InputVoltage;
            using PDU_OD_BoardTemp = signals::PDU_OD_BoardTemp;
            using PDU_OD_MemFree = signals::PDU_OD_MemFree;
            using PDU_OD_CpuUsage = signals::PDU_OD_CpuUsage;
            using PDU_OD_OdEntrySendInterval = signals::PDU_OD_OdEntrySendInterval;
            using PDU_OD_SendOdOnBootup = signals::PDU_OD_SendOdOnBootup;
            using PDU_OD_HeartbeatInterval = signals::PDU_OD_HeartbeatInterval;
            using PDU_OD_DbcVersion = signals::PDU_OD_DbcVersion;
            using PDU_OD_StackVersion = signals::PDU_OD_StackVersion;
            using PDU_OD_ProtocolVersion = signals::PDU_OD_ProtocolVersion;
            using PDU_OD_NodeStatus = signals::PDU_OD_NodeStatus;
            using PDU_OD_NodeID = signals::PDU_OD_NodeID;

            // Attributes of message 'PDU_SDO_Resp'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_SDO_Req_Up {
            public:
            constexpr static uint32_t id = 0x5DA;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_SDO_ID = signals::PDU_SDO_ID;
            using PDU_OD_CoolingPumpEnabled = signals::PDU_OD_CoolingPumpEnabled;
            using PDU_OD_LedCommands = signals::PDU_OD_LedCommands;
            using PDU_OD_projectXXEnabled = signals::PDU_OD_projectXXEnabled;
            using PDU_OD_watchdogTimeout = signals::PDU_OD_watchdogTimeout;
            using PDU_OD_statusSendInterval = signals::PDU_OD_statusSendInterval;
            using PDU_OD_currentReadInterval = signals::PDU_OD_currentReadInterval;
            using PDU_OD_batteryOvercurrent = signals::PDU_OD_batteryOvercurrent;
            using PDU_OD_overTempCritical = signals::PDU_OD_overTempCritical;
            using PDU_OD_overTempWarn = signals::PDU_OD_overTempWarn;
            using PDU_OD_batterVoltageCritical = signals::PDU_OD_batterVoltageCritical;
            using PDU_OD_batterVoltageLow = signals::PDU_OD_batterVoltageLow;
            using PDU_OD_CAN2_DelayedTxMessages = signals::PDU_OD_CAN2_DelayedTxMessages;
            using PDU_OD_CAN2_ErrorStatus = signals::PDU_OD_CAN2_ErrorStatus;
            using PDU_OD_CAN2_DiscardedTxMessages = signals::PDU_OD_CAN2_DiscardedTxMessages;
            using PDU_OD_CAN2_Status = signals::PDU_OD_CAN2_Status;
            using PDU_OD_CAN2_Baudrate = signals::PDU_OD_CAN2_Baudrate;
            using PDU_OD_CAN2_autoErrorReset = signals::PDU_OD_CAN2_autoErrorReset;
            using PDU_OD_CAN2_lastErrorCode = signals::PDU_OD_CAN2_lastErrorCode;
            using PDU_OD_CAN2_RxErrCnt = signals::PDU_OD_CAN2_RxErrCnt;
            using PDU_OD_CAN2_TxErrCnt = signals::PDU_OD_CAN2_TxErrCnt;
            using PDU_OD_CAN1_DelayedTxMessages = signals::PDU_OD_CAN1_DelayedTxMessages;
            using PDU_OD_CAN1_ErrorStatus = signals::PDU_OD_CAN1_ErrorStatus;
            using PDU_OD_CAN1_DiscardedTxMessages = signals::PDU_OD_CAN1_DiscardedTxMessages;
            using PDU_OD_CAN1_Status = signals::PDU_OD_CAN1_Status;
            using PDU_OD_CAN1_Baudrate = signals::PDU_OD_CAN1_Baudrate;
            using PDU_OD_CAN1_autoErrorReset = signals::PDU_OD_CAN1_autoErrorReset;
            using PDU_OD_CAN1_lastErrorCode = signals::PDU_OD_CAN1_lastErrorCode;
            using PDU_OD_CAN1_RxErrCnt = signals::PDU_OD_CAN1_RxErrCnt;
            using PDU_OD_CAN1_TxErrCnt = signals::PDU_OD_CAN1_TxErrCnt;
            using PDU_OD_BuildTime = signals::PDU_OD_BuildTime;
            using PDU_OD_BuildDate = signals::PDU_OD_BuildDate;
            using PDU_OD_ChipUID2 = signals::PDU_OD_ChipUID2;
            using PDU_OD_ChipUID1 = signals::PDU_OD_ChipUID1;
            using PDU_OD_SdcOut = signals::PDU_OD_SdcOut;
            using PDU_OD_SdcIn = signals::PDU_OD_SdcIn;
            using PDU_OD_runtime = signals::PDU_OD_runtime;
            using PDU_OD_InputVoltage = signals::PDU_OD_InputVoltage;
            using PDU_OD_BoardTemp = signals::PDU_OD_BoardTemp;
            using PDU_OD_MemFree = signals::PDU_OD_MemFree;
            using PDU_OD_CpuUsage = signals::PDU_OD_CpuUsage;
            using PDU_OD_OdEntrySendInterval = signals::PDU_OD_OdEntrySendInterval;
            using PDU_OD_SendOdOnBootup = signals::PDU_OD_SendOdOnBootup;
            using PDU_OD_HeartbeatInterval = signals::PDU_OD_HeartbeatInterval;
            using PDU_OD_DbcVersion = signals::PDU_OD_DbcVersion;
            using PDU_OD_StackVersion = signals::PDU_OD_StackVersion;
            using PDU_OD_ProtocolVersion = signals::PDU_OD_ProtocolVersion;
            using PDU_OD_NodeStatus = signals::PDU_OD_NodeStatus;
            using PDU_OD_NodeID = signals::PDU_OD_NodeID;

            // Attributes of message 'PDU_SDO_Req_Up'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_SDO_Req_Down {
            public:
            constexpr static uint32_t id = 0x61A;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using PDU_SDO_ID = signals::PDU_SDO_ID;

            // Attributes of message 'PDU_SDO_Req_Down'
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
        class PDU_BTL_TX {
            public:
            constexpr static uint32_t id = 0x75A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_BTL_TX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class PDU_BTL_RX {
            public:
            constexpr static uint32_t id = 0x79A;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'PDU_BTL_RX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };

        // Aliases for all CANzero messages of the current node, excluding the node name in the message name
        // This allows writing node independent code by using e.g. can::messages::CANZERO_BTL_RX instead of can::messages::PDU_BTL_RX
        // Also if the user renamed a PDO (e.g. PDU_TX_PDO1 to PDU_TX_Current) the alias will still be TX_PDO1
        using CANZERO_EMCY = PDU_EMCY;
        using CANZERO_TX_PDO1 = PDU_TX_Status;
        using CANZERO_RX_PDO1 = PDU_RX_Control;
        using CANZERO_TX_PDO2 = PDU_TX_HP_Current;
        using CANZERO_RX_PDO2 = PDU_RX_LP_Dutycycle;
        using CANZERO_TX_PDO3 = PDU_TX_HP_Short_Circuit_Debug;
        using CANZERO_RX_PDO3 = PDU_RX_HP_D_Dutycycle;
        using CANZERO_TX_PDO4 = PDU_TX_PDO4;
        using CANZERO_RX_PDO4 = PDU_RX_Manual_Control;
        using CANZERO_TX_PDO5 = PDU_TX_PDO5;
        using CANZERO_TX_PDO6 = PDU_TX_PDO6;
        using CANZERO_TX_PDO7 = PDU_TX_PDO7;
        using CANZERO_TX_PDO8 = PDU_TX_PDO8;
        using CANZERO_TX_PDO9 = PDU_TX_LP_Short_Circuit_Debug;
        using CANZERO_TX_PDO10 = PDU_TX_LV_BMS;
        using CANZERO_TX_PDO11 = PDU_TX_LP_Current1;
        using CANZERO_TX_PDO12 = PDU_TX_LP_Current2;
        using CANZERO_SDO_Resp = PDU_SDO_Resp;
        using CANZERO_SDO_Req_Up = PDU_SDO_Req_Up;
        using CANZERO_SDO_Req_Down = PDU_SDO_Req_Down;
        using CANZERO_Heartbeat = PDU_Heartbeat;
        using CANZERO_BTL_TX = PDU_BTL_TX;
        using CANZERO_BTL_RX = PDU_BTL_RX;
        
    }
}

#endif // DBCPARSER_DBC_PARSER_HPP