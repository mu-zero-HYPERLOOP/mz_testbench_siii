/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY DBC2CPP V1.7.7.
 * 
 * This header file was generated from 'pod2023_gen.dbc' on 15:42:49 18.05.2023.
 * It contains all messages and signals as well as value tables and attributes of the DBC file.
 * Only messages and signals received or sent from node 'BrakeF' were parsed.
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

        constexpr uint8_t num_std = 9;      // Number of used receive filters for standard (11-bit) ID messages
        constexpr uint32_t mask_std[9] = {   // Filter mask for standard (11-bit) ID messages
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF,            0x7FF,            0x7FF,            0x7FF, 
            0x7FF 
        };
        constexpr uint32_t id_std[9] = {     // Filter ID for standard (11-bit) ID messages
            0x002,            0x181,            0x1D1,            0x251, 
            0x2D1,            0x351,            0x5D1,            0x611, 
            0x791 
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
    constexpr uint32_t CANzero_DBCVersion = 176;
    constexpr char CANzero_ProtocolVersion[] = "V1.0";
    constexpr char BusType[] = "CAN";
    constexpr char DBName[] = "pod2022";
    
    /**********************************************************************************************
    * Namespace containing all signals with their value tables and attributes                     *
    ***********************************************************************************************/
    namespace signals {
        class BrakeF_TX_Pressure_Reservoir {
            public:
            using dataType = float;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x410 };
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
                intel |= (static_cast<uint64_t>(rawValue)) & 0x7ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x7ull));
                return value;
            }

            // Value table of signal 'SensorF_TX_PodState'
            constexpr static uint8_t POD_OFF = 0;
            constexpr static uint8_t POD_IDLE = 1;
            constexpr static uint8_t POD_LAUNCH_PREPARATION = 2;
            constexpr static uint8_t POD_FAULT = 3;
            constexpr static uint8_t POD_READY_TO_LAUNCH = 4;
            constexpr static uint8_t POD_LAUNCHING = 5;
            constexpr static uint8_t POD_PUSHABLE = 6;
            constexpr static uint8_t POD_SAFE_TO_APPROACH = 7;
            constexpr static uint8_t POD_START_LEVITATION = 8;
            constexpr static uint8_t POD_STOP_LEVITATION = 9;
            constexpr static uint8_t POD_LEVITATING = 10;
            constexpr static uint8_t POD_BREAKING = 11;
            constexpr static uint8_t POD_STOP = 12;
        };
        class SensorF_TX_PodState_Last {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x181 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x38ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x38ull) >> 3);
                return value;
            }

            // Value table of signal 'SensorF_TX_PodState_Last'
            constexpr static uint8_t POD_OFF = 0;
            constexpr static uint8_t POD_IDLE = 1;
            constexpr static uint8_t POD_LAUNCH_PREPARATION = 2;
            constexpr static uint8_t POD_FAULT = 3;
            constexpr static uint8_t POD_READY_TO_LAUNCH = 4;
            constexpr static uint8_t POD_LAUNCHING = 5;
            constexpr static uint8_t POD_PUSHABLE = 6;
            constexpr static uint8_t POD_SAFE_TO_APPROACH = 7;
        };
        class SensorF_TX_PodState_Target {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x181 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 8) & 0x700ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0x700ull) >> 8);
                return value;
            }

            // Value table of signal 'SensorF_TX_PodState_Target'
            constexpr static uint8_t POD_OFF = 0;
            constexpr static uint8_t POD_IDLE = 1;
            constexpr static uint8_t POD_LAUNCH_PREPARATION = 2;
            constexpr static uint8_t POD_FAULT = 3;
            constexpr static uint8_t POD_READY_TO_LAUNCH = 4;
            constexpr static uint8_t POD_LAUNCHING = 5;
            constexpr static uint8_t POD_PUSHABLE = 6;
            constexpr static uint8_t POD_SAFE_TO_APPROACH = 7;
        };
        class BrakeF_W0_highPressureActingChamber {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue)) & 0x1ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1ull));
                return value;
            }

            // Value table of signal 'BrakeF_W0_highPressureActingChamber'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;

            // Attributes of signal 'BrakeF_W0_highPressureActingChamber'
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_W0_highPressureActingChamber";
        };
        class BrakeF_W1_highPressureRetractingChamber {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 1) & 0x2ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2ull) >> 1);
                return value;
            }

            // Value table of signal 'BrakeF_W1_highPressureRetractingChamber'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;

            // Attributes of signal 'BrakeF_W1_highPressureRetractingChamber'
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_W1_highPressureRetractingChamber";
        };
        class BrakeF_W2_enableWithAnError {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 2) & 0x4ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4ull) >> 2);
                return value;
            }

            // Value table of signal 'BrakeF_W2_enableWithAnError'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class BrakeF_W3_externalError {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 3) & 0x8ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8ull) >> 3);
                return value;
            }

            // Value table of signal 'BrakeF_W3_externalError'
            constexpr static bool OK = 0;
            constexpr static bool WARN = 1;
        };
        class BrakeF_E0_pressureTooHigh {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0x1000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x1000ull) >> 12);
                return value;
            }

            // Value table of signal 'BrakeF_E0_pressureTooHigh'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class BrakeF_E1_pressureTooLow {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 13) & 0x2000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x2000ull) >> 13);
                return value;
            }

            // Value table of signal 'BrakeF_E1_pressureTooLow'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class BrakeF_E2_commWatchdogTimeout {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 14) & 0x4000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x4000ull) >> 14);
                return value;
            }

            // Value table of signal 'BrakeF_E2_commWatchdogTimeout'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;
        };
        class BrakeF_E3_retractUnsuccesful_errorFlag {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 15) & 0x8000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x8000ull) >> 15);
                return value;
            }

            // Value table of signal 'BrakeF_E3_retractUnsuccesful_errorFlag'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;

            // Attributes of signal 'BrakeF_E3_retractUnsuccesful_errorFlag'
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_E3_retractUnsuccesful_errorFlag";
        };
        class BrakeF_E4_retractUnsuccesful_notEnabled {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0x10000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x10000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_E4_retractUnsuccesful_notEnabled'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;

            // Attributes of signal 'BrakeF_E4_retractUnsuccesful_notEnabled'
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_E4_retractUnsuccesful_notEnabled";
        };
        class BrakeF_E5_retractUnsuccesful_openSDC {
            public:
            using dataType = bool;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x91 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, bool value) noexcept {
                bool rawValue = value;
                intel |= (static_cast<uint64_t>(rawValue) << 17) & 0x20000ull;
            }
            constexpr static inline bool get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                bool value = static_cast<bool>((intel & 0x20000ull) >> 17);
                return value;
            }

            // Value table of signal 'BrakeF_E5_retractUnsuccesful_openSDC'
            constexpr static bool OK = 0;
            constexpr static bool ERR = 1;

            // Attributes of signal 'BrakeF_E5_retractUnsuccesful_openSDC'
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_E5_retractUnsuccesful_openSDC";
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
        class BrakeF_TX_Pressure_Act {
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

            // Attributes of signal 'BrakeF_TX_Pressure_Act'
            constexpr static float GenSigStartValue = 400.0f;
        };
        class BrakeF_TX_Pressure_Retract {
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
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFF0000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF0000ull) >> 16);
                return value * (0.005f) + (-2.0f);
            }

            // Attributes of signal 'BrakeF_TX_Pressure_Retract'
            constexpr static float GenSigStartValue = 400.0f;
        };
        class BrakeF_TX_Pressure_Tank {
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
                intel |= (static_cast<uint64_t>(rawValue) << 32) & 0xFFF00000000ull;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint16_t value = static_cast<uint16_t>((intel & 0xFFF00000000ull) >> 32);
                return value * (0.005f) + (-2.0f);
            }

            // Attributes of signal 'BrakeF_TX_Pressure_Tank'
            constexpr static float GenSigStartValue = 400.0f;
        };
        class BrakeF_SDO_ID {
            public:
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 3;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1, 0x611 };
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

            // Value table of signal 'BrakeF_SDO_ID'
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
            constexpr static uint16_t TANKLOWERCONTROLLIMIT = 2048;
            constexpr static uint16_t TANKUPPERCONTROLLIMIT = 2049;
            constexpr static uint16_t DELAY = 2050;
            constexpr static uint16_t COUNTERLIMIT = 2051;
            constexpr static uint16_t COMMWATCHDOG = 2052;
            constexpr static uint16_t VALVEUPPERTOLERANCE = 2053;
        };
        class BrakeF_SDO_RespCode {
            public:
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 1;
            constexpr static uint32_t ids[] = { 0x591 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 12) & 0xF000ull;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                uint8_t value = static_cast<uint8_t>((intel & 0xF000ull) >> 12);
                return value;
            }

            // Value table of signal 'BrakeF_SDO_RespCode'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t ERR_NON_EXISTING_OBJECT = 1;
            constexpr static uint8_t ERR_WRITE_ONLY_OBJECT = 2;
            constexpr static uint8_t ERR_READ_ONLY_OBJECT = 3;
            constexpr static uint8_t ERR_NO_ACCESS_IN_THIS_STATE = 4;
            constexpr static uint8_t ERR_OUT_OF_RANGE = 5;
        };
        class BrakeF_OD_valveUpperTolerance {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2053            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(327.675);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2053);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2053) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.005f);
            }

            // Attributes of signal 'BrakeF_OD_valveUpperTolerance'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 2000.0f;
            constexpr static float CANzero_SDO_Default = 10.0f;
        };
        class BrakeF_OD_commWatchdog {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2052            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2052);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2052) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_commWatchdog'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 60.0f;
            constexpr static float CANzero_SDO_Default = 60.0f;
        };
        class BrakeF_OD_counterLimit {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2051            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2051);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2051) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_counterLimit'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 5.0f;
            constexpr static float CANzero_SDO_Default = 5.0f;
        };
        class BrakeF_OD_delay {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2050            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(127.5);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2050);
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2050) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'BrakeF_OD_delay'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 10.0f;
            constexpr static float CANzero_SDO_Default = 5.0f;
        };
        class BrakeF_OD_tankUpperControlLimit {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2049            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(327.675);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2049);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2049) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.005f);
            }

            // Attributes of signal 'BrakeF_OD_tankUpperControlLimit'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 3800.0f;
            constexpr static float CANzero_SDO_Default = 19.0f;
        };
        class BrakeF_OD_tankLowerControlLimit {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2048            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(327.675);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2048);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.005f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2048) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.005f);
            }

            // Attributes of signal 'BrakeF_OD_tankLowerControlLimit'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1400.0f;
            constexpr static float CANzero_SDO_Default = 7.0f;
        };
        class BrakeF_OD_CAN2_DelayedTxMessages {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1129            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1129);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1129) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN2_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN2_ErrorStatus {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1128            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1128);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1128) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN2_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'BrakeF_OD_CAN2_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN2_DiscardedTxMessages {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1127            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1127);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1127) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN2_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_OD_CAN2_DiscardedTxMessages";
        };
        class BrakeF_OD_CAN2_Status {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1126            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1126);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1126) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN2_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'BrakeF_OD_CAN2_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN2_Baudrate {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1124            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1124);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1124) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN2_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class BrakeF_OD_CAN2_autoErrorReset {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1123            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1123);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1123) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN2_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'BrakeF_OD_CAN2_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class BrakeF_OD_CAN2_lastErrorCode {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1122            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1122);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1122) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN2_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'BrakeF_OD_CAN2_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN2_RxErrCnt {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1121            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1121);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1121) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN2_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN2_TxErrCnt {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1120            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1120);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1120) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN2_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_DelayedTxMessages {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1113            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1113);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1113) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN1_DelayedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_ErrorStatus {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1112            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1112);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1112) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN1_ErrorStatus'
            constexpr static uint8_t OK = 0;
            constexpr static uint8_t WARN = 1;
            constexpr static uint8_t ERROR_PASSIVE = 2;
            constexpr static uint8_t BUS_OFF = 3;

            // Attributes of signal 'BrakeF_OD_CAN1_ErrorStatus'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_DiscardedTxMessages {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1111            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1111);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1111) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN1_DiscardedTxMessages'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
            constexpr static char SystemSignalLongSymbol[] = "BrakeF_OD_CAN1_DiscardedTxMessages";
        };
        class BrakeF_OD_CAN1_Status {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1110            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1110);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1110) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN1_Status'
            constexpr static uint8_t RESET = 0;
            constexpr static uint8_t READY = 1;
            constexpr static uint8_t LISTENING = 2;
            constexpr static uint8_t SLEEP_PENDING = 3;
            constexpr static uint8_t SLEEP_ACTIVE = 4;
            constexpr static uint8_t ERROR = 5;

            // Attributes of signal 'BrakeF_OD_CAN1_Status'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_Baudrate {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1108            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint16_t min = static_cast<uint16_t>(125);
            constexpr static uint16_t max = static_cast<uint16_t>(1000);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1108);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1108) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN1_Baudrate'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1000.0f;
            constexpr static float CANzero_SDO_Default = 1000.0f;
        };
        class BrakeF_OD_CAN1_autoErrorReset {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1107            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1107);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1107) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN1_autoErrorReset'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'BrakeF_OD_CAN1_autoErrorReset'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class BrakeF_OD_CAN1_lastErrorCode {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1106            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1106);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1106) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_CAN1_lastErrorCode'
            constexpr static uint32_t NO_ERROR = 0;
            constexpr static uint32_t STUFF_ERROR = 1;
            constexpr static uint32_t FORM_ERROR = 2;
            constexpr static uint32_t ACK_ERROR = 3;
            constexpr static uint32_t BIT_RECESSIVE_ERROR = 4;
            constexpr static uint32_t BIT_DOMINANT_ERROR = 5;
            constexpr static uint32_t CRC_ERROR = 6;

            // Attributes of signal 'BrakeF_OD_CAN1_lastErrorCode'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_RxErrCnt {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1105            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1105);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1105) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN1_RxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CAN1_TxErrCnt {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1104            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1104);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1104) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_CAN1_TxErrCnt'
            constexpr static char CANzero_SDO_Group[] = "CAN";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_BuildTime {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1073            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1073);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1073) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_BuildTime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_BuildDate {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1072            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1072);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFF0000ull;
                dlc = 6;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1072) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_BuildDate'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_ChipUID2 {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1057            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1057);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1057) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_ChipUID2'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_ChipUID1 {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1056            
            using dataType = uint64_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint64_t min = static_cast<uint64_t>(0);
            constexpr static uint64_t max = static_cast<uint64_t>(281474976710655);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint64_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1056);
                uint64_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFFFFFFFF0000ull;
                dlc = 8;
            }
            constexpr static inline uint64_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1056) {
                    while(1);
                }
                uint64_t value = static_cast<uint64_t>((intel & 0xFFFFFFFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_ChipUID1'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_SdcOut {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1046            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1046);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1046) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_SdcOut'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_SdcIn {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1045            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1045);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1045) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_SdcIn'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_runtime {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1044            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(16777215);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1044);
                uint32_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFFFF0000ull;
                dlc = 5;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1044) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_runtime'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_InputVoltage {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1043            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(65.535);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1043);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value) / (0.001f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1043) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.001f);
            }

            // Attributes of signal 'BrakeF_OD_InputVoltage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_BoardTemp {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1042            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(-30);
            constexpr static float max = static_cast<float>(625.35);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1042);
                uint16_t rawValue = static_cast<uint16_t>(STD_ROUND((value - (-30.0f)) / (0.01f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1042) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (0.01f) + (-30.0f);
            }

            // Attributes of signal 'BrakeF_OD_BoardTemp'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 3000.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_MemFree {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1041            
            using dataType = uint32_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static uint32_t min = static_cast<uint32_t>(0);
            constexpr static uint32_t max = static_cast<uint32_t>(262140);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint32_t value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1041);
                uint32_t rawValue = static_cast<uint32_t>((value) / (4));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint32_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1041) {
                    while(1);
                }
                uint32_t value = static_cast<uint32_t>((intel & 0xFFFF0000ull) >> 16);
                return value * (4);
            }

            // Attributes of signal 'BrakeF_OD_MemFree'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_CpuUsage {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1040            
            using dataType = float;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static float min = static_cast<float>(0);
            constexpr static float max = static_cast<float>(100);
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, float value) noexcept {
                if (value > max) {
                    value = max;
                }
                if (value < min) {
                    value = min;
                }
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1040);
                uint8_t rawValue = static_cast<uint8_t>(STD_ROUND((value) / (0.5f)));
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline float get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1040) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value * (0.5f);
            }

            // Attributes of signal 'BrakeF_OD_CpuUsage'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_OdEntrySendInterval {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 33            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 33);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 33) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_OdEntrySendInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 50.0f;
            constexpr static float CANzero_SDO_Default = 50.0f;
        };
        class BrakeF_OD_SendOdOnBootup {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 32            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 32);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 32) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_SendOdOnBootup'
            constexpr static uint8_t DISABLED = 0;
            constexpr static uint8_t ENABLED = 1;

            // Attributes of signal 'BrakeF_OD_SendOdOnBootup'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_HeartbeatInterval {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 16            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 16);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 16) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_HeartbeatInterval'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_WRITE;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 100.0f;
            constexpr static float CANzero_SDO_Default = 100.0f;
        };
        class BrakeF_OD_DbcVersion {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 5            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 5);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 5) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_DbcVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_StackVersion {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 4            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 4);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 4) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_StackVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_ProtocolVersion {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 3            
            using dataType = uint16_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint16_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 3);
                uint16_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFFFF0000ull;
                dlc = 4;
            }
            constexpr static inline uint16_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 3) {
                    while(1);
                }
                uint16_t value = static_cast<uint16_t>((intel & 0xFFFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_ProtocolVersion'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 1.0f;
            constexpr static float CANzero_SDO_Default = 1.0f;
        };
        class BrakeF_OD_NodeStatus {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 2            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 2);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 2) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Value table of signal 'BrakeF_OD_NodeStatus'
            constexpr static uint8_t BOOTUP = 0;
            constexpr static uint8_t STOPPED = 4;
            constexpr static uint8_t OPERATIONAL = 5;
            constexpr static uint8_t PREOPERATIONAL = 127;
            constexpr static uint8_t RESET = 128;

            // Attributes of signal 'BrakeF_OD_NodeStatus'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
        };
        class BrakeF_OD_NodeID {
            public:
            // This signal is multiplexed by BrakeF_SDO_ID == 1            
            using dataType = uint8_t;
            constexpr static uint8_t numIds = 2;
            constexpr static uint32_t ids[] = { 0x591, 0x5D1 };
            constexpr static inline void set(uint64_t& intel, uint64_t& motorola, uint8_t& dlc, uint8_t value) noexcept {
                BrakeF_SDO_ID::set(intel, motorola, dlc, 1);
                uint8_t rawValue = (value);
                intel |= (static_cast<uint64_t>(rawValue) << 16) & 0xFF0000ull;
                dlc = 3;
            }
            constexpr static inline uint8_t get(const uint64_t& intel, const uint64_t& motorola) noexcept {
                if (BrakeF_SDO_ID::get(intel, motorola) != 1) {
                    while(1);
                }
                uint8_t value = static_cast<uint8_t>((intel & 0xFF0000ull) >> 16);
                return value;
            }

            // Attributes of signal 'BrakeF_OD_NodeID'
            constexpr static char CANzero_SDO_Group[] = "";
            constexpr static CANzero_SDO_AccessType_t CANzero_SDO_AccessType = CANzero_SDO_AccessType_t::READ_ONLY;
            constexpr static CANzero_SDO_AccessIfOperational_t CANzero_SDO_AccessIfOperational = CANzero_SDO_AccessIfOperational_t::YES;
            constexpr static float GenSigStartValue = 0.0f;
            constexpr static float CANzero_SDO_Default = 0.0f;
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
    }

    /**********************************************************************************************
    * Namespace containing all messages                                                           *
    ***********************************************************************************************/
    namespace messages {
        class BrakeF_TX_PressureCooling {
            public:
            constexpr static uint32_t id = 0x410;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_TX_Pressure_Reservoir = signals::BrakeF_TX_Pressure_Reservoir;

            // Attributes of message 'BrakeF_TX_PressureCooling'
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
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using SensorF_TX_PodState = signals::SensorF_TX_PodState;
            using SensorF_TX_PodState_Last = signals::SensorF_TX_PodState_Last;
            using SensorF_TX_PodState_Target = signals::SensorF_TX_PodState_Target;

            // Attributes of message 'SensorF_TX_StatePod'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_EMCY {
            public:
            constexpr static uint32_t id = 0x91;
            constexpr static uint8_t dlc = 4;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_W0_highPressureActingChamber = signals::BrakeF_W0_highPressureActingChamber;
            using BrakeF_W1_highPressureRetractingChamber = signals::BrakeF_W1_highPressureRetractingChamber;
            using BrakeF_W2_enableWithAnError = signals::BrakeF_W2_enableWithAnError;
            using BrakeF_W3_externalError = signals::BrakeF_W3_externalError;
            using BrakeF_E0_pressureTooHigh = signals::BrakeF_E0_pressureTooHigh;
            using BrakeF_E1_pressureTooLow = signals::BrakeF_E1_pressureTooLow;
            using BrakeF_E2_commWatchdogTimeout = signals::BrakeF_E2_commWatchdogTimeout;
            using BrakeF_E3_retractUnsuccesful_errorFlag = signals::BrakeF_E3_retractUnsuccesful_errorFlag;
            using BrakeF_E4_retractUnsuccesful_notEnabled = signals::BrakeF_E4_retractUnsuccesful_notEnabled;
            using BrakeF_E5_retractUnsuccesful_openSDC = signals::BrakeF_E5_retractUnsuccesful_openSDC;

            // Attributes of message 'BrakeF_EMCY'
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
        class BrakeF_TX_PDO2 {
            public:
            constexpr static uint32_t id = 0x211;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO2'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_RX_PDO2 {
            public:
            constexpr static uint32_t id = 0x251;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_RX_PDO2'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO3 {
            public:
            constexpr static uint32_t id = 0x291;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO3'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_RX_PDO3 {
            public:
            constexpr static uint32_t id = 0x2D1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_RX_PDO3'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO4 {
            public:
            constexpr static uint32_t id = 0x311;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO4'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_RX_PDO4 {
            public:
            constexpr static uint32_t id = 0x351;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_RX_PDO4'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO5 {
            public:
            constexpr static uint32_t id = 0x391;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO5'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO6 {
            public:
            constexpr static uint32_t id = 0x3D1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO6'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_Pressure {
            public:
            constexpr static uint32_t id = 0x411;
            constexpr static uint8_t dlc = 6;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_TX_Pressure_Act = signals::BrakeF_TX_Pressure_Act;
            using BrakeF_TX_Pressure_Retract = signals::BrakeF_TX_Pressure_Retract;
            using BrakeF_TX_Pressure_Tank = signals::BrakeF_TX_Pressure_Tank;

            // Attributes of message 'BrakeF_TX_Pressure'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO8 {
            public:
            constexpr static uint32_t id = 0x451;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO8'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO9 {
            public:
            constexpr static uint32_t id = 0x491;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO9'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO10 {
            public:
            constexpr static uint32_t id = 0x4D1;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO10'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO11 {
            public:
            constexpr static uint32_t id = 0x511;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO11'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_TX_PDO12 {
            public:
            constexpr static uint32_t id = 0x551;
            constexpr static uint8_t dlc = 0;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_TX_PDO12'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_SDO_Resp {
            public:
            constexpr static uint32_t id = 0x591;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_SDO_ID = signals::BrakeF_SDO_ID;
            using BrakeF_SDO_RespCode = signals::BrakeF_SDO_RespCode;
            using BrakeF_OD_valveUpperTolerance = signals::BrakeF_OD_valveUpperTolerance;
            using BrakeF_OD_commWatchdog = signals::BrakeF_OD_commWatchdog;
            using BrakeF_OD_counterLimit = signals::BrakeF_OD_counterLimit;
            using BrakeF_OD_delay = signals::BrakeF_OD_delay;
            using BrakeF_OD_tankUpperControlLimit = signals::BrakeF_OD_tankUpperControlLimit;
            using BrakeF_OD_tankLowerControlLimit = signals::BrakeF_OD_tankLowerControlLimit;
            using BrakeF_OD_CAN2_DelayedTxMessages = signals::BrakeF_OD_CAN2_DelayedTxMessages;
            using BrakeF_OD_CAN2_ErrorStatus = signals::BrakeF_OD_CAN2_ErrorStatus;
            using BrakeF_OD_CAN2_DiscardedTxMessages = signals::BrakeF_OD_CAN2_DiscardedTxMessages;
            using BrakeF_OD_CAN2_Status = signals::BrakeF_OD_CAN2_Status;
            using BrakeF_OD_CAN2_Baudrate = signals::BrakeF_OD_CAN2_Baudrate;
            using BrakeF_OD_CAN2_autoErrorReset = signals::BrakeF_OD_CAN2_autoErrorReset;
            using BrakeF_OD_CAN2_lastErrorCode = signals::BrakeF_OD_CAN2_lastErrorCode;
            using BrakeF_OD_CAN2_RxErrCnt = signals::BrakeF_OD_CAN2_RxErrCnt;
            using BrakeF_OD_CAN2_TxErrCnt = signals::BrakeF_OD_CAN2_TxErrCnt;
            using BrakeF_OD_CAN1_DelayedTxMessages = signals::BrakeF_OD_CAN1_DelayedTxMessages;
            using BrakeF_OD_CAN1_ErrorStatus = signals::BrakeF_OD_CAN1_ErrorStatus;
            using BrakeF_OD_CAN1_DiscardedTxMessages = signals::BrakeF_OD_CAN1_DiscardedTxMessages;
            using BrakeF_OD_CAN1_Status = signals::BrakeF_OD_CAN1_Status;
            using BrakeF_OD_CAN1_Baudrate = signals::BrakeF_OD_CAN1_Baudrate;
            using BrakeF_OD_CAN1_autoErrorReset = signals::BrakeF_OD_CAN1_autoErrorReset;
            using BrakeF_OD_CAN1_lastErrorCode = signals::BrakeF_OD_CAN1_lastErrorCode;
            using BrakeF_OD_CAN1_RxErrCnt = signals::BrakeF_OD_CAN1_RxErrCnt;
            using BrakeF_OD_CAN1_TxErrCnt = signals::BrakeF_OD_CAN1_TxErrCnt;
            using BrakeF_OD_BuildTime = signals::BrakeF_OD_BuildTime;
            using BrakeF_OD_BuildDate = signals::BrakeF_OD_BuildDate;
            using BrakeF_OD_ChipUID2 = signals::BrakeF_OD_ChipUID2;
            using BrakeF_OD_ChipUID1 = signals::BrakeF_OD_ChipUID1;
            using BrakeF_OD_SdcOut = signals::BrakeF_OD_SdcOut;
            using BrakeF_OD_SdcIn = signals::BrakeF_OD_SdcIn;
            using BrakeF_OD_runtime = signals::BrakeF_OD_runtime;
            using BrakeF_OD_InputVoltage = signals::BrakeF_OD_InputVoltage;
            using BrakeF_OD_BoardTemp = signals::BrakeF_OD_BoardTemp;
            using BrakeF_OD_MemFree = signals::BrakeF_OD_MemFree;
            using BrakeF_OD_CpuUsage = signals::BrakeF_OD_CpuUsage;
            using BrakeF_OD_OdEntrySendInterval = signals::BrakeF_OD_OdEntrySendInterval;
            using BrakeF_OD_SendOdOnBootup = signals::BrakeF_OD_SendOdOnBootup;
            using BrakeF_OD_HeartbeatInterval = signals::BrakeF_OD_HeartbeatInterval;
            using BrakeF_OD_DbcVersion = signals::BrakeF_OD_DbcVersion;
            using BrakeF_OD_StackVersion = signals::BrakeF_OD_StackVersion;
            using BrakeF_OD_ProtocolVersion = signals::BrakeF_OD_ProtocolVersion;
            using BrakeF_OD_NodeStatus = signals::BrakeF_OD_NodeStatus;
            using BrakeF_OD_NodeID = signals::BrakeF_OD_NodeID;

            // Attributes of message 'BrakeF_SDO_Resp'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_SDO_Req_Up {
            public:
            constexpr static uint32_t id = 0x5D1;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_SDO_ID = signals::BrakeF_SDO_ID;
            using BrakeF_OD_valveUpperTolerance = signals::BrakeF_OD_valveUpperTolerance;
            using BrakeF_OD_commWatchdog = signals::BrakeF_OD_commWatchdog;
            using BrakeF_OD_counterLimit = signals::BrakeF_OD_counterLimit;
            using BrakeF_OD_delay = signals::BrakeF_OD_delay;
            using BrakeF_OD_tankUpperControlLimit = signals::BrakeF_OD_tankUpperControlLimit;
            using BrakeF_OD_tankLowerControlLimit = signals::BrakeF_OD_tankLowerControlLimit;
            using BrakeF_OD_CAN2_DelayedTxMessages = signals::BrakeF_OD_CAN2_DelayedTxMessages;
            using BrakeF_OD_CAN2_ErrorStatus = signals::BrakeF_OD_CAN2_ErrorStatus;
            using BrakeF_OD_CAN2_DiscardedTxMessages = signals::BrakeF_OD_CAN2_DiscardedTxMessages;
            using BrakeF_OD_CAN2_Status = signals::BrakeF_OD_CAN2_Status;
            using BrakeF_OD_CAN2_Baudrate = signals::BrakeF_OD_CAN2_Baudrate;
            using BrakeF_OD_CAN2_autoErrorReset = signals::BrakeF_OD_CAN2_autoErrorReset;
            using BrakeF_OD_CAN2_lastErrorCode = signals::BrakeF_OD_CAN2_lastErrorCode;
            using BrakeF_OD_CAN2_RxErrCnt = signals::BrakeF_OD_CAN2_RxErrCnt;
            using BrakeF_OD_CAN2_TxErrCnt = signals::BrakeF_OD_CAN2_TxErrCnt;
            using BrakeF_OD_CAN1_DelayedTxMessages = signals::BrakeF_OD_CAN1_DelayedTxMessages;
            using BrakeF_OD_CAN1_ErrorStatus = signals::BrakeF_OD_CAN1_ErrorStatus;
            using BrakeF_OD_CAN1_DiscardedTxMessages = signals::BrakeF_OD_CAN1_DiscardedTxMessages;
            using BrakeF_OD_CAN1_Status = signals::BrakeF_OD_CAN1_Status;
            using BrakeF_OD_CAN1_Baudrate = signals::BrakeF_OD_CAN1_Baudrate;
            using BrakeF_OD_CAN1_autoErrorReset = signals::BrakeF_OD_CAN1_autoErrorReset;
            using BrakeF_OD_CAN1_lastErrorCode = signals::BrakeF_OD_CAN1_lastErrorCode;
            using BrakeF_OD_CAN1_RxErrCnt = signals::BrakeF_OD_CAN1_RxErrCnt;
            using BrakeF_OD_CAN1_TxErrCnt = signals::BrakeF_OD_CAN1_TxErrCnt;
            using BrakeF_OD_BuildTime = signals::BrakeF_OD_BuildTime;
            using BrakeF_OD_BuildDate = signals::BrakeF_OD_BuildDate;
            using BrakeF_OD_ChipUID2 = signals::BrakeF_OD_ChipUID2;
            using BrakeF_OD_ChipUID1 = signals::BrakeF_OD_ChipUID1;
            using BrakeF_OD_SdcOut = signals::BrakeF_OD_SdcOut;
            using BrakeF_OD_SdcIn = signals::BrakeF_OD_SdcIn;
            using BrakeF_OD_runtime = signals::BrakeF_OD_runtime;
            using BrakeF_OD_InputVoltage = signals::BrakeF_OD_InputVoltage;
            using BrakeF_OD_BoardTemp = signals::BrakeF_OD_BoardTemp;
            using BrakeF_OD_MemFree = signals::BrakeF_OD_MemFree;
            using BrakeF_OD_CpuUsage = signals::BrakeF_OD_CpuUsage;
            using BrakeF_OD_OdEntrySendInterval = signals::BrakeF_OD_OdEntrySendInterval;
            using BrakeF_OD_SendOdOnBootup = signals::BrakeF_OD_SendOdOnBootup;
            using BrakeF_OD_HeartbeatInterval = signals::BrakeF_OD_HeartbeatInterval;
            using BrakeF_OD_DbcVersion = signals::BrakeF_OD_DbcVersion;
            using BrakeF_OD_StackVersion = signals::BrakeF_OD_StackVersion;
            using BrakeF_OD_ProtocolVersion = signals::BrakeF_OD_ProtocolVersion;
            using BrakeF_OD_NodeStatus = signals::BrakeF_OD_NodeStatus;
            using BrakeF_OD_NodeID = signals::BrakeF_OD_NodeID;

            // Attributes of message 'BrakeF_SDO_Req_Up'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_SDO_Req_Down {
            public:
            constexpr static uint32_t id = 0x611;
            constexpr static uint8_t dlc = 2;
            constexpr static bool isExtendedId = false;

            // Signals
            using BrakeF_SDO_ID = signals::BrakeF_SDO_ID;

            // Attributes of message 'BrakeF_SDO_Req_Down'
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
        class BrakeF_BTL_TX {
            public:
            constexpr static uint32_t id = 0x751;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_BTL_TX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };
        class BrakeF_BTL_RX {
            public:
            constexpr static uint32_t id = 0x791;
            constexpr static uint8_t dlc = 8;
            constexpr static bool isExtendedId = false;

            // Attributes of message 'BrakeF_BTL_RX'
            constexpr static uint16_t GenMsgCycleTime = 100;
        };

        // Aliases for all CANzero messages of the current node, excluding the node name in the message name
        // This allows writing node independent code by using e.g. can::messages::CANZERO_BTL_RX instead of can::messages::PDU_BTL_RX
        // Also if the user renamed a PDO (e.g. PDU_TX_PDO1 to PDU_TX_Current) the alias will still be TX_PDO1
        using CANZERO_EMCY = BrakeF_EMCY;
        using CANZERO_TX_PDO1 = BrakeF_TX_Status;
        using CANZERO_RX_PDO1 = BrakeF_RX_Control;
        using CANZERO_TX_PDO2 = BrakeF_TX_PDO2;
        using CANZERO_RX_PDO2 = BrakeF_RX_PDO2;
        using CANZERO_TX_PDO3 = BrakeF_TX_PDO3;
        using CANZERO_RX_PDO3 = BrakeF_RX_PDO3;
        using CANZERO_TX_PDO4 = BrakeF_TX_PDO4;
        using CANZERO_RX_PDO4 = BrakeF_RX_PDO4;
        using CANZERO_TX_PDO5 = BrakeF_TX_PDO5;
        using CANZERO_TX_PDO6 = BrakeF_TX_PDO6;
        using CANZERO_TX_PDO7 = BrakeF_TX_Pressure;
        using CANZERO_TX_PDO8 = BrakeF_TX_PDO8;
        using CANZERO_TX_PDO9 = BrakeF_TX_PDO9;
        using CANZERO_TX_PDO10 = BrakeF_TX_PDO10;
        using CANZERO_TX_PDO11 = BrakeF_TX_PDO11;
        using CANZERO_TX_PDO12 = BrakeF_TX_PDO12;
        using CANZERO_SDO_Resp = BrakeF_SDO_Resp;
        using CANZERO_SDO_Req_Up = BrakeF_SDO_Req_Up;
        using CANZERO_SDO_Req_Down = BrakeF_SDO_Req_Down;
        using CANZERO_Heartbeat = BrakeF_Heartbeat;
        using CANZERO_BTL_TX = BrakeF_BTL_TX;
        using CANZERO_BTL_RX = BrakeF_BTL_RX;
        
    }
}

#endif // DBCPARSER_DBC_PARSER_HPP