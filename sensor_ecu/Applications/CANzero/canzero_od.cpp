/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 17:02:25 18.05.2023.
 * It contains the object dictionary for the node 'SensorF'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#include "canzero_od.hpp"


/**************************************************************************
* Definition of all OD variables with default values from database.       *
***************************************************************************/
volatile uint8_t  OD_NodeID                   = can::signals::SensorF_OD_NodeID::CANzero_SDO_Default;
volatile uint8_t  OD_NodeStatus               = can::signals::SensorF_OD_NodeStatus::CANzero_SDO_Default;
volatile uint16_t OD_ProtocolVersion          = can::signals::SensorF_OD_ProtocolVersion::CANzero_SDO_Default;
volatile uint16_t OD_StackVersion             = can::signals::SensorF_OD_StackVersion::CANzero_SDO_Default;
volatile uint16_t OD_DbcVersion               = can::signals::SensorF_OD_DbcVersion::CANzero_SDO_Default;
volatile uint16_t OD_HeartbeatInterval        = can::signals::SensorF_OD_HeartbeatInterval::CANzero_SDO_Default;
volatile uint8_t  OD_SendOdOnBootup           = can::signals::SensorF_OD_SendOdOnBootup::CANzero_SDO_Default;
volatile uint16_t OD_OdEntrySendInterval      = can::signals::SensorF_OD_OdEntrySendInterval::CANzero_SDO_Default;
volatile float    OD_CpuUsage                 = can::signals::SensorF_OD_CpuUsage::CANzero_SDO_Default;
volatile uint32_t OD_MemFree                  = can::signals::SensorF_OD_MemFree::CANzero_SDO_Default;
volatile float    OD_BoardTemp                = can::signals::SensorF_OD_BoardTemp::CANzero_SDO_Default;
volatile float    OD_InputVoltage             = can::signals::SensorF_OD_InputVoltage::CANzero_SDO_Default;
volatile uint32_t OD_runtime                  = can::signals::SensorF_OD_runtime::CANzero_SDO_Default;
volatile uint8_t  OD_SdcIn                    = can::signals::SensorF_OD_SdcIn::CANzero_SDO_Default;
volatile uint8_t  OD_SdcOut                   = can::signals::SensorF_OD_SdcOut::CANzero_SDO_Default;
volatile uint64_t OD_ChipUID1                 = can::signals::SensorF_OD_ChipUID1::CANzero_SDO_Default;
volatile uint64_t OD_ChipUID2                 = can::signals::SensorF_OD_ChipUID2::CANzero_SDO_Default;
volatile uint32_t OD_BuildDate                = can::signals::SensorF_OD_BuildDate::CANzero_SDO_Default;
volatile uint32_t OD_BuildTime                = can::signals::SensorF_OD_BuildTime::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_TxErrCnt            = can::signals::SensorF_OD_CAN1_TxErrCnt::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_RxErrCnt            = can::signals::SensorF_OD_CAN1_RxErrCnt::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_lastErrorCode       = can::signals::SensorF_OD_CAN1_lastErrorCode::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_autoErrorReset      = can::signals::SensorF_OD_CAN1_autoErrorReset::CANzero_SDO_Default;
volatile uint16_t OD_CAN1_Baudrate            = can::signals::SensorF_OD_CAN1_Baudrate::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_Status              = can::signals::SensorF_OD_CAN1_Status::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_DiscardedTxMessages = can::signals::SensorF_OD_CAN1_DiscardedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_ErrorStatus         = can::signals::SensorF_OD_CAN1_ErrorStatus::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_DelayedTxMessages   = can::signals::SensorF_OD_CAN1_DelayedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_TxErrCnt            = can::signals::SensorF_OD_CAN2_TxErrCnt::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_RxErrCnt            = can::signals::SensorF_OD_CAN2_RxErrCnt::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_lastErrorCode       = can::signals::SensorF_OD_CAN2_lastErrorCode::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_autoErrorReset      = can::signals::SensorF_OD_CAN2_autoErrorReset::CANzero_SDO_Default;
volatile uint16_t OD_CAN2_Baudrate            = can::signals::SensorF_OD_CAN2_Baudrate::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_Status              = can::signals::SensorF_OD_CAN2_Status::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_DiscardedTxMessages = can::signals::SensorF_OD_CAN2_DiscardedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_ErrorStatus         = can::signals::SensorF_OD_CAN2_ErrorStatus::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_DelayedTxMessages   = can::signals::SensorF_OD_CAN2_DelayedTxMessages::CANzero_SDO_Default;
volatile float    OD_samplingInterval         = can::signals::SensorF_OD_samplingInterval::CANzero_SDO_Default;
volatile uint8_t  OD_TelemetryCommands        = can::signals::SensorF_OD_TelemetryCommands::CANzero_SDO_Default;
volatile uint8_t  OD_StateMachineInterval     = can::signals::SensorF_OD_StateMachineInterval::CANzero_SDO_Default;
volatile uint8_t  OD_StateMachineActivate     = can::signals::SensorF_OD_StateMachineActivate::CANzero_SDO_Default;
volatile uint8_t  OD_HVBatteryMode            = can::signals::SensorF_OD_HVBatteryMode::CANzero_SDO_Default;
volatile float    OD_EncoderWheelDiameter     = can::signals::SensorF_OD_EncoderWheelDiameter::CANzero_SDO_Default;
volatile uint8_t  OD_EncoderResetPosition     = can::signals::SensorF_OD_EncoderResetPosition::CANzero_SDO_Default;
volatile uint8_t  OD_SetReset                 = can::signals::SensorF_OD_SetReset::CANzero_SDO_Default;
volatile uint8_t  OD_IMU_number               = can::signals::SensorF_OD_IMU_number::CANzero_SDO_Default;
volatile float    OD_IMU1_Temperature         = can::signals::SensorF_OD_IMU1_Temperature::CANzero_SDO_Default;
volatile float    OD_IMU2_Temperature         = can::signals::SensorF_OD_IMU2_Temperature::CANzero_SDO_Default;
volatile float    OD_IMU3_Temperature         = can::signals::SensorF_OD_IMU3_Temperature::CANzero_SDO_Default;
volatile float    OD_IMU_AccelX               = can::signals::SensorF_OD_IMU_AccelX::CANzero_SDO_Default;
volatile float    OD_IMU_AccelY               = can::signals::SensorF_OD_IMU_AccelY::CANzero_SDO_Default;
volatile float    OD_IMU_AccelZ               = can::signals::SensorF_OD_IMU_AccelZ::CANzero_SDO_Default;
volatile float    OD_IMU_GyroX                = can::signals::SensorF_OD_IMU_GyroX::CANzero_SDO_Default;
volatile float    OD_IMU_GyroY                = can::signals::SensorF_OD_IMU_GyroY::CANzero_SDO_Default;
volatile float    OD_IMU_GyroZ                = can::signals::SensorF_OD_IMU_GyroZ::CANzero_SDO_Default;
volatile float    OD_CoolingPressure          = can::signals::SensorF_OD_CoolingPressure::CANzero_SDO_Default;
volatile float    OD_ReservoirTemperature     = can::signals::SensorF_OD_ReservoirTemperature::CANzero_SDO_Default;
volatile float    OD_Magnet_1_Temperature     = can::signals::SensorF_OD_Magnet_1_Temperature::CANzero_SDO_Default;
volatile float    OD_Magnet_2_Temperature     = can::signals::SensorF_OD_Magnet_2_Temperature::CANzero_SDO_Default;
volatile float    OD_Magnet_3_Temperature     = can::signals::SensorF_OD_Magnet_3_Temperature::CANzero_SDO_Default;
volatile float    OD_Magnet_4_Temperature     = can::signals::SensorF_OD_Magnet_4_Temperature::CANzero_SDO_Default;
volatile float    OD_Magnet_5_Temperature     = can::signals::SensorF_OD_Magnet_5_Temperature::CANzero_SDO_Default;
volatile float    OD_Magnet_6_Temperature     = can::signals::SensorF_OD_Magnet_6_Temperature::CANzero_SDO_Default;
volatile uint8_t  OD_MdbState                 = can::signals::SensorF_OD_MdbState::CANzero_SDO_Default;
volatile uint16_t OD_FiducialRightCounter     = can::signals::SensorF_OD_FiducialRightCounter::CANzero_SDO_Default;
volatile uint16_t OD_FiducialLeftCounter      = can::signals::SensorF_OD_FiducialLeftCounter::CANzero_SDO_Default;
volatile float    OD_Position                 = can::signals::SensorF_OD_Position::CANzero_SDO_Default;
volatile float    OD_Velocity                 = can::signals::SensorF_OD_Velocity::CANzero_SDO_Default;


/**************************************************************************
* Semaphores for access to OD values                                      *
***************************************************************************/
osMutexId_t mutex_OD_NodeID                   = osMutexNew(NULL);
osMutexId_t mutex_OD_NodeStatus               = osMutexNew(NULL);
osMutexId_t mutex_OD_ProtocolVersion          = osMutexNew(NULL);
osMutexId_t mutex_OD_StackVersion             = osMutexNew(NULL);
osMutexId_t mutex_OD_DbcVersion               = osMutexNew(NULL);
osMutexId_t mutex_OD_HeartbeatInterval        = osMutexNew(NULL);
osMutexId_t mutex_OD_SendOdOnBootup           = osMutexNew(NULL);
osMutexId_t mutex_OD_OdEntrySendInterval      = osMutexNew(NULL);
osMutexId_t mutex_OD_CpuUsage                 = osMutexNew(NULL);
osMutexId_t mutex_OD_MemFree                  = osMutexNew(NULL);
osMutexId_t mutex_OD_BoardTemp                = osMutexNew(NULL);
osMutexId_t mutex_OD_InputVoltage             = osMutexNew(NULL);
osMutexId_t mutex_OD_runtime                  = osMutexNew(NULL);
osMutexId_t mutex_OD_SdcIn                    = osMutexNew(NULL);
osMutexId_t mutex_OD_SdcOut                   = osMutexNew(NULL);
osMutexId_t mutex_OD_ChipUID1                 = osMutexNew(NULL);
osMutexId_t mutex_OD_ChipUID2                 = osMutexNew(NULL);
osMutexId_t mutex_OD_BuildDate                = osMutexNew(NULL);
osMutexId_t mutex_OD_BuildTime                = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_TxErrCnt            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_RxErrCnt            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_lastErrorCode       = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_autoErrorReset      = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_Baudrate            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_Status              = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_DiscardedTxMessages = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_ErrorStatus         = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN1_DelayedTxMessages   = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_TxErrCnt            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_RxErrCnt            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_lastErrorCode       = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_autoErrorReset      = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_Baudrate            = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_Status              = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_DiscardedTxMessages = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_ErrorStatus         = osMutexNew(NULL);
osMutexId_t mutex_OD_CAN2_DelayedTxMessages   = osMutexNew(NULL);
osMutexId_t mutex_OD_samplingInterval         = osMutexNew(NULL);
osMutexId_t mutex_OD_TelemetryCommands        = osMutexNew(NULL);
osMutexId_t mutex_OD_StateMachineInterval     = osMutexNew(NULL);
osMutexId_t mutex_OD_StateMachineActivate     = osMutexNew(NULL);
osMutexId_t mutex_OD_HVBatteryMode            = osMutexNew(NULL);
osMutexId_t mutex_OD_EncoderWheelDiameter     = osMutexNew(NULL);
osMutexId_t mutex_OD_EncoderResetPosition     = osMutexNew(NULL);
osMutexId_t mutex_OD_SetReset                 = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_number               = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU1_Temperature         = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU2_Temperature         = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU3_Temperature         = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_AccelX               = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_AccelY               = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_AccelZ               = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_GyroX                = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_GyroY                = osMutexNew(NULL);
osMutexId_t mutex_OD_IMU_GyroZ                = osMutexNew(NULL);
osMutexId_t mutex_OD_CoolingPressure          = osMutexNew(NULL);
osMutexId_t mutex_OD_ReservoirTemperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_1_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_2_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_3_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_4_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_5_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_Magnet_6_Temperature     = osMutexNew(NULL);
osMutexId_t mutex_OD_MdbState                 = osMutexNew(NULL);
osMutexId_t mutex_OD_FiducialRightCounter     = osMutexNew(NULL);
osMutexId_t mutex_OD_FiducialLeftCounter      = osMutexNew(NULL);
osMutexId_t mutex_OD_Position                 = osMutexNew(NULL);
osMutexId_t mutex_OD_Velocity                 = osMutexNew(NULL);


/**************************************************************************
* Functions to handle a SDO download and upload request.                  *
***************************************************************************/
void handleSDORequestDownload(const RxMessage& rxMsgSdoReq) {
    can::Message<can::messages::SensorF_SDO_Req_Down> msgSdoReq(rxMsgSdoReq);
    uint16_t sdoId = msgSdoReq.get<can::signals::SensorF_SDO_ID>();
    handleSDORequestDownloadBySDOID(sdoId);
}
void handleSDORequestDownloadBySDOID(const uint16_t sdoId) {    
    can::Message<can::messages::SensorF_SDO_Resp> msgSdoResp;
    uint8_t respCode = can::signals::SensorF_SDO_RespCode::ERR_NON_EXISTING_OBJECT;

    switch (sdoId) {
        case 0x1:    // OD_NodeID
            msgSdoResp.set<can::signals::SensorF_OD_NodeID>(OD_NodeID_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x2:    // OD_NodeStatus
            msgSdoResp.set<can::signals::SensorF_OD_NodeStatus>(OD_NodeStatus_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x3:    // OD_ProtocolVersion
            msgSdoResp.set<can::signals::SensorF_OD_ProtocolVersion>(OD_ProtocolVersion_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x4:    // OD_StackVersion
            msgSdoResp.set<can::signals::SensorF_OD_StackVersion>(OD_StackVersion_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x5:    // OD_DbcVersion
            msgSdoResp.set<can::signals::SensorF_OD_DbcVersion>(OD_DbcVersion_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x10:    // OD_HeartbeatInterval
            msgSdoResp.set<can::signals::SensorF_OD_HeartbeatInterval>(OD_HeartbeatInterval_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x20:    // OD_SendOdOnBootup
            msgSdoResp.set<can::signals::SensorF_OD_SendOdOnBootup>(OD_SendOdOnBootup_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x21:    // OD_OdEntrySendInterval
            msgSdoResp.set<can::signals::SensorF_OD_OdEntrySendInterval>(OD_OdEntrySendInterval_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x410:    // OD_CpuUsage
            msgSdoResp.set<can::signals::SensorF_OD_CpuUsage>(OD_CpuUsage_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x411:    // OD_MemFree
            msgSdoResp.set<can::signals::SensorF_OD_MemFree>(OD_MemFree_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x412:    // OD_BoardTemp
            msgSdoResp.set<can::signals::SensorF_OD_BoardTemp>(OD_BoardTemp_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x413:    // OD_InputVoltage
            msgSdoResp.set<can::signals::SensorF_OD_InputVoltage>(OD_InputVoltage_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x414:    // OD_runtime
            msgSdoResp.set<can::signals::SensorF_OD_runtime>(OD_runtime_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x415:    // OD_SdcIn
            msgSdoResp.set<can::signals::SensorF_OD_SdcIn>(OD_SdcIn_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x416:    // OD_SdcOut
            msgSdoResp.set<can::signals::SensorF_OD_SdcOut>(OD_SdcOut_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x420:    // OD_ChipUID1
            msgSdoResp.set<can::signals::SensorF_OD_ChipUID1>(OD_ChipUID1_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x421:    // OD_ChipUID2
            msgSdoResp.set<can::signals::SensorF_OD_ChipUID2>(OD_ChipUID2_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x430:    // OD_BuildDate
            msgSdoResp.set<can::signals::SensorF_OD_BuildDate>(OD_BuildDate_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x431:    // OD_BuildTime
            msgSdoResp.set<can::signals::SensorF_OD_BuildTime>(OD_BuildTime_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x450:    // OD_CAN1_TxErrCnt
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_TxErrCnt>(OD_CAN1_TxErrCnt_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x451:    // OD_CAN1_RxErrCnt
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_RxErrCnt>(OD_CAN1_RxErrCnt_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x452:    // OD_CAN1_lastErrorCode
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_lastErrorCode>(OD_CAN1_lastErrorCode_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x453:    // OD_CAN1_autoErrorReset
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_autoErrorReset>(OD_CAN1_autoErrorReset_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x454:    // OD_CAN1_Baudrate
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_Baudrate>(OD_CAN1_Baudrate_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x456:    // OD_CAN1_Status
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_Status>(OD_CAN1_Status_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x457:    // OD_CAN1_DiscardedTxMessages
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_DiscardedTxMessages>(OD_CAN1_DiscardedTxMessages_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x458:    // OD_CAN1_ErrorStatus
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_ErrorStatus>(OD_CAN1_ErrorStatus_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x459:    // OD_CAN1_DelayedTxMessages
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_DelayedTxMessages>(OD_CAN1_DelayedTxMessages_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x460:    // OD_CAN2_TxErrCnt
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_TxErrCnt>(OD_CAN2_TxErrCnt_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x461:    // OD_CAN2_RxErrCnt
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_RxErrCnt>(OD_CAN2_RxErrCnt_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x462:    // OD_CAN2_lastErrorCode
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_lastErrorCode>(OD_CAN2_lastErrorCode_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x463:    // OD_CAN2_autoErrorReset
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_autoErrorReset>(OD_CAN2_autoErrorReset_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x464:    // OD_CAN2_Baudrate
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_Baudrate>(OD_CAN2_Baudrate_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x466:    // OD_CAN2_Status
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_Status>(OD_CAN2_Status_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x467:    // OD_CAN2_DiscardedTxMessages
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_DiscardedTxMessages>(OD_CAN2_DiscardedTxMessages_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x468:    // OD_CAN2_ErrorStatus
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_ErrorStatus>(OD_CAN2_ErrorStatus_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x469:    // OD_CAN2_DelayedTxMessages
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_DelayedTxMessages>(OD_CAN2_DelayedTxMessages_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x800:    // OD_samplingInterval
            msgSdoResp.set<can::signals::SensorF_OD_samplingInterval>(OD_samplingInterval_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x900:    // OD_TelemetryCommands
            msgSdoResp.set<can::signals::SensorF_OD_TelemetryCommands>(OD_TelemetryCommands_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x901:    // OD_StateMachineInterval
            msgSdoResp.set<can::signals::SensorF_OD_StateMachineInterval>(OD_StateMachineInterval_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x902:    // OD_StateMachineActivate
            msgSdoResp.set<can::signals::SensorF_OD_StateMachineActivate>(OD_StateMachineActivate_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x903:    // OD_HVBatteryMode
            msgSdoResp.set<can::signals::SensorF_OD_HVBatteryMode>(OD_HVBatteryMode_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x904:    // OD_EncoderWheelDiameter
            msgSdoResp.set<can::signals::SensorF_OD_EncoderWheelDiameter>(OD_EncoderWheelDiameter_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0x905:    // OD_EncoderResetPosition
            respCode = can::signals::SensorF_SDO_RespCode::ERR_WRITE_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(0x905);
            break;
        case 0x910:    // OD_SetReset
            respCode = can::signals::SensorF_SDO_RespCode::ERR_WRITE_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(0x910);
            break;
        case 0xA20:    // OD_IMU_number
            msgSdoResp.set<can::signals::SensorF_OD_IMU_number>(OD_IMU_number_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA25:    // OD_IMU1_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_IMU1_Temperature>(OD_IMU1_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA26:    // OD_IMU2_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_IMU2_Temperature>(OD_IMU2_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA27:    // OD_IMU3_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_IMU3_Temperature>(OD_IMU3_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA28:    // OD_IMU_AccelX
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelX>(OD_IMU_AccelX_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA29:    // OD_IMU_AccelY
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelY>(OD_IMU_AccelY_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA30:    // OD_IMU_AccelZ
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelZ>(OD_IMU_AccelZ_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA31:    // OD_IMU_GyroX
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroX>(OD_IMU_GyroX_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA32:    // OD_IMU_GyroY
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroY>(OD_IMU_GyroY_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xA33:    // OD_IMU_GyroZ
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroZ>(OD_IMU_GyroZ_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB00:    // OD_CoolingPressure
            msgSdoResp.set<can::signals::SensorF_OD_CoolingPressure>(OD_CoolingPressure_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB01:    // OD_ReservoirTemperature
            msgSdoResp.set<can::signals::SensorF_OD_ReservoirTemperature>(OD_ReservoirTemperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB02:    // OD_Magnet_1_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_1_Temperature>(OD_Magnet_1_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB04:    // OD_Magnet_2_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_2_Temperature>(OD_Magnet_2_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB05:    // OD_Magnet_3_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_3_Temperature>(OD_Magnet_3_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB06:    // OD_Magnet_4_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_4_Temperature>(OD_Magnet_4_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB07:    // OD_Magnet_5_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_5_Temperature>(OD_Magnet_5_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB08:    // OD_Magnet_6_Temperature
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_6_Temperature>(OD_Magnet_6_Temperature_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xB10:    // OD_MdbState
            msgSdoResp.set<can::signals::SensorF_OD_MdbState>(OD_MdbState_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xC00:    // OD_FiducialRightCounter
            msgSdoResp.set<can::signals::SensorF_OD_FiducialRightCounter>(OD_FiducialRightCounter_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xC01:    // OD_FiducialLeftCounter
            msgSdoResp.set<can::signals::SensorF_OD_FiducialLeftCounter>(OD_FiducialLeftCounter_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xD00:    // OD_Position
            msgSdoResp.set<can::signals::SensorF_OD_Position>(OD_Position_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        case 0xD01:    // OD_Velocity
            msgSdoResp.set<can::signals::SensorF_OD_Velocity>(OD_Velocity_get());
            respCode = can::signals::SensorF_SDO_RespCode::OK;
            break;
        default:
            // Unknown SDO-ID, just reply unknown ID
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(sdoId);
            break;
    }

    msgSdoResp.set<can::signals::SensorF_SDO_RespCode>(respCode);

    // Send response message
    extern osMessageQueueId_t czSendQueue;
    TxMessage sendTxMessage = msgSdoResp.getTxMessage();
    osMessageQueuePut(czSendQueue, &sendTxMessage, 0, 0);
}

void handleSDORequestUpload(const RxMessage& rxMsgSdoReq) {
    can::Message<can::messages::SensorF_SDO_Req_Up> msgSdoReq(rxMsgSdoReq);
    can::Message<can::messages::SensorF_SDO_Resp> msgSdoResp;
    uint8_t respCode = can::signals::SensorF_SDO_RespCode::ERR_NON_EXISTING_OBJECT;
    uint16_t sdoId = msgSdoReq.get<can::signals::SensorF_SDO_ID>();

    switch (sdoId) {
        case 0x1: {   // OD_NodeID
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_NodeID>(OD_NodeID_get());
            break;
        }
        case 0x2: {   // OD_NodeStatus
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_NodeStatus>(OD_NodeStatus_get());
            break;
        }
        case 0x3: {   // OD_ProtocolVersion
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_ProtocolVersion>(OD_ProtocolVersion_get());
            break;
        }
        case 0x4: {   // OD_StackVersion
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_StackVersion>(OD_StackVersion_get());
            break;
        }
        case 0x5: {   // OD_DbcVersion
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_DbcVersion>(OD_DbcVersion_get());
            break;
        }
        case 0x10: {   // OD_HeartbeatInterval
            uint16_t value = msgSdoReq.get<can::signals::SensorF_OD_HeartbeatInterval>();
                OD_HeartbeatInterval_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_HeartbeatInterval>(OD_HeartbeatInterval_get());
            break;
        }
        case 0x20: {   // OD_SendOdOnBootup
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_SendOdOnBootup>();
                OD_SendOdOnBootup_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_SendOdOnBootup>(OD_SendOdOnBootup_get());
            break;
        }
        case 0x21: {   // OD_OdEntrySendInterval
            uint16_t value = msgSdoReq.get<can::signals::SensorF_OD_OdEntrySendInterval>();
                OD_OdEntrySendInterval_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_OdEntrySendInterval>(OD_OdEntrySendInterval_get());
            break;
        }
        case 0x410: {   // OD_CpuUsage
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CpuUsage>(OD_CpuUsage_get());
            break;
        }
        case 0x411: {   // OD_MemFree
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_MemFree>(OD_MemFree_get());
            break;
        }
        case 0x412: {   // OD_BoardTemp
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_BoardTemp>(OD_BoardTemp_get());
            break;
        }
        case 0x413: {   // OD_InputVoltage
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_InputVoltage>(OD_InputVoltage_get());
            break;
        }
        case 0x414: {   // OD_runtime
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_runtime>(OD_runtime_get());
            break;
        }
        case 0x415: {   // OD_SdcIn
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_SdcIn>(OD_SdcIn_get());
            break;
        }
        case 0x416: {   // OD_SdcOut
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_SdcOut>(OD_SdcOut_get());
            break;
        }
        case 0x420: {   // OD_ChipUID1
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_ChipUID1>(OD_ChipUID1_get());
            break;
        }
        case 0x421: {   // OD_ChipUID2
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_ChipUID2>(OD_ChipUID2_get());
            break;
        }
        case 0x430: {   // OD_BuildDate
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_BuildDate>(OD_BuildDate_get());
            break;
        }
        case 0x431: {   // OD_BuildTime
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_BuildTime>(OD_BuildTime_get());
            break;
        }
        case 0x450: {   // OD_CAN1_TxErrCnt
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_TxErrCnt>(OD_CAN1_TxErrCnt_get());
            break;
        }
        case 0x451: {   // OD_CAN1_RxErrCnt
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_RxErrCnt>(OD_CAN1_RxErrCnt_get());
            break;
        }
        case 0x452: {   // OD_CAN1_lastErrorCode
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_lastErrorCode>(OD_CAN1_lastErrorCode_get());
            break;
        }
        case 0x453: {   // OD_CAN1_autoErrorReset
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_CAN1_autoErrorReset>();
                OD_CAN1_autoErrorReset_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_autoErrorReset>(OD_CAN1_autoErrorReset_get());
            break;
        }
        case 0x454: {   // OD_CAN1_Baudrate
            uint16_t value = msgSdoReq.get<can::signals::SensorF_OD_CAN1_Baudrate>();
if (value < 125 || value > 1000) {
                respCode = can::signals::SensorF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_CAN1_Baudrate_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_Baudrate>(OD_CAN1_Baudrate_get());
            break;
        }
        case 0x456: {   // OD_CAN1_Status
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_Status>(OD_CAN1_Status_get());
            break;
        }
        case 0x457: {   // OD_CAN1_DiscardedTxMessages
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_DiscardedTxMessages>(OD_CAN1_DiscardedTxMessages_get());
            break;
        }
        case 0x458: {   // OD_CAN1_ErrorStatus
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_ErrorStatus>(OD_CAN1_ErrorStatus_get());
            break;
        }
        case 0x459: {   // OD_CAN1_DelayedTxMessages
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN1_DelayedTxMessages>(OD_CAN1_DelayedTxMessages_get());
            break;
        }
        case 0x460: {   // OD_CAN2_TxErrCnt
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_TxErrCnt>(OD_CAN2_TxErrCnt_get());
            break;
        }
        case 0x461: {   // OD_CAN2_RxErrCnt
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_RxErrCnt>(OD_CAN2_RxErrCnt_get());
            break;
        }
        case 0x462: {   // OD_CAN2_lastErrorCode
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_lastErrorCode>(OD_CAN2_lastErrorCode_get());
            break;
        }
        case 0x463: {   // OD_CAN2_autoErrorReset
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_CAN2_autoErrorReset>();
                OD_CAN2_autoErrorReset_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_autoErrorReset>(OD_CAN2_autoErrorReset_get());
            break;
        }
        case 0x464: {   // OD_CAN2_Baudrate
            uint16_t value = msgSdoReq.get<can::signals::SensorF_OD_CAN2_Baudrate>();
if (value < 125 || value > 1000) {
                respCode = can::signals::SensorF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_CAN2_Baudrate_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_Baudrate>(OD_CAN2_Baudrate_get());
            break;
        }
        case 0x466: {   // OD_CAN2_Status
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_Status>(OD_CAN2_Status_get());
            break;
        }
        case 0x467: {   // OD_CAN2_DiscardedTxMessages
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_DiscardedTxMessages>(OD_CAN2_DiscardedTxMessages_get());
            break;
        }
        case 0x468: {   // OD_CAN2_ErrorStatus
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_ErrorStatus>(OD_CAN2_ErrorStatus_get());
            break;
        }
        case 0x469: {   // OD_CAN2_DelayedTxMessages
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CAN2_DelayedTxMessages>(OD_CAN2_DelayedTxMessages_get());
            break;
        }
        case 0x800: {   // OD_samplingInterval
            float value = msgSdoReq.get<can::signals::SensorF_OD_samplingInterval>();
if (value < 0.01f || value > 100.0f) {
                respCode = can::signals::SensorF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_samplingInterval_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::SensorF_OD_samplingInterval>(OD_samplingInterval_get());
            break;
        }
        case 0x900: {   // OD_TelemetryCommands
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_TelemetryCommands>();
                OD_TelemetryCommands_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_TelemetryCommands>(OD_TelemetryCommands_get());
            break;
        }
        case 0x901: {   // OD_StateMachineInterval
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_StateMachineInterval>();
if (value < 5 || value > 100) {
                respCode = can::signals::SensorF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_StateMachineInterval_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::SensorF_OD_StateMachineInterval>(OD_StateMachineInterval_get());
            break;
        }
        case 0x902: {   // OD_StateMachineActivate
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_StateMachineActivate>();
                OD_StateMachineActivate_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_StateMachineActivate>(OD_StateMachineActivate_get());
            break;
        }
        case 0x903: {   // OD_HVBatteryMode
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_HVBatteryMode>();
                OD_HVBatteryMode_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_HVBatteryMode>(OD_HVBatteryMode_get());
            break;
        }
        case 0x904: {   // OD_EncoderWheelDiameter
            float value = msgSdoReq.get<can::signals::SensorF_OD_EncoderWheelDiameter>();
if (value < 1.0f || value > 300.0f) {
                respCode = can::signals::SensorF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_EncoderWheelDiameter_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::SensorF_OD_EncoderWheelDiameter>(OD_EncoderWheelDiameter_get());
            break;
        }
        case 0x905: {   // OD_EncoderResetPosition
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_EncoderResetPosition>();
                OD_EncoderResetPosition_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(0x905);
            break;
        }
        case 0x910: {   // OD_SetReset
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_SetReset>();
                OD_SetReset_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(0x910);
            break;
        }
        case 0xA20: {   // OD_IMU_number
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_number>(OD_IMU_number_get());
            break;
        }
        case 0xA25: {   // OD_IMU1_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU1_Temperature>(OD_IMU1_Temperature_get());
            break;
        }
        case 0xA26: {   // OD_IMU2_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU2_Temperature>(OD_IMU2_Temperature_get());
            break;
        }
        case 0xA27: {   // OD_IMU3_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU3_Temperature>(OD_IMU3_Temperature_get());
            break;
        }
        case 0xA28: {   // OD_IMU_AccelX
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelX>(OD_IMU_AccelX_get());
            break;
        }
        case 0xA29: {   // OD_IMU_AccelY
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelY>(OD_IMU_AccelY_get());
            break;
        }
        case 0xA30: {   // OD_IMU_AccelZ
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_AccelZ>(OD_IMU_AccelZ_get());
            break;
        }
        case 0xA31: {   // OD_IMU_GyroX
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroX>(OD_IMU_GyroX_get());
            break;
        }
        case 0xA32: {   // OD_IMU_GyroY
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroY>(OD_IMU_GyroY_get());
            break;
        }
        case 0xA33: {   // OD_IMU_GyroZ
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_IMU_GyroZ>(OD_IMU_GyroZ_get());
            break;
        }
        case 0xB00: {   // OD_CoolingPressure
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_CoolingPressure>(OD_CoolingPressure_get());
            break;
        }
        case 0xB01: {   // OD_ReservoirTemperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_ReservoirTemperature>(OD_ReservoirTemperature_get());
            break;
        }
        case 0xB02: {   // OD_Magnet_1_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_1_Temperature>(OD_Magnet_1_Temperature_get());
            break;
        }
        case 0xB04: {   // OD_Magnet_2_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_2_Temperature>(OD_Magnet_2_Temperature_get());
            break;
        }
        case 0xB05: {   // OD_Magnet_3_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_3_Temperature>(OD_Magnet_3_Temperature_get());
            break;
        }
        case 0xB06: {   // OD_Magnet_4_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_4_Temperature>(OD_Magnet_4_Temperature_get());
            break;
        }
        case 0xB07: {   // OD_Magnet_5_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_5_Temperature>(OD_Magnet_5_Temperature_get());
            break;
        }
        case 0xB08: {   // OD_Magnet_6_Temperature
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Magnet_6_Temperature>(OD_Magnet_6_Temperature_get());
            break;
        }
        case 0xB10: {   // OD_MdbState
            uint8_t value = msgSdoReq.get<can::signals::SensorF_OD_MdbState>();
                OD_MdbState_set(value);
                respCode = can::signals::SensorF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::SensorF_OD_MdbState>(OD_MdbState_get());
            break;
        }
        case 0xC00: {   // OD_FiducialRightCounter
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_FiducialRightCounter>(OD_FiducialRightCounter_get());
            break;
        }
        case 0xC01: {   // OD_FiducialLeftCounter
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_FiducialLeftCounter>(OD_FiducialLeftCounter_get());
            break;
        }
        case 0xD00: {   // OD_Position
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Position>(OD_Position_get());
            break;
        }
        case 0xD01: {   // OD_Velocity
            respCode = can::signals::SensorF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::SensorF_OD_Velocity>(OD_Velocity_get());
            break;
        }
        default:
            // Unknown SDO-ID, just reply unknown ID
            msgSdoResp.set<can::signals::SensorF_SDO_ID>(sdoId);
            break;
    }

    msgSdoResp.set<can::signals::SensorF_SDO_RespCode>(respCode);

    // Send response message
    extern osMessageQueueId_t czSendQueue;
    TxMessage sendTxMessage = msgSdoResp.getTxMessage();
    osMessageQueuePut(czSendQueue, &sendTxMessage, 0, 0);
}


// This is needed for the Windows State Machine Simulation Environment
// Because the windows compiler does not support weak symbols
#ifndef _WIN32
#define WEAK_SYMBOL __attribute__((weak))
#else
#define WEAK_SYMBOL
#endif


/**************************************************************************
* Functions for setting and getting an OD entry.                          *
* They are defined weak, so can be overwritten in an external file.       *
* They can be overwritten, e.g. to read a value directly from hardware    *
* or to trigger another function (e.g. enter debug mode).                 *
* ATTENTION: Then the threadsafe access has to be handled by the user,    *
* e.g with using the provided mutex.                                      *
***************************************************************************/
#ifndef OD_NodeID_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_NodeID_get() {
    osMutexAcquire(mutex_OD_NodeID, portMAX_DELAY);
    uint8_t value = OD_NodeID;
    osMutexRelease(mutex_OD_NodeID);
    return value;
}
#endif
#ifndef OD_NodeID_SET_OVERWRITE
void WEAK_SYMBOL OD_NodeID_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_NodeID, portMAX_DELAY);
    OD_NodeID = value;
    osMutexRelease(mutex_OD_NodeID);
}
#endif

#ifndef OD_NodeStatus_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_NodeStatus_get() {
    osMutexAcquire(mutex_OD_NodeStatus, portMAX_DELAY);
    uint8_t value = OD_NodeStatus;
    osMutexRelease(mutex_OD_NodeStatus);
    return value;
}
#endif
#ifndef OD_NodeStatus_SET_OVERWRITE
void WEAK_SYMBOL OD_NodeStatus_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_NodeStatus, portMAX_DELAY);
    OD_NodeStatus = value;
    osMutexRelease(mutex_OD_NodeStatus);
}
#endif

#ifndef OD_ProtocolVersion_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_ProtocolVersion_get() {
    osMutexAcquire(mutex_OD_ProtocolVersion, portMAX_DELAY);
    uint16_t value = OD_ProtocolVersion;
    osMutexRelease(mutex_OD_ProtocolVersion);
    return value;
}
#endif
#ifndef OD_ProtocolVersion_SET_OVERWRITE
void WEAK_SYMBOL OD_ProtocolVersion_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_ProtocolVersion, portMAX_DELAY);
    OD_ProtocolVersion = value;
    osMutexRelease(mutex_OD_ProtocolVersion);
}
#endif

#ifndef OD_StackVersion_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_StackVersion_get() {
    osMutexAcquire(mutex_OD_StackVersion, portMAX_DELAY);
    uint16_t value = OD_StackVersion;
    osMutexRelease(mutex_OD_StackVersion);
    return value;
}
#endif
#ifndef OD_StackVersion_SET_OVERWRITE
void WEAK_SYMBOL OD_StackVersion_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_StackVersion, portMAX_DELAY);
    OD_StackVersion = value;
    osMutexRelease(mutex_OD_StackVersion);
}
#endif

#ifndef OD_DbcVersion_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_DbcVersion_get() {
    osMutexAcquire(mutex_OD_DbcVersion, portMAX_DELAY);
    uint16_t value = OD_DbcVersion;
    osMutexRelease(mutex_OD_DbcVersion);
    return value;
}
#endif
#ifndef OD_DbcVersion_SET_OVERWRITE
void WEAK_SYMBOL OD_DbcVersion_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_DbcVersion, portMAX_DELAY);
    OD_DbcVersion = value;
    osMutexRelease(mutex_OD_DbcVersion);
}
#endif

#ifndef OD_HeartbeatInterval_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_HeartbeatInterval_get() {
    osMutexAcquire(mutex_OD_HeartbeatInterval, portMAX_DELAY);
    uint16_t value = OD_HeartbeatInterval;
    osMutexRelease(mutex_OD_HeartbeatInterval);
    return value;
}
#endif
#ifndef OD_HeartbeatInterval_SET_OVERWRITE
void WEAK_SYMBOL OD_HeartbeatInterval_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_HeartbeatInterval, portMAX_DELAY);
    OD_HeartbeatInterval = value;
    osMutexRelease(mutex_OD_HeartbeatInterval);
}
#endif

#ifndef OD_SendOdOnBootup_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_SendOdOnBootup_get() {
    osMutexAcquire(mutex_OD_SendOdOnBootup, portMAX_DELAY);
    uint8_t value = OD_SendOdOnBootup;
    osMutexRelease(mutex_OD_SendOdOnBootup);
    return value;
}
#endif
#ifndef OD_SendOdOnBootup_SET_OVERWRITE
void WEAK_SYMBOL OD_SendOdOnBootup_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_SendOdOnBootup, portMAX_DELAY);
    OD_SendOdOnBootup = value;
    osMutexRelease(mutex_OD_SendOdOnBootup);
}
#endif

#ifndef OD_OdEntrySendInterval_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_OdEntrySendInterval_get() {
    osMutexAcquire(mutex_OD_OdEntrySendInterval, portMAX_DELAY);
    uint16_t value = OD_OdEntrySendInterval;
    osMutexRelease(mutex_OD_OdEntrySendInterval);
    return value;
}
#endif
#ifndef OD_OdEntrySendInterval_SET_OVERWRITE
void WEAK_SYMBOL OD_OdEntrySendInterval_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_OdEntrySendInterval, portMAX_DELAY);
    OD_OdEntrySendInterval = value;
    osMutexRelease(mutex_OD_OdEntrySendInterval);
}
#endif

#ifndef OD_CpuUsage_GET_OVERWRITE
float WEAK_SYMBOL OD_CpuUsage_get() {
    osMutexAcquire(mutex_OD_CpuUsage, portMAX_DELAY);
    float value = OD_CpuUsage;
    osMutexRelease(mutex_OD_CpuUsage);
    return value;
}
#endif
#ifndef OD_CpuUsage_SET_OVERWRITE
void WEAK_SYMBOL OD_CpuUsage_set(const float value) {
    osMutexAcquire(mutex_OD_CpuUsage, portMAX_DELAY);
    OD_CpuUsage = value;
    osMutexRelease(mutex_OD_CpuUsage);
}
#endif

#ifndef OD_MemFree_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_MemFree_get() {
    osMutexAcquire(mutex_OD_MemFree, portMAX_DELAY);
    uint32_t value = OD_MemFree;
    osMutexRelease(mutex_OD_MemFree);
    return value;
}
#endif
#ifndef OD_MemFree_SET_OVERWRITE
void WEAK_SYMBOL OD_MemFree_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_MemFree, portMAX_DELAY);
    OD_MemFree = value;
    osMutexRelease(mutex_OD_MemFree);
}
#endif

#ifndef OD_BoardTemp_GET_OVERWRITE
float WEAK_SYMBOL OD_BoardTemp_get() {
    osMutexAcquire(mutex_OD_BoardTemp, portMAX_DELAY);
    float value = OD_BoardTemp;
    osMutexRelease(mutex_OD_BoardTemp);
    return value;
}
#endif
#ifndef OD_BoardTemp_SET_OVERWRITE
void WEAK_SYMBOL OD_BoardTemp_set(const float value) {
    osMutexAcquire(mutex_OD_BoardTemp, portMAX_DELAY);
    OD_BoardTemp = value;
    osMutexRelease(mutex_OD_BoardTemp);
}
#endif

#ifndef OD_InputVoltage_GET_OVERWRITE
float WEAK_SYMBOL OD_InputVoltage_get() {
    osMutexAcquire(mutex_OD_InputVoltage, portMAX_DELAY);
    float value = OD_InputVoltage;
    osMutexRelease(mutex_OD_InputVoltage);
    return value;
}
#endif
#ifndef OD_InputVoltage_SET_OVERWRITE
void WEAK_SYMBOL OD_InputVoltage_set(const float value) {
    osMutexAcquire(mutex_OD_InputVoltage, portMAX_DELAY);
    OD_InputVoltage = value;
    osMutexRelease(mutex_OD_InputVoltage);
}
#endif

#ifndef OD_runtime_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_runtime_get() {
    osMutexAcquire(mutex_OD_runtime, portMAX_DELAY);
    uint32_t value = OD_runtime;
    osMutexRelease(mutex_OD_runtime);
    return value;
}
#endif
#ifndef OD_runtime_SET_OVERWRITE
void WEAK_SYMBOL OD_runtime_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_runtime, portMAX_DELAY);
    OD_runtime = value;
    osMutexRelease(mutex_OD_runtime);
}
#endif

#ifndef OD_SdcIn_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_SdcIn_get() {
    osMutexAcquire(mutex_OD_SdcIn, portMAX_DELAY);
    uint8_t value = OD_SdcIn;
    osMutexRelease(mutex_OD_SdcIn);
    return value;
}
#endif
#ifndef OD_SdcIn_SET_OVERWRITE
void WEAK_SYMBOL OD_SdcIn_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_SdcIn, portMAX_DELAY);
    OD_SdcIn = value;
    osMutexRelease(mutex_OD_SdcIn);
}
#endif

#ifndef OD_SdcOut_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_SdcOut_get() {
    osMutexAcquire(mutex_OD_SdcOut, portMAX_DELAY);
    uint8_t value = OD_SdcOut;
    osMutexRelease(mutex_OD_SdcOut);
    return value;
}
#endif
#ifndef OD_SdcOut_SET_OVERWRITE
void WEAK_SYMBOL OD_SdcOut_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_SdcOut, portMAX_DELAY);
    OD_SdcOut = value;
    osMutexRelease(mutex_OD_SdcOut);
}
#endif

#ifndef OD_ChipUID1_GET_OVERWRITE
uint64_t WEAK_SYMBOL OD_ChipUID1_get() {
    osMutexAcquire(mutex_OD_ChipUID1, portMAX_DELAY);
    uint64_t value = OD_ChipUID1;
    osMutexRelease(mutex_OD_ChipUID1);
    return value;
}
#endif
#ifndef OD_ChipUID1_SET_OVERWRITE
void WEAK_SYMBOL OD_ChipUID1_set(const uint64_t value) {
    osMutexAcquire(mutex_OD_ChipUID1, portMAX_DELAY);
    OD_ChipUID1 = value;
    osMutexRelease(mutex_OD_ChipUID1);
}
#endif

#ifndef OD_ChipUID2_GET_OVERWRITE
uint64_t WEAK_SYMBOL OD_ChipUID2_get() {
    osMutexAcquire(mutex_OD_ChipUID2, portMAX_DELAY);
    uint64_t value = OD_ChipUID2;
    osMutexRelease(mutex_OD_ChipUID2);
    return value;
}
#endif
#ifndef OD_ChipUID2_SET_OVERWRITE
void WEAK_SYMBOL OD_ChipUID2_set(const uint64_t value) {
    osMutexAcquire(mutex_OD_ChipUID2, portMAX_DELAY);
    OD_ChipUID2 = value;
    osMutexRelease(mutex_OD_ChipUID2);
}
#endif

#ifndef OD_BuildDate_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_BuildDate_get() {
    osMutexAcquire(mutex_OD_BuildDate, portMAX_DELAY);
    uint32_t value = OD_BuildDate;
    osMutexRelease(mutex_OD_BuildDate);
    return value;
}
#endif
#ifndef OD_BuildDate_SET_OVERWRITE
void WEAK_SYMBOL OD_BuildDate_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_BuildDate, portMAX_DELAY);
    OD_BuildDate = value;
    osMutexRelease(mutex_OD_BuildDate);
}
#endif

#ifndef OD_BuildTime_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_BuildTime_get() {
    osMutexAcquire(mutex_OD_BuildTime, portMAX_DELAY);
    uint32_t value = OD_BuildTime;
    osMutexRelease(mutex_OD_BuildTime);
    return value;
}
#endif
#ifndef OD_BuildTime_SET_OVERWRITE
void WEAK_SYMBOL OD_BuildTime_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_BuildTime, portMAX_DELAY);
    OD_BuildTime = value;
    osMutexRelease(mutex_OD_BuildTime);
}
#endif

#ifndef OD_CAN1_TxErrCnt_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN1_TxErrCnt_get() {
    osMutexAcquire(mutex_OD_CAN1_TxErrCnt, portMAX_DELAY);
    uint8_t value = OD_CAN1_TxErrCnt;
    osMutexRelease(mutex_OD_CAN1_TxErrCnt);
    return value;
}
#endif
#ifndef OD_CAN1_TxErrCnt_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_TxErrCnt_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN1_TxErrCnt, portMAX_DELAY);
    OD_CAN1_TxErrCnt = value;
    osMutexRelease(mutex_OD_CAN1_TxErrCnt);
}
#endif

#ifndef OD_CAN1_RxErrCnt_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN1_RxErrCnt_get() {
    osMutexAcquire(mutex_OD_CAN1_RxErrCnt, portMAX_DELAY);
    uint8_t value = OD_CAN1_RxErrCnt;
    osMutexRelease(mutex_OD_CAN1_RxErrCnt);
    return value;
}
#endif
#ifndef OD_CAN1_RxErrCnt_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_RxErrCnt_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN1_RxErrCnt, portMAX_DELAY);
    OD_CAN1_RxErrCnt = value;
    osMutexRelease(mutex_OD_CAN1_RxErrCnt);
}
#endif

#ifndef OD_CAN1_lastErrorCode_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN1_lastErrorCode_get() {
    osMutexAcquire(mutex_OD_CAN1_lastErrorCode, portMAX_DELAY);
    uint32_t value = OD_CAN1_lastErrorCode;
    osMutexRelease(mutex_OD_CAN1_lastErrorCode);
    return value;
}
#endif
#ifndef OD_CAN1_lastErrorCode_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_lastErrorCode_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN1_lastErrorCode, portMAX_DELAY);
    OD_CAN1_lastErrorCode = value;
    osMutexRelease(mutex_OD_CAN1_lastErrorCode);
}
#endif

#ifndef OD_CAN1_autoErrorReset_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN1_autoErrorReset_get() {
    osMutexAcquire(mutex_OD_CAN1_autoErrorReset, portMAX_DELAY);
    uint8_t value = OD_CAN1_autoErrorReset;
    osMutexRelease(mutex_OD_CAN1_autoErrorReset);
    return value;
}
#endif
#ifndef OD_CAN1_autoErrorReset_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_autoErrorReset_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN1_autoErrorReset, portMAX_DELAY);
    OD_CAN1_autoErrorReset = value;
    osMutexRelease(mutex_OD_CAN1_autoErrorReset);
}
#endif

#ifndef OD_CAN1_Baudrate_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_CAN1_Baudrate_get() {
    osMutexAcquire(mutex_OD_CAN1_Baudrate, portMAX_DELAY);
    uint16_t value = OD_CAN1_Baudrate;
    osMutexRelease(mutex_OD_CAN1_Baudrate);
    return value;
}
#endif
#ifndef OD_CAN1_Baudrate_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_Baudrate_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_CAN1_Baudrate, portMAX_DELAY);
    OD_CAN1_Baudrate = value;
    osMutexRelease(mutex_OD_CAN1_Baudrate);
}
#endif

#ifndef OD_CAN1_Status_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN1_Status_get() {
    osMutexAcquire(mutex_OD_CAN1_Status, portMAX_DELAY);
    uint8_t value = OD_CAN1_Status;
    osMutexRelease(mutex_OD_CAN1_Status);
    return value;
}
#endif
#ifndef OD_CAN1_Status_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_Status_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN1_Status, portMAX_DELAY);
    OD_CAN1_Status = value;
    osMutexRelease(mutex_OD_CAN1_Status);
}
#endif

#ifndef OD_CAN1_DiscardedTxMessages_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN1_DiscardedTxMessages_get() {
    osMutexAcquire(mutex_OD_CAN1_DiscardedTxMessages, portMAX_DELAY);
    uint32_t value = OD_CAN1_DiscardedTxMessages;
    osMutexRelease(mutex_OD_CAN1_DiscardedTxMessages);
    return value;
}
#endif
#ifndef OD_CAN1_DiscardedTxMessages_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_DiscardedTxMessages_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN1_DiscardedTxMessages, portMAX_DELAY);
    OD_CAN1_DiscardedTxMessages = value;
    osMutexRelease(mutex_OD_CAN1_DiscardedTxMessages);
}
#endif

#ifndef OD_CAN1_ErrorStatus_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN1_ErrorStatus_get() {
    osMutexAcquire(mutex_OD_CAN1_ErrorStatus, portMAX_DELAY);
    uint8_t value = OD_CAN1_ErrorStatus;
    osMutexRelease(mutex_OD_CAN1_ErrorStatus);
    return value;
}
#endif
#ifndef OD_CAN1_ErrorStatus_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_ErrorStatus_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN1_ErrorStatus, portMAX_DELAY);
    OD_CAN1_ErrorStatus = value;
    osMutexRelease(mutex_OD_CAN1_ErrorStatus);
}
#endif

#ifndef OD_CAN1_DelayedTxMessages_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN1_DelayedTxMessages_get() {
    osMutexAcquire(mutex_OD_CAN1_DelayedTxMessages, portMAX_DELAY);
    uint32_t value = OD_CAN1_DelayedTxMessages;
    osMutexRelease(mutex_OD_CAN1_DelayedTxMessages);
    return value;
}
#endif
#ifndef OD_CAN1_DelayedTxMessages_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN1_DelayedTxMessages_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN1_DelayedTxMessages, portMAX_DELAY);
    OD_CAN1_DelayedTxMessages = value;
    osMutexRelease(mutex_OD_CAN1_DelayedTxMessages);
}
#endif

#ifndef OD_CAN2_TxErrCnt_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN2_TxErrCnt_get() {
    osMutexAcquire(mutex_OD_CAN2_TxErrCnt, portMAX_DELAY);
    uint8_t value = OD_CAN2_TxErrCnt;
    osMutexRelease(mutex_OD_CAN2_TxErrCnt);
    return value;
}
#endif
#ifndef OD_CAN2_TxErrCnt_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_TxErrCnt_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN2_TxErrCnt, portMAX_DELAY);
    OD_CAN2_TxErrCnt = value;
    osMutexRelease(mutex_OD_CAN2_TxErrCnt);
}
#endif

#ifndef OD_CAN2_RxErrCnt_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN2_RxErrCnt_get() {
    osMutexAcquire(mutex_OD_CAN2_RxErrCnt, portMAX_DELAY);
    uint8_t value = OD_CAN2_RxErrCnt;
    osMutexRelease(mutex_OD_CAN2_RxErrCnt);
    return value;
}
#endif
#ifndef OD_CAN2_RxErrCnt_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_RxErrCnt_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN2_RxErrCnt, portMAX_DELAY);
    OD_CAN2_RxErrCnt = value;
    osMutexRelease(mutex_OD_CAN2_RxErrCnt);
}
#endif

#ifndef OD_CAN2_lastErrorCode_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN2_lastErrorCode_get() {
    osMutexAcquire(mutex_OD_CAN2_lastErrorCode, portMAX_DELAY);
    uint32_t value = OD_CAN2_lastErrorCode;
    osMutexRelease(mutex_OD_CAN2_lastErrorCode);
    return value;
}
#endif
#ifndef OD_CAN2_lastErrorCode_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_lastErrorCode_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN2_lastErrorCode, portMAX_DELAY);
    OD_CAN2_lastErrorCode = value;
    osMutexRelease(mutex_OD_CAN2_lastErrorCode);
}
#endif

#ifndef OD_CAN2_autoErrorReset_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN2_autoErrorReset_get() {
    osMutexAcquire(mutex_OD_CAN2_autoErrorReset, portMAX_DELAY);
    uint8_t value = OD_CAN2_autoErrorReset;
    osMutexRelease(mutex_OD_CAN2_autoErrorReset);
    return value;
}
#endif
#ifndef OD_CAN2_autoErrorReset_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_autoErrorReset_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN2_autoErrorReset, portMAX_DELAY);
    OD_CAN2_autoErrorReset = value;
    osMutexRelease(mutex_OD_CAN2_autoErrorReset);
}
#endif

#ifndef OD_CAN2_Baudrate_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_CAN2_Baudrate_get() {
    osMutexAcquire(mutex_OD_CAN2_Baudrate, portMAX_DELAY);
    uint16_t value = OD_CAN2_Baudrate;
    osMutexRelease(mutex_OD_CAN2_Baudrate);
    return value;
}
#endif
#ifndef OD_CAN2_Baudrate_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_Baudrate_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_CAN2_Baudrate, portMAX_DELAY);
    OD_CAN2_Baudrate = value;
    osMutexRelease(mutex_OD_CAN2_Baudrate);
}
#endif

#ifndef OD_CAN2_Status_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN2_Status_get() {
    osMutexAcquire(mutex_OD_CAN2_Status, portMAX_DELAY);
    uint8_t value = OD_CAN2_Status;
    osMutexRelease(mutex_OD_CAN2_Status);
    return value;
}
#endif
#ifndef OD_CAN2_Status_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_Status_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN2_Status, portMAX_DELAY);
    OD_CAN2_Status = value;
    osMutexRelease(mutex_OD_CAN2_Status);
}
#endif

#ifndef OD_CAN2_DiscardedTxMessages_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN2_DiscardedTxMessages_get() {
    osMutexAcquire(mutex_OD_CAN2_DiscardedTxMessages, portMAX_DELAY);
    uint32_t value = OD_CAN2_DiscardedTxMessages;
    osMutexRelease(mutex_OD_CAN2_DiscardedTxMessages);
    return value;
}
#endif
#ifndef OD_CAN2_DiscardedTxMessages_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_DiscardedTxMessages_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN2_DiscardedTxMessages, portMAX_DELAY);
    OD_CAN2_DiscardedTxMessages = value;
    osMutexRelease(mutex_OD_CAN2_DiscardedTxMessages);
}
#endif

#ifndef OD_CAN2_ErrorStatus_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_CAN2_ErrorStatus_get() {
    osMutexAcquire(mutex_OD_CAN2_ErrorStatus, portMAX_DELAY);
    uint8_t value = OD_CAN2_ErrorStatus;
    osMutexRelease(mutex_OD_CAN2_ErrorStatus);
    return value;
}
#endif
#ifndef OD_CAN2_ErrorStatus_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_ErrorStatus_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_CAN2_ErrorStatus, portMAX_DELAY);
    OD_CAN2_ErrorStatus = value;
    osMutexRelease(mutex_OD_CAN2_ErrorStatus);
}
#endif

#ifndef OD_CAN2_DelayedTxMessages_GET_OVERWRITE
uint32_t WEAK_SYMBOL OD_CAN2_DelayedTxMessages_get() {
    osMutexAcquire(mutex_OD_CAN2_DelayedTxMessages, portMAX_DELAY);
    uint32_t value = OD_CAN2_DelayedTxMessages;
    osMutexRelease(mutex_OD_CAN2_DelayedTxMessages);
    return value;
}
#endif
#ifndef OD_CAN2_DelayedTxMessages_SET_OVERWRITE
void WEAK_SYMBOL OD_CAN2_DelayedTxMessages_set(const uint32_t value) {
    osMutexAcquire(mutex_OD_CAN2_DelayedTxMessages, portMAX_DELAY);
    OD_CAN2_DelayedTxMessages = value;
    osMutexRelease(mutex_OD_CAN2_DelayedTxMessages);
}
#endif

#ifndef OD_samplingInterval_GET_OVERWRITE
float WEAK_SYMBOL OD_samplingInterval_get() {
    osMutexAcquire(mutex_OD_samplingInterval, portMAX_DELAY);
    float value = OD_samplingInterval;
    osMutexRelease(mutex_OD_samplingInterval);
    return value;
}
#endif
#ifndef OD_samplingInterval_SET_OVERWRITE
void WEAK_SYMBOL OD_samplingInterval_set(const float value) {
    osMutexAcquire(mutex_OD_samplingInterval, portMAX_DELAY);
    OD_samplingInterval = value;
    osMutexRelease(mutex_OD_samplingInterval);
}
#endif

#ifndef OD_TelemetryCommands_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_TelemetryCommands_get() {
    osMutexAcquire(mutex_OD_TelemetryCommands, portMAX_DELAY);
    uint8_t value = OD_TelemetryCommands;
    osMutexRelease(mutex_OD_TelemetryCommands);
    return value;
}
#endif
#ifndef OD_TelemetryCommands_SET_OVERWRITE
void WEAK_SYMBOL OD_TelemetryCommands_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_TelemetryCommands, portMAX_DELAY);
    OD_TelemetryCommands = value;
    osMutexRelease(mutex_OD_TelemetryCommands);
}
#endif

#ifndef OD_StateMachineInterval_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_StateMachineInterval_get() {
    osMutexAcquire(mutex_OD_StateMachineInterval, portMAX_DELAY);
    uint8_t value = OD_StateMachineInterval;
    osMutexRelease(mutex_OD_StateMachineInterval);
    return value;
}
#endif
#ifndef OD_StateMachineInterval_SET_OVERWRITE
void WEAK_SYMBOL OD_StateMachineInterval_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_StateMachineInterval, portMAX_DELAY);
    OD_StateMachineInterval = value;
    osMutexRelease(mutex_OD_StateMachineInterval);
}
#endif

#ifndef OD_StateMachineActivate_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_StateMachineActivate_get() {
    osMutexAcquire(mutex_OD_StateMachineActivate, portMAX_DELAY);
    uint8_t value = OD_StateMachineActivate;
    osMutexRelease(mutex_OD_StateMachineActivate);
    return value;
}
#endif
#ifndef OD_StateMachineActivate_SET_OVERWRITE
void WEAK_SYMBOL OD_StateMachineActivate_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_StateMachineActivate, portMAX_DELAY);
    OD_StateMachineActivate = value;
    osMutexRelease(mutex_OD_StateMachineActivate);
}
#endif

#ifndef OD_HVBatteryMode_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_HVBatteryMode_get() {
    osMutexAcquire(mutex_OD_HVBatteryMode, portMAX_DELAY);
    uint8_t value = OD_HVBatteryMode;
    osMutexRelease(mutex_OD_HVBatteryMode);
    return value;
}
#endif
#ifndef OD_HVBatteryMode_SET_OVERWRITE
void WEAK_SYMBOL OD_HVBatteryMode_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_HVBatteryMode, portMAX_DELAY);
    OD_HVBatteryMode = value;
    osMutexRelease(mutex_OD_HVBatteryMode);
}
#endif

#ifndef OD_EncoderWheelDiameter_GET_OVERWRITE
float WEAK_SYMBOL OD_EncoderWheelDiameter_get() {
    osMutexAcquire(mutex_OD_EncoderWheelDiameter, portMAX_DELAY);
    float value = OD_EncoderWheelDiameter;
    osMutexRelease(mutex_OD_EncoderWheelDiameter);
    return value;
}
#endif
#ifndef OD_EncoderWheelDiameter_SET_OVERWRITE
void WEAK_SYMBOL OD_EncoderWheelDiameter_set(const float value) {
    osMutexAcquire(mutex_OD_EncoderWheelDiameter, portMAX_DELAY);
    OD_EncoderWheelDiameter = value;
    osMutexRelease(mutex_OD_EncoderWheelDiameter);
}
#endif

#ifndef OD_EncoderResetPosition_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_EncoderResetPosition_get() {
    osMutexAcquire(mutex_OD_EncoderResetPosition, portMAX_DELAY);
    uint8_t value = OD_EncoderResetPosition;
    osMutexRelease(mutex_OD_EncoderResetPosition);
    return value;
}
#endif
#ifndef OD_EncoderResetPosition_SET_OVERWRITE
void WEAK_SYMBOL OD_EncoderResetPosition_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_EncoderResetPosition, portMAX_DELAY);
    OD_EncoderResetPosition = value;
    osMutexRelease(mutex_OD_EncoderResetPosition);
}
#endif

#ifndef OD_SetReset_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_SetReset_get() {
    osMutexAcquire(mutex_OD_SetReset, portMAX_DELAY);
    uint8_t value = OD_SetReset;
    osMutexRelease(mutex_OD_SetReset);
    return value;
}
#endif
#ifndef OD_SetReset_SET_OVERWRITE
void WEAK_SYMBOL OD_SetReset_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_SetReset, portMAX_DELAY);
    OD_SetReset = value;
    osMutexRelease(mutex_OD_SetReset);
}
#endif

#ifndef OD_IMU_number_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_IMU_number_get() {
    osMutexAcquire(mutex_OD_IMU_number, portMAX_DELAY);
    uint8_t value = OD_IMU_number;
    osMutexRelease(mutex_OD_IMU_number);
    return value;
}
#endif
#ifndef OD_IMU_number_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_number_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_IMU_number, portMAX_DELAY);
    OD_IMU_number = value;
    osMutexRelease(mutex_OD_IMU_number);
}
#endif

#ifndef OD_IMU1_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU1_Temperature_get() {
    osMutexAcquire(mutex_OD_IMU1_Temperature, portMAX_DELAY);
    float value = OD_IMU1_Temperature;
    osMutexRelease(mutex_OD_IMU1_Temperature);
    return value;
}
#endif
#ifndef OD_IMU1_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU1_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_IMU1_Temperature, portMAX_DELAY);
    OD_IMU1_Temperature = value;
    osMutexRelease(mutex_OD_IMU1_Temperature);
}
#endif

#ifndef OD_IMU2_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU2_Temperature_get() {
    osMutexAcquire(mutex_OD_IMU2_Temperature, portMAX_DELAY);
    float value = OD_IMU2_Temperature;
    osMutexRelease(mutex_OD_IMU2_Temperature);
    return value;
}
#endif
#ifndef OD_IMU2_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU2_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_IMU2_Temperature, portMAX_DELAY);
    OD_IMU2_Temperature = value;
    osMutexRelease(mutex_OD_IMU2_Temperature);
}
#endif

#ifndef OD_IMU3_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU3_Temperature_get() {
    osMutexAcquire(mutex_OD_IMU3_Temperature, portMAX_DELAY);
    float value = OD_IMU3_Temperature;
    osMutexRelease(mutex_OD_IMU3_Temperature);
    return value;
}
#endif
#ifndef OD_IMU3_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU3_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_IMU3_Temperature, portMAX_DELAY);
    OD_IMU3_Temperature = value;
    osMutexRelease(mutex_OD_IMU3_Temperature);
}
#endif

#ifndef OD_IMU_AccelX_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_AccelX_get() {
    osMutexAcquire(mutex_OD_IMU_AccelX, portMAX_DELAY);
    float value = OD_IMU_AccelX;
    osMutexRelease(mutex_OD_IMU_AccelX);
    return value;
}
#endif
#ifndef OD_IMU_AccelX_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_AccelX_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_AccelX, portMAX_DELAY);
    OD_IMU_AccelX = value;
    osMutexRelease(mutex_OD_IMU_AccelX);
}
#endif

#ifndef OD_IMU_AccelY_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_AccelY_get() {
    osMutexAcquire(mutex_OD_IMU_AccelY, portMAX_DELAY);
    float value = OD_IMU_AccelY;
    osMutexRelease(mutex_OD_IMU_AccelY);
    return value;
}
#endif
#ifndef OD_IMU_AccelY_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_AccelY_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_AccelY, portMAX_DELAY);
    OD_IMU_AccelY = value;
    osMutexRelease(mutex_OD_IMU_AccelY);
}
#endif

#ifndef OD_IMU_AccelZ_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_AccelZ_get() {
    osMutexAcquire(mutex_OD_IMU_AccelZ, portMAX_DELAY);
    float value = OD_IMU_AccelZ;
    osMutexRelease(mutex_OD_IMU_AccelZ);
    return value;
}
#endif
#ifndef OD_IMU_AccelZ_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_AccelZ_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_AccelZ, portMAX_DELAY);
    OD_IMU_AccelZ = value;
    osMutexRelease(mutex_OD_IMU_AccelZ);
}
#endif

#ifndef OD_IMU_GyroX_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_GyroX_get() {
    osMutexAcquire(mutex_OD_IMU_GyroX, portMAX_DELAY);
    float value = OD_IMU_GyroX;
    osMutexRelease(mutex_OD_IMU_GyroX);
    return value;
}
#endif
#ifndef OD_IMU_GyroX_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_GyroX_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_GyroX, portMAX_DELAY);
    OD_IMU_GyroX = value;
    osMutexRelease(mutex_OD_IMU_GyroX);
}
#endif

#ifndef OD_IMU_GyroY_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_GyroY_get() {
    osMutexAcquire(mutex_OD_IMU_GyroY, portMAX_DELAY);
    float value = OD_IMU_GyroY;
    osMutexRelease(mutex_OD_IMU_GyroY);
    return value;
}
#endif
#ifndef OD_IMU_GyroY_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_GyroY_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_GyroY, portMAX_DELAY);
    OD_IMU_GyroY = value;
    osMutexRelease(mutex_OD_IMU_GyroY);
}
#endif

#ifndef OD_IMU_GyroZ_GET_OVERWRITE
float WEAK_SYMBOL OD_IMU_GyroZ_get() {
    osMutexAcquire(mutex_OD_IMU_GyroZ, portMAX_DELAY);
    float value = OD_IMU_GyroZ;
    osMutexRelease(mutex_OD_IMU_GyroZ);
    return value;
}
#endif
#ifndef OD_IMU_GyroZ_SET_OVERWRITE
void WEAK_SYMBOL OD_IMU_GyroZ_set(const float value) {
    osMutexAcquire(mutex_OD_IMU_GyroZ, portMAX_DELAY);
    OD_IMU_GyroZ = value;
    osMutexRelease(mutex_OD_IMU_GyroZ);
}
#endif

#ifndef OD_CoolingPressure_GET_OVERWRITE
float WEAK_SYMBOL OD_CoolingPressure_get() {
    osMutexAcquire(mutex_OD_CoolingPressure, portMAX_DELAY);
    float value = OD_CoolingPressure;
    osMutexRelease(mutex_OD_CoolingPressure);
    return value;
}
#endif
#ifndef OD_CoolingPressure_SET_OVERWRITE
void WEAK_SYMBOL OD_CoolingPressure_set(const float value) {
    osMutexAcquire(mutex_OD_CoolingPressure, portMAX_DELAY);
    OD_CoolingPressure = value;
    osMutexRelease(mutex_OD_CoolingPressure);
}
#endif

#ifndef OD_ReservoirTemperature_GET_OVERWRITE
float WEAK_SYMBOL OD_ReservoirTemperature_get() {
    osMutexAcquire(mutex_OD_ReservoirTemperature, portMAX_DELAY);
    float value = OD_ReservoirTemperature;
    osMutexRelease(mutex_OD_ReservoirTemperature);
    return value;
}
#endif
#ifndef OD_ReservoirTemperature_SET_OVERWRITE
void WEAK_SYMBOL OD_ReservoirTemperature_set(const float value) {
    osMutexAcquire(mutex_OD_ReservoirTemperature, portMAX_DELAY);
    OD_ReservoirTemperature = value;
    osMutexRelease(mutex_OD_ReservoirTemperature);
}
#endif

#ifndef OD_Magnet_1_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_1_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_1_Temperature, portMAX_DELAY);
    float value = OD_Magnet_1_Temperature;
    osMutexRelease(mutex_OD_Magnet_1_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_1_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_1_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_1_Temperature, portMAX_DELAY);
    OD_Magnet_1_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_1_Temperature);
}
#endif

#ifndef OD_Magnet_2_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_2_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_2_Temperature, portMAX_DELAY);
    float value = OD_Magnet_2_Temperature;
    osMutexRelease(mutex_OD_Magnet_2_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_2_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_2_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_2_Temperature, portMAX_DELAY);
    OD_Magnet_2_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_2_Temperature);
}
#endif

#ifndef OD_Magnet_3_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_3_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_3_Temperature, portMAX_DELAY);
    float value = OD_Magnet_3_Temperature;
    osMutexRelease(mutex_OD_Magnet_3_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_3_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_3_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_3_Temperature, portMAX_DELAY);
    OD_Magnet_3_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_3_Temperature);
}
#endif

#ifndef OD_Magnet_4_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_4_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_4_Temperature, portMAX_DELAY);
    float value = OD_Magnet_4_Temperature;
    osMutexRelease(mutex_OD_Magnet_4_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_4_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_4_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_4_Temperature, portMAX_DELAY);
    OD_Magnet_4_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_4_Temperature);
}
#endif

#ifndef OD_Magnet_5_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_5_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_5_Temperature, portMAX_DELAY);
    float value = OD_Magnet_5_Temperature;
    osMutexRelease(mutex_OD_Magnet_5_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_5_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_5_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_5_Temperature, portMAX_DELAY);
    OD_Magnet_5_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_5_Temperature);
}
#endif

#ifndef OD_Magnet_6_Temperature_GET_OVERWRITE
float WEAK_SYMBOL OD_Magnet_6_Temperature_get() {
    osMutexAcquire(mutex_OD_Magnet_6_Temperature, portMAX_DELAY);
    float value = OD_Magnet_6_Temperature;
    osMutexRelease(mutex_OD_Magnet_6_Temperature);
    return value;
}
#endif
#ifndef OD_Magnet_6_Temperature_SET_OVERWRITE
void WEAK_SYMBOL OD_Magnet_6_Temperature_set(const float value) {
    osMutexAcquire(mutex_OD_Magnet_6_Temperature, portMAX_DELAY);
    OD_Magnet_6_Temperature = value;
    osMutexRelease(mutex_OD_Magnet_6_Temperature);
}
#endif

#ifndef OD_MdbState_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_MdbState_get() {
    osMutexAcquire(mutex_OD_MdbState, portMAX_DELAY);
    uint8_t value = OD_MdbState;
    osMutexRelease(mutex_OD_MdbState);
    return value;
}
#endif
#ifndef OD_MdbState_SET_OVERWRITE
void WEAK_SYMBOL OD_MdbState_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_MdbState, portMAX_DELAY);
    OD_MdbState = value;
    osMutexRelease(mutex_OD_MdbState);
}
#endif

#ifndef OD_FiducialRightCounter_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_FiducialRightCounter_get() {
    osMutexAcquire(mutex_OD_FiducialRightCounter, portMAX_DELAY);
    uint16_t value = OD_FiducialRightCounter;
    osMutexRelease(mutex_OD_FiducialRightCounter);
    return value;
}
#endif
#ifndef OD_FiducialRightCounter_SET_OVERWRITE
void WEAK_SYMBOL OD_FiducialRightCounter_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_FiducialRightCounter, portMAX_DELAY);
    OD_FiducialRightCounter = value;
    osMutexRelease(mutex_OD_FiducialRightCounter);
}
#endif

#ifndef OD_FiducialLeftCounter_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_FiducialLeftCounter_get() {
    osMutexAcquire(mutex_OD_FiducialLeftCounter, portMAX_DELAY);
    uint16_t value = OD_FiducialLeftCounter;
    osMutexRelease(mutex_OD_FiducialLeftCounter);
    return value;
}
#endif
#ifndef OD_FiducialLeftCounter_SET_OVERWRITE
void WEAK_SYMBOL OD_FiducialLeftCounter_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_FiducialLeftCounter, portMAX_DELAY);
    OD_FiducialLeftCounter = value;
    osMutexRelease(mutex_OD_FiducialLeftCounter);
}
#endif

#ifndef OD_Position_GET_OVERWRITE
float WEAK_SYMBOL OD_Position_get() {
    osMutexAcquire(mutex_OD_Position, portMAX_DELAY);
    float value = OD_Position;
    osMutexRelease(mutex_OD_Position);
    return value;
}
#endif
#ifndef OD_Position_SET_OVERWRITE
void WEAK_SYMBOL OD_Position_set(const float value) {
    osMutexAcquire(mutex_OD_Position, portMAX_DELAY);
    OD_Position = value;
    osMutexRelease(mutex_OD_Position);
}
#endif

#ifndef OD_Velocity_GET_OVERWRITE
float WEAK_SYMBOL OD_Velocity_get() {
    osMutexAcquire(mutex_OD_Velocity, portMAX_DELAY);
    float value = OD_Velocity;
    osMutexRelease(mutex_OD_Velocity);
    return value;
}
#endif
#ifndef OD_Velocity_SET_OVERWRITE
void WEAK_SYMBOL OD_Velocity_set(const float value) {
    osMutexAcquire(mutex_OD_Velocity, portMAX_DELAY);
    OD_Velocity = value;
    osMutexRelease(mutex_OD_Velocity);
}
#endif


/**************************************************************************
* FreeRTOS task that will send out periodically all readable OD entries   *
* Create a task with the function sendOdEntriesTask.                      *
* The task should have a very low priority.                               *
***************************************************************************/
constexpr uint16_t READABLE_SDO_IDS[] = {
    0x001,    0x002,    0x003,    0x004, 
    0x005,    0x010,    0x020,    0x021, 
    0x410,    0x411,    0x412,    0x413, 
    0x414,    0x415,    0x416,    0x420, 
    0x421,    0x430,    0x431,    0x450, 
    0x451,    0x452,    0x453,    0x454, 
    0x456,    0x457,    0x458,    0x459, 
    0x460,    0x461,    0x462,    0x463, 
    0x464,    0x466,    0x467,    0x468, 
    0x469,    0x800,    0x900,    0x901, 
    0x902,    0x903,    0x904,    0xA20, 
    0xA25,    0xA26,    0xA27,    0xA28, 
    0xA29,    0xA30,    0xA31,    0xA32, 
    0xA33,    0xB00,    0xB01,    0xB02, 
    0xB04,    0xB05,    0xB06,    0xB07, 
    0xB08,    0xB10,    0xC00,    0xC01, 
    0xD00,    0xD01 
};
constexpr uint16_t NUMBER_OF_READABLE_SDO_IDS = 66;
extern RNG_HandleTypeDef hrng;

void sendOdEntriesTask(void *pvParameters) {
    // Delay for random amount so that not all nodes send exactly at the same time
    uint32_t randomDelayMs;
    HAL_RNG_GenerateRandomNumber(&hrng, &randomDelayMs);
    randomDelayMs = randomDelayMs & 0xFF;   // Use only lower byte, so delay will be 255ms at maximum
    osDelay(pdMS_TO_TICKS(randomDelayMs));

    uint16_t currentSdoListPos = 0;

    while(1) {
        uint16_t delayInterval = OD_OdEntrySendInterval_get();
        if(delayInterval >= 5) {    // Periodically sending of all readable OD entries is enabled
            uint16_t sdoId = READABLE_SDO_IDS[currentSdoListPos];

            currentSdoListPos++;
            if (currentSdoListPos >= NUMBER_OF_READABLE_SDO_IDS) {
                currentSdoListPos = 0;
            }

            handleSDORequestDownloadBySDOID(sdoId);

            osDelay(pdMS_TO_TICKS(delayInterval));

        } else {    // Sending OD entries is disabled, sleep for one second and then check again
            osDelay(pdMS_TO_TICKS(1000));
        }
    }
}