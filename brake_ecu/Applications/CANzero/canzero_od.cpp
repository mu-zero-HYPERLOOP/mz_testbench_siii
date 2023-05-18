/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 12:50:57 18.05.2023.
 * It contains the object dictionary for the node 'BrakeF'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#include "canzero_od.hpp"


/**************************************************************************
* Definition of all OD variables with default values from database.       *
***************************************************************************/
volatile uint8_t  OD_NodeID                   = can::signals::BrakeF_OD_NodeID::CANzero_SDO_Default;
volatile uint8_t  OD_NodeStatus               = can::signals::BrakeF_OD_NodeStatus::CANzero_SDO_Default;
volatile uint16_t OD_ProtocolVersion          = can::signals::BrakeF_OD_ProtocolVersion::CANzero_SDO_Default;
volatile uint16_t OD_StackVersion             = can::signals::BrakeF_OD_StackVersion::CANzero_SDO_Default;
volatile uint16_t OD_DbcVersion               = can::signals::BrakeF_OD_DbcVersion::CANzero_SDO_Default;
volatile uint16_t OD_HeartbeatInterval        = can::signals::BrakeF_OD_HeartbeatInterval::CANzero_SDO_Default;
volatile uint8_t  OD_SendOdOnBootup           = can::signals::BrakeF_OD_SendOdOnBootup::CANzero_SDO_Default;
volatile uint16_t OD_OdEntrySendInterval      = can::signals::BrakeF_OD_OdEntrySendInterval::CANzero_SDO_Default;
volatile float    OD_CpuUsage                 = can::signals::BrakeF_OD_CpuUsage::CANzero_SDO_Default;
volatile uint32_t OD_MemFree                  = can::signals::BrakeF_OD_MemFree::CANzero_SDO_Default;
volatile float    OD_BoardTemp                = can::signals::BrakeF_OD_BoardTemp::CANzero_SDO_Default;
volatile float    OD_InputVoltage             = can::signals::BrakeF_OD_InputVoltage::CANzero_SDO_Default;
volatile uint32_t OD_runtime                  = can::signals::BrakeF_OD_runtime::CANzero_SDO_Default;
volatile uint8_t  OD_SdcIn                    = can::signals::BrakeF_OD_SdcIn::CANzero_SDO_Default;
volatile uint8_t  OD_SdcOut                   = can::signals::BrakeF_OD_SdcOut::CANzero_SDO_Default;
volatile uint64_t OD_ChipUID1                 = can::signals::BrakeF_OD_ChipUID1::CANzero_SDO_Default;
volatile uint64_t OD_ChipUID2                 = can::signals::BrakeF_OD_ChipUID2::CANzero_SDO_Default;
volatile uint32_t OD_BuildDate                = can::signals::BrakeF_OD_BuildDate::CANzero_SDO_Default;
volatile uint32_t OD_BuildTime                = can::signals::BrakeF_OD_BuildTime::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_TxErrCnt            = can::signals::BrakeF_OD_CAN1_TxErrCnt::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_RxErrCnt            = can::signals::BrakeF_OD_CAN1_RxErrCnt::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_lastErrorCode       = can::signals::BrakeF_OD_CAN1_lastErrorCode::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_autoErrorReset      = can::signals::BrakeF_OD_CAN1_autoErrorReset::CANzero_SDO_Default;
volatile uint16_t OD_CAN1_Baudrate            = can::signals::BrakeF_OD_CAN1_Baudrate::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_Status              = can::signals::BrakeF_OD_CAN1_Status::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_DiscardedTxMessages = can::signals::BrakeF_OD_CAN1_DiscardedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN1_ErrorStatus         = can::signals::BrakeF_OD_CAN1_ErrorStatus::CANzero_SDO_Default;
volatile uint32_t OD_CAN1_DelayedTxMessages   = can::signals::BrakeF_OD_CAN1_DelayedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_TxErrCnt            = can::signals::BrakeF_OD_CAN2_TxErrCnt::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_RxErrCnt            = can::signals::BrakeF_OD_CAN2_RxErrCnt::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_lastErrorCode       = can::signals::BrakeF_OD_CAN2_lastErrorCode::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_autoErrorReset      = can::signals::BrakeF_OD_CAN2_autoErrorReset::CANzero_SDO_Default;
volatile uint16_t OD_CAN2_Baudrate            = can::signals::BrakeF_OD_CAN2_Baudrate::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_Status              = can::signals::BrakeF_OD_CAN2_Status::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_DiscardedTxMessages = can::signals::BrakeF_OD_CAN2_DiscardedTxMessages::CANzero_SDO_Default;
volatile uint8_t  OD_CAN2_ErrorStatus         = can::signals::BrakeF_OD_CAN2_ErrorStatus::CANzero_SDO_Default;
volatile uint32_t OD_CAN2_DelayedTxMessages   = can::signals::BrakeF_OD_CAN2_DelayedTxMessages::CANzero_SDO_Default;
volatile float    OD_tankLowerControlLimit    = can::signals::BrakeF_OD_tankLowerControlLimit::CANzero_SDO_Default;
volatile float    OD_tankUpperControlLimit    = can::signals::BrakeF_OD_tankUpperControlLimit::CANzero_SDO_Default;
volatile float    OD_delay                    = can::signals::BrakeF_OD_delay::CANzero_SDO_Default;
volatile uint8_t  OD_counterLimit             = can::signals::BrakeF_OD_counterLimit::CANzero_SDO_Default;
volatile uint16_t OD_commWatchdog             = can::signals::BrakeF_OD_commWatchdog::CANzero_SDO_Default;
volatile float    OD_valveUpperTolerance      = can::signals::BrakeF_OD_valveUpperTolerance::CANzero_SDO_Default;
volatile uint16_t OD_Pressure1                = can::signals::BrakeF_OD_Pressure1::CANzero_SDO_Default;
volatile uint16_t OD_Pressure2                = can::signals::BrakeF_OD_Pressure2::CANzero_SDO_Default;
volatile uint16_t OD_Pressure3                = can::signals::BrakeF_OD_Pressure3::CANzero_SDO_Default;
volatile uint16_t OD_Pressure4                = can::signals::BrakeF_OD_Pressure4::CANzero_SDO_Default;


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
osMutexId_t mutex_OD_tankLowerControlLimit    = osMutexNew(NULL);
osMutexId_t mutex_OD_tankUpperControlLimit    = osMutexNew(NULL);
osMutexId_t mutex_OD_delay                    = osMutexNew(NULL);
osMutexId_t mutex_OD_counterLimit             = osMutexNew(NULL);
osMutexId_t mutex_OD_commWatchdog             = osMutexNew(NULL);
osMutexId_t mutex_OD_valveUpperTolerance      = osMutexNew(NULL);
osMutexId_t mutex_OD_Pressure1                = osMutexNew(NULL);
osMutexId_t mutex_OD_Pressure2                = osMutexNew(NULL);
osMutexId_t mutex_OD_Pressure3                = osMutexNew(NULL);
osMutexId_t mutex_OD_Pressure4                = osMutexNew(NULL);


/**************************************************************************
* Functions to handle a SDO download and upload request.                  *
***************************************************************************/
void handleSDORequestDownload(const RxMessage& rxMsgSdoReq) {
    can::Message<can::messages::BrakeF_SDO_Req_Down> msgSdoReq(rxMsgSdoReq);
    uint16_t sdoId = msgSdoReq.get<can::signals::BrakeF_SDO_ID>();
    handleSDORequestDownloadBySDOID(sdoId);
}
void handleSDORequestDownloadBySDOID(const uint16_t sdoId) {    
    can::Message<can::messages::BrakeF_SDO_Resp> msgSdoResp;
    uint8_t respCode = can::signals::BrakeF_SDO_RespCode::ERR_NON_EXISTING_OBJECT;

    switch (sdoId) {
        case 0x1:    // OD_NodeID
            msgSdoResp.set<can::signals::BrakeF_OD_NodeID>(OD_NodeID_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x2:    // OD_NodeStatus
            msgSdoResp.set<can::signals::BrakeF_OD_NodeStatus>(OD_NodeStatus_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x3:    // OD_ProtocolVersion
            msgSdoResp.set<can::signals::BrakeF_OD_ProtocolVersion>(OD_ProtocolVersion_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x4:    // OD_StackVersion
            msgSdoResp.set<can::signals::BrakeF_OD_StackVersion>(OD_StackVersion_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x5:    // OD_DbcVersion
            msgSdoResp.set<can::signals::BrakeF_OD_DbcVersion>(OD_DbcVersion_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x10:    // OD_HeartbeatInterval
            msgSdoResp.set<can::signals::BrakeF_OD_HeartbeatInterval>(OD_HeartbeatInterval_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x20:    // OD_SendOdOnBootup
            msgSdoResp.set<can::signals::BrakeF_OD_SendOdOnBootup>(OD_SendOdOnBootup_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x21:    // OD_OdEntrySendInterval
            msgSdoResp.set<can::signals::BrakeF_OD_OdEntrySendInterval>(OD_OdEntrySendInterval_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x410:    // OD_CpuUsage
            msgSdoResp.set<can::signals::BrakeF_OD_CpuUsage>(OD_CpuUsage_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x411:    // OD_MemFree
            msgSdoResp.set<can::signals::BrakeF_OD_MemFree>(OD_MemFree_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x412:    // OD_BoardTemp
            msgSdoResp.set<can::signals::BrakeF_OD_BoardTemp>(OD_BoardTemp_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x413:    // OD_InputVoltage
            msgSdoResp.set<can::signals::BrakeF_OD_InputVoltage>(OD_InputVoltage_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x414:    // OD_runtime
            msgSdoResp.set<can::signals::BrakeF_OD_runtime>(OD_runtime_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x415:    // OD_SdcIn
            msgSdoResp.set<can::signals::BrakeF_OD_SdcIn>(OD_SdcIn_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x416:    // OD_SdcOut
            msgSdoResp.set<can::signals::BrakeF_OD_SdcOut>(OD_SdcOut_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x420:    // OD_ChipUID1
            msgSdoResp.set<can::signals::BrakeF_OD_ChipUID1>(OD_ChipUID1_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x421:    // OD_ChipUID2
            msgSdoResp.set<can::signals::BrakeF_OD_ChipUID2>(OD_ChipUID2_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x430:    // OD_BuildDate
            msgSdoResp.set<can::signals::BrakeF_OD_BuildDate>(OD_BuildDate_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x431:    // OD_BuildTime
            msgSdoResp.set<can::signals::BrakeF_OD_BuildTime>(OD_BuildTime_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x450:    // OD_CAN1_TxErrCnt
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_TxErrCnt>(OD_CAN1_TxErrCnt_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x451:    // OD_CAN1_RxErrCnt
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_RxErrCnt>(OD_CAN1_RxErrCnt_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x452:    // OD_CAN1_lastErrorCode
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_lastErrorCode>(OD_CAN1_lastErrorCode_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x453:    // OD_CAN1_autoErrorReset
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_autoErrorReset>(OD_CAN1_autoErrorReset_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x454:    // OD_CAN1_Baudrate
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_Baudrate>(OD_CAN1_Baudrate_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x456:    // OD_CAN1_Status
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_Status>(OD_CAN1_Status_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x457:    // OD_CAN1_DiscardedTxMessages
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_DiscardedTxMessages>(OD_CAN1_DiscardedTxMessages_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x458:    // OD_CAN1_ErrorStatus
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_ErrorStatus>(OD_CAN1_ErrorStatus_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x459:    // OD_CAN1_DelayedTxMessages
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_DelayedTxMessages>(OD_CAN1_DelayedTxMessages_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x460:    // OD_CAN2_TxErrCnt
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_TxErrCnt>(OD_CAN2_TxErrCnt_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x461:    // OD_CAN2_RxErrCnt
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_RxErrCnt>(OD_CAN2_RxErrCnt_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x462:    // OD_CAN2_lastErrorCode
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_lastErrorCode>(OD_CAN2_lastErrorCode_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x463:    // OD_CAN2_autoErrorReset
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_autoErrorReset>(OD_CAN2_autoErrorReset_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x464:    // OD_CAN2_Baudrate
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_Baudrate>(OD_CAN2_Baudrate_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x466:    // OD_CAN2_Status
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_Status>(OD_CAN2_Status_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x467:    // OD_CAN2_DiscardedTxMessages
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_DiscardedTxMessages>(OD_CAN2_DiscardedTxMessages_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x468:    // OD_CAN2_ErrorStatus
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_ErrorStatus>(OD_CAN2_ErrorStatus_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x469:    // OD_CAN2_DelayedTxMessages
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_DelayedTxMessages>(OD_CAN2_DelayedTxMessages_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x800:    // OD_tankLowerControlLimit
            msgSdoResp.set<can::signals::BrakeF_OD_tankLowerControlLimit>(OD_tankLowerControlLimit_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x801:    // OD_tankUpperControlLimit
            msgSdoResp.set<can::signals::BrakeF_OD_tankUpperControlLimit>(OD_tankUpperControlLimit_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x802:    // OD_delay
            msgSdoResp.set<can::signals::BrakeF_OD_delay>(OD_delay_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x803:    // OD_counterLimit
            msgSdoResp.set<can::signals::BrakeF_OD_counterLimit>(OD_counterLimit_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x804:    // OD_commWatchdog
            msgSdoResp.set<can::signals::BrakeF_OD_commWatchdog>(OD_commWatchdog_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0x805:    // OD_valveUpperTolerance
            msgSdoResp.set<can::signals::BrakeF_OD_valveUpperTolerance>(OD_valveUpperTolerance_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0xA00:    // OD_Pressure1
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure1>(OD_Pressure1_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0xA01:    // OD_Pressure2
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure2>(OD_Pressure2_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0xA02:    // OD_Pressure3
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure3>(OD_Pressure3_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        case 0xA03:    // OD_Pressure4
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure4>(OD_Pressure4_get());
            respCode = can::signals::BrakeF_SDO_RespCode::OK;
            break;
        default:
            // Unknown SDO-ID, just reply unknown ID
            msgSdoResp.set<can::signals::BrakeF_SDO_ID>(sdoId);
            break;
    }

    msgSdoResp.set<can::signals::BrakeF_SDO_RespCode>(respCode);

    // Send response message
    extern osMessageQueueId_t czSendQueue;
    TxMessage sendTxMessage = msgSdoResp.getTxMessage();
    osMessageQueuePut(czSendQueue, &sendTxMessage, 0, 0);
}

void handleSDORequestUpload(const RxMessage& rxMsgSdoReq) {
    can::Message<can::messages::BrakeF_SDO_Req_Up> msgSdoReq(rxMsgSdoReq);
    can::Message<can::messages::BrakeF_SDO_Resp> msgSdoResp;
    uint8_t respCode = can::signals::BrakeF_SDO_RespCode::ERR_NON_EXISTING_OBJECT;
    uint16_t sdoId = msgSdoReq.get<can::signals::BrakeF_SDO_ID>();

    switch (sdoId) {
        case 0x1: {   // OD_NodeID
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_NodeID>(OD_NodeID_get());
            break;
        }
        case 0x2: {   // OD_NodeStatus
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_NodeStatus>(OD_NodeStatus_get());
            break;
        }
        case 0x3: {   // OD_ProtocolVersion
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_ProtocolVersion>(OD_ProtocolVersion_get());
            break;
        }
        case 0x4: {   // OD_StackVersion
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_StackVersion>(OD_StackVersion_get());
            break;
        }
        case 0x5: {   // OD_DbcVersion
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_DbcVersion>(OD_DbcVersion_get());
            break;
        }
        case 0x10: {   // OD_HeartbeatInterval
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_HeartbeatInterval>();
                OD_HeartbeatInterval_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_HeartbeatInterval>(OD_HeartbeatInterval_get());
            break;
        }
        case 0x20: {   // OD_SendOdOnBootup
            uint8_t value = msgSdoReq.get<can::signals::BrakeF_OD_SendOdOnBootup>();
                OD_SendOdOnBootup_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_SendOdOnBootup>(OD_SendOdOnBootup_get());
            break;
        }
        case 0x21: {   // OD_OdEntrySendInterval
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_OdEntrySendInterval>();
                OD_OdEntrySendInterval_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_OdEntrySendInterval>(OD_OdEntrySendInterval_get());
            break;
        }
        case 0x410: {   // OD_CpuUsage
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CpuUsage>(OD_CpuUsage_get());
            break;
        }
        case 0x411: {   // OD_MemFree
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_MemFree>(OD_MemFree_get());
            break;
        }
        case 0x412: {   // OD_BoardTemp
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_BoardTemp>(OD_BoardTemp_get());
            break;
        }
        case 0x413: {   // OD_InputVoltage
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_InputVoltage>(OD_InputVoltage_get());
            break;
        }
        case 0x414: {   // OD_runtime
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_runtime>(OD_runtime_get());
            break;
        }
        case 0x415: {   // OD_SdcIn
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_SdcIn>(OD_SdcIn_get());
            break;
        }
        case 0x416: {   // OD_SdcOut
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_SdcOut>(OD_SdcOut_get());
            break;
        }
        case 0x420: {   // OD_ChipUID1
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_ChipUID1>(OD_ChipUID1_get());
            break;
        }
        case 0x421: {   // OD_ChipUID2
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_ChipUID2>(OD_ChipUID2_get());
            break;
        }
        case 0x430: {   // OD_BuildDate
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_BuildDate>(OD_BuildDate_get());
            break;
        }
        case 0x431: {   // OD_BuildTime
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_BuildTime>(OD_BuildTime_get());
            break;
        }
        case 0x450: {   // OD_CAN1_TxErrCnt
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_TxErrCnt>(OD_CAN1_TxErrCnt_get());
            break;
        }
        case 0x451: {   // OD_CAN1_RxErrCnt
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_RxErrCnt>(OD_CAN1_RxErrCnt_get());
            break;
        }
        case 0x452: {   // OD_CAN1_lastErrorCode
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_lastErrorCode>(OD_CAN1_lastErrorCode_get());
            break;
        }
        case 0x453: {   // OD_CAN1_autoErrorReset
            uint8_t value = msgSdoReq.get<can::signals::BrakeF_OD_CAN1_autoErrorReset>();
                OD_CAN1_autoErrorReset_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_autoErrorReset>(OD_CAN1_autoErrorReset_get());
            break;
        }
        case 0x454: {   // OD_CAN1_Baudrate
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_CAN1_Baudrate>();
if (value < 125 || value > 1000) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_CAN1_Baudrate_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_Baudrate>(OD_CAN1_Baudrate_get());
            break;
        }
        case 0x456: {   // OD_CAN1_Status
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_Status>(OD_CAN1_Status_get());
            break;
        }
        case 0x457: {   // OD_CAN1_DiscardedTxMessages
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_DiscardedTxMessages>(OD_CAN1_DiscardedTxMessages_get());
            break;
        }
        case 0x458: {   // OD_CAN1_ErrorStatus
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_ErrorStatus>(OD_CAN1_ErrorStatus_get());
            break;
        }
        case 0x459: {   // OD_CAN1_DelayedTxMessages
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN1_DelayedTxMessages>(OD_CAN1_DelayedTxMessages_get());
            break;
        }
        case 0x460: {   // OD_CAN2_TxErrCnt
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_TxErrCnt>(OD_CAN2_TxErrCnt_get());
            break;
        }
        case 0x461: {   // OD_CAN2_RxErrCnt
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_RxErrCnt>(OD_CAN2_RxErrCnt_get());
            break;
        }
        case 0x462: {   // OD_CAN2_lastErrorCode
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_lastErrorCode>(OD_CAN2_lastErrorCode_get());
            break;
        }
        case 0x463: {   // OD_CAN2_autoErrorReset
            uint8_t value = msgSdoReq.get<can::signals::BrakeF_OD_CAN2_autoErrorReset>();
                OD_CAN2_autoErrorReset_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_autoErrorReset>(OD_CAN2_autoErrorReset_get());
            break;
        }
        case 0x464: {   // OD_CAN2_Baudrate
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_CAN2_Baudrate>();
if (value < 125 || value > 1000) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_CAN2_Baudrate_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_Baudrate>(OD_CAN2_Baudrate_get());
            break;
        }
        case 0x466: {   // OD_CAN2_Status
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_Status>(OD_CAN2_Status_get());
            break;
        }
        case 0x467: {   // OD_CAN2_DiscardedTxMessages
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_DiscardedTxMessages>(OD_CAN2_DiscardedTxMessages_get());
            break;
        }
        case 0x468: {   // OD_CAN2_ErrorStatus
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_ErrorStatus>(OD_CAN2_ErrorStatus_get());
            break;
        }
        case 0x469: {   // OD_CAN2_DelayedTxMessages
            respCode = can::signals::BrakeF_SDO_RespCode::ERR_READ_ONLY_OBJECT;
            msgSdoResp.set<can::signals::BrakeF_OD_CAN2_DelayedTxMessages>(OD_CAN2_DelayedTxMessages_get());
            break;
        }
        case 0x800: {   // OD_tankLowerControlLimit
            float value = msgSdoReq.get<can::signals::BrakeF_OD_tankLowerControlLimit>();
if (value < 0.0f || value > 327.675f) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_tankLowerControlLimit_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_tankLowerControlLimit>(OD_tankLowerControlLimit_get());
            break;
        }
        case 0x801: {   // OD_tankUpperControlLimit
            float value = msgSdoReq.get<can::signals::BrakeF_OD_tankUpperControlLimit>();
if (value < 0.0f || value > 327.675f) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_tankUpperControlLimit_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_tankUpperControlLimit>(OD_tankUpperControlLimit_get());
            break;
        }
        case 0x802: {   // OD_delay
            float value = msgSdoReq.get<can::signals::BrakeF_OD_delay>();
if (value < 0.0f || value > 127.5f) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_delay_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_delay>(OD_delay_get());
            break;
        }
        case 0x803: {   // OD_counterLimit
            uint8_t value = msgSdoReq.get<can::signals::BrakeF_OD_counterLimit>();
                OD_counterLimit_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_counterLimit>(OD_counterLimit_get());
            break;
        }
        case 0x804: {   // OD_commWatchdog
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_commWatchdog>();
                OD_commWatchdog_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_commWatchdog>(OD_commWatchdog_get());
            break;
        }
        case 0x805: {   // OD_valveUpperTolerance
            float value = msgSdoReq.get<can::signals::BrakeF_OD_valveUpperTolerance>();
if (value < 0.0f || value > 327.675f) {
                respCode = can::signals::BrakeF_SDO_RespCode::ERR_OUT_OF_RANGE;
            }
            else {
                OD_valveUpperTolerance_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            }
            msgSdoResp.set<can::signals::BrakeF_OD_valveUpperTolerance>(OD_valveUpperTolerance_get());
            break;
        }
        case 0xA00: {   // OD_Pressure1
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_Pressure1>();
                OD_Pressure1_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure1>(OD_Pressure1_get());
            break;
        }
        case 0xA01: {   // OD_Pressure2
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_Pressure2>();
                OD_Pressure2_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure2>(OD_Pressure2_get());
            break;
        }
        case 0xA02: {   // OD_Pressure3
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_Pressure3>();
                OD_Pressure3_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure3>(OD_Pressure3_get());
            break;
        }
        case 0xA03: {   // OD_Pressure4
            uint16_t value = msgSdoReq.get<can::signals::BrakeF_OD_Pressure4>();
                OD_Pressure4_set(value);
                respCode = can::signals::BrakeF_SDO_RespCode::OK;
            msgSdoResp.set<can::signals::BrakeF_OD_Pressure4>(OD_Pressure4_get());
            break;
        }
        default:
            // Unknown SDO-ID, just reply unknown ID
            msgSdoResp.set<can::signals::BrakeF_SDO_ID>(sdoId);
            break;
    }

    msgSdoResp.set<can::signals::BrakeF_SDO_RespCode>(respCode);

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

#ifndef OD_tankLowerControlLimit_GET_OVERWRITE
float WEAK_SYMBOL OD_tankLowerControlLimit_get() {
    osMutexAcquire(mutex_OD_tankLowerControlLimit, portMAX_DELAY);
    float value = OD_tankLowerControlLimit;
    osMutexRelease(mutex_OD_tankLowerControlLimit);
    return value;
}
#endif
#ifndef OD_tankLowerControlLimit_SET_OVERWRITE
void WEAK_SYMBOL OD_tankLowerControlLimit_set(const float value) {
    osMutexAcquire(mutex_OD_tankLowerControlLimit, portMAX_DELAY);
    OD_tankLowerControlLimit = value;
    osMutexRelease(mutex_OD_tankLowerControlLimit);
}
#endif

#ifndef OD_tankUpperControlLimit_GET_OVERWRITE
float WEAK_SYMBOL OD_tankUpperControlLimit_get() {
    osMutexAcquire(mutex_OD_tankUpperControlLimit, portMAX_DELAY);
    float value = OD_tankUpperControlLimit;
    osMutexRelease(mutex_OD_tankUpperControlLimit);
    return value;
}
#endif
#ifndef OD_tankUpperControlLimit_SET_OVERWRITE
void WEAK_SYMBOL OD_tankUpperControlLimit_set(const float value) {
    osMutexAcquire(mutex_OD_tankUpperControlLimit, portMAX_DELAY);
    OD_tankUpperControlLimit = value;
    osMutexRelease(mutex_OD_tankUpperControlLimit);
}
#endif

#ifndef OD_delay_GET_OVERWRITE
float WEAK_SYMBOL OD_delay_get() {
    osMutexAcquire(mutex_OD_delay, portMAX_DELAY);
    float value = OD_delay;
    osMutexRelease(mutex_OD_delay);
    return value;
}
#endif
#ifndef OD_delay_SET_OVERWRITE
void WEAK_SYMBOL OD_delay_set(const float value) {
    osMutexAcquire(mutex_OD_delay, portMAX_DELAY);
    OD_delay = value;
    osMutexRelease(mutex_OD_delay);
}
#endif

#ifndef OD_counterLimit_GET_OVERWRITE
uint8_t WEAK_SYMBOL OD_counterLimit_get() {
    osMutexAcquire(mutex_OD_counterLimit, portMAX_DELAY);
    uint8_t value = OD_counterLimit;
    osMutexRelease(mutex_OD_counterLimit);
    return value;
}
#endif
#ifndef OD_counterLimit_SET_OVERWRITE
void WEAK_SYMBOL OD_counterLimit_set(const uint8_t value) {
    osMutexAcquire(mutex_OD_counterLimit, portMAX_DELAY);
    OD_counterLimit = value;
    osMutexRelease(mutex_OD_counterLimit);
}
#endif

#ifndef OD_commWatchdog_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_commWatchdog_get() {
    osMutexAcquire(mutex_OD_commWatchdog, portMAX_DELAY);
    uint16_t value = OD_commWatchdog;
    osMutexRelease(mutex_OD_commWatchdog);
    return value;
}
#endif
#ifndef OD_commWatchdog_SET_OVERWRITE
void WEAK_SYMBOL OD_commWatchdog_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_commWatchdog, portMAX_DELAY);
    OD_commWatchdog = value;
    osMutexRelease(mutex_OD_commWatchdog);
}
#endif

#ifndef OD_valveUpperTolerance_GET_OVERWRITE
float WEAK_SYMBOL OD_valveUpperTolerance_get() {
    osMutexAcquire(mutex_OD_valveUpperTolerance, portMAX_DELAY);
    float value = OD_valveUpperTolerance;
    osMutexRelease(mutex_OD_valveUpperTolerance);
    return value;
}
#endif
#ifndef OD_valveUpperTolerance_SET_OVERWRITE
void WEAK_SYMBOL OD_valveUpperTolerance_set(const float value) {
    osMutexAcquire(mutex_OD_valveUpperTolerance, portMAX_DELAY);
    OD_valveUpperTolerance = value;
    osMutexRelease(mutex_OD_valveUpperTolerance);
}
#endif

#ifndef OD_Pressure1_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_Pressure1_get() {
    osMutexAcquire(mutex_OD_Pressure1, portMAX_DELAY);
    uint16_t value = OD_Pressure1;
    osMutexRelease(mutex_OD_Pressure1);
    return value;
}
#endif
#ifndef OD_Pressure1_SET_OVERWRITE
void WEAK_SYMBOL OD_Pressure1_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_Pressure1, portMAX_DELAY);
    OD_Pressure1 = value;
    osMutexRelease(mutex_OD_Pressure1);
}
#endif

#ifndef OD_Pressure2_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_Pressure2_get() {
    osMutexAcquire(mutex_OD_Pressure2, portMAX_DELAY);
    uint16_t value = OD_Pressure2;
    osMutexRelease(mutex_OD_Pressure2);
    return value;
}
#endif
#ifndef OD_Pressure2_SET_OVERWRITE
void WEAK_SYMBOL OD_Pressure2_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_Pressure2, portMAX_DELAY);
    OD_Pressure2 = value;
    osMutexRelease(mutex_OD_Pressure2);
}
#endif

#ifndef OD_Pressure3_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_Pressure3_get() {
    osMutexAcquire(mutex_OD_Pressure3, portMAX_DELAY);
    uint16_t value = OD_Pressure3;
    osMutexRelease(mutex_OD_Pressure3);
    return value;
}
#endif
#ifndef OD_Pressure3_SET_OVERWRITE
void WEAK_SYMBOL OD_Pressure3_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_Pressure3, portMAX_DELAY);
    OD_Pressure3 = value;
    osMutexRelease(mutex_OD_Pressure3);
}
#endif

#ifndef OD_Pressure4_GET_OVERWRITE
uint16_t WEAK_SYMBOL OD_Pressure4_get() {
    osMutexAcquire(mutex_OD_Pressure4, portMAX_DELAY);
    uint16_t value = OD_Pressure4;
    osMutexRelease(mutex_OD_Pressure4);
    return value;
}
#endif
#ifndef OD_Pressure4_SET_OVERWRITE
void WEAK_SYMBOL OD_Pressure4_set(const uint16_t value) {
    osMutexAcquire(mutex_OD_Pressure4, portMAX_DELAY);
    OD_Pressure4 = value;
    osMutexRelease(mutex_OD_Pressure4);
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
    0x469,    0x800,    0x801,    0x802, 
    0x803,    0x804,    0x805,    0xA00, 
    0xA01,    0xA02,    0xA03 
};
constexpr uint16_t NUMBER_OF_READABLE_SDO_IDS = 47;
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