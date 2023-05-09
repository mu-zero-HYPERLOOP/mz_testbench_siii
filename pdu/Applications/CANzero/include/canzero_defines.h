/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains all CANzero defines (e.g. CAN-IDs) for the node 'PDU'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_DEFINES_H
#define CANZERO_DEFINES_H

#pragma once

// Node ID and name
#define CANZERO_NODE_ID     0x1A
#define CANZERO_NODE_NAME   "PDU"

// CAN-ID of the nodes function codes
#define CANZERO_EMCY_CAN_ID           0x09A
#define CANZERO_TX_PDO1_CAN_ID        0x19A
#define CANZERO_RX_PDO1_CAN_ID        0x1DA
#define CANZERO_TX_PDO2_CAN_ID        0x21A
#define CANZERO_RX_PDO2_CAN_ID        0x25A
#define CANZERO_TX_PDO3_CAN_ID        0x29A
#define CANZERO_RX_PDO3_CAN_ID        0x2DA
#define CANZERO_TX_PDO4_CAN_ID        0x31A
#define CANZERO_RX_PDO4_CAN_ID        0x35A
#define CANZERO_TX_PDO5_CAN_ID        0x39A
#define CANZERO_TX_PDO6_CAN_ID        0x3DA
#define CANZERO_TX_PDO7_CAN_ID        0x41A
#define CANZERO_TX_PDO8_CAN_ID        0x45A
#define CANZERO_TX_PDO9_CAN_ID        0x49A
#define CANZERO_TX_PDO10_CAN_ID       0x4DA
#define CANZERO_TX_PDO11_CAN_ID       0x51A
#define CANZERO_TX_PDO12_CAN_ID       0x55A
#define CANZERO_SDO_RESP_CAN_ID       0x59A
#define CANZERO_SDO_REQ_UP_CAN_ID     0x5DA
#define CANZERO_SDO_REQ_DOWN_CAN_ID   0x61A
#define CANZERO_HEARTBEAT_CAN_ID      0x71A
#define CANZERO_BTL_TX_CAN_ID         0x75A
#define CANZERO_BTL_RX_CAN_ID         0x79A

// CAN DLC of the nodes function codes
#define CANZERO_EMCY_CAN_DLC           4
#define CANZERO_TX_PDO1_CAN_DLC        1
#define CANZERO_RX_PDO1_CAN_DLC        1
#define CANZERO_TX_PDO2_CAN_DLC        8
#define CANZERO_RX_PDO2_CAN_DLC        6
#define CANZERO_TX_PDO3_CAN_DLC        3
#define CANZERO_RX_PDO3_CAN_DLC        6
#define CANZERO_TX_PDO4_CAN_DLC        0
#define CANZERO_RX_PDO4_CAN_DLC        3
#define CANZERO_TX_PDO5_CAN_DLC        0
#define CANZERO_TX_PDO6_CAN_DLC        0
#define CANZERO_TX_PDO7_CAN_DLC        0
#define CANZERO_TX_PDO8_CAN_DLC        0
#define CANZERO_TX_PDO9_CAN_DLC        8
#define CANZERO_TX_PDO10_CAN_DLC       4
#define CANZERO_TX_PDO11_CAN_DLC       8
#define CANZERO_TX_PDO12_CAN_DLC       8
#define CANZERO_SDO_RESP_CAN_DLC       8
#define CANZERO_SDO_REQ_UP_CAN_DLC     8
#define CANZERO_SDO_REQ_DOWN_CAN_DLC   2
#define CANZERO_HEARTBEAT_CAN_DLC      1
#define CANZERO_BTL_TX_CAN_DLC         8
#define CANZERO_BTL_RX_CAN_DLC         8

#endif // CANZERO_DEFINES_H