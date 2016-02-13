/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define NRF24_MAX_PAYLOAD_SIZE 32

#define BV(x) (1<<(x)) // bit value

// Register map of nRF24L01
enum {
    NRF24_00_CONFIG     = 0x00,
    NRF24_01_EN_AA      = 0x01, // Auto Acknowledge
    NRF24_02_EN_RXADDR  = 0x02,
    NRF24_03_SETUP_AW   = 0x03, // Address Width
    NRF24_04_SETUP_RETR = 0x04, // automatic RETRansmission
    NRF24_05_RF_CH      = 0x05, // RF CHannel
    NRF24_06_RF_SETUP   = 0x06,
    NRF24_07_STATUS     = 0x07,
    NRF24_08_OBSERVE_TX = 0x08,
    NRF24_09_RPD        = 0x09, //Received Power Detector in the nRF23L01+, called CD (Carrier Detect) in the nRF24L01
    NRF24_0A_RX_ADDR_P0 = 0x0A,
    NRF24_0B_RX_ADDR_P1 = 0x0B,
    NRF24_0C_RX_ADDR_P2 = 0x0C,
    NRF24_0D_RX_ADDR_P3 = 0x0D,
    NRF24_0E_RX_ADDR_P4 = 0x0E,
    NRF24_0F_RX_ADDR_P5 = 0x0F,
    NRF24_10_TX_ADDR    = 0x10,
    NRF24_11_RX_PW_P0   = 0x11, // Payload Width
    NRF24_12_RX_PW_P1   = 0x12,
    NRF24_13_RX_PW_P2   = 0x13,
    NRF24_14_RX_PW_P3   = 0x14,
    NRF24_15_RX_PW_P4   = 0x15,
    NRF24_16_RX_PW_P5   = 0x16,
    NRF24_17_FIFO_STATUS= 0x17,
    NRF24_1C_DYNPD      = 0x1C, // DYNamic PayloaD
    NRF24_1D_FEATURE    = 0x1D
};

// Bit position mnemonics
enum {
    NRF24_00_CONFIG_MASK_RX_DR      = 6,
    NRF24_00_CONFIG_MASK_TX_DS      = 5,
    NRF24_00_CONFIG_MASK_MAX_RT     = 4,
    NRF24_00_CONFIG_EN_CRC          = 3,
    NRF24_00_CONFIG_CRCO            = 2,
    NRF24_00_CONFIG_PWR_UP          = 1,
    NRF24_00_CONFIG_PRIM_RX         = 0,

    NRF24_02_EN_RXADDR_ERX_P0       = 0,

    NRF24_06_RF_SETUP_RF_DR_LOW     = 5,
    NRF24_06_RF_SETUP_RF_DR_HIGH    = 3,
    NRF24_06_RF_SETUP_RF_PWR_HIGH   = 2,
    NRF24_06_RF_SETUP_RF_PWR_LOW    = 1,

    NRF24_07_STATUS_RX_DR           = 6,
    NRF24_07_STATUS_TX_DS           = 5,
    NRF24_07_STATUS_MAX_RT          = 4,

    NRF24_17_FIFO_STATUS_FX_FULL    = 5,
    NRF24_17_FIFO_STATUS_TX_EMPTY   = 4,
    NRF24_17_FIFO_STATUS_RX_FULL    = 1,
    NRF24_17_FIFO_STATUS_RX_EMPTY   = 0,

    NRF24_1D_FEATURE_EN_DPL         = 2,
    NRF24_1D_FEATURE_EN_ACK_PAY     = 1,
    NRF24_1D_FEATURE_EN_DYN_ACK     = 0,
};

// Pre-shifted and combined bits
enum {
    NRF24_01_EN_AA_ALL_PIPES        = 0x3F,

    NRF24_02_EN_RXADDR_ERX_ALL_PIPES = 0x3F,

    NRF24_03_SETUP_AW_3BYTES        = 0x01,
    NRF24_03_SETUP_AW_4BYTES        = 0x02,
    NRF24_03_SETUP_AW_5BYTES        = 0x03,

    NRF24_04_SETUP_RETR_500uS       = 0x10,

    NRF24_06_RF_SETUP_RF_DR_2Mbps   = 0x08,
    NRF24_06_RF_SETUP_RF_DR_1Mbps   = 0x00,
    NRF24_06_RF_SETUP_RF_DR_250Kbps = 0x20,
    NRF24_06_RF_SETUP_RF_PWR_n18dbm = 0x01,
    NRF24_06_RF_SETUP_RF_PWR_n12dbm = 0x02,
    NRF24_06_RF_SETUP_RF_PWR_n6dbm  = 0x04,
    NRF24_06_RF_SETUP_RF_PWR_0dbm   = 0x06,

    NRF24_07_STATUS_RX_DR_TX_DS_MAX_RT = BV(NRF24_07_STATUS_RX_DR) | BV(NRF24_07_STATUS_TX_DS) | BV(NRF24_07_STATUS_MAX_RT),

    NRF24_1C_DYNPD_ALL_PIPES        = 0x3F,
};

void NRF24_Initialize(void);
uint8_t NRF24_WriteReg(uint8_t reg, uint8_t data);
uint8_t NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t NRF24_WritePayload(const uint8_t *data, uint8_t len);
uint8_t NRF24_ReadReg(uint8_t reg);
uint8_t NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);
uint8_t NRF24_ReadPayload(uint8_t *data, uint8_t len);

void NRF24_FlushRx(void);
void NRF24_FlushTx(void);
uint8_t NRF24_Activate(uint8_t code);

bool NRF24_ReadAvailable(void);
void NRF24_SetStandbyMode(void);
void NRF24_SetRxMode(void);
void NRF24_SetTxMode(void);

// Utility functions
void NRF24_SetRxAddrP0(const uint8_t *addr, int len);
void NRF24_SetTxAddr(const uint8_t *addr, int len);
void NRF24_SetChannel(uint8_t channel);
void NRF24_ClearRxDataReadyInterrupt(void);
void NRF24_ClearAllInterrupts(void);
bool NRF24_IsRxDataReady(void);
bool NRF24_IsRxFifoEmpty(void);
uint8_t NRF24_ReadMostRecentPayload(uint8_t *data, uint8_t length);

