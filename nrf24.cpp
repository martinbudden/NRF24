#include "nrf24.h"
#include "Arduino.h"

extern void CSN_LO();
extern void CSN_HI();
extern void CE_LO();
extern void CE_HI();
extern uint8_t nrf24TransferByte(uint8_t data);

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

// Instruction Mnemonics
// nRF24L01:  Table 16. Command set for the nRF24L01 SPI. Product Specification, p46
// nRF24L01+: Table 20. Command set for the nRF24L01+ SPI. Product Specification, p51
enum {
    R_REGISTER    = 0x00,
    W_REGISTER    = 0x20,
    REGISTER_MASK = 0x1F,
    R_RX_PAYLOAD  = 0x61,
    W_TX_PAYLOAD  = 0xA0,
    FLUSH_TX      = 0xE1,
    FLUSH_RX      = 0xE2,
    REUSE_TX_PL   = 0xE3,
    ACTIVATE      = 0x50, // used by nRF24L01, not by nRF24L01+
    R_RX_PL_WID   = 0x60,
    W_ACK_PAYLOAD = 0xA8,
    W_TX_PAYLOAD_NO_ACK = 0xB0,
    NOP           = 0xFF
};

// standby configuration: power up, 2 byte CRC
#define NRF24_STANDBY_CONFIG (BV(NRF24_00_CONFIG_PWR_UP) | BV(NRF24_00_CONFIG_EN_CRC) | BV( NRF24_00_CONFIG_CRCO))

void NRF24_Initialize(void)
{
    // power up into standby mode, 2 byte CRC
    CE_LO();
    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too.
    delayMicroseconds(5000);
    NRF24_WriteReg(NRF24_00_CONFIG, NRF24_STANDBY_CONFIG);
    delayMicroseconds(5000);
}

#if !defined(UNIT_TEST)

uint8_t NRF24_WriteReg(uint8_t reg, uint8_t data)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(W_REGISTER | (REGISTER_MASK & reg));
    nrf24TransferByte(data);
    CSN_HI();
    return ret;
}

uint8_t NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    CSN_HI();
    return ret;
}

uint8_t NRF24_ReadReg(uint8_t reg)
{
    CSN_LO();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    const uint8_t data = nrf24TransferByte(NOP);
    CSN_HI();
    return data;
}

uint8_t NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    for(uint8_t i = 0; i < length; i++) {
        data[i] = nrf24TransferByte(NOP);
    }
    CSN_HI();
    return ret;
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 * Should call NRF24_IsRxFifoEmpty first to check availability.
 */
uint8_t NRF24_ReadPayload(uint8_t *data, uint8_t length)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < length; i++) {
        data[i] = nrf24TransferByte(NOP);
    }
    CSN_HI();
    return ret;
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24_WritePayload(const uint8_t *data, uint8_t length)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    CSN_HI();
    return ret;
}

/**
 * Empty the receive FIFO buffer.
 */
void NRF24_FlushRx(void)
{
    // this discards everything in the receive FIFO
    CSN_LO();
    nrf24TransferByte(FLUSH_RX);
    CSN_HI();
}

/**
 * Empty the transmit FIFO buffer. This is generally not required in standard operation.
 */
void NRF24_FlushTx(void)
{
    // this discards everything in the transmit FIFO
    CSN_LO();
    nrf24TransferByte(FLUSH_TX);
    CSN_HI();
}
#endif // defined UNIT_TEST

uint8_t NRF24_Activate(uint8_t code)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(ACTIVATE);
    nrf24TransferByte(code);
    CSN_HI();
    return ret;
}

/*
 * Returns true if there is a packet available to be read in the nRF20L01 RX FIFO.
 */
bool NRF24_ReadAvailable(void)
{
    if (NRF24_ReadReg(NRF24_17_FIFO_STATUS) & BV(NRF24_17_FIFO_STATUS_RX_EMPTY)) {
        return false;
    } else {
        return true;
    }
}

void NRF24_SetStandbyMode(void)
{
    // set CE low and clear the PRIM_RX bit to enter standby mode
    CE_LO();
    NRF24_WriteReg(NRF24_00_CONFIG, NRF24_STANDBY_CONFIG);
}

void NRF24_SetRxMode(void)
{
    CE_LO(); // drop into standby mode
    // set the PRIM_RX bit
    NRF24_WriteReg(NRF24_00_CONFIG, NRF24_STANDBY_CONFIG | BV(NRF24_00_CONFIG_PRIM_RX));
    NRF24_ClearAllInterrupts();
    // finally set CE high to start listening
    CE_HI();
    // may take up to 130 microseconds to enter RX mode
    delayMicroseconds(130);
}

void NRF24_SetTxMode(void)
{
    // Ensure in standby mode, since can only enter TX mode from standby mode
    NRF24_SetStandbyMode();
    NRF24_ClearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    CE_HI();
    delayMicroseconds(10);
    CE_LO();
    // may take up to 130 microseconds to enter TX mode
    delayMicroseconds(130);
}

/*
 * Sets the RX address.
 * User must have previously set address width using NRF24_03_SETUP_AW.
 */
void NRF24_SetRxAddrP0(const uint8_t* addr, int len)
{
    NRF24_WriteRegisterMulti(NRF24_0A_RX_ADDR_P0, addr, len);
}

/*
 * Sets the TX address.
 * User must have previously set address width using NRF24_03_SETUP_AW.
 */
void NRF24_SetTxAddr(const uint8_t *addr, int len)
{
    NRF24_WriteRegisterMulti(NRF24_10_TX_ADDR, addr, len);
}

void NRF24_SetChannel(uint8_t channel)
{
    NRF24_WriteReg(NRF24_05_RF_CH, channel);
}

void NRF24_ClearAllInterrupts(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24_WriteReg(NRF24_07_STATUS, NRF24_07_STATUS_RX_DR_TX_DS_MAX_RT);
}

void NRF24_ClearRxDataReadyInterrupt(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24_WriteReg(NRF24_07_STATUS, BV(NRF24_07_STATUS_RX_DR));
}

bool NRF24_IsRxDataReady(void)
{
    return (NRF24_ReadReg(NRF24_07_STATUS) & BV(NRF24_07_STATUS_RX_DR));
}

bool NRF24_IsRxFifoEmpty(void)
{
    return NRF24_ReadReg(NRF24_17_FIFO_STATUS) & BV(NRF24_17_FIFO_STATUS_RX_EMPTY);
}

/*
 * Reads the payloads in the FIFO, discarding all but the most recent.
 * Returns the total number of payloads read, zero if no payloads were available
 */
uint8_t NRF24_ReadMostRecentPayload(uint8_t *data, uint8_t length)
{
    uint8_t payloadCount = 0;
    if (NRF24_ReadReg(NRF24_07_STATUS) & BV(NRF24_07_STATUS_RX_DR)) {  // data ready
        NRF24_WriteReg(NRF24_07_STATUS, NRF24_07_STATUS_RX_DR_TX_DS_MAX_RT); // clear interrupts
        while (!(NRF24_ReadReg(NRF24_17_FIFO_STATUS) & BV(NRF24_17_FIFO_STATUS_RX_EMPTY))) {
            NRF24_ReadPayload(data, length);
            ++payloadCount;
        }
    }
    return payloadCount;
}
