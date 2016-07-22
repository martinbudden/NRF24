/*
 * This file is part of the Arduino NRF24 library.
 *
 * Written by Martin Budden
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License, <http://www.gnu.org/licenses/>, for
 * more details.
 *
 * All the above text and this condition must be included in any redistribution.
 */

#include "Arduino.h"
#include <SPI.h>
#include "nrf24.h"


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

NRF24L01::NRF24L01(uint8_t _ce_pin, uint8_t _csn_pin)
    : ce_pin(_ce_pin), csn_pin(_csn_pin)
{
    pinMode(ce_pin, OUTPUT);
    pinMode(csn_pin, OUTPUT);
    SPI.begin();
}

void NRF24L01::initialize(uint8_t baseConfig, uint8_t _rfDataRate)
{
    standbyConfig = BV(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
    rfDataRate = NRF24L01_06_RF_SETUP_RF_DR_MASK & _rfDataRate;
    NRF24_CE_LO();
    // nRF24L01+ needs 4500 microseconds settling time from PowerOnReset to PowerDown mode, we conservatively use more
    delayMicroseconds(5000);
    // power up into standby mode
    writeReg(NRF24L01_00_CONFIG, standbyConfig);
    // nRF24L01+ needs 100 microseconds settling time from PowerDown mode to Standby mode
    delayMicroseconds(120);
    writeReg(NRF24L01_06_RF_SETUP, rfDataRate);
}

void NRF24L01::ENABLE_NRF24(void)
{
    digitalWrite(csn_pin, LOW);
    delayMicroseconds(5);
}

void NRF24L01::DISABLE_NRF24(void)
{
    digitalWrite(csn_pin, HIGH);  
    delayMicroseconds(5);
}

void NRF24L01::NRF24_CE_LO(void)
{
    digitalWrite(ce_pin, LOW);
}

void NRF24L01::NRF24_CE_HI(void)
{
    digitalWrite(ce_pin, HIGH);  
}

uint8_t NRF24L01::spiTransferByte(uint8_t data)
{
    return SPI.transfer(data);
}

uint8_t NRF24L01::writeReg(uint8_t reg, uint8_t data)
{
    ENABLE_NRF24();
    spiTransferByte(W_REGISTER | (REGISTER_MASK & reg));
    spiTransferByte(data);
    DISABLE_NRF24();
    return true;
}

uint8_t NRF24L01::writeRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = spiTransferByte(W_REGISTER | ( REGISTER_MASK & reg));
    for (int ii = 0; ii < length; ++ii) {
        spiTransferByte(data[ii]);
    }
    DISABLE_NRF24();
    return ret;
}

/*
 * Transfer the payload to the nRF24L01 TX FIFO
 * Packets in the TX FIFO are transmitted when the
 * nRF24L01 next enters TX mode
 */
uint8_t NRF24L01::writePayload(const uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = spiTransferByte(W_TX_PAYLOAD);
    for (int ii = 0; ii < length; ++ii) {
        spiTransferByte(data[ii]);
    }
    DISABLE_NRF24();
    return ret;
}

uint8_t NRF24L01::readReg(uint8_t reg)
{
    ENABLE_NRF24();
    spiTransferByte(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t data = spiTransferByte(NOP);
    DISABLE_NRF24();
    return data;
}

uint8_t NRF24L01::readRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = spiTransferByte(R_REGISTER | (REGISTER_MASK & reg));
    for (int ii = 0; ii < length; ++ii) {
        data[ii] = spiTransferByte(NOP);
    }
    DISABLE_NRF24();
    return ret;
}

/*
 * Read a packet from the nRF24L01 RX FIFO.
 */
uint8_t NRF24L01::readPayload(uint8_t *data, uint8_t length)
{
    ENABLE_NRF24();
    const uint8_t ret = spiTransferByte(R_RX_PAYLOAD);
    for (int ii = 0; ii < length; ++ii) {
        data[ii] = spiTransferByte(NOP);
    }
    DISABLE_NRF24();
    return ret;
}

/*
 * Empty the transmit FIFO buffer.
 */
void NRF24L01::flushTx()
{
    ENABLE_NRF24();
    spiTransferByte(FLUSH_TX);
    DISABLE_NRF24();
}

/*
 * Empty the receive FIFO buffer.
 */
void NRF24L01::flushRx()
{
    ENABLE_NRF24();
    spiTransferByte(FLUSH_RX);
    DISABLE_NRF24();
}

/*
 * Enter standby mode
 */
void NRF24L01::setStandbyMode(void)
{
    // set CE low and clear the PRIM_RX bit to enter standby mode
    NRF24_CE_LO();
    writeReg(NRF24L01_00_CONFIG, standbyConfig);
}

/*
 * Enter receive mode
 */
void NRF24L01::setRxMode(void)
{
    NRF24_CE_LO(); // drop into standby mode
    // set the PRIM_RX bit
    writeReg(NRF24L01_00_CONFIG, standbyConfig | BV(NRF24L01_00_CONFIG_PRIM_RX));
    clearAllInterrupts();
    // finally set CE high to start enter RX mode
    NRF24_CE_HI();
    // nRF24L01+ will now transition from Standby mode to RX mode after 130 microseconds settling time
}

/*
 * Enter transmit mode. Anything in the transmit FIFO will be transmitted.
 */
void NRF24L01::setTxMode(void)
{
    // Ensure in standby mode, since can only enter TX mode from standby mode
    setStandbyMode();
    clearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    NRF24_CE_HI();
    delayMicroseconds(10);
    NRF24_CE_LO();
    // nRF24L01+ will now transition from Standby mode to TX mode after 130 microseconds settling time.
    // Transmission will then begin and continue until TX FIFO is empty.
}

void NRF24L01::clearAllInterrupts(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    writeReg(NRF24L01_07_STATUS, BV(NRF24L01_07_STATUS_RX_DR) | BV(NRF24L01_07_STATUS_TX_DS) | BV(NRF24L01_07_STATUS_MAX_RT));
}

void NRF24L01::setChannel(uint8_t channel)
{
    writeReg(NRF24L01_05_RF_CH, channel);
}

uint8_t NRF24L01::getChannel(void)
{
    return readReg(NRF24L01_05_RF_CH);
}

void NRF24L01::setRfPower(uint8_t rfPower)
{
    writeReg(NRF24L01_06_RF_SETUP, rfDataRate | (rfPower & NRF24L01_06_RF_SETUP_RF_PWR_MASK));
}

bool NRF24L01::readPayloadIfAvailable(uint8_t *data, uint8_t length)
{
    if (readReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) {
        return false;
    }
    readPayload(data, length);
    return true;
}

