#include "nrf24.h"
#include "Arduino.h"

extern void CSN_LO();
extern void CSN_HI();
extern void CE_LO();
extern void CE_HI();
extern uint8_t nrf24TransferByte(uint8_t data);
#define UNUSED(x) (void)(x)


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

// standby configuration, used to simplify switching between RX, TX, and Standby modes
static uint8_t standbyConfig;

void NRF24L01_Initialize(uint8_t baseConfig)
{
    standbyConfig = BV(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
    CE_LO();
    // nRF24L01+ needs 1500 microseconds settling time from PowerOnReset to PowerDown mode, we conservatively use more
    delayMicroseconds(5000);
    // power up into standby mode
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig);
    // nRF24L01+ needs 100 microseconds settling time from PowerDown mode to Standby mode
    delayMicroseconds(120);
}

#if !defined(UNIT_TEST)

uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(W_REGISTER | (REGISTER_MASK & reg));
    nrf24TransferByte(data);
    CSN_HI();
    return ret;
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *data, uint8_t length)
{
    CSN_LO();
    const uint8_t ret = nrf24TransferByte(W_REGISTER | ( REGISTER_MASK & reg));
    for (uint8_t i = 0; i < length; i++) {
        nrf24TransferByte(data[i]);
    }
    CSN_HI();
    return ret;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    CSN_LO();
    nrf24TransferByte(R_REGISTER | (REGISTER_MASK & reg));
    const uint8_t data = nrf24TransferByte(NOP);
    CSN_HI();
    return data;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
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
 * Should call NRF24L01_IsRxFifoEmpty first to check availability.
 */
uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
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
uint8_t NRF24L01_WritePayload(const uint8_t *data, uint8_t length)
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
void NRF24L01_FlushRx(void)
{
    // this discards everything in the receive FIFO
    CSN_LO();
    nrf24TransferByte(FLUSH_RX);
    CSN_HI();
}

/**
 * Empty the transmit FIFO buffer. This is generally not required in standard operation.
 */
void NRF24L01_FlushTx(void)
{
    // this discards everything in the transmit FIFO
    CSN_LO();
    nrf24TransferByte(FLUSH_TX);
    CSN_HI();
}
#endif // defined UNIT_TEST

/*
 * Enter standby mode
 */
void NRF24L01_SetStandbyMode(void)
{
    // set CE low and clear the PRIM_RX bit to enter standby mode
    CE_LO();
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig);
}

/*
 * Enter receive mode
 */
void NRF24L01_SetRxMode(void)
{
    CE_LO(); // drop into standby mode
    // set the PRIM_RX bit
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, standbyConfig | BV(NRF24L01_00_CONFIG_PRIM_RX));
    NRF24L01_ClearAllInterrupts();
    // finally set CE high to start enter RX mode
    CE_HI();
    // nRF24L01+ needs 130 microseconds settling time from Standby mode to RX mode
    delayMicroseconds(130);
}

/*
 * Enter transmit mode. Anything in the transmit FIFO will be transmitted.
 */
void NRF24L01_SetTxMode(void)
{
    // Ensure in standby mode, since can only enter TX mode from standby mode
    NRF24L01_SetStandbyMode();
    NRF24L01_ClearAllInterrupts();
    // pulse CE for 10 microseconds to enter TX mode
    CE_HI();
    delayMicroseconds(10);
    CE_LO();
    // nRF24L01+ needs 130 microseconds settling time from Standby mode to TX mode
    delayMicroseconds(130);
}

/*
 * Sets the RX address.
 * User must have previously set address width using NRF24L01_03_SETUP_AW.
 */
void NRF24L01_SetRxAddrP0(const uint8_t* addr, int len)
{
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, addr, len);
}

/*
 * Sets the TX address.
 * User must have previously set address width using NRF24L01_03_SETUP_AW.
 */
void NRF24L01_SetTxAddr(const uint8_t *addr, int len)
{
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, addr, len);
}

void NRF24L01_SetChannel(uint8_t channel)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
}

void NRF24L01_ClearAllInterrupts(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24L01_WriteReg(NRF24L01_07_STATUS, NRF24L01_07_STATUS_RX_DR_TX_DS_MAX_RT);
}

void NRF24L01_ClearRxDataReadyInterrupt(void)
{
    // Writing to the STATUS register clears the specified interrupt bits
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_STATUS_RX_DR));
}

bool NRF24L01_IsRxDataReady(void)
{
    return NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_STATUS_RX_DR);
}

bool NRF24L01_IsRxFifoEmpty(void)
{
    return NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY);
}

bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length)
{
    if (NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) {
        return false;
    }
    NRF24L01_ReadPayload(data, length);
    return true;
}

/*
 * Basic initialization of NRF24, suitable for most protocols
 */
void NRF24L01_InitializeBasic(uint8_t rfChannel, const uint8_t *rxTxAddr, uint8_t payloadSize)
{
    NRF24L01_Initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO));
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    NRF24L01_SetChannel(rfChannel);

    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, 5);
    // RX_ADDR for pipes P2 to P5 are left at default values

    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize);
}
