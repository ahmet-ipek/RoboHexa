/*
 * bsp_nrf24.h
 * Professional nRF24L01+ Driver for Hexapod (Receiver Mode)
 */

#ifndef BSP_NRF24_H
#define BSP_NRF24_H

#include "stm32f4xx_hal.h"

// --- Configuration Constants ---
#define NRF_CHANNEL          52        // Matched to your Legacy Remote
#define NRF_PAYLOAD_SIZE     32        // Fixed 32-byte packet
#define NRF_ADDR_WIDTH       5         // 5-byte address

// --- Register Map ---
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_PW_P1    0x12
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

// --- Commands ---
#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_R_RX_PAYLOAD  0x61
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_NOP           0xFF

// --- Function Prototypes ---

/**
 * @brief  Initializes the nRF24L01+ module over SPI.
 * Configures the radio as a Receiver (PRX) on Channel 52,
 * sets up pipes, enables Auto-Ack, and powers up the radio.
 * @note   Uses hardcoded address {0xAA, 0x44, 0x33, 0x22, 0x11}.
 */
void BSP_NRF_Init(void);

/**
 * @brief  Transitions the radio into Listening Mode.
 * Sets the CE pin High to enable radio reception.
 */
void BSP_NRF_StartListening(void);

/**
 * @brief  Checks if new data has arrived.
 * @return 1 if data is available in RX FIFO, 0 otherwise.
 * @note   This clears the RX_DR status flag if set.
 */
uint8_t BSP_NRF_DataReady(void);

/**
 * @brief  Reads the top packet from the RX FIFO.
 * @param  pData Pointer to a 32-byte buffer to store received data.
 */
void BSP_NRF_ReadPacket(uint8_t* pData);

/**
 * @brief  Loads an Acknowledge Payload to be sent back to the Transmitter.
 * This payload is transmitted automatically when the next packet is received.
 * @param  pipe  The pipe number (usually 1 for this setup).
 * @param  pData Pointer to the data to send.
 * @param  len   Length of data (max 32 bytes).
 */
void BSP_NRF_WriteAckPayload(uint8_t pipe, uint8_t* pData, uint8_t len);

#endif /* BSP_NRF24_H */
