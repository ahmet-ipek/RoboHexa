#include "bsp_nrf24.h"
#include "spi.h"
#include "gpio.h"


static const uint8_t RX_ADDR_P1[5] = {0xAA, 0x44, 0x33, 0x22, 0x11};

// --- Low Level SPI Helper Functions ---

/**
 * @brief Pulls CSN (Chip Select) Low to start SPI transaction.
 */
static void CSN_Select(void) {
    // Select Slave (Pull Low) - using PC0 per your hardware
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief Pulls CSN High to end SPI transaction.
 */
static void CSN_Deselect(void) {
    // Release Slave (Pull High)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief Pulls CE (Chip Enable) High to enable Radio RX/TX.
 */
static void CE_Enable(void) {
    // Enable Radio (Pull High) - using PC13 per your hardware
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
 * @brief Pulls CE Low to enter Standby mode.
 */
static void CE_Disable(void) {
    // Disable Radio (Pull Low)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
 * @brief Writes a value to a specific register.
 */
static void Write_Reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = CMD_W_REGISTER | (reg & 0x1F);
    buf[1] = value;

    CSN_Select();
    HAL_SPI_Transmit(&hspi2, buf, 2, 100);
    CSN_Deselect();
}

/**
 * @brief Reads a value from a specific register.
 */
static uint8_t Read_Reg(uint8_t reg) {
    uint8_t cmd = CMD_R_REGISTER | (reg & 0x1F);
    uint8_t value = 0;

    CSN_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, &value, 1, 100);
    CSN_Deselect();
    return value;
}

/**
 * @brief Writes a multi-byte buffer to a register (used for Addresses).
 */
static void Write_Buf(uint8_t reg, const uint8_t* pBuf, uint8_t len) {
    uint8_t cmd = CMD_W_REGISTER | (reg & 0x1F);

    CSN_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi2, (uint8_t*)pBuf, len, 100);
    CSN_Deselect();
}

// --- Public API Functions ---

void BSP_NRF_Init(void) {
    // 1. Reset Pins
    CE_Disable();
    CSN_Deselect();
    HAL_Delay(100); // Wait for radio power stabilization

    // 2. Setup Retries (15 retries, 1500us delay)
    Write_Reg(REG_SETUP_RETR, 0x5F);

    // 3. Setup RF (Channel 52, 2Mbps, 0dBm)
    Write_Reg(REG_RF_CH, NRF_CHANNEL);
    Write_Reg(REG_RF_SETUP, 0x0E);

    // 4. Setup Address Width (5 Bytes)
    Write_Reg(REG_SETUP_AW, 0x03);

    // 5. Setup RX Address
    Write_Buf(REG_RX_ADDR_P1, RX_ADDR_P1, 5);

    // 6. Setup Payload Size (32 Bytes)
    Write_Reg(REG_RX_PW_P1, NRF_PAYLOAD_SIZE);

    // 7. Enable Auto-Ack & RX Pipe 1
    Write_Reg(REG_EN_AA, 0x02);     // Enable AutoAck on Pipe 1
    Write_Reg(REG_EN_RXADDR, 0x02); // Enable RX Pipe 1

    // 8. Enable Dynamic Features (Ack Payload)
    Write_Reg(REG_FEATURE, 0x06);   // EN_DPL | EN_ACK_PAY
    Write_Reg(REG_DYNPD, 0x02);     // Dynamic payload on Pipe 1

    // 9. Clear Status Flags
    Write_Reg(REG_STATUS, 0x70);

    // 10. Power Up (RX Mode, CRC Enabled)
    Write_Reg(REG_CONFIG, 0x0F);
}

void BSP_NRF_StartListening(void) {
    CE_Enable();
    // Short delay for radio transition
    HAL_Delay(1);
}

uint8_t BSP_NRF_DataReady(void) {
    // Check IRQ pin state or Status Register
    uint8_t status = Read_Reg(REG_STATUS);

    // Bit 6 is RX_DR (Data Ready)
    if (status & 0x40) {
        Write_Reg(REG_STATUS, 0x40); // Clear flag
        return 1;
    }
    return 0;
}

void BSP_NRF_ReadPacket(uint8_t* pData) {
    uint8_t cmd = CMD_R_RX_PAYLOAD;

    CSN_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, pData, NRF_PAYLOAD_SIZE, 100);
    CSN_Deselect();

    // Safety: Clear Status again
    Write_Reg(REG_STATUS, 0x40);
}

void BSP_NRF_WriteAckPayload(uint8_t pipe, uint8_t* pData, uint8_t len) {
    // Telemetry data sent back to remote
    uint8_t cmd = CMD_W_ACK_PAYLOAD | (pipe & 0x07);

    CSN_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi2, pData, len, 100);
    CSN_Deselect();
}
