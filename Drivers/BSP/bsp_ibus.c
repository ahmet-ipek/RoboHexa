/*
 * bsp_ibus.c
 */

#include "bsp_ibus.h"
#include <string.h> // For memcpy/memset

static UART_HandleTypeDef *ibus_uart;
static uint8_t dma_buffer[IBUS_BUFFER_SIZE]; // Circular buffer
static IBUS_State_t ibus_state;

void BSP_IBUS_Init(UART_HandleTypeDef *huart) {
    ibus_uart = huart;

    // Clear state
    memset(&ibus_state, 0, sizeof(IBUS_State_t));

    // Start Circular DMA
    // It will fill dma_buffer repeatedly.
    HAL_UART_Receive_DMA(ibus_uart, dma_buffer, IBUS_BUFFER_SIZE);
}

// Helper: Validate Checksum
static uint8_t IBUS_CheckSum(uint8_t *packet) {
    uint16_t sum = 0xFFFF;
    // Sum first 30 bytes
    for (int i = 0; i < 30; i++) {
        sum -= packet[i];
    }
    // Compare with last 2 bytes (Little Endian)
    uint16_t rx_checksum = packet[30] | (packet[31] << 8);
    return (sum == rx_checksum);
}

void BSP_IBUS_Process(void) {
    // We scan the buffer for the header 0x20 0x40
    // Note: This is a simplified scanner.
    // In strict industrial code, we would track DMA pointers.
    // For this project, scanning the buffer is fast enough.

    for (int i = 0; i < (IBUS_BUFFER_SIZE - IBUS_PACKET_SIZE); i++) {
        if (dma_buffer[i] == 0x20 && dma_buffer[i+1] == 0x40) {

            // Candidate packet found. Check Integrity.
            if (IBUS_CheckSum(&dma_buffer[i])) {

                // --- PARSE CHANNELS ---
                // Data starts at index 2. Each channel is 2 bytes (Little Endian).
                for (int ch = 0; ch < 14; ch++) {
                    uint16_t val = dma_buffer[i + 2 + (ch * 2)] |
                                  (dma_buffer[i + 3 + (ch * 2)] << 8);

                    ibus_state.channels[ch] = val;
                }

                // Update Heartbeat
                ibus_state.is_connected = 1;
                ibus_state.last_update_ms = HAL_GetTick();

                // Optimization: Clear the header so we don't re-read this packet
                dma_buffer[i] = 0;
                return; // Done for this cycle
            }
        }
    }

    // FAIL-SAFE: Check timeout (e.g., signal lost > 100ms)
    if ((HAL_GetTick() - ibus_state.last_update_ms) > 100) {
        ibus_state.is_connected = 0;
    }
}

uint16_t BSP_IBUS_ReadChannel(uint8_t ch_index) {
    if (ch_index >= 14) return 1500; // Safety center
    return ibus_state.channels[ch_index];
}

uint8_t BSP_IBUS_IsConnected(void) {
    return ibus_state.is_connected;
}
