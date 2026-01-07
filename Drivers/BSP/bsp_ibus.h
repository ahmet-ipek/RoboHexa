/*
 * bsp_ibus.h
 * FlySky i-BUS Driver for STM32
 */

#ifndef BSP_IBUS_H
#define BSP_IBUS_H

#include <stdint.h>
#include "usart.h" // Access to huart handle

// --- CONFIGURATION ---
// Packet is always 32 bytes
#define IBUS_PACKET_SIZE 32
// We use a double buffer (64 bytes) to safely catch overlaps
#define IBUS_BUFFER_SIZE 64

typedef struct {
    // Raw Channel Data (1000 - 2000)
    // CH1: Roll, CH2: Pitch, CH3: Throttle, CH4: Yaw
    uint16_t channels[14];

    // Status Flags
    uint8_t is_connected;    // 1 if receiving valid packets
    uint32_t last_update_ms; // For fail-safe timeout
} IBUS_State_t;

// --- API ---
void BSP_IBUS_Init(UART_HandleTypeDef *huart);
void BSP_IBUS_Process(void); // Call this in main loop (High Frequency)
uint16_t BSP_IBUS_ReadChannel(uint8_t ch_index);
uint8_t BSP_IBUS_IsConnected(void);

#endif
