/*
 * bsp_servos.h
 * Hardware Abstraction for MG996R Servos on STM32F446RE
 */

#ifndef BSP_SERVOS_H
#define BSP_SERVOS_H

#include "stm32f4xx_hal.h"

// Standard Servo definitions
#define SERVO_MIN_PULSE_US   500   // 0 degrees
#define SERVO_MAX_PULSE_US   2500  // 180 degrees

// Total Servo Count
#define HEXAPOD_LEG_COUNT    6
#define JOINTS_PER_LEG       3

// Leg Identifiers
typedef enum {
    LEG_RF = 0,
    LEG_LF = 1,
    LEG_RM = 2,
    LEG_LM = 3,
    LEG_RB = 4,
    LEG_LB = 5
} Hexapod_Leg_ID;

// Joint Identifiers
typedef enum {
    JOINT_COXA = 0, // Hip
    JOINT_FEMUR,    // Thigh
    JOINT_TIBIA     // Shin
} Hexapod_Joint_ID;


// Function Prototypes
void BSP_Servo_Init(void);
void BSP_Servo_Write(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, uint16_t pulse_us);
void BSP_Servo_Stop_All(void);

#endif /* BSP_SERVOS_H */
