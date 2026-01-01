/*
 * leg_manager.h
 * Leg Control Interface (Calibration + Hardware Abstraction)
 */

#ifndef LEG_MANAGER_H
#define LEG_MANAGER_H

#include <stdint.h>
#include "bsp_servos.h" // Needed for Hexapod_Leg_ID enum

extern const float SERVO_NEUTRAL[HEXAPOD_LEG_COUNT][JOINTS_PER_LEG];

/**
 * @brief  Initializes the Leg System.
 * Sets up physical dimensions and mounting offsets for the IK engine.
 * Must be called once before moving legs.
 */
void Leg_System_Init(void);

/**
 * @brief  Moves a specific leg's foot tip to a target 3D coordinate.
 * @param  leg  Which leg to move (LEG_RF, LEG_RM, etc.)
 * @param  x_mm Forward/Backward distance from Hip (mm)
 * @param  y_mm Left/Right distance from Hip (mm)
 * @param  z_mm Height relative to Hip (mm, usually negative)
 */
void Leg_Move_To_XYZ(Hexapod_Leg_ID leg_id, float x_offset_mm, float y_offset_mm, float z_mm);

void Leg_Move_To_XYZ_Smoothly(Hexapod_Leg_ID leg_id, float x_offset_mm, float y_offset_mm, float z_mm, float dps, float dt);

/**
 * @brief  Sets a specific joint angle directly (Degrees).
 */
void Leg_Set_Angle(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float angle_deg);

/**
 * @brief Move a single leg joint smoothly towards a target angle.
 *
 * @param leg       Leg identifier (Hexapod_Leg_ID)
 * @param joint     Joint identifier (Hexapod_Joint_ID)
 * @param current   Pointer to current angle in servo_state array
 * @param target    Target angle in degrees
 * @param dps       Maximum speed in degrees per second
 * @param dt        Time delta since last update (seconds)
 */
void Leg_Set_Angle_Smoothly(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float *current, float target, float dps, float dt);

/**
 * @brief Initialize all servos to their neutral position.
 *        Moves all legs to SERVO_NEUTRAL and updates servo_state array.
 */
void Servo_State_Init(void);

float* Servo_Get_Current(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint);

#endif /* LEG_MANAGER_H */
