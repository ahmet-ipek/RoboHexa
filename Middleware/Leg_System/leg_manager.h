/*
 * leg_manager.h
 * Leg Control Interface (Calibration + Hardware Abstraction)
 */

#ifndef LEG_MANAGER_H
#define LEG_MANAGER_H

#include <stdint.h>
#include "bsp_servos.h" // Needed for Hexapod_Leg_ID enum

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
void Leg_Move_To_XYZ(Hexapod_Leg_ID leg, float x_mm, float y_mm, float z_mm);

/**
 * @brief  Sets a specific joint angle directly (Degrees).
 */
void Leg_Set_Angle(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float angle_deg);

#endif /* LEG_MANAGER_H */
