/*
 * leg_manager.h
 * Leg Control Interface (Calibration + Hardware Abstraction)
 */

#ifndef LEG_MANAGER_H
#define LEG_MANAGER_H

#include <stdint.h>
#include "bsp_servos.h" // Needed for Hexapod_Leg_ID enum

/**
 * @brief  Moves a specific leg's foot tip to a target 3D coordinate.
 * This function handles:
 * 1. Inverse Kinematics Math (Coordinate -> Angles)
 * 2. Calibration Trims (Correcting crooked legs)
 * 3. Hardware Mapping (Angles -> PWM Pulses)
 *
 * @param  leg  Which leg to move (LEG_RF, LEG_RM, etc.)
 * @param  x_cm Forward/Backward distance from Hip (cm)
 * @param  y_cm Left/Right distance from Hip (cm)
 * @param  z_cm Height relative to Hip (cm)
 */
void Leg_Move_To_XYZ(Hexapod_Leg_ID leg, float x_cm, float y_cm, float z_cm);

#endif /* LEG_MANAGER_H */
