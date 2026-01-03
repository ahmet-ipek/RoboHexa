/*
 * leg_manager.h
 * Leg Control Interface (Calibration + Hardware Abstraction)
 */

#ifndef LEG_MANAGER_H
#define LEG_MANAGER_H

#include <stdint.h>
#include "bsp_servos.h"
#include "leg_ik.h"

extern const float SERVO_NEUTRAL[HEXAPOD_LEG_COUNT][JOINTS_PER_LEG];

// Standard "Home" offsets (from previous calibration steps)
extern float home_x_mm[6];
extern float home_y_mm[6];

// Expose Leg Geometry for IK Solver
extern HexapodLeg_t legs[6];

/**
 * @brief  Initializes the Leg System.
 * Sets up physical dimensions and mounting offsets for the IK engine.
 * Must be called once before moving legs.
 */
void Leg_System_Init(void);

uint16_t Angle_To_Pulse(uint8_t leg, uint8_t joint, float angle_deg);

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

/**
 * @brief Get the current angle of a servo joint.
 *
 * Returns a pointer to the internal state variable that stores the
 * current angle of the specified joint. This value is typically updated
 * by the servo feedback system and is used by the speed control algorithm.
 *
 * @param leg   Hexapod leg identifier.
 * @param joint Joint identifier within the leg.
 *
 * @return Pointer to the current joint angle.
 *         Units: degrees.
 */
float* Servo_Get_Current(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint);

/**
 * @brief  Updates all 6 legs to match the desired body pose.
 * Automatically handles all 18 servos.
 * @param  pose  Target body position and orientation.
 */
void Leg_Update_Pose(BodyPose_t pose);

#endif /* LEG_MANAGER_H */
