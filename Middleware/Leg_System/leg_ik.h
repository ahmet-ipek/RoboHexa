/*
 * leg_ik.h
 * Inverse Kinematics Math for 3-DOF Hexapod
 */

#ifndef LEG_IK_H
#define LEG_IK_H

#include <math.h>
#include <stdint.h>

// Physical Dimensions (cm)
#define L_COXA_CM    3.8f
#define L_FEMUR_CM   5.9f
#define L_TIBIA_CM   10.9f

typedef struct {
    float coxa;
    float femur;
    float tibia;
    uint8_t is_valid; // 1 if target is reachable
} Leg_Angles_t;

/**
 * @brief Computes joint angles for a target coordinate (x,y,z).
 * @param x_cm Forward/Backward distance from Hip
 * @param y_cm Left/Right distance from Hip
 * @param z_cm Height relative to Hip (usually negative)
 */
Leg_Angles_t IK_Compute(float x_cm, float y_cm, float z_cm);

#endif /* LEG_IK_H */
