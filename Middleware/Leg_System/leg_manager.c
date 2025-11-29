#include "leg_ik.h"
#include "bsp_servos.h"

// Calibration Trims
// Index: [Leg 0-5][Coxa, Femur, Tibia]
static float servo_trims[6][3] = {
    {0, 0, 0}, // RF
    {0, 0, 0}, // RM
    {0, 0, 0}, // RB
    {0, 0, 0}, // LF
    {0, 0, 0}, // LM
    {0, 0, 0}  // LB
};

// Map Degrees to Pulse Width
// 0 deg = 1500us. Range +/- 90 deg = +/- 1000us
uint16_t Angle_To_Pulse(float angle, float trim) {
    float target = angle + trim;

    // Linear Mapping: 1500 + (Angle * 11.11)
    // 90 * 11.11 = 1000. 1500+1000 = 2500 (Max)
    uint16_t pulse = 1500 + (uint16_t)(target * 11.11f);
    return pulse;
}

/**
 * @brief Public API to move a leg to a coordinate
 */
void Leg_Move_To_XYZ(Hexapod_Leg_ID leg, float x, float y, float z) {
    // 1. Math
    Leg_Angles_t angles = IK_Compute(x, y, z);

    if (!angles.is_valid) return;

    // 2. Calibration & Hardware Update
    // Note: We cast the 'leg' enum to int for array indexing
    BSP_Servo_Write(leg, JOINT_COXA,  Angle_To_Pulse(angles.coxa,  servo_trims[leg][0]));
    BSP_Servo_Write(leg, JOINT_FEMUR, Angle_To_Pulse(angles.femur, servo_trims[leg][1]));
    BSP_Servo_Write(leg, JOINT_TIBIA, Angle_To_Pulse(angles.tibia, servo_trims[leg][2]));
}
