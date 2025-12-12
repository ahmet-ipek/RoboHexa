#include "leg_manager.h"
#include "leg_ik.h"

#define RAD_TO_DEG 57.2957795f

// --- ROBOT CONFIGURATION (in mm) ---
#define D_COXA  38.0f
#define D_FEMUR 59.0f
#define D_TIBIA 109.0f

// --- CALIBRATION DATA ---
typedef struct {
    float slope;  // us per degree
    float offset; // degrees
} Servo_Config_t;



// CALIBRATION TABLE
// [Leg 0-5][Coxa, Femur, Tibia]
static const Servo_Config_t servo_config[6][3] = {
    // Leg 0 (RF)
    { {11.11f, 0.0f}, {11.11f, 0.0f}, {11.11f, 90.0f} },
    // Leg 1 (LF)
    { {11.11f, 0.0f}, {-11.11f, 0.0f}, {-11.11f, -90.0f} },
    // Leg 2 (RM)
    { {11.11f, -6.0f}, {-10.55f, -4.0f}, {11.11f, 75.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 3 (LM)
    { {10.55f, -3.0f}, {11.11f, -10.0f}, {-11.11f, 95.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 4 (RB)
    { {11.11f, 0.0f}, {11.11f, 0.0f}, {11.11f, 90.0f} },
    // Leg 5 (LB)
    { {11.11f, -5.0f}, {11.67f, 0.0f}, {-10.83f, 60.0f} } // Calibration done leg is ready for inverse kinematics
};

// Internal storage for leg geometry
static HexapodLeg_t legs[6];

// --- IMPLEMENTATION ---

void Leg_System_Init(void) {
    // --- LEG 0: Right Front ---
    // Position: Forward (+65) and Right (+55)
    // Angle: 45 deg (Points diagonal Front-Right)
    Hexapod_InitLeg(&legs[0], D_COXA, D_FEMUR, D_TIBIA, 55.0f, 65.0f, 45.0f);

    // --- LEG 1: Left Front ---
    // Position: Forward (+65) and Left (-55)
    // Angle: 135 deg (Points diagonal Front-Left)
    Hexapod_InitLeg(&legs[1], D_COXA, D_FEMUR, D_TIBIA, -55.0f, 65.0f, 135.0f);

    // --- LEG 2: Right Middle ---
    // Position: Centered (0) and Right (+55)
    // Angle: 0 deg (Points straight Right)
    Hexapod_InitLeg(&legs[2], D_COXA, D_FEMUR, D_TIBIA, 55.0f, 0.0f, 0.0f);

    // --- LEG 3: Left Middle ---
    // Position: Centered (0) and Left (-55)
    // Angle: 180 deg (Points straight Left)
    Hexapod_InitLeg(&legs[3], D_COXA, D_FEMUR, D_TIBIA, -55.0f, 0.0f, 180.0f);

    // --- LEG 4: Right Back ---
    // Position: Back (-65) and Right (+55)
    // Angle: -45 deg (Points diagonal Back-Right)
    Hexapod_InitLeg(&legs[4], D_COXA, D_FEMUR, D_TIBIA, 55.0f, -65.0f, -45.0f);

    // --- LEG 5: Left Back ---
    // Position: Back (-65) and Left (-55)
    // Angle: -135 deg (Points diagonal Back-Left)
    Hexapod_InitLeg(&legs[5], D_COXA, D_FEMUR, D_TIBIA, -55.0f, -65.0f, -135.0f);
}

// Converts Angle -> Pulse using your Slope/Offset table
static uint16_t Angle_To_Pulse(uint8_t leg, uint8_t joint, float angle_deg) {
    Servo_Config_t cfg = servo_config[leg][joint];

    // Apply Offset
    float corrected_angle = angle_deg - cfg.offset;

    // Apply Slope
    float pulse = 1500.0f + (corrected_angle * cfg.slope);

    return (uint16_t)pulse;
}

void Leg_Move_To_XYZ(Hexapod_Leg_ID leg_id, float x_mm, float y_mm, float z_mm) {

    Point3D_t target;
    target.x = x_mm;
    target.y = y_mm;
    target.z = z_mm;

    // Run IK Solver
    LegAngles_t rads = Hexapod_SolveIK(&legs[leg_id], target);

    if (!rads.is_reachable) {
        // Handle unreachable target
        // For now, we still move to the clamped position returned by IK
    }

    // Convert Radians to Degrees
    float deg_coxa  = rads.theta1 * RAD_TO_DEG;
    float deg_femur = rads.theta2 * RAD_TO_DEG;
    float deg_tibia = rads.theta3 * RAD_TO_DEG;

    // Send to Hardware
    // Note: The IK returns geometry assuming:
    // Femur 0 = Horizontal.
    // Tibia ~90 = Vertical Down (calculated as internal angle beta).
    // calibration table handles the mapping to pulses.

    BSP_Servo_Write(leg_id, JOINT_COXA,  Angle_To_Pulse(leg_id, JOINT_COXA,  deg_coxa));
    BSP_Servo_Write(leg_id, JOINT_FEMUR, Angle_To_Pulse(leg_id, JOINT_FEMUR, deg_femur));
    BSP_Servo_Write(leg_id, JOINT_TIBIA, Angle_To_Pulse(leg_id, JOINT_TIBIA, deg_tibia));
}

void Leg_Set_Angle(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float angle_deg) {
    BSP_Servo_Write(leg, joint, Angle_To_Pulse(leg, joint, angle_deg));
}
