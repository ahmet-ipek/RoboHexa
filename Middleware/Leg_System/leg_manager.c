#include "leg_manager.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

// --- ROBOT CONFIGURATION (in mm) ---
#define D_COXA  41.0f
#define D_FEMUR 61.0f
#define D_TIBIA 112.0f

// How far the foot should be from the mounting point when standing (Neutral)
// Typically: Coxa Length + Femur Length ensures the Tibia is vertical (L-Shape)
#define DEFAULT_STANCE_RADIUS_MM  110.0f

// Standard "Home" offsets (from previous calibration steps)
float home_x_mm[6];
float home_y_mm[6];

// --- CALIBRATION DATA ---
typedef struct {
    float slope;  // us per degree
    float offset; // degrees
} Servo_Config_t;

// CALIBRATION TABLE
// [Leg 0-5][Coxa, Femur, Tibia]
static const Servo_Config_t servo_config[6][3] = {
    // Leg 0 (RF)
    { {11.39f, -8.0f}, {-11.11f, -2.0f}, {11.61f, 80.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 1 (LF)
    { {10.83f, 22.0f}, {11.22f, -35.0f}, {-10.1f, 90.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 2 (RM)
    { {11.11f, -6.0f}, {-10.55f, -4.0f}, {11.11f, 75.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 3 (LM)
    { {10.55f, -3.0f}, {11.11f, -10.0f}, {-11.11f, 95.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 4 (RB)
    { {11.39f, 0.0f}, {-10.78f, -24.0f}, {11.11f, 92.0f} }, // Calibration done leg is ready for inverse kinematics
    // Leg 5 (LB)
    { {11.11f, -5.0f}, {10.55f, -2.0f}, {-10.83f, 60.0f} } // Calibration done leg is ready for inverse kinematics
};

/**
 * @brief Neutral angles for each leg and joint (standing pose)
 */
const float SERVO_NEUTRAL[HEXAPOD_LEG_COUNT][JOINTS_PER_LEG] = {
    {  45.0f, -90.0f, 90.0f }, // RF
    { -45.0f, -90.0f, 90.0f }, // LF
    {  0.0f, -90.0f, 90.0f }, // RM
    {  0.0f, -90.0f, 90.0f }, // LM
    { -45.0f, -90.0f, 90.0f }, // RB
    {  45.0f, -90.0f, 90.0f }  // LB
};

// storage for leg geometry
HexapodLeg_t legs[6];

// --- IMPLEMENTATION ---

void Leg_System_Init(void) {
    // --- LEG 0: Right Front ---
    // Position: Forward (+85) and Right (+82.5)
    // Angle: 45 deg (Points diagonal Front-Right)
    Hexapod_InitLeg(&legs[0], D_COXA, D_FEMUR, D_TIBIA, 82.5f, 85.0f, 45.0f);

    // --- LEG 1: Left Front ---
    // Position: Forward (+85) and Left (-82.5)
    // Angle: 135 deg (Points diagonal Front-Left)
    Hexapod_InitLeg(&legs[1], D_COXA, D_FEMUR, D_TIBIA, -82.5f, 85.0f, 135.0f);

    // --- LEG 2: Right Middle ---
    // Position: Centered (0) and Right (+82.5)
    // Angle: 0 deg (Points straight Right)
    Hexapod_InitLeg(&legs[2], D_COXA, D_FEMUR, D_TIBIA, 82.5f, 0.0f, 0.0f);

    // --- LEG 3: Left Middle ---
    // Position: Centered (0) and Left (-82.5)
    // Angle: 180 deg (Points straight Left)
    Hexapod_InitLeg(&legs[3], D_COXA, D_FEMUR, D_TIBIA, -82.5f, 0.0f, 180.0f);

    // --- LEG 4: Right Back ---
    // Position: Back (-85) and Right (+82.5)
    // Angle: -45 deg (Points diagonal Back-Right)
    Hexapod_InitLeg(&legs[4], D_COXA, D_FEMUR, D_TIBIA, 82.5f, -85.0f, -45.0f);

    // --- LEG 5: Left Back ---
    // Position: Back (-85) and Left (-82.5)
    // Angle: -135 deg (Points diagonal Back-Left)
    Hexapod_InitLeg(&legs[5], D_COXA, D_FEMUR, D_TIBIA, -82.5f, -85.0f, -135.0f);

    // 2. Calculate "Home" Coordinates (Neutral Stance)
    // This places the foot at DEFAULT_STANCE_RADIUS_MM away from the mount,
    // in the direction of the leg's angle.
        for(int i=0; i<6; i++) {
            float angle_rad = legs[i].mount_angle_rad; // Already calculated in InitLeg

            // Home = Mount_Point + (Radius * Vector_Direction)
            home_x_mm[i] = legs[i].mount_x + (cosf(angle_rad) * DEFAULT_STANCE_RADIUS_MM);
            home_y_mm[i] = legs[i].mount_y + (sinf(angle_rad) * DEFAULT_STANCE_RADIUS_MM);

        }
}

// Converts Angle -> Pulse using your Slope/Offset table
uint16_t Angle_To_Pulse(uint8_t leg, uint8_t joint, float angle_deg) {
    Servo_Config_t cfg = servo_config[leg][joint];

    // Apply Offset
    float corrected_angle = angle_deg - cfg.offset;

    // Apply Slope
    float pulse = 1500.0f + (corrected_angle * cfg.slope);

    return (uint16_t)pulse;
}


void Leg_Move_To_XYZ(Hexapod_Leg_ID leg_id, float x_offset_mm, float y_offset_mm, float z_mm) {

	Point3D_t target;
	target.x = home_x_mm[leg_id] + x_offset_mm;
	target.y = home_y_mm[leg_id] + y_offset_mm;
	target.z = z_mm;

	// 2. Run the Robust IK Solver
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

//    Leg_Set_Angle_Smoothly(leg_id, JOINT_COXA , &servo_state[leg_id][JOINT_COXA], deg_coxa, 30, 0.1);

    BSP_Servo_Write(leg_id, JOINT_COXA,  Angle_To_Pulse(leg_id, JOINT_COXA,  deg_coxa));
    BSP_Servo_Write(leg_id, JOINT_FEMUR, Angle_To_Pulse(leg_id, JOINT_FEMUR, deg_femur));
    BSP_Servo_Write(leg_id, JOINT_TIBIA, Angle_To_Pulse(leg_id, JOINT_TIBIA, deg_tibia));
}

// Smoothly ramp a value towards a target (Acceleration Control)
static float Smooth_Approach(float current, float target, float step) {
    if (current < target) return fminf(current + step, target);
    if (current > target) return fmaxf(current - step, target);
    return target;
}


/**
 * @brief Current angles of all servos.
 */
static float servo_state[HEXAPOD_LEG_COUNT][JOINTS_PER_LEG];

/**
 * @brief Initialize all servos to their neutral position.
 *        Moves all legs to SERVO_NEUTRAL and updates servo_state array.
 */
void Servo_State_Init(void)
{
    for (int leg = 0; leg < HEXAPOD_LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINTS_PER_LEG; joint++) {
            servo_state[leg][joint] = SERVO_NEUTRAL[leg][joint];

            BSP_Servo_Write(leg, joint, Angle_To_Pulse(leg, joint, servo_state[leg][joint]));
        }
        HAL_Delay(250);
    }
}

/**
 *  @brief Move a single leg joint smoothly towards a target angle.
 */
void Leg_Set_Angle_Smoothly(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float *current, float target, float dps, float dt) {

    // Calculate step based on speed and time
    float step = dps * dt;

    *current = Smooth_Approach(*current, target, step);

    BSP_Servo_Write(leg, joint, Angle_To_Pulse(leg, joint, *current));
}

float* Servo_Get_Current(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint) {
    return &servo_state[leg][joint];
}

void Leg_Move_To_XYZ_Smoothly(Hexapod_Leg_ID leg_id, float x_offset_mm, float y_offset_mm, float z_mm, float max_dps, float dt) {

	Point3D_t target;
	target.x = home_x_mm[leg_id] + x_offset_mm;
	target.y = home_y_mm[leg_id] + y_offset_mm;
	target.z = z_mm;

	// 2. Run the Robust IK Solver
	LegAngles_t rads = Hexapod_SolveIK(&legs[leg_id], target);

    if (!rads.is_reachable) {
        // Handle unreachable target
        // For now, we still move to the clamped position returned by IK
    }

    // Convert Radians to Degrees
    float tgt[3] = {
        rads.theta1 * RAD_TO_DEG,
        rads.theta2 * RAD_TO_DEG,
        rads.theta3 * RAD_TO_DEG
    };

    // Current angles
    float *cur[3] = {
        &servo_state[leg_id][JOINT_COXA],
        &servo_state[leg_id][JOINT_FEMUR],
        &servo_state[leg_id][JOINT_TIBIA]
    };

    // --- 3. Compute deltas ---
    float delta[3] = {
        fabsf(tgt[0] - *cur[0]),
        fabsf(tgt[1] - *cur[1]),
        fabsf(tgt[2] - *cur[2])
    };

    // --- 4. Find largest delta ---
    float max_delta = fmaxf(delta[0], fmaxf(delta[1], delta[2]));

    // Nothing to do
    if (max_delta < 0.001f) return;

    // --- 5. Compute sync time ---
    float T = max_delta / max_dps; // seconds

    // --- 6. Scale speeds ---
    float dps[3] = {
        delta[0] / T,
        delta[1] / T,
        delta[2] / T
    };


    // --- 7. Move joints ---
    Leg_Set_Angle_Smoothly(leg_id, JOINT_COXA,  cur[0], tgt[0], dps[0], dt);
    Leg_Set_Angle_Smoothly(leg_id, JOINT_FEMUR, cur[1], tgt[1], dps[1], dt);
    Leg_Set_Angle_Smoothly(leg_id, JOINT_TIBIA, cur[2], tgt[2], dps[2], dt);
}

/**
 * @brief  Sets a specific joint angle directly (Degrees).
 */
void Leg_Set_Angle(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, float angle_deg) {
    BSP_Servo_Write(leg, joint, Angle_To_Pulse(leg, joint, angle_deg));
}


void Leg_Update_Pose(BodyPose_t pose) {
    for (int i = 0; i < HEXAPOD_LEG_COUNT; i++) {

        // 1. Start with the Neutral Foot Position
        Point3D_t foot_vector;
        foot_vector.x = home_x_mm[i];
        foot_vector.y = home_y_mm[i];

        // CRITICAL FIX: The vector MUST start at the foot tip (e.g. -100mm)
        // If we use 0, the rotation has no "lever arm" to swing the leg.
        // We use the 'z' from the pose command itself as the baseline.
        foot_vector.z = pose.z;

        // 2. Rotate this vector by the INVERSE of the Body Rotation
        // (Handled inside Hexapod_Compute_Body_Transform now)
        // IMPORTANT: We pass a 'zeroed' pose for translation to the helper
        // because we want to rotate the vector in place first,
        // then apply translation manually if needed, or let the function handle it.
        // Let's use the function fully:

        // We temporarily zero the Z translation in the pose passed to the math
        // because we already put the Z-depth into 'foot_vector.z'.
        // If we don't do this, we might double-count the height.
        BodyPose_t rotation_only = pose;
        rotation_only.x = 0;
        rotation_only.y = 0;
        rotation_only.z = 0; // We handle Z height via the vector start point

        Point3D_t target = Hexapod_Compute_Body_Transform(foot_vector, rotation_only);

        // 3. Add Body Translation (Walking/Shifting)
        // If Body moves Forward (+Y), Feet move Backward (-Y) relative to Body.
        target.x -= pose.x;
        target.y -= pose.y;
        // target.z is already correct because foot_vector.z started at pose.z
        // and rotated.

        // 4. Solve IK
        LegAngles_t rads = Hexapod_SolveIK(&legs[i], target);

        // 5. Move Servos
        if (rads.is_reachable) {
             BSP_Servo_Write(i, JOINT_COXA,  Angle_To_Pulse(i, JOINT_COXA,  rads.theta1 * 57.2958f));
             BSP_Servo_Write(i, JOINT_FEMUR, Angle_To_Pulse(i, JOINT_FEMUR, rads.theta2 * 57.2958f));
             BSP_Servo_Write(i, JOINT_TIBIA, Angle_To_Pulse(i, JOINT_TIBIA, rads.theta3 * 57.2958f));
        }
    }
}
