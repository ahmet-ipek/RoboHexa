/*
 * leg_ik.h
 * Robust Inverse Kinematics for 3-DOF Hexapod Leg
 */

#ifndef LEG_IK_H
#define LEG_IK_H

#include <math.h>
#include <stdint.h>

// Standard PI definition if missing
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Data Structures ---

typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

typedef struct {
    float theta1; // Coxa Angle (Radians)
    float theta2; // Femur Angle (Radians)
    float theta3; // Tibia Angle (Radians)
    uint8_t is_reachable; // 1 if valid, 0 if target out of range
} LegAngles_t;

typedef struct {
    // Physical Dimensions (mm)
    float L1; // Coxa Length
    float L2; // Femur Length
    float L3; // Tibia Length

    // Mounting Offsets (mm) relative to Body Center
    float mount_x;
    float mount_y;

    // Mounting Angle (Radians)
    // Pre-calculating sin/cos saves cycles in the main loop
    float mount_angle_rad;
    float cos_mount_angle;
    float sin_mount_angle;
} HexapodLeg_t;

// --- BODY KINEMATICS ---

typedef struct {
    // Translation (Shift body in mm)
    float x;
    float y;
    float z;

    // Orientation (Rotate body in degrees)
    float roll;
    float pitch;
    float yaw;
} BodyPose_t;

// --- Function Prototypes ---

/**
 * @brief  Initializes a leg's geometry and pre-calculates trig values.
 * @param  leg Pointer to the leg struct
 * @param  L1, L2, L3 Physical link lengths (mm)
 * @param  mx, my Mounting offset from body center (mm)
 * @param  angle_deg Mounting angle (0=Right, 90=Front, etc.)
 */
void Hexapod_InitLeg(HexapodLeg_t *leg, float L1, float L2, float L3, float mx, float my, float angle_deg);

/**
 * @brief  Solves Inverse Kinematics for a target point.
 * @param  leg Pointer to the configured leg struct
 * @param  target_body Target coordinate (x,y,z) relative to leg root (or body center if offsets used)
 * @return LegAngles_t struct containing angles in Radians
 */
LegAngles_t Hexapod_SolveIK(HexapodLeg_t *leg, Point3D_t target_body);

/**
 * @brief  Applies a 3D rotation/translation matrix to a point.
 * Used to calculate where the foot "should be" when the body moves.
 * * @param  p_input     Original point (x,y,z) relative to body center (Neutral)
 * @param  pose        Requested Body Pose (Rotation + Translation)
 * @return Point3D_t   New point relative to the tilted body center
 */
Point3D_t Hexapod_Compute_Body_Transform(Point3D_t p_foot_neutral, BodyPose_t pose);

#endif /* LEG_IK_H */
