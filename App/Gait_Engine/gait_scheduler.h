
#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <stdint.h>
#include <math.h>
#include "leg_ik.h"     // Access to Hexapod_SolveIK
#include "leg_manager.h" // Access to home_x_mm, home_y_mm
#include "bsp_limits.h"

// --- TUNING CONSTANTS ---
#define BODY_HEIGHT_MM    100.0f  // Standard Walking Height
//#define STEP_HEIGHT_MM    20.0f   // How high the foot lifts
#define STANCE_RADIUS     120.0f  // Radius for turn calculations



// --- GAIT TYPES ---
typedef enum {
    GAIT_TRIPOD = 0,
    GAIT_WAVE   = 1,
    GAIT_RIPPLE = 2
} GaitType_e;

// --- STATE CONTAINER ---
// Holds all "memory" for the gait engine
typedef struct {
    // 1. Dynamic Variables
    float phase;          // 0.0 to 1.0 (The Master Clock)
    float smoothed_x;     // mm/s (Ramped velocity)
    float smoothed_y;     // mm/s
    float smoothed_turn;  // mm/s (Arc length)

    // Body Pose Smoothers
    // We store the "Physical" pose here, which lags behind the "Command" pose
    BodyPose_t smoothed_pose;

    // --- TERRAIN ADAPTATION ---
    float ground_offset[6];   // The height correction (mm) for each leg
    uint8_t is_grounded[6];   // Flag: Has this leg touched down yet?
    float liftoff_offset[6]; //  Memory for smooth liftoff


    // 2. Configuration
    float accel_rate;     // mm/s^2 (Slew Rate)
    float angle_rate;     // Angular Acceleration (deg/s^2)
    float ground_speed;   // Phase multiplier (Frequency)

} Gait_State_t;

typedef enum {
    noFeedBack_Walk = 0,
	FeedBack_Walk = 1
} Walking_Mode_e;

// --- API ---
void Gait_Init(Gait_State_t *state);

/**
 * @brief  Calculates leg targets combining Gait + Body Kinematics.
 * @param  state       Pointer to the Gait State struct
 * @param  cmd_pose    Desired Body Pose (Roll, Pitch, Yaw, Z-Height, etc.)
 * @param  cmd_stride  Desired Walking Stride (X/Y) and Turn (deg)
 * @param  cmd_turn_deg  Desired rotation in deg
 * @param  gait_type  Walking gait selector
 * @param  walking_mode  Walking mode selector
 * @param  dt          Delta Time (seconds)
 */
void Gait_Update(Gait_State_t *state, BodyPose_t *cmd_pose, float cmd_stride_x, float cmd_stride_y, float cmd_turn_deg, GaitType_e gait_type, Walking_Mode_e walking_mode,float STEP_HEIGHT_MM, float dt);

#endif /* GAIT_SCHEDULER_H */
