
#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <stdint.h>
#include <math.h>
#include "leg_ik.h"     // Access to Hexapod_SolveIK
#include "leg_manager.h" // Access to home_x_mm, home_y_mm

// --- TUNING CONSTANTS ---
#define BODY_HEIGHT_MM    100.0f  // Standard Walking Height
#define STEP_HEIGHT_MM    30.0f   // How high the foot lifts
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
    // 1. Dynamic Variables (Changed by Logic)
    float phase;          // 0.0 to 1.0 (The Master Clock)
    float smoothed_x;     // mm/s (Ramped velocity)
    float smoothed_y;     // mm/s
    float smoothed_turn;  // mm/s (Arc length)

    // 2. Configuration
    float accel_rate;     // mm/s^2 (Slew Rate)
    float ground_speed;   // Phase multiplier (Frequency)

} Gait_State_t;

// --- API ---
void Gait_Init(Gait_State_t *state);

/**
 * @brief  Calculates leg targets for the current moment in time.
 * @param  state      Pointer to the Gait State struct
 * @param  cmd_x      Target Strafe Speed (mm/s)
 * @param  cmd_y      Target Forward Speed (mm/s)
 * @param  cmd_turn   Target Turn Speed (deg/s)
 * @param  gait_type  Tripod, Wave, or Ripple
 * @param  dt         Delta Time since last call (seconds)
 */
void Gait_Update(Gait_State_t *state,
                 float cmd_x, float cmd_y, float cmd_turn,
                 GaitType_e gait_type, float dt);

#endif /* GAIT_SCHEDULER_H */
