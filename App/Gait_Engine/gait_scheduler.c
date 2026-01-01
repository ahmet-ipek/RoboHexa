/*
 * gait_scheduler.c
 */

#include "gait_scheduler.h"

// Helper: Time-Based Linear Ramp (Replaces Smooth_Approach)
static float Approach_Float(float current, float target, float rate, float dt) {
    float step = rate * dt;
    float error = target - current;

    if (error > step)       return current + step;
    else if (error < -step) return current - step;
    else                    return target;
}

void Gait_Init(Gait_State_t *state) {
    state->phase = 0.0f;
    state->smoothed_x = 0.0f;
    state->smoothed_y = 0.0f;
    state->smoothed_turn = 0.0f;

    // Tuning Defaults
    state->accel_rate = 20.0f; // 150 mm/s^2 acceleration
    state->ground_speed = 0.5f; // 1.0 Hz Cycle Frequency
}

void Gait_Update(Gait_State_t *state,
                 float cmd_x, float cmd_y, float cmd_turn_deg,
                 GaitType_e gait_type, float dt)
{
    // --- 1. INPUT FILTERING (Slew Rate Limiting) ---
    // Convert Turn Degrees/s -> Arc Length mm/s
    // Arc = Angle(rad) * Radius
    float cmd_turn_mm = (cmd_turn_deg * (3.14159f / 180.0f)) * STANCE_RADIUS;

    // Smooth inputs to prevent jerky motion
    state->smoothed_x    = Approach_Float(state->smoothed_x, cmd_x, state->accel_rate, dt);
    state->smoothed_y    = Approach_Float(state->smoothed_y, cmd_y, state->accel_rate, dt);
    state->smoothed_turn = Approach_Float(state->smoothed_turn, cmd_turn_mm, state->accel_rate * 0.5f, dt);

    // Check if robot is effectively moving
    uint8_t is_moving = (fabsf(state->smoothed_x) > 1.0f ||
                         fabsf(state->smoothed_y) > 1.0f ||
                         fabsf(state->smoothed_turn) > 1.0f);

    // --- 2. PHASE CLOCK INTEGRATION ---
    if (is_moving) {
        // Run phase clock
        state->phase += state->ground_speed * dt;
    } else {
        // COAST TO NEUTRAL:
        // If we stop input, keep stepping until phase reaches 0 (Start of cycle).
        // This prevents freezing with a leg in the air.
        if (state->phase > 0.05f && state->phase < 0.95f) {
            state->phase += state->ground_speed * dt;
        } else {
            state->phase = 0.0f; // Lock at neutral
        }
    }

    // Wrap Phase (0.0 -> 1.0)
    if (state->phase >= 1.0f) state->phase -= 1.0f;

    // --- 3. GAIT DEFINITIONS ---
    float stance_fraction = 0.5f; // Default Tripod

    if (gait_type == GAIT_WAVE)        stance_fraction = 0.833f; // 5/6 on ground
    else if (gait_type == GAIT_RIPPLE) stance_fraction = 0.666f; // 2/3 on ground

    float swing_duration = 1.0f - stance_fraction;

    // --- 4. LEG LOOP ---
    for (int i = 0; i < 6; i++) {

        // A. Calculate Local Phase Offset
        float leg_offset = 0.0f;

        switch (gait_type) {
            case GAIT_WAVE:
                // 5->4->3->2->1->0
                leg_offset = (5 - i) * 0.166f;
                break;
            case GAIT_RIPPLE:
                // Pairs [0,5], [1,4], [2,3]
                // Using a lookup table is faster, but switch is okay here
                {
                    const float rip[] = {0.0f, 0.33f, 0.66f, 0.66f, 0.33f, 0.0f};
                    leg_offset = rip[i];
                }
                break;
            default: // TRIPOD
                // Group A (0,3,4) vs Group B (1,2,5)
                if (i == 1 || i == 2 || i == 5) leg_offset = 0.5f;
                break;
        }

        // B. Compute Local Leg Phase (0.0 -> 1.0)
        float local_phase = state->phase + leg_offset;
        if (local_phase >= 1.0f) local_phase -= 1.0f;

        // C. Calculate Trajectory Offsets
        float x_off = 0.0f;
        float y_off = 0.0f;
        float z_h   = -BODY_HEIGHT_MM; // Note: Z is usually negative for feet

        // Calculate "Total Velocity Vector" for this leg
        // (Combines Linear Walk + Turning Tangent)
        // Tangent Vector = (-Y, X)
        // Turn factor scales the arc length
        float turn_scale = state->smoothed_turn / STANCE_RADIUS;

        // We need the Home X/Y to calculate the turn tangent
        // Assuming access to global home_x_mm/home_y_mm from leg_manager.h
        float vec_x = state->smoothed_x + (-home_y_mm[i] * turn_scale);
        float vec_y = state->smoothed_y + ( home_x_mm[i] * turn_scale);

        if (local_phase < swing_duration) {
            // === SWING PHASE (In Air) ===
            // 0.0 -> 1.0 progress through swing
            float progress = local_phase / swing_duration;

            // Z Trajectory: Sine Wave (Lift foot)
            z_h += sinf(progress * 3.14159f) * STEP_HEIGHT_MM;

            // XY Trajectory: Move from -Vector/2 to +Vector/2 (Forward)
            // -cos varies from -1 to +1
            float travel = -cosf(progress * 3.14159f);

            // Multiply by 0.5 to get half-stride in each direction
            // "50.0f" is an arbitrary Stride Amplitude scaler,
            // In reality, vec_x IS the speed (mm/s).
            // Distance = Speed * Time.
            // But for a simple gait, we treat 'vec_x' as the Stride Length directly.
            x_off = (vec_x * 0.5f) * travel;
            y_off = (vec_y * 0.5f) * travel;

        } else {
            // === STANCE PHASE (On Ground) ===
            // 0.0 -> 1.0 progress through stance
            float progress = (local_phase - swing_duration) / stance_fraction;

            // Z Trajectory: Flat on ground
            // z_h stays at -BODY_HEIGHT_MM

            // XY Trajectory: Move from +Vector/2 to -Vector/2 (Drag back)
            // cos varies from +1 to -1
            float travel = cosf(progress * 3.14159f);

            x_off = (vec_x * 0.5f) * travel;
            y_off = (vec_y * 0.5f) * travel;
        }

        // D. Execute Move (Using your High-Level Interface)
        // We reuse the Smooth Move function or simply Move_To_XYZ if called frequently
        // Since we are inside a 50Hz/100Hz loop, immediate update is fine.

        // NOTE: We pass "offsets" to your Leg_Move_To_XYZ.
        // This assumes that function handles Home + Offset.
        Leg_Move_To_XYZ((Hexapod_Leg_ID)i, x_off, y_off, z_h);
    }
}
