/*
 * gait_scheduler.c
 */

#include "gait_scheduler.h"

// Helper: Time-Based Linear Ramp
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
    state->accel_rate = 20.0f; // mm/s^2 acceleration
    state->ground_speed = 0.5f; // Hz Cycle Frequency
}

void Gait_Update(Gait_State_t *state, BodyPose_t *cmd_pose, float cmd_x, float cmd_y, float cmd_turn_deg, GaitType_e gait_type, float dt)
{
    // --- 1. INPUT FILTERING (Smoothing) ---
    float cmd_turn_mm = (cmd_turn_deg * (3.14159f / 180.0f)) * STANCE_RADIUS;
    state->smoothed_x    = Approach_Float(state->smoothed_x, cmd_x, state->accel_rate, dt);
    state->smoothed_y    = Approach_Float(state->smoothed_y, cmd_y, state->accel_rate, dt);
    state->smoothed_turn = Approach_Float(state->smoothed_turn, cmd_turn_mm, state->accel_rate * 0.5f, dt);

    uint8_t is_moving = (fabsf(state->smoothed_x) > 1.0f ||
                         fabsf(state->smoothed_y) > 1.0f ||
                         fabsf(state->smoothed_turn) > 1.0f);

    // --- 2. PHASE CLOCK ---
    if (is_moving) {
        state->phase += state->ground_speed * dt;
    } else {
        if (state->phase > 0.05f && state->phase < 0.95f) {
            state->phase += state->ground_speed * dt;
        } else {
            state->phase = 0.0f;
        }
    }
    if (state->phase >= 1.0f) state->phase -= 1.0f;

    // --- 3. GAIT SETUP ---
    float stance_fraction = 0.5f;
    if (gait_type == GAIT_WAVE)        stance_fraction = 0.833f;
    else if (gait_type == GAIT_RIPPLE) stance_fraction = 0.666f;
    float swing_duration = 1.0f - stance_fraction;

    // --- 4. LEG LOOP (THE KINEMATIC STACK) ---
    for (int i = 0; i < 6; i++) {

        // A. Calculate Local Phase
        float leg_offset = 0.0f;
        switch (gait_type) {
            case GAIT_WAVE:
            	leg_offset = (5 - i) * 0.166f;
            	break;
            case GAIT_RIPPLE:
            {const float r[]={0,0.33,0.66,0.66,0.33,0};
            leg_offset=r[i];}
            break;
            default:
            	if(i==1||i==2||i==5)
            		leg_offset=0.5f;
            	break;
        }

        float local_phase = state->phase + leg_offset;
        if (local_phase >= 1.0f) local_phase -= 1.0f;

        // B. Calculate GAIT COORDINATES
        // These are relative to the "Home" position
        float gait_x = 0.0f;
        float gait_y = 0.0f;
        float gait_z = 0.0f;

        float turn_scale = state->smoothed_turn / STANCE_RADIUS;
        float vec_x = state->smoothed_x + (-home_y_mm[i] * turn_scale);
        float vec_y = state->smoothed_y + ( home_x_mm[i] * turn_scale);

        if (local_phase < swing_duration) {
            // SWING
            float progress = local_phase / swing_duration;
            gait_z = sinf(progress * 3.14159f) * STEP_HEIGHT_MM; // LIFT

            float travel = -cosf(progress * 3.14159f);
            gait_x = (vec_x * 0.5f) * travel;
            gait_y = (vec_y * 0.5f) * travel;
        } else {
            // STANCE
            float progress = (local_phase - swing_duration) / stance_fraction;
            gait_z = 0.0f; // On ground

            float travel = cosf(progress * 3.14159f);
            gait_x = (vec_x * 0.5f) * travel;
            gait_y = (vec_y * 0.5f) * travel;
        }

		// --- C. STACKING THE LAYERS ---

		// 1. Start with Home
		Point3D_t foot_pos;
		foot_pos.x = home_x_mm[i];
		foot_pos.y = home_y_mm[i];

		// 2. Add Gait
		foot_pos.x += gait_x;
		foot_pos.y += gait_y;

		// 3. Set Z-Height manually
		foot_pos.z = cmd_pose->z + gait_z;

		// 4. Apply Body Kinematics
		// We create a temporary pose struct that ONLY has rotation.
		// We set X/Y/Z translation to 0 because we already handled offsets above.
		BodyPose_t rot_only_pose = *cmd_pose;
		rot_only_pose.x = 0;
		rot_only_pose.y = 0;
		rot_only_pose.z = 0;

		// Now rotate the foot vector around the body center
		Point3D_t final_target = Hexapod_Compute_Body_Transform(foot_pos, rot_only_pose);

		// 5. Apply Body Translation
		final_target.x -= cmd_pose->x;
		final_target.y -= cmd_pose->y;

		// --- D. SOLVE IK ---
		LegAngles_t sol = Hexapod_SolveIK(&legs[i], final_target);

        if (sol.is_reachable) {
             BSP_Servo_Write(i, JOINT_COXA,  Angle_To_Pulse(i, JOINT_COXA,  sol.theta1 * 57.29f));
             BSP_Servo_Write(i, JOINT_FEMUR, Angle_To_Pulse(i, JOINT_FEMUR, sol.theta2 * 57.29f));
             BSP_Servo_Write(i, JOINT_TIBIA, Angle_To_Pulse(i, JOINT_TIBIA, sol.theta3 * 57.29f));
        }
    }
}
