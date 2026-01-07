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

	// Clear Pose State
	state->smoothed_pose.x = 0;
	state->smoothed_pose.y = 0;
	state->smoothed_pose.roll = 0;
	state->smoothed_pose.pitch = 0;
	state->smoothed_pose.yaw = 0;

	// Tuning Defaults
	state->accel_rate = 50.0f; 		// mm/s^2 acceleration
	state->angle_rate = 30.0f;  	// For Rotation (deg/s^2)
	state->ground_speed = 0.5f; 	// Hz Cycle Frequency
}

void Gait_Update(Gait_State_t *state, BodyPose_t *cmd_pose, float cmd_x, float cmd_y, float cmd_turn_deg, GaitType_e gait_type, float dt)
{
    // --- STATION 0: BODY POSE SMOOTHING ---
    // Instead of using cmd_pose directly, we ramp state->smoothed_pose towards it.

    // 1. Smooth Translation (X, Y, Z) using accel_rate (mm/s)
    state->smoothed_pose.x = Approach_Float(state->smoothed_pose.x, cmd_pose->x, state->accel_rate * 2.0f, dt);
    state->smoothed_pose.y = Approach_Float(state->smoothed_pose.y, cmd_pose->y, state->accel_rate * 2.0f, dt);
    state->smoothed_pose.z = Approach_Float(state->smoothed_pose.z, cmd_pose->z, state->accel_rate * 2.0f, dt);

    // 2. Smooth Orientation (Roll, Pitch, Yaw) using angle_rate (deg/s)
    state->smoothed_pose.roll  = Approach_Float(state->smoothed_pose.roll,  cmd_pose->roll,  state->angle_rate, dt);
    state->smoothed_pose.pitch = Approach_Float(state->smoothed_pose.pitch, cmd_pose->pitch, state->angle_rate, dt);
    state->smoothed_pose.yaw   = Approach_Float(state->smoothed_pose.yaw,   cmd_pose->yaw,   state->angle_rate, dt);


    BodyPose_t *active_pose = &state->smoothed_pose;

    // --- STATION 1: INPUT FILTERING (Walking) ---
    float cmd_turn_mm = (cmd_turn_deg * (3.14159f / 180.0f)) * STANCE_RADIUS;
    state->smoothed_x    = Approach_Float(state->smoothed_x, cmd_x, state->accel_rate, dt);
    state->smoothed_y    = Approach_Float(state->smoothed_y, cmd_y, state->accel_rate, dt);
    state->smoothed_turn = Approach_Float(state->smoothed_turn, cmd_turn_mm, state->accel_rate * 0.5f, dt);

    uint8_t is_moving = (fabsf(state->smoothed_x) > 1.0f || fabsf(state->smoothed_y) > 1.0f || fabsf(state->smoothed_turn) > 1.0f);

    // --- STATION 2: PHASE CLOCK ---
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

    // --- STATION 3: GAIT PARAMETERS ---
    float stance_fraction = 0.5f;
    if (gait_type == GAIT_WAVE)        stance_fraction = 0.833f;
    else if (gait_type == GAIT_RIPPLE) stance_fraction = 0.666f;
    float swing_duration = 1.0f - stance_fraction;

    // --- STATION 4: KINEMATIC STACK ---
    for (int i = 0; i < 6; i++) {

        // A. Local Phase
        float leg_offset = 0.0f;
        switch (gait_type) {
            case GAIT_WAVE:   leg_offset = (5 - i) * 0.166f; break;
            case GAIT_RIPPLE: {const float r[]={0,0.33,0.66,0.66,0.33,0}; leg_offset=r[i];} break;
            default:          if(i==1||i==2||i==5) leg_offset=0.5f; break;
        }
        float local_phase = state->phase + leg_offset;
        if (local_phase >= 1.0f) local_phase -= 1.0f;

        // B. Gait Generation
        float gait_z = 0.0f;
        float gait_x = 0.0f;
        float gait_y = 0.0f;

        // Calculate Turn Vector
        float turn_scale = state->smoothed_turn / STANCE_RADIUS;
        float vec_x = state->smoothed_x + (-home_y_mm[i] * turn_scale);
        float vec_y = state->smoothed_y + ( home_x_mm[i] * turn_scale);

        if (local_phase < swing_duration) {
            // SWING
            float progress = local_phase / swing_duration;
            gait_z = sinf(progress * 3.14159f) * STEP_HEIGHT_MM;
            float travel = -cosf(progress * 3.14159f);
            gait_x = (vec_x * 0.5f) * travel;
            gait_y = (vec_y * 0.5f) * travel;
        } else {
            // STANCE
            float progress = (local_phase - swing_duration) / stance_fraction;
            gait_z = 0.0f;
            float travel = cosf(progress * 3.14159f);
            gait_x = (vec_x * 0.5f) * travel;
            gait_y = (vec_y * 0.5f) * travel;
        }

        // C. Stacking Layers
        Point3D_t foot_pos;
        foot_pos.x = home_x_mm[i] + gait_x;
        foot_pos.y = home_y_mm[i] + gait_y;

        foot_pos.z = active_pose->z + gait_z;

        BodyPose_t rot_only_pose = *active_pose;
        rot_only_pose.x = 0;
        rot_only_pose.y = 0;
        rot_only_pose.z = 0;

        Point3D_t final_target = Hexapod_Compute_Body_Transform(foot_pos, rot_only_pose);

        // Apply Smoothed Translation
        final_target.x -= active_pose->x;
        final_target.y -= active_pose->y;

        // D. IK
        LegAngles_t sol = Hexapod_SolveIK(&legs[i], final_target);

        if (sol.is_reachable) {
             BSP_Servo_Write(i, JOINT_COXA,  Angle_To_Pulse(i, JOINT_COXA,  sol.theta1 * 57.29f));
             BSP_Servo_Write(i, JOINT_FEMUR, Angle_To_Pulse(i, JOINT_FEMUR, sol.theta2 * 57.29f));
             BSP_Servo_Write(i, JOINT_TIBIA, Angle_To_Pulse(i, JOINT_TIBIA, sol.theta3 * 57.29f));
        }
    }
}
