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

static inline float Clamp_Float(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


void Gait_Init(Gait_State_t *state) {
	state->phase = 0.0f;
	state->smoothed_x = 0.0f;
	state->smoothed_y = 0.0f;
	state->smoothed_turn = 0.0f;

	// Clear Pose State
	state->smoothed_pose.x = 0;
	state->smoothed_pose.y = 0;
	state->smoothed_pose.z = -100;
	state->smoothed_pose.roll = 0;
	state->smoothed_pose.pitch = 0;
	state->smoothed_pose.yaw = 0;

	// Tuning Defaults
	state->accel_rate = 50.0f; 		// mm/s^2 acceleration
	state->angle_rate = 30.0f;  	// For Rotation (deg/s^2)
	state->ground_speed = 0.5f; 	// Hz Cycle Frequency

}

void Gait_Update(Gait_State_t *state, BodyPose_t *cmd_pose, float cmd_x, float cmd_y, float cmd_turn_deg, GaitType_e gait_type, Walking_Mode_e walking_mode, float dt)
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

        uint8_t contact = BSP_Limit_Read(i);

        float turn_scale = state->smoothed_turn / STANCE_RADIUS;
        float vec_x = state->smoothed_x + (-home_y_mm[i] * turn_scale);
        float vec_y = state->smoothed_y + ( home_x_mm[i] * turn_scale);

        uint8_t mode = walking_mode;

        if (local_phase < swing_duration) {
            // SWING
            float progress = local_phase / swing_duration;

            if (mode == 1)
            {
                // 1. TRANSITION LOGIC (Start of Swing)
                // Instead of just clearing variables, we capture the state.
                if (progress < 0.05f)
                {
                    // Now safe to clear the "Ground Lock" for the new step
                    state->ground_offset[i] = 0.0f;
                    state->is_grounded[i] = 0;
                }
                // 2. CALCULATE TRAJECTORY (BLENDED)
                // A. The Standard Step (Sine Wave)
                //	                        float sine_lift = sinf(progress * 3.14159f) * STEP_HEIGHT_MM;
                float sine_lift = sinf(progress * 3.14159f) * (-state->smoothed_pose.z * 0.5);

                // B. The Blend Out (Linear decay of the old offset)
                // This ensures we start at +20mm and fade to 0mm by the end
                float blend_lift = state->liftoff_offset[i] * (1.0f - progress);

                // Combine them
                gait_z = sine_lift + blend_lift;

                float travel = -cosf(progress * 3.14159f);
                gait_x = (vec_x * 0.5f) * travel;
                gait_y = (vec_y * 0.5f) * travel;

                // 3. CONTACT DETECTION (Search for ground)
                // Only search on the way DOWN (second half of swing)
                if (progress > 0.5f && !state->is_grounded[i])
                {
                    if (contact)
                    {
                        state->is_grounded[i] = 1;
                        state->ground_offset[i] = gait_z;

                        // Critical: Clear the liftoff memory so it doesn't interfere next time
                        state->liftoff_offset[i] = 0.0f;
                    }
                }

                // 4. Override Z if grounded
                if (state->is_grounded[i])
                {
                    gait_z = state->ground_offset[i];
                }
            }
            else{

                gait_z = sinf(progress * 3.14159f) * STEP_HEIGHT_MM; // LIFT

                float travel = -cosf(progress * 3.14159f);
                gait_x = (vec_x * 0.5f) * travel;
                gait_y = (vec_y * 0.5f) * travel;
            }

        } else {

            if (mode == 1)
            {
                // === STANCE PHASE (Ground) ===
                float progress = (local_phase - swing_duration) / stance_fraction;

                // 1. Maintain Contact Height
                // Instead of Z=0, we keep the offset we found (e.g., +10mm for a rock)
                gait_z = state->ground_offset[i];

                float travel = cosf(progress * 3.14159f);
                gait_x = (vec_x * 0.5f) * travel;
                gait_y = (vec_y * 0.5f) * travel;
                if (progress > 0.9f)
                    state->liftoff_offset[i] = state->ground_offset[i];
            }
            else
            {

                // STANCE
                float progress = (local_phase - swing_duration) / stance_fraction;
                gait_z = 0.0f; // On ground

                float travel = cosf(progress * 3.14159f);
                gait_x = (vec_x * 0.5f) * travel;
                gait_y = (vec_y * 0.5f) * travel;
            }
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

		// --- D. SOLVE IK ---
		LegAngles_t sol = Hexapod_SolveIK(&legs[i], final_target);

        if (sol.is_reachable) {
             BSP_Servo_Write(i, JOINT_COXA,  Angle_To_Pulse(i, JOINT_COXA,  sol.theta1 * 57.29f));
             BSP_Servo_Write(i, JOINT_FEMUR, Angle_To_Pulse(i, JOINT_FEMUR, sol.theta2 * 57.29f));
             BSP_Servo_Write(i, JOINT_TIBIA, Angle_To_Pulse(i, JOINT_TIBIA, sol.theta3 * 57.29f));
        }
    }
}
