#include "sensor_fusion.h"
#include <math.h>

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f
#define RAD_TO_DEG  57.2957795f
#define FILTER_ALPHA 0.96f

static Fusion_State_t state = {0};

void Fusion_Init(void) {
    state.roll = 0.0f;
    state.pitch = 0.0f;
}

void Fusion_Update(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                   int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                   float dt)
{
    // 1. Convert Units
    float ax = (float)acc_x / ACCEL_SCALE;
    float ay = (float)acc_y / ACCEL_SCALE;
    float az = (float)acc_z / ACCEL_SCALE;

    float gx = (float)gyro_x / GYRO_SCALE;
    float gy = (float)gyro_y / GYRO_SCALE;

    // 2. CALCULATE ANGLES (Adapted for Y-Forward Frame)

    // PITCH (Nose Up/Down)
    // Rotation around X-Axis (Right).
    // We look at Y (Forward) and Z (Up) vectors.
    float accel_pitch = atan2f(ay, az) * RAD_TO_DEG;

    // ROLL (Bank Left/Right)
    // Rotation around Y-Axis (Forward).
    // We look at X (Right) and Z (Up) vectors.
    // Note: The negative sign on 'ax' ensures correct "Right-Hand Rule" polarity
    float accel_roll = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

    // 3. COMPLEMENTARY FILTER
    // Pitch integrates Gyro X (Rotation around Right axis)
    state.pitch = FILTER_ALPHA * (state.pitch + gx * dt) + (1.0f - FILTER_ALPHA) * accel_pitch;

    // Roll integrates Gyro Y (Rotation around Forward axis)
    state.roll  = FILTER_ALPHA * (state.roll  + gy * dt) + (1.0f - FILTER_ALPHA) * accel_roll;
}

Fusion_State_t Fusion_Get_Angles(void) {
    return state;
}
