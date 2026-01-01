#include "kalman_filter.h"

void Kalman_Init(Kalman_t *kf) {
    // Initial State
    kf->angle = 0.0f;
    kf->bias  = 0.0f;

    // Initial Uncertainty
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;

    // Tuning Parameters
    kf->Q_angle = 0.005f;  // Accelerometer trust (Lower = Trust accel more)
    kf->Q_bias  = 0.003f;  // Gyro Drift trust
    kf->R_measure = 0.12f; // Measurement Noise
}

float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt) {
    // 1. PREDICT STEP
    // Estimate new angle based on Gyro Rate (Integration)
    // Angle = Angle + (GyroRate - GyroBias) * dt
    float rate = newRate - kf->bias;
    kf->angle += rate * dt;

    // Update Error Covariance Matrix (P)
    // We are adding Process Noise (Q) to our uncertainty
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 2. UPDATE STEP
    // Calculate the difference between Predicted Angle and Accel Angle
    float S = kf->P[0][0] + kf->R_measure; // Estimate Error

    // Calculate Kalman Gain (K)
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Calculate difference (Innovation)
    float y = newAngle - kf->angle;

    // Correct the State (Angle and Bias)
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // Update Error Covariance Matrix (P) for next loop
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
