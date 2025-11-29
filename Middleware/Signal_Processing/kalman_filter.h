/*
 * kalman_filter.h
 * 1D Kalman Filter for IMU Sensor Fusion (Angle + Bias)
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    // State Vector
    float angle; // The calculated angle (output)
    float bias;  // The calculated gyro bias (drift)

    // Error Covariance Matrix (2x2)
    float P[2][2];

    // Tuning Parameters
    float Q_angle; // Process noise for angle
    float Q_bias;  // Process noise for bias
    float R_measure; // Measurement noise (variance of accelerometer)
} Kalman_t;

/**
 * @brief Initializes the Kalman Filter struct with default tuning values.
 * @param kf Pointer to the Kalman struct
 */
void Kalman_Init(Kalman_t *kf);

/**
 * @brief Computes the new angle based on Gyro rate and Accel angle.
 * * @param kf Pointer to the Kalman struct
 * @param newAngle The raw angle from Accelerometer (Degrees)
 * @param newRate  The raw rate from Gyroscope (Degrees/sec)
 * @param dt       Delta time since last update (Seconds) - CRITICAL
 * @return float   The filtered, clean angle
 */
float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt);

#endif /* KALMAN_FILTER_H */
