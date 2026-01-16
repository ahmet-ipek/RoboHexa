/* control_pid.h */
#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include <stdint.h>

typedef struct {
    // --- Tuning Parameters ---
    float Kp;  // Proportional (Spring)
    float Ki;  // Integral (Memory)
	float Kd;  // Derivative (Damper)

	// --- Limits ---
	float output_limit;   // Max angle (e.g., +/- 15 deg)
	float integral_limit; // Anti-windup cap

	// NEW: Noise Suppression
	float deadband; // e.g., 0.5 degrees

	// --- State ---
	float prev_error;
	float integral_sum;

} PID_Controller_t;

// Initialize the PID structure
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit, float deadband);

// Reset memory (useful when switching modes)
void PID_Reset(PID_Controller_t *pid);

// Compute correction. Call this at a fixed rate (e.g. 200Hz)
float PID_Compute(PID_Controller_t *pid, float target, float measured, float dt);

#endif
