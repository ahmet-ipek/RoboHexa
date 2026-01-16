/* control_pid.c */
#include "control_pid.h"

//void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit, float deadband) {
//    pid->Kp = kp;
//    pid->Ki = ki;
//    pid->Kd = kd;
//    pid->output_limit = limit;
//    pid->integral_limit = limit; // Cap I-term at 100% of max output
//    pid->deadband = deadband;
//    PID_Reset(pid);
//}
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit, float deadband) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_limit = limit;

    // FIX: Scale the internal memory limit so (Ki * sum) can reach 'limit'
    // If Ki is 0.2 and Limit is 30, integral_limit becomes 150.
    // Max I-term = 0.2 * 150 = 30.0 degrees (Full Power).
    if (ki > 1e-5f) {
        pid->integral_limit = limit / ki;
    } else {
        pid->integral_limit = limit;
    }

    pid->deadband = deadband;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller_t *pid) {
    pid->prev_error = 0.0f;
    pid->integral_sum = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float target, float measured, float dt) {
    // 1. Calculate Error
    float error = target - measured;

//    //  APPLY DEADBAND
//	// If error is within +/- deadband, force it to 0.0
//	if (error > -pid->deadband && error < pid->deadband) {
//		error = 0.0f;
//	}

    // 2. Proportional
    float P = pid->Kp * error;

    // 3. Integral (with Anti-Windup)
    pid->integral_sum += error * dt;
    if (pid->integral_sum > pid->integral_limit) pid->integral_sum = pid->integral_limit;
    else if (pid->integral_sum < -pid->integral_limit) pid->integral_sum = -pid->integral_limit;
    float I = pid->Ki * pid->integral_sum;

    // 4. Derivative
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    // 5. Total & Clamp
    float output = P + I + D;
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
