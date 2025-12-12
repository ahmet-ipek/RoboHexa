#include "leg_ik.h"

// Helper: Constrain function for acos domain safety (prevents NaN)
static inline float f_constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void Hexapod_InitLeg(HexapodLeg_t *leg, float L1, float L2, float L3, float mx, float my, float angle_deg) {
    leg->L1 = L1;
    leg->L2 = L2;
    leg->L3 = L3;
    leg->mount_x = mx;
    leg->mount_y = my;

    // Store angle in radians
    leg->mount_angle_rad = angle_deg * (M_PI / 180.0f);

    // Pre-calculate trig values for rotation matrix
    leg->cos_mount_angle = cosf(leg->mount_angle_rad);
    leg->sin_mount_angle = sinf(leg->mount_angle_rad);
}

LegAngles_t Hexapod_SolveIK(HexapodLeg_t *leg, Point3D_t target_body) {
    LegAngles_t angles = {0};
    angles.is_reachable = 1;

    // --- STEP 1: TRANSLATION ---
    // Convert Body coordinates to Leg Root coordinates
    float dx = target_body.x - leg->mount_x;
    float dy = target_body.y - leg->mount_y;
    float dz = target_body.z;

    // --- STEP 2: ROTATION (Body Frame -> Leg Frame) ---
    // Align coordinates so X points along the leg's neutral axis
    float cos_a = leg->cos_mount_angle;
    float sin_a = leg->sin_mount_angle;

    // Standard Rotation Formula with negative angle substitution:
    // x_aligned = dx * cos(-a) - dy * sin(-a) => dx*cos(a) - dy*(-sin(a)) => dx*cos + dy*sin
    // y_aligned = dx * sin(-a) + dy * cos(-a) => dx*(-sin) + dy*cos       => -dx*sin + dy*cos

    float x_aligned =  dx * cos_a + dy * sin_a;
    float y_aligned = -dx * sin_a + dy * cos_a;

    // APPLY COORDINATE FIX (Rotate X 180) ---
    // x' = x
    // y' = -y
    // z' = -z
    float x_leg = x_aligned;
    float y_leg = -y_aligned;
    float z_leg = -dz;

    // --- STEP 3: IK SOLVER (2D Plane) ---

    // A. Coxa Angle (Theta 1) - Top Down View
    angles.theta1 = atan2f(y_leg, x_leg);

    // B. Geometric Setup for Side View
    // Calculate horizontal distance from Hip Axis to Target
    float r_total = sqrtf(x_leg*x_leg + y_leg*y_leg);
    float d = r_total - leg->L1; // Subtract Coxa Length

    // Calculate 3D hypotenuse (Femur Joint to Target)
    float h = sqrtf(d*d + z_leg*z_leg);

    // Safety: Check Reachability
    float max_reach = leg->L2 + leg->L3;
    if (h > max_reach) {
        h = max_reach; // Clamp to max reach to prevent math crash
        angles.is_reachable = 0; // Flag that we are overextended
    }

    // C. Law of Cosines
        float cos_phi = (leg->L2*leg->L2 + leg->L3*leg->L3 - h*h) / (2.0f * leg->L2 * leg->L3);
        float phi_internal = acosf(f_constrain(cos_phi, -1.0f, 1.0f)); // We Use f_constrain to prevent floating point errors (e.g. 1.000001) causing NaN in acos

        float cos_beta = (leg->L2*leg->L2 + h*h - leg->L3*leg->L3) / (2.0f * leg->L2 * h);
        float beta = acosf(f_constrain(cos_beta, -1.0f, 1.0f)); // We use f_constrain to prevent floating point errors (e.g. 1.000001) causing NaN in acos

        // D. Elevation Angle
        float psi = atan2f(z_leg, d);

        // E. Final Calculation
        angles.theta2 = psi - beta;
        angles.theta3 = M_PI - phi_internal;

        return angles;
}
