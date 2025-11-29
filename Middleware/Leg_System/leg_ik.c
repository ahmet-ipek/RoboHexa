#include "leg_ik.h"

#define RAD_TO_DEG 57.2957795f
#define PI         3.14159265f

Leg_Angles_t IK_Compute(float x, float y, float z) {
    Leg_Angles_t angles = {0};

    // 1. Coxa Angle (Horizontal Plane)
    angles.coxa = atan2f(y, x) * RAD_TO_DEG;

    // 2. Distance from Hip Axis to Foot Tip (Planar)
    float dist_horizontal = sqrtf(x*x + y*y);
    float d_planar = dist_horizontal - L_COXA_CM; // Subtract Coxa length

    // 3. Hypotenuse (Femur Joint to Foot Tip)
    float HF = sqrtf(d_planar*d_planar + z*z);

    // 4. Validity Check (Can the leg physically reach?)
    if (HF > (L_FEMUR_CM + L_TIBIA_CM)) {
        angles.is_valid = 0;
        return angles;
    }
    angles.is_valid = 1;

    // 5. Femur Angle Math (Law of Cosines)
    float alpha1 = atan2f(d_planar, -z); // Angle of HF relative to vertical

    float cos_A2 = (L_FEMUR_CM*L_FEMUR_CM + HF*HF - L_TIBIA_CM*L_TIBIA_CM) / (2.0f * L_FEMUR_CM * HF);

    // Clamp value to [-1, 1] to prevent NaN
    if(cos_A2 > 1.0f) cos_A2 = 1.0f;
    if(cos_A2 < -1.0f) cos_A2 = -1.0f;

    float alpha2 = acosf(cos_A2);

    // Resulting Femur Angle (adjusted for -90 offset if 0 is horizontal)
    angles.femur = (alpha1 + alpha2) * RAD_TO_DEG - 90.0f;

    // 6. Tibia Angle Math
    float cos_B = (L_FEMUR_CM*L_FEMUR_CM + L_TIBIA_CM*L_TIBIA_CM - HF*HF) / (2.0f * L_FEMUR_CM * L_TIBIA_CM);

    if(cos_B > 1.0f) cos_B = 1.0f;
    if(cos_B < -1.0f) cos_B = -1.0f;

    float beta = acosf(cos_B);
    angles.tibia = 180.0f - (beta * RAD_TO_DEG);

    return angles;
}
