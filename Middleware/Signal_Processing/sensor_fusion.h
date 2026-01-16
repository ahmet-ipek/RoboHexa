/* sensor_fusion.h */
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>

// Renamed from IMU_State_t to Fusion_State_t
typedef struct {
    float roll;
    float pitch;
} Fusion_State_t;

void Fusion_Init(void);
void Fusion_Update(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                   int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                   float dt);

// Update return type here too
Fusion_State_t Fusion_Get_Angles(void);

#endif
