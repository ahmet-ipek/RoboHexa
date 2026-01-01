#include "leg_manager.h"
#include "init_emotes.h"
#include <math.h>

void Robot_Init(void)
{
    uint32_t last_tick = HAL_GetTick();
    uint32_t start_tick = last_tick;

    while (1) {

        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_tick = now;

        for (int leg = 0; leg < HEXAPOD_LEG_COUNT; leg++) {
        	Leg_Move_To_XYZ_Smoothly(leg, 0, 0, -100, 20, dt);
        }

        // Exit condition: enough time has passed
        if (HAL_GetTick() - start_tick > 5000) { // 5 seconds
            break;
        }

        HAL_Delay(5);
    }
}
