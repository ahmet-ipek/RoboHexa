#include "bsp_limits.h"

// Map Leg Index to Hardware Pins
// Order: Leg 0, Leg 1, Leg 2, Leg 3, Leg 4, Leg 5
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} LimitSwitch_t;


static const LimitSwitch_t limits[6] = {
    {GPIOC, GPIO_PIN_4},  // MS1 (Leg 0)
    {GPIOC, GPIO_PIN_5},  // MS2 (Leg 1)
    {GPIOC, GPIO_PIN_11}, // MS3 (Leg 2)
    {GPIOB, GPIO_PIN_0},  // MS4 (Leg 3)
    {GPIOC, GPIO_PIN_12}, // MS5 (Leg 4)
    {GPIOC, GPIO_PIN_10}  // MS6 (Leg 5)
};

uint8_t BSP_Limit_Read(uint8_t leg_index) {
    if (leg_index > 5) return 0;

    // Read Pin (Active High: 1 = Pressed/Ground)
    if (HAL_GPIO_ReadPin(limits[leg_index].port, limits[leg_index].pin) == GPIO_PIN_SET) {
        return 1;
    }
    return 0;
}
