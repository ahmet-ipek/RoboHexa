#ifndef BSP_LIMITS_H
#define BSP_LIMITS_H

#include "main.h" // Access to HAL and GPIO definitions

// Reads the contact state of a specific leg (0-5)
// Returns: 1 if touching ground, 0 if in air
uint8_t BSP_Limit_Read(uint8_t leg_index);


#endif
