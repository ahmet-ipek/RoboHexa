#include "bsp_servos.h"
#include "tim.h"

// struct to hold hardware mapping
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} Servo_Hardware_Map_t;

// Lookup Table: [LEG_ID][JOINT_ID]
static const Servo_Hardware_Map_t SERVO_MAP[HEXAPOD_LEG_COUNT][JOINTS_PER_LEG] = {
    // Leg 1 (RF) - TIM3 CH4, TIM12 CH2, TIM12 CH1
    { {&htim3, TIM_CHANNEL_4}, {&htim12, TIM_CHANNEL_2}, {&htim12, TIM_CHANNEL_1} },
    // Leg 2 (LF) - TIM2 CH4, TIM1 CH4, TIM3 CH3
    { {&htim2, TIM_CHANNEL_4}, {&htim1, TIM_CHANNEL_4},  {&htim3, TIM_CHANNEL_3}  },
    // Leg 3 (RM) - TIM1 CH3, TIM2 CH2, TIM2 CH3
    { {&htim1, TIM_CHANNEL_3}, {&htim2, TIM_CHANNEL_2},  {&htim2, TIM_CHANNEL_3}  },
    // Leg 4 (LM) - TIM4 CH3, TIM4 CH4, TIM3 CH1
    { {&htim4, TIM_CHANNEL_3}, {&htim4, TIM_CHANNEL_4},  {&htim3, TIM_CHANNEL_1}  },
    // Leg 5 (RB) - TIM1 CH1, TIM2 CH1, TIM8 CH2
    { {&htim1, TIM_CHANNEL_1}, {&htim2, TIM_CHANNEL_1},  {&htim8, TIM_CHANNEL_2}  },
    // Leg 6 (LB) - TIM1 CH2, TIM8 CH1, TIM3 CH2
    { {&htim1, TIM_CHANNEL_2}, {&htim8, TIM_CHANNEL_1},  {&htim3, TIM_CHANNEL_2}  }
};

void BSP_Servo_Init(void) {
    // Start PWM for all defined channels
    for(int i = 0; i < HEXAPOD_LEG_COUNT; i++) {
        for(int j = 0; j < JOINTS_PER_LEG; j++) {
            TIM_HandleTypeDef *timer = SERVO_MAP[i][j].htim;
            uint32_t channel = SERVO_MAP[i][j].channel;

            // Initialize to Neutral (1500us)
//            __HAL_TIM_SET_COMPARE(timer, channel, 1500);
            HAL_TIM_PWM_Start(timer, channel);
        }
    }
}


void BSP_Servo_Write(Hexapod_Leg_ID leg, Hexapod_Joint_ID joint, uint16_t pulse_us) {
    if(leg >= HEXAPOD_LEG_COUNT || joint >= JOINTS_PER_LEG) return;

    // Safety Clamping
    if(pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if(pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    TIM_HandleTypeDef *timer = SERVO_MAP[leg][joint].htim;
    uint32_t channel = SERVO_MAP[leg][joint].channel;

    __HAL_TIM_SET_COMPARE(timer, channel, pulse_us);
}

void BSP_Servo_Stop_All(void) {
    for(int i = 0; i < HEXAPOD_LEG_COUNT; i++) {
        for(int j = 0; j < JOINTS_PER_LEG; j++) {
            HAL_TIM_PWM_Stop(SERVO_MAP[i][j].htim, SERVO_MAP[i][j].channel);
        }
    }
}
