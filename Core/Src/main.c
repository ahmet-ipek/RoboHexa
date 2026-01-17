/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_servos.h"
#include "bsp_imu.h"
#include "bsp_ibus.h"
#include "bsp_nrf24.h"
#include "kalman_filter.h"
#include "leg_manager.h"
#include "leg_ik.h"
#include "init_emotes.h"
#include "gait_scheduler.h"
#include "sensor_fusion.h"
#include "control_pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// debug variables
uint8_t leg=0, joint=0;
uint16_t pulse_us=1500;
float x=0.0, y=0.0, z=-100.0; // x, y and z coordinates of the body centered frame in (mm).
float angle = 0.0; // angle of rotation in degrees.
uint8_t work=0;

// 1. Control Inputs (Volatile is mandatory for variables shared with Interrupts)
float joy_stride_x_mm = 0.0f;
float joy_stride_y_mm = 0.0f; // Start with 0 (Stand still)
float joy_turn_deg    = 0.0f;
uint8_t joy_gait    = 0;

// 2. The Gait Engine Instance
Gait_State_t robot_gait;

// 3. Body Control Inputs (For IMU or Manual)
BodyPose_t current_pose = {0};

uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint16_t ch4;
uint16_t ch5;
uint16_t ch6;

uint8_t currentmode = 0;
// Timing (For Debouncing without blocking)
uint32_t last_button_press = 0;
uint32_t last_gait_switch = 0;

// Global Tuning for Balance Strength
float Kppitch = 0.3f;
float Kipitch = 4.5f;
float Kdpitch = 0.02f;

float Kproll = 0.25f;
float Kiroll = 4.5f;
float Kdroll = 0.015f;



Fusion_State_t ori = {0};


// PID Instances for Pitch and Roll
PID_Controller_t pid_pitch;
PID_Controller_t pid_roll;

// Shared Variables (Bridge between Fast Loop and Slow Loop)
float balance_pitch_output = 0.0f;
float balance_roll_output = 0.0f;

// --- SCHEDULER FLAGS ---
volatile uint8_t flag_50hz_gait = 0;
volatile uint8_t flag_200hz_gait = 0;

float pitchRef = 0.0f;
float rollRef = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* TIM6 PeriodElapsedCallback (Configured for 200Hz) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        flag_200hz_gait = 1;


        static uint8_t cnt = 0;  // persists across interrupts
        cnt++;

        if (cnt >= 4)            // 200Hz / 4 = 50Hz
        {
            cnt = 0;
            flag_50hz_gait = 1;
//            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

// Helper: Map function
float Map_Float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Deadzone helper (Stick drift removal)
float Apply_Deadzone(float val, float limit) {
    if (fabsf(val) < limit) return 0.0f;
    return val;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  I2C1_BusRecovery();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	// 1. Hardware Init: Start PWM Timers
	BSP_Servo_Init();

	// 2. Init Middleware (Math & Physics)
	Leg_System_Init();

	Servo_State_Init();
	HAL_Delay(1000);

	Robot_Init();

	// 3. Gait Init
	Gait_Init(&robot_gait);

	// 4. Start the Physics Timer
	HAL_TIM_Base_Start_IT(&htim6);

	// 5. Init IMU
	if (BSP_IMU_Init() == 1) {
		work = 99;
	} else {
		Robot_Sit();
		Error_Handler();
	}

	// 6. Init the complementary filter
	Fusion_Init();

	// 7. Init PID
	PID_Init(&pid_pitch, Kppitch, Kipitch, Kdpitch, 30.0f, 1.2f);
	PID_Init(&pid_roll,  Kproll, Kiroll, Kdroll, 30.0f, 1.2f);

	// 8. Init RC
	BSP_IBUS_Init(&huart4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // === TASK 1: FAST LOOP (200Hz) ===
	  if (flag_200hz_gait) {
		  flag_200hz_gait = 0;

			// 1. Read IMU (Non-blocking check)
			// If the driver is idle, ask for new data
			if (BSP_IMU_Get_State() == IMU_IDLE) {
				BSP_IMU_Start_Read_DMA();
			}

			// 2. Check if we actually have NEW data to process
			MPU6050_Data_t raw = BSP_IMU_Get_Data();

			// 3. Run Sensor Fusion
			Fusion_Update(raw.Accel_X_RAW, raw.Accel_Y_RAW, raw.Accel_Z_RAW, raw.Gyro_X_RAW, raw.Gyro_Y_RAW, raw.Gyro_Z_RAW, 0.005f);

			Fusion_State_t orient = Fusion_Get_Angles();

			if (currentmode == 0) {
			balance_pitch_output = PID_Compute(&pid_pitch, pitchRef, orient.pitch, 0.005f);
			balance_roll_output = PID_Compute(&pid_roll, rollRef, orient.roll, 0.005f);
			}

	  }


		if (flag_50hz_gait) {
			flag_50hz_gait = 0; // Clear flag

			// Apply Balance Correction
			if (currentmode == 0) {
				current_pose.pitch = balance_pitch_output;
				current_pose.roll = balance_roll_output;

				Leg_Update_Pose(current_pose);
			}

			if (currentmode == 3) {
			Gait_Update(&robot_gait, &current_pose, joy_stride_x_mm, joy_stride_y_mm, joy_turn_deg, joy_gait, currentmode, 0.02f);
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
		}


		// --- 1. MODE SWITCHING (Blue Button PC13) ---
		// Logic: Active LOW (Reset).
		// We use a timestamp to "debounce" without stopping the code.
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			uint32_t now = HAL_GetTick();
			if ((now - last_button_press) > 300) // 300ms cooldown
					{
				currentmode++;
				if (currentmode > 3)
					currentmode = 0;

				last_button_press = now;
			}
		}

		// --- 2. READ RC INPUTS ---
		BSP_IBUS_Process(); // Parse any new data
		if (BSP_IBUS_IsConnected()) {
			ch1 = BSP_IBUS_ReadChannel(0);
			ch2 = BSP_IBUS_ReadChannel(1);
			ch3 = BSP_IBUS_ReadChannel(2);
			ch4 = BSP_IBUS_ReadChannel(3);
			ch5 = BSP_IBUS_ReadChannel(4);
			ch6 = BSP_IBUS_ReadChannel(5);

			// --- 3. GAIT SWITCHING GESTURE ---
			// Gesture: VRB Low (<=1100) AND Right Stick Up (>=1800)
			if (ch6 <= 1020 && ch2 >= 1800) {
				uint32_t now = HAL_GetTick();
				if ((now - last_gait_switch) > 500) // 500ms cooldown
						{
					joy_gait++;
					if (joy_gait > 2)
						joy_gait = 0;

					last_gait_switch = now;
				}
			}

			// --- 4. EXECUTE MODES ---
			if (currentmode == 0 || currentmode == 1)
					{
				// Reset Pose Angles (Keep body flat while walking)
				current_pose.roll = 0;
				current_pose.pitch = 0;
				current_pose.yaw = 0;

				// Mapping
				float walkX = Map_Float((float) ch1, 1000.0f, 2000.0f, -60.0f, 60.0f);
				float walkY = Map_Float((float) ch2, 1119.0f, 2000.0f, -60.0f, 60.0f);
				float transY = Map_Float((float) ch5, 1000.0f, 2000.0f, -40.0f, 40.0f);
				float turn_deg = Map_Float((float) ch4, 1000.0f, 2000.0f, -15.0f, 15.0f);

				// Apply Deadzones & Update Globals
				joy_stride_x_mm = Apply_Deadzone(walkX, 10.0f);
				joy_stride_y_mm = Apply_Deadzone(walkY, 10.0f);
				joy_turn_deg = Apply_Deadzone(turn_deg, 2.5f);

				// Pose Mappings (Height & Body Shift)
				current_pose.z = Map_Float((float) ch3, 1118.0f, 2000.0f, 0.0f, -150.0f);
				current_pose.y = Apply_Deadzone(transY, 5.0f);

				// Speed Mapping
				robot_gait.ground_speed = Map_Float((float) ch6, 1000.0f, 2000.0f, 0.0f, 1.0f);
			} else // RC_MODE_POSE
			{
				// Stop Walking
				joy_stride_x_mm = 0.0f;
				joy_stride_y_mm = 0.0f;
				joy_turn_deg = 0.0f;

				// Translation Mappings
				float poseX = Map_Float((float) ch4, 1000.0f, 2000.0f, -40.0f, 40.0f);
				float poseY = Map_Float((float) ch3, 1118.0f, 2000.0f, -40.0f, 40.0f);

				current_pose.x = Apply_Deadzone(poseX, 2.0f);
				current_pose.y = Apply_Deadzone(poseY, 2.0f);
				current_pose.z = Map_Float((float) ch6, 1000.0f, 2000.0f, 0.0f, -150.0f);

				// Rotation Mappings (With manual trim offsets)
				current_pose.roll = Map_Float((float) ch1, 1000.0f, 2000.0f, -30.0f, 30.0f) - 0.4f;
				current_pose.pitch = Map_Float((float) ch2, 1119.0f, 2000.0f, -30.0f, 30.0f) - 4.0f;
				current_pose.yaw = Map_Float((float) ch5, 1000.0f, 2000.0f, -30.0f, 30.0f);
			}
		}

		ori = Fusion_Get_Angles();


//	  Leg_Set_Angle_Smoothly(Leg, joint, Servo_Get_Current(Leg, joint), angle, debug_speed, dt) ;
//	  Leg_Move_To_XYZ_Smoothly(Leg, x, y, z, debug_speed, dt);
//	  Leg_Update_Pose(test_pose);
//		            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us);
//		            Leg_Set_Angle(leg, joint, angle) ;


//	  HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // PC3 is the NRF IRQ Pin
    if (GPIO_Pin == GPIO_PIN_3)
    {
        if (BSP_NRF_DataReady())
        {
            uint8_t rx_buffer[32];
            BSP_NRF_ReadPacket(rx_buffer);

            // --- DATA RECEIVED! ---
            // rx_buffer[0] = delay_time2
            // rx_buffer[1] = walk command
            // rx_buffer[2] = init command
            // ... map these to the robot variables later ...

            // --- SEND TELEMETRY BACK ---
            // Create a dummy packet for now
            uint8_t telem_buffer[32] = {0};
            telem_buffer[0] = 77; // Example battery level
            BSP_NRF_WriteAckPayload(1, telem_buffer, 32);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
