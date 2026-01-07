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
uint8_t Leg=0, joint=0;
uint16_t pulse_us=1500;
float x=0.0, y=0.0, z=-100.0; // x, y and z coordinates of the body centered frame in (mm).
float angle = 0.0; // angle of rotation in degrees.
uint8_t work=0;


MPU6050_Data_t sensor_data;



Kalman_t KalmanPitch;
Kalman_t KalmanRoll;
int16_t Robot_Pitch = 0.0f;
int16_t Robot_Roll = 0.0f;
uint32_t last_tick = 0; // For dt calculation


// --- DEBUG COCKPIT (Add these to Live Watch) ---

// 1. Master Switch (0 = Automatic Demo, 1 = Manual Debug Control)
uint8_t debug_mode = 1;

// 2. Body Posture Controls (Smooth Body_Motion Engine)
float debug_body_x = 0.0f;
float debug_body_y = 0.0f;
float debug_body_z = -100.0f; // Default Height

// 3. Walking Controls (Gait Engine)
// Set 'debug_walk_on' to 1 to start moving
uint8_t debug_walk_on = 0;
float debug_stride_x  = 0.0f; // Strafing (mm)
float debug_stride_y  = 0.0f; // Walking Forward (mm)
float debug_turn_deg  = 0.0f; // Turning (deg)
float debug_speed     = 0.5f; // Smoothness
float debug_accel     = 150.0f; // Smoothness

BodyPose_t test_pose = {0};

// 1. Control Inputs (Volatile is mandatory for variables shared with Interrupts)
volatile float joy_stride_x_mm = 0.0f;
volatile float joy_stride_y_mm = 0.0f; // Start with 0 (Stand still)
volatile float joy_turn_deg    = 0.0f;
volatile uint8_t joy_gait    = 0;

// 2. The Gait Engine Instance
Gait_State_t robot_gait;

// 3. Body Control Inputs (For IMU or Manual)
BodyPose_t current_pose = {0};

typedef struct{
	uint8_t ms1;
	uint8_t ms2;
	uint8_t ms3;
	uint8_t ms4;
	uint8_t ms5;
	uint8_t ms6;
}msRead_t;

volatile msRead_t msread;

uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint16_t ch4;
uint16_t ch5;
uint16_t ch6;

RC_Mode_e rc_mode = RC_MODE_WALK; // Default
uint32_t toggle_timer = 0;        // For debouncing the gesture
uint8_t gesture_active = 0;       // Flag

uint8_t currentmode = 0;
// Timing (For Debouncing without blocking)
uint32_t last_button_press = 0;
uint32_t last_gait_switch = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- 50HZ Loop fires automatically every 20ms ---
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {
        Gait_Update(&robot_gait, &current_pose, joy_stride_x_mm, joy_stride_y_mm, joy_turn_deg, joy_gait, 0.02f);
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

	// 6. Init Kalman
	Kalman_Init(&KalmanPitch);
	Kalman_Init(&KalmanRoll);
	last_tick = HAL_GetTick();

	// 7. Init RC
	BSP_IBUS_Init(&huart4);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // 1. Get raw data from BSP
	  BSP_IMU_Start_Read_DMA();

	  MPU6050_Data_t raw = BSP_IMU_Get_Data();

	  // 2. Calculate dt (Delta Time in Seconds)
	  uint32_t current_tick = HAL_GetTick();
	  float dt = (current_tick - last_tick) / 1000.0f;
	  if(dt < 0.001f) dt = 0.001f; // Safety
	  last_tick = current_tick;

	  // 3. Calculate Raw Accelerometer Angles
	  float accel_roll = atan2f((float)raw.Accel_Y_RAW, (float)raw.Accel_Z_RAW) * 57.296f;
	  float accel_pitch = atan2f(-(float)raw.Accel_X_RAW,
	                                 sqrtf((float)raw.Accel_Y_RAW*(float)raw.Accel_Y_RAW +
	                                       (float)raw.Accel_Z_RAW*(float)raw.Accel_Z_RAW)
	                                 	 	 	 	 	 	 	 	 	 	 	 	 	 ) * 57.296f;

	  // 4. Convert Gyro to Deg/Sec
	  float gyro_roll_rate  = (float)raw.Gyro_X_RAW / 131.0f;
	  float gyro_pitch_rate = (float)raw.Gyro_Y_RAW / 131.0f;

	  // 5. Run Kalman Filter
	  Robot_Roll  = Kalman_Update(&KalmanRoll,  accel_roll,  gyro_roll_rate,  dt);
	  Robot_Pitch = Kalman_Update(&KalmanPitch, accel_pitch, gyro_pitch_rate, dt);



	    msread.ms1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
		msread.ms2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		msread.ms3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
		msread.ms4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		msread.ms5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
		msread.ms6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);

		// --- 1. MODE SWITCHING (Blue Button PC13) ---
		// Logic: Active LOW (Reset).
		// We use a timestamp to "debounce" without stopping the code.
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			uint32_t now = HAL_GetTick();
			if ((now - last_button_press) > 300) // 300ms cooldown
					{
				currentmode++;
				if (currentmode > 1)
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
			if (ch6 <= 1100 && ch2 >= 1800) {
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
			if (currentmode == 0) // RC_MODE_WALK
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


//	  Leg_Set_Angle_Smoothly(Leg, joint, Servo_Get_Current(Leg, joint), angle, debug_speed, dt) ;
//	  Leg_Move_To_XYZ_Smoothly(Leg, x, y, z, debug_speed, dt);
//	  Leg_Update_Pose(test_pose);

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
