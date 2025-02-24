/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "stdio.h"
#include "delay.h"
#include "string.h"
#include "dm4310_drv.h"
#include "dm4310_ctrl.h"

#include "usbd_cdc_if.h"
#include "usbsent.h"
//#include "usbreceive.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
int step = 1;
uint32_t Len = 26;
uint8_t receive_data_buffer[26] = {0x00};
int8_t result;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
		dm4310_ctrl_send(&hfdcan1, &motor[Motor1]);
		dm4310_ctrl_send(&hfdcan2, &motor[Motor7]);
//		delay_us(200);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor2]);
		dm4310_ctrl_send(&hfdcan2, &motor[Motor8]);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor3]);
		dm4310_ctrl_send(&hfdcan2, &motor[Motor9]);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor4]);
		dm4310_ctrl_send(&hfdcan2, &motor[MotorA]);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor5]);
		dm4310_ctrl_send(&hfdcan2, &motor[MotorB]);
		dm4310_ctrl_send(&hfdcan1, &motor[Motor6]);
		dm4310_ctrl_send(&hfdcan2, &motor[MotorC]);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint32_t time;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_FDCAN2_Init();
  MX_USB_DEVICE_Init();

  
  /* USER CODE BEGIN 2 */
	can_bsp_init();
	delay_init(480);
		HAL_Delay(500);
	dm4310_motor_init();
		HAL_Delay(2000);
	
	dm4310_enable(&hfdcan1, &motor[Motor1]);dm4310_enable(&hfdcan1, &motor[Motor2]);delay_us(200);
	dm4310_enable(&hfdcan2, &motor[Motor7]);dm4310_enable(&hfdcan2, &motor[Motor8]);delay_us(200);
	dm4310_enable(&hfdcan1, &motor[Motor3]);dm4310_enable(&hfdcan1, &motor[Motor4]);delay_us(200);
	dm4310_enable(&hfdcan2, &motor[Motor9]);dm4310_enable(&hfdcan2, &motor[MotorA]);delay_us(200);
	dm4310_enable(&hfdcan1, &motor[Motor5]);dm4310_enable(&hfdcan1, &motor[Motor6]);delay_us(200);
	dm4310_enable(&hfdcan2, &motor[MotorB]);dm4310_enable(&hfdcan2, &motor[MotorC]);delay_us(200);
	
	HAL_TIM_Base_Start_IT(&htim3);
	
	motor[Motor1].ctrl.kp_set = 0.0f;motor[Motor1].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor7].ctrl.kp_set = 0.0f;motor[Motor7].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor2].ctrl.kp_set = 0.0f;motor[Motor2].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor8].ctrl.kp_set = 0.0f;motor[Motor8].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor3].ctrl.kp_set = 0.0f;motor[Motor3].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor9].ctrl.kp_set = 0.0f;motor[Motor9].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor4].ctrl.kp_set = 0.0f;motor[Motor4].ctrl.pos_set = 0.0f;delay_us(200);
	motor[MotorA].ctrl.kp_set = 0.0f;motor[MotorA].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor5].ctrl.kp_set = 0.0f;motor[Motor5].ctrl.pos_set = 0.0f;delay_us(200);
	motor[MotorB].ctrl.kp_set = 0.0f;motor[MotorB].ctrl.pos_set = 0.0f;delay_us(200);
	motor[Motor6].ctrl.kp_set = 0.0f;motor[Motor6].ctrl.pos_set = 0.0f;delay_us(200);
	motor[MotorC].ctrl.kp_set = 0.0f;motor[MotorC].ctrl.pos_set = 0.0f;delay_us(200);

	for(int m = 1; m <= 100; m++){
		motor[Motor1].ctrl.kp_set = 1.5f * m;motor[Motor2].ctrl.kp_set = 0.75f * m;
		motor[Motor7].ctrl.kp_set = 1.5f * m;motor[Motor8].ctrl.kp_set = 0.75f * m;
		motor[Motor3].ctrl.kp_set = 1.5f * m;motor[Motor4].ctrl.kp_set = 0.75f * m;
		motor[Motor9].ctrl.kp_set = 1.5f * m;motor[MotorA].ctrl.kp_set = 0.75f * m;
		motor[Motor5].ctrl.kp_set = 0.75f * m;motor[Motor6].ctrl.kp_set = 0.75f * m;
		motor[MotorB].ctrl.kp_set = 0.75f * m;motor[MotorC].ctrl.kp_set = 0.75f * m;delay_ms(8);
	}
		motor[Motor1].ctrl.kp_set = 150.0f;motor[Motor2].ctrl.kp_set = 75.0f;
		motor[Motor7].ctrl.kp_set = 150.0f;motor[Motor8].ctrl.kp_set = 75.0f;delay_us(200);
		motor[Motor3].ctrl.kp_set = 150.0f;motor[Motor4].ctrl.kp_set = 75.0f;delay_us(200);
		motor[Motor9].ctrl.kp_set = 150.0f;motor[MotorA].ctrl.kp_set = 75.0f;delay_us(200);
		motor[Motor5].ctrl.kp_set = 75.0f;motor[Motor6].ctrl.kp_set = 75.0f;delay_us(200);
		motor[MotorB].ctrl.kp_set = 75.0f;motor[MotorC].ctrl.kp_set = 75.0f;delay_us(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
	  send_data(motor);
	  delay_us(200);
	  CDC_Receive_HS(receive_data_buffer, &Len);
	  delay_us(200);
//      uint8_t buffer[sizeof(int16_t)];
//	  memcpy(buffer, &motor[0].para.p_int, sizeof(int16_t));
//      CDC_Transmit_HS(buffer, sizeof(int16_t));
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 24;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
