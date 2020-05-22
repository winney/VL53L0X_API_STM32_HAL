/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *
 *  Created on: 20.05.2020
 *      Author: Stanislav Punegov
 *		 ADUC gmbh
 *      GitHub:  https://github.com/winney/VL53L0X_API_STM32_HAL
 *      Contact: st.punegov@gmail.com
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "vl53l0x_api.h"
#include "math.h"
//#ifndef LAGR_H
//#include "lagr.h"
//#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L0X_SENSORS_NUM 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Message[64];
uint16_t MessageLen;

VL53L0X_Dev_t  						vl53l0x_c[VL53L0X_SENSORS_NUM]; // center module
VL53L0X_DEV    						Dev = NULL; //vl53l0x_c;
uint16_t 							MyPinGPIO[VL53L0X_SENSORS_NUM];
uint16_t							MyPinXSHUT[VL53L0X_SENSORS_NUM];
const uint8_t 						MyI2cAddrList[]={0x54, 0x56, 0x58, 0x60, 0x62, 0x64, 0x66};
const double 						MySensOrient[]={5*M_PI/180,	30*M_PI/180,	40*M_PI/180,	5*M_PI/180};
uint32_t 							DistData[VL53L0X_SENSORS_NUM];
uint8_t 							size;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Configure_VL53L0X(uint8_t num);

void Configure_VL53L0X(uint8_t num)
{
	static uint32_t refSpadCount[VL53L0X_SENSORS_NUM];
	static uint8_t 	isApertureSpads[VL53L0X_SENSORS_NUM];
	static uint8_t 	VhvSettings[VL53L0X_SENSORS_NUM];
	static uint8_t 	PhaseCal[VL53L0X_SENSORS_NUM];

	Dev = &vl53l0x_c[num];
	//
	// VL53L0X init for Single Measurement
	//
	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_DataInit( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev, &(VhvSettings[num]), &(PhaseCal[num]));
	VL53L0X_PerformRefSpadManagement(Dev, &(refSpadCount[num]), &(isApertureSpads[num]));
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	//
	// Enable/Disable Sigma and Signal check
	//
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

//  while (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
//  {
//	  MessageLen = sprintf((char*)Message, "Push the button!!! \n\r");
//	  			CDC_Transmit_FS(Message,MessageLen);
//	  			HAL_Delay(500);
//  }


	MyPinXSHUT[0] = TOF_XSHUT_Pin;
	MyPinXSHUT[1] = TOF_XSHUT1_Pin;
	MyPinXSHUT[2] = TOF_XSHUT2_Pin;
	MyPinXSHUT[3] = TOF_XSHUT3_Pin;

	HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, MyPinXSHUT[0]|MyPinXSHUT[1]|MyPinXSHUT[2]|MyPinXSHUT[3], GPIO_PIN_RESET); // Disable XSHUT

	MessageLen = sprintf((char*)Message, "VL53L0X array test\n\r");
	CDC_Transmit_FS(Message,MessageLen);

	for (uint8_t i = 0; i < VL53L0X_SENSORS_NUM; i++)
	{
		VL53L0X_DeviceInfo_t DeviceInfo;
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;

		HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, MyPinXSHUT[i], GPIO_PIN_SET); // Enable XSHUT
		HAL_Delay(50);

		Dev = &(vl53l0x_c[i]);
		Dev->I2cDevAddr = 0x52;
		Dev->I2cHandle = &hi2c1;
		Status = VL53L0X_GetDeviceInfo(Dev,&DeviceInfo);

		if (DeviceInfo.ProductType != 0)
		{
			MessageLen = sprintf((char*)Message, "Sensor %d found: %s\n\r",i, DeviceInfo.Name);
			CDC_Transmit_FS(Message,MessageLen);
			Status = VL53L0X_SetDeviceAddress(Dev,MyI2cAddrList[i]);
			HAL_Delay(100);
			vl53l0x_c[i].I2cDevAddr = MyI2cAddrList[i];
			Status = VL53L0X_DataInit(&vl53l0x_c[i]);
			Configure_VL53L0X(i);
		}
		else
		{
			MessageLen = sprintf((char*)Message, "Sensor %d not found\n\r", i);
			CDC_Transmit_FS(Message,MessageLen);

		}
	}

//	PointsData[0][0] = 0;
//	PointsData[1][0] = 0.05;
//	size = 1;

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		static uint8_t cnt = 0;
		static VL53L0X_RangingMeasurementData_t 	RangingData[VL53L0X_SENSORS_NUM];


		Dev = &(vl53l0x_c[cnt]);

		VL53L0X_PerformSingleRangingMeasurement(Dev, &(RangingData[cnt]));



		if(RangingData[cnt].RangeStatus == 0)
		{
			DistData[cnt] = RangingData[cnt].RangeMilliMeter;

			MessageLen = sprintf((char*)Message, "Sensor %d distance: %i\n\r", cnt, RangingData[cnt].RangeMilliMeter);
			CDC_Transmit_FS((uint8_t*)Message,MessageLen);
		}

		if (++cnt >=VL53L0X_SENSORS_NUM)
		{
			cnt = 0;
//			PointsData[0][size] = PointsData[0][0];
//			PointsData[1][size] = PointsData[1][0];
//			size++;

			double dy = 0.05 + 0.0005*(cos(MySensOrient[1])*DistData[1] + cos(MySensOrient[2])*DistData[2]);
			double dx = 0.05 + 0.001 *(cos(MySensOrient[0])*DistData[0] + cos(MySensOrient[3])*DistData[3]);

			double S = dy*dx;

			MessageLen = sprintf((char*)Message, "Tube size %f m x %f m\n\r", dx, dy);
			CDC_Transmit_FS((uint8_t*)Message,MessageLen);
			HAL_Delay(500);
			MessageLen = sprintf((char*)Message, "Tube area %f sqr m\n\r", S);
			CDC_Transmit_FS((uint8_t*)Message,MessageLen);

		}

		HAL_Delay(500);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
     tex: printf_uart("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
