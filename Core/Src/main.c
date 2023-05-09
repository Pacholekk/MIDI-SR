/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "lcd.h"
#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "lsm303c.h"
#include "stm32l476g_discovery_compass.h"
#include "stm32l476g_discovery_gyroscope.h"
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
UART_HandleTypeDef huart2;
uint8_t txBuffer[FRAME_SIZE];
int16_t  accData[3];
int16_t  gyroData[3];
uint8_t  strTmpAcc[100];
uint8_t  strTmpGyro[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
// funkcja do tworzenia ramki danych
void createFrame(uint8_t *buffer, int16_t *accData, int16_t *gyroData) {
  int i = 0;
  uint8_t checksum = 0;

  // ustawiamy pierwszy bajt jako bajt startu
  buffer[i++] = FRAME_START_BYTE;
  checksum ^= FRAME_START_BYTE;

  // dodajemy nagłówek ramki
  buffer[i++] = 0x01; // id urządzenia
  checksum ^= 0x01;
  buffer[i++] = 0x10; // typ danych
  checksum ^= 0x10;
  buffer[i++] = FRAME_DATA_SIZE; // długość danych
  checksum ^= FRAME_DATA_SIZE;

  // dodajemy dane z akcelerometru
  for (int j = 0; j < 3; j++) {
    buffer[i++] = (uint8_t)(accData[j] & 0xFF);
    buffer[i++] = (uint8_t)((accData[j] >> 8) & 0xFF);
    checksum ^= buffer[i-2];
    checksum ^= buffer[i-1];
  }

  // dodajemy dane z żyroskopu
  for (int j = 0; j < 3; j++) {
    buffer[i++] = (uint8_t)(gyroData[j] & 0xFF);
    buffer[i++] = (uint8_t)((gyroData[j] >> 8) & 0xFF);
    checksum ^= buffer[i-2];
    checksum ^= buffer[i-1];
  }

  // dodajemy sumę kontrolną
  buffer[i++] = checksum;

  // dodajemy ostatni bajt jako bajt stopu
  buffer[i++] = FRAME_STOP_BYTE;
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
  // konfiguracja portu szeregowego UART
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart2);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  BSP_COMPASS_Init();
  BSP_GYRO_Init();

 if( BSP_COMPASS_Init() == COMPASS_ERROR)
 {
	 while(1)
	 {
		 HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
		 HAL_Delay(500);
	 }
 }


 if(BSP_GYRO_Init() == GYRO_ERROR)
 {
	 while(1)
	 {
		 HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin);
		 HAL_Delay(500);
	 }
 }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BSP_COMPASS_AccGetXYZ(accData);
	  BSP_GYRO_GetXYZ(gyroData);
	  sprintf(strTmpAcc,"ACC: X:%d Y:%d Z:%d\r\n",accData[0], accData[1], accData[2]);
	  HAL_UART_Transmit(&huart2, (uint8_t*)strTmpAcc, strlen(strTmpAcc), 100);
	 // HAL_Delay(300);
	  //sprintf(strTmpGyro,"GYRO: X:%d Y:%d Z:%d\r\n",gyroData[0], gyroData[1], gyroData[2]);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)strTmpGyro, strlen(strTmpGyro), 100);
	  //HAL_Delay(1000);

	    // tworzenie ramki danych
	   // createFrame(txBuffer, accData, gyroData);

	    // wysyłanie ramki przez UART
	   // HAL_UART_Transmit(&huart2, txBuffer, FRAME_SIZE, 1000);

	    // opóźnienie między wysyłkami
	    HAL_Delay(100);
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
