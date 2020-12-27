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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "onewire.h"
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "lcd.h"
#include "font.h"
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
float temperature;
char komunikat[20];
char buffer[8];
char buffer2[] = "test";
int test_val = 7;

RTC_TimeTypeDef RtcTime;
RTC_DateTypeDef RtcDate;
uint8_t CompareSeconds;
char RtcPrint[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lcd_reset(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_SET);
}

void lcd_data(const uint8_t* data, int size)
{
	HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)data, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin, GPIO_PIN_SET);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //-----------INICJALIZACJA PARAMETRÓW DS18B20-----------------
  DS18B20_Init(DS18B20_Resolution_12bits);	//rozdzielczość 10 bit (dokładność 0,25 C)





//  lcd_cmd(0x80 | 0x3f); //Ustawienie kontrastu
/*============WYCI�?CIE 2===============================
 * ====================================================

  HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_SET);	//włączenie podświetlenia

  //----------------------------------------------
  //==============================================
  HAL_Delay(1000);		//odczekaj po inicjalizacji czujnika temperatury - KONIECZNE!!!
  DS18B20_ReadAll();	//odczytanie skonwertowanej temperatury do odpowiednich elementów w tablicy czujników
  DS18B20_StartAll();	//rozesłanie do wszystkich podłączonych czujników komendy startu konwersji temperatury
  uint8_t i;
  for (i=0; i < DS18B20_Quantity(); i++)
  {
	  if (DS18B20_GetTemperature(i, &temperature))

//		  DS18B20_Read(0, &temperature);	//nie działa

//		  DS18B20_GetROM(i, ROM_tmp);
//		  memset(komunikat, 0, sizeof(komunikat));
	  sprintf(komunikat, "Temp: %.2f \n\r", temperature);
	  lcd_draw_text(1,2, "Temp: ");

	  sprintf(buffer, "%.2f", temperature);
	  lcd_draw_text(1, 35, (uint8_t *)buffer);

	  lcd_draw_text(1, 68, "C");

	  HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

	  if(RtcTime.Seconds != CompareSeconds)		//sprawia, że czas i data aktualizuje się na wyświetlaczu co sekundę
	  	  {
		  sprintf(RtcPrint, "%02d:%02d:%02d \n\r", RtcTime.Hours, RtcTime.Minutes, RtcTime.Seconds);
		  lcd_draw_text(3, 2, (uint8_t *)RtcPrint);
		  sprintf(RtcPrint, "%02d.%02d.20%02d\n\r", RtcDate.Date, RtcDate.Month, RtcDate.Year);
		  lcd_draw_text(4, 2, (uint8_t *)RtcPrint);
		  HAL_UART_Transmit(&huart2, (uint8_t*) RtcPrint, sizeof(RtcPrint), 1000);
		  CompareSeconds = RtcTime.Seconds;
	  	  }

//		  lcd_draw_text(2, 35, (uint8_t *)buffer2));		//wyświetla "test"
//	  }
	  lcd_copy();
	  HAL_UART_Transmit(&huart2, (uint8_t *)komunikat, sizeof(komunikat), 100);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, sizeof(buffer2), 100);
	  HAL_Delay(10000);
	lcd_deinit();
	HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_RESET);	//wyłączenie podświetlenia
  }


//==============WYCI�?CIE 2=========================
//-----------------------------------------------*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_setup();
	  lcd_clear();
	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	  {
		  HAL_Delay(100);
		  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		  {
			  HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_SET);	//włączenie podświetlenia
			  HAL_Delay(1000);		//odczekaj po inicjalizacji czujnika temperatury - KONIECZNE!!!

			  //----------------------------------------------
			  //==============================================

			  DS18B20_ReadAll();	//odczytanie skonwertowanej temperatury do odpowiednich elementów w tablicy czujników
			  DS18B20_StartAll();	//rozesłanie do wszystkich podłączonych czujników komendy startu konwersji temperatury
			  uint8_t i;
			  for (i=0; i < DS18B20_Quantity(); i++)
			  {
				  if (DS18B20_GetTemperature(i, &temperature))

			//		  DS18B20_Read(0, &temperature);	//nie działa

			//		  DS18B20_GetROM(i, ROM_tmp);
			//		  memset(komunikat, 0, sizeof(komunikat));
				  sprintf(komunikat, "Temp: %.2f ", temperature);
				  lcd_draw_text(1,2, "Temp: ");

				  sprintf(buffer, "%.2f", temperature);
				  lcd_draw_text(1, 35, (uint8_t *)buffer);

				  lcd_draw_text(1, 68, "C");

				  HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
				  HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

				  if(RtcTime.Seconds != CompareSeconds)		//sprawia, że czas i data aktualizuje się na wyświetlaczu co sekundę
					  {
					  sprintf(RtcPrint, "%02d:%02d:%02d ", RtcTime.Hours, RtcTime.Minutes, RtcTime.Seconds);
					  lcd_draw_text(3, 2, (uint8_t *)RtcPrint);
					  HAL_UART_Transmit(&huart2, (uint8_t *) RtcPrint, sizeof(RtcPrint), 100);
					  sprintf(RtcPrint, "%02d.%02d.20%02d", RtcDate.Date, RtcDate.Month, RtcDate.Year);
					  lcd_draw_text(5, 20, (uint8_t *)RtcPrint);
					  HAL_UART_Transmit(&huart2, (uint8_t*) RtcPrint, sizeof(RtcPrint), 1000);
					  CompareSeconds = RtcTime.Seconds;
					  }

			//		  lcd_draw_text(2, 35, (uint8_t *)buffer2));		//wyświetla "test"
			//	  }
				  lcd_copy();
				  HAL_UART_Transmit(&huart2, (uint8_t *)komunikat, sizeof(komunikat), 100);
				  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);
				  HAL_Delay(10000);
				lcd_deinit();
				HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_RESET);	//wyłączenie podświetlenia
			  }
		  }
	  }


/*	-------WYCI�?CIE 1-------------
	  DS18B20_ReadAll();	//odczytanie skonwertowanej temperatury do odpowiednich elementów w tablicy czujników
	  DS18B20_StartAll();	//rozesłanie do wszystkich podłączonych czujników komendy startu konwersji temperatury
	  uint8_t i;
	  for (i=0; i < DS18B20_Quantity(); i++)
      {
		  if (DS18B20_GetTemperature(i, &temperature))
		  {
//		  DS18B20_GetROM(i, ROM_tmp);
//		  memset(komunikat, 0, sizeof(komunikat));
		  sprintf(komunikat, "Temp: %.2f \n\r", temperature);
		  lcd_draw_text(2,2, "Temp: ");

		  sprintf(buffer, "%.2f", temperature);
		  lcd_draw_text(2, 35, (uint8_t *)buffer);

		  lcd_draw_text(2, 68, "C");

//		  lcd_draw_text(2, 35, (uint8_t *)buffer2));		//wyświetla "test"
		  }
		  lcd_copy();
		  HAL_UART_Transmit(&huart2, (uint8_t *)komunikat, sizeof(komunikat), 100);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, sizeof(buffer2), 100);
		  HAL_Delay(1000);

	  }
	--------WYCI�?CIE 1----------------*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
