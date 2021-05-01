/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_VIEW 0
#define SETTING_VIEW 1
#define HOUR_SELECT 1
#define MIN_SELECT 2
#define SEC_SELECT 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define KB1 HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)
#define KB2 HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin)
#define KB3 HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin)
#define KB4 HAL_GPIO_ReadPin(B4_GPIO_Port, B4_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
_Bool LED_State = 1;//led的状态
_Bool Led_Flag = 0;
_Bool Key_Flag = 0;
_Bool Adc_Flag = 0;
_Bool Alarm_Flag = 0;
_Bool rx_Flag = 0;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm;
uint16_t Led_Val;//led的值
uint8_t View_Model = 0;//展示的页面
float VDD = 3.3;
float v1_val;//电压值
float k_val = 0.1;//k的值

uint8_t Setting_Model = 1;//要设置的时间
uint8_t hour, min, sec;
uint8_t ahour, amin, asec;

uint8_t rx_buff[1];
uint8_t rx_data[100];
uint16_t rx_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//控制LED灯的函数
void TurnOff_LEDS()
{
	Led_Val = 0xFF00;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOn_LED(uint8_t i)
{
	Led_Val &= ~(0x01<<(7+i));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOff_LED(uint8_t i)
{
	Led_Val |= (0x01<<(7+i));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
_Bool ledodd = 1;
void LD_Blink()
{
	if(ledodd)
	{
		TurnOn_LED(1);
	}else{
		TurnOff_LED(1);
	}
	ledodd = !ledodd;
}
//获取ADC
uint32_t getADC(void)
{
	uint32_t adc = 0;
	
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	
	return adc;
}
void Display()
{
	uint8_t buf[30];
	if(Adc_Flag)
	{
		Adc_Flag = 0;
		v1_val = getADC() / 4096.0f *3.3f;
	}
	sprintf((char *)buf, "   V1: %.2f", v1_val);
	LCD_DisplayStringLine(Line2, (uint8_t *)buf);
	sprintf((char *)buf, "   k: %.1f", k_val);
	LCD_DisplayStringLine(Line3, (uint8_t *)buf);
	if(LED_State)
	{
		LCD_DisplayStringLine(Line4, (uint8_t *)"   LED: ON ");
	}else{
		LCD_DisplayStringLine(Line4, (uint8_t *)"   LED: OFF");
	}
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	hour = sTime.Hours;
	min = sTime.Minutes;
	sec = sTime.Seconds;
	sprintf((char *)buf, "   T:%02d-%02d-%02d", hour, min, sec);
	LCD_DisplayStringLine(Line5, (uint8_t *)buf);
	LCD_DisplayStringLine(Line8, (uint8_t *)"                 1");
}

void Setting()
{
	LCD_SetTextColor(Black);
	LCD_DisplayStringLine(Line2, (uint8_t *)"       SETTING   ");
	if(Setting_Model == HOUR_SELECT)
	{
		LCD_SetTextColor(Red);
		LCD_DisplayChar(Line4, 319 -16 *6, ahour/10+48);
		LCD_DisplayChar(Line4, 319 -16 *7, ahour%10+48);
		LCD_SetTextColor(Black);		
	}else{
		LCD_SetTextColor(Black);
		LCD_DisplayChar(Line4, 319 -16 *6, ahour/10+48);
		LCD_DisplayChar(Line4, 319 -16 *7, ahour%10+48);
	}
	
	if(Setting_Model == MIN_SELECT)
	{
		LCD_SetTextColor(Red);
		LCD_DisplayChar(Line4, 319 -16 *9, amin/10+48);
		LCD_DisplayChar(Line4, 319 -16 *10, amin%10+48);
		LCD_SetTextColor(Black);		
	}else{
		LCD_SetTextColor(Black);
		LCD_DisplayChar(Line4, 319 -16 *9, amin/10+48);
		LCD_DisplayChar(Line4, 319 -16 *10, amin%10+48);
	}
	
	if(Setting_Model == SEC_SELECT)
	{
		LCD_SetTextColor(Red);
		LCD_DisplayChar(Line4, 319 -16 *12, asec/10+48);
		LCD_DisplayChar(Line4, 319 -16 *13, asec%10+48);
		LCD_SetTextColor(Black);		
	}else{
		LCD_SetTextColor(Black);
		LCD_DisplayChar(Line4, 319 -16 *12, asec/10+48);
		LCD_DisplayChar(Line4, 319 -16 *13, asec%10+48);
	}
	LCD_SetTextColor(Black);
	LCD_DisplayChar(Line4, 319 -16 *8, '-');
	LCD_DisplayChar(Line4, 319 -16 *11, '-');
	LCD_DisplayStringLine(Line8, (uint8_t *)"                 2");
}
void ReadKey()
{
	//按键1
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			LED_State = !LED_State;
		}
	}else if(KB1 == 1)
	{
		b1_sum = 0;
	}
	//按键2
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			switch(View_Model)
			{
				case DISPLAY_VIEW://进入设置界面
					HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A,RTC_FORMAT_BIN);
					ahour = sAlarm.AlarmTime.Hours;
					amin = sAlarm.AlarmTime.Minutes;
					asec = sAlarm.AlarmTime.Seconds;
					LCD_Clear(White);
					View_Model = SETTING_VIEW;
					break;
				case SETTING_VIEW:
					LCD_Clear(White);
					sAlarm.AlarmTime.Hours = ahour;
					sAlarm.AlarmTime.Minutes = amin;
					sAlarm.AlarmTime.Seconds = asec;
					HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
					View_Model = DISPLAY_VIEW;
					break;
			}
		}
	}else if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1 && View_Model == SETTING_VIEW)
		{
			Setting_Model = Setting_Model % 3 + 1;		
		}
	}else if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
				switch(Setting_Model)
				{
					case HOUR_SELECT:
						if(++ahour == 24)
						{
							ahour = 0;
						}
						break;
					case MIN_SELECT:
						if(++amin == 60)
						{
							amin = 0;
						}
						break;
					case SEC_SELECT:
						if(++asec == 60)
						{
							asec = 0;
						}
						break;
				}
		}
	}else if(KB4 == 1)
	{
		b4_sum = 0;
	}
}
void Display_Proc()
{
	switch(View_Model)
	{
		case DISPLAY_VIEW:
			Display();
			break;
		case SETTING_VIEW:
			Setting();
			break;
	}
}
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
	TurnOff_LEDS();
	//开启RTC时钟
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
	//启动定时器4中断
	HAL_TIM_Base_Start_IT(&htim4);
	//开启串口中断接收
	HAL_UART_Receive_IT(&huart1, rx_buff, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Display_Proc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Key_Flag)
		{
			Key_Flag = 0;
			ReadKey();
		}
		if(Led_Flag && LED_State)
		{
			Led_Flag = 0;
			if(v1_val > (float)(VDD*k_val))
			{
				LD_Blink();
			}
		}else if(!LED_State){
			TurnOff_LED(1);
		}
		if(Alarm_Flag)
		{
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			printf("%.2f+%.1f+%02d%02d%02d\n", v1_val, k_val, sTime.Hours, sTime.Minutes, sTime.Seconds);
			Alarm_Flag = 0;
		}
		if(rx_Flag)
		{
			if(rx_count == 4 && rx_data[0] == 'k')
			{
				if(rx_data[1] == '0' && rx_data[2] == '.' && (rx_data[3] >= '1' && rx_data[3] <= '9'))
				{
					printf("ok\n");
					k_val = (float)(rx_data[3] - 48) * 0.1f;
				}else{
					printf("Error\n");
				}
			}else{
				printf("Error\n");
			}
			rx_count = 0;
			memset(rx_data, 0, sizeof(rx_data));
			rx_Flag = 0;
			
		}
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart->Instance == USART1)
 {
	 rx_data[rx_count] = rx_buff[0];
	 if(rx_data[rx_count] == 0x0A)
	 {
		 printf("%c", rx_data[rx_count]);
		 rx_Flag = 1;
	 }else{
		 rx_count++;
	 }
	 HAL_UART_Receive_IT(&huart1, rx_buff, 1);
 }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start_IT(&htim4);
	static uint8_t led_count;
	static uint8_t key_count;
	static uint16_t adc_count;
	if(htim->Instance == 	TIM4)
	{
		if(++led_count == 200)
		{
			led_count = 0;
			Led_Flag = 1;
		}
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
		if(++adc_count == 500)
		{
			adc_count = 0;
			Adc_Flag = 1;
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	Alarm_Flag = 1;
	HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
