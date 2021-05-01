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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_VIEW 0
#define SETTING_VIEW 1
#define ALARM_SETTING_VIEW 2//用于选择界面
#define HOUR_SELECT 1
#define MIN_SELECT 2
#define SEC_SELECT 3//用于选择时间的设置
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
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm;
uint8_t ViewModel = 0;
uint8_t SettingModel = 1;
char lcd_buf[20];
uint16_t Led_Val;
_Bool Key_Flag = 1;
//储存时间
uint8_t hour, min, sec;
uint8_t ahour, amin, asec;
uint8_t setting_count, alarm_setting_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t ee_read(uint8_t address)
{
 uint8_t val;
 I2CStart();
 I2CSendByte(0xa0);
 I2CWaitAck();
 
 I2CSendByte(address);
 I2CWaitAck();
 
 I2CStart();
 I2CSendByte(0xa1);
 I2CWaitAck();
 val = I2CReceiveByte();
 I2CWaitAck();
 I2CStop();
 return val;
}

//
void ee_write(uint8_t address, uint8_t val)
{
 I2CStart();
 I2CSendByte(0xa0);
 I2CWaitAck();
 
 I2CSendByte(address);
 I2CWaitAck();
 
 I2CSendByte(val);
 I2CWaitAck();
 I2CStop();
}

int fputc(int ch, FILE* f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
//对LED灯的操作
void TurnOff_LEDS()
{
	Led_Val |= 0xFF00;
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

_Bool ledflag = 1;
void LD_State1()
{
	if(ledflag)
	{
		TurnOn_LED(1);
	}else{
		TurnOff_LED(1);
	}
	ledflag = !ledflag;
}

void LD_State2()
{
	TurnOff_LEDS();
	TurnOn_LED(2);
}
void LD_State3()
{
	TurnOff_LEDS();
	TurnOn_LED(3);
}

void Display_Time()
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	LCD_DisplayStringLine(Line1, (uint8_t *)"        MAIN           ");
	hour = sTime.Hours;
	min = sTime.Minutes;
	sec = sTime.Seconds;
	sprintf(lcd_buf, "   RTC: %02d:%02d:%02d   ", hour, min, sec);
	LCD_DisplayStringLine(Line3, (uint8_t *)lcd_buf);
}

void Setting_Time()
{
	LCD_DisplayStringLine(Line1, (uint8_t *)"    RTC-SETTING        ");
	switch(SettingModel)
	{
		case HOUR_SELECT:
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3, 319-16*8, hour/10+48);
			LCD_DisplayChar(Line3, 319-16*9, hour%10+48);
			LCD_SetTextColor(Black);
			LCD_DisplayChar(Line3, 319-16*11, min/10+48);
			LCD_DisplayChar(Line3, 319-16*12, min%10+48);
			LCD_DisplayChar(Line3, 319-16*14, sec/10+48);
			LCD_DisplayChar(Line3, 319-16*15, sec%10+48);
			break;
		case MIN_SELECT:
			LCD_SetTextColor(Black);
			LCD_DisplayChar(Line3, 319-16*8, hour/10+48);
			LCD_DisplayChar(Line3, 319-16*9, hour%10+48);
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3, 319-16*11, min/10+48);
			LCD_DisplayChar(Line3, 319-16*12, min%10+48);
			LCD_SetTextColor(Black);
			LCD_DisplayChar(Line3, 319-16*14, sec/10+48);
			LCD_DisplayChar(Line3, 319-16*15, sec%10+48);
			break;
		case SEC_SELECT:
			LCD_SetTextColor(Black);
			LCD_DisplayChar(Line3, 319-16*8, hour/10+48);
			LCD_DisplayChar(Line3, 319-16*9, hour%10+48);
			LCD_DisplayChar(Line3, 319-16*11, min/10+48);
			LCD_DisplayChar(Line3, 319-16*12, min%10+48);
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3, 319-16*14, sec/10+48);
			LCD_DisplayChar(Line3, 319-16*15, sec%10+48);
			LCD_SetTextColor(Black);
			break;
	}
}

void Alarm_Time()
{
	LCD_DisplayStringLine(Line1, (uint8_t *)"    ALARM-SETTING        ");
	sprintf(lcd_buf, " Alarm");
	LCD_DisplayStringLine(Line3, (uint8_t *)lcd_buf);
	if(SettingModel == HOUR_SELECT)
	{
		LCD_SetTextColor(Red);
		LCD_DisplayChar(Line3, 319-16*8, ahour/10+48);
		LCD_DisplayChar(Line3, 319-16*9, ahour%10+48);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayChar(Line3, 319-16*8, ahour/10+48);
		LCD_DisplayChar(Line3, 319-16*9, ahour%10+48);
	}
	if(SettingModel == MIN_SELECT)
	{
		LCD_SetTextColor(Red);
		LCD_DisplayChar(Line3, 319-16*11, amin/10+48);
		LCD_DisplayChar(Line3, 319-16*12, amin%10+48);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayChar(Line3, 319-16*11, amin/10+48);
		LCD_DisplayChar(Line3, 319-16*12, amin%10+48);
	}
	if(SettingModel == SEC_SELECT)
	{
		LCD_SetTextColor(Red); 
		LCD_DisplayChar(Line3, 319-16*14, asec/10+48);
		LCD_DisplayChar(Line3, 319-16*15, asec%10+48);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayChar(Line3, 319-16*14, asec/10+48);
		LCD_DisplayChar(Line3, 319-16*15, asec%10+48);
	}
}
void Display_Proc()
{
	switch(ViewModel)
	{
		case MAIN_VIEW:
			Display_Time();
			break;
		case SETTING_VIEW:
			Setting_Time();
			break;
		case ALARM_SETTING_VIEW:
			Alarm_Time();
			break;
	}
}


//按键的操作
void Key_Proc()
{
	//按键1
	static uint16_t b1_sum = 0;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			if(ViewModel == ALARM_SETTING_VIEW)
			{
				SettingModel = SettingModel % 3 + 1;
			}
			switch(ViewModel)
			{
				case MAIN_VIEW:
					__HAL_RCC_RTC_DISABLE();
					LD_State2();
				  ViewModel = SETTING_VIEW;
					
					break;
				case SETTING_VIEW:	
					__HAL_RCC_RTC_ENABLE();
					TurnOff_LED(2);
					sTime.Hours = hour;
					sTime.Minutes = min;
					sTime.Seconds = sec;
					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					printf("New RTC:%02d:%02d:%02d\r\n", hour, min, sec);
					ViewModel = MAIN_VIEW;
					ee_write(0x02, ++setting_count);
					HAL_Delay(5);
					break;
			}
			
		}
	} 
	if(KB1 == 1){
		b1_sum = 0;
	}
	
	//按键2
	static uint16_t b2_sum = 0;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			if(ViewModel == SETTING_VIEW)
			{
				SettingModel = SettingModel % 3 + 1;
			}
			switch(ViewModel)
			{
				case MAIN_VIEW:
					__HAL_RCC_RTC_DISABLE();
					LD_State3();
					HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
					ahour = sAlarm.AlarmTime.Hours;
					amin = sAlarm.AlarmTime.Minutes;
					asec = sAlarm.AlarmTime.Seconds;
				  ViewModel = ALARM_SETTING_VIEW;
					break;
				case ALARM_SETTING_VIEW:			
					alarm_setting_count++;
					
					__HAL_RCC_RTC_ENABLE();
					TurnOff_LED(3);
					sAlarm.AlarmTime.Hours = ahour;
					sAlarm.AlarmTime.Minutes = amin;
					sAlarm.AlarmTime.Seconds = asec;
					HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN);
					
					printf("New Alarm:%02d:%02d:%02d\r\n", ahour, amin, asec);
					ViewModel = MAIN_VIEW;
					ee_write(0x02, ++alarm_setting_count);
					HAL_Delay(5);
					break;
			}
		}
	} 
	if(KB2 == 1){
		b2_sum = 0;
	}
	
	//按键3 用于时间加
	static uint16_t b3_sum = 0;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			if(ViewModel == SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(++hour == 24)
						{
							hour = 0;
						}
					break;
					case MIN_SELECT:
						if(++min == 60)
						{
							min = 0;
						}
					break;
					case SEC_SELECT:
						if(++sec == 60)
						{
							sec = 0;
						}
					break;						
				}
				Setting_Time();
			}
			if(ViewModel == ALARM_SETTING_VIEW)
			{
				switch(SettingModel)
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
				Alarm_Time();
			}
		}
		if(b3_sum == 100)
		{
			if(ViewModel == SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(++hour >= 24)
						{
							hour = 0;
						}
					break;
					case MIN_SELECT:
						if(++min >= 60)
						{
							min = 0;
						}
					break;
					case SEC_SELECT:
						if(++sec >= 60)
						{
							sec = 0;
						}
					break;						
				}
				Setting_Time();
				b3_sum = 95;
			}
			if(ViewModel == ALARM_SETTING_VIEW)
			{
				switch(SettingModel)
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
				Alarm_Time();
				b3_sum = 95;
			}
		}
	}
	if(KB3 == 1){
		b3_sum = 0;
	}

	//按键4 用于时间减
	static uint16_t b4_sum = 0;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			if(ViewModel == SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(hour != 0)
						{
							hour--;
						}
					break;
					case MIN_SELECT:
						if(min != 0)
						{
							min--;
						}
					break;
					case SEC_SELECT:
						if(sec != 0)
						{
							sec--;
						}
					break;						
				}
				Setting_Time();
			}
			if(ViewModel == ALARM_SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(ahour != 0)
						{
							ahour--;
						}
					break;
					case MIN_SELECT:
						if(amin != 0)
						{
							amin--;
						}
					break;
					case SEC_SELECT:
						if(asec != 0)
						{
							asec--;
						}
					break;						
				}
				Alarm_Time();
			}
		}
		if(b4_sum == 100)
		{
			if(ViewModel == SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(hour != 0)
						{
							hour--;
						}
					break;
					case MIN_SELECT:
						if(min != 0)
						{
							min--;
						}
					break;
					case SEC_SELECT:
						if(sec != 0)
						{
							sec--;
						}
					break;						
				}
				Setting_Time();
				b4_sum = 95;
			}
			if(ViewModel == ALARM_SETTING_VIEW)
			{
				switch(SettingModel)
				{
					case HOUR_SELECT:
						if(ahour != 0)
						{
							ahour--;
						}
					break;
					case MIN_SELECT:
						if(amin != 0)
						{
							amin--;
						}
					break;
					case SEC_SELECT:
						if(asec != 0)
						{
							asec--;
						}
					break;						
				}
				Alarm_Time();
				b4_sum = 95;
			}
		}
	}
	if(KB4 == 1){
		b4_sum = 0;
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	alarm_setting_count = ee_read(0x01);
	setting_count = ee_read(0x02);
	
	//关闭全部LED
	TurnOff_LEDS();
	HAL_TIM_Base_Start_IT(&htim4);
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
			Key_Proc();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t key_count = 0;
	static uint16_t led_count = 0;
	HAL_TIM_Base_Start_IT(&htim4);
	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			Key_Flag = 1;
			key_count = 0;
		}
		if(ViewModel == MAIN_VIEW && (++led_count) == 1000)
		{
			led_count = 0;
			LD_State1();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
