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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAYVIEW 0
#define SETTINGVIEW 1
#define UPPER 0
#define LOEWER 1
#define NORMAL 2
#define MAXVOLTSE 1
#define MINVOLTSE 2
#define UPLDSE 3
#define LOLDSE 4
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
uint8_t ViewModel = 0;
_Bool Key_Flag = 0;//按键标志位
_Bool Led_Flag = 0;//LED标志位
_Bool Adc_Flag = 0;//ADC标志位
//显示界面
float Volt_Val;//检测的电压值
uint8_t Status_Val;//状态
//设置界面
uint8_t Setting_Index;

float Max_Volt_Val = 2.4;//电压值上限
float Min_Volt_Val = 1.2;//电压值下限
uint8_t Max_Led = 1;
uint8_t Min_Led = 2;

uint16_t Led_Val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//操作LED
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
_Bool ledodd;
void Led_blink(uint8_t i)
{
	if(ledodd)
	{
		TurnOn_LED(i);
	}else{
		TurnOff_LED(i);
	}
	ledodd = !ledodd;
}
//读取ADC
float getADC()
{
	HAL_ADC_Start(&hadc2);
	return HAL_ADC_GetValue(&hadc2);
}	
void Display()
{
	uint8_t buf[30];
	LCD_DisplayStringLine(Line1, (uint8_t *)"        Main   ");
	sprintf((char *)buf, "    Volt: %.2fV", Volt_Val);
	LCD_DisplayStringLine(Line3, (uint8_t *)buf);
	switch(Status_Val)
	{
		case UPPER:
			LCD_DisplayStringLine(Line4, (uint8_t *)"    Status: Upper ");
			break;
		case LOEWER:
			LCD_DisplayStringLine(Line4, (uint8_t *)"    Status: Lower ");
			break;
		case NORMAL:
			LCD_DisplayStringLine(Line4, (uint8_t *)"    Status: Normal");
			break;
	}
}
void Setting()
{
	uint8_t buf[30];
	LCD_SetTextColor(Blue2);
	LCD_DisplayStringLine(Line1, (uint8_t *)"        Setting   ");
	
	sprintf((char *)buf, "   Max Volt: %.2fV", Max_Volt_Val);
	if(Setting_Index == MAXVOLTSE){
		LCD_SetBackColor(Green);
		LCD_DisplayStringLine(Line2, (uint8_t *)buf);
		LCD_SetBackColor(White);
	}else{
		LCD_DisplayStringLine(Line2, (uint8_t *)buf);
	}
	
	sprintf((char *)buf, "   Min Volt: %.2fV", Min_Volt_Val);
	if(Setting_Index == MINVOLTSE){
		LCD_SetBackColor(Green);
		LCD_DisplayStringLine(Line3, (uint8_t *)buf);
		LCD_SetBackColor(White);
	}else{
		LCD_DisplayStringLine(Line3, (uint8_t *)buf);
	}

	sprintf((char *)buf, "   Upper: LD%d", Max_Led);
	if(Setting_Index == UPLDSE){
		LCD_SetBackColor(Green);
		LCD_DisplayStringLine(Line4, (uint8_t *)buf);
		LCD_SetBackColor(White);
	}else{
		LCD_DisplayStringLine(Line4, (uint8_t *)buf);
	}
	
	sprintf((char *)buf, "   Lower: LD%d", Min_Led);
	if(Setting_Index == LOLDSE){
		LCD_SetBackColor(Green);
		LCD_DisplayStringLine(Line5, (uint8_t *)buf);
		LCD_SetBackColor(White);
	}else{
		LCD_DisplayStringLine(Line5, (uint8_t *)buf);
	}
}
void Display_Proc()
{
	switch(ViewModel)
	{
		case DISPLAYVIEW:
			Display();
			break;
		case SETTINGVIEW:
			Setting();
			break;
	}
}
void Key_Proc()
{
	//按键1 用于界面的切换
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			switch(ViewModel)
			{
				case DISPLAYVIEW:
					LCD_Clear(White);
					ViewModel = SETTINGVIEW;
				break;
				case SETTINGVIEW:
					LCD_Clear(White);
					ViewModel = DISPLAYVIEW;
				break;
			}
		}
	}else if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 用于设置选项的切换
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			if(ViewModel == SETTINGVIEW)			
				Setting_Index = Setting_Index % 4 + 1;		
		}
	}else if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 用于选中项的加
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			if(ViewModel == SETTINGVIEW)
			{
				switch(Setting_Index)
				{
					case MAXVOLTSE :
						{
							if((Max_Volt_Val > Min_Volt_Val) && (Max_Volt_Val + 0.3f <= 3.3f))
							{
								Max_Volt_Val += 0.3f;
							}
						}
						break;
					case MINVOLTSE :
						if((Min_Volt_Val +  0.4f < Max_Volt_Val) && (Min_Volt_Val >= 0.0f))
							{
								Min_Volt_Val += 0.3f;
							}
						break;
					case UPLDSE :
						if(Max_Led < 8)
						{
							TurnOff_LEDS();
							Max_Led++;
							if(Max_Led == Min_Led && Max_Led != 8)
								Max_Led++;
							else if(Max_Led == Min_Led && Max_Led == 8)   
								Max_Led--;
						}
						break;
					case LOLDSE :
						if(Min_Led < 8)
						{
							TurnOff_LEDS();
							Min_Led++;
							if(Max_Led == Min_Led && Min_Led != 8)
								Min_Led++;
							else if(Max_Led == Min_Led && Min_Led == 8)   
								Min_Led--;
						}
						break;
				}
			}
		}
	}else if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4 用于选中项的减
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			if(ViewModel == SETTINGVIEW)
			{
				switch(Setting_Index)
				{
					case MAXVOLTSE :
						{
							if(Max_Volt_Val - 0.4f > Min_Volt_Val)
							{
								Max_Volt_Val -= 0.3f;
							}
						}
						break;
					case MINVOLTSE :
						if(Min_Volt_Val - 0.3f >= 0.0f)
							{
								Min_Volt_Val -= 0.3f;
							}
						break;
					case UPLDSE :
						if(Max_Led > 1)
						{
							TurnOff_LEDS();
							Max_Led--;
							if(Max_Led == Min_Led && Max_Led != 1)
								Max_Led--;
							else if(Max_Led == Min_Led && Max_Led == 1)   
								Max_Led++;
						}
						break;
					case LOLDSE :
						if(Min_Led > 1)
						{
							TurnOff_LEDS();
							Min_Led--;
							if(Max_Led == Min_Led && Min_Led != 1)
								Min_Led--;
							else if(Max_Led == Min_Led && Min_Led == 1)   
								Min_Led++;
						}
						break;
				}
			}
		}
	}else if(KB4 == 1)
	{
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
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue2);
	//关闭LED
	TurnOff_LEDS();
  /* USER CODE END 2 */
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_TIM_Base_Start_IT(&htim4);
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
		if(Led_Flag)
		{
			Led_Flag = 0;
			if(Volt_Val > Max_Volt_Val)
			{
				Led_blink(Max_Led);
				Status_Val = UPPER;
			}else if(Volt_Val < Min_Volt_Val)
			{
				Led_blink(Min_Led);
				Status_Val = LOEWER;
			}else{
				TurnOff_LEDS();
				Status_Val = NORMAL;
			}
		}
		if(Adc_Flag)
		{
			Adc_Flag = 0;
			Volt_Val = getADC() / 4095.0f * 3.3f;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start_IT(&htim4);
	static uint8_t key_count;
	static uint8_t led_count;
	static uint16_t adc_count;
	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
		if(++led_count == 200)
		{
			led_count = 0;
			Led_Flag = 1;
		}
		if(++adc_count == 500)
		{
			adc_count = 0;
			Adc_Flag = 1;
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
