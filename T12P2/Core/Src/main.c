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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "string.h"
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATAVIEW 0
#define PARAVIEW 1
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
_Bool Key_Flag = 0;//按键标志位
_Bool Adc_Flag = 0, Adc_Con_Flag = 0;//ADC标志位
float Vmax_tmp, Vmin_tmp;
uint16_t Led_Val;//存储LED的值
uint8_t View_Model;
uint32_t ADC_buf[10];
float Adc_Val;
float V_Val;
uint8_t A_Val;

float Vmax_Val = 3.0f;
float Vmin_Val = 1.0f;
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
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOn_LED(uint8_t i)
{
	Led_Val &= ~(0x01<<(i+7));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOff_LED(uint8_t i)
{
	Led_Val |= (0x01<<(i+7));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void getADC()
{
	uint8_t i, j;
	Adc_Val = 0;
	float temp;
	for(i = 0; i < 10 - 1; i++)
	{
		for(j = 0; j < 10 - 1 - i; j++)
		{
			if (ADC_buf[j] < ADC_buf[j+1])
			{
					temp = ADC_buf[j];
					ADC_buf[j] = ADC_buf[j+1];
					ADC_buf[j+1] = temp;
			}
		}
	}
	for(i = 1; i < 10-1; i++) 
    	Adc_Val += ADC_buf[i]; 
	V_Val = (Adc_Val / 8) / 4096 * 3.3f;
}
void Display()
{
	uint8_t buf[30];
	LCD_DisplayStringLine(Line1, (uint8_t *)"      Data");
	sprintf((char *)buf, " V:%.2fV", V_Val);
	LCD_DisplayStringLine(Line4, buf);
	memset(buf, 0, sizeof(buf));
	sprintf((char *)buf, " A:%d ", A_Val);
	LCD_DisplayStringLine(Line5, buf);
}
void Setting()
{
	uint8_t buf[30];
	LCD_DisplayStringLine(Line1, (uint8_t *)"      Para");
	sprintf((char *)buf, " Vmax:%.1fV", Vmax_Val);
	LCD_DisplayStringLine(Line4, buf);
	memset(buf, 0, sizeof(buf));
	sprintf((char *)buf, " Vmin:%.1fV", Vmin_Val);
	LCD_DisplayStringLine(Line5, buf);
}
void Display_Proc()
{
	switch(View_Model)
	{
		case DATAVIEW:
			Display();
			break;
		case PARAVIEW:
			Setting();
			break;
	}
}
float Vmax_Prev, Vmin_Prev;//用于储存先前设置的
void Key_Proc()
{
	//按键1 用于按键切换
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			switch(View_Model)
			{
				case DATAVIEW:
					{
						Vmax_Prev = Vmax_Val;
						Vmin_Prev = Vmin_Val;
						LCD_Clear(Black);
						View_Model = PARAVIEW;
					}
					break;
				case PARAVIEW:
					{
						if(Vmax_Val <= (Vmin_Val + 0.5f))
						{
							Vmax_Val = Vmax_Prev;
							Vmin_Val = Vmin_Prev;
						}else{
							Vmax_Val = Vmax_Val;
							Vmin_Val = Vmin_Val;
						}
						LCD_Clear(Black);
						View_Model = DATAVIEW;
					}
					break;
			}
		}
	}
	else if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 用于Vmax减
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			if(View_Model == PARAVIEW){
				Vmax_Val-=0.1f;
				if(Vmax_Val < 0.0f)
				{
					Vmax_Val = 3.3f;
				}
			}
		}
	}
	else if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 用于Vmin加
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			if(View_Model == PARAVIEW){
				Vmin_Val+=0.1f;
				if(Vmin_Val > 3.3f)
				{
					Vmin_Val = 0.0f;
				}
			}	
		}
	}
	else if(KB3 == 1)
	{
		b3_sum = 0;
	}
}

void Uart_Proc()
{
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//启动定时器
	HAL_TIM_Base_Start_IT(&htim4);
	//启动定时器输出PWM波
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	//启动ADC转换DMA方式
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); 
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC_buf,10);
	//初始化LCD
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	TurnOff_LEDS();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Display_Proc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Adc_Flag)
		{
			HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&ADC_buf,10);
			Adc_Flag = 0;	
		}
		if(Adc_Con_Flag)
		{
			Adc_Con_Flag = 0;
			getADC();
		}
			
		if(Key_Flag)
		{
			Key_Flag = 0;
			Key_Proc();
		}
		
		if(V_Val > Vmax_Val)
		{//电压状态1
			TurnOn_LED(1);
			TurnOff_LED(2);
			TurnOff_LED(3);
			A_Val = 1;
			//50% 100Hz
			__HAL_TIM_SET_COUNTER(&htim2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 5000);
			__HAL_TIM_SET_AUTORELOAD(&htim2, 10000);
		}
		if(V_Val < Vmin_Val)
		{//电压状态2 1KHz
			TurnOff_LED(1);
			TurnOn_LED(2);
			TurnOff_LED(3);
			A_Val = 2;
			
			//80%
			__HAL_TIM_SET_COUNTER(&htim2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 800);
			__HAL_TIM_SET_AUTORELOAD(&htim2, 1000);
		}
		if(V_Val >= Vmin_Val && V_Val < Vmax_Val)
		{//电压状态3 10KHZ
			TurnOff_LED(1);
			TurnOff_LED(2);
			TurnOn_LED(3);
			A_Val = 3;
			
			//20%
			__HAL_TIM_SET_COUNTER(&htim2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 20);
			__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV12;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
	static uint8_t adc_count;
	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
		if(++adc_count == 100)
		{
			adc_count = 0;
			Adc_Flag = 1;
		}
	}
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc2)
  {
		Adc_Con_Flag = 1;
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
