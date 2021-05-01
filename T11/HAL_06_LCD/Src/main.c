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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//界面
#define DISPLAYVIEW 1
#define SETTINGVIEW 2

//模式
#define AUTOMODE 1
#define MANUMODE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define KB1 HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)
#define KB2 HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin)
#define KB3 HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin)
#define KB4 HAL_GPIO_ReadPin(B4_GPIO_Port, B4_Pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
_Bool Key_Flag = 0;//按键的标志位
_Bool Adc_Flag = 0;//ADC的标志位

uint32_t Led_Val;
uint8_t View_Model = DISPLAYVIEW;
float V_Val = 0.0;//测得的电压值
uint32_t Adc_buf[10];//存放ADC采集的值
uint8_t Mode_Val = AUTOMODE;

uint8_t PA6_Val = 10;
uint8_t PA7_Val = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TurnOff_LEDS()
{
	Led_Val = 0xFF00;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}

void TurnOn_LED(uint8_t _led)
{
	Led_Val &= ~(0x01 << ( 7 + _led));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}

void TurnOff_LED(uint8_t _led)
{
	Led_Val |= (0x01 << ( 7 + _led));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}

void Display_Proc()
{
	uint8_t buf[30];
	
	switch(View_Model)
	{
		case DISPLAYVIEW:
		{	
			sprintf((char *)buf, "      Data");
			LCD_DisplayStringLine(Line0, buf);
			sprintf((char *)buf, "    V:%.2fV", V_Val);
			LCD_DisplayStringLine(Line2, buf);
			if(Mode_Val == AUTOMODE)
			{
				uint8_t mode_buf[] = "AUTO";
				sprintf((char *)buf, "    Mode:%s", mode_buf);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)V_Val/3.3f*100);
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (uint32_t)V_Val/3.3f*100);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
			}else{
				uint8_t mode_buf[] = "MANU";
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)PA6_Val);
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (uint32_t)PA7_Val);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
				sprintf((char *)buf, "    Mode:%s", mode_buf);
			}
			LCD_DisplayStringLine(Line4, buf);
		}
			break;
		case SETTINGVIEW:
		{
			sprintf((char *)buf, "      Para");
			LCD_DisplayStringLine(Line0, buf);
			sprintf((char *)buf, "    PA6:%d%%", PA6_Val);
			LCD_DisplayStringLine(Line2, buf);
			sprintf((char *)buf, "    PA7:%d%%", PA7_Val);
			LCD_DisplayStringLine(Line4, buf);
		}
			break;
	}
}
//ADC均值滤波
float Adc_Proc(uint32_t _buf[])
{
	uint8_t i, j;
	float Adc_Val = 0;
	float temp;
	for(i = 0; i < 10 - 1; i++)
	{
		for(j = 0; j < 10 - 1 - i; j++)
		{
			if (_buf[j] < _buf[j+1])
			{
					temp = _buf[j];
					_buf[j] = _buf[j+1];
					_buf[j+1] = temp;
			}
		}
	}
	
	for(i = 1; i < 10-1; i++) 
    	Adc_Val += _buf[i]; 
	return (Adc_Val / 8) / 4096 * 3.3f;
}

void Key_Proc()
{
	//按键1 用于界面切换
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			if(View_Model == DISPLAYVIEW)
			{				
				View_Model = SETTINGVIEW;
				TurnOff_LED(2);
				LCD_Clear(Black);
			}else if(View_Model == SETTINGVIEW){
				View_Model = DISPLAYVIEW;
				TurnOn_LED(2);
				LCD_Clear(Black);
			}			
		}
	}
	if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 用于PA6的占空比增加
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1 && View_Model == SETTINGVIEW)
		{
			PA6_Val += 10;
			if(PA6_Val > 90)
				PA6_Val = 10;
		}
	}
	if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 用于PA7的占空比增加
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1 && View_Model == SETTINGVIEW)
		{
			PA7_Val += 10;
			if(PA7_Val > 90)
				PA7_Val = 10;
		}
	}
	if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4 用于模式切换
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			if(Mode_Val == AUTOMODE)
			{				
				Mode_Val = MANUMODE;
				TurnOff_LED(1);
			}else if(Mode_Val == MANUMODE){
				Mode_Val = AUTOMODE;
				TurnOn_LED(1);
			}			
		}
	}
	if(KB4 == 1)
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	//校准ADC
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	//开启ADC的DMA方式采集
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&Adc_buf,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	TurnOff_LEDS();
	TurnOn_LED(1);
	TurnOn_LED(2);
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
		if(Adc_Flag)
		{
			Adc_Flag = 0;
			HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&Adc_buf,10);
		}
		
		if(V_Val >= 3.29f || V_Val == 0.0f)
		{			
			if(V_Val == 0.0f)
			{
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
			}else{
				//占空比拉满设置成高电平
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 100);
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 99;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B4_Pin */
  GPIO_InitStruct.Pin = B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_CLK_Pin */
  GPIO_InitStruct.Pin = LD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC2)
  {
		V_Val = Adc_Proc(Adc_buf);
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
