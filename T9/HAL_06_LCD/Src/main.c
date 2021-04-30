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
#include <string.h>
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//界面
#define DISPLAYVIEW 1
#define SETTINGVIEW 2
//设置选择项
#define HOR_SEL 1
#define MIN_SEL 2
#define SEC_SEL 3
//当前状态
#define STANDBY 0
#define SETTING 1
#define RUNNING 2
#define PAUSE   3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define KB1 HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)
#define KB2 HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin)
#define KB3 HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin)
#define KB4 HAL_GPIO_ReadPin(B4_GPIO_Port, B4_Pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
_Bool Key_Flag = 0;//按键的标志位
_Bool Led_Flag = 0;//LED的标志位
_Bool Lcd_Flag = 0;
_Bool T1s_Flag = 0;
uint16_t Led_Val;
uint8_t View_Model = 1;

uint8_t Current_Index = 1;//当前存储序列
int Hour_Val[6], Min_Val[6], Sec_Val[6];//存储时间
uint8_t State_Val[6];//储存序列状态
uint8_t Setting_Index = 1;//当前设置序列
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//读写eeprom
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
//操作LED灯
void TurnOff_LEDS()
{
	Led_Val = 0xFF00;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOff_LED(uint8_t _led)
{
	Led_Val |= (0x01 << (7 + _led));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOn_LED(uint8_t _led)
{
	Led_Val &= ~(0x01 << (7 + _led));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}

void Display_Time(int _fci, int _sci, int _time[])
{
	LCD_DisplayChar(Line3, 319 - 16 * _fci, (uint8_t)_time[Current_Index] / 10 + '0');
	LCD_DisplayChar(Line3, 319 - 16 * _sci, (uint8_t)_time[Current_Index] % 10 + '0');
}

void Add_Time()
{
	if(View_Model == SETTINGVIEW)
	{
		switch(Setting_Index)
		{
			case HOR_SEL:
				if(++Hour_Val[Current_Index] >= 24)
				{
					Hour_Val[Current_Index] = 0;
				}
			break;
			case MIN_SEL:
				if(++Min_Val[Current_Index] >= 60)
				{
					Min_Val[Current_Index] = 0;
				}
			break;
			case SEC_SEL:
				if(++Sec_Val[Current_Index] >= 60)
				{
					Sec_Val[Current_Index] = 0;
				}
			break;
		}
	}
}

void Display_Proc()
{
	uint8_t buf[30];
	//显示序号
	sprintf((char *)buf, "    No %d", Current_Index);
	LCD_DisplayStringLine(Line1, buf);
	
	//显示时间
	if(View_Model == DISPLAYVIEW)
	{
		//显示界面下
		sprintf((char *)buf, "       %02d: %02d: %02d", Hour_Val[Current_Index], Min_Val[Current_Index], Sec_Val[Current_Index]);
		LCD_DisplayStringLine(Line3, buf);
	}
	if(View_Model == SETTINGVIEW)
	{
		//设置界面下
		if(Setting_Index == HOR_SEL)
		{
			LCD_SetTextColor(Red);
			Display_Time(7, 8, Hour_Val);
			LCD_SetTextColor(Black);
		}else{
			Display_Time(7, 8, Hour_Val);
		}
		if(Setting_Index == MIN_SEL)
		{
			LCD_SetTextColor(Red);
			Display_Time(11, 12, Min_Val);
			LCD_SetTextColor(Black);
		}else{
			Display_Time(11, 12, Min_Val);
		}
		if(Setting_Index == SEC_SEL)
		{
			LCD_SetTextColor(Red);
			Display_Time(15, 16, Sec_Val);
			LCD_SetTextColor(Black);
		}else{
			Display_Time(15, 16, Sec_Val);
		}
	}
	//显示状态值
	switch(State_Val[Current_Index])
	{
		case STANDBY:
			LCD_DisplayStringLine(Line5, (uint8_t *)"        Standby  ");
		break;
		case SETTING:
			LCD_DisplayStringLine(Line5, (uint8_t *)"        Setting  ");
		break;
		case RUNNING:
			LCD_DisplayStringLine(Line5, (uint8_t *)"        Running  ");
		break;
		case PAUSE:
			LCD_DisplayStringLine(Line5, (uint8_t *)"        Pause    ");
		break;
	}
}
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
//按键扫描服务函数
void Key_Proc()
{
	//按键1 用于界面切换
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			Current_Index = Current_Index % 5 + 1;
		}
	}
	if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 用于设置切换
	static uint8_t b2_sum;
	static uint16_t first_time;
	if(KB2 == 0)
	{
		b2_sum++;
	}
	if(KB2 == 1)
	{
		if(b2_sum >= 1 && b2_sum <= 80)
		{
			if(View_Model == DISPLAYVIEW && State_Val[Current_Index] == STANDBY)
			{
				View_Model = SETTINGVIEW;	
				for(int i = 1; i <= 5; i++){
					State_Val[i] = SETTING;
				}
			}else if(View_Model == SETTINGVIEW){
				Setting_Index = Setting_Index % 3 + 1;
				
			}
		}else if(b2_sum > 80){
			View_Model = DISPLAYVIEW;
			ee_write(0x01, Hour_Val[1]);
			HAL_Delay(5);
			ee_write(0x02, Min_Val[1]);
			HAL_Delay(5);
			ee_write(0x03, Sec_Val[1]);
			HAL_Delay(5);
			for(int i = 1; i <= 5; i++){
					State_Val[i] = STANDBY;
			}
		}
		b2_sum  = 0;
	}
	
	//按键3 用于数字的快速增加
	static uint8_t b3_sum;

	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			Add_Time();
		}
		if(b3_sum >= 80)
		{
			Add_Time();
			b3_sum = 75;
		}
	}
	if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4 用于定时器启动
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;		
	}
	if(KB4 == 1)
	{
		if(b4_sum >= 1 && b4_sum <= 80)
		{
			if(State_Val[Current_Index] == STANDBY || State_Val[Current_Index] == SETTING)
			{
				for(int i = 1; i <= 5; i++){
					State_Val[i] = STANDBY;
				}
				View_Model = DISPLAYVIEW;
				State_Val[Current_Index] = RUNNING;//启动定时器
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			}
			else if(State_Val[Current_Index] == RUNNING)
			{
				State_Val[Current_Index] = PAUSE;//暂停定时器
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				TurnOff_LED(1);
			}
			else if(State_Val[Current_Index] == PAUSE)
			{
				State_Val[Current_Index] = RUNNING;//恢复定时器
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			}
		}
		if(b4_sum > 80)
		{
			if(State_Val[Current_Index] == RUNNING)
			{
				State_Val[Current_Index] = STANDBY;//暂停定时器
			}
		}
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//初始化I2C
	I2CInit();

	Hour_Val[1] = ee_read(1);
	HAL_Delay(5);
	Min_Val[1] = ee_read(2);
	HAL_Delay(5);
	Sec_Val[1] = ee_read(3);
	//启动定时器4
	HAL_TIM_Base_Start_IT(&htim4);
	//初始化LCD
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	TurnOff_LEDS();	
  while (1)
  {
		if(Lcd_Flag){
			Lcd_Flag = 0;
			Display_Proc();//占用20ms
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Key_Flag)
		{
			Key_Flag = 0;
			Key_Proc();//占用20ms
		}
		if(State_Val[Current_Index] == RUNNING)
		{
			if(Led_Flag)
				TurnOn_LED(1);
			else
				TurnOff_LED(1);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 800;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start_IT(&htim4);
	static uint8_t key_count;//用于按键扫描计数
	static uint16_t led_count;//用于LED切换
	static uint16_t timer_count;//倒计时
	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
		if(++led_count == 500)
		{
			led_count = 0;
			Led_Flag = !Led_Flag;
		}
		if(++timer_count == 1000)
		{
			timer_count = 0;
			if(State_Val[Current_Index] == RUNNING)
			{			
				if(Sec_Val[Current_Index] == 0)
				{
					Sec_Val[Current_Index] = 60;	
					if(Min_Val[Current_Index] == 0)
					{
						Min_Val[Current_Index] = 60;
						if(Hour_Val[Current_Index] == 0)
						{
							Hour_Val[Current_Index] = 1;
							Min_Val[Current_Index] = 1;
							Sec_Val[Current_Index] = 1;	
							State_Val[Current_Index] = STANDBY;	
							TurnOff_LED(1);							
						}
						Hour_Val[Current_Index]--;
					}
					Min_Val[Current_Index]--;
				}
				Sec_Val[Current_Index]--;
			}
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
