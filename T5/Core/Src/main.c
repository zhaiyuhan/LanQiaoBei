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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOCAL_MODEL 0
#define SERIAL_MODEL 1
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
_Bool rx_Flag = 0;//串口的标志位
_Bool rx_er_Flag = 0;//串口接收错误的标志位
_Bool Key_Flag = 0;
uint16_t Led_Val;
uint32_t C1_Val;//Period(s) = Diff捕获差值 / TIMx_CLK定时器时钟 * (PSC预分频系数 + 1)
								//Freq(Hz)  = TIMx_CLK / (Diff * (PSC + 1))
								//CCRx_1 < CCRx_2 捕获差值 = CCRx_2 - CCRx_1
								//CCRx_1 > CCRx_2 捕获差值 = (ARR + 1 - CCRx_1) + CCRx_2 
uint8_t N1_Val = 2;//倍频数，在1到9之间，默认为2
uint32_t C2_Val;
uint8_t N2_Val = 2;
uint8_t Current_Channel = 1;
_Bool Setting_Model = 0;
uint8_t rx_buf[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) 
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch; 
}
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

//控制LED
void TurnOff_LEDS()
{
	Led_Val = 0xFF00;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOn_LED(uint8_t i)
{
	Led_Val &= ~(0x01<<(i+7));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOff_LED(uint8_t i)
{
	Led_Val |= (0x01<<(i+7));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = (uint32_t)Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}


void Display_Proc()
{
	uint8_t buf[30];
	sprintf((char *)buf, "Channel(1): %-dHz        ", C1_Val);
	LCD_DisplayStringLine(Line2, buf);
	
	sprintf((char *)buf, "N(1): %d  ", N1_Val);
	LCD_DisplayStringLine(Line4, buf);
	
	sprintf((char *)buf, "Channel(2): %-dHz        ", C2_Val);
	LCD_DisplayStringLine(Line6, buf);
	
	sprintf((char *)buf, "N(2): %d  ", N2_Val);
	LCD_DisplayStringLine(Line8, buf);
	
	sprintf((char *)buf, "                 %d", Current_Channel);
	LCD_DisplayStringLine(Line9, buf);
}

void Key_Proc()
{
	//按键1 切换设定模式
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			Setting_Model = !Setting_Model;
		}
	}
	if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 切换通道
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1 && Setting_Model == LOCAL_MODEL)
		{
			Current_Channel = Current_Channel % 2 + 1;
		}
	}
	if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 当前通道倍数减1
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			if(Setting_Model == LOCAL_MODEL)
			{
				switch(Current_Channel)
				{
					case 1:
					{
						if(N1_Val > 1)
						{
							N1_Val--;
							ee_write(0x01, N1_Val);
						}
					}
						break;
					case 2:
					{
						if(N2_Val > 1)
						{
							N2_Val--;										
							ee_write(0x02, N2_Val);
						}
					}
						break;
				}
			}
		}
	}
	if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4 当前通道倍数加1
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			if(Setting_Model == LOCAL_MODEL)
			{
				switch(Current_Channel)
				{
					case 1:
					{
						if(N1_Val < 10)
						{
							N1_Val++;
							ee_write(0x01, N1_Val);
						}
					}
						break;
					case 2:
					{
						if(N2_Val < 10)
						{
							N2_Val++;
							ee_write(0x02, N2_Val);
						}
					}
						break;
				}
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	I2CInit();
	if(ee_read(0x20) == 'T' && ee_read(0x21) == '5')
	{
		N1_Val = ee_read(0x01);
		N2_Val = ee_read(0x02);
	}else{
		ee_write(0x20, 'T');
		HAL_Delay(5);
		ee_write(0x21, '5');
		HAL_Delay(5);
		ee_write(0x01, 2);
		HAL_Delay(5);
		ee_write(0x02, 2);
		HAL_Delay(5);
	}
	//启动定时器
	HAL_TIM_Base_Start_IT(&htim4);
	//开启中断接收
	HAL_UART_Receive_IT(&huart1, rx_buf, 8);
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	TurnOff_LEDS();
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Display_Proc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(rx_Flag)
		{
			rx_Flag = 0;
			uint8_t N_Val;
			if(rx_buf[4] == '1' || rx_buf[4] == '2')
			{
				N_Val = (rx_buf[6] - '0') * 10 + (rx_buf[7] - '0');
				if(N_Val >= 2 && N_Val <= 10)
				{
					switch(rx_buf[4])
					{
						case '1':
							Current_Channel = 1;
							N1_Val = N_Val;
							break;
						case '2':
							Current_Channel = 2;
							N2_Val = N_Val;
							break;
					}
				}
			}
		}
		if(rx_er_Flag)
		{
			rx_er_Flag = 0;
			memset(rx_buf, 0, sizeof(rx_buf));
		}
		if(Key_Flag)
		{
			Key_Flag = 0;
			Key_Proc();
		}
		//指示灯1和2
		switch(Current_Channel)
		{
			case 1:
			{
				TurnOn_LED(1);
				TurnOff_LED(2);
				__HAL_TIM_SET_COUNTER(&htim2,0);
				if(C1_Val < 50 || C1_Val > 50000)
				{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				}else{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, HAL_RCC_GetPCLK2Freq()/C1_Val*N1_Val/2);
				}					
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_AUTORELOAD(&htim2, HAL_RCC_GetPCLK2Freq()/C1_Val*N1_Val);
			}
				break;
			case 2:
			{
				TurnOff_LED(1);
				TurnOn_LED(2);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				if(C2_Val < 50 || C2_Val > 50000)
				{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				}else{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, HAL_RCC_GetPCLK2Freq()/C2_Val*N2_Val/2);
				}	
				
				__HAL_TIM_SET_AUTORELOAD(&htim2, HAL_RCC_GetPCLK2Freq()/C2_Val*N2_Val);
			}
				break;
		}
		//指示灯3
		if(Setting_Model == SERIAL_MODEL)
		{
			TurnOn_LED(3);
		}else if(Setting_Model == LOCAL_MODEL)
		{
			TurnOff_LED(3);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1 && Setting_Model == SERIAL_MODEL)
	{
		if(rx_buf[0] == 'S' && rx_buf[1] == 'E' && rx_buf[2] == 'T')
		{
			if(rx_buf[3] == ':' && rx_buf[5] == ':')
			{
				rx_Flag = 1;
			}else{
				//返回错误的标志位
				rx_er_Flag = 1;
			}
		}else{
			//返回错误的标志位
			rx_er_Flag = 1;
		}
	}
	HAL_UART_Receive_IT(&huart1, rx_buf, 8);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t key_count;
	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
	}
	HAL_TIM_Base_Start_IT(&htim4);
}
uint32_t IC2_Value1_1, IC2_Value2_1, uwDiffCapture_1;
uint32_t IC2_Value1_2, IC2_Value2_2, uwDiffCapture_2;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
	static uint8_t CaptureIndex_1;
	static uint8_t CaptureIndex_2;
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(CaptureIndex_1 == 0)
		{
			IC2_Value1_1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			CaptureIndex_1 = 1;
		}
		else if(CaptureIndex_1 == 1)
		{
			IC2_Value2_1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			if (IC2_Value2_1 > IC2_Value1_1)
			{
				uwDiffCapture_1 = (IC2_Value2_1 - IC2_Value1_1);
			}
			else if (IC2_Value2_1 < IC2_Value1_1)
			{
				uwDiffCapture_1 = ((0xffffffff - IC2_Value1_1) + IC2_Value2_1) + 1;
			}
			C1_Val = HAL_RCC_GetPCLK2Freq() / uwDiffCapture_1;
			CaptureIndex_1 = 0;
		}
	}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		if(CaptureIndex_2 == 0)
		{
			IC2_Value1_2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			CaptureIndex_2 = 1;
		}
		else if(CaptureIndex_2 == 1)
		{
			IC2_Value2_2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			if (IC2_Value2_2 > IC2_Value1_2)
			{
				uwDiffCapture_2 = (IC2_Value2_2 - IC2_Value1_2);
			}
			else if (IC2_Value2_2 < IC2_Value1_2)
			{
				uwDiffCapture_2 = ((0xffffffff - IC2_Value1_2) + IC2_Value2_2) + 1;
			}
			C2_Val = HAL_RCC_GetPCLK2Freq() / uwDiffCapture_2;
			CaptureIndex_2 = 0;
		}
	}
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
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
