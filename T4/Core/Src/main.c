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
#include "string.h"
#include "lcd.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
_Bool Key_Flag = 0;//按键的标志位
_Bool PA1_Flag = 0;
_Bool PA2_Flag = 0;
_Bool rx_Flag = 0;
_Bool rx_er_Flag = 0;
_Bool Command_Flag = 0;//是否有命令的标志位
uint8_t x, y;
uint16_t Led_Val;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
uint8_t PA1_Val;//PA1通道的
uint8_t PA2_Val;//PA2通道的
uint16_t Hour, Min, Sec, SubSec;//时间
uint16_t hh, mm, ss;//开始的时间
uint8_t Current_Channel;//当前输出的通道，为0时即没有输出
uint8_t rx_data[15];
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
void Key_Proc()
{
	//按键1 PA1输出PWM调整
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
	}
	if(KB1 == 1)
	{
		if(b1_sum >= 1)
		{
			PA1_Flag = !PA1_Flag;
			if(PA1_Flag)
			{
				TurnOn_LED(1);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PA1_Val*10);
				
				//防止PA2开启
				TurnOff_LED(2);
				PA2_Flag = 0;
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
			}else if(!PA1_Flag)
			{
				TurnOff_LED(1);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			}
		}
		b1_sum = 0;
	}
	
	//按键2 PA1调整占空比
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			PA1_Val += 10;
			if(PA1_Val == 110)
			{
				PA1_Val = 0;
			}
			ee_write(0x01, PA1_Val);
			if(PA1_Flag)//如果PA1开启的话
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PA1_Val*10);
			}
		}
	}
	if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 PA2输出PWM调整
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
	}
	if(KB3 == 1)
	{
		if(b3_sum >= 1)
		{
			PA2_Flag = !PA2_Flag;
			if(PA2_Flag)
			{
				TurnOn_LED(2);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PA2_Val*10);
				
				//防止PA1开启
				PA1_Flag = 0;
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
				TurnOff_LED(1);
			}else if(!PA2_Flag)
			{
				TurnOff_LED(2);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
			}
		}
		b3_sum = 0;
	}
	
	//按键4 PA2调整占空比
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			PA2_Val += 10;
			ee_write(0x02, PA2_Val);
			if(PA2_Val == 110)
			{
				PA2_Val = 0;
			}
			if(PA2_Flag)//如果PA2开启的话
			{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PA2_Val*10);
			}
		}
	}
	if(KB4 == 1)
	{
		b4_sum = 0;
	}
}
void Display_Porc()
{
	uint8_t buf[30];
	sprintf((char *)buf, "  PWM-PA1: %02d%%  ", PA1_Val);
	LCD_DisplayStringLine(Line1, buf);
	
	sprintf((char *)buf, "  PWM-PA2: %02d%%  ", PA2_Val);
	LCD_DisplayStringLine(Line2, buf);
	
	HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
	Hour = sTime.Hours;
	Min = sTime.Minutes;
	Sec = sTime.Seconds;
	SubSec = sTime.SubSeconds;
	sprintf((char *)buf, "  Time:%02d: %02d: %02d", Hour, Min, Sec);
	LCD_DisplayStringLine(Line3, buf);
	
	if(Current_Channel == 0)
	{
		sprintf((char *)buf, "  Channel:None  ");
	}else{
		sprintf((char *)buf, "  Channel:PA%d  ", Current_Channel);
	}
	LCD_DisplayStringLine(Line4, buf);
	
	LCD_DisplayStringLine(Line5, (uint8_t *)"  Command:");
	
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */	
	//启动定时器
	HAL_TIM_Base_Start_IT(&htim4);
	//开启中断接收
	HAL_UART_Receive_IT(&huart1, rx_data, 15);
	//初始化I2C
	I2CInit();
	if(ee_read(0x20 == 'T') && ee_read(0x21) == '4')
	{
		PA1_Val = ee_read(0x01);
		PA2_Val = ee_read(0x02);
	}else{
		ee_write(0x20, 'T');
		HAL_Delay(5);
		ee_write(0x21, '4');
		HAL_Delay(5);
		ee_write(0x01, 80);
		HAL_Delay(5);
		ee_write(0x02, 10);
		HAL_Delay(5);
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PA1_Val*10);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PA2_Val*10);
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetTextColor(Black);
	LCD_DisplayStringLine(Line6, (uint8_t *)"        None       ");

	TurnOff_LEDS();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Display_Porc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Command_Flag)
		{
			//初次计时	
			if(Hour == hh && Min == mm && Sec == ss)
			{
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
				TurnOff_LED(1);
				TurnOff_LED(2);
				Current_Channel = 0;
				PA1_Flag = 0;
				PA2_Flag = 0;
				if(x == 1)
				{
					PA1_Flag = 1;
					TurnOn_LED(1);
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
				}
				if(x == 2)
				{
					PA2_Flag = 1;
					TurnOn_LED(2);
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);				
				}
			}
			
			//到达计时
			if(Hour == hh && Min == mm && Sec == ss + y)
			{
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
				TurnOff_LED(1);
				TurnOff_LED(2);
				Current_Channel = 0;
				PA1_Flag = 0;
				PA2_Flag = 0;
				Command_Flag = 0;
			}
		}
		
		
		if(rx_er_Flag)
		{
			rx_er_Flag = 0;
			memset(rx_data, 0, sizeof(rx_data));
			LCD_DisplayStringLine(Line6, (uint8_t *)"        Error       ");
		}
		if(rx_Flag)
		{
			rx_Flag = 0;
			if(rx_data[11]== '1' || rx_data[11] == '2')
			{
				if(rx_data[13] - '0' > 0 && rx_data[13] - '0' < 10)
				{
					x = rx_data[11] - '0';
					y = rx_data[13] - '0';
					hh = (rx_data[0] - '0') * 10 + (rx_data[1] - '0');
					mm = (rx_data[3] - '0') * 10 + (rx_data[4] - '0');
					ss = (rx_data[6] - '0') * 10 + (rx_data[7] - '0');
					Command_Flag = 1;		
				
					uint8_t buf[30];
					sprintf((char *)buf, "   %s      ", rx_data);
					LCD_DisplayStringLine(Line6, buf);
				}else{
					rx_er_Flag = 1;
				}
			}else{
				rx_er_Flag = 1;
			}
		}
		
		if(PA1_Flag)
		{
			Current_Channel = 1;	
		}
		if(PA2_Flag)
		{
			Current_Channel = 2;
		}
		if(!PA1_Flag && !PA2_Flag)
		{
			Current_Channel = 0;
		}
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
	HAL_TIM_Base_Start_IT(&htim4);
	if(htim->Instance == TIM4)
	{
		static uint8_t key_count;
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(rx_data[2] == ':' && rx_data[5] == ':' && rx_data[8] == '-' && rx_data[12] == '-')
		{
			rx_Flag = 1;
		}else{
			rx_er_Flag = 1;
			
		}
		HAL_UART_Receive_IT(&huart1, rx_data, 15);
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
