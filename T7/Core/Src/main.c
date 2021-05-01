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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "i2c.h"
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
#define Th1_SEL 1
#define Th2_SEL 2
#define Th3_SEL 3
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
_Bool Adc_Flag = 1;//ADC的标志位
_Bool Key_Flag = 0;//KEY的标志位
_Bool Led_Flag = 0;//LED的标志位
_Bool Led2_Flag, Led2_200ms_flag = 0;
_Bool Led3_Flag, Led3_200ms_flag = 0;

uint16_t Led_Val;
uint8_t View_Model = 0;
uint8_t Height_Val;//液位高度
float Adc_Val;//ADC的值
uint8_t Level_Val = 1;//液位等级
uint8_t Pre_Level_Val = 1;//液位等级

uint8_t Th_Index = 1;
uint8_t Th1, Th2, Th3;

uint8_t rx_buf[30];
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

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
//控制LED
void TurnOff_LEDS(void)
{
	Led_Val = 0xFF00;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOn_LED(uint8_t i)
{
	Led_Val &= ~(0x01<<(7+i));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}
void TurnOff_LED(uint8_t i)
{
	Led_Val |= (0x01<<(7+i));
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_SET);
	GPIOC->ODR = Led_Val;
	HAL_GPIO_WritePin(LD_CLK_GPIO_Port, LD_CLK_Pin, GPIO_PIN_RESET);
}

_Bool ledodd;
void LD1_Blink()
{
	if(ledodd)
	{
		TurnOn_LED(1);
	}else{
		TurnOff_LED(1);
	}
	ledodd = !ledodd;
}

_Bool ledodd2;
uint8_t led2_count;
void LD2_Blink()
{
	if(ledodd2)
	{
		TurnOn_LED(2);
	}else{
		TurnOff_LED(2);
	}
	ledodd2 = !ledodd2;
}

_Bool ledodd3;
uint8_t led3_count;
void LD3_Blink()
{
	if(ledodd3)
	{
		TurnOn_LED(3);
	}else{
		TurnOff_LED(3);
	}
	ledodd3 = !ledodd3;
}

//获取ADC
uint32_t getADC()
{
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,100);
  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC))
	{
		return HAL_ADC_GetValue(&hadc2);
  }
	return 0;
}
void Display()
{
	LCD_DisplayStringLine(Line1, (uint8_t *)"     Liquid Level     ");
	uint8_t buf[30];
	sprintf((char *)buf, "Height:       %-2dcm", Height_Val);
	LCD_DisplayStringLine(Line3, buf);
	
	sprintf((char *)buf, "ADC:          %.2fV", Adc_Val);
	LCD_DisplayStringLine(Line4, buf);
	
	
	sprintf((char *)buf, "Level:        %d", Level_Val);
	LCD_DisplayStringLine(Line5, buf);
}
void Setting()
{
	LCD_DisplayStringLine(Line1, (uint8_t *)"   Parameter Setup    ");
	uint8_t buf2[30];
	
	sprintf((char *)buf2, " Threshold 1: %-2dcm", Th1);
	if(Th_Index == Th1_SEL)
	{
		LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line3, buf2);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayStringLine(Line3, buf2);
	}
	
	sprintf((char *)buf2, " Threshold 2: %-2dcm", Th2);
	if(Th_Index == Th2_SEL)
	{
		LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line4, buf2);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayStringLine(Line4, buf2);
	}
	
	sprintf((char *)buf2, " Threshold 3: %-2dcm", Th3);
	if(Th_Index == Th3_SEL)
	{
		LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line5, buf2);
		LCD_SetTextColor(Black);
	}else{
		LCD_DisplayStringLine(Line5, buf2);
	}
	
}
void Display_Proc()
{
	switch(View_Model)
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
	//按键1 切换显示界面
	static uint8_t b1_sum;
	if(KB1 == 0)
	{
		b1_sum++;
		if(b1_sum == 1)
		{
			switch(View_Model)
			{
				case DISPLAYVIEW:
					LCD_Clear(White);
					View_Model = SETTINGVIEW;
					break;
				case SETTINGVIEW:
					ee_write(0x01, Th1);
					HAL_Delay(5);
					ee_write(0x02, Th2);
					HAL_Delay(5);
					ee_write(0x03, Th3);
					HAL_Delay(5);
					LCD_Clear(White);
					View_Model = DISPLAYVIEW;
					break;
			}
		}
	}else if(KB1 == 1)
	{
		b1_sum = 0;
	}
	
	//按键2 切换显示界面
	static uint8_t b2_sum;
	if(KB2 == 0)
	{
		b2_sum++;
		if(b2_sum == 1)
		{
			if(View_Model == SETTINGVIEW)
				Th_Index = Th_Index % 3 + 1;
		}
	}else if(KB2 == 1)
	{
		b2_sum = 0;
	}
	
	//按键3 阈值加
	static uint8_t b3_sum;
	if(KB3 == 0)
	{
		b3_sum++;
		if(b3_sum == 1)
		{
			if(View_Model == SETTINGVIEW)
			{
				switch(Th_Index)
				{
					case Th1_SEL:
						if(Th1 + 5 < Th2)
						{
							Th1+=5;
						}
						break;
					case Th2_SEL:
						if(Th2 + 5 < Th3)
						{
							Th2+=5;
						}
						break;
					case Th3_SEL:
						if(Th3 + 5 <= 95)
						{
							Th3+=5;
						}
						break;
				}
			}		
		}
	}else if(KB3 == 1)
	{
		b3_sum = 0;
	}
	
	//按键4 阈值减
	static uint8_t b4_sum;
	if(KB4 == 0)
	{
		b4_sum++;
		if(b4_sum == 1)
		{
			if(View_Model == SETTINGVIEW)
			{
				switch(Th_Index)
				{
					case Th1_SEL:
						if(Th1 - 5 >= 5)
						{
							Th1-=5;
						}
						break;
					case Th2_SEL:
						if(Th2 - 5 > Th1)
						{
							Th2-=5;
						}
						break;
					case Th3_SEL:
						if(Th3 - 5 > Th2)
						{
							Th3-=5;
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

//读取eeprom 初始化数据
void Init_Data()
{
	if(ee_read(0x10) == 'T' && ee_read(0x11) == '7')
	{
		Th1 = ee_read(0x01);
		Th2 = ee_read(0x02);
		Th3 = ee_read(0x03);
	}else{
		ee_write(0x10, 'T');
		HAL_Delay(10);
		ee_write(0x11, '7');
		HAL_Delay(10);
		ee_write(0x01, 30);
		HAL_Delay(10);
		ee_write(0x02, 50);
		HAL_Delay(10);
		ee_write(0x03, 70);
		HAL_Delay(10);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buf, 1);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	//初始化I2C读取数据
	I2CInit();
	Init_Data();
	//初始化LCD
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	//关闭LED
	TurnOff_LEDS();
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
		if(Led_Flag)
		{
			Led_Flag = 0;
			LD1_Blink();
		}
		
		if(Adc_Flag)
		{
			Adc_Flag = 0;
			Adc_Val = getADC() / 4096.0f * 3.3f;
			Height_Val = Adc_Val * (100.0f / 3.3f);
			
			if(Height_Val <= Th1)
			{
				Level_Val = 0;
			}
			else if(Height_Val > Th1 && Height_Val <= Th2)
			{
				Level_Val = 1;
			}
			else if(Height_Val > Th2 && Height_Val <= Th3)
			{
				Level_Val = 2;
			}
			else if(Height_Val > Th3)
			{
				Level_Val = 3;
			}
			
			if(Level_Val != Pre_Level_Val)
			{
				if(Level_Val < Pre_Level_Val)
				{	
					printf("A:H%02d+L%d+D\r\n", Height_Val, Level_Val);
				}else if(Level_Val > Pre_Level_Val){
					printf("A:H%02d+L%d+U\r\n", Height_Val, Level_Val);
				}
				Led2_Flag = 1;
			}
			Pre_Level_Val = Level_Val;
		}
		
		if(Led2_Flag)
		{
			if(Led2_200ms_flag)
			{
				Led2_200ms_flag = 0;
				led2_count++;
				if(led2_count <= 11)
				{
					LD2_Blink();
				}else
				{
					ledodd2 = 0;
					led2_count = 0;
					Led2_Flag = 0;
				}
			}		
		}	
		if(Led3_Flag)
		{
			if(Led3_200ms_flag)
			{
				Led3_200ms_flag = 0;
				led3_count++;
				if(led3_count <= 11)
				{
					LD3_Blink();
				}else
				{
					ledodd3 = 0;
					led3_count = 0;
					Led3_Flag = 0;
				}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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
	static uint16_t led_count;
	static uint8_t led2_count, led3_count;
	static uint16_t adc_count;

	if(htim->Instance == TIM4)
	{
		if(++key_count == 10)
		{
			key_count = 0;
			Key_Flag = 1;
		}
		if(++led_count == 1000)
		{
			led_count = 0;
			Led_Flag = 1;
		}

		if(++led2_count == 200)
		{
			led2_count = 0;
			Led2_200ms_flag = 1;		
		}
		
		if(++led3_count == 200)
		{
			led3_count = 0;
			Led3_200ms_flag = 1;
		}
		if(++adc_count == 1000)
		{
			adc_count = 0;
			Adc_Flag = 1;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(rx_buf[0] == 'C' || rx_buf[0] == 'c')
		{
			printf("C:H%02d+L%d\r\n", Height_Val, Level_Val);
			Led3_Flag = 1;
		}
		if(rx_buf[0] == 'S' || rx_buf[0] == 's')
		{
			printf("S:TL%02d+TM%02d+TH%02d\r\n", Th1, Th2, Th3);
			Led3_Flag = 1;
		}
		else
		{
			memset(rx_buf, 0, sizeof(rx_buf));
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buf, 1);
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
