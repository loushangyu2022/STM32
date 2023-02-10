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
#include "stdbool.h"
#include "i2c.h"
#include "stdio.h"
#include "string.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	unsigned char judge_sta; // 0:表示按键没有按下 1:表示按键按下，但是不知道是不是真的按下，需要通过延时消抖进行判断 2:表示按键按下，已经经过了延时消抖，可以认为是真的按下了
	bool key_sta;			 // 按键的状态，0表示按下，1表示没有按下
	bool single_flag;		 // 单次按键标志，0表示没有按下，1表示按下了
} keys;

keys key[4] = {0};//按键初始化
unsigned char menu = 1;//菜单选择 1:密码输入 2：状态显示
unsigned char psd[3] = {'@', '@', '@'};//用于保存当前密码值
char lcd_string[30] = {'@'};//用于作为LCD显示的字符串
char password_line[11] = {'@', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};//保存的是密码的可输入字符，当初始时没有按键按下的情况时，显示的是@，当按下按键时，显示的是对应的数字
unsigned char key_num[3] = {0, 0, 0};//用于保存当前按键按下的次数，用于作为密码的索引
char uart_text[7];//用于保存串口接收到的字符串,这是由uart_word组成的字符串
unsigned char uart_word;//用于保存串口接收到的字符
unsigned char uart_len;//用于保存串口接收到的字符串的长度
__IO uint32_t PWM_Tick;//用于保存系统运行到当前的时间
__IO uint32_t led_Tick_1;//用于保存led1亮时的时间
__IO uint32_t led_Tick_2;//用于保存led2亮时的时间，用于闪烁
unsigned char error_time;//用于保存错误次数
unsigned char error_flag;//用于保存错误标志，0表示没有错误，1表示有错误
unsigned char right_flag;//用于保存正确标志，0表示没有正确，1表示有正确

unsigned char i2c_read(unsigned char address)//读取eeprom存储的密码值
{
	unsigned char i2c_val;
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();

	I2CSendByte(address);
	I2CWaitAck();

	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();

	i2c_val = I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	return i2c_val;
}

void i2c_write(unsigned char address, unsigned char info)//向eeprom存储密码值
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();

	I2CSendByte(address);
	I2CWaitAck();

	I2CSendByte(info);
	I2CWaitAck();
	I2CStop();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器3回调函数，每20ms执行一次，用于判断是否有按键按下
{
	if (htim->Instance == TIM3)
	{
		key[0].key_sta = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		key[1].key_sta = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		key[2].key_sta = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		key[3].key_sta = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		for (int i = 0; i < 4; i++)
		{
			switch (key[i].judge_sta)
			{
			case 0:
			{
				if (key[i].key_sta == 0)
				{
					key[i].judge_sta = 1;
				}
				else
				{
					key[i].judge_sta = 0;
				}
			}
			break;
			case 1:
			{
				if (key[i].key_sta == 0)
				{
					key[i].single_flag = 1;
					key[i].judge_sta = 2;
				}
				else
				{
					key[i].judge_sta = 0;
				}
			}
			break;
			case 2:
			{
				if (key[i].key_sta == 1)
				{
					key[i].judge_sta = 0;
				}
			}
			break;
			}
		}
	}
}
void key_proc()//按键处理函数,当按键按下时，key_num的值会加1，当key_num的值大于10时，key_num的值会变为1，这样就可以实现循环显示0-9，并且跳过@
{
	if (key[0].single_flag == 1)
	{
		key_num[0] = key_num[0] + 1;
		if (key_num[0] > 10)
		{
			key_num[0] = 1;
		}
		key[0].single_flag = 0;
	}

	if (key[1].single_flag == 1)
	{
		key_num[1] = key_num[1] + 1;
		if (key_num[1] > 10)
		{
			key_num[1] = 1;
		}
		key[1].single_flag = 0;
	}
	if (key[2].single_flag == 1)
	{
		key_num[2] = key_num[2] + 1;
		if (key_num[2] > 10)
		{
			key_num[2] = 1;
		}
		key[2].single_flag = 0;
	}

	if (key[3].single_flag == 1)
	{
		psd[0] = i2c_read(0x00);//读取eeprom中的密码值
		HAL_Delay(1);//延时1ms,这是因为i2c的读写速度太快了，如果不延时，会导致读取的值不正确
		psd[1] = i2c_read(0x01);
		HAL_Delay(1);
		psd[2] = i2c_read(0x02);
		HAL_Delay(1);
		if ((password_line[key_num[0]] == psd[0]) && (password_line[key_num[1]] == psd[1]) && (password_line[key_num[2]] == psd[2]))//判断输入的密码是否正确
		{
			menu = 2;//正确，进入菜单2
			PWM_Tick = uwTick;//记录当前时间
			led_Tick_1 = uwTick;//记录当前时间
			error_time = 0;//错误次数清零
			right_flag = 1;//正确标志位置1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
		}
		else
		{
			key_num[0] = 0;//错误，清空输入的密码
			key_num[1] = 0;
			key_num[2] = 0;
			error_time++;//错误次数加1
			if (error_time >= 3)//错误次数大于等于3次时，led2闪烁
			{
				error_flag = 1;//错误标志位置1
				led_Tick_2 = uwTick;//记录当前时间
			}
		}
		key[3].single_flag = 0;
	}
}
void lcd_disp()
{

	switch (menu)
	{
	case 1:
	{
		__HAL_TIM_SET_AUTORELOAD(&htim2, 999);//密码输入界面就是一个PWM波，频率为1KHz，占空比为50%
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 500);
		sprintf(lcd_string, "       PSD");
		LCD_DisplayStringLine(Line2, (unsigned char *)lcd_string);
		sprintf(lcd_string, "    B1:%c", password_line[key_num[0]]);
		LCD_DisplayStringLine(Line4, (unsigned char *)lcd_string);
		sprintf(lcd_string, "    B2:%c", password_line[key_num[1]]);
		LCD_DisplayStringLine(Line5, (unsigned char *)lcd_string);
		sprintf(lcd_string, "    B3:%c", password_line[key_num[2]]);
		LCD_DisplayStringLine(Line6, (unsigned char *)lcd_string);
	}
	break;
	case 2:
	{
		__HAL_TIM_SET_AUTORELOAD(&htim2, 499);//菜单2界面就是一个PWM波，频率为2KHz，占空比为10%
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 50);
		sprintf(lcd_string, "       STA");
		LCD_DisplayStringLine(Line2, (unsigned char *)lcd_string);
		sprintf(lcd_string, "    F:%4dHz", 2000);
		LCD_DisplayStringLine(Line4, (unsigned char *)lcd_string);
		sprintf(lcd_string, "    D:%d%%", 10);
		LCD_DisplayStringLine(Line5, (unsigned char *)lcd_string);
		LCD_ClearLine(Line6);
	}
	break;
	}
	if (menu == 2)
	{
		if (uwTick - PWM_Tick > 5000)//5秒后自动返回菜单1
		{
			__HAL_TIM_SET_AUTORELOAD(&htim2, 999);//密码输入界面就是一个PWM波，频率为1KHz，占空比为50%
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 500);
			menu = 1;
			LCD_Clear(Black);//清屏
			key_num[0] = 0;
			key_num[1] = 0;
			key_num[2] = 0;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		uart_text[uart_len++] = uart_word;//将接收到的字符存入数组,uart_len++是先赋值再加1,就是初始时uart_len=0,所以先存入数组的是uart_text[0]
		HAL_UART_Receive_IT(&huart1, &uart_word, 1);//开启串口中断,&uart_word是接收到的字符	,要求特别注意这里，需要传入的是地址不是值
	}
}
unsigned char judge_uart_text()//判断接收到的字符串是否符合要求
{

	if (uart_len == 0)
	{
		return 0;
	}
	if (uart_len != 7)
	{
		return 2;
	}
	if (uart_text[3] == '-')
	{
		for (int i = 0; i < 3; i++)
		{
			if ((uart_text[i] < '0') || (uart_text[i] > '9'))
			{
				return 2;
			}
		}
		for (int i = 4; i < 7; i++)
		{
			if ((uart_text[i] < '0') || uart_text[i] > '9')
			{
				return 2;
			}
		}
		return 1;
	}
	else
	{
		return 2;
	}
}
void change_password()//修改密码
{
	unsigned char tips[30];
	if (judge_uart_text() == 1)
	{
		if ((uart_text[0] == psd[0]) && (uart_text[1] == psd[1]) && (uart_text[2] == psd[2]))
		{
			psd[0] = uart_text[4];
			psd[1] = uart_text[5];
			psd[2] = uart_text[6];
			i2c_write(0x00, psd[0]);
			HAL_Delay(5);
			i2c_write(0x01, psd[1]);
			HAL_Delay(5);
			i2c_write(0x02, psd[2]);
			HAL_Delay(5);
			sprintf((char *)tips, "OK\n");
			HAL_UART_Transmit(&huart1, tips, strlen(tips), 50);
		}
		else
		{
			sprintf((char *)tips, "Errors\n");
			HAL_UART_Transmit(&huart1, tips, strlen(tips), 50);
		}
	}
	else if (judge_uart_text() == 2)
	{
		sprintf((char *)tips, "Error\n");
		HAL_UART_Transmit(&huart1, tips, strlen(tips), 50);
	}
	uart_len = 0;
}
void led_proc()
{
	if ((uwTick - led_Tick_1 >= 5000) && (right_flag == 1))//5秒后自动关闭LED,并且只有在输入正确的密码之后才会在5秒后关闭led1与led2
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
		error_flag = 0;
		right_flag = 0;
	}
	if (error_flag == 1)
	{

		if (uwTick - led_Tick_2 >= 100)//每100ms就闪烁一次
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
			led_Tick_2 = uwTick;
		}
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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	LCD_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 999);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 500);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart1, &uart_word, 1);
	i2c_write(0x00, '1');
	HAL_Delay(5);
	i2c_write(0x01, '2');
	HAL_Delay(5);
	i2c_write(0x02, '3');
	HAL_Delay(5);

	psd[0] = '1';
	psd[1] = '2';
	psd[2] = '3';

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		key_proc();
		lcd_disp();
		change_password();
		led_proc();
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
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

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1598;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
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
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
							 PC1 PC2 PC3 PC4
							 PC5 PC6 PC7 PC8
							 PC9 PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PB5 PB6 PB7 PB8
							 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

#ifdef USE_FULL_ASSERT
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
