/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "humd.h"
#include "Temp.h"
#include "LCD_I2C.h"
#include "Queue.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum user
{
  Mode_Normal,
  Mode_Humd,
  Mode_Temp
} user_t;
user_t Mode = Mode_Normal;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT_ADC 500U
#define UART_BUFFER_LEN 6
#define START_BYTE 1
#define END_BYTE 2
#define ERROR_FRAME 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
queue_t xQueue[10] = {None, NULL};
int check;
int check10;
int check3;
uint16_t frequency;
uint16_t Humd1;
uint16_t humd;
uint16_t frequency1;
uint8_t current;
uint8_t flag_ADC = 0;
uint8_t period;

// UART
uint8_t ucRxData;
uint8_t ucRxBuffer[UART_BUFFER_LEN] = {0};
uint8_t ucRxFlag = 0;
uint8_t ucRxCnt = 0;
uint8_t ucRxBuffer_c[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Task_Humd(void);
void Task_Temp(void);
void Task_LCD(void);
void Task_UART(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void check2()
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // check = Sizeof_Queue_Current(xQueue);
  LCD_I2C_Init();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    while (Sizeof_Queue_Current(xQueue) != 0)
    {
      //check = Sizeof_Queue_Current(xQueue);
      (*xQueue[Sizeof_Queue_Current(xQueue) - 1].Func)();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 35;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim4.Init.Prescaler = 35;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (ucRxData == '&')
  {
    memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
    ucRxBuffer[0] = ucRxData;
    ucRxFlag = START_BYTE;
    HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  else if (ucRxFlag == START_BYTE && ucRxCnt < UART_BUFFER_LEN)
  {
    ucRxBuffer[++ucRxCnt] = ucRxData;
    if (ucRxData == '*')
    {
      // xEventGroupSetBitsFromISR(DISPLAY_GROUP, BIT_RECV, &xHigherPriorityTaskWoken);
			check++;
      Push_Queue(xQueue, IRQ_Event_UART_Recv, &Task_UART, 0, ucRxBuffer);
      memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
      ucRxCnt = 0;
      ucRxFlag = 0;
      HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
    }
    else
      HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  else if (ucRxCnt == UART_BUFFER_LEN)
  {
    ucRxFlag = ERROR_FRAME;
    memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
    ucRxCnt = 0;
    HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //check10++;
  if (htim->Instance == htim2.Instance)
  {
    Push_Queue(xQueue, IRQ_Event_Timer2, &Task_Temp, 0, NULL);
  }
  if (htim->Instance == htim3.Instance)
  {
    static uint8_t turn = 0;
    turn++;
    if( turn >= 100)
    {
     turn = 0;
     //check10++;
     Push_Queue(xQueue, IRQ_Event_Timer3, &Task_Temp, 0, NULL);
     Push_Queue(xQueue, IRQ_Event_Timer3, &Task_Humd, 0, NULL);
    }

  }
  

  /* Prevent unused argument(s) compilation warning */
  // UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == hadc1.Instance)
  {
    flag_ADC = 1;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  check3++;
  static int a = 0;
  a++;

  // uint16_t frequency;
  uint32_t timer1_count = __HAL_TIM_GET_COUNTER(&htim1); // lay gia tri moi cua timer
  static uint32_t lastTime = 0;
  uint32_t period = (timer1_count - lastTime);

  // tinh tan so
  frequency = (1000000 / period); // Chuyen doi ra HZ

  lastTime = timer1_count; // gia tri cua cua timer
  if (a == 100)
  {
    a = 0;
    Push_Queue(xQueue, IRQ_Event_Fre, &Task_Humd, frequency, NULL);
  }
}

void Task_Humd()
{
  uint8_t current = Sizeof_Queue_Current(xQueue);
  if (xQueue[current - 1].State == IRQ_Event_Fre)
  {
    frequency1 = xQueue[current - 1].data;
    Pop_Queue(xQueue);
    humd = Get_Humd(frequency1 - 700);
    Push_Queue(xQueue, Task_End_Event_Humd, &Task_LCD, humd, NULL);
  }
  else if(xQueue[current - 1].State == IRQ_Event_Timer3)
  {
    Pop_Queue(xQueue);
    Push_Queue(xQueue, IRQ_Event_UART_TransH, &Task_UART, humd, NULL);
  }
  else
  {
    Pop_Queue(xQueue);
  }
}

void Task_Temp()
{
  static double temp;
  uint8_t current = Sizeof_Queue_Current(xQueue);
  if (xQueue[current - 1].State == IRQ_Event_Timer2)
  {
    Pop_Queue(xQueue);
    uint16_t adc;
    HAL_ADC_Start_IT(&hadc1);
    int time = HAL_GetTick();
    while (flag_ADC != 1 && ((HAL_GetTick() - time) < TIMEOUT_ADC));
    if (flag_ADC == 1)
    {
      adc = HAL_ADC_GetValue(&hadc1); // Lay gia tri cua ADC
      temp = temp_convert(adc) * 10;
      Push_Queue(xQueue, Task_End_Event_Temp, &Task_LCD, (uint16_t)temp, NULL);
      flag_ADC = 0;
    }
  }
  else if(xQueue[current - 1].State == IRQ_Event_Timer3)
  {
    Pop_Queue(xQueue);
    Push_Queue(xQueue, IRQ_Event_UART_TransT, &Task_UART,(uint16_t)temp, NULL);
  }
  else
  {
    Pop_Queue(xQueue);
  }
}

void Task_LCD()
{
  // check++;
  uint8_t current = Sizeof_Queue_Current(xQueue);
  if (xQueue[current - 1].State == Task_End_Event_Humd)
  {
    Humd1 = xQueue[current - 1].data;
    Pop_Queue(xQueue);
    LCD_I2C_Set_Cursor(0, 0);
    LCD_I2C_Write_String("Humd:");
    LCD_I2C_WRITE_NUMBER(((int)Humd1));
    //HAL_Delay(100);
  }
  else if (xQueue[current - 1].State == Task_End_Event_Temp)
  {
    double temp = ((double)xQueue[current - 1].data) / 10;
    Pop_Queue(xQueue);
    LCD_I2C_Set_Cursor(1, 0);
    LCD_I2C_Write_String("Temp:");
    LCD_I2C_WRITE_Num_f(temp);
    //HAL_Delay(100);
  }
  else
  {
    Pop_Queue(xQueue);
  }
}

void Task_UART()
{
	check++;
  uint8_t current = Sizeof_Queue_Current(xQueue);
  if (xQueue[current - 1].State == IRQ_Event_UART_Recv)
  {
		//check++;
    uint8_t ucRxBuffer_c[6];
    memset((char *)ucRxBuffer_c, 0, strlen((char *)ucRxBuffer_c));
    strcpy( (char*) ucRxBuffer_c, (char*) xQueue[current - 1].array);
		Pop_Queue(xQueue);
    if (strcmp( (char *)ucRxBuffer_c, "&norm*") == 0)
      Mode = Mode_Normal;
    else if (strcmp((char *)ucRxBuffer_c, "&humd*") == 0)
      Mode = Mode_Humd;
    else if (strcmp((char *)ucRxBuffer_c, "&temp*") == 0)
      Mode = Mode_Temp;
    else if (ucRxBuffer_c[1] == 'T' && ucRxBuffer_c[2] == '_')
    {
      // check++;
      uint8_t per[2] = {ucRxBuffer_c[3], ucRxBuffer_c[4]};
      period = atoi((char *)per);
    }
  }
  else if ( (xQueue[current - 1].State == IRQ_Event_UART_TransH) || (xQueue[current - 1].State == IRQ_Event_UART_TransT)  )
  {
    check10++;
    char str[100];
    double Temp = 0;
    uint8_t Humd = 0;
    if(xQueue[current - 1].State == IRQ_Event_UART_TransT)
    {
    Temp = ((double)(xQueue[current - 1].data))/10;
    }
    if(xQueue[current - 1].State == IRQ_Event_UART_TransH)
    {
     Humd = (uint8_t) xQueue[current - 1].data;
    }
    Pop_Queue(xQueue);
    switch (Mode)
    {
    case Mode_Temp:
      // if(flag == 1)
      //{
      if (Temp != 0)
      {
      sprintf(str, " Temp = %.1lf\n", Temp);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
      }
      // vTaskDelay(100);
      //  flag = 0;
      //  }
      break;
    case Mode_Humd:
      // if(flag == 1)
      //{
      if( Humd != 0 )
      {
      sprintf(str, "Humd = %d\n", Humd);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
      }
      // vTaskDelay(100);
      // flag = 0;
      // }
      break;
    case Mode_Normal:
      // if(flag == 1)
      //{
      //Get_Time();
      if(Humd != 0 )
      {
      sprintf(str, "Humd = %d\n", Humd);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
      }
     // vTaskDelay(100);
      if(Temp != 0 )
      {
      sprintf(str, "Temp = %.1lf\n", Temp);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
      }
      // vTaskDelay(100);
      // flag = 0;
      // }
      break;
    //default:
        //trans("Nhap sai Mode vui long nhap lai\n");
      // vTaskDelay(5);
    }

    // vTaskDelay(5);
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
