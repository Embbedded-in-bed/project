/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_8

#define HC_SRO4_PORT GPIOB
#define HC_SRO4_PIN GPIO_PIN_6

#define WF_PORT GPIOB
#define WF_PIN GPIO_PIN_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}




void delay(uint16_t Delay){ // will delay = Delay * 1 micro second (Delay should not exceed 2^16 - 1)
    __HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
    while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) < Delay);// wait for the counter to reach the us input in the parameter
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
uint16_t RH, TEMP;

float Temp = 0,Humi = 0;
uint8_t Presence = 0;

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start (void)
{
	Set_Pin_Output(DHT22_PORT, DHT22_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	HAL_Delay(1200);   // wait for > 1ms

	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	delay (20);   // wait for 30us
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin high

	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
}
uint8_t DHT22_Check_Response (void)
{
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
	uint8_t Response = 0;
	delay (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) // if the pin is low
	{
		delay (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));// wait for the pin to go low
	return Response;
}
uint8_t DHT22_Read (void)
{
	uint8_t i=0,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))){
			delay(1); // wait for the pin to go high
		}
		delay (40);   // wait for 40 us
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))){
			delay(1);  // wait for the pin to go low
		}
	}

	return i;
}

// delay
int mainUltra = 0;
int mainTemp = 0;
int mainfloat = 0;


int Delay_Array[3];
//

int u0=0;
int u1=0;
int u2=0;


int mainState;

int isTemp = 0;
int isFloat= 0;
int isUltra = 0;

int isTempTran = 0;
int isLevelTran = 0;

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



//	  if(isUltra == 1) {
//		  uint8_t pData_main[7] = {' ',' ',' ',' ',' ',' ',' '};
//		  sprintf(pData_main, "%d\r\n", mainUltra);
//		  HAL_UART_Transmit_IT(&huart1, &pData_main, 7);
//
//	  }

	  /*if(isTemp == 1 && mainTemp != 0 && isUltra ==1) {
		uint8_t T_main[3];
		T_main[0] = '0' + mainTemp/100;
		T_main[1] = '0' + (mainTemp%100)/10;
		T_main[2] = '0' + (mainTemp%10);

		  uint8_t pData_main[7] = {' ',' ',' ',' ',' ',' ',' '};
		  sprintf(pData_main, "%d\r\n", mainUltra);
		  //HAL_UART_Transmit_IT(&huart1, &pData_main, 7);

		  uint8_t arr[10];
		  int id = 0;
		  for(int i=0; i<3; i++){
			  arr[id++] = T_main[i];
		  }
		  for(int i=0; i<8; i++){
			  arr[id++] = pData_main[i];
		  }

		  HAL_UART_Transmit_IT(&huart1, &arr, 10);
	  }*/

	 // uint8_t arr

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 49;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /*if(isTemp == 1 && mainTemp != 0 && isUltra ==1) {
		uint8_t T_main[3];
		T_main[0] = '0' + mainTemp/100;
		T_main[1] = '0' + (mainTemp%100)/10;
		T_main[2] = '0' + (mainTemp%10);

		  uint8_t pData_main[7] = {' ',' ',' ',' ',' ',' ',' '};
		  sprintf(pData_main, "%d\r\n", mainUltra);
		  //HAL_UART_Transmit_IT(&huart1, &pData_main, 7);

		  uint8_t arr[10];
		  int id = 0;
		  for(int i=0; i<3; i++){
			  arr[id++] = T_main[i];
		  }
		  for(int i=0; i<8; i++){
			  arr[id++] = pData_main[i];
		  }

		  HAL_UART_Transmit_IT(&huart1, &arr, 10);
	  }*/


    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	HAL_TIM_Base_Start(&htim3);
			uint8_t T[3],H[3],Check;
			uint16_t Tsum, Hsum;
  /* Infinite loop */
  for(;;)
  {
	  isTemp = 1;
	 // taskENTER_CRITICAL();
	  	  HAL_UART_Transmit(&huart2, "Do\r\n", 4, 10);
	  	  DHT22_Start();
	  	  Presence = DHT22_Check_Response();
	  	  Rh_byte1 = DHT22_Read();
	  	  Rh_byte2 = DHT22_Read();

	  	  Temp_byte1 = DHT22_Read();
	  	  Temp_byte2 = DHT22_Read();

	  	  SUM = DHT22_Read();
	  	  Check = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
	  	  //HAL_UART_Transmit(&huart2, SUM, sizeof(SUM), HAL_MAX_DELAY);

	  	TEMP = ((Temp_byte1<<8)|Temp_byte2)&32767;
	  	//RH = ((Rh_byte1<<8)|Rh_byte2)&32767;

	  	  Temp = (float)(TEMP/10.0);
	  		 	  //Humi = (float)(RH/10.0);



	  		 	 /* Hsum = Rh_byte1*256 + Rh_byte2;

	  		 	H[0] = '0' + Hsum/100;
	  		    H[1] = '0' + (Hsum%100)/10;
	  		 	H[2] = '0' + (Hsum%10);*/



	  		 	/*HAL_UART_Transmit(&huart2, "Humid: ", 7, HAL_MAX_DELAY);
	  		 	HAL_UART_Transmit(&huart2, H, 3, 10);*/

	  		 	//HAL_UART_Transmit(&huart2, "\r\n", 2, 10);

	  		 		 	  if(Check == SUM){
	  		 		 		  if(Temp_byte1>127) {Tsum = (float)Temp_byte2/10*(-1);}
	  		 		 		  else {Tsum = (float)((Temp_byte1<<8) | Temp_byte2)/10;}


	  		 		 		T[0] = '0' + Tsum/100;
	  		 		 		T[1] = '0' + (Tsum%100)/10;
	  		 		 		T[2] = '0' + (Tsum%10);

	  		 		 	// HUART 1-----------------

	  		 		 		//isTempTran = 0;
							HAL_UART_Transmit(&huart1, T, 3, HAL_MAX_DELAY);
							HAL_UART_Transmit(&huart1, "\r\n", 2, HAL_MAX_DELAY);
							//isTempTran = 1;


							HAL_UART_Transmit(&huart2, "Temp: ", 6, HAL_MAX_DELAY);
	  		 		 		HAL_UART_Transmit(&huart2, T, 3, 10);



	  		 		 		HAL_UART_Transmit(&huart2, "\r\n", 2, 10);


	  		 		 		  HAL_UART_Transmit(&huart2, "Good\r\n", 6, 10);

	  		 		 		  mainTemp = Tsum;

	  		 		 		  //	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,sensorState);

	  		 		 		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	  		 		 	  }else{
	  		 		 		  HAL_UART_Transmit(&huart2, "Bad\r\n", 5, 10);
	  		 		 	  }
	  		  		 	HAL_UART_Transmit(&huart2, "\r\n", 2, 10);


	  		  		  osDelay(500);

	  	}




  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	  isFloat = 1;
	  //HAL_UART_Transmit(&huart2, "3\r\n", 3, 10);
	  uint8_t sensorState = HAL_GPIO_ReadPin(WF_PORT, WF_PIN);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,sensorState);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,sensorState);

	  mainfloat = sensorState;

    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	//taskENTER_CRITICAL();
	isUltra = 1;
	int hit = 0;
	int miss_count = 0;
	uint8_t state = 0;
	int delay;
	int echo=1;
	int id=0;
	int countultra = 0;
	int div = 50;
	uint8_t arr[div];

	int ultmp[3];

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	TIM1->CCR1 = 20;
  /* Infinite loop */
  for(;;)
  {
	//HAL_UART_Transmit(&huart2, "4\r\n", 3, 10);
	uint8_t test = "TEST\r\n";
	delay = 1000001;
	echo =  HAL_GPIO_ReadPin(HC_SRO4_PORT,HC_SRO4_PIN);

	if (echo) {
		delay = 0;
		while ( HAL_GPIO_ReadPin(HC_SRO4_PORT,HC_SRO4_PIN)) {
			delay++;
		}
	}

	if (delay < 1000001) {

		if (delay < 1000) {
			hit++;
			miss_count = 0;
		} else {
			if (miss_count++ > 100) { hit = 0; }
		 }

		if (hit > 100) {
			if (state == 0) {
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
				state = 1;
			}
		} else {
			if (state == 1) {
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

				state = 0;
			}
		}


		uint8_t pData[7] = {' ',' ',' ',' ',' ',' ',' '};

			if(countultra<div){
				arr[id] = delay;
				id = (id+1)%div;
				countultra++;
			}
			else{
				arr[id] = delay;
				id = (id+1)%div;

				delay = 0;

				for(int i=0; i<div; i++){
					delay += arr[i];
				}

				delay = delay/div;

				if(mainfloat == 1 && delay <= 120){
					sprintf(pData, "%d", delay);

				HAL_UART_Transmit(&huart1, &pData, 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart1, "\r\n", 2, HAL_MAX_DELAY);

				HAL_UART_Transmit(&huart2, "Ultra: ", 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, &pData, 7,HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "\r\n", 2, 10);

				}else {
					HAL_UART_Transmit(&huart2, "noooo: ", 7, HAL_MAX_DELAY);
					HAL_UART_Transmit(&huart2, "\r\n", 2, 10);

				}


			}

			//HAL_UART_Transmit(&huart2, "ABCTg: ", 7, HAL_MAX_DELAY);

	}







	osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

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
