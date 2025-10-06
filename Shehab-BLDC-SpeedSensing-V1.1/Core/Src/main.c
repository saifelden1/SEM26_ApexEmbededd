/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author 		: ShehabMohamed
  * @brief          : Main program body
  * @version 		: 0.2
  * @date			: 2025-18-03
  * @detail			: BLDC Drive Program
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "SpeedModule.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	/**
	 * @typedef EXTI_CallBack_t
	 * @brief Configure any variable to be pointer to function .
	 */
	typedef void(*EXTI_CallBack_t)(void);//Global Pointer to function Data Type
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	/* USER CODE BEGIN PD */
#define HALL_OverSample 	3
		/**
			 * @def TIM_DELAY
			 * @brief Control the wait state between steps
			 * @note change in this control the motor speed and torque
			 */
	#define TIM_DELAY	5
	#define MAX_PWM		100
		/**
			 * @def MIN_PWM
			 * @brief define the Minimum duty cycle value that adc reach to avoid the fault in pwm
			 * @note Do not decrease this value to avoid faults
			 */
	#define MIN_PWM		2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float MotorSpeedRpm=0;
float MotorSpeedKmh=0;
extern uint32_t PulsesPerInterval;
/**
	 * @var     :PrevMotorStep
	 * @brief   :Global variable storing Previous step of the motor .
	 * @warning :Do not modify this value directly.
	 * @Type    :uint8_t
	 */
	uint8_t PrevMotorStep;
		/**
		 * @var     :MotorStep
		 * @brief   :Global variable storing Motor step .
		 * @warning :Do not modify this value directly.
		 * @Type    :uint8_t
		 */
	uint8_t MotorStep;
			/**
			 * @var     :MotorStep
			 * @brief   :Global variable storing DutyCycle of motor to operate .
			 * @warning :Do not modify this value directly.
			 * @Type    :uint16_t
			 */
	uint16_t DutyCycle;
			/**
			 * @var     :HallState1
			 * @var 	:HallState2
			 * @var		:HallState3
			 * @brief   :Global variables storing reads of hall sensors .
			 * @warning :Do not modify this value directly.
			 * @Type    :uint8_t
			 */
	uint8_t HallState1=0,HallState2=0,HallState3=0;
			/**
			 * @var     :USER_CallBack
			 * @brief   :Global variable storing address of the call back funtion .
			 * @warning :Do not modify this value directly.
			 * @Type    :pointer to function
			 */
	EXTI_CallBack_t USER_CallBack=NULL;//GlobalPointerToFunction

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void RegisterCallBack(EXTI_CallBack_t Function);
void State1(void);
void State2(void);
void State3(void);

void WaitState(void);
void NextStep(void);
void HallState(void);

void Delay_us(uint16_t DELAY);
void Delay_ms(uint16_t DELAY);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	 HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
     HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
     HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);


     HAL_NVIC_EnableIRQ(EXTI3_IRQn);
     HAL_NVIC_EnableIRQ(EXTI4_IRQn);
     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

      // HAL_TIM_Base_Start(&htim1);
     HAL_TIM_Base_Start_IT(&htim2);
//       HAL_TIM_Base_Start(&htim3);
//       HAL_TIM_Base_Start(&htim4);

//       HAL_TIM_PWM_Init(&htim1);
//       HAL_TIM_PWM_Init(&htim3);
//       HAL_TIM_PWM_Init(&htim4);

       /*******************************************************************///A-Driving
       HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//TIMER1 1->2//PA10
       HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//TIMER1 1->2//PA9
       /*******************************************************************///B-Driving
       HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//TIMER3 1->4//PA7
       HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//TIMER3 1->4//PB0
       /*******************************************************************///C-Driving
       HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);//TIMER4 1->4//PB6
       HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);//TIMER4 1->4//PB7
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, 30);
	  	  DutyCycle=((HAL_ADC_GetValue(&hadc1)*100)+2047)/4095;
	  	  if(DutyCycle>=MAX_PWM)
	  	  {
	  		  DutyCycle=MAX_PWM;
	  	  }
	  	  if(DutyCycle<=MIN_PWM)
	  	  {
	  		  DutyCycle=0;

	  	  }

	  	  HallState();


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 3599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
  GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*	@Fun   			:RegisterCallBack
 * 	@brief 			:the function take address of call back function to assign it in global pointer to function
 * 	@parameter[in] 	:Function carry the address of the called back function
 * 	@return			:void
 *
 */
void 		RegisterCallBack(EXTI_CallBack_t Function)
{
	USER_CallBack=Function;
}
/*	@Fun   			:State1
 * 	@brief 			:the function reads the hall sensor 1 output
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void State1(void)//A
{
	HallState1=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}
/*	@Fun   			:State2
 * 	@brief 			:the function reads the hall sensor 2 output
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void State2(void)//B
{
	HallState2=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}
/*	@Fun   			:State3
 * 	@brief 			:the function reads the hall sensor 3 output
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void State3(void)//C
{
	HallState3=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

}

/*	@Fun   			:HallState
 * 	@brief 			:the function select the motor step according to the hall sensor reads
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void HallState(void)//Need To be run after few us
{

	uint8_t HallCounts[]={0,0,0};
	for (uint8_t i=0;i<HALL_OverSample;i++)
	{
		HallCounts[0]+=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		HallCounts[1]+=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		HallCounts[2]+=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	}
	uint8_t hall=0;
	if(HallCounts[0]>=HALL_OverSample/2)
		hall|=(1<<0);
	if(HallCounts[1]>=HALL_OverSample/2)
		hall|=(1<<1);
	if(HallCounts[2]>=HALL_OverSample/2)
		hall|=(1<<2);
	hall&=0x7;
	PrevMotorStep=MotorStep;

	switch (hall)
	{
		case 3://011
			MotorStep=0;
		break;

		case 2://010
			MotorStep=1;
		break;

		case 6://110
			MotorStep=2;
		break;

		case 4://100
			MotorStep=3;
		break;

		case 5 ://101
			MotorStep=4;
		break;

		case 1://001
			MotorStep=5;
		break;

		case 7 ://Invalid cases
		case 0:	//Invalid cases
			MotorStep=6;
		break;

	}
	NextStep();


}
/*	@Fun   			:NextStep
 * 	@brief 			:the function give the signals to drive the gate drivers according to the motor step that comes from hall sensors
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void NextStep(void)
{
	/*******************************************************************///U-Driving
	//TIM1->CCR1---u+
	//TIM1->CCR2---u-
	/*******************************************************************///V-Driving
	//TIM4->CCR2---v+
	//TIM4->CCR1---v-
	/*******************************************************************///W-Driving
	//TIM3->CCR3---w+
	//TIM3->CCR2---w-
	/*******************************************************************/
	WaitState();/*before changing the state checks if the state already change or not
if yes that means the motor does electric change and also should do delay between the state changes
if no that means
no pulse generated and doesnt need to wait between state changes  */
	switch (MotorStep)
	{
	case 0://011

		/*******************************************************************///U-Driving
		TIM1->CCR3=0;//u+
		TIM1->CCR2=0;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=DutyCycle;//v+
		TIM4->CCR1=0;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=0;//w+
		TIM3->CCR2=DutyCycle;//w-
		/*******************************************************************/
	break;

	case 1://010

		/*******************************************************************///U-Driving
		TIM1->CCR3=0;//u+
		TIM1->CCR2=DutyCycle;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=DutyCycle;//v+
		TIM4->CCR1=0;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=0;//w+
		TIM3->CCR2=0;//w-
		/*******************************************************************/

	break;

	case 2://110

		/*******************************************************************///U-Driving
		TIM1->CCR3=0;//u+
		TIM1->CCR2=DutyCycle;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=0;//v+
		TIM4->CCR1=0;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=DutyCycle;//w+
		TIM3->CCR2=0;//w-
		/*******************************************************************/
	break;

	case 3://100
		/*******************************************************************///U-Driving
		TIM1->CCR3=0;//u+
		TIM1->CCR2=0;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=0;//v+
		TIM4->CCR1=DutyCycle;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=DutyCycle;//w+
		TIM3->CCR2=0;//w-
		/*******************************************************************/

	break;

	case 4://101
		/*******************************************************************///U-Driving
		TIM1->CCR3=DutyCycle;//u+
		TIM1->CCR2=0;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=0;//v+
		TIM4->CCR1=DutyCycle;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=0;//w+
		TIM3->CCR2=0;//w-
		/*******************************************************************/
	break;

	case 5://001
		/*******************************************************************///U-Driving
		TIM1->CCR3=DutyCycle;//u+
		TIM1->CCR2=0;//u-
		/*******************************************************************///V-Driving
		TIM4->CCR2=0;//v+
		TIM4->CCR1=0;//v-
		/*******************************************************************///W-Driving
		TIM3->CCR3=0;//w+
		TIM3->CCR2=DutyCycle;//w-
	/*******************************************************************/
		break;
	case 6:
		/*******************************************************************///U-Driving
				TIM1->CCR3=0;//u+
				TIM1->CCR2=0;//u-
				/*******************************************************************///V-Driving
				TIM4->CCR2=0;//v+
				TIM4->CCR1=0;//v-
				/*******************************************************************///W-Driving
				TIM3->CCR3=0;//w+
				TIM3->CCR2=0;//w-
			/*******************************************************************/
	break;


	}


}
/*	@Fun   			:WaitState
 * 	@brief 			:the function force the pwm to halt for a specific off time this control the power of motor and speed
 * 	@parameter[in] 	:void
 * 	@return			:void
 *
 */
void WaitState(void)
{
		//Speed Calculate Mechanism
	if(PrevMotorStep!=MotorStep)
		{
		PulsesPerInterval++;
		/*******************************************************************///A-Driving
		TIM1->CCR3=0;
		TIM1->CCR2=0;
		/*******************************************************************///B-Driving
		TIM3->CCR2=0;
		TIM3->CCR3=0;
		/*******************************************************************///C-Driving
		TIM4->CCR1=0;
		TIM4->CCR2=0;
		/*******************************************************************/
		Delay_us(TIM_DELAY);
		}
		else
		{

		}
	PrevMotorStep = MotorStep;

}



/*	@Fun   			:Delay_us
 * 	@brief 			:the function do a delay with micro seconds
 * 	@parameter[in] 	:DELAY
 * 	@type 			:uint16_t
 * 	@return			:void
 *
 */
void 		Delay_us		(uint16_t DELAY)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	 while (__HAL_TIM_GET_COUNTER(&htim2) <= DELAY);
}
/*	@Fun   			:Delay_ms
 * 	@brief 			:the function do a delay with milli seconds
 * 	@parameter[in] 	:DELAY
 * 	@type 			:uint16_t
 * 	@return			:void
 *
 */
void 		Delay_ms		(uint16_t DELAY)
{

	for (uint16_t Counter=1;Counter<=DELAY;Counter++)
	{
		Delay_us(1000);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	SpeedCalculation(&MotorSpeedRpm,&MotorSpeedKmh);
    	PulsesPerInterval = 0;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
