/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include <math.h>
#include <stdbool.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Q15_ONE      	32767      // Max. positiver Q15-Wert (~+0.99997)
#define Q15_MIN     	-32768      // Min. Q15-Wert (-1.0)
#define Q15_SCALE    	32768      // Skalierungsfaktor (für float <-> Q15)
#define Q15_ROUND    	16384      // Für korrektes Runden bei >>15
#define ALPHA_Q15    	8192             // = 0.25
#define ONE_MINUS_A   	(32767 - ALPHA_Q15)  // = 24575
#define ALPHA_F		  	0.25f
#define N				16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
const float coeffs[N];

// Ringbuffer for storing the input values
float buffer[N] = { 0.0f };
int buf_index = 0;  // zeigt immer auf das nächste freie Feld (x[n])
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Exercise - Part 1
 *
 * Theoretical Answers:
 *  - ema_filter() and ema_filter_q15() almost take up the same time to execute the filter
 *  -> barely a difference
 *  - There is also no huge difference regarding the code size
 *  - There are many reasons, why the Q15 format is preferred:
 *  	-> No FPU on the MCU, Integer is executed faster
 *  	-> Q15 takes up less space
 *  	-> Q15 is executed faster on DSP hardware
 */
float ema_filter(float x) {
	static float y = 0;
	y = ALPHA_F * x + (1 - ALPHA_F) * y;
	return y;
}

int16_t ema_filter_q15(int16_t x_q15) {
	static int16_t y_q15 = 0;

	// apply the filter
	int32_t acc = (int32_t) ALPHA_Q15 * x_q15 + (int32_t) ONE_MINUS_A * y_q15;

	// Rounding
	acc += 16384;

	// Shift back to Q15 format
	acc >>= 15;

	// Saturate to int16_t range
	y_q15 = __SSAT(acc, 16);
	return y_q15;
}

/*
 * Exercise - Part 2
 */
// FIR-Filterfunktion (Einzelwert)
float fir_filter(float x) {
	float y = 0.0f;

	// a) Naive Implementation (with Array Shifting)
//	for (int i = N - 1; i > 0; i--) {
//		buffer[i] = buffer[i - 1];
//	}
//	buffer[0] = x;
//
//	// Implementation of the filter
//	for (int n = 0; n < N; n++) {
//		y += coeffs[n] * buffer[n];
//	}

	// b) Circular Buffer Approach
	// Store the current sample at the current buffer index
	buffer[buf_index] = x;

	// Iterate backwards through the samples (x[n], x[n-1], ..., x[n-(N-1)])
	int i = buf_index;
	for (int n = 0; n < N; n++) {
		y += coeffs[n] * buffer[i];

		// Advance index forward with wrap-around
		i = (i + 1) & (N - 1);
	}

	// Increment the buffer index for the next sample
	buf_index = (buf_index + 1) & (N - 1);

	return y;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM6_Init();
	MX_DAC1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6); // Start TIM 6 with interrupts

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Enable channel 1
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // Enable channel 2
	HAL_ADC_Start_IT(&hadc1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 4;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 708;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | DBG_Pin_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
	GPIO_InitStruct.Pin = LPUART1_TX_Pin | LPUART1_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 DBG_Pin_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | DBG_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Coefficients for the FIR filter, created by the Matlab function fir1()
const float coeffs[N] = { 0.00850954f, 0.01329192f, 0.02670814f, 0.04712802f,
		0.07130295f, 0.09499066f, 0.11382579f, 0.12424298f, 0.12424298f,
		0.11382579f, 0.09499066f, 0.07130295f, 0.04712802f, 0.02670814f,
		0.01329192f, 0.00850954f };

/*
 * Coefficients for the FIR filter with a order of N = 128
 * Not recommandable, since the cut off frequency is lower than simulated, if using optimization -O0
 */
//const float coeffs[N] = { -0.0003177704f, -0.0002494781f, -0.0001627163f,
//		-0.0000583425f, 0.0000620575f, 0.0001955030f, 0.0003369948f,
//		0.0004790615f, 0.0006115993f, 0.0007220988f, 0.0007963152f,
//		0.0008193837f, 0.0007773267f, 0.0006588361f, 0.0004571689f,
//		0.0001719469f, -0.0001893611f, -0.0006105005f, -0.0010662309f,
//		-0.0015230116f, -0.0019406457f, -0.0022748694f, -0.0024807729f,
//		-0.0025168461f, -0.0023493517f, -0.0019566596f, -0.0013331356f,
//		-0.0004921694f, 0.0005320479f, 0.0016843138f, 0.0028899921f,
//		0.0040584651f, 0.0050883940f, 0.0058745647f, 0.0063159375f,
//		0.0063243847f, 0.0058334920f, 0.0048067372f, 0.0032443459f,
//		0.0011881670f, -0.0012759932f, -0.0040189885f, -0.0068721229f,
//		-0.0096341099f, -0.0120809652f, -0.0139783613f, -0.0150957633f,
//		-0.0152214911f, -0.0141777372f, -0.0118345129f, -0.0081215170f,
//		-0.0030370127f, 0.0033470373f, 0.0108841068f, 0.0193567702f,
//		0.0284848567f, 0.0379377516f, 0.0473502114f, 0.0563408074f,
//		0.0645319139f, 0.0715700298f, 0.0771451743f, 0.0810081397f,
//		0.0829845036f, 0.0829845036f, 0.0810081397f, 0.0771451743f,
//		0.0715700298f, 0.0645319139f, 0.0563408074f, 0.0473502114f,
//		0.0379377516f, 0.0284848567f, 0.0193567702f, 0.0108841068f,
//		0.0033470373f, -0.0030370127f, -0.0081215170f, -0.0118345129f,
//		-0.0141777372f, -0.0152214911f, -0.0150957633f, -0.0139783613f,
//		-0.0120809652f, -0.0096341099f, -0.0068721229f, -0.0040189885f,
//		-0.0012759932f, 0.0011881670f, 0.0032443459f, 0.0048067372f,
//		0.0058334920f, 0.0063243847f, 0.0063159375f, 0.0058745647f,
//		0.0050883940f, 0.0040584651f, 0.0028899921f, 0.0016843138f,
//		0.0005320479f, -0.0004921694f, -0.0013331356f, -0.0019566596f,
//		-0.0023493517f, -0.0025168461f, -0.0024807729f, -0.0022748694f,
//		-0.0019406457f, -0.0015230116f, -0.0010662309f, -0.0006105005f,
//		-0.0001893611f, 0.0001719469f, 0.0004571689f, 0.0006588361f,
//		0.0007773267f, 0.0008193837f, 0.0007963152f, 0.0007220988f,
//		0.0006115993f, 0.0004790615f, 0.0003369948f, 0.0001955030f,
//		0.0000620575f, -0.0000583425f, -0.0001627163f, -0.0002494781f,
//		-0.0003177704f };
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
