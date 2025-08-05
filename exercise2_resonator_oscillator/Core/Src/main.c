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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 1024
uint16_t adc_values[SAMPLE_SIZE];
volatile uint16_t sample_index = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
// Gain factor in order to normalize peak magnitude to 1
float b0 = 0.01;

// Controlling bandwidth of the filter, has to be between 0 and 1
float alpha = 0.99;

// The resonance frequency
float w0 = 2 * M_PI * 500 / 48000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float resonance_filter(float currentInput) {
	static float x[3] = { 0 };
	static float y[3] = { 0 };

	// saves the current value
	x[0] = currentInput;

	// Applies the filter to the input
	y[0] = 2 * alpha * cos(w0) * y[1] - powf(alpha, 2) * y[2]
			+ b0 * (x[0] - x[2]);

	// Update Delay Lines
	x[2] = x[1];
	x[1] = x[0];
	y[2] = y[1];
	y[1] = y[0];

	return y[0];
}

float oscillator() {
    static float y[3] = {0};
    static int started = 0;
    float x;

    // Set impulse only once
    if (!started) {
        x = 2042.0f;
        started = 1;
    } else {
        x = 0.0f;
    }

    // Resonator filter equation
    y[0] = 2.0f * cos(w0) * y[1] - y[2] + sin(w0) * x;

    // Update delay lines
    y[2] = y[1];
    y[1] = y[0];

    return y[0];
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
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6); // Start TIM 6 with interrupts

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Enable channel 1
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // Enable channel 2

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
	htim6.Init.Prescaler = 170 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 20;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

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

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
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
/**
 * Simulates the input data for the ADC values.
 * The input is a signal composed of two sinusoidal components:
 * 		- One sinus with the frequency of  500 Hz
 * 		- One sinus with the frequency of 8 kHz
 */
uint16_t adc_values[1024] = { 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197, 275, 244, 1102, 1965,
		1946, 1044, 147, 140, 1024, 1913, 1920, 1044, 173, 192, 1102, 2016,
		2048, 1197, 350, 392, 1324, 2260, 2311, 1479, 650, 709, 1656, 2605,
		2669, 1848, 1028, 1095, 2048, 3001, 3068, 2248, 1427, 1491, 2440, 3387,
		3446, 2617, 1785, 1836, 2772, 3704, 3746, 2899, 2048, 2080, 2994, 3904,
		3923, 3052, 2176, 2183, 3071, 3956, 3949, 3052, 2150, 2131, 2994, 3852,
		3821, 2899, 1974, 1931, 2772, 3609, 3557, 2617, 1673, 1614, 2440, 3263,
		3199, 2248, 1295, 1229, 2048, 2867, 2801, 1848, 897, 833, 1656, 2482,
		2423, 1479, 539, 487, 1324, 2165, 2122, 1197 };
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
