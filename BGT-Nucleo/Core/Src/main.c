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
#include "fft.h"
#include "bgt60ltr11_spi.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Radar 1 data structures
uint16_t IFI_radar1 = 0;
uint16_t IFQ_radar1 = 0;
float32_t buffer1_radar1[2 * FFT_BUFFER_SIZE];
float32_t buffer2_radar1[2 * FFT_BUFFER_SIZE];
float32_t *active_buffer_radar1 = buffer1_radar1;
float32_t *processing_buffer_radar1 = buffer2_radar1;
uint16_t acquired_sample_count_radar1 = 0;
uint8_t data_ready_f_radar1 = 0;
float32_t peak_index_radar1 = 0.0f;
float32_t max_value_radar1 = 0.0f;
float32_t target_velocity_radar1 = 0.0f;

// Radar 2 data structures
uint16_t IFI_radar2 = 0;
uint16_t IFQ_radar2 = 0;
float32_t buffer1_radar2[2 * FFT_BUFFER_SIZE];
float32_t buffer2_radar2[2 * FFT_BUFFER_SIZE];
float32_t *active_buffer_radar2 = buffer1_radar2;
float32_t *processing_buffer_radar2 = buffer2_radar2;
uint16_t acquired_sample_count_radar2 = 0;
uint8_t data_ready_f_radar2 = 0;
float32_t peak_index_radar2 = 0.0f;
float32_t max_value_radar2 = 0.0f;
float32_t target_velocity_radar2 = 0.0f;

// Shared variables
uint8_t radar1_initialized = 0;
uint8_t radar2_initialized = 0;
uint32_t error_cnt = 0;

#define VELOCITY_BUFFER_SIZE 3
float32_t velocity_buffer_radar1[VELOCITY_BUFFER_SIZE];
uint8_t velocity_buffer_index_radar1 = 0;
uint8_t velocity_buffer_filled_radar1 = 0;
float32_t velocity_average_radar1 = 0.0f;

float32_t velocity_buffer_radar2[VELOCITY_BUFFER_SIZE];
uint8_t velocity_buffer_index_radar2 = 0;
uint8_t velocity_buffer_filled_radar2 = 0;
float32_t velocity_average_radar2 = 0.0f;

uint32_t pa10_toggle_timestamp = 0;
const uint32_t PA10_TOGGLE_INTERVAL = 5000; // 5 seconds in milliseconds
GPIO_PinState pa10_current_state = GPIO_PIN_RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float32_t calculate_rolling_average_with_filtering1(float32_t new_value);
float32_t calculate_rolling_average_with_filtering2(float32_t new_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Test

  // Initialize CS pins
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Radar 1 CS high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // Radar 2 CS high

  // Initialize Radar 1
  bgt60ltr11_HW_reset(RADAR_1);
  HAL_Delay(100);  // Wait for radar to stabilize
  if (bgt60ltr11_pulsed_mode_init_extended_range(RADAR_1) != HAL_OK) {
      Error_Handler();
  }

  // Initialize Radar 2
  bgt60ltr11_HW_reset(RADAR_2);
  HAL_Delay(100);  // Wait for radar to stabilize
  if (bgt60ltr11_pulsed_mode_init_extended_range(RADAR_2) != HAL_OK) {
      Error_Handler();
  }

  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim2);

  // Radars successfully initialized
  radar1_initialized = 1;
  radar2_initialized = 1;

  for (uint8_t i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
	  velocity_buffer_radar1[i] = 0.0f;
	  velocity_buffer_radar2[i] = 0.0f;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HI JASON!!!!!!
  while (1)
  {
	  //printf("PD Pin state: %s\r\n", HAL_GPIO_ReadPin(PD_GPIO_Port, PD_Pin) == GPIO_PIN_SET ? "HIGH" : "LOW");

	  // Process Radar 1 data when ready
	  //printf("radar_init=%u, data_ready=%u\r\n", radar1_initialized, data_ready_f_radar1);
	  if (radar1_initialized && data_ready_f_radar1) {
		  fft256_spectrum(processing_buffer_radar1);
		  find_peak_frequency(processing_buffer_radar1, FFT_BUFFER_SIZE, 1000, &peak_index_radar1, &max_value_radar1, &target_velocity_radar1);
		  printf("Radar 1 - velocity: %.5f m/s\t", target_velocity_radar1);
		  //printf("Radar 1 - velocity: %.5f m/s \r\n", target_velocity_radar1);
//		  if ((HAL_GPIO_ReadPin(PD_GPIO_Port, PD_Pin) == GPIO_PIN_SET && target_velocity_radar1 > 0) ||
//		      (HAL_GPIO_ReadPin(PD_GPIO_Port, PD_Pin) == GPIO_PIN_RESET && target_velocity_radar1 < 0)) {
//
//		    //velocity_average_radar1 = calculate_rolling_average_with_filtering1(target_velocity_radar1);
//		    printf("Radar 1 - velocity: %.5f m/s \r\n", target_velocity_radar1);
//
//		  }
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  data_ready_f_radar1 = 0;
	  } else {
		  printf("%-35s", "");
	  }

	  // Process Radar 2 data when ready
	  if (radar2_initialized && data_ready_f_radar2) {
		  fft256_spectrum(processing_buffer_radar2);
		  find_peak_frequency(processing_buffer_radar2, FFT_BUFFER_SIZE, 1000, &peak_index_radar2, &max_value_radar2, &target_velocity_radar2);
		  printf("Radar 2 - velocity: %.5f m/s", target_velocity_radar2);
//		  if ((HAL_GPIO_ReadPin(PD2_GPIO_Port, PD2_Pin) == GPIO_PIN_SET && target_velocity_radar2 > 0) ||
//			  (HAL_GPIO_ReadPin(PD2_GPIO_Port, PD2_Pin) == GPIO_PIN_RESET && target_velocity_radar2 < 0)) {
//
//			//velocity_average_radar2 = calculate_rolling_average_with_filtering2(target_velocity_radar2);
//			//printf("Radar 2 - velocity: %.5f m/s \r\n", target_velocity_radar2);
//			printf("\n");
//
//		  }
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  data_ready_f_radar2 = 0;
	  } else {
		  printf("\r\n");
	  }

//	  printf("-\r\n");
//	  if (HAL_GPIO_ReadPin(TD_GPIO_Port, TD_Pin) == GPIO_PIN_RESET) {
//		  // TD = 0
//		  if (HAL_GPIO_ReadPin(PD_GPIO_Port, PD_Pin) == GPIO_PIN_SET) {
//			  // PD = 1
//			  printf("APPROACHING!!!!!\r\n");
//		  } else {
//			  // PD = 0
//			  printf("DEPARTING!!!!!\r\n");
//		  }
//	  }

	  // Check if it's time to toggle PA10
//	  uint32_t current_time = HAL_GetTick();
//	  if(current_time - pa10_toggle_timestamp >= PA10_TOGGLE_INTERVAL) {
//		  // Toggle the state by writing the opposite of current state
//		  pa10_current_state = (pa10_current_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, pa10_current_state);
//		  pa10_toggle_timestamp = current_time;
//		  printf("PA10 set to %s\r\n", (pa10_current_state == GPIO_PIN_SET) ? "HIGH" : "LOW"); // improved debug message
//	  }



	  HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|PD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD_Pin */
  GPIO_InitStruct.Pin = PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin PD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|PD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        // Alternate between radars (you could also use a counter to give equal time)
        static uint8_t current_radar = RADAR_1;

        if (current_radar == RADAR_1) {
            // Process Radar 1
            if (acquired_sample_count_radar1 < FFT_BUFFER_SIZE) {
                if (bgt60ltr11_get_RAW_data(RADAR_1, &IFI_radar1, &IFQ_radar1) == HAL_OK) {
                    if (IFI_radar1 <= 0x3FC && IFQ_radar1 <= 0x3FC) {
                        active_buffer_radar1[2 * acquired_sample_count_radar1] =  (float32_t)(IFI_radar1 >> 2) / 255.0f;
                        active_buffer_radar1[2 * acquired_sample_count_radar1 + 1] = (float32_t)(IFQ_radar1 >> 2) / 255.0f;
                        acquired_sample_count_radar1++;
                    }
                } else {
                    error_cnt++;
                }
            } else {
                // Buffer full, swap buffers
                float32_t *temp = active_buffer_radar1;
                active_buffer_radar1 = processing_buffer_radar1;
                processing_buffer_radar1 = temp;
                data_ready_f_radar1 = 1;
                acquired_sample_count_radar1 = 0;
            }
        } else {
            // Process Radar 2
            if (acquired_sample_count_radar2 < FFT_BUFFER_SIZE) {
                if (bgt60ltr11_get_RAW_data(RADAR_2, &IFI_radar2, &IFQ_radar2) == HAL_OK) {
                    if (IFI_radar2 <= 0x3FC && IFQ_radar2 <= 0x3FC) {
                        active_buffer_radar2[2 * acquired_sample_count_radar2] = (float32_t)(IFI_radar2 >> 2) / 255.0f;
                        active_buffer_radar2[2 * acquired_sample_count_radar2 + 1] = (float32_t)(IFQ_radar2 >> 2) / 255.0f;
                        acquired_sample_count_radar2++;
                    }
                } else {
                    error_cnt++;
                }
            } else {
                // Buffer full, swap buffers
                float32_t *temp = active_buffer_radar2;
                active_buffer_radar2 = processing_buffer_radar2;
                processing_buffer_radar2 = temp;
                data_ready_f_radar2 = 1;
                acquired_sample_count_radar2 = 0;
            }
        }

        // Toggle between radars
        current_radar = (current_radar == RADAR_1) ? RADAR_2 : RADAR_1;
    }
}



void sendDataToMonitor(float32_t vel) {
	printf("DATA,%.5f\n", vel);
}

float32_t calculate_rolling_average_with_filtering1(float32_t new_value) {
    // Set threshold for outlier detection (adjust based on your expected velocity range)
    float32_t max_deviation = 20.0f; // Maximum allowed deviation from current average

    // If we have a previous average and the new value deviates too much, reject it
    if (velocity_buffer_filled_radar1 && fabsf(new_value - velocity_average_radar1) > max_deviation) {
        // Outlier detected, don't add to buffer
        //printf("Outlier rejected: %.5f (current avg: %.5f)\r\n", new_value, velocity_average_radar1);
        return velocity_average_radar1; // Return previous average
    }

    // Regular rolling average calculation
    velocity_buffer_radar1[velocity_buffer_index_radar1] = new_value;
    velocity_buffer_index_radar1 = (velocity_buffer_index_radar1 + 1) % VELOCITY_BUFFER_SIZE;

    if (velocity_buffer_index_radar1 == 0 && velocity_buffer_filled_radar1 == 0) {
        velocity_buffer_filled_radar1 = 1;
    }

    float32_t sum = 0.0f;
    uint8_t count = velocity_buffer_filled_radar1 ? VELOCITY_BUFFER_SIZE : velocity_buffer_index_radar1;

    if (count == 0) {
        return new_value;
    }

    for (uint8_t i = 0; i < count; i++) {
        sum += velocity_buffer_radar1[i];
    }

    return sum / count;
}


float32_t calculate_rolling_average_with_filtering2(float32_t new_value) {
    // Set threshold for outlier detection (adjust based on your expected velocity range)
    float32_t max_deviation = 20.0f; // Maximum allowed deviation from current average

    // If we have a previous average and the new value deviates too much, reject it
    if (velocity_buffer_filled_radar2 && fabsf(new_value - velocity_average_radar2) > max_deviation) {
        // Outlier detected, don't add to buffer
        //printf("Outlier rejected: %.5f (current avg: %.5f)\r\n", new_value, velocity_average_radar2);
        return velocity_average_radar2; // Return previous average
    }

    // Regular rolling average calculation
    velocity_buffer_radar2[velocity_buffer_index_radar2] = new_value;
    velocity_buffer_index_radar2 = (velocity_buffer_index_radar2 + 1) % VELOCITY_BUFFER_SIZE;

    if (velocity_buffer_index_radar2 == 0 && velocity_buffer_filled_radar2 == 0) {
        velocity_buffer_filled_radar2 = 1;
    }

    float32_t sum = 0.0f;
    uint8_t count = velocity_buffer_filled_radar2 ? VELOCITY_BUFFER_SIZE : velocity_buffer_index_radar2;

    if (count == 0) {
        return new_value;
    }

    for (uint8_t i = 0; i < count; i++) {
        sum += velocity_buffer_radar2[i];
    }

    return sum / count;
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
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	  HAL_Delay(50);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	  HAL_Delay(50);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	  HAL_Delay(50);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	  HAL_Delay(500);  // Longer pause between double-blinks
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
