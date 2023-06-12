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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VREFINT_CAL_ADDR          ((uint16_t*) ((uint32_t) 0x1FF80078))
#define MIN_VOLTAGE               5000
#define MAX_VOLTAGE               22000
#define MIN_CURRENT               200
#define SLEEP_VOLTAGE             1000
#define CHARGE_VOLTAGE_THESHOLD   2600

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// *** Load switch functions *** //
void load_switch_enable(void);
void load_switch_disable(void);

// *** Blocking led functions ***//
void led_blink(uint16_t time_delay_1, uint16_t time_delay_2);

// *** SAFELY ENABLE/DISABLE BUCK CONVERTER *** //
void Enable_buck_converter(void);
void Disable_buck_converter(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t ADC_CH5, ADC_CH6, ADC_CH7, ADC_REF;
uint16_t supply_voltage_mv = 0;
uint16_t input_voltage_mv = 0;
uint16_t buck_output_voltage_mv = 0;
uint16_t buck_current_ma = 0;
uint8_t previous_state = 0;
uint16_t timer_check = 0;

// State machine
typedef enum {INIT, START_CHARGING, CHARGING, STOP_CHARGING, OVER_VOLTAGE, SLEEP} State_type;
State_type current_state = INIT;

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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // Start ADC Conversion and read two ADC values
		HAL_ADC_Start(&hadc);
    
    // ADC_CHANNEL_5 & Read The ADC Conversion Result
    HAL_ADC_PollForConversion(&hadc, 100);
		ADC_CH5 = HAL_ADC_GetValue(&hadc);

    // ADC_CHANNEL_6 & Read The ADC Conversion Result
    HAL_ADC_PollForConversion(&hadc, 100);
		ADC_CH6 = HAL_ADC_GetValue(&hadc);

    // ADC_CHANNEL_7 & Read The ADC Conversion Result
    HAL_ADC_PollForConversion(&hadc, 100);
		ADC_CH7 = HAL_ADC_GetValue(&hadc);

    // ADC_CHANNEL_VREFINT & Read The ADC Conversion Result
    HAL_ADC_PollForConversion(&hadc, 100);
		ADC_REF = HAL_ADC_GetValue(&hadc);

    // Stop ADC Conversion
    HAL_ADC_Stop(&hadc);

    // Calculate supply voltage MCU
    supply_voltage_mv = 1200*4096/ADC_REF;

    // Calculate input voltage BUCK converter
    input_voltage_mv = ADC_CH5*supply_voltage_mv/4096*224700/4700;

    // Calculate buck converter output voltage
    buck_output_voltage_mv = ADC_CH7*supply_voltage_mv/4096*71000/15000;

    // Calculate buck converter output current
    buck_current_ma = ADC_CH6*supply_voltage_mv/4096*1000/780;


    if(input_voltage_mv > MAX_VOLTAGE){
      current_state = OVER_VOLTAGE;
    }

    
    ///////////////////////////
    // *** State machine *** //
    ///////////////////////////

    switch (current_state){
      case INIT:
        // No charging may occur in the INIT phase
        Disable_buck_converter();

        // Check input voltage before start charging
        if(input_voltage_mv < MAX_VOLTAGE && input_voltage_mv > MIN_VOLTAGE){
          current_state = START_CHARGING;
          break;
        }

        // Check input voltage in not below minimum voltage
        if(input_voltage_mv < SLEEP_VOLTAGE){
          current_state = SLEEP;
        }

        // Blink led
        led_blink(500, 500);
      break;
      case START_CHARGING: 
        // Enable buck converter safely
        Enable_buck_converter();

        // Change state
        current_state = CHARGING;
      break;
      case CHARGING:
        led_blink(900, 100);

        // If under voltage occurs, wait for enough voltage to resume
        if(input_voltage_mv < MIN_VOLTAGE){
          current_state = INIT;
          break;
        }

        // Check for current lower then MIN_CURRENT after 10 sec
        if(buck_current_ma < MIN_CURRENT && timer_check > 10 && supply_voltage_mv > CHARGE_VOLTAGE_THESHOLD){
          timer_check = 0;
          current_state = STOP_CHARGING;
        }

        timer_check++;

      break;
      case STOP_CHARGING:
        // Disable buck conveter safely
        Disable_buck_converter();

        if(buck_current_ma < MIN_CURRENT){
          current_state = SLEEP;
        }
        else{
          current_state = INIT;
        }
      break;
      case OVER_VOLTAGE:
        led_blink(100, 100);

        // Disable buck conveter safely
        Disable_buck_converter();

        // Check for save voltage
        if(input_voltage_mv < MAX_VOLTAGE){
          current_state = INIT;
        }
      break;
      case SLEEP:

        // Disable LED
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

        // Check if input voltage is above 2 V and enable timer interrupt

        // Reconfigure GPIO pins for sleep mode: EN1_Pin EN2_Pin PG_Pin BUCK_EN_Pin
        // GPIO_InitTypeDef GPIO_InitStruct;
        // GPIO_InitStruct.Pin = EN1_Pin|EN2_Pin|PG_Pin|BUCK_EN_Pin;
        // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        // GPIO_InitStruct.Pull = GPIO_NOPULL;
        // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        // Disable the systick interrupt
        HAL_SuspendTick();

        // Enter Stop Mode
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

        // Change state
        current_state = INIT;

      break;
      default:
      current_state = INIT;
      break;
    }

    uint8_t length = 10;
    uint8_t buffer [length];
    buffer [0] = 0x02;
    buffer [1] = length;
    buffer [2] = (input_voltage_mv >> 8);
    buffer [3] = (uint8_t)(input_voltage_mv);
    buffer [4] = (buck_output_voltage_mv >> 8);
    buffer [5] = (uint8_t)(buck_output_voltage_mv);
    buffer [6] = (buck_current_ma >> 8);
    buffer [7] = (uint8_t)(buck_current_ma);
    buffer [8] = (supply_voltage_mv >> 8);
    buffer [9] = (uint8_t)(supply_voltage_mv);
    HAL_UART_Transmit(&huart2, buffer, length, 100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, EN1_Pin|EN2_Pin|PG_Pin|BUCK_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN1_Pin EN2_Pin PG_Pin BUCK_EN_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|EN2_Pin|PG_Pin|BUCK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// *** Load Switch functions *** //
// Enable load switch 1 and 2
void load_switch_enable(void){
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
}

// Disable load switch 1 and 2
void load_switch_disable(void){
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
}

void led_blink(uint16_t time_delay_1, uint16_t time_delay_2){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(time_delay_1);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(time_delay_2);
}

// *** WAKE UP *** //
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //if(GPIO_Pin == INT_Pin)
  //{
	  SystemClock_Config();
	  HAL_ResumeTick();
  //}
}

// *** SAFELY ENABLE/DISABLE BUCK CONVERTER *** //
void Enable_buck_converter(void){
  // Enable load switches
  load_switch_enable();

  HAL_Delay(100);

  // Enable CC/CV buck converter
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_SET);

  // Connect input voltage to buck converter
  HAL_GPIO_WritePin(PG_GPIO_Port, PG_Pin, GPIO_PIN_SET);
}


void Disable_buck_converter(void){
    // Disable CC/CV buck converter
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_RESET);
  
  // Disconnect input voltage from buck converter
  HAL_GPIO_WritePin(PG_GPIO_Port, PG_Pin, GPIO_PIN_RESET);

  // Disable load switches
  load_switch_disable();
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
