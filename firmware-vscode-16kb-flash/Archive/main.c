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
#define MIN_CHARGE_CUTTOF_VOLTAGE 2600
#define PID_SETPOINT              10000

// PID Parameters
#define PID_UNDER_THRESHOLD       0
#define PID_UPPER_THRESHOLD       30 // = 50/255*50kOhm approx 10 kOhm (50kohm digital potentiometer)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// *** Load switch functions *** //
void load_switch_enable(void);
void load_switch_disable(void);

// *** Blocking led functions *** //
void led_blink(uint16_t time_delay_1, uint16_t time_delay_2);

// *** Non blocking led functios *** //
void led_blink_nb(uint16_t time_delay_1, uint16_t time_delay_2);

// *** SAFELY ENABLE/DISABLE BUCK CONVERTER *** //
void Enable_buck_converter(void);
void Disable_buck_converter(void);

// *** Calculate CRC *** //
uint8_t calculate_crc_xor(const uint8_t *data, size_t length);

// *** PID controller *** //
uint16_t pid_controller(uint16_t measured_value);

// *** Change resistance digital potentiometer *** //
void change_potentiometer(uint8_t new_value);

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

// Non blocking blink  
uint8_t isOn;
uint32_t timestamp;

// Send message
uint8_t send_status = 1;

// PID result
uint16_t pid_result;
uint16_t pid_setpoint = 0;
uint8_t check = 0;
int32_t control_signal = PID_UPPER_THRESHOLD;
int32_t prev_control_signal = PID_UPPER_THRESHOLD;

int32_t integral = PID_UPPER_THRESHOLD*1000;     // Integral term accumulator
// uint32_t integral_2 = 0;     // Integral term accumulator
int32_t prev_error = 0;   // Previous error for derivative term
uint8_t last_stable_PID_value;
uint16_t last_stable_voltage;

// Buffer keeping SPI commands to change resistance of programmable potentiometer 
uint8_t spi_buffer [2] = {0, 0};

// State machine
typedef enum {INIT, START_CHARGING, CHARGING_PID, CHARGING_NO_PID, STOP_CHARGING, OVER_VOLTAGE, SLEEP} State_type;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);

  // Start up completed (small blink)
  led_blink(500, 500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Delay (prevent sampling ADC to quickly)
    HAL_Delay(100);

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

    // Check input voltage in not below minimum voltage
    if(input_voltage_mv < SLEEP_VOLTAGE){
      current_state = SLEEP;
    }

    switch (current_state){
      case INIT:
        // No charging may occur in the INIT phase
        Disable_buck_converter();

        // Reset PID result
        pid_result = PID_UPPER_THRESHOLD;
        control_signal = PID_UPPER_THRESHOLD;
        prev_control_signal = PID_UPPER_THRESHOLD;
        integral = PID_UPPER_THRESHOLD*1000;
        prev_error = 0;

        // Check input voltage before start charging
        if(input_voltage_mv < MAX_VOLTAGE && input_voltage_mv > MIN_VOLTAGE){
          current_state = START_CHARGING;
          break;
        }

        // Blink led
        led_blink_nb(500, 500);
      break;
      case START_CHARGING: 
        // Enable buck converter safely
        Enable_buck_converter();

        // Change state
        current_state = CHARGING_PID;
      break;
      case CHARGING_PID:
        // Blink led
        led_blink_nb(100, 900);
        
        // Check input voltage with PID controller
        if(check == 5){
          pid_result = pid_controller(input_voltage_mv);
          check = 0;
        }
        check++;

        // Compensate programmable potentiometer
        change_potentiometer(pid_result);

        // If under voltage occurs, wait for enough voltage to resume
        if(input_voltage_mv < MIN_VOLTAGE){
          current_state = INIT;
          
          // // Not enough power at the receiver - keep last stable PID value
          // last_stable_PID_value = pid_result + 2;
          // last_stable_voltage = input_voltage_mv;

          // // Compensate programmable potentiometer
          // change_potentiometer(last_stable_PID_value);

          // // No PID
          // current_state = CHARGING_NO_PID;

          break;
        }

        // Check for current lower then MIN_CURRENT after 10 sec
        if(buck_current_ma < MIN_CURRENT && timer_check > 100 && supply_voltage_mv > MIN_CHARGE_CUTTOF_VOLTAGE){
          timer_check = 0;
          current_state = STOP_CHARGING;
        }

        timer_check++;

      break;
      // case CHARGING_NO_PID:

      //   // Blink led
      //   led_blink_nb(100, 900);

      //   int16_t num = last_stable_voltage - input_voltage_mv;
      //   if(num < 0){
      //     num = -num;
      //   }
      //   if(num > 1000){
      //     current_state = INIT;
      //   }

      // break;
      case STOP_CHARGING:
        // Disable buck conveter safely
        Disable_buck_converter();

        // Reset PID result
        pid_result = PID_UPPER_THRESHOLD;
        control_signal = PID_UPPER_THRESHOLD;
        prev_control_signal = PID_UPPER_THRESHOLD;

        // SEND MESSAGE TO TRANSMITTER

        // if(buck_current_ma < MIN_CURRENT && supply_voltage_mv > MIN_CHARGE_CUTTOF_VOLTAGE){
        //   current_state = SLEEP;
        // }
        // else{
        //   current_state = INIT;
        // }
        current_state = SLEEP;
      break;
      case OVER_VOLTAGE:
        led_blink_nb(100, 100);

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

    if(send_status){
      send_status = 0;
      uint8_t length = 12;
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
      buffer [10] = pid_result;
      buffer [11] = calculate_crc_xor(buffer, length-1);//0xFF;
      HAL_UART_Transmit(&huart2, buffer, length, 100);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
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
  htim2.Init.Prescaler = 2000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN1_Pin|EN2_Pin|PG_Pin|BUCK_EN_Pin
                          |SC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Pin EN2_Pin PG_Pin BUCK_EN_Pin
                           SC_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|EN2_Pin|PG_Pin|BUCK_EN_Pin
                          |SC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

void led_blink_nb(uint16_t time_delay_1, uint16_t time_delay_2){
  uint32_t currentTime = HAL_GetTick();  // Get current timestamp (implementation-dependent)

  if (isOn) {
    if (currentTime - timestamp >= time_delay_1) {
      isOn = 0;
      timestamp = currentTime;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
  } else {
    if (currentTime - timestamp >= time_delay_2) {
      isOn = 1;
      timestamp = currentTime;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
  }
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

// *** TIMER INTEGRATION *** //
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
  send_status = 1;
}


// *** Calculate CRC *** //
uint8_t calculate_crc_xor(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    size_t i;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
    }

    return crc;
}

// PID Controller Variables
// int32_t integral = PID_UPPER_THRESHOLD*1000;     // Integral term accumulator
// // uint32_t integral_2 = 0;     // Integral term accumulator
// int32_t prev_error = 0;   // Previous error for derivative term
// uint16_t prev_error_2 = 0; 

uint16_t kp = 5;  // Proportional gain (Scaled by 100)
uint16_t ki = 100;  // Integral gain (Scaled by 100)
uint16_t kd = 10;  // Derivative gain (Scaled by 100)


// *** PID controller *** //
// uint16_t pid_controller(uint16_t measured_value){
//   if(measured_value < PID_SETPOINT){
//     uint16_t error = (PID_SETPOINT - measured_value)/1000;
//     if(error == 0)  { }//control_signal++;                             }
//     else            { control_signal = control_signal + error; }
//     if(control_signal > PID_UPPER_THRESHOLD){
//       control_signal = PID_UPPER_THRESHOLD;
//     }
//   }
//   if(measured_value > PID_SETPOINT){
//     uint8_t error = (measured_value - PID_SETPOINT)/1000;
//     if(control_signal > error + PID_UNDER_THRESHOLD){
//       if(error == 0)  { }//control_signal--;                         }
//       else            { control_signal = control_signal - error;  }
//     }
//     else{
//       control_signal = PID_UNDER_THRESHOLD;
//     }
//   }
uint16_t pid_controller(uint16_t measured_value){
  // if(measured_value < PID_SETPOINT){
  //   uint16_t error = (PID_SETPOINT - measured_value);
  //   prev_error_1 = error;
  //   if(error == 0)  { }//control_signal++;                             }
  //   else            { 
  //     // Proportional term
  //     int p_term = (kp * error) / 1000;

  //     // Integral term
  //     integral_1 += (ki * error) / 1000;

  //     // Derivative term
  //     int d_term = (kd * (error - prev_error_1)) / 1000;
  //     control_signal = p_term + integral_1 + d_term;
  //   }
  //   if(control_signal > PID_UPPER_THRESHOLD){
  //     control_signal = PID_UPPER_THRESHOLD;
  //   }
  // }
  // if(measured_value > PID_SETPOINT){
  //   uint16_t error = (measured_value - PID_SETPOINT);
  //   prev_error_2 = error;
  //   if(control_signal > error + PID_UNDER_THRESHOLD){
  //     if(error == 0)  { }//control_signal--;                         }
  //     else            { 
  //       // Proportional term
  //       int p_term = (kp * error) / 1000;

  //       // Integral term
  //       integral_2 += (ki * error) / 1000;

  //       // Derivative term
  //       int d_term = (kd * (error - prev_error_1)) / 1000;
  //       control_signal = p_term + integral_2 + d_term;  
  //     }
  //   }
  //   else{
  //     control_signal = PID_UNDER_THRESHOLD;
  //   }
  // }
  // return control_signal;

// V2
  // if(measured_value < PID_SETPOINT){
  //   pid_setpoint = measured_value + 500;
  // }
  // else{
  //   pid_setpoint = PID_SETPOINT;
  // }

  // int32_t error = pid_setpoint - measured_value;

  // // Proportional term
  // int32_t p_term = (kp * error) / 100;

  // // Integral term
  // integral += (ki * error) / 100;

  // // Derivative term
  // int32_t d_term = (kd * (error - prev_error)) / 100;
  // prev_error = error;

  // // Calculate the control signal
  // control_signal = (p_term + integral + d_term)/1000;

  // // Limit the control output to a maximum of 256
  // // control_signal = limit_max(control_signal, 50);

  // // if(control_signal < PID_UNDER_THRESHOLD){
  // //   control_signal = PID_UNDER_THRESHOLD;
  // //   integral = PID_UNDER_THRESHOLD*1000;
  // // }
  // if(control_signal > PID_UPPER_THRESHOLD){
  //   control_signal = PID_UPPER_THRESHOLD;
  //   integral = PID_UPPER_THRESHOLD*1000;
  // }
  // prev_control_signal = control_signal;

  // if(control_signal < prev_control_signal){
  //   control_signal--;
  // }
  // else{
  //   control_signal++;
  // }

  // V3

  if(measured_value > PID_SETPOINT){
    control_signal--;
  }
  else{
    uint16_t res = (PID_SETPOINT - measured_value)/1000;
    control_signal = control_signal + res;
  }

  if(control_signal > PID_UPPER_THRESHOLD){control_signal = PID_UPPER_THRESHOLD;}
  if(control_signal < PID_UNDER_THRESHOLD){control_signal = PID_UNDER_THRESHOLD;}

  return control_signal;

}

// *** Change resistance digital potentiometer *** //
void change_potentiometer(uint8_t new_value){
  spi_buffer[1] = new_value;
  HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spi_buffer, 2, 200);
  HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_SET);
}

/* USER CODE E
ND 4 */

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
