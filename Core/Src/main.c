/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "TM1637.h"
#include "74hc165.h"
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define HC165_MY_NUM_CHIPS   3
#define HC165_MY_TOTAL_PINS  (HC165_MY_NUM_CHIPS * 8)

// --- Variabel untuk Input Capture ---
volatile uint32_t timer_overflow = 0;
volatile uint32_t last_capture = 0;
volatile uint32_t rpm = 0;
volatile uint32_t last_capture_time = 0;

// --- Variabel untuk display & UART ---
uint32_t last_display_tick = 0;
#define DISPLAY_UPDATE_MS  100

// --- Variabel untuk ADC dan PWM ---
uint32_t adc_raw = 0;           // nilai ADC mentah 0-4095
uint32_t adc_mv = 0;            // nilai dalam milivolt
uint32_t pwm_duty = 0;          // duty cycle saat ini (0-4199)

/*
 * ===== KONFIGURASI PWM UNTUK BLDC_ANALOG =====
 * - Timer: TIM5, Channel 2 (PA1)
 * - Frekuensi PWM: 20 kHz (Period = 4199, Prescaler = 0)
 * - Rangkaian eksternal:
 *   1) RC Low-Pass Filter: R = 1kΩ, C = 2.2µF → cut-off ≈ 72 Hz
 *   2) Opamp non-inverting (MCP6002, supply 5V):
 *      Rf = 6.8 kΩ, Rg = 10 kΩ → Gain = 1 + 6.8/10 = 1.68
 * - Output opamp = 0–5V analog untuk driver BLDC.
 * - Duty cycle maksimum dibatasi agar output ≤ 5V:
 *   Duty_max = 5 / (3.3 * 1.68) ≈ 0.902 → 4199 * 0.902 ≈ 3788
 */
#define PWM_MAX_DUTY  3788
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(char *msg);
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
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // --- 1. Inisialisasi 74HC165 ---
  HC165_SetPins(SHIFT_PL_GPIO_Port, SHIFT_PL_Pin,
                SHIFT_CP_GPIO_Port, SHIFT_CP_Pin,
                SHIFT_Q7_GPIO_Port, SHIFT_Q7_Pin);

  // --- 2. Inisialisasi TM1637 ---
  TM1637_SetBrightness(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin,
                       DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, 4);
  TM1637_DisplayNumber(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin,
                       DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, 0, 0);

  // --- 3. Inisialisasi PB8 sebagai TIM4_CH3 (manual, jika CubeMX belum generate) ---
  // Baris ini akan memastikan PB8 berfungsi sebagai input capture meskipun
  // MX_GPIO_Init() belum mengaturnya.
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = BLDC_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;   // AF2 untuk TIM4
  HAL_GPIO_Init(BLDC_PULSE_GPIO_Port, &GPIO_InitStruct);

  // --- 4. Aktifkan Input Capture dan overflow interrupt ---
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

  // --- 5. Aktifkan PWM output (BLDC_ANALOG) ---
  // Pin PA1 (TIM5_CH2) menghasilkan PWM 20 kHz
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0); // awal mati

  // --- 6. Mulai ADC ---
  HAL_ADC_Start(&hadc1);

  last_display_tick = HAL_GetTick();
  last_capture_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // --- Baca tombol 74HC165 ---
    uint8_t status_tombol[HC165_MY_NUM_CHIPS];
    uint8_t pins[HC165_MY_TOTAL_PINS];
    HC165_Read(status_tombol, HC165_MY_NUM_CHIPS);
    HC165_Unpack(status_tombol, pins, HC165_MY_NUM_CHIPS);

    if (pins[0]) {
      HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
    }

    // --- Update display & UART setiap 100ms ---
    if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
      // Jika tidak ada pulsa selama 500 ms, set RPM = 0
      if (HAL_GetTick() - last_capture_time > 500) {
        rpm = 0;
      }

      // Tampilkan RPM di display
      TM1637_DisplayNumber(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin,
                           DISP_DIO1_GPIO_Port, DISP_DIO1_Pin, rpm, 0);

      // Kirim data RPM, ADC, dan PWM duty via UART (debug)
      char uart_buf[80];
      sprintf(uart_buf, "RPM:%lu ADC:%lu(%lumV) PWM:%lu\r\n",
              (unsigned long)rpm, adc_raw, adc_mv, pwm_duty);
      UART_Print(uart_buf);

      last_display_tick = HAL_GetTick();
    }

    // --- Baca potensiometer dan update PWM setiap 50ms ---
    static uint32_t last_adc_tick = 0;
    if (HAL_GetTick() - last_adc_tick >= 50) {
      // Mulai konversi ADC (single conversion)
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_raw = HAL_ADC_GetValue(&hadc1);
        // Konversi ke mV (referensi 3.3V)
        adc_mv = (adc_raw * 3300) / 4095;
      }

      // --- Map ADC ke duty cycle dengan deadband dan batas maksimum ---
      // Deadband bawah: jika ADC < 20, motor mati total
      if (adc_raw < 20) {
        pwm_duty = 0;
      } else {
        // Mapping linear 0-4095 -> 0-4199
        pwm_duty = (adc_raw * 4200) / 4096;
        if (pwm_duty > 4199) pwm_duty = 4199;
      }

      // Batasi duty cycle agar output opamp tidak melebihi 5V
      if (pwm_duty > PWM_MAX_DUTY) {
        pwm_duty = PWM_MAX_DUTY;
      }

      // Set duty cycle ke timer (TIM5 Channel 2)
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm_duty);

      last_adc_tick = HAL_GetTick();
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim5);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SHIFT_PL_Pin|SHIFT_CP_Pin|MOTOR_EL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_DIO1_Pin|DISP_DIO2_Pin|DISP_DIO3_Pin|DISP_DIO7_Pin
                          |DISP_DIO5_Pin|DISP_DIO6_Pin|DISP_DIO4_Pin|FLASH_CS_Pin
                          |TM1637_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MCU_LED_Pin */
  GPIO_InitStruct.Pin = MCU_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHIFT_PL_Pin SHIFT_CP_Pin MOTOR_EL_Pin */
  GPIO_InitStruct.Pin = SHIFT_PL_Pin|SHIFT_CP_Pin|MOTOR_EL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_DIO1_Pin DISP_DIO2_Pin DISP_DIO3_Pin DISP_DIO7_Pin
                           DISP_DIO5_Pin DISP_DIO6_Pin DISP_DIO4_Pin FLASH_CS_Pin
                           TM1637_CLK_Pin */
  GPIO_InitStruct.Pin = DISP_DIO1_Pin|DISP_DIO2_Pin|DISP_DIO3_Pin|DISP_DIO7_Pin
                          |DISP_DIO5_Pin|DISP_DIO6_Pin|DISP_DIO4_Pin|FLASH_CS_Pin
                          |TM1637_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_VBUS_SENSE_Pin SHIFT_Q7_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin|SHIFT_Q7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Inisialisasi ulang pin TM1637 agar idle aman
  GPIO_InitStruct.Pin = ALL_DIO_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TM1637_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TM1637_CLK_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(TM1637_CLK_GPIO_Port, TM1637_CLK_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// ==================== CALLBACK INPUT CAPTURE ====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        uint32_t current = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t delta;

        // Hitung selisih dengan wrap-around 16-bit
        if (current > last_capture) {
            delta = current - last_capture;
        } else {
            delta = (0xFFFF - last_capture) + current + 1;
        }
        // Tambahkan overflow yang terjadi selama periode ini
        delta += timer_overflow * 65536UL;
        timer_overflow = 0;   // reset

        if (delta > 0) {
            // Timer clock = 10 kHz → 1 tick = 100 µs
            // RPM = 60 / (6 * delta * 0.0001) = 100000 / delta
            rpm = 100000 / delta;
        } else {
            rpm = 0;
        }

        last_capture = current;
        last_capture_time = HAL_GetTick();  // perbarui timestamp
    }
}

// ==================== CALLBACK OVERFLOW TIMER ====================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        timer_overflow++;
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
