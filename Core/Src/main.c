/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Rotarod + 7 display + Flash W25Q128 + RTC DS3231
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
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
#include "w25q64.h"
#include "ds3231.h"
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

volatile uint32_t timer_overflow = 0;
volatile uint32_t last_capture = 0;
volatile uint32_t rpm = 0;
volatile uint32_t last_capture_time = 0;

uint32_t last_display_tick = 0;
#define DISPLAY_UPDATE_MS  100

uint32_t adc_raw = 0;
uint32_t pwm_duty = 0;
uint32_t set_value = 0;
#define PWM_MAX_DUTY  3788

uint32_t last_button_read_tick = 0;
#define BUTTON_READ_MS  20

uint8_t status_tombol[HC165_MY_NUM_CHIPS];
uint8_t pins[HC165_MY_TOTAL_PINS];

// ====== 7 Display ======
#define NUM_DISPLAYS  7
static GPIO_TypeDef* clk_ports[NUM_DISPLAYS] = {
    DISP_CLK1_GPIO_Port,   // PC13
    DISP_CLK2_GPIO_Port,   // PB3
    DISP_CLK3_GPIO_Port,   // PB14
    DISP_CLK4_GPIO_Port,   // PB15
    DISP_CLK5_GPIO_Port,   // PC14
    DISP_CLK6_GPIO_Port,   // PC15
    DISP_CLK7_GPIO_Port    // PB9
};
static uint16_t clk_pins[NUM_DISPLAYS] = {
    DISP_CLK1_Pin,
    DISP_CLK2_Pin,
    DISP_CLK3_Pin,
    DISP_CLK4_Pin,
    DISP_CLK5_Pin,
    DISP_CLK6_Pin,
    DISP_CLK7_Pin
};
static GPIO_TypeDef* dio_ports[NUM_DISPLAYS] = {
    DISP_DIO1_GPIO_Port,    // PB0
    DISP_DIO2_GPIO_Port,    // PB1
    DISP_DIO3_GPIO_Port,    // PB2
    DISP_DIO4_GPIO_Port,    // PB4
    DISP_DIO5_GPIO_Port,    // PB12
    DISP_DIO6_GPIO_Port,    // PB13
    DISP_DIO7_GPIO_Port     // PB10
};
static uint16_t dio_pins[NUM_DISPLAYS] = {
    DISP_DIO1_Pin,
    DISP_DIO2_Pin,
    DISP_DIO3_Pin,
    DISP_DIO4_Pin,
    DISP_DIO5_Pin,
    DISP_DIO6_Pin,
    DISP_DIO7_Pin
};

static void Display_ShowNumber(uint8_t idx, uint16_t num, uint8_t dots) {
    if (idx >= NUM_DISPLAYS) return;
    TM1637_DisplayNumber(clk_ports[idx], clk_pins[idx],
                         dio_ports[idx], dio_pins[idx],
                         num, dots);
}
static void Display_SetBrightness(uint8_t idx, uint8_t brightness) {
    if (idx >= NUM_DISPLAYS) return;
    TM1637_SetBrightness(clk_ports[idx], clk_pins[idx],
                         dio_ports[idx], dio_pins[idx],
                         brightness);
}
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  // --- 74HC165 ---
  HC165_SetPins(SHIFT_PL_GPIO_Port, SHIFT_PL_Pin,
                SHIFT_CP_GPIO_Port, SHIFT_CP_Pin,
                SHIFT_Q7_GPIO_Port, SHIFT_Q7_Pin);

  // --- CLK output idle HIGH ---
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = DISP_CLK2_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin;
  HAL_GPIO_Init(DISP_CLK2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, DISP_CLK2_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin, GPIO_PIN_SET);

  // --- Inisialisasi display ---
  for (uint8_t i = 0; i < NUM_DISPLAYS; i++) {
      Display_SetBrightness(i, 4);
  }
  Display_ShowNumber(0, 1111, 0);
  Display_ShowNumber(1, 2222, 0);
  Display_ShowNumber(2, 3333, 0);
  Display_ShowNumber(3, 4444, 0);
  Display_ShowNumber(4, 5555, 0);
  Display_ShowNumber(5, 6666, 0);
  Display_ShowNumber(6, 7777, 0);

  // --- PB8 TIM4_CH3 ---
  GPIO_InitStruct.Pin = BLDC_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(BLDC_PULSE_GPIO_Port, &GPIO_InitStruct);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

  // --- PWM ---
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

  // --- ADC ---
  HAL_ADC_Start(&hadc1);

  // --- SPI Flash ---
  W25Q64_Init();
  uint32_t flash_id = W25Q64_ReadID();
  char id_buf[80];

  uint8_t cap_byte = flash_id & 0xFF;
  uint32_t capacity_mb = 0;
  if (cap_byte == 0x17) capacity_mb = 8;
  else if (cap_byte == 0x18) capacity_mb = 16;
  else if (cap_byte == 0x19) capacity_mb = 32;
  else capacity_mb = 0;

  if (capacity_mb > 0) {
      sprintf(id_buf, "Flash ID: 0x%06lX (%lu MB)\r\n", flash_id, capacity_mb);
  } else {
      sprintf(id_buf, "Flash ID: 0x%06lX (unknown)\r\n", flash_id);
  }
  UART_Print(id_buf);

  if (flash_id == 0xEF4017) {
      UART_Print("W25Q64JV terdeteksi (8MB)\r\n");
  } else if (flash_id == 0xEF4018) {
      UART_Print("W25Q128JV terdeteksi (16MB)\r\n");
  } else if (flash_id == 0xEF4019) {
      UART_Print("W25Q256JV terdeteksi (32MB)\r\n");
  } else {
      UART_Print("ID tidak sesuai dengan Winbond 64/128/256\r\n");
  }

  // ========================== TEST FLASH ==========================
  UART_Print("=== Flash Test Start ===\r\n");

  uint8_t test_data[4] = {0x01, 0x02, 0x03, 0x04};
  uint8_t read_buf[4] = {0};
  char result_buf[50];

  // 1. Erase sector di alamat 0x0000
  UART_Print("Erasing sector 0x0000...\r\n");
  W25Q64_SectorErase(0x0000);
  UART_Print("Erase done.\r\n");

  // 2. Tulis data
  UART_Print("Writing test data...\r\n");
  W25Q64_PageProgram(0x0000, test_data, 4);
  UART_Print("Write done.\r\n");

  // 3. Baca kembali
  UART_Print("Reading back...\r\n");
  W25Q64_ReadData(0x0000, read_buf, 4);
  sprintf(result_buf, "Read: %02X %02X %02X %02X\r\n",
          read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
  UART_Print(result_buf);

  // 4. Verifikasi
  if (memcmp(test_data, read_buf, 4) == 0) {
      UART_Print("Test PASSED: Write/Read OK\r\n");
  } else {
      UART_Print("Test FAILED: Data mismatch!\r\n");
  }

  // 5. Erase ulang, baca lagi (harus 0xFF)
  UART_Print("Erasing sector again...\r\n");
  W25Q64_SectorErase(0x0000);
  UART_Print("Reading after erase...\r\n");
  W25Q64_ReadData(0x0000, read_buf, 4);
  sprintf(result_buf, "After erase: %02X %02X %02X %02X\r\n",
          read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
  UART_Print(result_buf);

  if (read_buf[0] == 0xFF && read_buf[1] == 0xFF &&
      read_buf[2] == 0xFF && read_buf[3] == 0xFF) {
      UART_Print("Erase test PASSED (all 0xFF)\r\n");
  } else {
      UART_Print("Erase test FAILED!\r\n");
  }

  UART_Print("=== Flash Test End ===\r\n");
  // ===============================================================

  // --- RTC DS3231 ---
  DS3231_Init();

  // (Opsional) Set waktu jika diperlukan, contoh set 23:59:00, 20/07/2025 (Minggu)
  // DS3231_SetTime(0, 59, 23, 7, 20, 7, 25); // Hari: 1=Senin, 2=Selasa, ..., 7=Minggu

  last_display_tick = HAL_GetTick();
  last_capture_time = HAL_GetTick();
  last_button_read_tick = HAL_GetTick();

  UART_Print("System Start\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // --- Baca shift register setiap 20ms ---
    if (HAL_GetTick() - last_button_read_tick >= BUTTON_READ_MS) {
      HC165_Read(status_tombol, HC165_MY_NUM_CHIPS);
      HC165_Unpack(status_tombol, pins, HC165_MY_NUM_CHIPS);
      last_button_read_tick = HAL_GetTick();
    }

    // --- Update display & UART setiap 100ms ---
    if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
      if (HAL_GetTick() - last_capture_time > 500) {
        rpm = 0;
      }

      Display_ShowNumber(2, (uint16_t)rpm, 0); // RPM di display 3

      // Baca waktu RTC untuk timestamp
      uint8_t sec, min, hour, day, date, month, year;
      DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);

      // Debug: dengan timestamp RTC
      char uart_buf[250];
      sprintf(uart_buf,
              "[%02d:%02d:%02d %02d/%02d/%02d] RPM:%lu ADC:%lu PWM:%lu SET:%lu SR:%02X%02X%02X D0:%d %d %d\r\n",
              hour, min, sec, date, month, year,
              (unsigned long)rpm,
              adc_raw,
              pwm_duty,
              set_value,
              status_tombol[0], status_tombol[1], status_tombol[2],
              pins[0], pins[8], pins[16]);
      UART_Print(uart_buf);

      last_display_tick = HAL_GetTick();
    }

    // --- Baca potensiometer dan update PWM setiap 50ms ---
    static uint32_t last_adc_tick = 0;
    if (HAL_GetTick() - last_adc_tick >= 50) {
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_raw = HAL_ADC_GetValue(&hadc1);
        set_value = (adc_raw * 150) / 4095;
        if (set_value > 150) set_value = 150;
      }

      if (adc_raw < 20) {
        pwm_duty = 0;
      } else {
        pwm_duty = (adc_raw * 4200) / 4096;
        if (pwm_duty > 4199) pwm_duty = 4199;
      }
      if (pwm_duty > PWM_MAX_DUTY) pwm_duty = PWM_MAX_DUTY;

      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm_duty);
      last_adc_tick = HAL_GetTick();
    }

    // --- (Opsional) Cetak waktu RTC setiap 1 detik (sudah di debug di atas) ---
    // Jika ingin waktu terpisah, bisa ditambahkan di sini.

    /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// ==================== FUNGSI INISIALISASI (CubeMX) ====================

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
}

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
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim5);
}

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
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

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
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, SHIFT_PL_Pin | SHIFT_CP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO7_Pin
                          | DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin
                          | DISP_CLK2_Pin | DISP_DIO4_Pin | FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DISP_CLK7_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DISP_CLK2_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin | DISP_CLK7_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO4_Pin
                      | DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_DIO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SHIFT_PL_Pin | SHIFT_CP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin | SHIFT_Q7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = ALL_DIO_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        uint32_t current = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t delta;

        if (current > last_capture) {
            delta = current - last_capture;
        } else {
            delta = (0xFFFF - last_capture) + current + 1;
        }
        delta += timer_overflow * 65536UL;
        timer_overflow = 0;

        if (delta > 0) {
            rpm = 100000 / delta;
        } else {
            rpm = 0;
        }

        last_capture = current;
        last_capture_time = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        timer_overflow++;
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
