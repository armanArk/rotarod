/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Rotarod + FatFs on W25Q Flash + USB MSC
  *          - Partition 0 (0:): event log (ROTAROD.CSV)
  *          - Partition 1 (1:): export copy for Windows USB MSC
  *          - Auto unmount/mount on USB plug/unplug
  *          - Auto-reformat if FS size != half flash size
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "74hc165.h"
#include "TM1637.h"
#include "w25q64.h"
#include "fs_logger.h"
#include "ds3231.h"
#include "eeprom.h"
#include "cli.h"
#include "ui.h"
#include "motor_control.h"
#include "usbd_storage_if.h"
#include "settings.h"
#include "staging.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern void STORAGE_Invalidate(void);
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

/* USER CODE BEGIN PV */
#define USB_CHECK_MS        100
// Debounce for VBUS sense EXTI to avoid duplicate open events (ms)
#define VBUS_DEBOUNCE_MS    200

// USB
uint8_t usb_connected = 0;
static uint8_t last_usb_state = 0;
// VBUS EXTI event pending flag
volatile uint8_t vbus_event_pending = 0;

uint32_t last_usb_check = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Simple register-based IWDG initializer (no HAL dependency) */
static void Simple_IWDG_Init(void) {
    /* Enable LSI oscillator for IWDG clock */
    __HAL_RCC_LSI_ENABLE();
    uint32_t tickstart = HAL_GetTick();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) {
        if ((HAL_GetTick() - tickstart) > 500) break;
    }

    /* Enable write access to IWDG_PR and IWDG_RLR */
    IWDG->KR = 0x5555;
    /* Prescaler: 32 (PR = 3) */
    IWDG->PR = 3;
    /* Reload value: ~3000 (approx 3s with LSI ~32kHz) */
    IWDG->RLR = 3000;
    /* Wait for registers update with timeout */
    tickstart = HAL_GetTick();
    while (IWDG->SR != 0) {
        if ((HAL_GetTick() - tickstart) > 500) break;
    }
    /* Reload counter */
    IWDG->KR = 0xAAAA;
    /* Start the watchdog */
    IWDG->KR = 0xCCCC;
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
  // MX_ADC1_Init(); // ADC removed for Rotary Encoder
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
    // Disconnect DP pull-up initially until VBUS is detected (keeps USB core running but host won't enumerate yet)
    HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);

    // Enable UART RX Interrupt
    USART1->CR1 |= USART_CR1_RXNEIE;

    // 74HC165
    HC165_SetPins(SHIFT_PL_GPIO_Port, SHIFT_PL_Pin, SHIFT_CP_GPIO_Port, SHIFT_CP_Pin,
                  SHIFT_Q7_GPIO_Port, SHIFT_Q7_Pin);

    // CLK idle HIGH
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

    // Displays are initialized via UI_Init() later

    // TIM4 IC on PB8
    GPIO_InitStruct.Pin = BLDC_PULSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(BLDC_PULSE_GPIO_Port, &GPIO_InitStruct);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    // PWM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

    // ADC (Removed for Rotary Encoder)

    // SPI Flash init
    W25Q64_Init();
    uint8_t sr = W25Q64_ReadStatusRegister1();
    if (sr != 0x00) W25Q64_WriteStatusRegister1(0x00);

    uint32_t flash_id = W25Q64_ReadID();
    uint8_t cap_byte = flash_id & 0xFF;
    uint32_t flash_cap = (cap_byte == 0x17) ? 8*1024*1024 :
                           (cap_byte == 0x18) ? 16*1024*1024 :
                           (cap_byte == 0x19) ? 32*1024*1024 : 8*1024*1024;
    FS_SetFlashCapacity(flash_cap);
    char id_buf[80];
    sprintf(id_buf, "Flash ID: 0x%06lX (%lu MB)\r\n", flash_id, flash_cap/(1024*1024));
    UART_Print(id_buf);

    // Mount FatFs
    MountFS();
    if (!FS_IsMounted() && !FS_IsFormatted()) {
        UART_Print("Flash is completely empty. Performing one-time format...\r\n");
        FormatFS();
    }

    // RTC and EEPROM
    DS3231_Init();
    EEPROM_Init();

    // Initialize staging area for queued writes while USB attached
    if (staging_init() == 0) UART_Print("Staging initialized\r\n");

    // Independent watchdog: simple register-based init (avoids HAL IWDG dependency)
    Simple_IWDG_Init();

    // Initialize UI (Displays)
    UI_Init();
    // Logging init
    Log_Init();

    uint32_t last_debug_tick = HAL_GetTick();
    last_usb_check = HAL_GetTick();

    UART_Print("System Start\r\n");

    // Auto-load PID settings dari Flash saat startup
    {
        MotorSettings saved;
        if (Settings_Load(&saved)) {
            Motor_SetPID(saved.kp, saved.ki, saved.kd);
            UART_Print("[BOOT] PID settings loaded from Flash\r\n");
        } else {
            UART_Print("[BOOT] No saved PID settings, using defaults\r\n");
        }
    }

    typedef enum {
        USB_SM_IDLE = 0,
        USB_SM_CONN_START,
        USB_SM_CONN_EXPORT,
        USB_SM_CONN_WAIT,
        USB_SM_DISCONN_START,
        USB_SM_DISCONN_WAIT
    } UsbSmState_t;
    UsbSmState_t usb_sm_state = USB_SM_IDLE;
    uint32_t usb_sm_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // Process commands over FTDI UART (e.g., "FORMAT", "CHECKFS", "HELP")
        ProcessUartRxCommand();
        // Shift register and displays
        UI_Process();
        // ADC & PWM
        Motor_Process();

        // USB VBUS detection (handled by EXTI or periodic poll)
        // Kita boleh cek perubahan VBUS saat idle ATAU saat sedang tersambung
        if (usb_sm_state == USB_SM_IDLE || usb_sm_state == USB_SM_CONN_WAIT) {
            if (vbus_event_pending || (HAL_GetTick() - last_usb_check >= USB_CHECK_MS)) {
                uint8_t new_usb = HAL_GPIO_ReadPin(USB_VBUS_SENSE_GPIO_Port, USB_VBUS_SENSE_Pin);
                if (HAL_GetTick() < 2000) new_usb = 0; // Ignore VBUS during early boot
                
                if (new_usb && !last_usb_state) {
                    usb_sm_state = USB_SM_CONN_START;
                } else if (!new_usb && last_usb_state) {
                    usb_sm_state = USB_SM_DISCONN_START;
                }
                
                last_usb_state = new_usb;
                usb_connected = new_usb;
                last_usb_check = HAL_GetTick();
                vbus_event_pending = 0;
            }
        }

        // USB State Machine (Non-Blocking)
        switch (usb_sm_state) {
            case USB_SM_IDLE:
                break;
            case USB_SM_CONN_EXPORT:
                break;

            case USB_SM_CONN_START:
                UART_Print("USB connected - exposing flash\r\n");
                HAL_PCD_DevConnect(&hpcd_USB_OTG_FS);
                STORAGE_UpdateCapacity(FS_GetFlashCapacity());
                STORAGE_SetPartitionOffset(0); // Export entire flash
                STORAGE_SetMediaReady(1);
                usb_sm_state = USB_SM_CONN_WAIT;
                break;

            case USB_SM_CONN_WAIT:
                break;

            case USB_SM_DISCONN_START:
                UART_Print("USB disconnected - unmounting\r\n");
                HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
                STORAGE_SetMediaReady(0);
                usb_sm_tick = HAL_GetTick();
                usb_sm_state = USB_SM_DISCONN_WAIT;
                break;
                
            case USB_SM_DISCONN_WAIT:
                if (HAL_GetTick() - usb_sm_tick >= 100) { // Pengganti HAL_Delay(100)
                    STORAGE_Invalidate();
                    staging_commit(); 
                    usb_sm_state = USB_SM_IDLE;
                }
                break;
        }

        // Debug UART (100 ms)
        uint8_t hour, min, sec, day, date, month, year;
        if (HAL_GetTick() - last_debug_tick >= 1000) {
            DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);

            // DEBUG USB / FILESYSTEM
            char uart_buf[128];
            sprintf(uart_buf, "[%02d:%02d:%02d] USB FS: VBUS=%d, Mtd=%d, Fmt=%d, Events=%u, Cap=%lu MB\r\n",
                    hour, min, sec, 
                    usb_connected, FS_IsMounted(), FS_IsFormatted(), 
                    Log_GetEventCount(), 
                    (unsigned long)(FS_GetFlashCapacity() / 1024 / 1024));
            UART_Print(uart_buf);
            last_debug_tick = HAL_GetTick();
        }

        // Heartbeat every 500ms
        static uint32_t last_heartbeat = 0;
        if (HAL_GetTick() - last_heartbeat >= 500) {
            // minimal heartbeat removed
            STORAGE_Flush();
            if (Log_GetEventCount() > 0) {
                Log_FlushToCSV();
            }
            last_heartbeat = HAL_GetTick();
        }

        // Refresh independent watchdog (register-based)
        IWDG->KR = 0xAAAA;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
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
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin|DISP_CLK5_Pin|DISP_CLK6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SHIFT_PL_Pin|SHIFT_CP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_DIO1_Pin|DISP_DIO2_Pin|DISP_DIO3_Pin|DISP_DIO7_Pin
                          |DISP_DIO5_Pin|DISP_DIO6_Pin|DISP_CLK3_Pin|DISP_CLK4_Pin
                          |DISP_CLK2_Pin|DISP_DIO4_Pin|FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP_CLK1_Pin DISP_CLK5_Pin DISP_CLK6_Pin */
  GPIO_InitStruct.Pin = DISP_CLK1_Pin|DISP_CLK5_Pin|DISP_CLK6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHIFT_PL_Pin SHIFT_CP_Pin */
  GPIO_InitStruct.Pin = SHIFT_PL_Pin|SHIFT_CP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_DIO1_Pin DISP_DIO2_Pin DISP_DIO3_Pin DISP_DIO7_Pin
                           DISP_DIO5_Pin DISP_DIO6_Pin DISP_CLK3_Pin DISP_CLK4_Pin
                           DISP_CLK2_Pin DISP_DIO4_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = DISP_DIO1_Pin|DISP_DIO2_Pin|DISP_DIO3_Pin|DISP_DIO7_Pin
                          |DISP_DIO5_Pin|DISP_DIO6_Pin|DISP_CLK3_Pin|DISP_CLK4_Pin
                          |DISP_CLK2_Pin|DISP_DIO4_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_VBUS_SENSE_Pin SHIFT_Q7_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin|SHIFT_Q7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DISP_CLK7_Pin */
  GPIO_InitStruct.Pin = DISP_CLK7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DISP_CLK7_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Revert VBUS to NOPULL because PULLDOWN lowers the voltage too much on the divider */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /* Configure ROTARY_CLK_Pin (PA0) as EXTI0 */
  GPIO_InitStruct.Pin = ROTARY_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROTARY_CLK_GPIO_Port, &GPIO_InitStruct);
  
  /* Configure ROTARY_DT_Pin (PA4) as EXTI4 */
  GPIO_InitStruct.Pin = ROTARY_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROTARY_DT_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  // PRIORITY DITURUNKAN KE 5 AGAR TIDAK MEMBUAT USB NGE-HANG SAAT ENCODER DIPUTAR
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ROTARY_CLK_Pin || GPIO_Pin == ROTARY_DT_Pin) {
        static const int8_t encoder_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        static uint8_t old_AB = 0;
        static int8_t counter = 0;
        
        uint8_t A = HAL_GPIO_ReadPin(ROTARY_CLK_GPIO_Port, ROTARY_CLK_Pin) == GPIO_PIN_SET ? 1 : 0;
        uint8_t B = HAL_GPIO_ReadPin(ROTARY_DT_GPIO_Port, ROTARY_DT_Pin) == GPIO_PIN_SET ? 1 : 0;
        
        old_AB <<= 2;                   // Shift previous state
        old_AB |= ( (A << 1) | B );     // Add current state
        old_AB &= 0x0f;                 // Keep only lowest 4 bits
        
        int8_t move = encoder_states[old_AB];
        counter += move;
        
        // 4 steps per detent for most standard rotary encoders
        if (counter >= 4) { 
            extern void Motor_RotaryIncrement(void);
            Motor_RotaryIncrement();
            counter = 0;
        } else if (counter <= -4) {
            extern void Motor_RotaryDecrement(void);
            Motor_RotaryDecrement();
            counter = 0;
        }
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
