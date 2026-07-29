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

#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

#include "fs_logger.h"
#include "cli.h"
#include "motor_control.h"
#include "ui.h"
#include "w25q64.h"
#include "ds3231.h"
#include "usbd_core.h"
#include "staging.h"
#include "settings.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
/* Control MSC readiness/capacity in USB storage layer */
extern void STORAGE_SetMediaReady(uint8_t ready);
extern void STORAGE_UpdateCapacity(uint32_t bytes);
extern void STORAGE_SetPartitionOffset(uint32_t byte_offset);

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim4, htim5;
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

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    MX_TIM4_Init();

    // Set USB OTG priority 3 (di bawah TIM4=0 dan USART1=2, di atas DMA=6)
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 3, 0);


    /* Initialize FatFs (disk detection) before USB so MSC reports correct capacity */
    MX_FATFS_Init();
    MX_USB_DEVICE_Init();

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

    // ADC
    HAL_ADC_Start(&hadc1);

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

    // Mount FatFs (formatting can be triggered on-demand via FTDI command "FORMAT")
    MountFS();
    if (!FS_IsMounted() && !FS_IsFormatted()) {
        UART_Print("Flash is completely empty. Performing one-time format...\r\n");
        FormatFS();
    }

    // RTC
    DS3231_Init();

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

    while (1)
    {
        // Process commands over FTDI UART (e.g., "FORMAT", "CHECKFS", "HELP")
        ProcessUartRxCommand();
        // Shift register and displays
        UI_Process();
        // ADC & PWM
        Motor_Process();

        // USB VBUS detection (handled by EXTI or periodic poll)
        if (vbus_event_pending || (HAL_GetTick() - last_usb_check >= USB_CHECK_MS)) {
            uint8_t new_usb = HAL_GPIO_ReadPin(USB_VBUS_SENSE_GPIO_Port, USB_VBUS_SENSE_Pin);
            if (HAL_GetTick() < 2000) new_usb = 0; // Ignore VBUS during early boot (debounce brownout)
            if (new_usb && !last_usb_state) {
                UART_Print("USB connected - exporting to partition 1\r\n");
                STORAGE_SetMediaReady(0);

                if (!FS_IsMounted()) MountFS();

                if (Log_GetEventCount() > 0) Log_FlushToCSV();

                if (staging_has_entries()) {
                    UART_Print("Committing staged entries to FS...\r\n");
                    if (staging_commit() == 0) UART_Print("Staging committed\r\n");
                    else UART_Print("Staging commit failed\r\n");
                }

                ExportCsvToPartition1();

                HAL_Delay(50);
                UnmountAllFS();

                uint32_t lba_start = 0, sector_count = 0;
                if (ReadExportPartition(&lba_start, &sector_count)) {
                    STORAGE_SetPartitionOffset(lba_start * 512U);
                    STORAGE_UpdateCapacity(sector_count * 512U);
                    char dbg[80];
                    sprintf(dbg, "USB MSC: LBA %lu, %lu sectors\r\n",
                            (unsigned long)lba_start, (unsigned long)sector_count);
                    UART_Print(dbg);
                } else {
                    STORAGE_SetPartitionOffset(FS_GetFlashCapacity() / 2);
                    STORAGE_UpdateCapacity(FS_GetFlashCapacity() / 2);
                    UART_Print("USB MSC: fallback half-flash export\r\n");
                }
                STORAGE_SetMediaReady(1);
            } else if (!new_usb && last_usb_state) {
                UART_Print("USB disconnected - remounting FS\r\n");
                STORAGE_SetPartitionOffset(0);
                STORAGE_UpdateCapacity(FS_GetFlashCapacity());
                STORAGE_SetMediaReady(0);
                HAL_Delay(100);
                MountFS();
            }
            last_usb_state = new_usb;
            usb_connected = new_usb;
            last_usb_check = HAL_GetTick();
            vbus_event_pending = 0;
        }

        // Debug UART (100 ms)
        if (HAL_GetTick() - last_debug_tick >= 100) {
            uint8_t sec, min, hour, day, date, month, year;
            DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);

            /* 
            float kp, ki, kd;
            Motor_GetPID(&kp, &ki, &kd);
            
            float err, integ;
            Motor_GetPIDState(&err, &integ);
            
            int kp_i = (int)kp, kp_f = (int)((kp - kp_i) * 100);
            int ki_i = (int)ki, ki_f = (int)((ki - ki_i) * 100);
            int kd_i = (int)kd, kd_f = (int)((kd - kd_i) * 100);
            if (kp_f < 0) kp_f = -kp_f;
            if (ki_f < 0) ki_f = -ki_f;
            if (kd_f < 0) kd_f = -kd_f;
            
            int err_i = (int)err, err_f = (int)((err - err_i) * 10);
            int int_i = (int)integ, int_f = (int)((integ - int_i) * 10);
            if (err_f < 0) err_f = -err_f;
            if (int_f < 0) int_f = -int_f;

            float precise_orpm = Motor_GetPreciseOutputRPM();
            int orpm_i = (int)precise_orpm;
            int orpm_f = (int)((precise_orpm - orpm_i) * 10);
            if (orpm_f < 0) orpm_f = -orpm_f;

            float precise_mrpm = Motor_GetPreciseMotorRPM();
            int mrpm_i = (int)precise_mrpm;
            int mrpm_f = (int)((precise_mrpm - mrpm_i) * 10);
            if (mrpm_f < 0) mrpm_f = -mrpm_f;

            char uart_buf[320];
            sprintf(uart_buf, "[%02d:%02d:%02d.%03lu] SET:%lu RPM:%d.%d TRPM:%lu MRPM:%d.%d PWM:%lu ERR:%d.%d INT:%d.%d Kp:%d.%02d Ki:%d.%02d Kd:%d.%02d\r\n",
                    hour, min, sec, (unsigned long)(HAL_GetTick() % 1000),
                    Motor_GetSetValue(), orpm_i, orpm_f,           // RPM float 1 desimal
                    (unsigned long)(Motor_GetSetValue() * 25),     // TRPM = Target Motor RPM
                    mrpm_i, mrpm_f,                                // MRPM = Actual Motor RPM (float presisi)
                    Motor_GetPWMDuty(),
                    err_i, err_f, int_i, int_f,
                    kp_i, kp_f, ki_i, ki_f, kd_i, kd_f);
            UART_Print(uart_buf);
            */

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
            last_heartbeat = HAL_GetTick();
        }

        // Refresh independent watchdog (register-based)
        IWDG->KR = 0xAAAA;
    }
}

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
    htim4.Init.Prescaler = 83; // 84MHz / 84 = 1 MHz → 1 tick = 1 µs (resolusi 100× lebih baik)
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

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, SHIFT_PL_Pin | SHIFT_CP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO7_Pin |
                            DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin |
                            DISP_CLK2_Pin | DISP_DIO4_Pin | FLASH_CS_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DISP_CLK1_Pin | DISP_CLK5_Pin | DISP_CLK6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SHIFT_PL_Pin | SHIFT_CP_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | DISP_DIO7_Pin |
                          DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_CLK3_Pin | DISP_CLK4_Pin |
                          DISP_CLK2_Pin | DISP_DIO4_Pin | FLASH_CS_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin | SHIFT_Q7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable and set EXTI line Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    GPIO_InitStruct.Pin = DISP_CLK7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DISP_CLK7_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DISP_CLK7_GPIO_Port, DISP_CLK7_Pin, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Motor_TIM_IC_CaptureCallback(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    Motor_TIM_PeriodElapsedCallback(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USB_VBUS_SENSE_Pin) {
        uint32_t now = HAL_GetTick();
        static uint32_t last_vbus_irq = 0;
        if ((now - last_vbus_irq) >= VBUS_DEBOUNCE_MS) {
            vbus_event_pending = 1;
            last_vbus_irq = now;
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
