/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Rotarod with FatFs on W25Q Flash + USB MSC
  *                   - CSV file visible in Windows Explorer
  *                   - Auto unmount/mount on USB plug/unplug
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TM1637.h"
#include "74hc165.h"
#include "w25q64.h"
#include "ds3231.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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

// ====== 7 Display (CLK direct) ======
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

// ====== FatFs Variables ======
FATFS FatFs;
FIL csv_file;
UINT bw;
FRESULT fr;
static uint8_t fs_mounted = 0;
static uint8_t fs_formatted = 0;

// ====== Logging ======
#define MAX_EVENTS          400

typedef struct {
    uint32_t timestamp;
    uint16_t duration_ms;
    uint16_t rpm;
    uint8_t  lane;
} FallEvent_t;

static FallEvent_t event_queue[MAX_EVENTS];
static uint16_t event_count = 0;
static uint8_t csv_header_written = 0;

// ====== USB VBUS detection ======
uint8_t usb_connected = 0;
uint32_t last_usb_check = 0;
#define USB_CHECK_MS  100
static uint8_t last_usb_state = 0;

// ====== Debounce for fall simulation ======
static uint8_t prev_fall = 0;
static uint32_t last_fall_time = 0;
#define FALL_DEBOUNCE_MS  500

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
void UART_Print(char *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ====== FatFs Helper Functions ======

static void MountFS(void) {
    if (fs_mounted) return;
    fr = f_mount(&FatFs, "", 1);
    if (fr == FR_OK) {
        fs_mounted = 1;
        UART_Print("FatFS mounted OK\r\n");
    } else if (fr == FR_NO_FILESYSTEM) {
        UART_Print("No filesystem - needs format\r\n");
        fs_mounted = 0;
    } else {
        char msg[32];
        sprintf(msg, "FatFS mount err: %d\r\n", fr);
        UART_Print(msg);
        fs_mounted = 0;
    }
}

static void UnmountFS(void) {
    if (!fs_mounted) return;
    f_mount(NULL, "", 0);
    fs_mounted = 0;
    UART_Print("FatFS unmounted\r\n");
}

static void FormatFS(void) {
    UART_Print("Formatting flash...\r\n");
    BYTE work[_MAX_SS];
    fr = f_mkfs("", FM_FAT, 0, work, sizeof(work));
    if (fr == FR_OK) {
        UART_Print("Format OK\r\n");
        fs_formatted = 1;
        MountFS();
    } else {
        char msg[32];
        sprintf(msg, "Format failed: %d\r\n", fr);
        UART_Print(msg);
    }
}

static void WriteCSVHeader(void) {
    if (!fs_mounted) return;
    fr = f_open(&csv_file, "events.csv", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        char header[] = "timestamp,duration_ms,rpm,lane\r\n";
        f_write(&csv_file, header, strlen(header), &bw);
        f_close(&csv_file);
        csv_header_written = 1;
        UART_Print("CSV header written\r\n");
    } else {
        char msg[32];
        sprintf(msg, "Cannot create CSV: %d\r\n", fr);
        UART_Print(msg);
    }
}

static void AppendCSVLine(FallEvent_t *ev) {
    if (!fs_mounted) return;
    fr = f_open(&csv_file, "events.csv", FA_WRITE | FA_OPEN_APPEND);
    if (fr == FR_OK) {
        char line[64];
        int len = sprintf(line, "%lu,%d,%d,%d\r\n",
                          ev->timestamp, ev->duration_ms, ev->rpm, ev->lane);
        f_write(&csv_file, line, len, &bw);
        f_close(&csv_file);
    }
}

static void Log_Init(void) {
    event_count = 0;
    csv_header_written = 0;
}

static void Log_AddEvent(uint16_t duration_ms, uint16_t rpm_val, uint8_t lane) {
    if (event_count >= MAX_EVENTS) {
        UART_Print("Queue full!\r\n");
        return;
    }
    event_queue[event_count].timestamp = HAL_GetTick() / 1000;
    event_queue[event_count].duration_ms = duration_ms;
    event_queue[event_count].rpm = rpm_val;
    event_queue[event_count].lane = lane;
    event_count++;
    char msg[64];
    sprintf(msg, "Event added: dur=%d, rpm=%d, lane=%d\r\n", duration_ms, rpm_val, lane);
    UART_Print(msg);
}

static void Log_FlushToCSV(void) {
    if (event_count == 0) {
        UART_Print("No events.\r\n");
        return;
    }
    if (!csv_header_written) {
        WriteCSVHeader();
    }
    for (int i = 0; i < event_count; i++) {
        AppendCSVLine(&event_queue[i]);
    }
    event_count = 0;
    UART_Print("Flushed to CSV file\r\n");
}

static void Log_ReadCSV(void) {
    if (!fs_mounted) {
        UART_Print("FS not mounted\r\n");
        return;
    }
    fr = f_open(&csv_file, "events.csv", FA_READ);
    if (fr == FR_OK) {
        char buf[256];
        UINT br;
        UART_Print("\r\n=== CSV File ===\r\n");
        while (f_read(&csv_file, buf, sizeof(buf)-1, &br) == FR_OK && br > 0) {
            buf[br] = '\0';
            UART_Print(buf);
        }
        UART_Print("\r\n=== End ===\r\n");
        f_close(&csv_file);
    } else {
        UART_Print("Cannot open events.csv\r\n");
    }
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // --- 1. 74HC165 ---
  HC165_SetPins(SHIFT_PL_GPIO_Port, SHIFT_PL_Pin,
                SHIFT_CP_GPIO_Port, SHIFT_CP_Pin,
                SHIFT_Q7_GPIO_Port, SHIFT_Q7_Pin);

  // --- 2. CLK output idle HIGH ---
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

  // --- 3. Display init ---
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

  // --- 4. PB8 TIM4_CH3 ---
  GPIO_InitStruct.Pin = BLDC_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(BLDC_PULSE_GPIO_Port, &GPIO_InitStruct);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

  // --- 5. PWM ---
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

  // --- 6. ADC ---
  HAL_ADC_Start(&hadc1);

  // --- 7. SPI Flash init (for FatFs disk driver) ---
  W25Q64_Init();
  uint8_t sr = W25Q64_ReadStatusRegister1();
  if (sr != 0x00) {
      W25Q64_WriteStatusRegister1(0x00);
  }

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

  // --- 8. Mount or format FatFs ---
  MountFS();
  if (!fs_mounted && !fs_formatted) {
      FormatFS();
  }

  // --- 9. RTC ---
  DS3231_Init();

  // --- 10. Logging init ---
  Log_Init();

  last_display_tick = HAL_GetTick();
  last_capture_time = HAL_GetTick();
  last_button_read_tick = HAL_GetTick();
  last_usb_check = HAL_GetTick();
  prev_fall = 0;
  last_fall_time = 0;

  UART_Print("System Start\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // --- Baca shift register setiap 20ms ---
    if (HAL_GetTick() - last_button_read_tick >= BUTTON_READ_MS) {
      HC165_Read(status_tombol, HC165_MY_NUM_CHIPS);
      HC165_Unpack(status_tombol, pins, HC165_MY_NUM_CHIPS);
      last_button_read_tick = HAL_GetTick();
    }

    // ====== Simulasi Jatuh: rising edge + debounce 500ms ======
    if (pins[0] && !prev_fall && (HAL_GetTick() - last_fall_time > FALL_DEBOUNCE_MS)) {
        uint16_t duration = 1000 + (rand() % 2001);
        Log_AddEvent(duration, rpm, 3);
        last_fall_time = HAL_GetTick();
    }
    prev_fall = pins[0];

    // ====== Flush: D1 IC A ======
    if (pins[1]) {
      static uint32_t last_flush = 0;
      if (HAL_GetTick() - last_flush > 300) {
        Log_FlushToCSV();
        last_flush = HAL_GetTick();
      }
    }

    // ====== Read CSV: D2 IC A ======
    if (pins[2]) {
      static uint32_t last_read = 0;
      if (HAL_GetTick() - last_read > 300) {
        Log_ReadCSV();
        last_read = HAL_GetTick();
      }
    }

    // --- USB VBUS detection via PA8 ---
    if (HAL_GetTick() - last_usb_check >= USB_CHECK_MS) {
        uint8_t new_usb = HAL_GPIO_ReadPin(USB_VBUS_SENSE_GPIO_Port, USB_VBUS_SENSE_Pin);

        if (new_usb && !last_usb_state) {
            // USB CONNECTED: Unmount FatFs so Windows can take over
            UART_Print("USB connected - unmounting FS\r\n");
            if (event_count > 0) {
                Log_FlushToCSV();  // Flush remaining events first
            }
            UnmountFS();
            HAL_Delay(50);  // Let USB stack settle
        } else if (!new_usb && last_usb_state) {
            // USB DISCONNECTED: Remount FatFs for STM32 access
            UART_Print("USB disconnected - remounting FS\r\n");
            HAL_Delay(100);
            MountFS();
        }

        last_usb_state = new_usb;
        usb_connected = new_usb;
        last_usb_check = HAL_GetTick();
    }

    // --- Update display & UART setiap 100ms ---
    if (HAL_GetTick() - last_display_tick >= DISPLAY_UPDATE_MS) {
      if (HAL_GetTick() - last_capture_time > 500) {
        rpm = 0;
      }

      Display_ShowNumber(2, (uint16_t)rpm, 0);

      // Baca waktu RTC
      uint8_t sec, min, hour, day, date, month, year;
      DS3231_ReadTime(&sec, &min, &hour, &day, &date, &month, &year);

      char uart_buf[250];
      sprintf(uart_buf,
              "[%02d:%02d:%02d %02d/%02d/%02d] RPM:%lu ADC:%lu PWM:%lu SET:%lu Q:%d USB:%d FS:%d\r\n",
              hour, min, sec, date, month, year,
              (unsigned long)rpm,
              adc_raw,
              pwm_duty,
              set_value,
              event_count,
              usb_connected,
              fs_mounted);
      UART_Print(uart_buf);

      last_display_tick = HAL_GetTick();
    }

    // --- ADC & PWM ---
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISP_CLK7_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DISP_CLK7_GPIO_Port, DISP_CLK7_Pin, GPIO_PIN_SET);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // FIX: Removed the reconfiguration of DIO pins as inputs with pull‑up,
  // because TM1637 requires them as outputs (push‑pull) for communication.
  // The DIO pins are correctly set as outputs above.
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
