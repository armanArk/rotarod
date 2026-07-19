/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_LED_Pin GPIO_PIN_13
#define MCU_LED_GPIO_Port GPIOC
#define POTENTIOMETER_Pin GPIO_PIN_0
#define POTENTIOMETER_GPIO_Port GPIOA
#define SHIFT_PL_Pin GPIO_PIN_2
#define SHIFT_PL_GPIO_Port GPIOA
#define SHIFT_CP_Pin GPIO_PIN_3
#define SHIFT_CP_GPIO_Port GPIOA
#define MOTOR_EL_Pin GPIO_PIN_4
#define MOTOR_EL_GPIO_Port GPIOA
#define DISP_DIO1_Pin GPIO_PIN_0
#define DISP_DIO1_GPIO_Port GPIOB
#define DISP_DIO2_Pin GPIO_PIN_1
#define DISP_DIO2_GPIO_Port GPIOB
#define DISP_DIO3_Pin GPIO_PIN_2
#define DISP_DIO3_GPIO_Port GPIOB
#define DISP_DIO7_Pin GPIO_PIN_10
#define DISP_DIO7_GPIO_Port GPIOB
#define DISP_DIO5_Pin GPIO_PIN_12
#define DISP_DIO5_GPIO_Port GPIOB
#define DISP_DIO6_Pin GPIO_PIN_13
#define DISP_DIO6_GPIO_Port GPIOB
#define USB_VBUS_SENSE_Pin GPIO_PIN_8
#define USB_VBUS_SENSE_GPIO_Port GPIOA
#define FTDI_TX_Pin GPIO_PIN_9
#define FTDI_TX_GPIO_Port GPIOA
#define FTDI_RX_Pin GPIO_PIN_10
#define FTDI_RX_GPIO_Port GPIOA
#define SHIFT_Q7_Pin GPIO_PIN_15
#define SHIFT_Q7_GPIO_Port GPIOA
#define DISP_DIO4_Pin GPIO_PIN_4
#define DISP_DIO4_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_5
#define FLASH_CS_GPIO_Port GPIOB
#define RTC_SCL_Pin GPIO_PIN_6
#define RTC_SCL_GPIO_Port GPIOB
#define RTC_SDA_Pin GPIO_PIN_7
#define RTC_SDA_GPIO_Port GPIOB
#define BLDC_PULSE_Pin GPIO_PIN_8
#define BLDC_PULSE_GPIO_Port GPIOB
#define TM1637_CLK_Pin GPIO_PIN_9
#define TM1637_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ALL_DIO_PINS (DISP_DIO1_Pin | DISP_DIO2_Pin | DISP_DIO3_Pin | \
                      DISP_DIO4_Pin | DISP_DIO5_Pin | DISP_DIO6_Pin | DISP_DIO7_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
