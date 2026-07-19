#ifndef TM1637_H
#define TM1637_H

#include "main.h"

void TM1637_SetBrightness(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin,
                          GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin,
                          uint8_t brightness);

void TM1637_DisplayNumber(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin,
                          GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin,
                          uint16_t num, uint8_t dots);

void TM1637_SetColon(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin,
                     GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin,
                     uint16_t current_num, uint8_t status);

void TM1637_SetDecimalDot(GPIO_TypeDef* CLK_Port, uint16_t CLK_Pin,
                          GPIO_TypeDef* DIO_Port, uint16_t DIO_Pin,
                          uint16_t current_num, uint8_t digit_index, uint8_t status);

#endif
