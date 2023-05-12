#ifndef __MY_LIB_H__
#define __MY_LIB_H__
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include <stdlib.h>
#include <stdio.h>
#define on 1
#define off 0
#define led1(stt) (stt)?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0)
#define led2(stt) (stt)?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0)
#define led3(stt) (stt)?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0)
#define read_btn HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)

void print_number(UART_HandleTypeDef *huart,unsigned int number);
void printf_number(UART_HandleTypeDef *huart,unsigned int number);
void print_string(UART_HandleTypeDef *huart,uint8_t *s);
void printf_string(UART_HandleTypeDef *huart,uint8_t *s);
unsigned char size(unsigned char *s);


#endif
