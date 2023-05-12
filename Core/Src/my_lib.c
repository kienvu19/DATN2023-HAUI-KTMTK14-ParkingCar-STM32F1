#include "my_lib.h"

void print_number(UART_HandleTypeDef *huart,unsigned int number)
{
	char buff_value[50];
	HAL_UART_Transmit(huart,(uint8_t*)buff_value,sprintf(buff_value, "%d", number),10);
	//HAL_USART_Transmit(huart,"\n",1,10);
}


void printf_number(UART_HandleTypeDef *huart,unsigned int number)
{
	char buff_value[50];
	HAL_UART_Transmit(huart,(uint8_t*)buff_value,sprintf(buff_value, "%d", number),10);
	HAL_UART_Transmit(huart,"\n",1,10);
}

void print_string(UART_HandleTypeDef *huart,uint8_t *s)
{
	HAL_UART_Transmit(huart,s,size(s),10);
}

void printf_string(UART_HandleTypeDef *huart,uint8_t *s)
{
	HAL_UART_Transmit(huart,s,size(s),10);
	HAL_UART_Transmit(huart,"\n",1,10);
}

unsigned char size(unsigned char *s)
{
    int count=0;
    while(s[count]!=0) count++;
    return count;
}




