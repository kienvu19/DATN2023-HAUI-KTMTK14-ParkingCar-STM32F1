/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "lcd_i2c.h"
#include "my_lib.h"
#include "mfrc522.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <stdarg.h>

#define SERVO1 0
#define SERVO2 1
//servo1
#define PUSH 2
#define WAIT 3
//servo2
#define PUSH2 WAIT
#define WAIT2 PUSH

#define _for for
#define delay HAL_Delay
uint8_t CardID[5];
char szBuff[16];

#define SERVO_RA SERVO1
#define SERVO_VAO SERVO2
#define OPEN 1
#define CLOSE 2
#define IN 1
#define OUT 2

#define read(port,pin) HAL_GPIO_ReadPin(port, pin)
void delay_us (uint32_t us);
void servo(unsigned char servo,unsigned char vi_tri);
void barie_vao(char stt);
void barie_ra(char stt);

char stt_in_out=0;
bool rfid_valid=0;

uint8_t hex_to_char(uint8_t data);
void char_to_hex(uint8_t data);
void StrTo();
void ToStr(uint32_t number);

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

/* USER CODE BEGIN PV */
uint8_t 	k;
uint8_t 	i;
uint8_t 	j;
uint8_t 	b;
uint8_t 	q;
uint8_t 	en;
uint8_t 	ok;
uint8_t 	comand;
uint8_t		text1[63] = "STM32F103 Mifare RC522 RFID Card reader 13.56 MHz for KEIL HAL\r";
uint8_t		text2[9] = "Card ID: ";
uint8_t		end[1] = "\r";
uint8_t		txBuffer[8] = "";
uint8_t 	retstr[10];
uint8_t 	rxBuffer[8];
uint8_t		lastID[4];
uint8_t		memID[8] = "9C55A1B5";
uint8_t		str[MFRC522_MAX_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void servo(unsigned char servo,unsigned char vi_tri)
{
  unsigned int t1=0;
  //ve vi tri cho
  if(vi_tri==WAIT)
  for(t1=0;t1<10;t1++)
  {
    if(servo==SERVO1) HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, 1);
    if(servo==SERVO2) HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, 1);
    delay_us(2300);
    if(servo==SERVO1) HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, 0);
    if(servo==SERVO2) HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, 0);
    HAL_Delay(19);
  }

  //ve vi tri day vat
  else if(vi_tri==PUSH)
  for(t1=0;t1<10;t1++)
  {
	if(servo==SERVO1) HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, 1);
	if(servo==SERVO2) HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, 1);
	delay_us(1500);
    if(servo==SERVO1) HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, 0);
    if(servo==SERVO2) HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, 0);
    HAL_Delay(19);
  }

}

void barie_vao(char stt)
{
	if(stt==OPEN) servo(SERVO_VAO,PUSH);
	else servo(SERVO_VAO,WAIT);
}

void barie_ra(char stt)
{
	if(stt==OPEN) servo(SERVO_RA,WAIT);
	else servo(SERVO_RA,PUSH);
}

void delay_us (uint32_t us)
{
	HAL_TIM_Base_Start(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}




char uart_data_rec[100];
char uart_char_rec=0;
int uart_count_char=0;
bool uart_stop_rec=false;
bool uart_done_frame=false;
unsigned int count_led=0;
char count_fix=0;

/*
 * Ngắt nhận dữ liệu từ máy tính gửi xuống
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		//interrupt rs422
	  if(huart->Instance==huart1.Instance)
	  {
		  if(uart_char_rec!='\n' && uart_done_frame==false)
		  {
			  uart_data_rec[uart_count_char]=uart_char_rec;
			  uart_count_char++;
		  }
		  else
		  {
			  uart_done_frame=true;
		  }
	  }

	  HAL_UART_Receive_IT(&huart1,&uart_char_rec,1);
}

void rs485_print(const char *fmt, ...)
{
	static char buffer[256];
	unsigned int len=0;
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	len=strlen(buffer);
	HAL_GPIO_WritePin(DTE_GPIO_Port, DTE_Pin, 1);
	HAL_UART_Transmit(&huart1,buffer,len,100);
	HAL_GPIO_WritePin(DTE_GPIO_Port, DTE_Pin, 0);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char stt_servo_in=WAIT;
char stt_servo_out=WAIT;
char stt_servo_in_last=WAIT;
char stt_servo_out_last=WAIT;

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  MFRC522_Init();
  lcd_init(); HAL_Delay(100);;
  lcd_string(1, 4, "BAI DO XE");
  lcd_string(2, 4, "WELL COME");

  //HAL_TIM_Base_Start(&htim2);
  HAL_UART_Receive_IT(&huart1,&uart_char_rec,1);

  servo(SERVO1, WAIT);
  servo(SERVO2, WAIT2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  	//IN
		if(!read(CB_IN_GPIO_Port,CB_IN_Pin)) stt_servo_in=PUSH;
		else stt_servo_in=WAIT;

		if(stt_servo_in_last==WAIT && stt_servo_in==PUSH)
		{
			servo(SERVO1,PUSH);
		}
		else if(stt_servo_in_last==PUSH && stt_servo_in==WAIT)
		{
			servo(SERVO1,WAIT);
		}
		stt_servo_in_last=stt_servo_in;


		//OUT
		if(!read(CB_OUT_GPIO_Port,CB_OUT_Pin))
		{
			stt_servo_out=PUSH2;
		}
		else
		{
			stt_servo_out=WAIT2;
		}
		*/

		//if(stt_servo_out_last==WAIT2 && stt_servo_out==PUSH2) servo(SERVO2,PUSH2);
		//else if(stt_servo_out_last==PUSH2 && stt_servo_out==WAIT2) servo(SERVO2,WAIT2);
		//stt_servo_out_last=stt_servo_out;



	if (!MFRC522_Request(PICC_REQIDL, str))
	{
		if (!MFRC522_Anticoll(str))
		{
			j = 0;
			q = 0;
			b = 9;
			en = 1;

			for (i=0; i<4; i++) if (lastID[i] != str[i]) j = 1;								// Repeat test
			//if (j && en)
			{
				q = 0;
				en = 0;
				for (i=0; i<4; i++) lastID[i] = str[i];
				for (i=0; i<4; i++)
				{
					char_to_hex(str[i]);
					txBuffer[b] = retstr[0];
					b++;
					txBuffer[b] = retstr[1];
					b++;
				}

				 //Gửi thông tin mã thẻ lên máy tính theo frame đã quy ước
				 //Frame format $ INOUT RFIDCODE VTVALUEE @ VT là vị trí xe còn trống
           	   	 //Ex $ IN RFID123456 VT9E @

				//gửi Header
				rs485_print("$");

				//gửi trạng thái xe vào/ra
				if(!HAL_GPIO_ReadPin(CB_IN_GPIO_Port, CB_IN_Pin))
				{
					stt_in_out=IN;
					rs485_print(" IN ");
				}
				else if(!HAL_GPIO_ReadPin(CB_OUT_GPIO_Port, CB_OUT_Pin))
				{
					rs485_print(" OUT ");
					stt_in_out=OUT;
				}

				rs485_print("RFID");

				//gửi mã thẻ RFID
				char *temp;
				for(int i=0;i<18;i++)
				{
					char txtTemp[2]="";
					if(txBuffer[i]!=0)
					{
						sprintf(txtTemp,"%x",txBuffer[i]);
						rs485_print(txtTemp);
					}
				}

				//gửi trạng thái 5 vị trí xe trong bãi
				rs485_print(" ");
				char viTriXe=0;
				if(!HAL_GPIO_ReadPin(VT1_GPIO_Port, VT1_Pin)) rs485_print("VT1ON"); else rs485_print("VT1OFF");//viTriXe|=0x01;
				if(!HAL_GPIO_ReadPin(VT2_GPIO_Port, VT2_Pin)) rs485_print("VT2ON"); else rs485_print("VT2OFF");//viTriXe|=0x02;
				if(!HAL_GPIO_ReadPin(VT3_GPIO_Port, VT3_Pin)) rs485_print("VT3ON"); else rs485_print("VT3OFF");//viTriXe|=0x04;
				if(!HAL_GPIO_ReadPin(VT4_GPIO_Port, VT4_Pin)) rs485_print("VT4ON"); else rs485_print("VT4OFF");//viTriXe|=0x08;
				if(!HAL_GPIO_ReadPin(VT5_GPIO_Port, VT5_Pin)) rs485_print("VT5ON"); else rs485_print("VT5OFF");//viTriXe|=0x10;

				rs485_print(" VTE\r\n",viTriXe);

				//đợi 1s sau mới đ�?c thẻ tiếp
				HAL_Delay(1000);
				count_fix=0;
				ok = 1;
				for (i=0; i<8; i++) if (txBuffer[9+i] != memID[i]) ok = 0;
			}
		}
	}//


	q++;
	if (!q)
	{
		en = 1;								// Delay against scan kode
		for (i=0; i<4; i++) lastID[i] = 0;	// Delay reading the same card 3s
	}


	//nháy led status
	if(count_led<10) count_led++;
	else
	{
		count_led=0;
		HAL_GPIO_TogglePin(LED_STT_GPIO_Port, LED_STT_Pin);

		rs485_print("$");
		if(!HAL_GPIO_ReadPin(VT1_GPIO_Port, VT1_Pin)) rs485_print("VT1ON"); else rs485_print("VT1OFF");//viTriXe|=0x01;
		if(!HAL_GPIO_ReadPin(VT2_GPIO_Port, VT2_Pin)) rs485_print("VT2ON"); else rs485_print("VT2OFF");//viTriXe|=0x02;
		if(!HAL_GPIO_ReadPin(VT3_GPIO_Port, VT3_Pin)) rs485_print("VT3ON"); else rs485_print("VT3OFF");//viTriXe|=0x04;
		if(!HAL_GPIO_ReadPin(VT4_GPIO_Port, VT4_Pin)) rs485_print("VT4ON"); else rs485_print("VT4OFF");//viTriXe|=0x08;
		if(!HAL_GPIO_ReadPin(VT5_GPIO_Port, VT5_Pin)) rs485_print("VT5ON"); else rs485_print("VT5OFF");//viTriXe|=0x10;
		rs485_print("E\r\n");
	}



	//đi�?u khiển barie khi thẻ đúng/sai
	/*
	 * Nếu xe đi vào
	 */
	if(stt_in_out==IN)
	{
		/*
		 *	Nếu máy tính trả v�? kết quả thẻ là hơp lệ, máy tính sẽ gửi ký tự T (true), nếu mã thẻ là sai máy tính sẽ gửi ký tự là F (false)
		 *	uart_char_rec là biến lưu ký tự máy tính gửi xuống
		 */
		if(uart_char_rec=='T')
		{
			lcd_clear();
			lcd_string(1, 4, "THE DUNG");
			servo(SERVO1,PUSH);
			servo(SERVO2,WAIT2);
			HAL_Delay(3000);
			servo(SERVO1,WAIT);
			servo(SERVO2,WAIT2);

		}
		else if(uart_char_rec=='F')
		{
			lcd_clear();
			lcd_string(1, 4, "THE SAI");
			servo(SERVO1,WAIT);
			servo(SERVO2,WAIT2);
			HAL_Delay(3000);
		}
		lcd_string(1, 4, "BAI DO XE");
		lcd_string(2, 4, "WELL COME");
		uart_char_rec=0;
	}
	/*
	 * Nếu xe đi ra
	 */
	else if(stt_in_out==OUT)
	{
		if(uart_char_rec=='T')
		{
			lcd_clear();
			lcd_string(1, 4, "THE DUNG");
			servo(SERVO1,WAIT);
			servo(SERVO2,PUSH2);
			HAL_Delay(3000);
			servo(SERVO1,WAIT);
			servo(SERVO2,WAIT2);
		}
		else if(uart_char_rec=='F')
		{
			lcd_clear();
			lcd_string(1, 4, "THE SAI");
			servo(SERVO1,WAIT);
			servo(SERVO2,WAIT2);
			HAL_Delay(3000);
		}
		lcd_string(1, 4, "BAI DO XE");
		lcd_string(2, 4, "WELL COME");
		uart_char_rec=0;
	}

    HAL_Delay(5);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* USER CODE BEGIN 4 */
// string hex to char number
uint8_t hex_to_char(uint8_t data)
{
	uint8_t number;

	if (rxBuffer[data] < 58) number = (rxBuffer[data]-48)*16; else number = (rxBuffer[data]-55)*16;
	data++;
	if (rxBuffer[data] < 58) number = number+(rxBuffer[data]-48); else number = number+(rxBuffer[data]-55);
	return number;
}

// char number to string hex (FF) (Only big letters!)
void char_to_hex(uint8_t data)
{
	uint8_t digits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

	if (data < 16)
	{
		retstr[0] = '0';
		retstr[1] = digits[data];
	} else {
		retstr[0] = digits[(data & 0xF0)>>4];
		retstr[1] = digits[(data & 0x0F)];
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

