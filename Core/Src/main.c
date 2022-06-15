/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "lwip.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "netif.h"
#include "adcstream.h"
#include "neo7m.h"
#include "mydebug.h"
#include "ip4.h"
#include "lwip.h"
#include "lwipopts.h"
#include "lwip/prot/dhcp.h"

#include "udpstream.h"
#include "version.h"
#include "www.h"
#include "dhcp.h"
#include "splat1.h"
#include "lcd.h"
#include "eeprom.h"
#include <queue.h>

//#define netif_dhcp_data(netif) ((struct dhcp*)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP))

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile unsigned long rtos_debug_timer = 0;		// freeRTOS debug

const unsigned char phaser_wav[] = /* { 128, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255,128,128,0,255,128 }; */
{ 114, 89, 141, 101, 114, 127, 102, 127, 89, 127, 128, 113, 141, 101, 128, 113, 128, 127, 128, 127, 153, 113, 114, 140,
		141, 140, 153, 113, 114, 127, 128, 127, 153, 140, 141, 152, 128, 113, 128, 127, 141, 141, 128, 127, 128, 101,
		128, 152, 140, 114, 101, 128, 127, 128, 113, 114, 128, 114, 128, 127, 102, 113, 102, 127, 127, 114, 113, 102,
		113, 114, 141, 102, 89, 101, 114, 89, 102, 102, 88, 141, 114, 128, 128, 114, 102, 113, 114, 127, 141, 127, 114,
		152, 141, 128, 114, 141, 141, 140, 128, 127, 128, 165, 141, 140, 102, 88, 102, 101, 89, 152, 128, 113, 102, 102,
		113, 102, 127, 114, 128, 113, 141, 127, 128, 113, 140, 166, 127, 114, 127, 141, 113, 128, 127, 114, 140, 153,
		102, 101, 102, 141, 127, 89, 88, 114, 101, 141, 113, 141, 152, 114, 113, 128, 152, 102, 101, 114, 141, 113, 89,
		101, 128, 101, 128, 140, 76, 113, 141, 89, 101, 114, 114, 140, 89, 101, 114, 114, 127, 141, 101, 114, 141, 152,
		128, 101, 152, 166, 127, 141, 113, 128, 152, 76, 140, 114, 141, 102, 128, 128, 113, 114, 152, 128, 113, 153,
		128, 88, 102, 140, 64, 88, 141, 88, 89, 101, 166, 102, 88, 179, 49, 114, 140, 64, 141, 113, 152, 102, 140, 141,
		101, 140, 166, 101, 153, 128, 88, 128, 127, 102, 128, 113, 102, 192, 128, 114, 140, 192, 114, 127, 141, 101,
		141, 88, 166, 75, 140, 166, 49, 166, 64, 127, 114, 76, 165, 102, 140, 102, 113, 166, 63, 179, 76, 179, 49, 153,
		75, 141, 102, 140, 114, 101, 166, 63, 191, 76, 165, 114, 178, 102, 128, 166, 114, 153, 76, 192, 49, 166, 37,
		178, 38, 152, 88, 102, 127, 76, 101, 76, 153, 64, 152, 64, 140, 76, 152, 102, 152, 76, 178, 76, 178, 89, 165,
		89, 152, 89, 204, 64, 191, 141, 114, 127, 153, 101, 179, 75, 205, 89, 141, 114, 114, 152, 64, 140, 141, 102,
		191, 64, 191, 89, 102, 152, 50, 204, 50, 191, 50, 101, 166, 63, 179, 24, 179, 88, 114, 152, 102, 140, 64, 128,
		64, 179, 75, 76, 179, 25, 205, 76, 101, 141, 88, 192, 89, 89, 178, 38, 165, 114, 113, 179, 11, 179, 166, 24,
		166, 140, 64, 205, 88, 102, 140, 64, 141, 178, 89, 101, 166, 37, 179, 140, 37, 205, 191, 38, 178, 153, 63, 205,
		75, 114, 230, 64, 114, 216, 49, 76, 204, 50, 88, 217, 89, 113, 205, 75, 114, 165, 76, 127, 153, 37, 141, 216,
		50, 128, 191, 50, 128, 179, 89, 101, 179, 88, 113, 205, 88, 89, 204, 102, 49, 192, 140, 75, 141, 165, 102, 140,
		192, 89, 88, 179, 89, 63, 166, 102, 64, 166, 140, 76, 140, 141, 140, 192, 88, 37, 166, 191, 50, 49, 179, 166,
		24, 64, 191, 128, 49, 152, 179, 37, 50, 178, 166, 50, 50, 178, 179, 63, 50, 166, 165, 25, 63, 205, 152, 50, 113,
		192, 152, 25, 49, 179, 152, 25, 64, 178, 166, 63, 64, 204, 192, 37, 25, 192, 191, 38, 25, 165, 230, 88, 12, 127,
		216, 128, 24, 102, 229, 192, 75, 64, 178, 205, 75, 25, 166, 255, 140, 12, 75, 205, 216, 76, 38, 127, 192, 127,
		64, 128, 216, 179, 64, 63, 179, 204, 102, 50, 140, 243, 165, 25, 37, 153, 242, 178, 76, 49, 114, 165, 128, 63,
		64, 141, 165, 102, 49, 114, 216, 153, 37, 38, 165, 243, 152, 38, 24, 140, 217, 152, 25, 25, 127, 192, 128, 25,
		25, 140, 230, 178, 50, 25, 101, 217, 204, 102, 38, 75, 179, 243, 178, 64, 24, 128, 242, 217, 101, 12, 50, 166,
		217, 166, 75, 38, 88, 153, 152, 101, 38, 50, 140, 192, 127, 24, 0, 75, 192, 243, 178, 76, 0, 50, 165, 217, 165,
		63, 12, 63, 166, 229, 179, 75, 25, 89, 191, 243, 204, 89, 25, 50, 141, 230, 229, 153, 63, 38, 101, 205, 243,
		191, 89, 11, 38, 127, 230, 229, 153, 38, 0, 50, 152, 230, 204, 114, 37, 50, 113, 179, 192, 128, 38, 12, 75, 166,
		204, 179, 88, 12, 37, 128, 216, 230, 179, 113, 64, 63, 114, 152, 166, 127, 89, 63, 64, 101, 153, 165, 127, 89,
		63, 89, 101, 141, 153, 140, 102, 89, 113, 141, 166, 152, 141, 113, 76, 75, 114, 165, 192, 165, 102, 38, 24, 76,
		165, 217, 216, 153, 75, 38, 49, 114, 191, 217, 192, 140, 89, 75, 114, 179, 205, 166, 114, 63, 38, 63, 128, 191,
		205, 165, 101, 50, 24, 64, 140, 205, 205, 140, 64, 12, 25, 89, 165, 230, 229, 192, 113, 50, 24, 38, 101, 179,
		217, 204, 153, 88, 38, 37, 76, 153, 205, 216, 179, 101, 38, 25, 49, 102, 153, 191, 179, 141, 88, 63, 50, 75,
		102, 140, 179, 165, 141, 101, 76, 75, 102, 140, 179, 178, 166, 114, 75, 50, 63, 114, 178, 217, 216, 166, 101,
		50, 24, 38, 101, 166, 204, 217, 179, 127, 76, 37, 50, 88, 153, 205, 216, 179, 140, 76, 37, 38, 88, 153, 205,
		230, 191, 128, 63, 12, 11, 49, 114, 178, 205, 191, 153, 102, 49, 25, 37, 76, 141, 191, 205, 179, 140, 76, 25,
		24, 50, 101, 166, 204, 217, 191, 141, 75, 38, 24, 64, 127, 179, 230, 229, 191, 128, 75, 38, 38, 49, 102, 141,
		178, 192, 166, 127, 102, 75, 76, 102, 140, 179, 178, 166, 140, 102, 75, 76, 101, 141, 165, 192, 191, 179, 127,
		88, 50, 49, 76, 113, 166, 204, 217, 178, 128, 64, 11, 0, 24, 76, 127, 192, 230, 229, 205, 152, 102, 50, 24, 38,
		75, 114, 165, 192, 192, 178, 141, 101, 49, 25, 24, 50, 101, 153, 191, 205, 191, 166, 113, 76, 37, 38, 63, 102,
		152, 205, 217, 217, 178, 128, 75, 38, 25, 38, 63, 114, 165, 205, 217, 216, 192, 153, 101, 50, 24, 25, 63, 102,
		152, 192, 216, 217, 178, 141, 76, 37, 25, 24, 50, 101, 153, 192, 217, 217, 192, 152, 102, 49, 25, 24, 38, 76,
		127, 179, 217, 216, 205, 165, 114, 63, 25, 11, 25, 63, 114, 166, 204, 230, 217, 204, 166, 113, 64, 38, 37, 50,
		88, 128, 178, 204, 217, 204, 166, 128, 76, 25, 12, 24, 50, 101, 140, 192, 216, 217, 192, 165, 128, 75, 38, 24,
		25, 49, 76, 127, 166, 204, 217, 204, 179, 141, 101, 64, 49, 50, 63, 89, 113, 166, 191, 205, 204, 192, 152, 114,
		76, 50, 49, 64, 88, 128, 165, 192, 204, 192, 178, 153, 128, 101, 76, 64, 64, 75, 102, 127, 166, 178, 192, 178,
		153, 127, 89, 49, 38, 38, 49, 76, 101, 128, 152, 179, 191, 192, 178, 153, 128, 88, 76, 63, 50, 63, 76, 102, 127,
		153, 166, 165, 166, 152, 128, 114, 76, 63, 50, 38, 49, 76, 101, 141, 165, 192, 205, 204, 192, 165, 141, 101, 76,
		64, 49, 50, 76, 89, 127, 153, 191, 205, 204, 191, 179, 152, 114, 88, 76, 63, 50, 64, 75, 102, 127, 153, 166,
		166, 165, 153, 140, 128, 113, 89, 75, 76, 76, 88, 102, 113, 141, 153, 166, 179, 178, 166, 140, 114, 101, 76, 64,
		64, 63, 76, 89, 127, 153, 178, 192, 204, 192, 192, 165, 128, 101, 64, 50, 37, 37, 50, 75, 102, 127, 166, 192,
		204, 205, 192, 178, 153, 127, 102, 75, 50, 38, 24, 38, 63, 89, 114, 140, 179, 191, 217, 204, 192, 178, 153, 140,
		102, 75, 38, 24, 12, 24, 50, 75, 102, 140, 166, 191, 205, 204, 205, 179, 166, 141, 114, 89, 63, 50, 50, 50, 76,
		89, 114, 140, 166, 191, 205, 205, 205, 191, 179, 140, 114, 75, 50, 38, 24, 25, 49, 64, 88, 102, 140, 166, 178,
		192, 204, 205, 191, 166, 140, 114, 75, 49, 25, 24, 25, 37, 64, 89, 113, 141, 165, 192, 205, 216, 217, 204, 179,
		166, 127, 102, 75, 50, 38, 37, 38, 64, 88, 102, 127, 153, 178, 192, 204, 205, 205, 192, 165, 141, 113, 102, 75,
		64, 49, 50, 49, 76, 88, 114, 140, 166, 178, 205, 216, 205, 192, 165, 141, 113, 89, 63, 38, 24, 25, 25, 37, 50,
		75, 102, 127, 153, 165, 192, 204, 205, 204, 192, 165, 153, 127, 102, 76, 64, 50, 38, 38, 37, 50, 76, 101, 128,
		152, 166, 191, 204, 217, 205, 204, 192, 165, 141, 127, 102, 75, 50, 38, 37, 38, 37, 64, 75, 102, 127, 153, 178,
		192, 205, 204, 192, 191, 179, 165, 140, 128, 101, 89, 63, 50, 50, 38, 37, 50, 75, 89, 113, 141, 165, 179, 191,
		204, 205, 204, 192, 165, 153, 127, 114, 88, 76, 50, 37, 38, 37, 50, 76, 89, 101, 128, 152, 166, 191, 192, 205,
		204, 205, 191, 179, 152, 141, 114, 88, 76, 49, 38, 38, 37, 38, 50, 63, 89, 101, 128, 152, 179, 192, 192, 205,
		204, 192, 178, 166, 152, 141, 113, 102, 75, 64, 49, 38, 38, 38, 50, 50, 63, 89, 101, 128, 152, 166, 179, 191,
		192, 192, 178, 179, 166, 152, 128, 113, 89, 75, 63, 50, 49, 50, 63, 63, 76, 88, 102, 113, 141, 152, 166, 192,
		191, 192, 191, 192, 178, 179, 165, 153, 128, 113, 89, 76, 63, 64, 49, 64, 64, 75, 89, 102, 113, 128, 152, 166,
		179, 192, 191, 192, 178, 179, 165, 153, 140, 128, 101, 89, 75, 64, 63, 49, 50, 64, 63, 76, 88, 102, 114, 127,
		141, 165, 179, 191, 192, 192, 192, 178, 179, 165, 153, 140, 128, 101, 89, 75, 50, 49, 50, 37, 38, 50, 63, 76,
		75, 89, 113, 128, 140, 153, 165, 179, 191, 192, 192, 191, 179, 178, 166, 152, 128, 113, 102, 89, 63, 64, 49, 38,
		38, 50, 50, 63, 76, 88, 114, 127, 141, 152, 166, 178, 192, 191, 192, 204, 192, 178, 166, 152, 153, 127, 114,
		101, 89, 75, 64, 63, 50, 49, 50, 49, 64, 75, 89, 101, 114, 127, 153, 165, 166, 178, 192, 204, 192, 192, 192,
		178, 166, 152, 141, 127, 114, 101, 89, 75, 64, 50, 49, 50, 63, 64, 76, 76, 89, 102, 127, 141, 140, 153, 166,
		179, 178, 179, 178, 179, 179, 166, 152, 141, 127, 114, 102, 88, 76, 63, 64, 49, 50, 49, 64, 63, 76, 75, 89, 102,
		114, 127, 141, 152, 166, 179, 178, 192, 191, 192, 192, 191, 179, 178, 166, 152, 141, 128, 113, 102, 89, 76, 63,
		64, 49, 50 };

const unsigned int phaser_wav_len = 1792;
unsigned int circuitboardpcb;
unsigned int newbuild;	// the (later) firmware build number on the server

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct netif *netif, *netif2;
extern const ip4_addr_t *ipaddr;
uint32_t t1sec = 0;
extern uint32_t t2cap[1];
int main_init_done = 0;
int lptask_init_done = 0;
char trigtimestr[32] = { "No Triggers" };
char nowtimestr[32] = { "\"No Time\"" };
char pressstr[16] = { "0" };
char tempstr[10] = { "0" };
char snstr[164] = { "\"No S/N\"" };
char statstr[264] = { "\"No status\"" };
char gpsstr[64] = { "\"No GPS data\"" };
uint16_t agc = 1;	// agc enable > 0
uint32_t myip;
uint32_t uip;		// udp target
QueueHandle_t consolerxq;
unsigned char con_ch;	// console uart input char

//char snstr[4] = {"No"};
//char statstr[4] = {"No"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	int DataIdx;

	if (file == 1) {
		for (DataIdx = 0; DataIdx < len; DataIdx++) {
			__io_putchar(*ptr++);
		}
	} else {
		for (DataIdx = 0; DataIdx < len; DataIdx++) {
			HAL_UART_Transmit(&huart5, (uint8_t*) *ptr++, 1, 10);
		}
	}
	return len;
}

#ifdef configGENERATE_RUN_TIME_STATS
void rtos_debug_timer_setup() {

	HAL_TIM_Base_Start_IT(&htim14);		// TIM14 used for RTOS profiling
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_DAC_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_I2C4_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  MX_TIM14_Init();
  MX_TIM5_Init();
  MX_UART8_Init();
  MX_UART7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM8_TRG_COM_TIM14_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
}

/* USER CODE BEGIN 4 */

// find the crc for the running firmware (printout needed for server)
crc_rom() {
	unsigned char *base;
	extern xcrc32(const unsigned char *buf, int len, unsigned int init);
	extern uint32_t __fini_array_end;
	extern uint32_t _edata, _sdata;

	uint32_t xinit = 0xffffffff;
	uint32_t length, romcrc;

	if ((unsigned long) MX_NVIC_Init < 0x8100000) {
		base = 0x8000000;
	} else {
		base = 0x8100000;
	}

	length = (uint32_t) &__fini_array_end - (uint32_t) base + ((uint32_t) &_edata - (uint32_t) &_sdata);
	romcrc = xcrc32(base, length, xinit);
	printf("XCRC=0x%08x, base=0x%08x, len=%d\n", romcrc, base, length);
}

err_leds(int why) {
	volatile int i;

	for (;;) {
		switch (why) {
		case 0:
			break;
		case 1:			// LAN disconnected
			HAL_GPIO_TogglePin(GPIOD, LED_D1_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D2_Pin);
			break;
		case 2:			// GPS not found
			HAL_GPIO_TogglePin(GPIOD, LED_D1_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D3_Pin);
			break;
		case 3:			// udp target not found
			HAL_GPIO_TogglePin(GPIOD, LED_D1_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D4_Pin);
			break;
		case 4:			// system crash - task exited
			HAL_GPIO_TogglePin(GPIOD, LED_D2_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D3_Pin);
			break;
		case 5:			// GPS stopped responding
			HAL_GPIO_TogglePin(GPIOD, LED_D2_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D4_Pin);
			break;
		case 6:			// server commanded a reboot
			HAL_GPIO_TogglePin(GPIOD, LED_D3_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D4_Pin);
			break;
		case 7:			// server lookup failed
			HAL_GPIO_TogglePin(GPIOD, LED_D4_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D5_Pin);
			break;
		case 8:			// serial number and udp target lookup failed
			HAL_GPIO_TogglePin(GPIOD, LED_D5_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_D1_Pin);
			break;
		}
		for (i = 0; i < 3500000; i++)
			;
	}
}

void rebootme(int why) {
	volatile unsigned int i;

	while (1) {
#ifdef HARDWARE_WATCHDOG
		__disable_irq();			// mask all interrupts
		err_leds(why);
#else
		err_leds(why);
		osDelay(6000);
		__NVIC_SystemReset();   // reboot
#endif
	}
}

void netif_status_callbk_fn(struct netif *netif) {

	printf("netif_status changed\n");
	/*	printf("netif: IP address of '%c%c' set to %"U16_F".%"U16_F".%"U16_F".%"U16_F"\n", netif->name[0],
	 netif->name[1], ip4_addr1_16(netif_ip4_addr(netif)), ip4_addr2_16(netif_ip4_addr(netif)),
	 ip4_addr3_16(netif_ip4_addr(netif)), ip4_addr4_16(netif_ip4_addr(netif)));
	 */

	/*	printf("netif: Gateway %"U16_F".%"U16_F".%"U16_F".%"U16_F"\n",
	 ip4_addr1_16(netif_ip4_gw(netif)),
	 ip4_addr2_16(netif_ip4_gw(netif)),
	 ip4_addr3_16(netif_ip4_gw(netif)),
	 ip4_addr4_16(netif_ip4_gw(netif)));
	 printf("netif: NetMask %"U16_F".%"U16_F".%"U16_F".%"U16_F"\n",
	 ip4_addr1_16(netif_ip4_netmask(netif)),
	 ip4_addr2_16(netif_ip4_netmask(netif)),
	 ip4_addr3_16(netif_ip4_netmask(netif)),
	 ip4_addr4_16(netif_ip4_netmask(netif)));
	 */
//	netif_set_addr(netif,netif_ip4_addr(netif),netif_ip4_gw(netif),netif_ip4_netmask(netif));
}

void netif_link_callbk_fn(struct netif *netif) {

	if (netif->flags & NETIF_FLAG_LINK_UP) {
		printf("netif_link UP, flags=0x%02x\n", netif->flags);
	} else {
		printf("netif_link DOWN, flags=0x%02x\n", netif->flags);
		if (!(netif_is_link_up(netif))) {
#ifndef TESTING
			printf("LAN interface appears disconnected, rebooting...\n");
			rebootme(1);
#endif
		}
	}
}

uint32_t movavg(uint32_t new) {
	static uint32_t data[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int i;
	uint32_t sum = 0;

	for (i = 0; i < 15; i++) {
		data[i] = data[i + 1];		// old data is low index
		sum += data[i];
	}
	data[15] = new;		// new data at the end
	sum += new;

	return (sum >> 4);
}

// dma for DAC finished callback
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) { // every second 1 pps (on external signal)
	uint32_t diff;
	static uint32_t lastcap = 0;

	if (htim->Instance == TIM2) {
		rtseconds = (statuspkt.NavPvt.sec + 1) % 60; // assume we get here before serial comms updates gps seconds field

#ifdef SPLAT1
		if (!(ledsenabled)) {
			HAL_GPIO_WritePin(GPIOD, LED_D1_Pin, GPIO_PIN_RESET);
		} else
			HAL_GPIO_TogglePin(GPIOD, LED_D1_Pin);
#else
	HAL_GPIO_TogglePin(GPIOB, LD1_Pin);		// green led
#endif

//printf(":%d ",rtseconds);
//		statuspkt.epochsecs++;
//		neotime();
#if 0		// clktrim with non-resetting pps counter
				if (lastcap > t2cap[0]) {		// counter wrapped
					diff = 4000000000 /*UINT_MAX*/- lastcap + t2cap[0];
				} else {
					diff = t2cap[0] - lastcap;
				}
#else
		diff = lastcap;
#endif
		statuspkt.clktrim = movavg(diff);
		/*		printf("TIM2 IC Callback CCR3=%08x, [0]%08xu, [1]%08x diff=%u, clktrim=%08x",
		 htim->Instance->CCR3, t2cap[0], lastcap, diff, statuspkt.clktrim);
		 printf(" globaladcavg=%u\n",globaladcavg);
		 */
		lastcap = t2cap[0];			// dma has populated t2cap from Channel 3 trigger on Timer 2
	} else if (htim->Instance == TIM4) {
		printf("Timer4 callback\n");
	}

//	htim->Instance->SR = 0;		// cheat

	/* USER CODE END Callback 1 */
}

// try to figure out what PCB is connected to the STM
// PC0 and PF5
void getboardpcb() {
	if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)) {// floats high on SPLAT1, so this must be a lightningboard
//		circuitboardpcb = LIGHTNINGBOARD1;		// prototype
        circuitboardpcb = LIGHTNINGBOARD2;		// Rev 1A and Rev 1B		// compile time!
	} else {
		circuitboardpcb = SPLATBOARD1;		// assumed
	}
}

void setupnotify() {
	/* Store the handle of the calling task. */
	xTaskToNotify = xTaskGetCurrentTaskHandle();
}

void printaline(char *str) {
	printf("%s----------------------------------------------------------------------------\n", str);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

#ifdef configGENERATE_RUN_TIME_STATS

	if (htim->Instance == TIM14) {				// TIM14 used for RTOS profiling
//		printf("T14 PeriodElapsedCallback\n");
		rtos_debug_timer++;
		return;
	}
#endif

	if (htim->Instance == TIM5) {// TIM5 interrupt is used as hook to run ADC_Conv_complete() at a lower IRQ  priority than dmacomplete

//		printf("T5\n");
		ADC_Conv_complete();			// It is a one-shot
		return;
	}

	if (htim->Instance == TIM2) {
		printf("T2P PeriodElapsedCallback %lu %lu\n", t2cap[0], statuspkt.clktrim);
		return;
	}
	if (htim->Instance == TIM3) {
		printf("T3 PeriodElapsedCallback\n");
		return;
	}

	if (htim->Instance == TIM6) { // 1 second (internally timed, not compensated by GPS)
//		printf("T6 PeriodElapsedCallback %u SR=%u\n", myfullcomplete, TIM6->SR);
		t1sec++;
		statuspkt.sysuptime++;
		if (netup)
			statuspkt.netuptime++;
		if (gpslocked) {
			statuspkt.gpsuptime++;
			/*	if (epochvalid == 0) */{
				statuspkt.epochsecs = calcepoch32();
				epochvalid = 1;
			}
		} else {
			statuspkt.gpsuptime = 0;	// gps uptime is zero
			statuspkt.epochsecs = 0;	// make epoch time obviously wrong
			epochvalid = 0;
		}
		return;
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else {
		printf("Unknown Timer Period Elapsed callback\n");
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		printf("HAL error (main.c 2343)\n");
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
