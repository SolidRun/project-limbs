/*
    DMA-accelerated multi-UART USB CDC for STM32F072 microcontroller

    Copyright (C) 2015,2016 Peter Lawrence

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#include "usbd_desc.h"
#include "usbd_composite.h"
#include "stm32f0xx_hal_uart.h"

static void SystemClock_Config(void);
static void MX_GPTO_Init(void);
static void MX_USART2_UART_Init(void);
void rprint (const char *str);


USBD_HandleTypeDef USBD_Device;
static UART_HandleTypeDef s_UARTHandle;

#define BUFF_SIZE 10
uint8_t bufftx[BUFF_SIZE]="Hello!!\n\r";
UART_HandleTypeDef huart2;
/* for use Transmit uart2 */
#include "usbd_cdc.h"
#include "config.h"
//extern USBD_CDC_HandleTypeDef context[NUM_OF_CDC_UARTS];
UART_HandleTypeDef huart4;
/* End*/

void rprint (const char *str) {
  #if 1
    int i = 0;
    for (int j=0;j<100;j++){
      while (str[i]) {
        //HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
          HAL_UART_Transmit(&s_UARTHandle, (unsigned char *)&str[i], 1, HAL_MAX_DELAY);
          if (str[i] == '\n') HAL_UART_Transmit(&s_UARTHandle, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
          i++;
        }
        i=0;
    }
  #endif
  #if 0
    char *msg = "Hello Nucleo Fun!\n\r";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  #endif
}

#if 0
  void MX_USART2_UART_Init(void)
  {
      huart2.Instance = USART2;
      huart2.Init.BaudRate = 115200;
      huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
      huart2.Init.Parity = UART_PARITY_NONE;
      huart2.Init.Mode = UART_MODE_TX_RX;
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      HAL_UART_Init(&huart2);
  }
#endif

#if 0
    __USART2_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = GPIO_AF1_USART2;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    s_UARTHandle.Instance        = USART2;
    s_UARTHandle.Init.BaudRate   = 115200;
    s_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
    s_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
    s_UARTHandle.Init.Parity     = UART_PARITY_NONE;
    s_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    s_UARTHandle.Init.Mode       = UART_MODE_TX_RX;

    s_UARTHandle.State = HAL_UART_STATE_BUSY;
    if (HAL_UART_Init(&s_UARTHandle) != HAL_OK){
        rprint ("HAL Not Ok there\n");
        while(1){};//asm("bkpt 255");
      }
    rprint ("Hello there\n");

#endif


static void MX_GPTO_Init(void){ __GPIOA_CLK_ENABLE(); }

int main(void)
{
  /*
  With code compiled outside Rowley, I'm seeing the USB ISR fire between USBD_Init() and USBD_RegisterClass().
  Interrupts are enabled at reset, and ST's (mis)decision is to start enabling NVIC interrupts in USBD_Init().
  By disabling interrupts here, and enabling later when everything is ready, we avoid this race condition.
  */
  __disable_irq();
  /* STM32F0xx HAL library initialization */
  HAL_Init();
  /* configure the system clock to get correspondent USB clock source */
  SystemClock_Config();

  /* remap flash - critical when booting via DFU */
    __HAL_REMAPMEMORY_FLASH();

    // USBD_Device : start from the main, USB Devices handle, USBD_HandleTypeDef struct from usbd_def.h
  /* Initialize Device Library */
  // USBD_DescriptorsTypeDef USBD_Desc :start from the usbd_desc.c struct contain USBs Device Discreptors & function pointers...
  // USB Device initialize and assign the Descriptor to the USB_device also config the PMA enpoints for all cdcs
  USBD_Init(&USBD_Device, &USBD_Desc, 0);

  /* to work within ST's drivers, I've written a special USBD_Composite class that then invokes several classes */
  //  USBD_Device : start from the main, USB Devices handle, struct from usbd_def.h after init usb device with the fun pointers
  // USBD_Composite : interface class callbacks structure writen from usbd_composite.c and it contain the Tx/Rx functions
  //  link the class to the USB Device handle
  // USBD_Device->pClass = USBD_Composite;
  USBD_RegisterClass(&USBD_Device, &USBD_Composite);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  /* OK, only *now* it is OK for the USB interrupts to fire */
  __enable_irq();

/* Debug Range  */
#if 1
  MX_GPTO_Init();
  //MX_USART2_UART_Init();
  //HAL_UART_MspInit(&huart2);
  //UART_HandleTypeDef huart3 = (context[0]).UartHandle;
  //USBD_CDC_HandleTypeDef *hcdc2 = context;
  //hcdc2++;
  //UART_HandleTypeDef huart3 = hcdc2->UartHandle;
  //UART_HandleTypeDef huart3 = context[1].UartHandle;
#endif

#if 1
  //HAL_UART_Transmit_DMA(&huart3, bufftx, BUFF_SIZE);
  HAL_UART_Transmit_DMA(&huart4, bufftx, BUFF_SIZE);
  //HAL_UART_Transmit(&huart3, bufftx, BUFF_SIZE, 100);
  //HAL_Delay(200);
#endif

  // inifinit loop
  for (;;)
  {
    // instructions for entering low-power standby state where most clocks
    // use WFI for the interupt and sleep mode, wakeup ....
    //WFI is targeted at entering either standby, dormant or shutdown mode, where an interrupt is
    //required to wake-up the processor.
    HAL_UART_Transmit_DMA(&huart4, bufftx, BUFF_SIZE);
    __WFI();
  }

}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  static RCC_CRSInitTypeDef RCC_CRSInitStruct;

  /* Enable HSI48 Oscillator to be used as system clock source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select HSI48 as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select HSI48 as system clock source and configure the HCLK and PCLK1 clock dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  /*Configure the clock recovery system (CRS)**********************************/

  /*Enable CRS Clock*/
  __CRS_CLK_ENABLE();

  /* Default Synchro Signal division factor (not divided) */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

  /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

  /* HSI48 is synchronized with USB SOF at 1KHz rate */
  RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_CALCULATE_RELOADVALUE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

  /* Set the TRIM[5:0] to the default value*/
  RCC_CRSInitStruct.HSI48CalibrationValue = 0x20;

  /* Start automatic synchronization */
  HAL_RCCEx_CRSConfig (&RCC_CRSInitStruct);
}
