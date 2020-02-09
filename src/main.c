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
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal.h"
#include "stm32f042x6.h"

/* for use Transmit uart2 */
#include "usbd_cdc.h"

#include "config.h"
#if ADC_ENABLE
  #include "adc.h"
#endif

#if SPI_ENABLE
  #include "spi.h"
#endif

static void SystemClock_Config(void);
static void MX_GPTO_Init(void);
static void MX_USART2_UART_Init(void);

USBD_HandleTypeDef USBD_Device;
static UART_HandleTypeDef s_UARTHandle;

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
  MX_GPTO_Init();
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

  #if ADC_ENABLE
    /* Init ADC */
    MX_ADC_Init();
  #endif

  #if SPI_ENABLE
    /* Init SPI */
    MX_SPI1_Init();
  #endif


/* Debug Range  */

  // inifinit loop
  for (;;)
  {
    // instructions for entering low-power standby state where most clocks
    // use WFI for the interupt and sleep mode, wakeup ....
    //WFI is targeted at entering either standby, dormant or shutdown mode, where an interrupt is
    //required to wake-up the processor.
    __WFI();

  }

}

static void MX_GPTO_Init(void)
{

    /* STM Pins Description
    PA2 - TX2                     PB0 - ADC_IN8 (VOLTAGE)
    PA3 - RX2                     PB1 - ADC_IN9 (CURRENT)
    PA4  - SPI1_CS                PB2 - SPI1_SWITCH
    PA5  - SPI1_SCK               PB3 - VBAT_CONNECTED
    PA6  - SPI1_MISO              PB4 - MASTER_RESET
    PA7  - SPI1_MOSI              PB5 - PWR_BTN
    PA9  - TX1                    PB6 - I2C1_SCK
    PA10 - RX1                    PB7 - I2C1_SDA
    PA11 - USB_FT_DM (mUSB)
    PA12 - USB_FT_DP (mUSB)
    PA15 - FORCE_RECOVERY_BOOT

    GPIO_MODE_INPUT   // Input Floating Mode
    GPIO_MODE_OUTPUT_PP < Output Push Pull Mode >
    GPIO_MODE_OUTPUT_OD < Output Open Drain Mode >
    GPIO_MODE_AF_PP  < Alternate Function Push Pull Mode >
    GPIO_MODE_AF_OD
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* GPIO Ports Clock Enable */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_3, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pins : PA2 PA3 (USART2) */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB2 PB3(VBAT_CONNECTED) PB4(MASTER_RESET)  PB5(PWR_BTN) */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PB6 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
