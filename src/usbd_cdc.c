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

#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_composite.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "adc.h"

#include "spi.h"
#include "w25qxx.h"

/* USB handle declared in main.c */
extern USBD_HandleTypeDef USBD_Device;

/* CDC buffers declaration for VCP */
static int8_t vcp_cmd_control(USBD_HandleTypeDef *pdev,uint8_t ep_addr, uint8_t* pbuf, uint16_t length);
//static uint8_t vcp_cmd_control( uint8_t* pbuf, uint16_t length);
#define BUF_SIZE 11
// TX
uint8_t vcp_tx[BUF_SIZE];
uint16_t countTx=0;
uint16_t writePointerTx=0, readPointerTx=0;
// RX
uint8_t vcp_rx[BUF_SIZE];
uint16_t countRx=0;
uint16_t writePointerRx=0, readPointerRx=0;
uint8_t res[7]="ls\n\r";

/* ADC Help */
#if ADC_ENABLE
  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
  uint16_t ADC_Read(void);
  uint16_t adcvalue=0;
  #define ADC_VALUE_SIZE 7
  char adc_buff[ADC_VALUE_SIZE];
#endif

/* SPI Help */
#if SPI_ENABLE

  #if 0
    /* remove from spi.c to usbd_cdc.c */
    // SPI transfer Status
    enum {
      TRANSFER_WAIT,
      TRANSFER_COMPLETE,
      TRANSFER_ERROR
    };
    /* transfer state */
    extern __IO uint32_t wTransferState;
    #define SPI_BUFFERSIZE 8
    /* Buffer used for transmission */
    extern uint8_t spiTxBuffer[SPI_BUFFERSIZE];
    /* Buffer used for reception */
    extern uint8_t spiRxBuffer[SPI_BUFFERSIZE];
  #endif

#endif

/* local function prototyping */

static uint8_t USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t USBD_CDC_SOF (struct _USBD_HandleTypeDef *pdev);
static void USBD_CDC_PMAConfig(PCD_HandleTypeDef *hpcd, uint32_t *pma_address);

static USBD_StatusTypeDef USBD_CDC_ReceivePacket (USBD_HandleTypeDef *pdev, unsigned index);
static USBD_StatusTypeDef USBD_CDC_TransmitPacket (USBD_HandleTypeDef *pdev, unsigned index, uint16_t offset, uint16_t length);

static int8_t CDC_Itf_Control (USBD_CDC_HandleTypeDef *hcdc, uint8_t cmd, uint8_t* pbuf, uint16_t length);
static void Error_Handler (void);
static void ComPort_Config (USBD_CDC_HandleTypeDef *hcdc);

/* CDC interface class callbacks structure that is used by main.c */
const USBD_CompClassTypeDef USBD_CDC =
{
  .Init                  = USBD_CDC_Init,
  .DeInit                = USBD_CDC_DeInit,
  .Setup                 = USBD_CDC_Setup,
  .EP0_TxSent            = NULL,
  .EP0_RxReady           = USBD_CDC_EP0_RxReady,
  .DataIn                = USBD_CDC_DataIn,
  .DataOut               = USBD_CDC_DataOut,
  .SOF                   = USBD_CDC_SOF,
  .PMAConfig             = USBD_CDC_PMAConfig,
};

/*
parameters for this CDC implementation
*/

/* default state of all UARTs when first initialized */
// Endpoint 0 for configuration.
static const USBD_CDC_LineCodingTypeDef defaultLineCoding =
{
  .bitrate    = 115200, /* baud rate */
  .format     = 0x00,   /* stop bits-1 */
  .paritytype = 0x00,   /* parity - none */
  .datatype   = 0x08    /* nb. of bits 8 */
};

/* endpoint numbers and "instance" (base register address) for each UART */
//  PMA_address (hardcoded as 0x18, 0x58...)
// Consists of using two buffers in PMA (buffer0 and buffer1) at any time CPU should Consists of using two buffers in PMA (buffer0 and buffer1), at any time CPU should
// be accessing one buffer (for R/W) while USB IP is accessing the other buffer
static const struct
{
  USART_TypeDef *Instance;
  uint8_t data_in_ep, data_out_ep, command_ep, command_itf;
} parameters[NUM_OF_CDC_UARTS] =
// Endpoint Address
{
#if (NUM_OF_CDC_UARTS > 0)
  {
    .Instance = USART1,
    .data_in_ep  = 0x81,
    .data_out_ep = 0x01,
    .command_ep  = 0x82,
    .command_itf = 0x00,
  },
#endif
#if (NUM_OF_CDC_UARTS > 1)
  {
    .Instance = USART2,
    .data_in_ep  = 0x83,
    .data_out_ep = 0x03,
    .command_ep  = 0x84,
    .command_itf = 0x02,
  },
#endif
#if 1 // #VCP
#if (NUM_OF_CDC_UARTS > 2)
  {
    .Instance = USART3,
    .data_in_ep  = 0x85,
    .data_out_ep = 0x05,
    .command_ep  = 0x86,
    .command_itf = 0x04,
  },
#endif
#endif
};

/* context for each and every UART managed by this CDC implementation */
// context this base array contain all CDC_devices Handle
static USBD_CDC_HandleTypeDef context[NUM_OF_CDC_UARTS];

// it's function call from composite.c to init, from fun USBD_Composite_Init()
static uint8_t USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (index == 2){ // #VCP
      // TO DO
    #if 1
      /* Open EP IN */
      USBD_LL_OpenEP(pdev, parameters[index].data_in_ep, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE); // when use in_ep=0x85 the acm2 Not work fine

      /* Open EP OUT */
      USBD_LL_OpenEP(pdev, parameters[index].data_out_ep, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);

      /* Open Command IN EP */
      USBD_LL_OpenEP(pdev, parameters[index].command_ep, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    #endif

      /* Configure the UART peripheral */
      hcdc->InboundBufferReadIndex = 0;
      hcdc->InboundTransferInProgress = 0;
      hcdc->OutboundTransferNeedsRenewal = 0;
      hcdc->UartHandle.Instance = parameters[index].Instance;
      hcdc->LineCoding = defaultLineCoding;
      __HAL_LINKDMA(&hcdc->UartHandle, hdmatx, hcdc->hdma_tx);
      __HAL_LINKDMA(&hcdc->UartHandle, hdmarx, hcdc->hdma_rx);
      //ComPort_Config(hcdc);

      /* Prepare Out endpoint to receive next packet */
      USBD_CDC_ReceivePacket(pdev, index);

    }
    else
    {
      /* Open EP IN */
      USBD_LL_OpenEP(pdev, parameters[index].data_in_ep, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);

      /* Open EP OUT */
      USBD_LL_OpenEP(pdev, parameters[index].data_out_ep, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);

      /* Open Command IN EP */
      USBD_LL_OpenEP(pdev, parameters[index].command_ep, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
      /* Configure the UART peripheral */
      hcdc->InboundBufferReadIndex = 0;
      hcdc->InboundTransferInProgress = 0;
      hcdc->OutboundTransferNeedsRenewal = 0;
      hcdc->UartHandle.Instance = parameters[index].Instance;
      hcdc->LineCoding = defaultLineCoding;
      __HAL_LINKDMA(&hcdc->UartHandle, hdmatx, hcdc->hdma_tx);
      __HAL_LINKDMA(&hcdc->UartHandle, hdmarx, hcdc->hdma_rx);
      ComPort_Config(hcdc);

      /* Prepare Out endpoint to receive next packet */
      USBD_CDC_ReceivePacket(pdev, index);

    }
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (index == 2){ // #VCP
      // it's should be 2 when we know what to do here !!
      // TO DO
    }
    else
    {
      /* Close EP IN */
      USBD_LL_CloseEP(pdev, parameters[index].data_in_ep);

      /* Close EP OUT */
      USBD_LL_CloseEP(pdev, parameters[index].data_out_ep);

      /* Close Command IN EP */
      USBD_LL_CloseEP(pdev, parameters[index].command_ep);

      /* DeInitialize the UART peripheral */
      if (hcdc->UartHandle.Instance)
        if(HAL_UART_DeInit(&hcdc->UartHandle) != HAL_OK)
        {
          /* Initialization Error */
          Error_Handler();
        }
    }
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;
  //rprint ("In CDC setup\n");
  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
      if (parameters[index].command_itf != req->wIndex)
        continue;

      if (index == 2){ // #VCP
        // VCP Setup
        // TO DO
      }
      else
      {
        switch (req->bmRequest & USB_REQ_TYPE_MASK)
        {
        case USB_REQ_TYPE_CLASS :
          if (req->wLength)
          {
            if (req->bmRequest & 0x80)
            {
              CDC_Itf_Control(hcdc, req->bRequest, (uint8_t *)hcdc->SetupBuffer, req->wLength);
              USBD_CtlSendData (pdev, (uint8_t *)hcdc->SetupBuffer, req->wLength);
            }
            else
            {
              hcdc->CmdOpCode = req->bRequest;
              hcdc->CmdLength = req->wLength;

              USBD_CtlPrepareRx (pdev, (uint8_t *)hcdc->SetupBuffer, req->wLength);
            }
          }
          else
          {
              CDC_Itf_Control(hcdc, req->bRequest, NULL, 0);
          }
          break;

        default:
          break;
        }
      }

    break;

  }

  return USBD_OK;
}

static uint8_t USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;
  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    #if 0
    // Data in UART3
    if (parameters[index].data_in_ep == 0x85) // #VCP
    {
      // TO DO
      break;
    }
    #endif

    if (parameters[index].data_in_ep == (epnum | 0x80))
    {
      hcdc->InboundTransferInProgress = 0;
      break;
    }
  }

  return USBD_OK;
}

// help functions - compare between two strings
int my_strcmp(char *strg1, char *strg2)
{
    while( ( *strg1 != '\0' && *strg2 != '\0' ) && *strg1 == *strg2 )
    {
        strg1++;
        strg2++;
    }
    if(*strg1 == *strg2)
    {
        return 0; // strings are identical
    }
    else
    {
        return *strg1 - *strg2;
    }
}

#if ADC_ENABLE

  uint16_t ADC_Read(void)
  {
    uint16_t adcVal;
    // enable ADC and start ADC conversion
    HAL_ADC_Start(&hadc);
    // waith until ADC conversion to be completed
    HAL_ADC_PollForConversion(&hadc, 1);
    // get ADC value from ADC register
    adcVal = HAL_ADC_GetValue(&hadc);
    // stop ADC conversion and disable ADC
    HAL_ADC_Stop(&hadc);
    sConfig.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
    return adcVal;
  }

#if ADC_IT_MODE
  /* if you need use it mode can write the code in the fun HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
    and can read the adc value adcVal = HAL_ADC_GetValue(&hadc);
    start convertion use Interrupts Mode
  */
  HAL_ADC_Start_IT(&hadc);// Start ADC1 under Interrupt
  // Callback Fun when ADC conversion completed
  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
    adcvalue = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Start_IT(&hadc); // Re-Start ADC1 under Interrupt
                            // this is necessary because we don'use
                            // the Continuos Conversion Mode
  }
#endif

// help functions - Convert string number to int
uint8_t StringToInt(char a[]) {
  uint8_t c, n;
  n = 0;
  for (c = 0; a[c] != '\0'; c++) {
    n = n * 10 + a[c] - '0';
  }
  return n;
}

void Current_Cmd(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
{
  /** Configure for the selected ADC regular channel to be converted.**/
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      Error_Handler_ADC();
    }
    /* read adc value */
    adcvalue=ADC_Read();
    double adc_current=(double)adcvalue * CURRENT_ADC_FACTORE;
    /* convert double to Str */
    itoa(adc_current,adc_buff,10);
    const char * point=".";
    char lower_buff[4];
    uint16_t lower_adc_current=(adc_current-(double)StringToInt(adc_buff))*1000;
    itoa(lower_adc_current,lower_buff,10);
    /* show result value */
    const char * A_new_line="A\r\n";
    char * str_adc_current=strcat(strcat(adc_buff,point) ,strcat(lower_buff, A_new_line));
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)str_adc_current, ADC_VALUE_SIZE+2);
}

void Voltage_Cmd(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
{
  /** Configure for the selected ADC regular channel to be converted.**/
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
      Error_Handler_ADC();
    /* read adc value */
    adcvalue=ADC_Read();
    double adc_voltage=(double)adcvalue * VOLTAGE_ADC_FACTORE;
    /* convert double to Str */
    //char upper_buff[ADC_VALUE_SIZE];
    itoa(adc_voltage,adc_buff,10);
    const char * point=".";
    char lower_buff[4];
    uint16_t lower_adc_voltage=(adc_voltage-(double)StringToInt(adc_buff))*1000;
    itoa(lower_adc_voltage,lower_buff,10);
    /* show result value */
    const char * v_new_line="v\r\n";
    char * str_adc_voltage=strcat(strcat(adc_buff,point) ,strcat(lower_buff, v_new_line));
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)str_adc_voltage, ADC_VALUE_SIZE+2);
}

#endif

#if SPI_ENABLE

  /* help function to reverse string */
  void reverse(char *x, uint8_t begin, uint8_t end)
  {
     char c;

     if (begin >= end)
        return;

     c          = *(x+begin);
     *(x+begin) = *(x+end);
     *(x+end)   = c;

     reverse(x, ++begin, --end);
  }

  volatile uint32_t jedec_id=0;
  /* show the W25qxx spi flash JEDEC ID (0x40ef16 -> W25Q32) */
  void spi_flash_id(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
  {
    jedec_id=W25qxx_ReadID();
    reverse((char*) &jedec_id,0,3);
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t*) &jedec_id, 4);
  }

  void spi_test(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
  {
    if (w25qxx.ID == W25Q32){
      USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)res,4);
    }

    HAL_Delay(100);
    jedec_id=W25qxx_ReadID();
    reverse((char*) &jedec_id,0,3);
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t*) &jedec_id, 4);
    //reverse((char*) &w25qxx.JEDEC_ID,0,3);
    //USBD_LL_Transmit(pdev,ep_addr,(uint8_t*) &w25qxx.JEDEC_ID, 4);

    W25qxx_WriteByte(0x97, 0x0000100a);
    //uint8_t pBuffer[2]="c\n";
    uint8_t pBuffer=0x63;
    W25qxx_ReadByte(&pBuffer ,0x0000100a);
    HAL_Delay(100);
    USBD_LL_Transmit(pdev,ep_addr,&pBuffer, 1);

  }
    //USBD_LL_Transmit(pdev,ep_addr,w25qxx.UniqID, 8);
    //itoa(w25qxx.BlockCount,adc_buff,4);
    //USBD_LL_Transmit(pdev,ep_addr, (uint8_t *)adc_buff, 4);

    //SELECT();
    /*
    if( flage_ini_w25q==0 )
    {
      flage_ini_w25q=1;
      W25qxx_Init();
      USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)res, 5);
    }
    */
    //W25qxx_Init();
    //W25qxx_ReadUniqID();
    //if (w25qxx.ID == W25Q32){
      //USBD_LL_Transmit(pdev,ep_addr,& w25qxx.ID, 1);
      //HAL_Delay(100);
      //USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)res, 5);
      //HAL_Delay(100);
      //USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)res, 5);
    //}
    //else
    //{
      //USBD_LL_Transmit(pdev,ep_addr, w25qxx.UniqID,8 );
    //}
    //W25qxx_ReadUniqID();
    //HAL_Delay(100);
    //USBD_LL_Transmit(pdev,ep_addr,w25qxx.UniqID, 8);
    //w25qxx.JEDEC_ID=W25qxx_ReadID();
    //HAL_Delay(100);
    //USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)res, 5);

     //W25qxx_WriteByte(0x97, 0x0000000a);
     //uint8_t pBuffer[2]="c\n";
     //uint8_t pBuffer=0x63;
     //W25qxx_ReadByte(&pBuffer ,0x0000000a);
     //USBD_LL_Transmit(pdev,ep_addr,&pBuffer, 1);
  //}

  #if 0
      /**
      * @brief  Compares two buffers.
      * @param  pBuffer1, pBuffer2: buffers to be compared.
      * @param  BufferLength: buffer's length
      * @retval 0  : pBuffer1 identical to pBuffer2
      *         >0 : pBuffer1 differs from pBuffer2
      */
    static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
    {

      /* Compare the two strings provided up to the first 9 characters */
      int result = memcmp(pBuffer1, pBuffer2, 9);
      /* If the two arrays are the same say so */
      if (result == 0) // Arrays are the same
        return 0;
      else
        return -1 ;

      while (BufferLength--)
      {
        if((*pBuffer1) != *pBuffer2)
        {
          return BufferLength;
        }
        pBuffer1++;
        pBuffer2++;
      }

      return 0;
    }
  #endif
#endif


static int8_t vcp_cmd_control(USBD_HandleTypeDef *pdev,uint8_t ep_addr, uint8_t* pbuf, uint16_t length) // #VCP
{
  /* Comands ID */
  enum CMD_ID{
        RESET,
        POWERON,
        POWEROFF,
        POWERBTN,
        VBATON,
        VBATOFF,
        CURRENT,
        VOLTAGE,
        SPI_SW_STM,
        SPI_SW_COM,
        SPI_TEST,
        SPI_ID,
        CMD_NUM
  };

  //#define CMD_NUM 13
  char *arr_cmd[] = {
     "reset",
     "poweron",
     "poweroff",
     "powerbtn",
     "vbaton",
     "vbatoff",
     "current",
     "voltage",
     "spi_sw_stm",
     "spi_sw_com",
     "spi_test",
     "spi_id",
     "else"
   };

  // CMD string to CMD_ID
  int8_t cmd_id=-1;
  for (int8_t i=0; i <= CMD_NUM; i++)
  {
    cmd_id=i;
    //if(strcmp((char*)pbuf,(char*)arr_cmd[i]) == 0) break;
    if(my_strcmp((char*)pbuf,(char*)arr_cmd[i]) == 0) break;
  }
    //char s[100];
    //printf("Reverse of the string: %s\n", s);
    // USER CODE BEGIN 5 - Execute the CMD
    switch(cmd_id)
    {
      case RESET:
        /* Short Press RESET Button */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        break;
      case POWERON:
        /* Short Press POWER-ON Button */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
        break;
      case POWEROFF:
        /* Long Press POWER-OFF Button */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        HAL_Delay(10000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
        break;
      case POWERBTN:
        /* Press/unpress POWER-OFF Button */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        break;
      case VBATON:
        /* Enable the RTC Battery Voltage */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
        break;
      case VBATOFF:
        /* Disable the RTC Battery Voltage */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
        break;
      case CURRENT:
        /* Read Total Current Input */
        #if ADC_ENABLE
          Current_Cmd(pdev,ep_addr);
        #endif
        break;
      case VOLTAGE:
        /* Read Total Voltage Input */
        #if ADC_ENABLE
          Voltage_Cmd(pdev,ep_addr);
        #endif
        break;
      case SPI_SW_STM:
        /* Connect the spi-flash to the STM32 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
        break;
      case SPI_SW_COM:
        /* Connect the spi-flash to the COM */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
        break;
      case SPI_TEST:
        spi_test(pdev,ep_addr);
        break;
      case SPI_ID:
        spi_flash_id(pdev,ep_addr);
        break;
      case CMD_NUM:
        break;
      default:
        //return (USBD_FAIL);
        break;
    }

    return (USBD_OK);
    /* USER CODE END 5 */
}

const char hi[]="hi\n\r";
const char TEST[]="ls\n\r";
uint8_t new_line[3]="\n\r";
static uint8_t USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  uint32_t RxLength;
  unsigned index;
  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (parameters[index].data_out_ep == epnum)
    {
        /* Get the received data length */
        RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
        HAL_UART_Transmit_DMA(&hcdc->UartHandle, (uint8_t *)hcdc->OutboundBuffer, RxLength);

      if (hcdc->UartHandle.Instance == USART2) // #VCP
      {
        uint8_t * outbuff = (uint8_t *)hcdc->OutboundBuffer;
        for ( uint8_t i=0; i< (uint8_t)RxLength; i++,outbuff++)
        {
           //0:when need to use the USART2 and VCP in the same interface ACM1
          #if USART2_VCP_REPRINT_ENABLE
          // reprint
          if ( (char) *outbuff == '\n' || (char) *outbuff == (char)'\r' || *outbuff == (uint8_t)'\n' || (uint8_t)*outbuff == 10 || (uint8_t)*outbuff == 13)
            USBD_LL_Transmit(pdev,parameters[index].data_in_ep,new_line, 2);
          else
            USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)hcdc->OutboundBuffer, RxLength);
          #endif

          #if 1
          // reset cmd_id
          if ( (char) *outbuff == '1' )
          {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
          }
          // poweron cmd_id
          if ( (char) *outbuff == '2' )
          {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
          }
          // poweroff cmd_id
          if ( (char) *outbuff == '3' )
          {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
            HAL_Delay(10000);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
          }
          // powerbtn
          if ( (char) *outbuff == '4' )
          {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
          }
          // voltage
          if ( (char) *outbuff == '5' )
          {

            sConfig.Channel = ADC_CHANNEL_8;
            if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
            {
              Error_Handler_ADC();
            }

            adcvalue=ADC_Read();
            double d_adcvalue=(double)adcvalue * VOLTAGE_ADC_FACTORE;
            //itoa(adcvalue,adc_buff,10);
            //USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)adc_buff, ADC_VALUE_SIZE);
            itoa(d_adcvalue,adc_buff,10);

            const char * point=".";
            char adc_buff2[4];
            uint16_t d_adcvalue2=(d_adcvalue-(double)StringToInt(adc_buff))*1000;
            itoa(d_adcvalue2,adc_buff2,10);

            const char * v_new_line="v\r\n";
            char * str_adc_value=strcat(strcat(adc_buff,point) ,strcat(adc_buff2, v_new_line));
            //char * str_adc_value=strcat(adc_buff, v_new_line);
            //sprintf(adc_buff,"%hd\n",(short)adcvalue);
            USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)str_adc_value, ADC_VALUE_SIZE+2);

          }
          // current
          if ( (char) *outbuff == '6' )
          {
            /** Configure for the selected ADC regular channel to be converted.
            */

            sConfig.Channel = ADC_CHANNEL_9;
            if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
            {
              Error_Handler_ADC();
            }
            adcvalue=ADC_Read();
            double d_adcvalue=(double)adcvalue * VOLTAGE_ADC_FACTORE;
            //itoa(adcvalue,adc_buff,10);
            //USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)adc_buff, ADC_VALUE_SIZE);
            itoa(d_adcvalue,adc_buff,10);

            const char * point=".";
            char adc_buff2[4];
            uint16_t d_adcvalue2=(d_adcvalue-(double)StringToInt(adc_buff))*1000;
            itoa(d_adcvalue2,adc_buff2,10);

            const char * v_new_line="v\r\n";
            char * str_adc_value=strcat(strcat(adc_buff,point) ,strcat(adc_buff2, v_new_line));
            //char * str_adc_value=strcat(adc_buff, v_new_line);
            //sprintf(adc_buff,"%hd\n",(short)adcvalue);
            USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)str_adc_value, ADC_VALUE_SIZE+2);
          }

          // SPI
          if ( (char) *outbuff == '7' )
          {
            jedec_id=W25qxx_ReadID();
            reverse((char*) &jedec_id,0,3);
            USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t*) &jedec_id, 4);
          }
          // SPI - Test
          if ( (char) *outbuff == '9' )
          {
            SELECT();
          }
          // SPI - Test
          if ( (char) *outbuff == 'a' )
          {
            DESELECT();
          }
          #endif

          // input managment
          if ( (char) *outbuff == '\n' || (char) *outbuff == ' ' ||(char) *outbuff == '-' || countRx >= BUF_SIZE-1 )
          {
            vcp_rx[writePointerRx]=(uint8_t)'\0';
            vcp_cmd_control(pdev,parameters[index].data_in_ep,vcp_rx,countRx);
            #if USART2_VCP_REPRINT_ENABLE
              USBD_LL_Transmit(pdev,parameters[index].data_in_ep,vcp_rx,countRx);
            #endif
            writePointerRx=0;
            countRx=0;
          }
          else {
            vcp_rx[writePointerRx]=(uint8_t)*outbuff;
            writePointerRx++;
            countRx++;
          }

        }

      }

    break;
    }
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_SOF (struct _USBD_HandleTypeDef *pdev)
{
  uint32_t buffsize, write_index;
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    write_index = INBOUND_BUFFER_SIZE - hcdc->hdma_rx.Instance->CNDTR;

    /* the circular DMA should reset CNDTR when it reaches zero, but just in case it is briefly zero, we fix the value */
    if (INBOUND_BUFFER_SIZE == write_index)
      write_index = 0;

    if(hcdc->InboundBufferReadIndex != write_index)
    {
      if(hcdc->InboundBufferReadIndex > write_index)
      {
        /* write index has looped around, so send partial data from the write index to the end of the buffer */
        buffsize = INBOUND_BUFFER_SIZE - hcdc->InboundBufferReadIndex;
      }
      else
      {
        /* send all data between read index and write index */
        buffsize = write_index - hcdc->InboundBufferReadIndex;
      }

      if(USBD_CDC_TransmitPacket(pdev, index, hcdc->InboundBufferReadIndex, buffsize) == USBD_OK)
      {
        hcdc->InboundBufferReadIndex += buffsize;
        /* if we've reached the end of the buffer, loop around to the beginning */
        if (hcdc->InboundBufferReadIndex == INBOUND_BUFFER_SIZE)
        {
          hcdc->InboundBufferReadIndex = 0;
        }
      }
    }

    if (hcdc->OutboundTransferNeedsRenewal) /* if there is a lingering request needed due to a HAL_BUSY, retry it */
      USBD_CDC_ReceivePacket(pdev, index);
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (parameters[index].command_itf != pdev->request.wIndex)
      continue;

    if (hcdc->CmdOpCode != 0xFF)
    {
      CDC_Itf_Control(hcdc, hcdc->CmdOpCode, (uint8_t *)hcdc->SetupBuffer, hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFF;
    }

    break;
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, unsigned index, uint16_t offset, uint16_t length)
{
  USBD_StatusTypeDef outcome;

  if (context[index].InboundTransferInProgress)
    return USBD_BUSY;

  /* Transmit next packet */
  outcome = USBD_LL_Transmit(pdev, parameters[index].data_in_ep, (uint8_t *)(context[index].InboundBuffer) + offset, length);

  if (USBD_OK == outcome)
  {
    /* Tx Transfer in progress */
    context[index].InboundTransferInProgress = 1;
  }

  return outcome;
}

static uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev, unsigned index)
{
  USBD_StatusTypeDef outcome;

  outcome = USBD_LL_PrepareReceive(pdev, parameters[index].data_out_ep, (uint8_t *)context[index].OutboundBuffer, CDC_DATA_OUT_MAX_PACKET_SIZE);

  context[index].OutboundTransferNeedsRenewal = (USBD_OK != outcome); /* set if the HAL was busy so that we know to retry it */

  return outcome;
}

static int8_t CDC_Itf_Control (USBD_CDC_HandleTypeDef *hcdc, uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    hcdc->LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
    hcdc->LineCoding.format     = pbuf[4];
    hcdc->LineCoding.paritytype = pbuf[5];
    hcdc->LineCoding.datatype   = pbuf[6];

    /* Set the new configuration */
    ComPort_Config(hcdc);
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(hcdc->LineCoding.bitrate);
    pbuf[1] = (uint8_t)(hcdc->LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(hcdc->LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(hcdc->LineCoding.bitrate >> 24);
    pbuf[4] = hcdc->LineCoding.format;
    pbuf[5] = hcdc->LineCoding.paritytype;
    pbuf[6] = hcdc->LineCoding.datatype;

    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
    /* Add your code here */
    break;

  default:
    break;
  }

  return USBD_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (&hcdc->UartHandle != huart)
      continue;

    /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
    USBD_CDC_ReceivePacket(&USBD_Device, index);

    break;
  }
}

static void ComPort_Config(USBD_CDC_HandleTypeDef *hcdc)
{
  if (hcdc->UartHandle.State != HAL_UART_STATE_RESET)
    if (HAL_UART_DeInit(&hcdc->UartHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

  /* set the Stop bit */
  switch (hcdc->LineCoding.format)
  {
  case 0:
    hcdc->UartHandle.Init.StopBits = UART_STOPBITS_1;
    break;
  case 2:
    hcdc->UartHandle.Init.StopBits = UART_STOPBITS_2;
    break;
  default:
    hcdc->UartHandle.Init.StopBits = UART_STOPBITS_1;
    break;
  }

  /* set the parity bit*/
  switch (hcdc->LineCoding.paritytype)
  {
  case 0:
    hcdc->UartHandle.Init.Parity = UART_PARITY_NONE;
    break;
  case 1:
    hcdc->UartHandle.Init.Parity = UART_PARITY_ODD;
    break;
  case 2:
    hcdc->UartHandle.Init.Parity = UART_PARITY_EVEN;
    break;
  default:
    hcdc->UartHandle.Init.Parity = UART_PARITY_NONE;
    break;
  }

  /*set the data type : only 8bits and 9bits is supported */
  switch (hcdc->LineCoding.datatype)
  {
  case 0x07:
    /* With this configuration a parity (Even or Odd) must be set */
    hcdc->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  case 0x08:
    if(hcdc->UartHandle.Init.Parity == UART_PARITY_NONE)
    {
      hcdc->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else
    {
      hcdc->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
    }

    break;
  default:
    hcdc->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  }

  hcdc->UartHandle.Init.BaudRate = hcdc->LineCoding.bitrate;
  hcdc->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  hcdc->UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_Init(&hcdc->UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Start reception */
  HAL_UART_Receive_DMA(&hcdc->UartHandle, (uint8_t *)(hcdc->InboundBuffer), INBOUND_BUFFER_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Transfer error occurred in reception and/or transmission process */
  Error_Handler();
}

static void Error_Handler(void)
{
  __BKPT();
}

static void USBD_CDC_PMAConfig(PCD_HandleTypeDef *hpcd, uint32_t *pma_address)
{
  unsigned index;

  /* allocate PMA memory for all endpoints associated with CDC */
  // packet address
  for (index = 0; index < NUM_OF_CDC_UARTS; index++)
  {
    HAL_PCDEx_PMAConfig(hpcd, parameters[index].data_in_ep,  PCD_SNG_BUF, *pma_address);
    *pma_address += CDC_DATA_IN_MAX_PACKET_SIZE;
    HAL_PCDEx_PMAConfig(hpcd, parameters[index].data_out_ep, PCD_SNG_BUF, *pma_address);
    *pma_address += CDC_DATA_OUT_MAX_PACKET_SIZE;
    HAL_PCDEx_PMAConfig(hpcd, parameters[index].command_ep,  PCD_SNG_BUF, *pma_address);
    *pma_address += CDC_CMD_PACKET_SIZE;
  }
}

/* DMA Interrupts Fun */

// IRQhandler function call from interupt vector, can see this vector in startup_stm32f0xx.c file
void DMA1_Channel2_3_IRQHandler(void)
{
  /* FIXME: the array index is manually coded */
#if (NUM_OF_CDC_UARTS > 0)
  // At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can
  //  add his own function by customization of function pointer XferCpltCallback
  HAL_DMA_IRQHandler(context[0].UartHandle.hdmatx);
  HAL_DMA_IRQHandler(context[0].UartHandle.hdmarx);
#endif
}

void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* FIXME: the array index is manually coded */
#if (NUM_OF_CDC_UARTS > 1)
// At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can
//  add his own function by customization of function pointer XferCpltCallback
  HAL_DMA_IRQHandler(context[1].UartHandle.hdmatx);
  HAL_DMA_IRQHandler(context[1].UartHandle.hdmarx);

  #if (NUM_OF_CDC_UARTS > 2) // #VCP ??
  // At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can
  //  add his own function by customization of function pointer XferCpltCallback
    //HAL_DMA_IRQHandler(context[2].UartHandle.hdmatx);
    //HAL_DMA_IRQHandler(context[2].UartHandle.hdmarx);
  #endif

#endif
}

#if 0
void DMA1_Channel1_IRQHandler(void)
{
  /* FIXME: the array index is manually coded */
#if (NUM_OF_CDC_UARTS > 2)
// At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can
//  add his own function by customization of function pointer XferCpltCallback
  HAL_DMA_IRQHandler(context[2].UartHandle.hdmatx);
  HAL_DMA_IRQHandler(context[2].UartHandle.hdmarx);
#endif
}
#endif

#if SPI_ENABLE
  /**
    * @brief  This function handles DMA Rx interrupt request.
    * @param  None
    * @retval None
    */
  //void SPIx_DMA_RX_IRQHandler(void)
  //{
    //HAL_DMA_IRQHandler(hspi1.hdmarx);
  //}

  /**
    * @brief  This function handles DMA Tx interrupt request.
    * @param  None
    * @retval None
    */
  //void SPIx_DMA_TX_IRQHandler(void)
  //{
    //HAL_DMA_IRQHandler(hspi1.hdmatx);
  //}
#endif
