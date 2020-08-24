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
#include <math.h>

#include "spi.h"
#include "w25qxx.h"

#define PAGE_SIZE	sFLASH_SPI_PAGESIZE
/* USB handle declared in main.c */
extern USBD_HandleTypeDef USBD_Device;

#define BUF_SIZE 550 /* Should hold spi page program */
// RX
uint8_t vcp_rx[BUF_SIZE];
uint16_t writePointerRx=0;
int echo_off = 0;
/* ADC Help */
#if ADC_ENABLE
  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
  uint16_t ADC_Read(void);
  uint16_t adcvalue=0;
  #define ADC_VALUE_SIZE 7
  char adc_buff[ADC_VALUE_SIZE];
#endif

/* local function prototyping */
static uint8_t console(USBD_HandleTypeDef *pdev, unsigned index, char *buffer, uint16_t length);
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
#if 0 /* Never reached */
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
#endif
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
int my_strcmp(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
		char *strg1, char *strg2)
{
    while( ( (*strg1 != 0) && (*strg2 != 0) ) && (*strg1 == *strg2) )
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

void my_ctostrol_16(char *buff, uint32_t val, int length) {
	int i;
	buff[0] = '0';
	buff[1] = 'x';
	for (i = 0 ; i < (length * 2) ; i ++) {
		unsigned int ch;
		if ((val & 0xf) <= 0x9) ch = (val & 0xf) + '0';
		else ch = (val & 0xf) - 0xa + 'a';
		buff[length*2-i+1] = ch; /* Fill the buffer backward */
		val = val >> 4;
	}
	buff[length*2+2] = '\n';
	buff[length*2+3] = '\r';
}

uint32_t my_strtol_16(char *op) {
	uint32_t result = 0;
	int i = 2, num;
	if ((op[0] != '0') || (op[1] != 'x')) return 0;
	for (i = 2 ; i < (2 + 8 /* 0x12345678 */) ; i ++) {
		char ch = op[i];
		if (!ch) break;
		if ((ch >= '0') && (ch <= '9')) ch = ch-'0';
		else if ((ch >= 'a') && (ch <= 'f')) ch = ch-'a'+10;
		else if ((ch >= 'A') && (ch <= 'F')) ch = ch-'A'+10;
		result = (result << 4) | (ch & 0xf);
	}
	return result;
}	

int my_long_strtol_16(char *op, uint8_t *buffer, int max_chars) {
	uint32_t result = 0;
	int i = 2, num = 0;
	if ((op[0] != '0') || (op[1] != 'x')) return -1;
	for (i = 2 ; i < (2 + max_chars) ; i ++) {
		char ch = op[i];
		if ((i >= 4) && (!(i%2))) {
		       buffer[(i-4) >> 1] = (uint8_t) result;
		       result = 0;
		       num++;
		}
		if (!ch) break;
		if ((ch >= '0') && (ch <= '9')) ch = ch-'0';
		else if ((ch >= 'a') && (ch <= 'f')) ch = ch-'a'+10;
		else if ((ch >= 'A') && (ch <= 'F')) ch = ch-'A'+10;
		result = (result << 4) | (ch & 0xf);
	}
	return num;
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

// help functions - Convert string number to integer
uint8_t StringToInt(char a[])
{
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
    adcvalue = ADC_Read();
    double adc_current = (double)adcvalue * CURRENT_ADC_FACTORE;
    /* convert double to Str */
    itoa(adc_current,adc_buff,10);
    const char * point = ".";
    char lower_buff[4];
    uint16_t lower_adc_current = (adc_current-(double)StringToInt(adc_buff)) * 1000;
    itoa(lower_adc_current,lower_buff,10);
    /* show result value */
    const char * A_new_line = "A\r\n";
    char * str_adc_current = strcat(strcat(adc_buff,point) ,strcat(lower_buff, A_new_line));
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)str_adc_current, strlen(str_adc_current));
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
/* show the W25qxx spi flash JEDEC ID (0x0040ef16 -> W25Q32) */
void spi_flash_id(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
{
    jedec_id=W25qxx_ReadID();
    reverse((char*) &jedec_id,0,3);
    USBD_LL_Transmit(pdev,ep_addr,(uint8_t*) &jedec_id, 4);
}

/* show the W25qxx UniqID */
void spi_flash_uniq_id(USBD_HandleTypeDef *pdev,uint8_t ep_addr)
{
	USBD_LL_Transmit(pdev,ep_addr,(uint8_t*) &w25qxx.UniqID, 8);
}
void spi_read_mem(USBD_HandleTypeDef *pdev,uint8_t ep_addr,uint32_t address)
{
	uint8_t pBuffer=0x63;
	W25qxx_ReadByte(&pBuffer ,address);
	USBD_LL_Transmit(pdev,ep_addr,&pBuffer, 1);
	HAL_Delay(1);
}

void spi_write_mem(USBD_HandleTypeDef *pdev,uint8_t ep_addr,uint32_t address,uint8_t data)
{
	W25qxx_WriteByte(data, address);
}

#endif
static char *arr_cmd[] = {
	"re", // MCU Reset
	"ra", // reset assert
	"rd", // reset deassert
	"fl", // Force recovery signal low
	"ff", // Force recovery signal float
	"pl", // power button low (assert)
	"ph", // power button high (de-assert)
	"vn", // short vbat transistor
	"vf", // disconnect vbat transistor
	"cu", // measure current
	"vo", // measure voltage
	"ss", // spi mux on stm
	"sc", // spi mux on com
	"si", // spi id
	"su", // spi uniq id
	"sr", // spi byte read
	"sw", // spi byte write
	"sp", // spi page write
	"se", // spi erase
	"ee", // Enable STM32 echo - on when powered on
	"ed", // Disable STM32 echo
	"el", // nop
};

static int8_t vcp_cmd_control(unsigned int index, USBD_HandleTypeDef *pdev,uint8_t ep_addr, uint8_t* pbuf, uint16_t length) // #VCP
{
	/* Comands ID */
	enum CMD_ID{
		MCU_RESET = 0,
		RESET_ASSERT,
		RESET_DEASSERT,
		FORCE_RECOVERY_LOW,
		FORCE_RECOVERY_FLOAT,
		POWERBTN_LOW,
		POWERBTN_HIGH,
		VBATON,
		VBATOFF,
		CURRENT,
		VOLTAGE,
		SPI_SW_STM,
		SPI_SW_COM,
		SPI_ID,
		SPI_UNIQ_ID,
		SPI_R,
		SPI_W,
		SPI_PAGE_W,
		SPI_ERASE_CHIP,
		ECHO_ON,
		ECHO_OFF,
		CMD_NUM
	};


	uint32_t addr,data;
	uint8_t buffer[PAGE_SIZE]; /* Holds page */
	char *opcode = (char *)pbuf, *op1 = 0, *op2 = 0;
	int i, page_write_length = 0;
	int8_t cmd_id=-1;

	/* Mark op1 and op2 start pointer; and then replace space with null */
	for (i = 0 ; i < length ; i++) {
		if (pbuf[i] == 0) break;
		if (pbuf[i] == ' ') {
			if (!op2 && op1) op2 = (char *)pbuf + i + 1;
			if (!op1) op1 = (char *)pbuf + i + 1;
			pbuf[i] = 0;
		}
	}
	for (i=0; i <= CMD_NUM; i++) {
		cmd_id=i;
		if (my_strcmp (pdev, ep_addr, opcode, arr_cmd[i]) == 0) {
//			USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)"FOUND MATCH\n\r",13); HAL_Delay(10);
		       	break;
		}
	}
#if 0
	if (op1) {
			USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)"OP1 is\n\r",8); HAL_Delay(100);
			i = 0;
			while (1) {
				USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)&op1[i],1); HAL_Delay(10);
				if (!op1[i]) break;
				i++;
			}
	}

	if (op2) {
			USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)"OP2 is\n\r",8); HAL_Delay(100);
			i = 0;
			while (1) {
				USBD_LL_Transmit(pdev,ep_addr,(uint8_t *)&op2[i],1); HAL_Delay(10);
				if (!op2[i]) break;
				i++;
			}
	}
#endif
    switch(cmd_id)
    {
	case MCU_RESET:
		/* Reset the MCU; should go back to DFU mode */
		HAL_NVIC_SystemReset();
		break;
	case RESET_ASSERT:
		/* Assert the reset signal */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case RESET_DEASSERT:
		/* Deassert the reset signal (open drain) */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case FORCE_RECOVERY_LOW:
	/* Set the force recovery signal to low */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case FORCE_RECOVERY_FLOAT:
		/* Unset the force recovery signal */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case POWERBTN_LOW:
		/* Press POWER Button */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case POWERBTN_HIGH:
		/* Unpress POWER Button */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
		console(pdev,index,"OK\n\r", 4);
		break;
#if 0
	case VBATON:
		/* Enable the RTC Battery Voltage */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
		console(pdev,index,"OK\n\r", 4);
	break;
	case VBATOFF:
		/* Disable the RTC Battery Voltage */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
		console(pdev,index,"OK\n\r", 4);
	break;
#endif
#if 0
      case CURRENT:
        /* Read Total Current Input */
        #if ADC_ENABLE
          Current_Cmd(pdev,ep_addr);
        #endif
        break;
#endif
#if ADC_ENABLE
	case VOLTAGE:
		/* Read Total Voltage Input */
		/** Configure for the selected ADC regular channel to be converted.**/
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
			Error_Handler_ADC();
		/* read adc value */
		adcvalue = ADC_Read();
		my_ctostrol_16((char *)buffer, (uint32_t) adcvalue, 2);
		console(pdev,index,(char *)buffer, 8);
		break;
#endif
	case SPI_SW_STM:
		/* Connect the spi-flash to the STM32 */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case SPI_SW_COM:
		/* Connect the spi-flash to the COM */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
		console(pdev,index,"OK\n\r", 4);
		break;
	case SPI_ID:
		spi_flash_id(pdev,ep_addr);
		console(pdev,index,"OK\n\r", 4);
		break;
	case SPI_UNIQ_ID:
		spi_flash_uniq_id(pdev,ep_addr);
		console(pdev,index,"OK\n\r", 4);
		break;
	case SPI_R:
		/* op1 = address */
		addr=my_strtol_16(op1);
#if 0
		my_ctostrol_16((char *)buffer, addr, 4);
		console(pdev,index,(char *)buffer, 12);
#endif
		W25qxx_ReadByte(buffer ,addr);
		my_ctostrol_16((char *)buffer, buffer[0], 1);
		console(pdev,index,(char *)buffer, 6);
		break;
#if 0
	case SPI_W:
		/* op1 = address, op2 = data */
		addr = my_strtol_16(op1);
		data = my_strtol_16(op2);
		W25qxx_WriteByte(data, addr);
		console(pdev,index,"OK\n\r", 4);
		break;
#endif
	case SPI_PAGE_W:
		/* op1 = address, op2 = data */
		addr=my_strtol_16(op1);
		page_write_length = my_long_strtol_16(op2, buffer, 514 /*PAGE_SIZE*2*/ /* 512 characters that are 256 bytes */);
		W25qxx_WritePage(buffer, addr >> 8, addr & 0xff, page_write_length);
		console(pdev,index,"OK\n\r", 4);
		break;
	case SPI_ERASE_CHIP:
		W25qxx_EraseChip();
		console(pdev,index,"OK\n\r", 4);
		break;
	case ECHO_ON:
		echo_off = 0;
		console(pdev,index,"OK\n\r", 4);
		break;
	case ECHO_OFF:
		echo_off = 1;
		break;
	case CMD_NUM:
	default:
		break;
	}
    return (USBD_OK);
    /* USER CODE END 5 */
}


void *memset (void *s, int c, size_t n)
{
	for (int i = 0 ; i < (int)n ; i ++) {
		((uint8_t *)s)[i] = c;
	}
	return s;
}
static uint8_t USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	USBD_CDC_HandleTypeDef *hcdc = context;
	uint32_t RxLength;
	unsigned int index;
	for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++) {
		if (parameters[index].data_out_ep == epnum) {
			/* Get the received data length */
			RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
			if (hcdc->UartHandle.Instance == USART1)
				HAL_UART_Transmit_DMA(&hcdc->UartHandle, (uint8_t *)hcdc->OutboundBuffer, RxLength);
			if (hcdc->UartHandle.Instance == USART2) { // #VCP
				int i;
				int8_t *buff = (int8_t *)hcdc->OutboundBuffer;
#if 1 /* Echo back */
//				console (pdev, index, (char *)buff, RxLength);
				if (!echo_off) {
					USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)buff,RxLength); HAL_Delay(100);
				}
#endif
				for (i = 0 ; i < (int)RxLength; i++ ){
					vcp_rx[writePointerRx] = buff[i];
					if ((buff[i] == '\r') || (buff[i] == '\n') || (writePointerRx == (BUF_SIZE-1))) {
#if 1 /* Echo back */
//				console (pdev, index, (char *)"\n", 1);
				if (!echo_off) {
					USBD_LL_Transmit(pdev,parameters[index].data_in_ep,(uint8_t *)"\n",1); HAL_Delay(100);
				}
#endif
						vcp_rx[writePointerRx] = 0;
						vcp_cmd_control(index, pdev, parameters[index].data_in_ep,
							       vcp_rx,
							       writePointerRx + 1);
						memset(vcp_rx, 0, BUF_SIZE);
						writePointerRx = 0;
						
			      		} else writePointerRx++;
				}
				USBD_CDC_ReceivePacket(pdev, index);
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

static uint8_t console(USBD_HandleTypeDef *pdev, unsigned index, char *buffer, uint16_t length)
{
	int i = 200;
	USBD_StatusTypeDef outcome;
	while (context[index].InboundTransferInProgress) {
		HAL_Delay(1);
		if (i == 0) return USBD_BUSY;
		i--;
	}

	/* Transmit next packet */
	outcome = USBD_LL_Transmit(pdev, parameters[index].data_in_ep, (uint8_t *)buffer, length);

	if (USBD_OK == outcome)
	{
		/* Tx Transfer in progress */
		context[index].InboundTransferInProgress = 1;
	}
	return outcome;
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
  #if 0
  /**
    * @brief  This function handles DMA Rx interrupt request.
    * @param  None
    * @retval None
    */
  void SPIx_DMA_RX_IRQHandler(void)
  {
    HAL_DMA_IRQHandler(hspi1.hdmarx);
  }

  /**
    * @brief  This function handles DMA Tx interrupt request.
    * @param  None
    * @retval None
    */
  void SPIx_DMA_TX_IRQHandler(void)
  {
    HAL_DMA_IRQHandler(hspi1.hdmatx);
  }
  #endif
#endif
