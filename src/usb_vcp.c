
#include "usbd_vcp.h"
#include "usbd_cdc.c"
#include "usbd_desc.h"
#include "usbd_composite.h"
#include "config.h"

/* USB handle declared in main.c */
extern USBD_HandleTypeDef USBD_Device;
/* local function prototyping */

static uint8_t USBD_VCP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_VCP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_VCP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_VCP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_VCP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_VCP_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t USBD_VCP_SOF (struct _USBD_HandleTypeDef *pdev);
static void USBD_VCP_PMAConfig(PCD_HandleTypeDef *hpcd, uint32_t *pma_address);

static USBD_StatusTypeDef USBD_VCP_ReceivePacket (USBD_HandleTypeDef *pdev, unsigned index);
static USBD_StatusTypeDef USBD_VCP_TransmitPacket (USBD_HandleTypeDef *pdev, unsigned index, uint16_t offset, uint16_t length);

static int8_t VCP_Itf_Control (USBD_VCP_HandleTypeDef *hcdc, uint8_t cmd, uint8_t* pbuf, uint16_t length);
static void Error_Handler (void);
static void ComPort_Config (USBD_VCP_HandleTypeDef *hcdc);

/* VCP interface class callbacks structure that is used by main.c */
const USBD_CompClassTypeDef USBD_VCP =
{
  .Init                  = USBD_VCP_Init,
  .DeInit                = USBD_VCP_DeInit,
  .Setup                 = USBD_VCP_Setup,
  .EP0_TxSent            = NULL,
  .EP0_RxReady           = USBD_VCP_EP0_RxReady,
  .DataIn                = USBD_VCP_DataIn,
  .DataOut               = USBD_VCP_DataOut,
  .SOF                   = USBD_VCP_SOF,
  .PMAConfig             = USBD_VCP_PMAConfig,
};

/*
parameters for this VCP implementation
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
//  pmaadress (hardcoded as 0x18, 0x58...)
// Consists of using two buffers in PMA (buffer0 and buffer1) at any time CPU should Consists of using two buffers in PMA (buffer0 and buffer1), at any time CPU should
// be accessing one buffer (for R/W) while USB IP is accessing the other buffer
static const struct
{
  USART_TypeDef *Instance;
  uint8_t data_in_ep, data_out_ep, command_ep, command_itf;
} parameters[1] =
// Normally we can use 7 endpoints with CDC_FS, 3 for each CDC/VCPs ?
// Endpoint Address
{
#if 1
  {
    .Instance = VCP,
    .data_in_ep  = 0x85,
    .data_out_ep = 0x05,
    .command_ep  = 0x86,
    .command_itf = 0x04,
  },
#endif
};

/* context for each and every UART managed by this CDC implementation */
// context this base array contain all CDC_devices Handle
static USBD_CDC_HandleTypeDef context[NUM_OF_CDC_UARTS];

// it's function call from composite.c to init, in from fun USBD_Composite_Init()
static uint8_t USBD_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_CDC_HandleTypeDef *hcdc = context;
  unsigned index;

  for (index = 0; index < NUM_OF_CDC_UARTS; index++,hcdc++)
  {
    if (index == 3){
      // it's should be 2 when we know what to do here !!
      // hcdc=
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
    if (parameters[index].data_in_ep == 0x85)
    {

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

const char hi[]="hi\n\r";
const char TEST[]="RESET";
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

      /* hand the data to the HAL */
#if 1
      if ((hcdc->UartHandle.Instance == USART2))// && !strncmp ((char *)hcdc->OutboundBuffer, TEST, 5))
        HAL_UART_TxCpltCallback(&hcdc->UartHandle);//HAL_UART_Transmit_DMA(&hcdc->UartHandle, (uint8_t *)hi, 1);

      else
        HAL_UART_Transmit_DMA(&hcdc->UartHandle, (uint8_t *)hcdc->OutboundBuffer, RxLength);
#else
	if (hcdc->UartHandle.Instance == USART1)
	        HAL_UART_Transmit_DMA(&hcdc->UartHandle, (uint8_t *)hcdc->OutboundBuffer, RxLength);
#endif

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
#endif
}

#if 1
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





/*=============================================================================================================================*/



















#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

/**
  * @brief  Manage the CDC VCP class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
#define CDC_POWER_OFF 0x00
#define CDC_RESET 0x01
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_POWER_OFF:

    break;

    case CDC_RESET:

    break;

  default:
    break;
  }

  return (USBD_OK);
}


/**
  * @brief  Data received over USB OUT endpoint are sent over CDC VCP interface
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

uint8_t buf_command[6];
uint8_t i=0;
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  if (Buf[*Len-2] == 0x0D && Buf[*Len-1] == 0x0A){
		CDC_Transmit_FS(Buf, *Len);
	}
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  // This function CDC_Receive_FS is a callback function invoked when data is received -
  // add 3 extra lines of code to copy the data to my own buffer

  // show data Recived to console

  if( *Buf == 'q'){
    uint8_t new_line[3]="\n\r";
    uint8_t res[10]="\n\rreset\n\r";
    //CDC_Transmit_FS(new_line,3);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    CDC_Transmit_FS(res,10);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

  }
  else
  {
    CDC_Transmit_FS(Buf,*Len);
  }

  uint8_t received_data[APP_RX_DATA_SIZE];
  uint32_t received_data_size=0;
  uint32_t receive_total=0;


  received_data_size = *Len;
  memcpy(received_data, Buf, received_data_size);
  receive_total += received_data_size;

  return (USBD_OK);
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef2 *hcdc = (USBD_CDC_HandleTypeDef2*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff, uint16_t length)
{
  USBD_CDC_HandleTypeDef2   *hcdc = (USBD_CDC_HandleTypeDef2*) pdev->pClassData;

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff)
{
  USBD_CDC_HandleTypeDef2   *hcdc = (USBD_CDC_HandleTypeDef2*) pdev->pClassData;

  hcdc->RxBuffer = pbuff;

  return USBD_OK;
}



//
