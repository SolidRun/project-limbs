#ifndef _W25QXX_H
#define _W25QXX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "spi.h"

#define _W25QXX_SPI                   hspi1
#define _W25QXX_CS_GPIO               GPIOA
#define _W25QXX_CS_PIN                GPIO_PIN_4
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

/* M25P SPI Flash supported commands */
#define SPINOR_OP_WREN		0x06	/* Write enable */
#define SPINOR_OP_WRDI		0x04	/* Write disable */
#define SPINOR_OP_RDSR		0x05	/* Read status register (S7-S0)*/
#define SPINOR_OP_RDCR		0x35	/* Read configuration register (S15-S8)*/
#define SPINOR_OP_RDID		0x9f	/* Read JEDEC ID ***/
#define SPINOR_OP_READ		0x03	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST	0x0b	/* Read data bytes (high frequency) */
#define SPINOR_OP_BE_32K	0x52	/* Erase 32KiB block */

//========//

#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0xD8  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

// { "w25q32", INFO(0xef4016, 0, 64 * 1024,  64, SECT_4K) },
#define sFLASH_w25q32_ID         0xef4016

typedef enum
{
	W25Q10=1,
	W25Q20,
	W25Q40,
	W25Q80,
	W25Q16,
	W25Q32,
	W25Q64,
	W25Q128,
	W25Q256,
	W25Q512,

}W25QXX_ID_t;

typedef struct
{
	W25QXX_ID_t	ID;
  uint32_t	JEDEC_ID;
	uint8_t		UniqID[8];
	uint16_t	PageSize;
	uint32_t	PageCount;
	uint32_t	SectorSize;
	uint32_t	SectorCount;
	uint32_t	BlockSize;
	uint32_t	BlockCount;
	uint32_t	CapacityInKiloByte;
	uint8_t		StatusRegister1;
	uint8_t		StatusRegister2;
	uint8_t		StatusRegister3;
	uint8_t		Lock;

}w25qxx_t;

extern w25qxx_t	w25qxx;
//############################################################################
// in Page,Sector and block read/write functions, can put 0 to read maximum bytes
//############################################################################
bool		W25qxx_Init(void);

void		W25qxx_EraseChip(void);
void 		W25qxx_EraseSector(uint32_t SectorAddr);
void 		W25qxx_EraseBlock(uint32_t BlockAddr);
void    W25qxx_ReadUniqID(void);

uint32_t	W25qxx_PageToSector(uint32_t	PageAddress);
uint32_t	W25qxx_PageToBlock(uint32_t	PageAddress);
uint32_t	W25qxx_SectorToBlock(uint32_t	SectorAddress);
uint32_t	W25qxx_SectorToPage(uint32_t	SectorAddress);
uint32_t	W25qxx_BlockToPage(uint32_t	BlockAddress);

bool 		W25qxx_IsEmptyPage(uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_PageSize);
bool 		W25qxx_IsEmptySector(uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_SectorSize);
bool 		W25qxx_IsEmptyBlock(uint32_t Block_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_BlockSize);

void 		W25qxx_WriteByte(uint8_t pBuffer,uint32_t Bytes_Address);
void 		W25qxx_WritePage(uint8_t *pBuffer	,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_PageSize);
void 		W25qxx_WriteSector(uint8_t *pBuffer,uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_SectorSize);
void 		W25qxx_WriteBlock(uint8_t* pBuffer,uint32_t Block_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_BlockSize);

void 		W25qxx_ReadByte(uint8_t *pBuffer,uint32_t Bytes_Address);
void 		W25qxx_ReadBytes(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t NumByteToRead);
void 		W25qxx_ReadPage(uint8_t *pBuffer,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_PageSize);
void 		W25qxx_ReadSector(uint8_t *pBuffer,uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_SectorSize);
void 		W25qxx_ReadBlock(uint8_t* pBuffer,uint32_t Block_Address,uint32_t OffsetInByte,uint32_t	NumByteToRead_up_to_BlockSize);
//############################################################################
#ifdef __cplusplus
}
#endif

#endif
