/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _HEXIWEAR_SPI_FLASH_H_
#define _HEXIWEAR_SPI_FLASH_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(CPU_MKW41Z512VHT4)
#define RPK_DSPI_MASTER_BASE (DSPI0)
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#elif defined(CPU_MK64FN1M0VMD12)
#define RPK_DSPI_MASTER_BASE (DSPI1)
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI1_CLK_SRC)
#endif
#define SPI_FLASH_WINBOND_VENDER_ID (0xEF) /* WINBOND vendor id: 0xEF */
#define SPI_FLASH_MICRON_VENDER_ID (0x20) /* MICRON vendor id: 0x20 */

#define SPI_FLASH_NO_ERROR 0
#define SPI_FLASH_ERROR 1

#define IDCODE_LEN (0x3)

/* flash capacity configuration for WINBOND*/
//#define FLASH_PAGE_SIZE 256U             /* 256B page size */
//#define FLASH_SECTOR_SIZE (256U * 16U)   /* 4K sector size */
//#define FLASH_TOTAL_SIZE (8*1024U * 1024U) /* 8MB total size */  

/* flash capacity configuration for MICRON */
#define EXT_FLASH_ERASE_PAGE 4096U
#define FLASH_PAGE_SIZE 256U             /* 256B page size */
#define FLASH_SECTOR_SIZE (256U * 16U)   /* 4K smallest sector size */
#define FLASH_TOTAL_SIZE (16* 1024U * 1024U) /* 16MB total size */


#define ROUND(a, b) (((a) + (b)) & ~((b)-1))

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef SFDEBUG
#define SF_DEBUG(str) printf(str)
#else
#define SF_DEBUG(str)
#endif

#define SPI_FLASH_TIMEOUT (1000000)

/* flash commands */
#define CMD_READ_ID 0x9f
#define CMD_READ_ARRAY_SLOW 0x03
#define CMD_READ_ARRAY_FAST 0x0b
#define CMD_WRITE_STATUS 0x01
#define CMD_PAGE_PROGRAM 0x02
#define CMD_WRITE_DISABLE 0x04
#define CMD_CLEAR_STATUS 0x50
#define CMD_READ_STATUS 0x70
//#define CMD_READ_STATUS 0x05
#define CMD_WRITE_ENABLE 0x06
#define CMD_ERASE_4K 0x20
#define CMD_ERASE_32K 0x52
#define CMD_ERASE_64K 0xd8
#define CMD_ERASE_CHIP 0xc7

/* flash status */
#define STATUS_BUSY 0x80

/* Erase block size */
#define ERASE_4K_SIZE (4096)
#define ERASE_32K_SIZE (32768)
#define ERASE_64K_SIZE (65536)

extern bool g_SPI_In_Use;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif
status_t FLASH_SPI_init(void);
status_t FLASH_SPI_deinit(void);
void SPI_Flash_Addr2cmd(uint32_t addr, uint8_t *cmd);
status_t SPI_Flash_Rw(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len);
status_t SPI_Flash_Rw_Read(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len);
status_t SPI_Flash_Generic_Write(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len);
status_t SPI_Flash_Enable_Write(uint8_t is_enabled);
status_t SPI_Flash_Write_Page(uint8_t *buf, uint32_t page_offset, uint32_t byte_offset, uint32_t len);
status_t SPI_Flash_Chk_Status(uint32_t timeout, uint8_t cmd, uint8_t poll_bit);
status_t SPI_Flash_Write(uint32_t offset, uint32_t len, void *buf);
status_t SPI_Flash_Read(uint32_t offset, uint32_t data_len, void *data);
status_t SPI_Flash_Erase_Block(uint32_t offset, uint32_t blkSize);
status_t SPI_Flash_Erase_Sector(uint32_t offset, uint32_t len);
status_t SPI_Flash_Erase_All(void);
status_t SPI_Flash_Write_Status(uint8_t sts_reg);
status_t SPI_Flash_Readid(uint8_t *vendorId, uint8_t devId[]);
status_t SPI_Flash_Configure(uint32_t pageSize, uint32_t sectorSize, uint32_t sectorNum);
status_t SPI_Flash_Clear_Status();
uint8_t SPI_Flash_Get_Status(bool masteractive);
#if defined(__cplusplus)
}
#endif
#endif /* _HEXIWEAR_SPI_FLASH_H_ */
