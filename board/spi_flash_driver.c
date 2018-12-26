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

#include <string.h>
#include "fsl_gpio.h"
#include "board.h"
#ifdef FULL_SFLASH_DEBUG
#include "fsl_debug_console.h"
#endif
#include "fsl_dspi.h"
#include "spi_flash_driver.h"
#include "pin_mux_rpk.h"
#include "spi_bus_share.h"


struct spi_flash
{
    uint32_t size;
    uint32_t page_size;
    uint32_t sector_size;
};

static struct spi_flash g_SpiFlash;
dspi_master_handle_t g_m_handle;
volatile bool isDspiTransferCompleted;
bool g_SPI_Flash_Initialized = false;


void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    isDspiTransferCompleted = true;
}


status_t FLASH_SPI_init(void)
{
    uint32_t sourceClock;
    dspi_master_config_t masterConfig;

    if(!g_SPI_Flash_Initialized)
    {
    	SPI_Bus_Share_Get_Access();

    	/* Get default Master configuration */
    	DSPI_MasterGetDefaultConfig(&masterConfig);

        masterConfig.whichCtar = kDSPI_Ctar1;
		masterConfig.ctarConfig.baudRate = 5000000;

		sourceClock = DSPI_MASTER_CLK_FREQ;
		DSPI_MasterInit(RPK_DSPI_MASTER_BASE, &masterConfig, sourceClock);

		/* Set up master transfer */
		DSPI_MasterTransferCreateHandle(RPK_DSPI_MASTER_BASE, &g_m_handle, DSPI_MasterUserCallback, NULL);

		g_SPI_Flash_Initialized = true;

		SPI_Bus_Share_Release_Access();
	}
    return SPI_FLASH_NO_ERROR;
}


status_t FLASH_SPI_deinit(void)
{
	if(g_SPI_Flash_Initialized)
	{
		SPI_Bus_Share_Get_Access();
    	DSPI_Deinit(RPK_DSPI_MASTER_BASE);

    	SPI_Bus_Share_Release_Access();

		g_SPI_Flash_Initialized = false;
	}
    return SPI_FLASH_NO_ERROR;
}


void SPI_Flash_Addr2cmd(uint32_t addr, uint8_t *cmd)
{
    cmd[1] = addr >> 16;
    cmd[2] = addr >> 8;
    cmd[3] = addr >> 0;
}


status_t SPI_Flash_Rw(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len)
{

    dspi_transfer_t masterXfer;

	SPI_Bus_Share_Get_Access();

    masterXfer.txData = cmd;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = cmd_len;
    if(data_len>0)
    	masterXfer.configFlags = kDSPI_MasterCtar1 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous | kDSPI_MasterActiveAfterTransfer;
    else
    	masterXfer.configFlags = kDSPI_MasterCtar1 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    isDspiTransferCompleted = false;
    DSPI_MasterTransferBlocking(RPK_DSPI_MASTER_BASE, &masterXfer);

    if (data_len != 0)
    {
        masterXfer.txData = data_out;
        masterXfer.rxData = data_in;
        masterXfer.dataSize = data_len;
        masterXfer.configFlags = kDSPI_MasterCtar1 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;// | kDSPI_MasterActiveAfterTransfer;;

        isDspiTransferCompleted = false;
        if (DSPI_MasterTransferBlocking(RPK_DSPI_MASTER_BASE, &masterXfer))
        {
#ifdef FULL_SFLASH_DEBUG
            PRINTF("\r\nSF:xfer failed\r\n");
#endif
        }
    }

    SPI_Flash_Get_Status(false);

    return SPI_FLASH_NO_ERROR;
}


status_t SPI_Flash_Chk_Status(uint32_t timeout, uint8_t cmd, uint8_t poll_bit)
{
	uint32_t ret = SPI_FLASH_NO_ERROR;
	uint8_t idcode[IDCODE_LEN] = {0};
	uint8_t cmd2 = CMD_READ_STATUS;

	SPI_Bus_Share_Get_Access();

	for (uint32_t i = 0; i < timeout; ++i)
	{
		/* Send CMD_READ_STATUS to get flag status register */
		ret = SPI_Flash_Rw(&cmd2, 1, NULL, idcode, 1);
		if (ret==SPI_FLASH_NO_ERROR && ((idcode[0] & poll_bit) == poll_bit))
			return(SPI_FLASH_NO_ERROR);
	}
	return(SPI_FLASH_ERROR);
}

uint8_t SPI_Flash_Get_Status(bool masteractive)
{
    dspi_transfer_t masterXfer;
    uint8_t status = SPI_FLASH_NO_ERROR;

	masterXfer.txData = NULL;
	masterXfer.rxData = &status;
	masterXfer.dataSize = 1;
	if(masteractive)
		masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous | kDSPI_MasterActiveAfterTransfer;
	else
		masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0;

	isDspiTransferCompleted = false;
	DSPI_MasterTransferBlocking(RPK_DSPI_MASTER_BASE, &masterXfer);
    return(status);
}


status_t SPI_Flash_Clear_Status()
{
    uint8_t cmd = CMD_CLEAR_STATUS;
    dspi_transfer_t masterXfer;

	SPI_Bus_Share_Get_Access();

	masterXfer.txData = &cmd;
	masterXfer.rxData = NULL;
	masterXfer.dataSize = 1;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

	isDspiTransferCompleted = false;
	DSPI_MasterTransferBlocking(RPK_DSPI_MASTER_BASE, &masterXfer);
    return SPI_FLASH_NO_ERROR;
}


status_t SPI_Flash_Enable_Write(uint8_t is_enabled)
{
	SPI_Bus_Share_Get_Access();
	uint8_t cmd = is_enabled ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;
    status_t tstatus = SPI_Flash_Rw(&cmd, 1, NULL, NULL, 0);
    return tstatus;
}


status_t SPI_Flash_Write_Page(uint8_t *buf, uint32_t page_offset, uint32_t byte_offset, uint32_t len)
{
    uint8_t cmd[4] = {0};
    uint32_t status = SPI_FLASH_NO_ERROR;

    cmd[0] = CMD_PAGE_PROGRAM;
    cmd[1] = page_offset >> 8;
    cmd[2] = page_offset;
    cmd[3] = byte_offset;

    SPI_Bus_Share_Get_Access();
    status = SPI_Flash_Generic_Write(cmd, 4, buf, NULL, len);
    return(status);
}


status_t SPI_Flash_Generic_Write(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len)
{

	SPI_Bus_Share_Get_Access();

    /* Each write need to enable write */
    if (SPI_Flash_Enable_Write(1))
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: enabling write failed\n");
#endif
       	return(SPI_FLASH_ERROR);
    }

    if (SPI_Flash_Rw(cmd, cmd_len, data_out, data_in, data_len))
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: write failed\n");
#endif
       	return(SPI_FLASH_ERROR);
    }

    if (SPI_Flash_Chk_Status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY))
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: check status failed\n");
#endif
       	return(SPI_FLASH_ERROR);
    }
	return SPI_FLASH_NO_ERROR;
}


status_t SPI_Flash_Write(uint32_t offset, uint32_t len, void *buf)
{
    uint32_t page_offset = 0, byte_offset = 0, page_size = 0;
    uint32_t data_chunk_len = 0, data_transferred = 0;
    uint32_t ret = 0;
    struct spi_flash *flash = &g_SpiFlash;

    SPI_Bus_Share_Get_Access();

    page_size = flash->page_size;
    page_offset = offset / page_size;
    byte_offset = offset % page_size;

    while (data_transferred < len)
    {
        /* First and last sector might be unaligned to page_size,
           So transfer unaligned sector first. */
        data_chunk_len = min(len - data_transferred, page_size - byte_offset);

        ret = SPI_Flash_Write_Page(((uint8_t *)buf + data_transferred), page_offset, byte_offset, data_chunk_len);
        if (1 == ret)
        {
            break;
        }

        byte_offset += data_chunk_len;
        if (byte_offset == page_size)
        {
            page_offset++;
            byte_offset = 0;
        }
        data_transferred += data_chunk_len;
    }

    if (ret)
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: program failed!\r\n");
#endif
    }
    else
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: program success!\r\n");
#endif
    }
    return ret;
}


status_t SPI_Flash_Read(uint32_t offset, uint32_t data_len, void *data)
{
    uint8_t cmd[5];

    cmd[0] = CMD_READ_ARRAY_FAST;
    SPI_Flash_Addr2cmd(offset, cmd);

    // Add a dummy byte, required for CMD_READ_ARRAY_FAST
    cmd[4] = 0x00;
    SPI_Bus_Share_Get_Access();
    status_t tstatus = SPI_Flash_Rw(cmd, sizeof(cmd), NULL, data, data_len);
    return tstatus;
}


status_t SPI_Flash_Erase_Block(uint32_t offset, uint32_t blkSize)
{
    uint8_t cmd[4];
    uint32_t status = SPI_FLASH_NO_ERROR;

    if (offset % blkSize)
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: Erase offset or length is not multiple of erase size\n");
#endif
        return SPI_FLASH_ERROR;
    }

    cmd[0] = CMD_ERASE_4K;
    SPI_Flash_Addr2cmd(offset, cmd);
    SPI_Bus_Share_Get_Access();
    status = SPI_Flash_Generic_Write(cmd, sizeof(cmd), NULL, NULL, 0);
    return(status);
}


status_t SPI_Flash_Erase_All(void)
{
    uint8_t cmd = CMD_ERASE_CHIP;
    uint32_t status = SPI_FLASH_NO_ERROR;

	SPI_Bus_Share_Get_Access();
    status = SPI_Flash_Generic_Write(&cmd, 1, NULL, NULL, 0);
    return(status);
}


status_t SPI_Flash_Write_Status(uint8_t sts_reg)
{
    uint8_t cmd;
    uint32_t ret;

	SPI_Bus_Share_Get_Access();

    ret = SPI_Flash_Enable_Write(1);
    if (ret != 0)
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: enabling write failed\n");
#endif
       	return(SPI_FLASH_ERROR);
    }

    cmd = CMD_WRITE_STATUS;
    ret = SPI_Flash_Rw(&cmd, 1, &sts_reg, NULL, 1);
    if (ret != 0)
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: fail to write status register\n");
#endif
       	return(SPI_FLASH_ERROR);
    }

    ret = SPI_Flash_Chk_Status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
    if (ret != 0)
    {
#ifdef FULL_SFLASH_DEBUG
        PRINTF("\r\nSF: write status register timed out\n");
#endif
       	return(SPI_FLASH_ERROR);
    }
	return SPI_FLASH_NO_ERROR;
}


status_t SPI_Flash_Readid(uint8_t *vendorId, uint8_t devId[])
{
    uint32_t ret = SPI_FLASH_NO_ERROR;
    uint8_t idcode[IDCODE_LEN] = {0};
    uint8_t cmd = CMD_READ_ID;

	SPI_Bus_Share_Get_Access();

    /* Send CMD_READ_ID to get flash chip ID codes */
    ret = SPI_Flash_Rw(&cmd, 1, NULL, idcode, sizeof(idcode));
    if (ret==SPI_FLASH_NO_ERROR)
    {
        *vendorId = idcode[0];
        devId[0] = idcode[1];
        devId[1] = idcode[2];
    }
	return(ret);
}


status_t SPI_Flash_Configure(uint32_t pageSize, uint32_t sectorSize, uint32_t sectorNum)
{
    struct spi_flash *flash = &g_SpiFlash;

    flash->page_size = pageSize;
    flash->sector_size = sectorSize;
    flash->size = flash->sector_size * sectorNum;

	SPI_Bus_Share_Get_Access();
    status_t tstatus = SPI_Flash_Chk_Status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
	return(tstatus);
}

/******************************************************************************
 * End of module                                                              *
 ******************************************************************************/
