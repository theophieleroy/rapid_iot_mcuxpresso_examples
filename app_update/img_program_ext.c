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

#include "spi_flash_driver.h"
#include "flash_ica_driver.h"
#include "img_program_ext.h"
#include "spi_bus_share.h"

//! @addtogroup image_store
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////



//! @brief Start image program to external flash initialization
int32_t img_program_ext_init()
{
	uint32_t status = img_verify_ext_flash();
	return(img_release_status(status));
}

//! @brief Blocking image program to external flash
// This is part of a blocking image program, but the actual small buffer flash write is blocking
int32_t img_program_ext(uint32_t imgaddr, uint8_t *bufptr, uint32_t writelen)
{
	int32_t status = FICA_app_program_ext_abs(imgaddr, bufptr, writelen, TYPE_IMG);
	return(img_release_status(status));
}


//! @brief Blocking image read from external flash to a buffer, returns kAppStatus_Success if successful
int32_t img_read_ext(uint32_t imgaddr, uint8_t *bufptr, uint32_t readlen)
{
	int32_t status = SPI_FLASH_ERROR;

	// Read the image from external flash to the passed buffer
	// check if some newbie passed in a null pointer for a buffer
	if(bufptr==NULL)
		return(img_release_status(status));

	// length can not be greater than the total flash size allocated for images
	if(((imgaddr - IMG_FLASH_START_ADDR) + readlen) > IMG_FLASH_TOTAL_SIZE)
		return(img_release_status(status));

	// Read the data into the buffer
	status = SPI_Flash_Read(imgaddr, readlen, bufptr);
	return(img_status(status));
}


//! @brief Initialize External Flash driver
int32_t img_verify_ext_flash(void)
{
	// Initialize RPK External Flash (MT25QL128)
	uint32_t status = SPI_FLASH_ERROR;
	uint8_t vendorId = 0U;
	uint8_t devId[2];

	// Read flash vendor ID
	status = SPI_Flash_Readid(&vendorId, devId);
	if (status!=SPI_FLASH_NO_ERROR)
		return(img_release_status(status));

	/* check if it's Micron chip or not */
	if (SPI_FLASH_MICRON_VENDER_ID == vendorId)
	{
		status = SPI_Flash_Configure(FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, 256);
		if (status!=SPI_FLASH_NO_ERROR)
			return(img_release_status(status));
	}
	else
		return(img_release_status(SPI_FLASH_ERROR));
	return(img_release_status(SPI_FLASH_NO_ERROR));
}


//! @brief Release SPI Bus Share and return img error code
int32_t img_release_status(uint32_t spi_status)
{
	SPI_Bus_Share_Release_Access();
	if(spi_status!=SPI_FLASH_NO_ERROR)
		return(IMG_EXT_ERROR);
	return(IMG_EXT_NO_ERROR);
}

//! @brief returns img error code from spi flash error code
int32_t img_status(uint32_t spi_status)
{
	if(spi_status!=SPI_FLASH_NO_ERROR)
		return(IMG_EXT_ERROR);
	return(IMG_EXT_NO_ERROR);
}

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
