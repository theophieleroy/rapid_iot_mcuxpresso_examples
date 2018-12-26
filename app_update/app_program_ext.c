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

#include "spi_bus_share.h"
#include "spi_flash_driver.h"
#include "flash_ica_driver.h"
#include "app_program_ext.h"
#include "spi_bus_share.h"

//! @addtogroup app_store
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

//! @brief Start application program to external flash initialization
int32_t app_program_ext_init(uint32_t newimgtype)
{
	uint32_t status = FICA_app_program_ext_init(newimgtype);
	return(app_release_status(status));
}

//! @brief Continue non-blocking image program to external flash, returns kAppStatus_Success if successful
// This is part of a non-blocking image program, but the actual small buffer flash write is blocking
int32_t app_program_ext_cont(uint8_t *bufptr, uint32_t len)
{
	// Write the image buffer block to the external flash
	// len should be the flash page size until the last call, then the remainder
	uint32_t status = FICA_app_program_ext_cont((void *)bufptr, len);
	return(app_release_status(status));
}

//! @brief Continue non-blocking image program to external flash, returns kAppStatus_Success if successful
// This is part of a non-blocking image program, but the actual small buffer flash write is blocking
int32_t app_program_ext_abs(uint32_t imgaddr, uint8_t *bufptr, uint32_t len)
{
	// Write the image buffer block to the external flash
	// len should be the flash page size until the last call, then the remainder
	uint32_t status = FICA_app_program_ext_abs(imgaddr, (void *)bufptr, len, TYPE_APP);
	return(app_release_status(status));
}

//! @brief Flush remaining buffered data to external flash
// Flush will force the remaining buffered bytes of data to get programmed to the external flash
int32_t app_program_ext_flush()
{
	// Call the FICA flush to program any remaining buffered data
	uint32_t status = FICA_app_program_ext_flush();
	return(app_release_status(status));
}

//! @brief Get the CRC of the application
int32_t app_program_ext_get_crc(uint32_t newimgtype, uint32_t *pcrc)
{
	*pcrc = 0;

	uint32_t status = FICA_Calculate_CRC(newimgtype, pcrc);
	return(app_release_status(status));
}

//! @brief Program the CRC of the application
int32_t app_program_ext_program_crc(uint32_t newimgtype, uint32_t crc)
{
	uint32_t status = FICA_app_program_ext_program_crc(newimgtype, crc);
	return(app_release_status(status));
}

//! @brief Release SPI Bus Share and return img error code
int32_t app_release_status(uint32_t spi_status)
{
	SPI_Bus_Share_Release_Access();
	if(spi_status!=SPI_FLASH_NO_ERROR)
		return(IMG_EXT_ERROR);
	return(IMG_EXT_NO_ERROR);
}


//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
