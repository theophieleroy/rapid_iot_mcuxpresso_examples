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
#include "spi_flash_driver.h"
#include "flash_ica_driver.h"
#include "app_program_ext.h"
#include "img_program_ext.h"
#include "pin_mux.h"
#include "pin_mux_rpk.h"
#include "board.h"
#include "rpk_led.h"
#include "spi_bus_share.h"
#if defined(CPU_MK64FN1M0VMD12)
#include "rgb_led.h"
#endif
#ifdef USE_CRC_16
#include "crc16.h"
#else
#include "crc32.h"
#endif

//! @addtogroup flash_ica
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

static uint32_t FICA_RECORD_SIZE = 100;
static uint32_t ProgExtAppImgType = 0;
static uint32_t ProgExtAppImgStartAddr = 0;
static uint32_t ProgExtAppImgCurLen = 0;
static uint32_t ProgExtAppImgCurPageLen = 0;
static uint32_t ProgExtAppImgCurPageOffset = 0;
static uint8_t ProgExtAppImgBuf[EXT_FLASH_ERASE_PAGE] = {0};
static uint8_t *pProgExtAppImgBuf=ProgExtAppImgBuf;
static uint8_t *pProgExtAppImgBufRun=ProgExtAppImgBuf;
static FICA_Record ProgExtAppInfo = {0};

#define DEBUG_IMG

#ifdef DEBUG_IMG
	uint8_t tdata[EXT_FLASH_ERASE_PAGE + 1]={0};
	volatile uint32_t ErrorCnt = 0;
#endif


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*
 * FICA - Flash ICA (Image Configuration Area) Driver
 *
 */

//! @brief Initialize the Application Program External Flash Interface
int32_t FICA_app_program_ext_init(uint32_t newimgtype)
{

	ProgExtAppImgType = newimgtype;
	ProgExtAppImgStartAddr = FICA_get_app_img_start_addr(ProgExtAppImgType);
	ProgExtAppImgCurLen = 0;
	ProgExtAppImgCurPageLen = 0;
	ProgExtAppImgCurPageOffset = 0;
	FICA_clear_buf(pProgExtAppImgBuf,0xFF);
	pProgExtAppImgBufRun=ProgExtAppImgBuf;

	// Initialize the Flash ICA (Image Configuration Area)
	if(FICA_initialize()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Read the new application information
	if(FICA_read_record(newimgtype, &ProgExtAppInfo)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);

}

//! @brief  Programs or continue to program the external flash using the passed buffer and buffer length
// Actual programming to the flash will only occur when there is a flash page worth of data staged for programming
int32_t FICA_app_program_ext_cont(void *pbuf, uint32_t len)
{
	uint32_t offset = 0;

	// check if some newbie passed in a null pointer for a buffer
	if(pbuf==NULL)
		return(SPI_FLASH_ERROR);

	// length can not be greater than EXT_FLASH_ERASE_PAGE size
	if(len > EXT_FLASH_ERASE_PAGE)
		return(SPI_FLASH_ERROR);

	// Check if its exceeded the allowed size for this application
	if((ProgExtAppImgCurLen + len) > FICA_MAX_APP_SIZE)
		return(SPI_FLASH_ERROR);

	// Spin around doing some magical things
	for(uint32_t ProgExtAppImgCurRunLen=0; ProgExtAppImgCurRunLen<len; ProgExtAppImgCurRunLen++)
	{
		// Move the characters one at time from the passed buffer to the collection buffer
		*pProgExtAppImgBufRun++=*(uint8_t *)pbuf++;
		ProgExtAppImgCurPageLen++;
		ProgExtAppImgCurLen++;

		// Check if its time to write a page
		if(ProgExtAppImgCurPageLen == EXT_FLASH_ERASE_PAGE)
		{
			offset = ProgExtAppImgStartAddr + (ProgExtAppImgCurPageOffset * EXT_FLASH_ERASE_PAGE);

#ifdef DEBUG_FICA
			FICA_clear_buf(tdata,0xFF);
			SPI_Flash_Read(offset, EXT_FLASH_ERASE_PAGE, tdata);
#endif

			// Erase page before writing
			if(SPI_Flash_Erase_Block(offset, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
				return(SPI_FLASH_ERROR);

#ifdef DEBUG_FICA
			SPI_Flash_Read(offset, EXT_FLASH_ERASE_PAGE, tdata);
#endif

			if(SPI_Flash_Write(offset, EXT_FLASH_ERASE_PAGE, pProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
				return(SPI_FLASH_ERROR);

#ifdef DEBUG_FICA
			SPI_Flash_Read(offset, EXT_FLASH_ERASE_PAGE, tdata);
#endif

			FICA_clear_buf(pProgExtAppImgBuf, 0xFF);
			ProgExtAppImgCurPageLen = 0;
			pProgExtAppImgBufRun=ProgExtAppImgBuf;
			ProgExtAppImgCurPageOffset++;
		}
	}
	return(SPI_FLASH_NO_ERROR);
}

//! @brief Blocking image program to external flash
// This is part of a blocking image program, but the actual small buffer flash write is blocking
// apporimg == 0 for app, 1 for img
int32_t FICA_app_program_ext_abs(uint32_t imgaddr, uint8_t *bufptr, uint32_t writelen, bool apporimg)
{
	// Write the image buffer to the external flash
	// len should be the flash page size until the last call, then the remainder
	// Erase page before writing

	uint32_t curextlen = 0; // current length of the write, accumulates to total image write length (writelen)
	uint32_t curwritelen = 0; // will be EXT_FLASH_ERASE_PAGE size until the last page
	uint32_t curextpage = 0; // Increments up each loop to index the page that is to be written
	uint32_t curwriteaddr = 0; // address to erase page and write the current buffer data, buffer reindexes on each loop

	uint32_t pagenumrem = imgaddr % EXT_FLASH_ERASE_PAGE;
	uint32_t pagestartaddr = (imgaddr / EXT_FLASH_ERASE_PAGE) * EXT_FLASH_ERASE_PAGE;
	uint32_t prestartlen = imgaddr - pagestartaddr;
	uint32_t tempaddr = 0;

	// check if some newbie passed in a null pointer for a buffer
	if(bufptr==NULL)
		return(IMG_EXT_ERROR);

	if(writelen==0)
		return(IMG_EXT_NO_ERROR);

	if(apporimg==TYPE_APP)
	{
		// length can not be greater than the total flash size allocated for images
		if((imgaddr + writelen) > FICA_MAX_APP_SIZE)
			return(IMG_EXT_ERROR);
	}
	else
	{
		// length can not be greater than the total flash size allocated for images
		if((imgaddr + writelen) > (IMG_FLASH_START_ADDR + IMG_FLASH_TOTAL_SIZE))
			return(IMG_EXT_ERROR);
	}
	FICA_clear_buf(pProgExtAppImgBuf,0xFF);

	if(apporimg==TYPE_APP)
	{
		uint32_t templen = imgaddr + writelen;
		if(templen>ProgExtAppImgCurLen)
			ProgExtAppImgCurLen = templen;
	}
	else
	{
		ProgExtAppImgCurLen = writelen;
	}

	// If its not on a flash page boundary, need to read flash before erasing the page
	if(pagenumrem > 0)
	{
		if(apporimg==TYPE_APP) pagestartaddr += ProgExtAppImgStartAddr;

		// the image address doesn't start on a page boundary, read the flash from the start of the page
		if(SPI_Flash_Read(pagestartaddr, prestartlen, pProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
			return(IMG_EXT_ERROR);
	}

	// Erase External Flash and Program it from the buffer, by external flash page
	while(curextlen < writelen)
	{
		// Write either a page or less if only less than a page is left
		curwritelen = prestartlen + (writelen - curextlen) >= EXT_FLASH_ERASE_PAGE ? EXT_FLASH_ERASE_PAGE - prestartlen : writelen - curextlen;
		curwriteaddr = pagestartaddr+(curextpage*EXT_FLASH_ERASE_PAGE);

		// See if there is data after the data we need to write, that needs to be saved as well
		if(prestartlen+curwritelen<EXT_FLASH_ERASE_PAGE)
		{
			uint32_t remainpagelen = EXT_FLASH_ERASE_PAGE - prestartlen - curwritelen;
			tempaddr = curwriteaddr+prestartlen+curwritelen;
			if(apporimg==TYPE_APP) tempaddr += ProgExtAppImgStartAddr;
			if(SPI_Flash_Read(tempaddr, remainpagelen, &pProgExtAppImgBuf[prestartlen+curwritelen])!=SPI_FLASH_NO_ERROR)
				return(IMG_EXT_ERROR);
		}

		// Copy the data from the passed buffer to the Program Image Buffer
		memcpy(&pProgExtAppImgBuf[prestartlen], bufptr, curwritelen);

		tempaddr = curwriteaddr;
		if(apporimg==TYPE_APP) tempaddr += ProgExtAppImgStartAddr;


#ifdef DEBUG_IMG
		FICA_clear_buf(tdata,0xFF);
		SPI_Flash_Read(tempaddr, EXT_FLASH_ERASE_PAGE, tdata);
#endif

		// Erase page before writing
		if(SPI_Flash_Erase_Block(tempaddr, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
			return(IMG_EXT_ERROR);

#ifdef DEBUG_IMG
		SPI_Flash_Read(tempaddr, EXT_FLASH_ERASE_PAGE, tdata);
#endif

		// Write the buffer to the external SPI Flash
		if(SPI_Flash_Write(tempaddr, EXT_FLASH_ERASE_PAGE, pProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
			return(IMG_EXT_ERROR);

#ifdef DEBUG_IMG
		SPI_Flash_Read(tempaddr, EXT_FLASH_ERASE_PAGE, tdata);
		for(uint32_t i=0; i<EXT_FLASH_ERASE_PAGE; i++)
		{
			if(pProgExtAppImgBuf[i]!=tdata[i])
			{
				ErrorCnt+=1;
				return(IMG_EXT_ERROR);
			}
		}
#endif

		curextlen += curwritelen;
		bufptr += curwritelen;
		curextpage++;
		prestartlen = 0;
	}
	return(IMG_EXT_NO_ERROR);
}


//! @brief  Flush the program buffer.  Write any remaining bytes out to the flash
int32_t FICA_app_program_ext_flush()
{
	uint32_t offset = ProgExtAppImgStartAddr + (ProgExtAppImgCurPageOffset * EXT_FLASH_ERASE_PAGE);

	if(ProgExtAppImgCurPageLen>0)
	{
		if(ProgExtAppImgCurPageLen<EXT_FLASH_ERASE_PAGE)
		{
			uint8_t *skipbuf = (uint8_t *)&ProgExtAppImgBuf[ProgExtAppImgCurPageLen];
			uint32_t fillsize = EXT_FLASH_ERASE_PAGE - ProgExtAppImgCurPageLen;
			SPI_Flash_Read(offset+ProgExtAppImgCurPageLen, fillsize, skipbuf);
		}

		// Erase page before writing
		if(SPI_Flash_Erase_Block(offset, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);

	#ifdef DEBUG_FICA
		SPI_Flash_Read(offset, EXT_FLASH_ERASE_PAGE, tdata);
	#endif

		if(SPI_Flash_Write(offset, EXT_FLASH_ERASE_PAGE, pProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);

	#ifdef DEBUG_FICA
				SPI_Flash_Read(offset, EXT_FLASH_ERASE_PAGE, tdata);
	#endif
	}

	if(FICA_write_image_info(ProgExtAppImgType, FICA_IMG_FMT_BIN, ProgExtAppImgCurLen)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief  Calculates the CRC for the passed Image Type
// If the image was from MSD, it uses the tracked image length, otherwise it loads it from the FICA record
int32_t FICA_app_program_ext_calculate_crc(uint32_t imgtype, uint32_t *pcrc)
{
	if(FICA_Calculate_CRC(imgtype, pcrc)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief  Programs the CRC for the passed Image Type
int32_t FICA_app_program_ext_program_crc(uint32_t imgtype, uint32_t crc)
{
	if(FICA_Program_CRC(imgtype, crc)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Clears the buffer with the passed initialization value
int32_t FICA_clear_buf(uint8_t *pbuf, uint8_t initval)
{
	// Read an Erase Page size (4K for this flash)
	for(int i = 0; i < EXT_FLASH_ERASE_PAGE; i++)
		*pbuf++=initval;

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Read the FICA database into a buffer
// Usually done just prior to modify, write
int32_t FICA_read_db()
{
	// Read the entire FICA buffer
	if(SPI_Flash_Read(FICA_START_ADDR, EXT_FLASH_ERASE_PAGE, (void *)ProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Write the FICA database from the buffer
int32_t FICA_write_db()
{
	// Erase the 1st page of the flash
	if(SPI_Flash_Erase_Block(FICA_START_ADDR, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write the entire FICA buffer
	if(SPI_Flash_Write(FICA_START_ADDR, EXT_FLASH_ERASE_PAGE, (void *)ProgExtAppImgBuf)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	if(FICA_read_db() != SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Writes the passed buffer to external flash starting at address passed by offset
int32_t FICA_write_buf(uint32_t offset, uint32_t len, void *buf)
{
	uint8_t *pbuf = (uint8_t *)buf;

	if(pbuf==NULL) return(SPI_FLASH_ERROR);
	for(int i = 0; i < len; i++)
	{
		if((offset + i) >= EXT_FLASH_ERASE_PAGE)
			return(SPI_FLASH_ERROR);
		ProgExtAppImgBuf[offset+i] = *pbuf++;
	}
	return(SPI_FLASH_NO_ERROR);
}

//! @brief Reads the external flash to the passed buffer starting at address passed by offset
int32_t FICA_read_buf(uint32_t offset, uint32_t len, void *buf)
{
	uint8_t *pbuf = (uint8_t *)buf;

	if(pbuf==NULL) return(SPI_FLASH_ERROR);
	for(int i = 0; i < len; i++)
	{
		if((offset + i) >= EXT_FLASH_ERASE_PAGE) return(SPI_FLASH_ERROR);
		*pbuf++ = ProgExtAppImgBuf[offset+i];
	}
	return(SPI_FLASH_NO_ERROR);
}

//! @brief Get the field flash address based on the image type and field offset
int32_t FICA_get_field_addr(uint32_t imgtype, uint32_t fieldoffset, uint32_t *pfieldaddr)
{
	uint32_t recnum = 0;

	// Check if the image type is valid or no image type was passed in
	if(imgtype==FICA_IMG_TYPE_NONE)
	{
		// No image type passed in, desired field is at the start of the ICA
		*pfieldaddr = FICA_START_ADDR + fieldoffset;
	}
	else
	{
		// Get the record number based on the image type. Image type starts at 1, recnum starts at zero
		if(imgtype>0)
			recnum = imgtype - 1;

		// Valid image type, calculate the field address
		*pfieldaddr = FICA_START_ADDR + FICA_ICA_DEFINITION_SIZE + (recnum * FICA_RECORD_SIZE) + fieldoffset;
	}
	return(SPI_FLASH_NO_ERROR);
}


//! @brief Write a value to a specific image field, write it to the external flash
int32_t FICA_write_field(uint32_t imgtype, uint32_t fieldoffset, uint32_t val)
{

	// Check if its a valid image type
	if(imgtype>FICA_MAX_IMAGES)
		return(SPI_FLASH_ERROR);

	if(FICA_read_db()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	FICA_write_field_no_save(imgtype, fieldoffset, val);

	if(FICA_write_db()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

//#define FICA_VALIDATE_WRITE 1

#ifdef FICA_VALIDATE_WRITE

	uint32_t readval = 0;

	if(FICA_read_field(imgtype, fieldoffset, &readval)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	if(val != readval)
		return(SPI_FLASH_ERROR);
#endif

	return(SPI_FLASH_NO_ERROR);
}


//! @brief Write a value to a specific image field, writes to buffer only, not external flash
int32_t FICA_write_field_no_save(uint32_t imgtype, uint32_t fieldoffset, uint32_t val)
{
	uint32_t fieldaddr = 0;
	uint32_t *pfieldaddr = &fieldaddr;

	uint8_t valbuf[100]={0};
	uint8_t *pvalbuf=valbuf;

	uint8_t *pval = (uint8_t *)&val;

	valbuf[0] = *pval++;
	valbuf[1] = *pval++;
	valbuf[2] = *pval++;
	valbuf[3] = *pval++;

	// Check if its a valid image type
	if(imgtype>FICA_MAX_IMAGES) return(SPI_FLASH_ERROR);

	// Get the image record offset based on the image type and field offset
	if(FICA_get_field_addr(imgtype, fieldoffset, pfieldaddr)!=SPI_FLASH_NO_ERROR)
		return SPI_FLASH_ERROR;

	if(FICA_write_buf(fieldaddr, FICA_FIELD_SIZE, (void *)pvalbuf)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Read any field given the passed image type and field offset
int32_t FICA_read_field(uint32_t imgtype, uint32_t field_offset, uint32_t *pfieldval)
{
	uint32_t fieldaddr = 0;
	uint32_t *pfieldaddr = &fieldaddr;

	// Check if its a valid image type
	if(imgtype>FICA_MAX_IMAGES) return(SPI_FLASH_ERROR);

	// Get the field address based on the image type and field offset
	if(FICA_get_field_addr(imgtype, field_offset, pfieldaddr)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Read the field value from the SPI Flash
	if(SPI_Flash_Read(fieldaddr, FICA_FIELD_SIZE, pfieldval)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Calculates the external flash image CRC given the image type, length, and pointer to store crc value
int32_t FICA_Calculate_CRC(uint32_t imgtype, uint32_t *crcval)
{
	uint32_t len = 0;

#ifdef USE_CRC_16

	uint16_t crc16val = 0;
	uint16_t *p_crc16val = &crc16val;

	crc16_data_t crc16Config = {0};
	crc16_data_t *pcrc16Config = &crc16Config;

	uint32_t startaddr = FICA_get_app_img_start_addr(imgtype);

	len = FICA_get_app_img_len(imgtype);

	crc16_init(pcrc16Config);

	uint32_t tLen = 0;
	uint32_t tOffset = startaddr;

	while (len)
	{
		uint8_t temp[FLASH_PAGE_SIZE] = {0};

#if defined(BLINK_PROGRESS) && defined(BOOTLOADER)
#if defined(CPU_MK64FN1M0VMD12)
		Blink_LED_RGB(RGB_LED_BRIGHT_LOW, RGB_LED_COLOR_BLUE);
#else
		Blink_LED(BOARD_INITLEDS_LED_BLUE_GPIO_PIN);
#endif
#endif

		// Set temporary length to process a FLASH_PAGE worth of bytes or length if smaller
		if (FLASH_PAGE_SIZE > len)
			tLen = len;
		else
			tLen = FLASH_PAGE_SIZE;

		// Get some data from SPI
		if (SPI_Flash_Read(tOffset, tLen, temp))
			return SPI_FLASH_ERROR;

		// Update CRC
		crc16_update(pcrc16Config, (const uint8_t *)temp, tLen);

		// Update offset and remaining length in bytes
		tOffset += tLen;
		len -= tLen;
	}

	crc16_finalize(pcrc16Config, p_crc16val);
	*crcval = (uint32_t) crc16val;


#else

	crc32_data_t crc32Config = {0};
	crc32_data_t *pcrc32Config = &crc32Config;

	uint32_t startaddr = FICA_get_app_img_start_addr(imgtype);
	if(len==0)
		len = FICA_get_app_img_len(imgtype);

	crc32_init(pcrc32Config);

	uint32_t tLen = 0;
	uint32_t tOffset = startaddr;

	while (len)
	{
		uint8_t temp[FLASH_PAGE_SIZE] = {0};

		// Set temporary length to process a FLASH_PAGE worth of bytes or length if smaller
		if (FLASH_PAGE_SIZE > len)
			tLen = len;
		else
			tLen = FLASH_PAGE_SIZE;

		// Get some data from SPI
		if (SPI_Flash_Read(tOffset, tLen, temp))
			return SPI_FLASH_ERROR;

		// Update CRC
		crc32_update(pcrc32Config, (const uint8_t *)temp, tLen);

		// Update offset and remaining length in bytes
		tOffset += tLen;
		len -= tLen;
	}

	crc32_finalize(pcrc32Config, crcval);

#endif

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Calculates the internal flash image CRC given the image type, length, and pointer to store crc value
int32_t FICA_Calculate_CRC_Internal(uint32_t startaddr, uint32_t len, uint32_t *crcval)
{

#ifdef USE_CRC_16

	uint16_t crc16val;
	uint16_t *p_crc16val = &crc16val;

	crc16_data_t crc16Config = {0};
	crc16_data_t *pcrc16Config = &crc16Config;

	crc16_init(pcrc16Config);
	crc16_update(pcrc16Config, (const uint8_t *)startaddr, len);
	crc16_finalize(pcrc16Config, p_crc16val);
	*crcval = (uint32_t) crc16val;

#else
	crc32_data_t crc32Config = {0};
	crc32_data_t *pcrc32Config = &crc32Config;

	crc32_init(pcrc32Config);
	crc32_update(pcrc32Config, (const uint8_t *)startaddr, len);
	crc32_finalize(pcrc32Config, crcval);

#endif

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Program the passed crc value to the external flash given the passed image type
int32_t FICA_Program_CRC(uint32_t imgtype, uint32_t crcval)
{
	if(FICA_write_field(imgtype, FICA_OFFSET_IMG_HASH_LOC, crcval)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Get the application external flash start address, given the passed image type
uint32_t FICA_get_app_img_start_addr(uint32_t imgtype)
{
	uint32_t startaddr = FICA_IMG_DEFAULT_APP_ADDR;

	switch(imgtype)
	{
		case 1: startaddr = FICA_IMG_FAC_K64F_APP_ADDR; break;
		case 2: startaddr = FICA_IMG_FAC_K41Z_APP_ADDR; break;
		case 3: startaddr = FICA_IMG_NEW_K64F_APP_ADDR; break;
		case 4: startaddr = FICA_IMG_NEW_K41Z_APP_ADDR; break;
		case 5: startaddr = FICA_IMG_CUR_K64F_APP_ADDR; break;
		case 6: startaddr = FICA_IMG_CUR_K41Z_APP_ADDR; break;
		default: startaddr = FICA_IMG_DEFAULT_APP_ADDR; break; // default
	}
	return(startaddr);
}

//! @brief Get the Application Image Length
uint32_t FICA_get_app_img_len(uint32_t imgtype)
{
	uint32_t len=0;

	if(FICA_read_field(imgtype, FICA_OFFSET_IMG_SIZE, &len)!=SPI_FLASH_NO_ERROR)
		return(0);

	return(len);
}


//! @brief Write the image to external memory
int32_t FICA_write_image_info(uint32_t imgtype, uint32_t imgfmt, uint32_t len)
{
	// Check if its a valid image type
	if(imgtype>FICA_MAX_IMAGES)
		return(SPI_FLASH_ERROR);

	// Set the start address of the image based on the image type
	uint32_t startaddr = FICA_get_app_img_start_addr(imgtype);

	if(FICA_read_db() != SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write the rest of the record values for this image types ICA (Image Config Area)
	if(FICA_write_field_no_save(imgtype, FICA_OFFSET_IMG_DESC, FICA_IMG_DESC_ID)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	if(FICA_write_field_no_save(imgtype, FICA_OFFSET_IMG_TYPE, imgtype)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write Image Start Address into FICA Record for this image type
	if(FICA_write_field_no_save(imgtype, FICA_OFFSET_IMG_START, startaddr)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write Image Size into FICA Record for this image type
	if(FICA_write_field_no_save(imgtype, FICA_OFFSET_IMG_SIZE, len)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write Image Type into FICA Record for this image type
	if(FICA_write_field_no_save(imgtype, FICA_OFFSET_IMG_FMT, imgfmt)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write the info to the flash
	if(FICA_write_db()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

#ifdef UART_DEBUG
	sprintf(pmsg,"Flash ICA record written successfully\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Read ICA record from the buffer
int32_t FICA_read_record(uint32_t imgtype, FICA_Record *pimgrec)
{
	// Make sure pointer to image record is not NULL
	if(pimgrec==NULL) return(SPI_FLASH_ERROR);

	// Check if its a valid image type
	if(imgtype==0 || imgtype>FICA_MAX_IMAGES)
		return(SPI_FLASH_ERROR);

	// Calculate the offset in the ICA buffer
	uint32_t bufaddr = FICA_ICA_DEFINITION_SIZE + ((imgtype-1) * sizeof(FICA_Record));

	// Read the buffer data into the FICA record variable
	if(FICA_read_buf(bufaddr, sizeof(FICA_Record), pimgrec)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Write an ICA record to the FICA buffer and flash
int32_t FICA_write_record(uint32_t imgtype, FICA_Record *pimgrec)
{
	// Make sure pointer to image record is not NULL
	if(pimgrec==NULL) return(SPI_FLASH_ERROR);

	// Check if its a valid image type
	if(imgtype==0 || imgtype>FICA_MAX_IMAGES)
		return(SPI_FLASH_ERROR);

	// Calculate the offset in the ICA buffer
	uint32_t bufaddr = FICA_ICA_DEFINITION_SIZE + ((imgtype-1) * sizeof(FICA_Record));

	if(FICA_read_db()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write FICA record variable data to the buffer
	if(FICA_write_buf(bufaddr, sizeof(FICA_Record), pimgrec)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);

	// Write the info to the flash
	if(FICA_write_db()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

#ifdef UART_DEBUG
	sprintf(pmsg,"Flash ICA record written successfully\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif

	return(SPI_FLASH_NO_ERROR);
}

//! @brief Is Flash Image Config Area Initialized ?
bool is_FICA_initialized(void)
{
	uint32_t fieldval=0;
	uint32_t *pfieldval = &fieldval;
	uint32_t verval=0;
	uint32_t *pverval = &verval;
	uint32_t fieldaddr = 0;
//	char msg[100];

	// Get the field address based on the image type and field offset
	if(FICA_get_field_addr(FICA_IMG_TYPE_NONE, FICA_OFFSET_ICA_DESC, &fieldaddr)!=SPI_FLASH_NO_ERROR)
		return(false);

	// See if the Start ICA Identifier is there
	if(SPI_Flash_Read(fieldaddr, FICA_FIELD_SIZE, (void *)pfieldval)!=SPI_FLASH_NO_ERROR)
		return(false);

	// Get the field version based on the image type and field offset
	if(FICA_get_field_addr(FICA_IMG_TYPE_NONE, FICA_OFFSET_ICA_VER, &fieldaddr)!=SPI_FLASH_NO_ERROR)
		return(false);

	// See if the Start ICA Identifier is there
	if(SPI_Flash_Read(fieldaddr, FICA_FIELD_SIZE, (void *)pverval)!=SPI_FLASH_NO_ERROR)
		return(false);

	// Check ICA Start Identifier, value should be 0x5A5A5A5A
	if(fieldval == FICA_ICA_DESC && verval == FICA_VER)
	{
#ifdef UART_DEBUG
		sprintf(pmsg,"Flash ICA already initialized\r\n");
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
		return(true);
	}
#ifdef UART_DEBUG
	sprintf(pmsg,"Flash ICA needs initialization\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
	return(false);
}

//! @brief Check if FICA is initialized, initialize it if needed
int32_t FICA_initialize(void)
{

	if(FICA_verify_ext_flash()!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

#ifdef UART_DEBUG
	sprintf(pmsg,"\r\nChecking Image Config Area (ICA) initialization\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif

	FICA_RECORD_SIZE = sizeof(FICA_Record);

	// If its already initialized, return no error
	if(is_FICA_initialized())
	{
		if(FICA_read_db()!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);

		return(SPI_FLASH_NO_ERROR);
	}

#ifdef UART_DEBUG
	sprintf(msg,"Flash ICA initialization started\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
#endif

	FICA_clear_buf(ProgExtAppImgBuf, 0);

	// ICA is not initialized, so initialize it, what are you waiting for, xmas, what the...
	// Write the ICA Start ID 0x5A5A5A5A
	if(FICA_write_field(FICA_IMG_TYPE_NONE, FICA_START_ADDR, FICA_ICA_DESC)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Write the ICA Version Number
	if(FICA_write_field(FICA_IMG_TYPE_NONE, FICA_OFFSET_ICA_VER, FICA_VER)!=SPI_FLASH_NO_ERROR)
		return(SPI_FLASH_ERROR);

	// Initialize all the records
	for(int imgtype = 1; imgtype <= FICA_NUM_IMG_TYPES; imgtype++)
	{
		// Write defaults to all Image Records
		if(FICA_write_image_info(imgtype, FICA_IMG_FMT_NONE, 0)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);
	}

#ifdef UART_DEBUG
	sprintf(msg,"Flash ICA initialization complete\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
#endif

	return(SPI_FLASH_NO_ERROR);

}

//! @brief Initialize External Flash driver
int32_t FICA_verify_ext_flash(void)
{
	// Initialize RPK External Flash (MT25QL128)
	uint32_t result;
	uint8_t vendorId = 0U;
	uint8_t devId[2];

#ifdef UART_DEBUG
	sprintf(pmsg,"**** VERSION %s       ****\n\n", TEST_VERSION);
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));

	strcpy(pmsg,"RPK SPI FLASH Init\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif

	// Read flash vendor ID
	result = SPI_Flash_Readid(&vendorId, devId);
	if (result)
	{
#ifdef UART_DEBUG
		strcpy(pmsg,"   Can not find any SPI Flash device!\r\n");
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
		return(SPI_FLASH_ERROR);
	}
#ifdef UART_DEBUG
	else {
		strcpy(pmsg,"   found SPI FLASH\n");
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
		sprintf(pmsg,"   vendor ID : 0x%X\n   device ID : 0x%X.0x%X\n------------------------------\n",vendorId,devId[0],devId[1]);
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
	}
#endif

	/* check if it's Micron chip or not */
	if (SPI_FLASH_MICRON_VENDER_ID == vendorId)
	{
		if (SPI_Flash_Configure(FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, 256))
		{
#ifdef UART_DEBUG
			strcpy(pmsg,"   Failed to init SPI flash chip!\r\n");
			PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
			return(SPI_FLASH_ERROR);
		}
#ifdef UART_DEBUG
		strcpy(pmsg,"   SPI flash initialized!\r\n");
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
	}
	else
	{
#ifdef UART_DEBUG
		sprintf(pmsg,"   Not a Micron SPI flash chip! VID:%x\r\n", vendorId);
		PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
#endif
		return(SPI_FLASH_ERROR);
	}
	return(SPI_FLASH_NO_ERROR);
}


//! @brief Resets this MCU
void reset_mcu(void)
{
    NVIC_SystemReset();
    while (1);
}

/************************************************************************************
*  Updates the CRC based on the received data to process.
*  Updates the global CRC value. This was determined to be optimal from a resource
*  consumption POV.
*
************************************************************************************/
uint16_t FICA_compute_chunk_CRC(uint8_t *pData, uint16_t lenData, uint16_t crcValueOld)
{
    uint8_t i;

    while(lenData--)
    {
        crcValueOld ^= (uint16_t)((uint16_t)*pData++ << 8);
        for( i = 0; i < 8; ++i )
        {
            if( crcValueOld & 0x8000 )
            {
                crcValueOld = (crcValueOld << 1) ^ 0x1021U;
            }
            else
            {
                crcValueOld = crcValueOld << 1;
            }
        }
    }
    return crcValueOld;
}


//! @brief Gets the Processor Communication Flag
int32_t FICA_get_comm_flag(uint32_t comflag, bool *flagstate)
{
	uint32_t retval = 0;
	if(FICA_read_field(FICA_IMG_ICA_DEF_ADDR, FICA_OFFSET_ICA_COMM, &retval)!=SPI_FLASH_NO_ERROR)
	{
		SPI_Bus_Share_Release_Access();
		return(SPI_FLASH_ERROR);
	}
	if((retval & comflag)>0)
		*flagstate = true;
	else
		*flagstate = false;
	SPI_Bus_Share_Release_Access();
	return(SPI_FLASH_NO_ERROR);
}


//! @brief Sets the Processor Communication Flag
int32_t FICA_set_comm_flag(uint32_t comflag, bool flagstate)
{
	uint32_t readflags = 0;
	uint32_t cmpval = 0;

	if(FICA_read_field(FICA_IMG_ICA_DEF_ADDR, FICA_OFFSET_ICA_COMM, &readflags)!=SPI_FLASH_NO_ERROR)
	{
		SPI_Bus_Share_Release_Access();
		return(SPI_FLASH_ERROR);
	}
	cmpval = comflag & readflags;
	if(!flagstate && cmpval>0)
	{
		readflags &= ~comflag;
		if(FICA_write_field(FICA_IMG_ICA_DEF_ADDR, FICA_OFFSET_ICA_COMM, readflags)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);
	}
	else if(flagstate && cmpval==0)
	{
		readflags |= comflag;
		if(FICA_write_field(FICA_IMG_ICA_DEF_ADDR, FICA_OFFSET_ICA_COMM, readflags)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);
	}
	return(SPI_FLASH_NO_ERROR);
}


//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
