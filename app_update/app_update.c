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

#include <stdbool.h>
#include "fsl_common.h"
#include "fsl_gpio.h"
//#include "memory.h"
#include "app_update.h"
#include "spi_flash_driver.h"
#include "flash_ica_driver.h"
#include "app_program_ext.h"
#include "app_switches.h"
#include "pin_mux.h"
#include "pin_mux_rpk.h"
#include "rpk_led.h"
#if defined(CPU_MK64FN1M0VMD12)
#include "rgb_led.h"
#endif
#include "board.h"
#ifdef UART_DEBUG
#include "fsl_uart.h"
#endif
#include "microseconds.h"
#include "spi_bus_share.h"

#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
#include "usb.h"
#include "usb_device_config.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_descriptor.h"
#include "usb_device_msc.h"
#endif

// Uncomment this to invalidate the current image to force an update
//#define INVALIDATE_CUR_IMAGE

// Uncomment to Erase all flash before reprogramming flash app start to flash app end
//#define ERASE_ALL_APP_FLASH

// Uncomment to Erase all SPI flash before programming flash app starts
// This will force the FICA Index table to rebuild as well
//#define ERASE_ALL_SPI_FLASH

// Uncomment to force into MSD mode
//#define FORCE_MSD

//! @addtogroup app_update
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

static FICA_Record curappinfo = {0};
static FICA_Record newappinfo = {0};
static uint32_t curndx = 0;
static uint32_t newndx = 0;
static appstatus_t AppUpdStatus = kAppStatus_Success;
static uint8_t updbuf[EXT_FLASH_ERASE_PAGE]={0};

#ifdef UART_DEBUG
static char printmsg[100]={0};
char *pmsg = printmsg;
#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Continue page blocking image program to external flash, returns kAppStatus_TransferComplete if successful
appstatus_t apu_blocking(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype)
{

	appstatus_t status = kAppStatus_Success;

#ifdef UART_DEBUG
	PrintMsg("Application Update Blocking Start...");
#endif

#if defined(CPU_MK64FN1M0VMD12)
	BOARD_Put_K41_InReset();
	apu_blocking_k64(curimgtype, newimgtype, facimgtype);
	SPI_Bus_Share_Release_Access();
	OS_App_WaitUsec(100);
	BOARD_Release_K41_Reset();
	apu_wait_for_k41();
#else
	apu_blocking_k41(curimgtype, newimgtype, facimgtype);
	SPI_Bus_Share_Release_Access();
#endif

	return(status);
}

#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
//! @brief Continue page blocking image program to external flash, returns kAppStatus_TransferComplete if successful
appstatus_t apu_blocking_k64(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype)
{
	appstatus_t status = kAppStatus_Success;
	bool progflag = false;

	// Initialize the Application Update Driver
	if(apu_init(curimgtype, newimgtype)!=kAppStatus_Success)
	{
		return(kAppStatus_Fail);
	}

#define CLEAR_MSD
#ifdef CLEAR_MSD
	apu_clear_msd();
#endif

#ifdef INVALIDATE_CUR_IMAGE
	if(FICA_write_field(FICA_IMG_TYPE_K64F_CUR, FICA_OFFSET_IMG_HASH_LOC, 0x00000000)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);
#endif

#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
#if defined(FORCE_MSD)
	USB_DeviceApplicationInit();
#else

	// if SW1, SW2, SW3 pressed, enter factory application programming mode
	if((Is_MSD_Button_Pressed() && Is_KW41Z_Button_Pressed() && Is_K64F_Button_Pressed()) || Is_Factory_User_Button_Pressed())
	{
		if(Is_Factory_User_Button_Pressed())
			USB_DeviceApplicationInit(MODE_FACT_USER);
		else
			USB_DeviceApplicationInit(MODE_MSD_FACT);
//		return(kAppStatus_Success);
	}
	// if MSD(SW3) button pressed, enter MSD mode
	if(Is_MSD_Button_Pressed())
	{
		USB_DeviceApplicationInit(MODE_MSD_USER);
//		return(kAppStatus_Success);
	}
#endif // FORCE_MSD
#endif // RAPID_IOT

	// If factory application replacement wanted, set factory image type and continue to programming below
	if(apu_is_fac_app_needed(curimgtype, facimgtype))
	{
		newimgtype = facimgtype;
	} // If new application needed, continue to programming below
	else if(!apu_is_new_app_needed(curimgtype, newimgtype))
	{
		return(status);
	}


#ifdef ERASE_ALL_APP_FLASH
	// Erase Internal Flash and Program it from the buffer
	for(int i = 0; i < 236; i++) // Erase all pages above the bootloader, K64 page size is 4096, and app start address was 0x14000 when i wrote this
	{
		// Erase the Internal Flash Page
		if(mem_erase((APP_UPDATE_VECTOR_TABLE_ADDRESS+(i*INT_FLASH_ERASE_PAGE)), INT_FLASH_ERASE_PAGE)!=kStatus_Success)
			return(kAppStatus_Fail);
	}
#endif

#ifdef ERASE_ALL_SPI_FLASH

	// Erase Internal Flash and Program it from the buffer
	for(int i = 0; i < 256; i++)
	{
		// Erase the External Flash Page
		if(SPI_Flash_Erase_Block(i*EXT_FLASH_ERASE_PAGE, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);
	}
#endif

	// Start the transfer from External Flash to Internal Flash
	status = apu_start(newimgtype);
	if(status==kAppStatus_TransferInProgress)
	{
		// Move the new application from external flash to internal flash
		status = apu_program(curimgtype, newimgtype);
	}
	if(status == kAppStatus_TransferPassed)
		FICA_set_comm_flag(ICA_COMM_K64_FACT_BOOT, false);
	return(status);
}

#else

//! @brief Continue page blocking image program to external flash, returns kAppStatus_TransferComplete if successful
appstatus_t apu_blocking_k41(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype)
{

	appstatus_t status = kAppStatus_Success;
	bool progflag = false;

	// Initialize the Application Update Driver
	if(apu_init(curimgtype, newimgtype)!=kAppStatus_Success)
	{
		return(kAppStatus_Fail);
	}

	if(apu_is_fac_app_needed(curimgtype, facimgtype))
	{
		newimgtype = facimgtype;
	}
	else if(!apu_is_new_app_needed(curimgtype, newimgtype))
	{
		return(status);
	}

#ifdef ERASE_ALL_APP_FLASH

	// Erase Internal Flash and Program it from the buffer
	for(int i = 0; i < 248; i++) // Erase all pages above the bootloader. K41 page size is 2048, and app start address was 0x4000 when i wrote this
	{
		// Erase the Internal Flash Page
		if(mem_erase((APP_UPDATE_VECTOR_TABLE_ADDRESS+(i*INT_FLASH_ERASE_PAGE)), INT_FLASH_ERASE_PAGE)!=kStatus_Success)
			return(kAppStatus_Fail);
	}
#endif

#ifdef ERASE_ALL_SPI_FLASH

	// Erase Internal Flash and Program it from the buffer
	for(int i = 0; i < 256; i++)
	{
		// Erase the External Flash Page
		if(SPI_Flash_Erase_Block(i*EXT_FLASH_ERASE_PAGE, EXT_FLASH_ERASE_PAGE)!=SPI_FLASH_NO_ERROR)
			return(SPI_FLASH_ERROR);
	}
#endif

	// Reset the load factory image bit on the KW41Z, so it won't try to load it again after this one
	FICA_set_comm_flag(ICA_COMM_K41_PROGRAM, true);

	// Start the transfer from External Flash to Internal Flash
	status = apu_start(newimgtype);
	if(status==kAppStatus_TransferInProgress)
	{
		// Move the new application from external flash to internal flash
		status = apu_program(curimgtype, newimgtype);
	}

	// Reset the load factory image bit on the KW41Z, so it won't try to load it again after this one
	FICA_set_comm_flag(ICA_COMM_K41_PROGRAM, false);
	return(status);
}
#endif

//! @brief Continue page blocking image program to external flash, returns kAppStatus_TransferComplete if complete
appstatus_t apu_program(uint32_t curimgtype, uint32_t newimgtype)
{

	uint32_t curlen = 0;
	uint32_t curwritelen = 0;
	uint32_t curintpage = 0;
	uint32_t curextpage = 0;
	uint32_t inttoextratio = EXT_FLASH_ERASE_PAGE / INT_FLASH_ERASE_PAGE;

#ifdef UART_DEBUG
	PrintMsg("Application Update Program...");
#endif

	// No valid image there
	if(newappinfo.imgaddr == 0xFFFFFFFF || newappinfo.imgaddr == 0)
		return(kAppStatus_Fail);

	// can't handle this case yet
	if(INT_FLASH_ERASE_PAGE > EXT_FLASH_ERASE_PAGE)
		return(kAppStatus_Fail);

	// Write Image a page at a time
	while(curlen < newappinfo.imglen)
	{
#if defined(BLINK_PROGRESS)
#if defined(CPU_MK64FN1M0VMD12)
		Blink_LED_RGB(RGB_LED_BRIGHT_LOW, RGB_LED_COLOR_GREEN);
#else
		Blink_LED(BOARD_INITLEDS_LED_WHITE_GPIO_PIN);
#endif
#endif

		// Write either a page or less if only less than a page is left
		curwritelen = (newappinfo.imglen - curlen) >= EXT_FLASH_ERASE_PAGE ? EXT_FLASH_ERASE_PAGE : (newappinfo.imglen - curlen);

		// Read from external and write to internal flash
		if(apu_move_ext_page_to_int_page(APP_UPDATE_VECTOR_TABLE_ADDRESS, newappinfo.imgaddr, curintpage, curextpage, updbuf, INT_FLASH_ERASE_PAGE, EXT_FLASH_ERASE_PAGE, curwritelen)!=kAppStatus_Success)
			return(kAppStatus_Fail);

		OS_App_WaitUsec(1);

		curextpage++;
		curintpage+=inttoextratio;
		curlen += curwritelen;
	}

	if(apu_finish(curimgtype)!=kAppStatus_Success)
		return(kAppStatus_Fail);

#if defined(CPU_MKW41Z512VHT4)
	GPIO_SetPinsOutput(GPIOA, (1 << BOARD_INITLEDS_LED_WHITE_GPIO_PIN));
	GPIO_SetPinsOutput(GPIOA, (1 << BOARD_INITLEDS_LED_BLUE_GPIO_PIN));

	for(uint16_t i=0;i<10;i++)
	{
		OS_App_WaitMsec(500);
		GPIO_TogglePinsOutput(GPIOA, (1 << BOARD_INITLEDS_LED_WHITE_GPIO_PIN));
		GPIO_TogglePinsOutput(GPIOA, (1 << BOARD_INITLEDS_LED_BLUE_GPIO_PIN));
	}
#endif

	return(kAppStatus_TransferPassed);
}


//! @brief Read data from external flash, erase the internal flash page, program the data to the internal flash
appstatus_t apu_move_ext_page_to_int_page(uint32_t intimgaddr, uint32_t extimgaddr, uint32_t intpagenum, uint32_t extpagenum, uint8_t *pbuf, uint32_t intpagelen, uint32_t extpagelen, uint32_t writelen)
{
	uint32_t curintlen = 0;
	uint32_t curintpage = intpagenum;
	uint32_t curwritelen = 0;

#ifdef UART_DEBUG
	PrintMsg("AU Read External Page...");
#endif
	// Clear out the page buffer, hopefully the flash read will overwrite it all anyway
//	apu_clear_buf(pbuf, extpagelen);

	// Erase Internal Flash and Program it from the buffer
	while(curintlen < writelen)
	{
		// Write either a page or less if only less than a page is left
		curwritelen = (writelen - curintlen) >= intpagelen ? intpagelen : (writelen - curintlen);

		// Read the image from the external flash
		if(SPI_Flash_Read((extimgaddr+(curintpage*intpagelen)), intpagelen, pbuf)!=SPI_FLASH_NO_ERROR)
			return(kAppStatus_Fail);

		if(curwritelen<intpagelen)
		{
			uint8_t *skipbuf = &pbuf[curwritelen];
			uint32_t fillsize = intpagelen - curwritelen;

			if(mem_read((intimgaddr+(curintpage*intpagelen))+curwritelen, fillsize, skipbuf)!=kStatus_Success)
				return(kAppStatus_Fail);
		}

		// Erase the Internal Flash Page
#ifdef UART_DEBUG
		PrintMsg("AU Erase Internal Page...");
#endif
		if(mem_erase((intimgaddr+(curintpage*intpagelen)), intpagelen)!=kStatus_Success)
			return(kAppStatus_Fail);

		// Write the image to the internal flash
#ifdef UART_DEBUG
		PrintMsg("AU Write Internal Page...");
#endif

		if(mem_write((intimgaddr+(curintpage*intpagelen)), intpagelen, pbuf)!=kStatus_Success)
			return(kAppStatus_Fail);

		curintlen += curwritelen;
		pbuf += curwritelen;
		curintpage++;
	}
	return(kAppStatus_Success);
}

//! @brief Start application image read from external flash and program to internal flash
// This function just gets things initialized, the apu_cont function does the actual work
appstatus_t apu_start(uint32_t newimgtype)
{

#ifdef UART_DEBUG
	PrintMsg("Application Update Start...");
#endif

	curndx = 0;
	newndx = 0;

	// Read the new application information
	if(FICA_read_record(newimgtype, &newappinfo)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	return(kAppStatus_TransferInProgress);
}

//! @brief Finish application image update by writing hash value to FICA
appstatus_t apu_finish(uint32_t curimgtype)
{
	uint32_t crcval = 0;

#ifdef UART_DEBUG
	PrintMsg("AU Write New Record to Current Record...");
#endif

	if(FICA_Calculate_CRC_Internal(APP_UPDATE_VECTOR_TABLE_ADDRESS, newappinfo.imglen, &crcval)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	// Write the new application information to the current information record
	if(FICA_write_record(curimgtype, &newappinfo)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

#ifdef UART_DEBUG
	PrintMsg("AU Read Current Record...");
#endif

	// Read the current application information back, should be the same, but life doesn't always make sense
	if(FICA_read_record(curimgtype, &curappinfo)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	return(kAppStatus_Success);
}

//! @brief Initialize the external flash if needed
appstatus_t apu_init()
{
	AppUpdStatus=kAppStatus_Success;

#ifdef UART_DEBUG
	PrintMsg("Application Update Initialization...");

	PrintMsg("AU FICA Initialize...");
#endif
	// Initialize the Flash ICA (Image Configuration Area)
	if(FICA_initialize()!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	return(kAppStatus_Success);
}


//! @brief Calculates the CRC for the passed Image Type and length
// If len=0, it uses the length stored in the ICA for that Image Type
appstatus_t apu_calculate_crc(uint32_t imgtype, uint32_t len, uint32_t *pcrc)
{
	// If len=0, use the length stored in the ICA for that Image Type
	if(len==0)
	{
		if(FICA_read_field(imgtype, FICA_OFFSET_IMG_SIZE, &len)!=SPI_FLASH_NO_ERROR)
			return(kAppStatus_Fail);
	}

	if(FICA_Calculate_CRC(imgtype, pcrc)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	return(kAppStatus_Success);
}

//! @brief Programs the CRC for the passed Image Type
appstatus_t apu_program_crc(uint32_t imgtype, uint32_t crc)
{
	if(FICA_Program_CRC(imgtype, crc)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	return(kAppStatus_Success);
}



//! @brief Get the current status of this driver
appstatus_t apu_info_get_status()
{
	return(AppUpdStatus);
}


//! @brief Check if a new app is present
bool apu_is_new_app_needed(uint32_t curimgtype, uint32_t newimgtype)
{
	// New Flash CRC value, new application
	uint32_t newcrcval = 0;
	uint32_t *pnewcrcval = &newcrcval;

	// Current Flash CRC value, current application
	uint32_t curcrcval = 0;
	uint32_t *pcurcrcval = &curcrcval;

	// Check if Flash ICA is initialized, if not, it will try to initialize it
	if(is_FICA_initialized())
	{
		// Read the CRC value from the Image Location
		if (FICA_read_field(newimgtype, FICA_OFFSET_IMG_HASH_LOC, (void *)pnewcrcval)!=SPI_FLASH_NO_ERROR)
			return(false);

		// return false if the new app crc is 0 or all Fs
		if(newcrcval==0 || newcrcval==0xFFFFFFFF)
			return(false);

		// New app is ready, read the current app CRC value from the Current Image Location
		if (FICA_read_field(curimgtype, FICA_OFFSET_IMG_HASH_LOC, pcurcrcval)!=SPI_FLASH_NO_ERROR)
			return(false);

		// if the current app crc value doesn't match the new app crc value, return true
		if(newcrcval!=curcrcval)
		{
			// Check if the application image is valid
		    if(apu_validate_image_crc(newimgtype)==kAppStatus_Success)
		    	return(true);
		}
		return(false);
	}
	return(false);
}


//! @brief Check if factory application is needed
bool apu_is_fac_app_needed()
{
	bool progflag = false;

#if defined(CPU_MK64FN1M0VMD12)

	if(FICA_get_comm_flag(ICA_COMM_K64_FACT_BOOT, &progflag)!=SPI_FLASH_NO_ERROR)
		return(false);

	if(Is_K64F_Button_Pressed() || progflag>0)
	{
		// Invalidate the User application so it doesn't try to immediately load after the factory image goes in
		if(FICA_write_field(FICA_IMG_TYPE_K64F_USER, FICA_OFFSET_IMG_HASH_LOC, 0x00000000)!=SPI_FLASH_NO_ERROR)
			return(false);

		return(true);
	}

	if(Is_KW41Z_Button_Pressed())
		FICA_set_comm_flag(ICA_COMM_K41_FACT_BOOT, true);

	OS_App_WaitUsec(5);
	return(false);

#elif defined(CPU_MKW41Z512VHT4)

	// Check if factory reset is needed
	if(FICA_get_comm_flag(ICA_COMM_K41_FACT_BOOT, &progflag)!=SPI_FLASH_NO_ERROR)
		return(false);

	// If load factory image bit is set, load factory image
	if(progflag)
	{
		// Invalidate the User application so it doesn't try to immediately load after the factory image goes in
		if(FICA_write_field(FICA_IMG_TYPE_K41Z_USER, FICA_OFFSET_IMG_HASH_LOC, 0x00000000)!=SPI_FLASH_NO_ERROR)
			return(false);

		return(true);
	}

#endif

	return(false);
}


//! @brief Validate the CRC of the application in external flash
// This function will invalidate the image if the crc is not valid
appstatus_t apu_validate_image_crc(uint32_t imgtype)
{
	// CRC calculated from image in external flash
	uint32_t calculatedcrc = 0;
	uint32_t *pcalculatedcrc = &calculatedcrc;

	// CRC read from FICA for the passed image type
	uint32_t readcrc = 0;
	uint32_t *preadcrc = &readcrc;

	// read the image CRC value from the Image FICA Location
	if (FICA_read_field(imgtype, FICA_OFFSET_IMG_HASH_LOC, preadcrc)!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	// If the read crc is zero, its either not initialized or its already been invalidated
	if(readcrc==0)
		return(kAppStatus_Fail);

	// Calculate the CRC from external flash for the passed image
	if(app_program_ext_get_crc(imgtype, pcalculatedcrc)!=IMG_EXT_NO_ERROR)
		return(kAppStatus_Fail);

	// If the calculated crc is zero, return error so the app update will not run
	if(calculatedcrc==0)
		return(kAppStatus_Fail);

	for(uint32_t j=0;j<20;j++)
	{
		if(readcrc==calculatedcrc)
			break;
		app_program_ext_get_crc(imgtype, pcalculatedcrc);
	}

	// Check if the calculated crc from the external flash matches the one stored in the FICA table for the passed image type
	if(calculatedcrc!=readcrc)
	{
		// CRCs don't match, invalidate this application
		FICA_write_field(imgtype, FICA_OFFSET_IMG_HASH_LOC, 0);
		Blink_Fail_LED();
		return(kAppStatus_Fail);
	}
	// No issue, a good application is ready to be loaded
	return(kAppStatus_Success);
}

//! @brief Clears the passed buffer
void apu_clear_buf(uint8_t *buf, uint32_t len)
{
	for(int j = 0; j < len; j++) buf[j]=0xFF;
}

void apu_clear_msd()
{
	FICA_set_comm_flag(ICA_COMM_K64_FACT_BOOT, false);
	FICA_set_comm_flag(ICA_COMM_K41_FACT_BOOT, false);
	FICA_set_comm_flag(ICA_COMM_K41_PROGRAM, false);
}

#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)

#define K41_BOOT_UPDATING_TIMEOUT 60000000  // 60s

//! @brief Wait for KW41Z to complete external flash based updates before continuing
appstatus_t apu_wait_for_k41()
{
	volatile uint64_t endtime = microseconds_get_ticks() + microseconds_convert_to_ticks(K41_BOOT_UPDATING_TIMEOUT);
	volatile uint64_t curtime = microseconds_get_ticks();
	bool progflag = false;

	Blink_Init();
	OS_App_WaitMsec(1000);

	// Wait for K41 to request access to the external flash bus
	while(curtime<endtime)
	{
		// Break if K41Z is not programming external flash
		FICA_get_comm_flag(ICA_COMM_K41_PROGRAM, &progflag);

		if(!progflag)
			break;

		OS_App_WaitMsec(1000); // 1 sec

		curtime = microseconds_get_ticks();
    	Blink_LED_RGB(RGB_LED_BRIGHT_LOW, RGB_LED_COLOR_WHITE);
	}

	OS_App_WaitMsec(2000); // 2 sec
	return(kAppStatus_Success);
}
#endif


#ifdef UART_DEBUG

void InitTerminalUART()
{

	uart_config_t terminal_config;
	char msg[100];

	/* terminal config */
	UART_GetDefaultConfig(&terminal_config);
	terminal_config.baudRate_Bps = TERMINAL_UART_BAUDRATE;
	terminal_config.enableTx = true;
	terminal_config.enableRx = true;

	UART_Init(TERMINAL_UART, &terminal_config, TERMINAL_UART_CLK_FREQ);

	strcpy(msg,"RPK Bootloader Initialized\n");
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
}

void PrintUart(UART_Type *base, const uint8_t *data, size_t length)
{
 	UART_WriteBlocking(base, data, length);
}

void PrintMsg(char *msg)
{
	sprintf(pmsg, "%s %s", msg, "\r\n");
	PrintUart(TERMINAL_UART, (uint8_t *)pmsg, strlen(pmsg));
}
#endif

//#define TEST_CODE 1
#ifdef TEST_CODE
	uint8_t buf[1025]={0};
	uint8_t buf2[1025]={0};

	for(uint32_t i=0; i<1024; i++)
		buf[i]=i+1;

	if(FICA_verify_ext_flash()!=SPI_FLASH_NO_ERROR)
		return(kAppStatus_Fail);

	SPI_Flash_Erase_Block(0, EXT_FLASH_ERASE_PAGE);

	SPI_Flash_Read(0, 5, buf2);

	SPI_Flash_Write(0, 5, buf);

	SPI_Flash_Read(0, 5, buf2);

	SPI_Flash_Erase_Block(0, EXT_FLASH_ERASE_PAGE);

	SPI_Flash_Write(0, 512, buf);

	SPI_Flash_Read(257, 20, buf2);

#endif

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
