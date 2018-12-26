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
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "spi_flash_driver.h"
#include "pin_mux.h"
#include "pin_mux_rpk.h"

#include "rpk_led.h"

#if defined(FSL_RTOS_FREE_RTOS)
#include "fsl_os_abstraction.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif
#include "spi_bus_share.h"
#if defined(CPU_MK64FN1M0VMD12)
#include "port_interrupts.h"
#endif
#if !defined(FSL_RTOS_FREE_RTOS)
#include "microseconds.h"
#endif
//#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
//#include "portmacro.h"
//#endif

//! @addtogroup spi_bus_share
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

// Uncomment to use SPI bus flow control between the K64 and K41 processors
#define RPK_USE_FLOW_CONTROL 1
volatile bool bSPIBusgInitialized = false;
volatile bool mSPIBusLocked = false;
volatile bool mSPIBusRequest = false;

volatile static int icount = 0;
volatile static int IRQCount = 0;
volatile static int IRQCountSave = 0;


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize SPI Bus so it can be used
status_t SPI_Bus_Share_Init(void)
{
	if (!bSPIBusgInitialized)
	{
		SPI_Bus_Share_Release_Access();
		bSPIBusgInitialized = true;
	}
	return(SPI_BUS_SHARE_NO_ERROR);
}


#if defined(CPU_MK64FN1M0VMD12)
void BOARD_K41Z_RTS_IRQ_HANDLER(uint32_t pin_nb)
{
	IRQCount+=1;

	if(!bSPIBusgInitialized)
		return;

	if(!mSPIBusLocked && !mSPIBusRequest)
		   GPIO_WritePinOutput(BOARD_INITPINS_KW41_UART_CTS_GPIO, BOARD_INITPINS_KW41_UART_CTS_PIN, 1);

	GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_KW41_UART_RTS_GPIO, 1U << BOARD_INITPINS_KW41_UART_RTS_PIN);
}
#endif


//! @brief Uninitialize SPI Bus so it can be used by another app / device
status_t SPI_Bus_Share_DeInit(void)
{
	SPI_Bus_Share_Release_Access();
	return(SPI_BUS_SHARE_NO_ERROR);
}


status_t SPI_Bus_Share_Get_Access(void)
{
	if(mSPIBusLocked)
		return(SPI_BUS_SHARE_NO_ERROR);

	// Make sure this driver is initialized
	if (!bSPIBusgInitialized)
		SPI_Bus_Share_Init();

	SPI_Bus_Share_Wait_For_Access();
	OS_App_WaitUsec(20);
	BOARD_ConfigurePins_RPK_SPI_Bus();
	OS_App_WaitUsec(20);
	icount+=1;
	return(SPI_BUS_SHARE_NO_ERROR);
}


status_t SPI_Bus_Share_Release_Access(void)
{
	if(!mSPIBusLocked)
		return(SPI_BUS_SHARE_NO_ERROR);

	OS_App_WaitUsec(20);
	BOARD_UnConfigurePins_RPK_SPI_Bus();
	OS_App_WaitUsec(20);
	icount+=1;

#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
	// Release SPI access to K64F by setting RTS pin low
    GPIO_WritePinOutput(BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO, BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO_PIN, 0);
#endif

	mSPIBusLocked = false;
	return(SPI_BUS_SHARE_NO_ERROR);
}

status_t SPI_Bus_Share_Wait_For_Access(void)
{

#ifndef RPK_USE_FLOW_CONTROL
	return(SPI_BUS_SHARE_NO_ERROR);
#endif

	// PC4 (CTS) connected to PE27
	// PC5 (RTS) connected to PE26
#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)

	uint32_t i=0;
	icount=0;

	// Wait until the K64F approves the request
	while(1)
	{
		// Request SPI access from the K64F
		GPIO_WritePinOutput(BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO, BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO_PIN, 0);
		OS_App_WaitUsec(1);
		GPIO_WritePinOutput(BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO, BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO_PIN, 1);
		OS_App_WaitUsec(1);
		for(i=0;i<100;i++)
		{
			if(GPIO_ReadPinInput(BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO, BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO_PIN)!=0)
			{
				OS_App_WaitUsec(1);
				if(GPIO_ReadPinInput(BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO, BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO_PIN)!=0)
					break;
			}
			icount+=1;
		}
		if(i<100)
		{
			icount+=1;
			mSPIBusLocked=true;
			break;
		}
	}

#elif defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
	if(GPIO_ReadPinInput(BOARD_INITPINS_KW41_RST_GPIO, BOARD_INITPINS_KW41_RST_GPIO_PIN)!=0)
	{
		while(1)
		{
			mSPIBusRequest = true;
			IRQCountSave = IRQCount;
			if(GPIO_ReadPinInput(BOARD_INITPINS_KW41_UART_RTS_GPIO, BOARD_INITPINS_KW41_UART_RTS_PIN)==0)
			{
				GPIO_WritePinOutput(BOARD_INITPINS_KW41_UART_CTS_GPIO, BOARD_INITPINS_KW41_UART_CTS_PIN, 0);
				if(IRQCountSave == IRQCount)
				{
					mSPIBusLocked = true;
					break;
				}
			}
			else
			{
				GPIO_WritePinOutput(BOARD_INITPINS_KW41_UART_CTS_GPIO, BOARD_INITPINS_KW41_UART_CTS_PIN, 1);
				while(GPIO_ReadPinInput(BOARD_INITPINS_KW41_UART_RTS_GPIO, BOARD_INITPINS_KW41_UART_RTS_PIN)!=0)
				{
					OS_App_WaitMsec(1);
				}
				GPIO_WritePinOutput(BOARD_INITPINS_KW41_UART_CTS_GPIO, BOARD_INITPINS_KW41_UART_CTS_PIN, 0);
				while(GPIO_ReadPinInput(BOARD_INITPINS_KW41_UART_RTS_GPIO, BOARD_INITPINS_KW41_UART_RTS_PIN)!=0)
				{
					OS_App_WaitMsec(1);
				}
				mSPIBusLocked = true;
				break;
			}
		}
	}
	else
	{
		mSPIBusLocked = true;
	}
#endif
	mSPIBusRequest = false;
	return(SPI_BUS_SHARE_NO_ERROR);
}


//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
