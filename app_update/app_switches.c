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

#if defined(CPU_MK64FN1M0VMD12)

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "app_switches.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "pin_mux_rpk.h"
#include "spi_bus_share.h"


//! @addtogroup rpk_switch
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

// Uncomment to use the Switch IRQ to detect switch presses
//#define USE_SWITCH_IRQ
#ifdef USE_SWITCH_IRQ

/* user switches variables */
volatile bool g_ButtonPress_SW1 = false;
volatile bool g_ButtonPress_SW2 = false;
volatile bool g_ButtonPress_SW3 = false;
volatile bool g_ButtonPress_SW4 = false;

void Init_Switches(void)
{
	PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW1_PORT, BOARD_INITPINS_USER_SW1_GPIO_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW2_PORT, BOARD_INITPINS_USER_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW3_PORT, BOARD_INITPINS_USER_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW4_PORT, BOARD_INITPINS_USER_SW4_GPIO_PIN, kPORT_InterruptFallingEdge);
	EnableIRQ(BOARD_SW_IRQ);

	return;
}

void BOARD_SW_IRQ_HANDLER(void) {

	uint32_t pin_nb = PORT_GetPinsInterruptFlags(BOARD_INITPINS_USER_SW1_PORT);
	BOARD_SWITCH_IRQ_HANDLER(pin_nb);
	BOARD_K41Z_RTS_IRQ_HANDLER(pin_nb);
}


void BOARD_SWITCH_IRQ_HANDLER(uint32_t pin_nb)
{

	if (pin_nb & (1 << BOARD_INITPINS_USER_SW1_GPIO_PIN)){
		g_ButtonPress_SW1 = true;
		GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW1_GPIO, 1U << BOARD_INITPINS_USER_SW1_GPIO_PIN);
	}

	if (pin_nb & (1 << BOARD_INITPINS_USER_SW2_GPIO_PIN)){
		g_ButtonPress_SW2 = true;
		GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW2_GPIO, 1U << BOARD_INITPINS_USER_SW2_GPIO_PIN);
	}

	if (pin_nb & (1 << BOARD_INITPINS_USER_SW3_GPIO_PIN)){
		g_ButtonPress_SW3 = true;
		GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW3_GPIO, 1U << BOARD_INITPINS_USER_SW3_GPIO_PIN);
	}

	if (pin_nb & (1 << BOARD_INITPINS_USER_SW4_GPIO_PIN)){
		g_ButtonPress_SW4 = true;
		GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW4_GPIO, 1U << BOARD_INITPINS_USER_SW4_GPIO_PIN);
	}
}

bool Is_K64F_Button_Pressed(void)
{
	return(g_ButtonPress_SW1);
}

bool Is_KW41Z_Button_Pressed(void)
{
	return(g_ButtonPress_SW2);
}

void Clear_K64F_Button_Press(void)
{
	g_ButtonPress_SW1 = false;
}

void Clear_KW41Z_Button_Press(void)
{
	g_ButtonPress_SW2 = false;
}

#else // USE_SWITCH_IRQ


bool Is_K64F_Button_Pressed(void)
{
	uint32_t butpress = GPIO_ReadPinInput(BOARD_INITPINS_USER_SW1_GPIO, BOARD_INITPINS_USER_SW1_GPIO_PIN);
	return(butpress > 0 ? false : true);
}

bool Is_KW41Z_Button_Pressed(void)
{
	uint32_t butpress = GPIO_ReadPinInput(BOARD_INITPINS_USER_SW2_GPIO, BOARD_INITPINS_USER_SW2_GPIO_PIN);
	return(butpress > 0 ? false : true);
}

bool Is_MSD_Button_Pressed(void)
{
	uint32_t butpress = GPIO_ReadPinInput(BOARD_INITPINS_USER_SW3_GPIO, BOARD_INITPINS_USER_SW3_GPIO_PIN);
	return(butpress > 0 ? false : true);
}

bool Is_Factory_User_Button_Pressed(void)
{
	uint32_t butpress = GPIO_ReadPinInput(BOARD_INITPINS_USER_SW4_GPIO, BOARD_INITPINS_USER_SW4_GPIO_PIN);
	return(butpress > 0 ? false : true);
}

#endif // USE_SWITCH_IRQ

#endif // CPU_MK64FN1M0VMD12


//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
