/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include <stdio.h>

#include "board.h"
#include "shell.h"

#include "clock_config.h"
#include "pin_mux_rpk.h"
#include "GUI.h"

#include "sensors.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{

    /* Init board hardware. */
    BOARD_Init_RPK();
    BOARD_InitDebugConsole();
    MEM_Init();

    Init_backlight();


    Display_Connect(); /* triggers GUI_Init() */
    Backlight_SetLevel(BLIGHT_LEVEL_HIGH);

    GUI_SetBkColor(GUI_BLACK);
    GUI_SetColor(GUI_DARKRED);
    GUI_SetFont(&GUI_Font8x18);

    /* Init I2C. */
    I2C1_init();
    I2C2_init();

    /* Init Accelerometer. */
    if (Init_accel_mag())
	{
		GUI_DispStringHCenterAt("Error while initializing Accel & Mag!", 88, 68);
	}

    char buff[50]               = {0};

    float AccelerometerValue[3] = {0};
	uint8_t AccelerometerSize   = 3 * sizeof(float);

	GUI_DispStringHCenterAt("Accelerometer", 88, 18);

	/* Once every 0.5 seconds, read and display data from accelerometer. */
    while (1)
    {
    	if (get_acceleration((uint8_t *)AccelerometerValue, &AccelerometerSize) == 0)
		{
    		sprintf(buff, " X: %+.3f\n\n Y: %+.3f\n\n Z: %+.3f", AccelerometerValue[0], AccelerometerValue[1], AccelerometerValue[2]);
			GUI_DispStringHCenterAt(buff, 88, 48);
		}

    	App_WaitMsec(500);
    }
}
