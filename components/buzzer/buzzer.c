/*
 * Copyright (c) 2018 NXP
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

/*!
 * @file buzzer.c
 * This is the source file for the buzzer driver.
 */

#include "buzzer.h"
#include "peripherals.h"
#include "board.h"

/*****************************************************************************
 * Variables
 ****************************************************************************/

static bool initDriverDone = false;
static bool initHwDone = false;

/*****************************************************************************
 * Static functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void Buzzer_Init_Driver()
{
    initDriverDone = true;
}

void Buzzer_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        BUZZER_OFF;
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

buzzer_status_t Buzzer_Init_Hw()
{
    if (!initDriverDone) return buzzer_status_noinit;

    initHwDone = true;
    return buzzer_status_success;
}

buzzer_status_t Buzzer_On(void)
{
    if (!initHwDone) return buzzer_status_noinit;

    BUZZER_ON;
    return buzzer_status_success;
}

buzzer_status_t Buzzer_Off(void)
{
    if (!initHwDone) return buzzer_status_noinit;

    BUZZER_OFF;
    return buzzer_status_success;
}
