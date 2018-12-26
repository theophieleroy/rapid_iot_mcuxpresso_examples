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
 * @file backlight.c
 * This is the source file for the LCD backlight driver.
 */

#include "backlight.h"
#include <assert.h>

/*****************************************************************************
 * Variables
 ****************************************************************************/
static backlight_fct_t fctBacklight;
static bool initDriverDone = false;
static bool initHwDone = false;
static backlight_level_t backlightLevel_current;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void Backlight_Init_Driver(backlight_fct_t *fct)
{
    assert((fct != NULL) &&
            (fct->connect_hw != NULL) &&
            (fct->disconnect_hw != NULL) &&
            (fct->set_level != NULL));
    fctBacklight = *fct;
    initDriverDone = true;
}

void Backlight_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        Backlight_SetLevel(BLIGHT_LEVEL_OFF); // skip error management
        fctBacklight.disconnect_hw();
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

backlight_status_t Backlight_Init_Hw()
{
    backlight_status_t status;
    if (!initDriverDone) return backlight_status_noinit;

    fctBacklight.connect_hw();

    /* trick to allow calling internal public functions */
    initHwDone = true;

    status = Backlight_SetLevel(BLIGHT_LEVEL_OFF);
    if (status != backlight_status_success) initHwDone = false;

    return status;
}

backlight_status_t Backlight_SetLevel(backlight_level_t level)
{
    uint8_t pwm_duty_cycle;
    if (!initHwDone) return backlight_status_noinit;

    if (level == BLIGHT_LEVEL_OFF)
        pwm_duty_cycle = 0;
    else if (level == BLIGHT_LEVEL_LOW)
        pwm_duty_cycle = 25;
    else if (level == BLIGHT_LEVEL_MEDIUM)
        pwm_duty_cycle = 50;
    else if (level == BLIGHT_LEVEL_HIGH)
        pwm_duty_cycle = 100;
    else return backlight_status_error;

    fctBacklight.set_level(pwm_duty_cycle);
    backlightLevel_current = level;

    return backlight_status_success;
}

backlight_status_t Backlight_GetLevel(backlight_level_t *level)
{
    if (!initHwDone) return backlight_status_noinit;
    assert(level != NULL);

    *level = backlightLevel_current;

    return backlight_status_success;
}
