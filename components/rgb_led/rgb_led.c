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
 * @file rgb_led.c
 * This is the source file for the RGB LED driver.
 */

#include "rgb_led.h"
#include <stdbool.h>
#include <assert.h>

#define BRIGHTNESS_DEFAULT RGB_LED_BRIGHT_OFF
#define COLOR_DEFAULT      RGB_LED_COLOR_WHITE
#define LED_RGB_BLACK            0, 0, 0         // 0% duty cycle <=> black

/*****************************************************************************
 * Variables
 ****************************************************************************/
static rgbled_fct_t fctRgbLed;
static bool initDriverDone = false;
static bool initHwDone = false;
static uint8_t currentBrightness;
static uint8_t currentColor;

/*****************************************************************************
 * Static functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void RGB_Led_Init_Driver(rgbled_fct_t *fct)
{
    assert((fct != NULL) &&
            (fct->connect_hw != NULL) &&
            (fct->disconnect_hw != NULL) &&
            (fct->set_rgb_colors != NULL));
    fctRgbLed = *fct;
    initDriverDone = true;
}

void RGB_Led_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        fctRgbLed.set_rgb_colors(LED_RGB_BLACK); /* off */
        fctRgbLed.disconnect_hw();
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

RgbLedErrorCode_t RGB_Led_Init_Hw()
{
    if (!initDriverDone) {
        return kStatus_Fail;
    }
    currentBrightness = BRIGHTNESS_DEFAULT;
    currentColor = COLOR_DEFAULT;

    fctRgbLed.connect_hw();
    fctRgbLed.set_rgb_colors(LED_RGB_BLACK); /* off */

    initHwDone = true;
    return kStatus_Success;
}

RgbLedErrorCode_t RGB_Led_Get_State(rgb_led_brightness_t *brightness, rgb_led_color_t *color)
{
    if (!initHwDone) return kStatus_Fail;
    assert((brightness != NULL) && (color != NULL));

    *brightness = currentBrightness;
    *color = currentColor;

    return kStatus_Success;
}

RgbLedErrorCode_t RGB_Led_Set_State(rgb_led_brightness_t brightness, rgb_led_color_t color)
{
    /* PWM values within the range [0-100] */
    uint8_t pwmRed = 0;
    uint8_t pwmGreen = 0;
    uint8_t pwmBlue = 0;

    if (!initHwDone)
    {
        return kStatus_Fail;
    }

    if (color == RGB_LED_COLOR_RED)
    {
        pwmRed = 100;
    }
    else if (color == RGB_LED_COLOR_GREEN)
    {
        pwmGreen = 100;
    }
    else if (color == RGB_LED_COLOR_BLUE)
    {
        pwmBlue = 100;
    }
    else if (color == RGB_LED_COLOR_WHITE)
    {
        /* actual white is not 100/100/100 */
        pwmRed = 40;
        pwmGreen = 100;
        pwmBlue = 80;
    }
    else if (color == RGB_LED_COLOR_YELLOW)
    {
        pwmRed = 100;
        pwmGreen = 100;
        pwmBlue = 0;
    }
    else if (color == RGB_LED_COLOR_CYAN)
    {
        pwmRed = 0;
        pwmGreen = 100;
        pwmBlue = 100;
    }
    else if (color == RGB_LED_COLOR_PURPLE)
    {
        pwmRed = 100;
        pwmGreen = 0;
        pwmBlue = 100;
    }
    else if (color == RGB_LED_COLOR_BLACK)
    {
        /* nothing */
    }
    else
    {
        /* error */
        return kStatus_Fail;
    }

    /* brightness = 0 (Off), 1 (Low), 2 (Medium) or 3 (High) */
    /* tune the PWM values based on brightness */
    if (brightness == RGB_LED_BRIGHT_OFF)
    {
        /* off */
        pwmRed = 0;
        pwmGreen = 0;
        pwmBlue = 0;
    }
    else if (brightness == RGB_LED_BRIGHT_LOW)
    {
        /* low */
        pwmRed /= 3;
        pwmGreen /= 3;
        pwmBlue /= 3;
    }
    else if (brightness == RGB_LED_BRIGHT_MEDIUM)
    {
        /* medium */
        pwmRed = (pwmRed*2)/3;
        pwmGreen = (pwmGreen*2)/3;
        pwmBlue = (pwmBlue*2)/3;
    }
    else if (brightness == RGB_LED_BRIGHT_HIGH)
    {
        /* high */
    }
    else
    {
        /* error */
        return kStatus_Fail;
    }

    fctRgbLed.set_rgb_colors(pwmRed, pwmGreen, pwmBlue);
    currentBrightness = brightness;
    currentColor = color;
    return kStatus_Success;
}

