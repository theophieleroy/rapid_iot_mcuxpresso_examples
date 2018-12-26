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
 * @file rgb_led.h
 * This is the header file for the RGB LED driver.
 */

#ifndef RGB_LED_H_
#define RGB_LED_H_

/*!
 * @addtogroup rgb_led RGB LED
 * This module provides the API to operate the RGB LED.
 *
 * The basic steps to operate the RGB LED are as follows:
 * -# Initialize the driver with callback functions (#RGB_Led_Init_Driver)
 * -# Initialize the hardware (#RGB_Led_Init_Hw)
 * -# Set the new RGB LED brightness and color (#RGB_Led_Set_State)
 * -# If the RGB LED is not needed anymore, de-initialize the driver (#RGB_Led_Deinit_Driver). The RGB LED will
 * be switched off. It allows to eventually release shared resources.
 *
 * Example - Sample application code to set RGB LED without error management
 * -------------------------------------------------------------------------
 * @code
 * #include "rgb_led.h"
 *
 * rgbled_fct_t rgbled_fct;
 *
 * rgbled_fct.connect_hw = Rgb_Led_Connect;        // callback function to activate RGB LED hardware resource
 * rgbled_fct.disconnect_hw = Rgb_Led_Disconnect;  // callback function to deactivate RGB LED hardware resource
 * rgbled_fct.set_rgb_colors = Rgb_Led_Set_Colors; // callback function to control separately the R/G/B colors
 *
 * RGB_Led_Init_Driver(&rgbled_fct);
 * RGB_Led_Init_Hw();
 * // set the new RGB LED brightness and color
 * RGB_Led_Set_State(RGB_LED_BRIGHT_MEDIUM, RGB_LED_COLOR_GREEN);
 * [..]
 * // if the RGB LED is not needed anymore, the driver can be de-initialized
 * printf("Deinitialize RGB LED (it will be turned off)\n");
 * RGB_Led_Deinit_Driver();
 *
 * @endcode
 * @{
 */

#include <EmbeddedTypes.h>
#include "fsl_common.h"

typedef enum _generic_status RgbLedErrorCode_t;

/*! @brief Structure of external functions or values. */
typedef struct _rgbled_fct_t
{
    void (*connect_hw)(void);    /*!< External function to activate the RGB LED hardware */
    void (*disconnect_hw)(void); /*!< External function to deactivate the RGB LED hardware */
    void (*set_rgb_colors)(uint8_t R, uint8_t G, uint8_t B); /*!< External function to set R/G/B colors */
} rgbled_fct_t;

/*! @brief LED Brightness */
typedef enum {
    RGB_LED_BRIGHT_OFF    = 0,      /*!< LED Off */
    RGB_LED_BRIGHT_LOW    = 1,      /*!< LED Low Brightness */
    RGB_LED_BRIGHT_MEDIUM = 2,      /*!< LED Medium Brightness */
    RGB_LED_BRIGHT_HIGH   = 3       /*!< LED High Brightness */
} rgb_led_brightness_t;

/*! @brief LED Color */
typedef enum {
    RGB_LED_COLOR_RED    = 0,       /*!< LED Color: Red */
    RGB_LED_COLOR_GREEN  = 1,       /*!< LED Color: Green */
    RGB_LED_COLOR_BLUE   = 2,       /*!< LED Color: Blue*/
    RGB_LED_COLOR_WHITE  = 3,       /*!< LED Color: White*/
    RGB_LED_COLOR_YELLOW = 4,       /*!< LED Color: Yellow */
    RGB_LED_COLOR_CYAN   = 5,       /*!< LED Color: Cyan*/
    RGB_LED_COLOR_PURPLE = 6,       /*!< LED Color: Purple*/
    RGB_LED_COLOR_BLACK  = 7        /*!< LED Color: Black /OFF */
} rgb_led_color_t;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * @brief Initialize RGB LED driver.
 *
 */
void RGB_Led_Init_Driver();

/*!
 * @brief De-initialize RGB LED driver.
 *
 */
void RGB_Led_Deinit_Driver();

/*!
 * @brief Initialize RGB LED hardware.
 * @return Status value (0 for success)
 */
RgbLedErrorCode_t RGB_Led_Init_Hw();

/*!
 * @brief Get current RGB LED brightness and color values.
 *
 * @note The values come from the internal sw settings, not from hw acquisition.
 * @param brightness Pointer to a preselected brightness value: 0 (Off), 1 (Low), 2 (Medium) or 3 (High)
 * @param color      Pointer to a preselected color value: 0 (Red), 1 (Green), 2 (Blue) or 3 (White)
 * @return           Status value (0 for success)
 */
RgbLedErrorCode_t RGB_Led_Get_State(uint8_t *brightness, uint8_t *color);

/*!
 * @brief Set RGB LED based on preselected brightness and color values.
 *
 * @param brightness Preselected brightness value: 0 (Off), 1 (Low), 2 (Medium) or 3 (High)
 * @param color      Preselected color value: 0 (Red), 1 (Green), 2 (Blue) or 3 (White)
 * @return           Status value (0 for success)
 */
RgbLedErrorCode_t RGB_Led_Set_State(uint8_t brightness, uint8_t color);

/*! @}*/

#endif /* RGB_LED_H_ */
