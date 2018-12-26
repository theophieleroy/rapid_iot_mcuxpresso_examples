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
 * @file backlight.h
 * This is the header file for the LCD backlight driver.
 */

#ifndef BACKLIGHT_H_
#define BACKLIGHT_H_

/*!
 * @addtogroup backlight   LCD Backlight
 * This module provides the API to control the LCD backlight level through callback functions.
 *
 * The basic steps to operate the LCD backlight are as follows:
 * -# Initialize the driver with callback functions (#Backlight_Init_Driver)
 * -# Initialize the hardware (#Backlight_Init_Hw)
 * -# Set the backlight level (#Backlight_SetLevel)
 * -# If the backlight is not needed anymore, de-initialize the driver (#Backlight_Deinit_Driver). The backlight will
 * be switched off. It allows to eventually release shared resources.
 *
 * Example - Sample application code to set LCD backlight level without error management
 * -------------------------------------------------------------------------------------
 * @code
 * #include "backlight.h"
 *
 * backlight_fct_t backlight_fct;
 * backlight_level_t level;
 *
 * backlight_fct.connect_hw = Backlight_Connect;       // callback function to activate backlight hardware resource
 * backlight_fct.disconnect_hw = Backlight_Disconnect; // callback function to deactivate backlight hardware resource
 * backlight_fct.set_level = Backlight_Set_Level;      // callback function to set backlight level
 *
 * Backlight_Init_Driver(&backlight_fct);
 * Backlight_Init_Hw();
 * printf("Set backlight level to medium\n");
 * Backlight_SetLevel(BLIGHT_LEVEL_MEDIUM);
 * [..]
 * Backlight_GetLevel(&level);
 * printf("The current backlight level is: %d\n", level);
 * [..]
 * // if the backlight is not needed anymore, the driver can be de-initialized
 * printf("Deinitialize backlight (it will be turned off)\n");
 * Backlight_Deinit_Driver();
 *
 * @endcode
 * @{
 */

#include <EmbeddedTypes.h>
#include <stdbool.h>

/*! @brief Status return codes. */
typedef enum _backlight_status
{
    backlight_status_success, /*!< Success */
    backlight_status_noinit,  /*!< Hardware or driver not initialized */
    backlight_status_error    /*!< Internal error */
} backlight_status_t;

/*! @brief Predefined backlight levels. */
typedef enum _backlight_level
{
    BLIGHT_LEVEL_OFF    = 0, /*!< Backlight Off */
    BLIGHT_LEVEL_LOW    = 1, /*!< Lowest predefined Backlight level */
    BLIGHT_LEVEL_MEDIUM = 2, /*!< Medium predefined Backlight level */
    BLIGHT_LEVEL_HIGH   = 3  /*!< Highest predefined Backlight level */
} backlight_level_t;

/*! @brief Structure of external functions or values. */
typedef struct _backlight_fct_t
{
    void (*connect_hw)(void);         /*!< External function to activate the backlight hardware */
    void (*disconnect_hw)(void);      /*!< External function to deactivate the backlight hardware */
    void (*set_level)(uint8_t level); /*!< External function to set the backlight level */
} backlight_fct_t;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * @brief Initialize the LCD backlight driver
 *
 * @param fct Pointer to a structure of external functions or values
 */
void Backlight_Init_Driver(backlight_fct_t *fct);

/*!
 * @brief De-initialize the LCD backlight driver
 *
 */
void Backlight_Deinit_Driver();

/*!
 * @brief Initialize the LCD backlight hardware
 *
 * @return Status value (0 for success)
 */
backlight_status_t Backlight_Init_Hw();

/*!
 * @brief Set current backlight level
 *
 * @param level  Backlight level (Off=0, Low=1, Medium=2, High=3)
 * @return Status value (0 for success)
 */
backlight_status_t Backlight_SetLevel(backlight_level_t level);

/*!
 * @brief Get current backlight level
 *
 * @param level  Pointer to backlight level (Off=0, Low=1, Medium=2, High=3)
 * @return Status value (0 for success)
 */
backlight_status_t Backlight_GetLevel(backlight_level_t *level);

/*! @}*/

#endif /* BACKLIGHT_H_ */
