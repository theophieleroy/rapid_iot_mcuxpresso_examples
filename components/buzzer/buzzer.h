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
 * @file buzzer.h
 * This is the header file for the buzzer driver.
 */

#ifndef BUZZER_H_
#define BUZZER_H_

/*!
 * @addtogroup buzzer Buzzer
 * This module provides the API to control the buzzer.
 *
 * The basic steps to operate the buzzer are as follows:
 * -# Initialize the driver (#Buzzer_Init_Driver)
 * -# Initialize the hardware (#Buzzer_Init_Hw)
 * -# Turn on the buzzer (#Buzzer_On) or off (#Buzzer_Off)
 * -# If the buzzer is not needed anymore, de-initialize the driver (#Buzzer_Deinit_Driver).
 *
 * Example - Sample application code to turn on or off the buzzer without error management
 * ---------------------------------------------------------------------------------------
 * @code
 * #include "buzzer.h"
 *
 * Buzzer_Init_Driver();
 * Buzzer_Init_Hw();
 * printf("Turn on the buzzer\n");
 * Buzzer_On();
 * [..]
 * printf("Turn off the buzzer\n");
 * Buzzer_Off();
 * // if the buzzer is not needed anymore, the driver can be de-initialized
 * printf("De-initialize buzzer (it will be turned off)\n");
 * Buzzer_Deinit_Driver();
 *
 * @endcode
 * @{
 */

#include <EmbeddedTypes.h>
#include <stdbool.h>

/*! @brief Status return codes. */
typedef enum buzzer_status_
{
    buzzer_status_success, /*!< Success */
    buzzer_status_noinit,  /*!< Hardware or driver not initialized */
    buzzer_status_error    /*!< Internal error */
} buzzer_status_t;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * @brief Initialize the buzzer driver
 *
 */
void Buzzer_Init_Driver();

/*!
 * @brief De-initialize the buzzer driver
 *
 */
void Buzzer_Deinit_Driver();

/*!
 * @brief Initialize the buzzer hardware
 *
 * @return Status value (0 for success)
 */
buzzer_status_t Buzzer_Init_Hw();

/*!
 * @brief Put the buzzer on
 *
 * @return Status value (0 for success)
 */
buzzer_status_t Buzzer_On(void);

/*!
 * @brief Put the buzzer off
 *
 * @return Status value (0 for success)
 */
buzzer_status_t Buzzer_Off(void);

/*! @}*/

#endif /* BUZZER_H_ */
