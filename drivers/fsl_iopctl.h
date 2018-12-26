/*
 * The Clear BSD License
 * Copyright (c) 2013-2016, NXP Semiconductors.
 * All rights reserved.
 *
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
 * o Neither the name of copyright holder nor the names of its
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

#ifndef _FSL_IOPCTL_H_
#define _FSL_IOPCTL_H_

#include "fsl_common.h"

/*!
 * @addtogroup iopctl_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.lpc_iopctl"
#endif


/*! @name Driver version */
/*@{*/
/*! @brief IOPCTL driver version 2.0.0. */
#define LPC_IOPCTL_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/**
 * @brief Array of IOPCTL pin definitions passed to IOPCTL_SetPinMuxing() must be in this format
 */
typedef struct _iopctl_group
{
    uint32_t port : 8;      /* Pin port */
    uint32_t pin : 32;       /* Pin number */
    uint32_t modefunc : 12; /* Function and mode */
} iopctl_group_t;

/**
 * @brief IOPCTL function and mode selection definitions
 * @note See the User Manual for specific modes and functions supported by the various pins.
 */
#define IOPCTL_FUNC0 0x0                   /*!< Selects pin function 0 */
#define IOPCTL_FUNC1 0x1                   /*!< Selects pin function 1 */
#define IOPCTL_FUNC2 0x2                   /*!< Selects pin function 2 */
#define IOPCTL_FUNC3 0x3                   /*!< Selects pin function 3 */
#define IOPCTL_FUNC4 0x4                   /*!< Selects pin function 4 */
#define IOPCTL_FUNC5 0x5                   /*!< Selects pin function 5 */
#define IOPCTL_FUNC6 0x6                   /*!< Selects pin function 6 */
#define IOPCTL_FUNC7 0x7                   /*!< Selects pin function 7 */
#define IOPCTL_FUNC8 0x8                   /*!< Selects pin function 8 */
#define IOPCTL_FUNC9 0x9                   /*!< Selects pin function 9 */
#define IOPCTL_FUNC10 0xA                  /*!< Selects pin function 10 */
#define IOPCTL_FUNC11 0xB                  /*!< Selects pin function 11 */
#define IOPCTL_FUNC12 0xC                  /*!< Selects pin function 12 */
#define IOPCTL_FUNC13 0xD                  /*!< Selects pin function 13 */
#define IOPCTL_FUNC14 0xE                  /*!< Selects pin function 14 */
#define IOPCTL_FUNC15 0xF                  /*!< Selects pin function 15 */
#define IOPCTL_PUPD_EN (0x1 << 4)          /*!< Enables Pullup / Pulldown */
#define IOPCTL_PULLDOWN_EN (0x0 << 5)      /*!< Selects pull-down function */
#define IOPCTL_PULLUP_EN (0x1 << 5)        /*!< Selects pull-up function */
#define IOPCTL_INBUF_EN (0x1 << 6)         /*!< Enables buffer function  on input */
#define IOPCTL_SLEW_RATE (0x0 << 7)        /*!< Slew Rate Control */
#define IOPCTL_FULLDRIVE_EN (0x1 << 8)     /*!< Selects full drive */
#define IOPCTL_ANAMUX_EN (0x1 << 9)        /*!< Enables analog mux function by setting 0 to bit 7 */
#define IOPCTL_PSEDRAIN_EN (0x1 << 10)     /*!< Enables pseudo output drain function */
#define IOPCTL_INV_EN (0x1 << 11)          /*!< Enables invert function on input */

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * @brief   Sets I/O Pad Control pin mux
 * @param   base        : The base of IOPCTL peripheral on the chip
 * @param   port        : Port to mux
 * @param   pin         : Pin to mux
 * @param   modefunc    : OR'ed values of type IOPCTL_*
 * @return  Nothing
 */
__STATIC_INLINE void IOPCTL_PinMuxSet(IOPCTL_Type *base, uint8_t port, uint8_t pin, uint32_t modefunc)
{
    base->PIO[port][pin] = modefunc;
}

/**
 * @brief   Set all I/O Control pin muxing
 * @param   base        : The base of IOPCTL peripheral on the chip
 * @param   pinArray    : Pointer to array of pin mux selections
 * @param   arrayLength : Number of entries in pinArray
 * @return  Nothing
 */
__STATIC_INLINE void IOPCTL_SetPinMuxing(IOPCTL_Type *base, const iopctl_group_t *pinArray, uint32_t arrayLength)
{
    uint32_t i;

    for (i = 0; i < arrayLength; i++)
    {
        IOPCTL_PinMuxSet(base, pinArray[i].port, pinArray[i].pin, pinArray[i].modefunc);
    }
}

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_IOPCTL_H_ */
