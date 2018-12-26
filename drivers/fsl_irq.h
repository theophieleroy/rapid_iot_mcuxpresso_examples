/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#ifndef _FSL_IRQ_H_
#define _FSL_IRQ_H_

#include "fsl_common.h"

/*!
 * @addtogroup irq
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_IRQ_DRIVER_VERSION (MAKE_VERSION(2, 0, 1)) /*!< Version 2.0.1. */
/*@}*/

/*! @brief Interrupt Request (IRQ) Edge Select */
typedef enum _irq_edge
{
    kIRQ_FallingEdgeorLowlevel = 0U, /*!< IRQ is falling-edge or falling-edge/low-level sensitive */
    kIRQ_RisingEdgeorHighlevel = 1U  /*!< IRQ is rising-edge or rising-edge/high-level sensitive */
} irq_edge_t;

/*! @brief Interrupt Request (IRQ) Detection Mode */
typedef enum _irq_mode
{
    kIRQ_DetectOnEdgesOnly = 0U,    /*!< IRQ event is detected only on falling/rising edges */
    kIRQ_DetectOnEdgesAndEdges = 1U /*!< IRQ event is detected on falling/rising edges and low/high levels */
} irq_mode_t;

/*!
 * @brief The IRQ pin configuration structure.
 *
 */
typedef struct _irq_config
{
    bool enablePullDevice; /*!< Enable/disable the internal pullup device when the IRQ pin is enabled */
    irq_edge_t edgeSelect; /*!< Select the polarity of edges or levels on the IRQ pin that cause IRQF to be set */
    irq_mode_t detectMode; /*!< select either edge-only detection or edge-and-level detection */
} irq_config_t;

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup irq
 * @{
 */

/*! @name IRQ Configuration */
/*@{*/

/*!
 * @brief Initializes the IRQ pin used by the board.
 *
 * To initialize the IRQ pin, define a irq configuration, specify whhether enable pull-up, the edge and detect mode.
 * Then, call the IRQ_Init() function.
 *
 * This is an example to initialize irq configuration.
 * @code
 * // define IRQ pin input with internal pull-up, falling edge detect
 * irq_config_t config =
 * {
 *   true,
 *   kIRQ_FallingEdgeorLowlevel,
 *   kIRQ_DetectOnEdgesOnly
 * }
 * @endcode
 *
 * @param base   IRQ peripheral base pointer
 * @param config IRQ configuration pointer
 */
void IRQ_Init(IRQ_Type *base, const irq_config_t *config);

/*!
 * @brief   Deinitialize IRQ peripheral.
 *
 * This function disables the IRQ clock.
 *
 * @param base IRQ peripheral base pointer.
 *
 * @retval None.
 */
void IRQ_Deinit(IRQ_Type *base);

/*!
 * @brief   Enable/disable IRQ pin.
 *
 * @param base IRQ peripheral base pointer.
 * @param enable true to enable IRQ pin, else disable IRQ pin.
 * @retval None.
 */
static inline void IRQ_Enable(IRQ_Type *base, bool enable)
{
    if (enable)
    {
        base->SC |= IRQ_SC_IRQPE_MASK;
    }
    else
    {
        base->SC &= ~IRQ_SC_IRQPE_MASK;
    }
}

/*@}*/

/*! @name IRQ interrupt Operations */
/*@{*/

/*!
 * @brief   Enable/disable IRQ pin interrupt.
 *
 * @param base IRQ peripheral base pointer.
 * @param enable true to enable IRQF assert interrupt request, else disable.
 * @retval None.
 */
static inline void IRQ_EnableInterrupt(IRQ_Type *base, bool enable)
{
    {
        if (enable)
        {
            base->SC |= IRQ_SC_IRQIE_MASK;
        }
        else
        {
            base->SC &= ~IRQ_SC_IRQIE_MASK;
        }
    }
}

/*!
 * @brief   Clear IRQF flag.

 * This function clears the IRQF flag.
 *
 * @param base IRQ peripheral base pointer.
 *
 * @retval None.
 */
static inline void IRQ_ClearIRQFlag(IRQ_Type *base)
{
    base->SC |= IRQ_SC_IRQACK_MASK;
}

/*!
 * @brief   Get IRQF flag.

 * This function returns the IRQF flag.
 *
 * @param base IRQ peripheral base pointer.
 *
 * @retval status = 0 IRQF flag deasserted.  = 1  IRQF flag asserted.
 */
static inline uint32_t IRQ_GetIRQFlag(IRQ_Type *base)
{
    return (base->SC & IRQ_SC_IRQF_MASK);
}

/*@}*/
#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_IRQ_H_ */
