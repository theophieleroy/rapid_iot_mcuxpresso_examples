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
#ifndef _FSL_KBI_H_
#define _FSL_KBI_H_

#include "fsl_common.h"

/*!
 * @addtogroup kbi
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief KBI driver version 2.0.0. */
#define FSL_KBI_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief KBI detection mode. */
typedef enum _kbi_detect_mode
{
    kKBI_EdgesDetect = 0, /*!< The keyboard detects edges only. */
    kKBI_EdgesLevelDetect /*!< The keyboard detects both edges and levels. */
} kbi_detect_mode_t;

/*! @brief KBI configuration. */
typedef struct _kbi_config
{
    uint32_t pinsEnabled;   /*!< The eight kbi pins, set 1 to enable the corresponding KBI interrupt pins. */
    uint32_t pinsEdge;      /*!< The edge selection for each kbi pin: 1 -- rinsing edge, 0 -- falling edge. */
    kbi_detect_mode_t mode; /*!< The kbi detection mode. */
} kbi_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
  * @name Initialization and De-initialization
  * @{
  */

/*!
 * @brief KBI initialize.
 * This function ungates the KBI clock and initializes KBI.
 * This function must be called before calling any other KBI driver functions.
 *
 * @param base KBI peripheral base address.
 * @param configure The KBI configuration structure pointer.
 */
void KBI_Init(KBI_Type *base, kbi_config_t *configure);

/*!
 * @brief Deinitializes the KBI module and gates the clock.
 * This function gates the KBI clock. As a result, the KBI
 * module doesn't work after calling this function.
 *
 * @param base KBI peripheral base address.
 */
void KBI_Deinit(KBI_Type *base);

/* @} */

/*!
 * @name KBI Basic Operation
 * @{
 */

/*!
 * @brief Enables the interrupt.
 *
 * @param base KBI peripheral base address.
 */
static inline void KBI_EnableInterrupts(KBI_Type *base)
{
    base->SC |= KBI_SC_KBIE_MASK;
}

/*!
 * @brief Disables the interrupt.
 *
 * @param base KBI peripheral base address.
 */
static inline void KBI_DisableInterrupts(KBI_Type *base)
{
    base->SC &= ~KBI_SC_KBIE_MASK;
}

/*!
 * @brief Gets the KBI interrupt event status.
 *
 * @param base KBI peripheral base address.
 * @return The status of the KBI interrupt request is detected.
 */
static inline bool KBI_IsInterruptRequestDetected(KBI_Type *base)
{
    return ((base->SC & KBI_SC_KBF_MASK) ? true : false);
}

/*!
 * @brief Clears KBI status flag.
 *
 * @param base KBI peripheral base address.
 */
static inline void KBI_ClearInterruptFlag(KBI_Type *base)
{
    base->SC |= KBI_SC_KBACK_MASK;
}

#if defined(FSL_FEATURE_KBI_HAS_SOURCE_PIN) && FSL_FEATURE_KBI_HAS_SOURCE_PIN
/*!
 * @brief Gets the KBI Source pin status.
 *
 * @param base KBI peripheral base address.
 * @return The status indicates the active pin defined as keyboard interrupt
 * which is pushed.
 */
static inline uint32_t KBI_GetSourcePinStatus(KBI_Type *base)
{
    return base->SP;
}
#endif /* FSL_FEATURE_KBI_HAS_SOURCE_PIN */

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_KBI_H_*/
