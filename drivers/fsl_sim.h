/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#ifndef _FSL_SIM_H_
#define _FSL_SIM_H_

#include "fsl_common.h"

/*! @addtogroup sim */
/*! @{*/

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief PMC driver version */
#define FSL_SIM_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
/*@}*/

/*!@brief System reset status definitions. */
typedef enum _sim_reset_source
{
    kSIM_SourceSackerr = SIM_SRSID_SACKERR_MASK, /*!< Stop mode acknowledge error reset */
    kSIM_SourceMdmap = SIM_SRSID_MDMAP_MASK,     /*!< MDM-AP system Reset request */
    kSIM_SourceSw = SIM_SRSID_SW_MASK,           /*!< Software reset */
    kSIM_SourceLockup = SIM_SRSID_LOCKUP_MASK,   /*!< Core lockup reset */
    kSIM_SourcePor = SIM_SRSID_POR_MASK,         /*!< Power on reset */
    kSIM_SourcePin = SIM_SRSID_PIN_MASK,         /*!< External pin reset */
    kSIM_SourceWdog = SIM_SRSID_WDOG_MASK,       /*!< Wdog reset */
    kSIM_SourceLoc = SIM_SRSID_LOC_MASK,         /*!< Internal clock source Module reset */
    kSIM_SourceLvd = SIM_SRSID_LVD_MASK,         /*!< Low voltage detect reset */
    kSIM_SourceAll = 0xFFFFU,
} sim_reset_source_t;

/*!@brief Unique ID. */
typedef struct _sim_uid
{
#if (defined(FSL_FEATURE_SIM_HAS_UIDH) && FSL_FEATURE_SIM_HAS_UIDH)
    uint32_t H; /*!< UUIDH.  */
#endif          /* FSL_FEATURE_SIM_HAS_UIDH */

#if (defined(FSL_FEATURE_SIM_HAS_UIDM) && FSL_FEATURE_SIM_HAS_UIDM)
    uint32_t MH; /*!< UUIDMH. */
    uint32_t ML; /*!< UUIDML. */
#endif           /* FSL_FEATURE_SIM_HAS_UIDM */
    uint32_t L;  /*!< UUIDL.  */
} sim_uid_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief Gets the unique identification register value.
 *
 * @param uid Pointer to the structure to save the UID value.
 */
void SIM_GetUniqueId(sim_uid_t *uid);

/*!
 * @brief Gets the reset source.
 * This function gets the SIM reset source.
 * Use source masks defined in the sim_reset_source_t to get the desired source status.
 *
 * This is an example.
   @code
   uint32_t resetStatus;

// To get all reset source statuses.
   resetStatus = SIM_GetSysResetStatus() & kSIM_SourceAll;

   // To test whether the MCU is reset using Watchdog.
   resetStatus = SIM_GetSysResetStatus() & kSIM_SourceWdog;

   // To test multiple reset sources.
   resetStatus = SIM_GetSysResetStatus() & (kSIM_SourceWdog | kSIM_SourcePin);
   @endcode

 * @return system reset status.
 */
static inline uint32_t SIM_GetSysResetStatus(void)
{
    return SIM->SRSID & 0xFFFFU;
}

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* _FSL_SIM_H_ */
