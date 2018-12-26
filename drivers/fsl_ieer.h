/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2017, NXP
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

#ifndef _FSL_IEER_H_
#define _FSL_IEER_H_

#include "fsl_common.h"

/*!
 * @addtogroup ieer
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief IEER driver version. Version 2.0.0.
 *
 * Current version: 2.0.0
 *
 * Change log:
 * - Version 2.0.0
 *   - Initial version
 */
#define FSL_IEER_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

typedef enum _ieer_region
{
    kIEER_Region0 = 0U, /*!< IEER region 0 */
    kIEER_Region1 = 1U, /*!< IEER region 1 */
    kIEER_Region2 = 2U, /*!< IEER region 2 */
    kIEER_Region3 = 3U  /*!< IEER region 3 */
} ieer_region_t;

/*! @brief IEER configuration structure. */
typedef struct _ieer_config
{
    uint32_t startaddr; /*!< IEER Memory Region W0 Descriptor */
    uint32_t endaddr;   /*!< IEER Memory Region W2 Descriptor */
    uint32_t rmsg;      /*!< IEER Memory Region W4 Descriptor RMSG field */
    bool valid;         /*!< IEER Memory Region W4 Descriptor VLD bit */
} ieer_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Clears the IEER Memory Region Descriptors.
 *
 * This function clears IEER Memory Region Descriptors to reset values.
 *
 * @param base IEER peripheral address.
 */
void IEER_Init(IEER_Type *base);

/*!
 * @brief Sets the IEER Memory Region Descriptors.
 *
 * This function configures IEER Memory Region Descriptor according to region configuration structure.
 *
 * @param base IEER peripheral address.
 * @param region Selection of the IEER region to be configured.
 * @param config Configuration for the selected IEER region.
 */
void IEER_SetRegionConfig(IEER_Type *base, ieer_region_t region, ieer_config_t *config);

/*!
 * @brief Loads default values to the IEER memory region configuration structure.
 *
 * Loads default values to the IEER memory region configuration structure. The default values are as follows.
 * @code
 *   config->startaddr = 0;
 *   config->endaddr = 0x00000FFFU;
 *   config->rmsg = 0;
 *   config->valid = false;
 * @endcode
 *
 * @param config Configuration for the selected IEER region.
 */
void IEER_GetDefaultConfig(ieer_config_t *config);

#if defined(__cplusplus)
}
#endif

/*!
 *@}
 */

#endif /* _FSL_IEER_H_ */
