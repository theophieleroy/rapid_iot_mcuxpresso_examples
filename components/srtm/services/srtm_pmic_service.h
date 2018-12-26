/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
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

#ifndef __SRTM_PMIC_SERVICE_H__
#define __SRTM_PMIC_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable PMIC service debugging messages. */
#ifndef SRTM_PMIC_SERVICE_DEBUG_OFF
#define SRTM_PMIC_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_PMIC_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/**
* @brief SRTM PMIC adapter structure pointer.
*/
typedef struct _srtm_pmic_adapter *srtm_pmic_adapter_t;

/**
* @brief SRTM PMIC adapter structure
*/
struct _srtm_pmic_adapter
{
    srtm_status_t (*enable)(srtm_pmic_adapter_t adapter, uint8_t regulator);
    srtm_status_t (*disable)(srtm_pmic_adapter_t adapter, uint8_t regulator);
    bool (*isEnabled)(srtm_pmic_adapter_t adapter, uint8_t regulator);
    srtm_status_t (*setVoltage)(srtm_pmic_adapter_t adapter, uint8_t regulator, uint32_t volt);
    srtm_status_t (*getVoltage)(srtm_pmic_adapter_t adapter, uint8_t regulator, uint32_t *pVolt);
    srtm_status_t (*setRegister)(srtm_pmic_adapter_t adapter, uint8_t register, uint32_t value);
    srtm_status_t (*getRegister)(srtm_pmic_adapter_t adapter, uint8_t register, uint32_t *pValue);
    srtm_status_t (*setStandbyVoltage)(srtm_pmic_adapter_t adapter, uint8_t regulator, uint32_t volt);
};

/**
* @brief SRTM PMIC payload structure
*/
SRTM_PACKED_BEGIN struct _srtm_pmic_payload
{
    uint8_t reg;
    uint8_t retCode;
    uint8_t status;
    uint32_t value;
} SRTM_PACKED_END;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create PMIC service.
 *
 * @param adapter PMIC adapter to handle real pmic operations.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_PmicService_Create(srtm_pmic_adapter_t adapter);

/*!
 * @brief Destroy PMIC service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_PmicService_Destroy(srtm_service_t service);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_PMIC_SERVICE_H__ */
