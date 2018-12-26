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

#ifndef __SRTM_SEM_H__
#define __SRTM_SEM_H__

#include <srtm_defs.h>

/*!
 * @addtogroup srtm
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM semaphore handle
*/
typedef void *srtm_sem_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create a semaphore.
 *
 * @param maxCount maximum count value of the semaphore.
 * @param initCount initial count value of the semaphore.
 * @return Created semaphore handle, or NULL on failure.
 */
srtm_sem_t SRTM_Sem_Create(uint32_t maxCount, uint32_t initCount);

/*!
 * @brief Destroy the semaphore.
 *
 * @param sem The semaphore to destroy.
 */
void SRTM_Sem_Destroy(srtm_sem_t sem);

/*!
 * @brief Posts a semaphore.
 *
 * @param sem Semaphore handle
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Sem_Post(srtm_sem_t sem);

/*!
 * @brief Waits on a semaphore.
 *
 * @param sem Semaphore handle
 * @param timeout   The maximum milliseconds to wait for the semaphore.
 * @return SRTM_Status_Success on success, SRTM_Status_Timeout on timeout.
 */
srtm_status_t SRTM_Sem_Wait(srtm_sem_t sem, uint32_t timeout);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_SEM_H__ */
