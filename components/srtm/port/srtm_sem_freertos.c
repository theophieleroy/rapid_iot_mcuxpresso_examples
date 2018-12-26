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

#include <FreeRTOS.h>
#include <semphr.h>

#include "srtm_sem.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
srtm_sem_t SRTM_Sem_Create(uint32_t maxCount, uint32_t initCount)
{
    return xSemaphoreCreateCounting(maxCount, initCount);
}

void SRTM_Sem_Destroy(srtm_sem_t sem)
{
    assert(sem);

    vSemaphoreDelete(sem);
}

srtm_status_t SRTM_Sem_Post(srtm_sem_t sem)
{
    portBASE_TYPE taskToWake = pdFALSE;

    if (__get_IPSR())
    {
        if (xSemaphoreGiveFromISR(sem, &taskToWake) == pdPASS)
        {
            portYIELD_FROM_ISR(taskToWake);
            return SRTM_Status_Success;
        }
    }
    else
    {
        if (xSemaphoreGive(sem) == pdTRUE)
        {
            return SRTM_Status_Success;
        }
    }

    return SRTM_Status_Error;
}

srtm_status_t SRTM_Sem_Wait(srtm_sem_t sem, uint32_t timeout)
{
    uint32_t ticks;

    if (timeout == SRTM_WAIT_FOR_EVER)
    {
        ticks = portMAX_DELAY;
    }
    else if (timeout == SRTM_NO_WAIT)
    {
        ticks = 0U;
    }
    else
    {
        ticks = ((uint32_t)configTICK_RATE_HZ * (uint32_t)(timeout - 1U)) / 1000U + 1U;
    }

    if (xSemaphoreTake(sem, ticks) == pdFALSE)
    {
        return SRTM_Status_Timeout;
    }

    return SRTM_Status_Success;
}
