/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "fsl_spi_ucosiii.h"
#include <os.h>
#include <lib_mem.h>

static void SPI_RTOS_Callback(SPI_Type *base, spi_master_handle_t *drv_handle, status_t status, void *userData)
{
    OS_ERR err;
    spi_rtos_handle_t *handle = (spi_rtos_handle_t *)userData;
    OSFlagPost(&handle->event, RTOS_SPI_COMPLETE, OS_OPT_POST_FLAG_SET, &err);
}

status_t SPI_RTOS_Init(spi_rtos_handle_t *handle,
                       SPI_Type *base,
                       const spi_master_config_t *masterConfig,
                       uint32_t srcClock_Hz)
{
    OS_ERR err;

    if (handle == NULL)
    {
        return kStatus_InvalidArgument;
    }

    if (base == NULL)
    {
        return kStatus_InvalidArgument;
    }

    memset(handle, 0, sizeof(spi_rtos_handle_t));

    OSSemCreate(&handle->mutex, "SPI", (OS_SEM_CTR)1, &err);
    if (OS_ERR_NONE != err)
    {
        return kStatus_Fail;
    }

    OSFlagCreate(&handle->event, "SPI", (OS_FLAGS)0, &err);
    if (OS_ERR_NONE != err)
    {
        OSSemDel(&handle->mutex, OS_OPT_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }

    handle->base = base;

    SPI_MasterInit(handle->base, masterConfig, srcClock_Hz);
    SPI_MasterTransferCreateHandle(handle->base, &handle->drv_handle, SPI_RTOS_Callback, (void *)handle);

    return kStatus_Success;
}

status_t SPI_RTOS_Deinit(spi_rtos_handle_t *handle)
{
    OS_ERR err;
    SPI_Deinit(handle->base);
    OSFlagDel(&handle->event, OS_OPT_DEL_ALWAYS, &err);
    OSSemDel(&handle->mutex, OS_OPT_DEL_ALWAYS, &err);

    return kStatus_Success;
}

status_t SPI_RTOS_Transfer(spi_rtos_handle_t *handle, spi_transfer_t *transfer)
{
    status_t status;
    OS_FLAGS ev;
    OS_ERR err;

    /* Lock resource mutex */
    OSSemPend(&handle->mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (OS_ERR_NONE != err)
    {
        return kStatus_Fail;
    }

    ev = OSFlagPost(&handle->event, RTOS_SPI_COMPLETE, OS_OPT_POST_FLAG_CLR, &err);
    assert((ev & RTOS_SPI_COMPLETE) == 0);

    status = SPI_MasterTransferNonBlocking(handle->base, &handle->drv_handle, transfer);
    if (status != kStatus_Success)
    {
        OSSemPost(&handle->mutex, OS_OPT_POST_1, &err);
        return status;
    }

    /* Wait for transfer to finish */
    ev = OSFlagPend(&handle->event, RTOS_SPI_COMPLETE, 0, OS_OPT_PEND_FLAG_SET_ALL, NULL, &err);
    if (!(ev & RTOS_SPI_COMPLETE))
    {
        OSSemPost(&handle->mutex, OS_OPT_POST_1, &err);
        return kStatus_Fail;
    }

    /* Unlock resource mutex */
    OSSemPost(&handle->mutex, OS_OPT_POST_1, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not post back the semaphore, exit with error */
        return kStatus_Fail;
    }

    /* Return status captured by callback function */
    return handle->async_status;
}
