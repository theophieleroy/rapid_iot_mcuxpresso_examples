/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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

#ifndef _FSL_FLEXIO_CAMERA_ADAPTER_H_
#define _FSL_FLEXIO_CAMERA_ADAPTER_H_

#include "fsl_common.h"
#include "fsl_flexio_camera.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_camera_receiver.h"
#include "fsl_ov7670.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Size of the frame buffer queue used in FLEXIO CAMERA Queue. */
#ifndef FLEXIO_CAMERA_QUEUE_SIZE
#define FLEXIO_CAMERA_QUEUE_SIZE 4U
#endif

/*! @brief Error codes for the Camera driver. */
enum _flexio_camera_queue_status
{
    kStatus_FLEXIO_CAMERA_QueueFrameDone = MAKE_STATUS(kStatusGroup_FLEXIO_CAMERA, 2),
    /*!< Frame ceceive done in queue. */
    kStatus_FLEXIO_CAMERA_QueueNoEmptyBuffer = MAKE_STATUS(kStatusGroup_FLEXIO_CAMERA, 3),
    /*!< No empty buffer in the queue. */
    kStatus_FLEXIO_CAMERA_QueueFull = MAKE_STATUS(kStatusGroup_FLEXIO_CAMERA, 4),
    /*!< Queue full. */
    kStatus_FLEXIO_CAMERA_QueueNoFullBuffer = MAKE_STATUS(kStatusGroup_FLEXIO_CAMERA, 5),
    /*!< No full buffer ready in the queue. */
};

/*!
 * @brief The resources used by the FLEXIO camera queue.
 *
 */
typedef struct _flexio_camera_queue
{
    uint32_t frameBufferQueue[FLEXIO_CAMERA_QUEUE_SIZE + 1]; /*!< Frame buffer queue. */

    volatile uint8_t queueUserReadIdx;  /*!< Application gets full-filled frame buffer from this index. */
    volatile uint8_t queueUserWriteIdx; /*!< Application puts empty frame buffer to this index. */
    volatile uint8_t queueDrvReadIdx;   /*!< Driver gets empty frame buffer from this index. */
    volatile uint8_t queueDrvWriteIdx;  /*!< Driver puts the full-filled frame buffer to this index. */
    volatile bool transferStarted; /*!< User has called @ref FLEXIO_CAMERA_ADAPTER_Start to start frame receiving. */
} flexio_camera_queue_t;

/*! @brief The private data used by the FLEXIO camera receiver. */
typedef struct _flexio_camera_private_data
{
    flexio_camera_queue_t cameraQueue;   /*!< FLEXIO camera queue. */
    uint32_t frameBytes;                 /*!< Frame bytes in each frame. */
    camera_receiver_callback_t callback; /*!< Save the callback. */
    void *userData;                      /*!< Parameter for the callback. */
} flexio_camera_private_data_t;

/*!
 * @brief The resources used by the FLEXIO camera receiver.
 *
 * Don't need to initialize the resource before FLEXIO initialization.
 */
typedef struct _flexio_camera_resource
{
    FLEXIO_CAMERA_Type device; /*!< FLEXIO camera device. */
    DMAMUX_Type *dmamuxBase;   /*!< DMAMUX base address. */
    DMA_Type *dmaBase;         /*!< EDMA base address. */
    uint8_t dmaChannel;        /*!< FLEXIO DMA Channel number. */
    void (*vsyncInit)(void);   /*!< Configure vsync pin interrupt. */
} flexio_camera_resource_t;

/*! @brief FLEXIO camera receiver operations structure. */
extern const camera_receiver_operations_t flexio_camera_ops;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

void FLEXIO_CAMERA_ADAPTER_VsyncHandleIRQ(camera_receiver_handle_t *handle);

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_FLEXIO_CAMERA_ADAPTER_H_ */
