/*
 * The Clear BSD License
 * Copyright (c) 2018, Freescale Semiconductor, Inc.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "fsl_pdm.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/
/*! @brief Typedef for pdm rx interrupt handler. */
typedef void (*pdm_isr_t)(PDM_Type *base, pdm_handle_t *pdmHandle);
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get the instance number for PDM.
 *
 * @param base PDM base pointer.
 */
uint32_t PDM_GetInstance(PDM_Type *base);

/*!
 * @brief Get the instance number for PDM.
 *
 * @param channelMask enabled channel.
 * @param qualitymode selected quality mode.
 * @param osr      oversample rate.
 * @param regdiv   register divider.
 */
static status_t PDM_ValidateSrcClockRate(uint32_t channelMask,
                                         pdm_df_quality_mode_t qualityMode,
                                         uint8_t osr,
                                         uint32_t regDiv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Base pointer array */
static PDM_Type *const s_pdmBases[] = PDM_BASE_PTRS;
/*!@brief PDM handle pointer */
pdm_handle_t *s_pdmHandle[ARRAY_SIZE(s_pdmBases)];
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Clock name array */
static const clock_ip_name_t s_pdmClock[] = PDM_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/*! @brief Pointer to tx IRQ handler for each instance. */
static pdm_isr_t s_pdmIsr;
/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t PDM_GetInstance(PDM_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_pdmBases); instance++)
    {
        if (s_pdmBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_pdmBases));

    return instance;
}

void PDM_ReadNonBlocking(PDM_Type *base, uint32_t startChannel, uint32_t channelNums, int16_t *buffer, size_t size)
{
    uint32_t i = 0, j = 0U;
    uint32_t *startSrcAddr = (uint32_t *)(&base->DATACH)[startChannel];

    for (i = 0U; i < size; i++)
    {
        for (j = 0; j < channelNums; j++)
        {
            *buffer++ = startSrcAddr[j];
        }
    }
}

static status_t PDM_ValidateSrcClockRate(uint32_t channelMask,
                                         pdm_df_quality_mode_t qualityMode,
                                         uint8_t osr,
                                         uint32_t regDiv)
{
    uint32_t enabledChannel = 0U, i = 0U, factor = 0U;

    for (i = 0U; i < FSL_FEATURE_PDM_CHANNEL_NUM; i++)
    {
        if (channelMask >> i)
        {
            enabledChannel++;
        }
    }

    if (qualityMode <= kPDM_QualityModeLow)
    {
        factor = 125U;
    }
    else
    {
        factor = 19U;
    }

    if (regDiv < ((10U + factor * enabledChannel) / (8U * osr)))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

status_t PDM_SetSampleRate(
    PDM_Type *base, uint32_t enableChannelMask, pdm_df_quality_mode_t qualityMode, uint8_t osr, uint32_t clkDiv)
{
    uint8_t realOsr = 16U - (osr & (PDM_CTRL_2_CICOSR_MASK >> PDM_CTRL_2_CICOSR_SHIFT));
    uint32_t regDiv = clkDiv >> 1U;

    switch (qualityMode)
    {
        case kPDM_QualityModeHigh:
            regDiv <<= 1U;
            break;
        case kPDM_QualityModeLow:
        case kPDM_QualityModeVeryLow1:
            regDiv >>= 1U;
            break;
        case kPDM_QualityModeVeryLow2:
            regDiv >>= 2U;
            break;
        default:
            break;
    }

    if (PDM_ValidateSrcClockRate(enableChannelMask, qualityMode, realOsr, regDiv) == kStatus_Fail)
    {
        return kStatus_Fail;
    }

    assert(regDiv <= PDM_CTRL_2_CLKDIV_MASK);
    base->CTRL_2 = (base->CTRL_2 & (~PDM_CTRL_2_CLKDIV_MASK)) | PDM_CTRL_2_CLKDIV(regDiv);

    return kStatus_Success;
}

void PDM_Init(PDM_Type *base, const pdm_config_t *config)
{
    assert(config != NULL);
    assert(config->fifoWatermark <= PDM_FIFO_CTRL_FIFOWMK_MASK);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the PDM clock */
    CLOCK_EnableClock(s_pdmClock[PDM_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Enable the module and disable the interface/all channel */
    base->CTRL_1 &= ~(PDM_CTRL_1_MDIS_MASK | PDM_CTRL_1_PDMIEN_MASK | PDM_CTRL_1_ERREN_MASK | kPDM_EnableChannelAll);

    /* wait all filter stopped */
    while (base->STAT & PDM_STAT_BSY_FIL_MASK)
    {
    }

    /* software reset */
    base->CTRL_1 |= PDM_CTRL_1_SRES_MASK;

    /* Set the configure settings */
    base->CTRL_1 = (base->CTRL_1 & (~PDM_CTRL_1_DOZEN_MASK)) | PDM_CTRL_1_DOZEN(config->enableDoze);

    base->CTRL_2 = (base->CTRL_2 & (~(PDM_CTRL_2_CICOSR_MASK | PDM_CTRL_2_QSEL_MASK))) |
                   PDM_CTRL_2_CICOSR(config->cicOverSampleRate) | PDM_CTRL_2_QSEL(config->qualityMode);

    /* Set the watermark */
    base->FIFO_CTRL = PDM_FIFO_CTRL_FIFOWMK(config->fifoWatermark);
}

void PDM_Deinit(PDM_Type *base)
{
    /* disable PDM interface */
    PDM_Enable(base, false);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(s_pdmClock[PDM_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void PDM_EnableInterrupts(PDM_Type *base, uint32_t mask)
{
    if (mask & kPDM_FIFOInterruptEnable)
    {
        base->CTRL_1 = (base->CTRL_1 & (~PDM_CTRL_1_DISEL_MASK)) | kPDM_FIFOInterruptEnable;
    }
    if (mask & kPDM_ErrorInterruptEnable)
    {
        base->CTRL_1 = (base->CTRL_1 & (~PDM_CTRL_1_ERREN_MASK)) | kPDM_ErrorInterruptEnable;
    }
}

void PDM_SetChannelConfig(PDM_Type *base, uint32_t channel, const pdm_channel_config_t *config)
{
    assert(config != NULL);
    assert(channel <= FSL_FEATURE_PDM_CHANNEL_NUM);

    uint32_t dcCtrl = base->DC_CTRL;
    uint32_t outCtrl = base->OUT_CTRL;

    /* configure gain and cut off freq */
    dcCtrl &= ~(PDM_DC_CTRL_DCCONFIG0_MASK << (channel << 1U));
    dcCtrl |= config->cutOffFreq << (channel << 1U);
    outCtrl &= ~(PDM_OUT_CTRL_OUTGAIN0_MASK << (channel << 2U));
    outCtrl |= config->gain << (channel << 2U);

    base->DC_CTRL = dcCtrl;
    base->OUT_CTRL = outCtrl;

    /* enable channel */
    base->CTRL_1 |= 1U << channel;
}

void PDM_TransferCreateHandle(PDM_Type *base, pdm_handle_t *handle, pdm_transfer_callback_t callback, void *userData)
{
    assert(handle);

    uint32_t val = 0, i = 0;

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    s_pdmHandle[PDM_GetInstance(base)] = handle;

    handle->callback = callback;
    handle->userData = userData;
    handle->watermark = base->FIFO_CTRL & PDM_FIFO_CTRL_FIFOWMK_MASK;

    /* Compute enabled channel number */
    val = base->CTRL_1 & 0xFFU;
    for (i = 0U; i < FSL_FEATURE_PDM_CHANNEL_NUM; i++)
    {
        if (val & (1 << i))
        {
            handle->startChannel = i;
            handle->channelNums++;
        }
    }
    /* get start channel number, current value in startchannel is end channel number actually */
    handle->startChannel = handle->startChannel - (handle->channelNums - 1U);

    /* Set the isr pointer */
    s_pdmIsr = PDM_TransferHandleIRQ;

    /* Enable RX event IRQ */
    EnableIRQ(PDM_EVENT_IRQn);
    /* Enable FIFO error IRQ */
    EnableIRQ(PDM_ERROR_IRQn);
}

status_t PDM_TransferReceiveNonBlocking(PDM_Type *base, pdm_handle_t *handle, pdm_transfer_t *xfer)
{
    assert(handle);

    /* Check if the queue is full */
    if (handle->pdmQueue[handle->queueUser].data)
    {
        return kStatus_PDM_QueueFull;
    }

    /* Add into queue */
    handle->transferSize[handle->queueUser] = xfer->dataSize;
    handle->pdmQueue[handle->queueUser].data = xfer->data;
    handle->pdmQueue[handle->queueUser].dataSize = xfer->dataSize;
    handle->queueUser = (handle->queueUser + 1) % PDM_XFER_QUEUE_SIZE;

    /* Set state to busy */
    handle->state = kStatus_PDM_Busy;

    /* Enable interrupt */
    PDM_EnableInterrupts(base, kPDM_FIFOInterruptEnable);

    PDM_Enable(base, true);

    return kStatus_Success;
}

void PDM_TransferAbortReceive(PDM_Type *base, pdm_handle_t *handle)
{
    assert(handle);

    /* Use FIFO request interrupt and fifo error */
    PDM_DisableInterrupts(base, kPDM_FIFOInterruptEnable | kPDM_ErrorInterruptEnable);
    PDM_Enable(base, false);
    handle->state = kStatus_PDM_Idle;
    /* Clear the queue */
    memset(handle->pdmQueue, 0, sizeof(pdm_transfer_t) * PDM_XFER_QUEUE_SIZE);
    handle->queueDriver = 0;
    handle->queueUser = 0;
}

void PDM_TransferHandleIRQ(PDM_Type *base, pdm_handle_t *handle)
{
    assert(handle);

    int16_t *buffer = (int16_t *)handle->pdmQueue[handle->queueDriver].data;

    /* Handle transfer */
    if ((base->STAT & 0xFFU) && (handle->channelNums) &&
        ((base->CTRL_1 & PDM_CTRL_1_DISEL_MASK) == (0x2U << PDM_CTRL_1_DISEL_SHIFT)))
    {
        PDM_ClearStatus(base, 0xFFU);
        /* Judge if the data need to transmit is less than space */
        uint8_t size =
            MIN((handle->pdmQueue[handle->queueDriver].dataSize), (handle->watermark * handle->channelNums * 2U));

        PDM_ReadNonBlocking(base, handle->startChannel, handle->channelNums, buffer,
                            (size / handle->channelNums) >> 1U);
        /* Update the internal counter */
        handle->pdmQueue[handle->queueDriver].dataSize -= size;
        handle->pdmQueue[handle->queueDriver].data += size;
    }

    /* If finished a block, call the callback function */
    if (handle->pdmQueue[handle->queueDriver].dataSize == 0U)
    {
        handle->pdmQueue[handle->queueDriver].data = NULL;
        handle->queueDriver = (handle->queueDriver + 1) % PDM_XFER_QUEUE_SIZE;
        if (handle->callback)
        {
            (handle->callback)(base, handle, kStatus_PDM_Idle, handle->userData);
        }
    }

    /* If all data finished, just stop the transfer */
    if (handle->pdmQueue[handle->queueDriver].data == NULL)
    {
        PDM_TransferAbortReceive(base, handle);
    }
}

void PDM_SetHwvadConfig(PDM_Type *base, const pdm_hwvad_config_t *config)
{
    assert(config != NULL);

    uint32_t ctrl1 = base->VAD0_CTRL_1;

    /* Configure VAD0_CTRL_1 register */
    ctrl1 &= ~(PDM_VAD0_CTRL_1_VADCHSEL_MASK | PDM_VAD0_CTRL_1_VADCICOSR_MASK | PDM_VAD0_CTRL_1_VADINITT_MASK);
    ctrl1 |= (PDM_VAD0_CTRL_1_VADCHSEL(config->channel) | PDM_VAD0_CTRL_1_VADCICOSR(config->cicOverSampleRate) |
              PDM_VAD0_CTRL_1_VADINITT(config->initializeTime));
    base->VAD0_CTRL_1 = ctrl1;

    /* Configure VAD0_CTRL_2 register */
    base->VAD0_CTRL_2 =
        (PDM_VAD0_CTRL_2_VADFRENDIS(config->enableFrameEnergy) | PDM_VAD0_CTRL_2_VADPREFEN(config->enablePreFilter) |
         PDM_VAD0_CTRL_2_VADFRAMET(config->frameTime) | PDM_VAD0_CTRL_2_VADINPGAIN(config->inputGain) |
         PDM_VAD0_CTRL_2_VADHPF(config->cutOffFreq));
}

void PDM_SetHwvadSignalFilterConfig(PDM_Type *base, bool enableMaxBlock, uint32_t signalGain)
{
    uint32_t signalConfig = base->VAD0_SCONFIG;

    signalConfig &= ~(PDM_VAD0_SCONFIG_VADSMAXEN_MASK | PDM_VAD0_SCONFIG_VADSGAIN_MASK);
    signalConfig |= (PDM_VAD0_SCONFIG_VADSMAXEN(enableMaxBlock) | PDM_VAD0_SCONFIG_VADSGAIN(signalGain)) |
                    PDM_VAD0_SCONFIG_VADSFILEN_MASK;
    base->VAD0_SCONFIG = signalConfig;
}

void PDM_SetHwvadNoiseFilterConfig(PDM_Type *base, const pdm_hwvad_noise_filter_t *config)
{
    assert(config != NULL);

    base->VAD0_NCONFIG =
        (PDM_VAD0_NCONFIG_VADNFILAUTO(config->enableAutoNoiseFilter) |
         PDM_VAD0_NCONFIG_VADNMINEN(config->enableNoiseMin) |
         PDM_VAD0_NCONFIG_VADNDECEN(config->enableNoiseDecimation) |
         PDM_VAD0_NCONFIG_VADNFILADJ(config->noiseFilterAdjustment) | PDM_VAD0_NCONFIG_VADNGAIN(config->noiseGain));
}

void PDM_SetHwvadZeroCrossDetectorConfig(PDM_Type *base, const pdm_hwvad_zero_cross_detector_t *config)
{
    assert(config != NULL);

    uint32_t zcd = (base->VAD0_ZCD & (~(PDM_VAD0_ZCD_VADZCDTH_MASK | PDM_VAD0_ZCD_VADZCDADJ_MASK |
                                        PDM_VAD0_ZCD_VADZCDAUTO_MASK | PDM_VAD0_ZCD_VADZCDAND_MASK)));

    zcd |= (PDM_VAD0_ZCD_VADZCDTH(config->threshold) | PDM_VAD0_ZCD_VADZCDADJ(config->adjustmentThreshold) |
            PDM_VAD0_ZCD_VADZCDAUTO(config->enableAutoThreshold) | PDM_VAD0_ZCD_VADZCDAND(config->zcdAnd)) |
           PDM_VAD0_ZCD_VADZCDEN_MASK;

    base->VAD0_ZCD = zcd;
}

#if defined(PDM)
void PDM_DriverIRQHandler(void)
{
    assert(s_pdmHandle[0]);
    s_pdmIsr(PDM, s_pdmHandle[0]);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
