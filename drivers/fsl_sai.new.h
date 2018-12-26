/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#ifndef _FSL_SAI_H_
#define _FSL_SAI_H_

#include "fsl_common.h"

/*!
 * @addtogroup sai
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FSL_SAI_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */

/*! @brief SAI return status*/
enum _sai_status_t
{
    kStatus_SAI_TxBusy = MAKE_STATUS(kStatusGroup_SAI, 0), /*!< SAI Tx is busy. */
    kStatus_SAI_RxBusy = MAKE_STATUS(kStatusGroup_SAI, 1), /*!< SAI Rx is busy. */
    kStatus_SAI_TxError = MAKE_STATUS(kStatusGroup_SAI, 2), /*!< SAI Tx FIFO error. */
    kStatus_SAI_RxError = MAKE_STATUS(kStatusGroup_SAI, 3), /*!< SAI Rx FIFO error. */
    kStatus_SAI_QueueFull = MAKE_STATUS(kStatusGroup_SAI, 4), /*!< SAI transfer queue is full. */
    kStatus_SAI_TxIdle = MAKE_STATUS(kStatusGroup_SAI, 5), /*!< SAI Tx is idle */
    kStatus_SAI_RxIdle = MAKE_STATUS(kStatusGroup_SAI, 6) /*!< SAI Rx is idle */
};

/*! @brief Define the SAI bus type */
typedef enum _sai_protocol
{
    kSAI_BusLeftJustified = 0x0U, /*!< Uses left justified format.*/
    kSAI_BusRightJustified, /*!< Uses right justified format. */
    kSAI_BusI2S, /*!< Uses I2S format. */
    kSAI_BusPCMA, /*!< Uses I2S PCM A format.*/
    kSAI_BusPCMB /*!< Uses I2S PCM B format. */
} sai_protocol_t;

/*! @brief Master or slave mode */
typedef enum _sai_master_slave
{
    kSAI_Master = 0x0U, /*!< Master mode */
    kSAI_Slave = 0x1U /*!< Slave mode */
} sai_master_slave_t;

/*! @brief Mono or stereo audio format */
typedef enum _sai_mono_stereo
{
    kSAI_Stereo = 0x0U, /*!< Stereo sound. */
    kSAI_MonoLeft, /*!< Only left channel have sound. */
    kSAI_MonoRight /*!< Only Right channel have sound. */
} sai_mono_stereo_t;

/*! @brief Synchronous or asynchronous mode */
typedef enum _sai_sync_mode
{
    kSAI_ModeAsync = 0x0U, /*!< Asynchronous mode */
    kSAI_ModeSync, /*!< Synchronous mode (with receiver or transmit) */
    kSAI_ModeSyncWithOtherTx, /*!< Synchronous with another SAI transmit */
    kSAI_ModeSyncWithOtherRx /*!< Synchronous with another SAI receiver */
} sai_sync_mode_t;

/*! @brief Mater clock source */
typedef enum _sai_mclk_source
{
    kSAI_MclkSourceSysclk = 0x0U, /*!< Master clock from the system clock */
    kSAI_MclkSourceSelect1, /*!< Master clock from source 1 */
    kSAI_MclkSourceSelect2, /*!< Master clock from source 2 */
    kSAI_MclkSourceSelect3 /*!< Master clock from source 3 */
} sai_mclk_source_t;

/*! @brief Bit clock source */
typedef enum _sai_bclk_source
{
    kSAI_BclkSourceBusclk = 0x0U, /*!< Bit clock using bus clock */
    kSAI_BclkSourceMclkDiv, /*!< Bit clock using master clock divider */
    kSAI_BclkSourceOtherSai0, /*!< Bit clock from other SAI device  */
    kSAI_BclkSourceOtherSai1 /*!< Bit clock from other SAI device */
} sai_bclk_source_t;

/*! @brief The SAI interrupt enable flag */
enum _sai_interrupt_enable_t
{
    kSAI_WordStartInterruptEnable = I2S_TCSR_WSIE_MASK, /*!< Word start flag, means the first word
                                                           in a frame detected */
    kSAI_SyncErrorInterruptEnable = I2S_TCSR_SEIE_MASK, /*!< Sync error flag, means the sync error
                                                           is detected */
    kSAI_FIFOWarningInterruptEnable = I2S_TCSR_FWIE_MASK, /*!< FIFO warning flag, means the FIFO is
                                                             empty */
    kSAI_FIFOErrorInterruptEnable = I2S_TCSR_FEIE_MASK, /*!< FIFO error flag */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestInterruptEnable = I2S_TCSR_FRIE_MASK, /*!< FIFO request, means reached watermark
                                                             */
#endif /* FSL_FEATURE_SAI_FIFO_COUNT */
};

/*! @brief The DMA request sources */
enum _sai_dma_enable_t
{
    kSAI_FIFOWarningDMAEnable = I2S_TCSR_FWDE_MASK, /*!< FIFO warning caused by the DMA request */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestDMAEnable = I2S_TCSR_FRDE_MASK, /*!< FIFO request caused by the DMA request */
#endif /* FSL_FEATURE_SAI_FIFO_COUNT */
};

/*! @brief The SAI status flag */
enum _sai_flags
{
    kSAI_WordStartFlag = I2S_TCSR_WSF_MASK, /*!< Word start flag, means the first word in a frame
                                               detected */
    kSAI_SyncErrorFlag = I2S_TCSR_SEF_MASK, /*!< Sync error flag, means the sync error is detected
                                               */
    kSAI_FIFOErrorFlag = I2S_TCSR_FEF_MASK, /*!< FIFO error flag */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestFlag = I2S_TCSR_FRF_MASK, /*!< FIFO request flag. */
#endif /* FSL_FEATURE_SAI_FIFO_COUNT */
    kSAI_FIFOWarningFlag = I2S_TCSR_FWF_MASK, /*!< FIFO warning flag */
    kSAI_SoftResetFlag = I2S_TCSR_SR_MASK, /*!< Software reset flag */
};

/*! @brief The reset type */
typedef enum _sai_reset_type
{
    kSAI_ResetTypeSoftware = I2S_TCSR_SR_MASK, /*!< Software reset, reset the logic state */
    kSAI_ResetTypeFIFO = I2S_TCSR_FR_MASK, /*!< FIFO reset, reset the FIFO read and write pointer */
    kSAI_ResetAll = I2S_TCSR_SR_MASK | I2S_TCSR_FR_MASK /*!< All reset. */
} sai_reset_type_t;

#if defined(FSL_FEATURE_SAI_HAS_FIFO_PACKING) && FSL_FEATURE_SAI_HAS_FIFO_PACKING
/*!
 * @brief The SAI packing mode
 * The mode includes 8 bit and 16 bit packing.
 */
typedef enum _sai_fifo_packing
{
    kSAI_FifoPackingDisabled = 0x0U, /*!< Packing disabled */
    kSAI_FifoPacking8bit = 0x2U, /*!< 8 bit packing enabled */
    kSAI_FifoPacking16bit = 0x3U /*!< 16bit packing enabled */
} sai_fifo_packing_t;
#endif /* FSL_FEATURE_SAI_HAS_FIFO_PACKING */

/*! @brief SAI user configure structure */
typedef struct _sai_config
{
    sai_protocol_t protocol; /*!< Audio bus protocol in SAI */
    sai_sync_mode_t syncMode; /*!< SAI sync mode, control Tx/Rx clock sync */
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    bool mclkOutputEnable; /*!< Master clock output enable, true means master clock divider enabled
                              */
#endif /* FSL_FEATURE_SAI_HAS_MCR */
    sai_mclk_source_t mclkSource; /*!< Master Clock source */
    sai_bclk_source_t bclkSource; /*!< Bit Clock source */
    sai_master_slave_t masterSlave; /*!< Master or slave */
} sai_config_t;

/*!@brief SAI transfer queue size, user can refine it according to use case. */
#define SAI_XFER_QUEUE_SIZE (4)

/*! @brief Audio sample rate */
typedef enum _sai_sample_rate
{
    kSAI_SampleRate8KHz = 8000U, /*!< Sample rate 8000Hz */
    kSAI_SampleRate11025Hz = 11025U, /*!< Sample rate 11025Hz */
    kSAI_SampleRate12KHz = 12000U, /*!< Sample rate 12000Hz */
    kSAI_SampleRate16KHz = 16000U, /*!< Sample rate 16000Hz */
    kSAI_SampleRate22050Hz = 22050U, /*!< Sample rate 22050Hz */
    kSAI_SampleRate24KHz = 24000U, /*!< Sample rate 24000Hz */
    kSAI_SampleRate32KHz = 32000U, /*!< Sample rate 32000Hz */
    kSAI_SampleRate44100Hz = 44100U, /*!< Sample rate 44100Hz */
    kSAI_SampleRate48KHz = 48000U, /*!< Sample rate 48000Hz */
    kSAI_SampleRate96KHz = 96000U /*!< Sample rate 96000Hz */
} sai_sample_rate_t;

/*! @brief Audio word width */
typedef enum _sai_word_width
{
    kSAI_WordWidth8bits = 8U, /*!< Audio data width 8 bits */
    kSAI_WordWidth16bits = 16U, /*!< Audio data width 16 bits */
    kSAI_WordWidth24bits = 24U, /*!< Audio data width 24 bits */
    kSAI_WordWidth32bits = 32U /*!< Audio data width 32 bits */
} sai_word_width_t;

/*! @brief sai transfer format */
typedef struct _sai_transfer_format
{
    uint32_t sampleRate_Hz; /*!< Sample rate of audio data */
    uint32_t bitWidth; /*!< Data length of audio data, usually 8/16/24/32bits */
    sai_mono_stereo_t stereo; /*!< Mono or stereo */
    uint32_t masterClockHz; /*!< Master clock frequency in Hz */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    uint8_t watermark; /*!< Watermark value */
#endif /* FSL_FEATURE_SAI_FIFO_COUNT */
    uint8_t channel; /*!< Data channel used in transfer.*/
    sai_protocol_t protocol; /*!< Which audio protocol used */
} sai_transfer_format_t;

/*! @brief SAI transfer structure */
typedef struct _sai_transfer
{
    uint8_t * data; /*!< Data start address to transfer. */
    size_t dataSize; /*!< Transfer size. */
} sai_transfer_t;

typedef struct _sai_handle sai_handle_t;

/*! @brief SAI xfer callback prototype */
typedef void (*sai_transfer_callback_t)(I2S_Type * base, sai_handle_t * handle, status_t status, void * userData);

/*! @brief SAI handle structure */
struct _sai_handle
{
    uint32_t state; /*!< Transfer status */
    sai_transfer_callback_t callback; /*!< Callback function called at transfer event*/
    void * userData; /*!< Callback parameter passed to callback function*/
    uint8_t bitWidth; /*!< Bit width for transfer, 8/16/24/32bits */
    uint8_t channel; /*!< Transfer channel */
    sai_transfer_t saiQueue[SAI_XFER_QUEUE_SIZE]; /*!< Transfer queue storing queued transfer */
    volatile uint8_t queueUser; /*!< Index for user to queue transfer */
    volatile uint8_t queueDriver; /*!< Index for driver to get the transfer data and size */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    uint8_t watermark; /*!< Watermark value */
#endif
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initialize the SAI Tx peripheral
 *
 * This API to ungate the SAI clock, reset the module and configure SAI Tx with configuration
 * structure.
 * The configuration structure can be filled by user from scratch, or be set with default values by
 * SAI_TxGetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the sai driver, or any access to the sai module could cause hard fault
 * because clock is not enabled.
 *
 * @param base SAI base pointer
 * @param config SAI configure structure.
*/
void SAI_TxInit(I2S_Type * base, const sai_config_t * config);

/*!
 * @brief Initialize the SAI Rx peripheral
 *
 * This API to ungate the SAI clock, reset the module and configure SAI Rx with configuration
 * structure.
 * The configuration structure can be filled by user from scratch, or be set with default values by
 * SAI_RxGetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the sai driver, or any access to the sai module could cause hard fault
 * because clock is not enabled.
 *
 * @param base SAI base pointer
 * @param config SAI configure structure.
 */
void SAI_RxInit(I2S_Type * base, const sai_config_t * config);

/*!
 * @brief  Set the SAI Tx configuration structure to default values.
 *
 * The purpose of this API is to get the config structure initialized for use in SAI_TxConfig().
 * User may use the initialized structure unchanged in SAI_TxConfig(), or modify
 * some fields of the structure before calling SAI_TxConfig().
 * Example:
   @code
   sai_config_t config;
   SAI_TxGetDefaultConfig(&config);
   @endcode
 *
 * @param userConfig pointer to master config structure
 */
void SAI_TxGetDefaultConfig(sai_config_t * config);

/*!
 * @brief  Set the SAI Rx configuration structure to default values.
 *
 * The purpose of this API is to get the config structure initialized for use in SAI_RxConfig().
 * User may use the initialized structure unchanged in SAI_RxConfig(), or modify
 * some fields of the structure before calling SAI_RxConfig().
 * Example:
   @code
   sai_config_t config;
   SAI_RxGetDefaultConfig(&config);
   @endcode
 *
 * @param userConfig pointer to master config structure
 */
void SAI_RxGetDefaultConfig(sai_config_t * config);

/*!
 * @brief De-initialize the SAI peripheral.
 *
 * Call thi API will gate the sai clock, so the SAI module can not work unless call SAI_TxInit
 * or SAI_RxInit to enable the clock.
 *
 * @param base SAI base pointer
*/
void SAI_Deinit(I2S_Type * base);

/*!
 * @brief Reset SAI Tx.
 *
 * This function will enable the software reset and FIFO reset of SAI Tx. After reset clear the
 * reset bit.
 *
 * @param base SAI base pointer
 */
void SAI_TxReset(I2S_Type * base);

/*!
 * @brief Reset SAI Rx.
 *
 * This function will enable the software reset and FIFO reset of SAI Rx. After reset clear the
 * reset bit.
 *
 * @param base SAI base pointer
 */
void SAI_RxReset(I2S_Type * base);

/*!
 * @brief Enable/Disable SAI Tx.
 *
 * @param base SAI base pointer
 */
void SAI_TxEnable(I2S_Type * base, bool enable);

/*!
 * @brief Enable/Disable SAI Rx.
 *
 * @param base SAI base pointer
 */
void SAI_RxEnable(I2S_Type * base, bool enable);

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the SAI Tx status flag state.
 *
 * @param base SAI base pointer
 * @return SAI Tx status flag value, users need to use Status Mask to get the status value needed.
 */
static inline uint32_t SAI_TxGetStatusFlag(I2S_Type * base)
{
    return base->TCSR;
}

/*!
 * @brief Gets the SAI Tx status flag state.
 *
 * @param base SAI base pointer
 * @return SAI Rx status flag value, users need to use Status Mask to get the status value needed.
 */
static inline uint32_t SAI_RxGetStatusFlag(I2S_Type * base)
{
    return base->RCSR;
}

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables SAI Tx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_TxEnableInterrupts(I2S_Type * base, uint32_t mask)
{
    base->TCSR |= mask;
}

/*!
 * @brief Enables SAI Rx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_RxEnableInterrupts(I2S_Type * base, uint32_t mask)
{
    base->RCSR |= mask;
}

/*!
 * @brief Disables SAI Tx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_TxDisableInterrupts(I2S_Type * base, uint32_t mask)
{
    base->TCSR &= ~mask;
}

/*!
 * @brief Disables SAI Rx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_RxDisableInterrupts(I2S_Type * base, uint32_t mask)
{
    base->RCSR &= ~mask;
}

/*! @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enables/Disable SAI Tx DMA requests.
 * @param base SAI base pointer
 * @param mask DMA source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_FIFOWarningDMAEnable
 *     @arg kSAI_FIFORequestDMAEnable
 * @param enable True means enable DMA, false means disable DMA.
 */
static inline void SAI_TxEnableDMA(I2S_Type * base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->TCSR |= mask;
    }
    else
    {
        base->TCSR &= ~mask;
    }
}

/*!
 * @brief Enables/Disable SAI Rx DMA requests.
 * @param base SAI base pointer
 * @param mask DMA source
 *     The parameter can be combination of the following source if defined:
 *     @arg kSAI_FIFOWarningDMAEnable
 *     @arg kSAI_FIFORequestDMAEnable
 */
static inline void SAI_RxEnableDMA(I2S_Type * base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->RCSR |= mask;
    }
    else
    {
        base->RCSR &= ~mask;
    }
}

/*!
 * @brief  Get SAI tx data register address.
 *
 * This API is used to provide transfer address for SAI DMA transfer configuration.
 *
 * @param base SAI base pointer
 * @param channel Which data channel used.
 * @return data register address
 */
static inline uint32_t SAI_TxGetDataRegAddr(I2S_Type * base, uint32_t channel)
{
    return (uint32_t)(&(base->TDR)[channel]);
}

/*!
 * @brief  Get SAI rx data register address.
 *
 * This API is used to provide transfer address for SAI DMA transfer configuration.
 *
 * @param base SAI base pointer
 * @param channel Which data channel used.
 * @return data register address
 */
static inline uint32_t SAI_RxGetDataRegAddr(I2S_Type * base, uint32_t channel)
{
    return (uint32_t)(&(base->RDR)[channel]);
}

/*! @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Configure SAI Tx audio format.
 *
 * Audio format can be changed in run-time of SAI. This function configures the sample rate and
 * audio data
 * format to be transferred.
 *
 * @param handle SAI handle pointer
 * @param format Pointer to SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If bit clock source is master
 * clock, this value should equals to masterClockHz in format.
*/
void SAI_TxSetFormat(I2S_Type * base, sai_transfer_format_t * format, uint32_t mclkSourceClockHz, uint32_t bclkSourceClockHz);

/*!
 * @brief Configure SAI Rx audio format.
 *
 * Audio format can be changed in run-time of SAI. This function configures the sample rate and
 * audio data
 * format to be transferred.
 *
 * @param handle SAI handle pointer
 * @param format Pointer to SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If bit clock source is master
 * clock, this value should equals to masterClockHz in format.
*/
void SAI_RxSetFormat(I2S_Type * base, sai_transfer_format_t * format, uint32_t mclkSourceClockHz, uint32_t bclkSourceClockHz);

/*!
 * @brief sends a piece of data in blocking way.
 *
 * @note This function blocks via polling until data is ready to be sent.
 *
 * @param base SAI base pointer
 * @param channel Data channel used.
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SAI_WriteBlocking(I2S_Type * base, uint32_t channel, uint32_t bitWidth, uint8_t * buffer, uint32_t size);

/*!
 * @brief sends a piece of data in non-blocking way.
 *
 * @param base SAI base pointer
 * @param channel Data channel used.
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SAI_WriteNonBlocking(I2S_Type * base, uint32_t channel, uint32_t bitWidth, uint8_t * buffer, uint32_t size);

/*!
 * @brief Receive a piece of data in blocking way.
 *
 * @note This function blocks via polling until data is ready to be sent.
 *
 * @param base SAI base pointer
 * @param channel Data channel used.
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SAI_ReadBlocking(I2S_Type * base, uint32_t channel, uint32_t bitWidth, uint8_t * buffer, uint32_t size);

/*!
 * @brief Receive a piece of data in non-blocking way.
 *
 * @param base SAI base pointer
 * @param channel Data channel used.
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SAI_ReadNonBlocking(I2S_Type * base, uint32_t channel, uint32_t bitWidth, uint8_t * buffer, uint32_t size);

/*! @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initialize the SAI Tx handle.
 *
 * This function initializes the Tx handle for SAI Tx transactional APIs. Users only need to call
 * this function once to get the handle initialized.
 *
 * @param handle SAI handle pointer.
 * @param base SAI base pointer
 * @param callback pointer to user callback function
 * @param param user parameter passed to the callback function
 */
void SAI_TxCreateHandle(I2S_Type * base, sai_handle_t * handle, sai_transfer_callback_t callback, void * userData);

/*!
 * @brief Initialize the SAI Rx handle.
 *
 * This function initializes the Rx handle for SAI Rx transactional APIs. Users only need to call
 * this function once to get the handle initialized.
 *
 * @param handle SAI handle pointer.
 * @param base SAI base pointer
 * @param callback pointer to user callback function
 * @param param user parameter passed to the callback function
 */
void SAI_RxCreateHandle(I2S_Type * base, sai_handle_t * handle, sai_transfer_callback_t callback, void * userData);

/*!
 * @brief Configure SAI Tx audio format.
 *
 * Audio format can be changed in run-time of SAI. This function configures the sample rate and
 * audio data
 * format to be transferred.
 *
 * @param handle SAI handle pointer
 * @param format Pointer to SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If bit clock source is master
 * clock, this value should equals to masterClockHz in format.
 * @return Status of this function. Return value is one of status_t.
*/
status_t SAI_TxSetTransferFormat(I2S_Type * base, sai_handle_t * handle, sai_transfer_format_t * format, uint32_t mclkSourceClockHz, uint32_t bclkSourceClockHz);

/*!
 * @brief Configure SAI Rx audio format.
 *
 * Audio format can be changed in run-time of SAI. This function configures the sample rate and
 * audio data
 * format to be transferred.
 *
 * @param handle SAI handle pointer
 * @param format Pointer to SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If bit clock source is master
 * clock, this value should equals to masterClockHz in format.
 * @return Status of this function. Return value is one of status_t.
*/
status_t SAI_RxSetTransferFormat(I2S_Type * base, sai_handle_t * handle, sai_transfer_format_t * format, uint32_t mclkSourceClockHz, uint32_t bclkSourceClockHz);

/*!
 * @brief Performs an interrupt non-blocking send transfer on SAI
 *
 * @note Calling the API will return immediately after transfer initiates, user needs
 * to call SAI_TxGetTransferStatusIRQ to poll the transfer status to check whether
 * the transfer is finished, if the return status is not kStatus_SAI_Busy, the transfer
 * is finished.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 * @param xfer pointer to sai_transfer_t structure
 * @return status of status_t
 */
status_t SAI_SendNonBlocking(I2S_Type * base, sai_handle_t * handle, sai_transfer_t * xfer);

/*!
 * @brief Performs an interrupt non-blocking receive transfer on SAI
 *
 * @note Calling the API will return immediately after transfer initiates, user needs
 * to call SAI_RxGetTransferStatusIRQ to poll the transfer status to check whether
 * the transfer is finished, if the return status is not kStatus_SAI_Busy, the transfer
 * is finished.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 * @param xfer pointer to sai_transfer_t structure
 * @return status of status_t
 */
status_t SAI_ReceiveNonBlocking(I2S_Type * base, sai_handle_t * handle, sai_transfer_t * xfer);

/*!
 * @brief Get the remaining bytes to be sent.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 * @return Remaining bytes to be sent.
 */
size_t SAI_GetSendRemainingBytes(I2S_Type * base, sai_handle_t * handle);

/*!
 * @brief Get the remaining bytes to be received.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 * @return Remaining bytes to be received.
 */
size_t SAI_GetReceiveRemainingBytes(I2S_Type * base, sai_handle_t * handle);

/*!
 * @brief Abort the current Send.
 *
 * @note This API could be called at any time when interrupt non-blocking transfer initiates
 * to abort the transfer in a early time.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 */
void SAI_AbortSend(I2S_Type * base, sai_handle_t * handle);

/*!
 * @brief Abort the current IRQ receive.
 *
 * @note This API could be called at any time when interrupt non-blocking transfer initiates
 * to abort the transfer in a early time.
 *
 * @param handle pointer to sai_handle_t structure which stores the transfer state
 */
void SAI_AbortReceive(I2S_Type * base, sai_handle_t * handle);

/*!
 * @brief Tx interrupt handler
 *
 * @param handle pointer to sai_handle_t structure
 */
void SAI_TxHandleIRQ(I2S_Type * base, sai_handle_t * handle);

/*!
 * @brief Tx interrupt handler
 *
 * @param handle pointer to sai_handle_t structure
 */
void SAI_RxHandleIRQ(I2S_Type * base, sai_handle_t * handle);

/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* _FSL_SAI_H_ */
