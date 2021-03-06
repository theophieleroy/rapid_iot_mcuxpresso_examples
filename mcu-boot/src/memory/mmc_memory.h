/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
#ifndef _MMC_MEMORY_H_
#define _MMC_MEMORY_H_

#include "fsl_card.h"
#include "memory/memory.h"

/*!
 * @addtogroup mmccard
 * @{
 */

/*************************************************************************************************
 * API
 ************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief MMC card initialization function.
 *
 * Init the MMC card on a specific host controller.
 *
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_NotSupportYet Card not support.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SendRelativeAddressFailed Send relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendScrFailed Send SCR failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_init(void);

/*!
 * @brief MMC card re-initialization function.
 *
 * Re-init the MMC card on a specific host controller.
 *
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_NotSupportYet Card not support.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SendRelativeAddressFailed Send relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendScrFailed Send SCR failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_config(uint32_t *config);

/*!
 * @brief Reads data from the MMC card.
 *
 * This function reads data from the MMC card to specific destination.
 *
 * @param address Start address to read the data.
 * @param length The number of bytes to read.
 * @param buffer The buffer to save the data read from card.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

/*!
 * @brief Writes data to the MMC card.
 *
 * This function writes Data to the MMC card.
 *
 * @param address Start address to write the data to.
 * @param length The number of bytes to write.
 * @param buffer The buffer to save the data to write to the card.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

/*!
 * @brief Flush the cached data to the MMC card.
 *
 * This function writes one block data to the MMC card.
 *
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_flush(void);

/*!
 * @brief Finalize the write or read operation of the MMC card.
 *
 * This function will finalize the write or read progress, reset the state machine.
 *
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_finalize(void);

/*!
 * @brief Erases blocks of the MMC card.
 *
 * This function erases blocks of the specific card.
 *
 * @param address The start address.
 * @param length The number of blocks to erase.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryVerifyFailed Erase operation is failed at Erase verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_erase(uint32_t address, uint32_t length);

/*!
 * @brief Get the default configuration of the MMC card.
 *
 * This function gets the default configuration.
 *
 * @param card The mmc card structure.

 * @retval kStatus_Fail Operate failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t get_mmc_default_configuration(mmc_card_t *card);

#if defined(__cplusplus)
}
#endif
/*! @} */

#endif /* _MMC_MEMORY_H_*/
