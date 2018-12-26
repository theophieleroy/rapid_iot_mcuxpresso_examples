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
#ifndef __FSL_OCOTP_H__
#define __FSL_OCOTP_H__

#include "fsl_common.h"
#include "bootloader_common.h"

/**********************************************************************************************************************
 * Definitions
 *********************************************************************************************************************/

/*! @brief Status for FLEXSPI NAND driver */
enum _serial_nand_status
{
    kStatus_OCOTP_ReadFailure = MAKE_STATUS(kStatusGroup_OCOTP, 0),    /*! < OCOTP Read Failure */
    kStatus_OCOTP_ProgramFailure = MAKE_STATUS(kStatusGroup_OCOTP, 1), /*! < OCOTP Program Failure */
    kStatus_OCOTP_ReloadFailure = MAKE_STATUS(kStatusGroup_OCOTP, 2),  /*! < OCOTP Reload Shadows Failure */
    kStatus_OCOTP_WaitTimeout = MAKE_STATUS(kStatusGroup_OCOTP, 3),    /*! < OCOTP Access Timeout */
};

/**********************************************************************************************************************
 * API
 *********************************************************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

//!@brief Initialize OCOTP
status_t ocotp_init(OCOTP_Type *base);

//!@brief Program OCOTP
status_t ocotp_program_once(OCOTP_Type *base, uint32_t index, uint32_t *src, uint32_t lengthInBytes);

//!@brief Read OCOTP
status_t ocotp_read_once(OCOTP_Type *base, uint32_t index, uint32_t *dst, uint32_t lengthInBytes);

uint32_t get_ocotp_clock(void);

#ifdef __cplusplus
}
#endif

#endif // #ifndef __FSL_OCOTP_H__
