/*
 * The Clear BSD License
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#include "fsl_flexspi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* LUT sequence index for READ cache sequence  */
#define NAND_CMD_LUT_SEQ_IDX_READCACHE 0
/* LUT sequence index for Read Status sequence */
#define NAND_CMD_LUT_SEQ_IDX_READSTATUS 1
/* LUT sequence index for Read ID sequence */
#define NAND_CMD_LUT_SEQ_IDX_READJEDECID 2
/* LUT sequence index for write enable sequence */
#define NAND_CMD_LUT_SEQ_IDX_WRITEENABLE 3
/* LUT sequence index for Read cache for odd blocks */
#define NAND_CMD_LUT_SEQ_IDX_READCACHE_ODD 4
/* LUT sequence index for erase block */
#define NAND_CMD_LUT_SEQ_IDX_ERASEBLOCK 5
/* LUT sequence index for program load */
#define NAND_CMD_LUT_SEQ_IDX_PROGRAMLOAD 6
/* LUT sequence index for program load for odd blocks */
#define NAND_CMD_LUT_SEQ_IDX_PROGRAMLOAD_ODD 7
/* LUT sequence index for program load for read page */
#define NAND_CMD_LUT_SEQ_IDX_READPAGE 8
/* LUT sequence index for read ecc status  */
#define NAND_CMD_LUT_SEQ_IDX_READECCSTAT 9
/* LUT sequence index for program execute */
#define NAND_CMD_LUT_SEQ_IDX_PROGRAMEXECUTE 10
/* LUT sequence index for get parameter table */
#define NAND_CMD_LUT_SEQ_IDX_SETFEATURE 11
/* Unlock all blocks */
#define NAND_CMD_LUT_SEQ_IDX_UNLOCKALL 12

/* !@brief FlexSPI Memory Configuration Block */
typedef struct _flexspi_memory_config
{
    flexspi_device_config_t deviceConfig; /*!< Device configuration structure */
    flexspi_port_t devicePort;     /*!< Device connected to which port, SS0_A means port A1, SS0_B means port B1*/
    uint32_t dataBytesPerPage;     /*!< Data Size in one page, usually it is 2048 or 4096*/
    uint32_t bytesInPageSpareArea; /*!< Total size in one page, usually, it equals 2 ^ width of column address*/
    uint32_t pagesPerBlock;        /*!< Pages per block*/
    uint16_t busyOffset; /*!< Busy offset, valid value: 0-31, only need for check option kNandReadyCheckOption_RB */
    uint16_t busyBitPolarity; /*!< Busy flag polarity, 0 - busy flag is 1 when flash device is busy, 1 -busy flag is 0
                                   when flash device is busy, only need for check option kNandReadyCheckOption_RB */
    uint32_t eccStatusMask;   /*!< ECC status mask */
    uint32_t eccFailureMask;  /*!< ECC failure mask */
    uint32_t lookupTable[64]; /*!< Lookup table holds Flash command sequences */
} flexspi_mem_config_t;

/*!@brief NAND Flash handle info*/
typedef struct _flexspi_mem_nand_handle
{
    flexspi_port_t port; /*!< Device connected to which port, SS0_A means port A1, SS0_B means port B1*/
    uint16_t busyOffset; /*!< Busy offset, valid value: 0-31, only need for check option kNandReadyCheckOption_RB */
    uint16_t busyBitPolarity; /*!< Busy flag polarity, 0 - busy flag is 1 when flash device is busy, 1 -busy flag is 0
                                   when flash device is busy, only need for check option kNandReadyCheckOption_RB */
    uint32_t eccStatusMask;   /*!< ECC status mask */
    uint32_t eccFailureMask;  /*!< ECC failure mask */
} flexspi_mem_nand_handle_t;
