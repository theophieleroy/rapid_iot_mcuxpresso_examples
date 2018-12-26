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

#ifndef __FSL_NOR_FLASH_H__
#define __FSL_NOR_FLASH_H__

#include "fsl_common.h"

/*!
 * @addtogroup nor flash component
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief NOR Flash Config block structure */
typedef struct _nor_config
{
    void *memControlConfig; /*!< memory controller configuration, should be assigned to specific controller configuration
                               structure pointer.*/
    void *driverBaseAddr;   /*! Driver Base address. */
} nor_config_t;

/*!@brief NOR Flash handle info*/
typedef struct _nor_handle
{
  /*------------Common parameters used for normal NOR flash controller operation ----------*/    
    void *driverBaseAddr;       /*! Driver Base address. */
    uint32_t bytesInPageSize;   /*!< Page size in byte of Serial NOR */
    uint32_t bytesInSectorSize;   /*!< Minimun Sector size in byte supported by Serial NOR */ 
    uint32_t bytesInMemorySize;   /*!< Memory size in byte of Serial NOR */  
   /*------------Specific parameters used for specific NOR flash controller ----------*/  
    void *deviceSpecific;                    /*!< Device specific control parameter */
} nor_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
  * @name NOR FLASH Driver
  * @{
  */

/*!
 * @brief Initialize NOR FLASH devices.
 *
 *  This function initialize NOR Flash controller and NOR Flash.
 *
 * @param config    NOR flash configuration.
 *        The "memControlConfig" and "driverBaseAddr" are controller specific structure.
 *        please set those two parameter with your Nand controller configuration structure type pointer.
 *        such as for SEMC:
 *
 *        spifi_mem_nor_config_t spifiNorconfig =
 *        {
 *            .....
 *        }
 *        nor_config_t config =
 *        {
 *            .memControlConfig = (void *)&spifiNorconfig;
 *            .driverBaseAddr   = (void *)SPIFI0;
 *        }
 * @param handle    The NOR Flash handler.
 * @retval execution status
 */
status_t Nor_Flash_Init(nor_config_t *config, nor_handle_t *handle);

/*!
 * @brief Read page data from NOR Flash.
 *
 * @param handle    The NOR Flash handler.
 * @param address  NOR flash start address to read data from.
 * @param buffer  NOR flash buffer to read data to.
 * @param length  NOR flash read length.
 * @retval execution status
 */
status_t Nor_Flash_Read(nor_handle_t *handle, uint32_t address, uint8_t *buffer, uint32_t length);

/*!
 * @brief Program page data to NOR Flash.
 *
 * @param handle    The NOR Flash handler.
 * @param address  The address to be programed.
 * @param buffer  The buffer to be programed to the page.
 * @retval execution status
 */
status_t Nor_Flash_Page_Program(nor_handle_t *handle, uint32_t address, uint8_t *buffer);

/*!
 * @brief Erase block NOR Flash int the unit of the supported minimum sector size.
 *
 * @param handle    The NOR Flash handler.
 * @param address   The start address to be erased.
 * @param size_Byte Erase block size. 
 * @retval execution status
 */
status_t Nor_Flash_Erase_Block(nor_handle_t *handle, uint32_t address, uint32_t size_Byte);

/*!
 * @brief Erase Chip NOR Flash .
 *
 * @param handle    The NOR Flash handler.
 * @retval execution status
 */
status_t Nor_Flash_Erase_Chip(nor_handle_t *handle);

/* @} */

#ifdef __cplusplus
}
#endif

#endif /* __FSL_NOR_FLASH_H__ */
