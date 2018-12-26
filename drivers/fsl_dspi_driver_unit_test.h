/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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
#ifndef __FSL_DSPI_DRIVER_UNIT_TEST_H__
#define __FSL_DSPI_DRIVER_UNIT_TEST_H__

#include <stdint.h>
#include "dspi/hal/fsl_dspi_hal.h"
#include "dspi/hal/fsl_dspi_features.h"
#include "bootloader_common.h"
#include "dspi/fsl_dspi_master_driver.h"
#include "dspi/fsl_dspi_slave_driver.h"

/*! @addtogroup dspi_driver_unit_test*/
/*! @{*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name DSPI Driver Unit test*/
/*@{*/

/*!
 * @brief Tests to ensure the function sets the features and bit that they are supposed to
 */
uint32_t dspi_test_master_functions(void);

/*!
 * @brief Tests the dspi instances SOUT-to-SIN self loopback test (used before the slave driver
 *        is ready, no slave driver needed).
 */
uint32_t dspi_self_loopback_test(void);

/*!
 * @brief Tests two dspi instances, one configured as master, the other as a slave, and transfers
 *        data from master-to-slave. Is it required for a hard wired connection between the
 *        two instances.
 */
uint32_t dspi_sameboard_loopback_test(uint32_t masterInstance, uint32_t slaveInstance);

/*!
 * @brief Function to setup dspi pins
 *
 *
 */
void dspi_test_pinmux_setup(uint32_t dspi_instance);

/*@}  */

/*! @}*/

#endif /* __FSL_DSPI_DRIVER_UNIT_TEST_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/
