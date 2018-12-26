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
#if !defined(__FSL_DSPI_IRQS_H__)
#define __FSL_DSPI_IRQS_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*!
 * @addtogroup dspi_shared_irq
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Configuration of the DSPI IRQs shared by master and slave drivers.
 */
typedef struct DspiSharedIrqConfig
{
    bool isMaster; /*!< Whether the IRQ is used by the master mode driver.*/
    void *state;   /*!< Void pointer to driver state information */
} dspi_shared_irq_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Contains global IRQ configuration information for the DSPI drivers.*/
extern dspi_shared_irq_config_t g_dspiSharedIrqConfig[SPI_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Set the shared IRQ state structure
 *
 * This function sets whether the master or slave driver IRQ handler will be invoked and
 * to set the pointer to the driver run-time state structure associated with the desired module
 * instance number.  This is not a public API.
 */
static inline void dspi_set_shared_irq_state(uint32_t instance, void *state, bool isMaster)
{
    g_dspiSharedIrqConfig[instance].isMaster = isMaster;
    g_dspiSharedIrqConfig[instance].state = state;
}

/* Weak extern for the DSPI master driver's interrupt handler.*/
#pragma weak dspi_master_irq_handler
void dspi_master_irq_handler(void *state);

/* Weak extern for the SPI slave driver's interrupt handler.*/
#if !defined(__CC_ARM)
#pragma weak dspi_slave_irq_handler
#endif
void dspi_slave_irq_handler(void *state);

/*! @} */

#endif /* __FSL_DSPI_IRQS_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/
