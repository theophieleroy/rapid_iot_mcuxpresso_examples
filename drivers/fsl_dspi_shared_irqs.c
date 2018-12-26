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

#include "fsl_dspi_shared_irqs.h"
#include "bootloader_common.h"

#if BL_CONFIG_DSPI

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*!
 * @brief Declare instantiations of the shared irq configuration. This is used in the irq handler
 * to keep track of the driver type (master or slave) and the driver run-time state.
 */
dspi_shared_irq_config_t g_dspiSharedIrqConfig[SPI_INSTANCE_COUNT];

/*!
 * @brief Table to save DSPI IRQ enum numbers defined in CMSIS files.
 *
 * This is used by DSPI master and slave init functions to enable or disable DSPI interrupts.
 * This table is indexed by the module instance number and returns DSPI IRQ numbers.
 */
/*
#if defined (K64F12_SERIES) || defined (K70F12_SERIES)
IRQn_Type dspi_irq_ids[SPI_INSTANCE_COUNT] = {SPI0_IRQn, SPI1_IRQn, SPI2_IRQn};
#elif defined (K22F51212_SERIES)
IRQn_Type dspi_irq_ids[SPI_INSTANCE_COUNT] = {SPI0_IRQn, SPI1_IRQn};
#endif
*/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void dspi_handle_shared_irq(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief The function dspi_handle_shared_irq passes IRQ control to either the master or
 * slave driver.
 *
 * The address of the IRQ handlers are checked to make sure they are non-zero before
 * they are called. If the IRQ handler's address is zero, it means that driver was
 * not present in the link (because the IRQ handlers are marked as weak). This would
 * actually be a program error, because it means the master/slave config for the IRQ
 * was set incorrectly.
 */
static void dspi_handle_shared_irq(uint32_t instance)
{
    if (g_dspiSharedIrqConfig[instance].isMaster)
    {
        /* Master mode.*/
        if (&dspi_master_irq_handler)
        {
            dspi_master_irq_handler(g_dspiSharedIrqConfig[instance].state);
        }
    }
    else
    {
        /* Slave mode.*/
        if (&dspi_slave_irq_handler)
        {
            dspi_slave_irq_handler(g_dspiSharedIrqConfig[instance].state);
        }
    }
}

/*!
 * @brief This function is the implementation of SPI0 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI0_IRQHandler(void)
{
    dspi_handle_shared_irq(SPI0_IDX);
}

#if (SPI_INSTANCE_COUNT > 1U)
/*!
 * @brief This function is the implementation of SPI1 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI1_IRQHandler(void)
{
    dspi_handle_shared_irq(SPI1_IDX);
}
#endif //(SPI_INSTANCE_COUNT > 1U)

#if (SPI_INSTANCE_COUNT > 2U)
/*!
 * @brief This function is the implementation of SPI2 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI2_IRQHandler(void)
{
    dspi_handle_shared_irq(SPI2_IDX);
}
#endif //(SPI_INSTANCE_COUNT > 2U)

#endif // BL_CONFIG_DSPI

/*******************************************************************************
 * EOF
 ******************************************************************************/
