/*
 * The Clear BSD License
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include "fsl_lpspi_shared_irqs.h"
#include "lpspi/hal/fsl_lpspi_hal.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <stdlib.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

lpspi_shared_irq_config_t g_lpspiSharedIrqConfig[FSL_FEATURE_SOC_LPSPI_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static void lpspi_handle_shared_irq(uint32_t instance);

void LPSPI0_IRQHandler(void);
void LPSPI1_IRQHandler(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Pass IRQ control to either the master or slave driver.
//!
//! The address of the IRQ handlers are checked to make sure they are non-zero before
//! they are called. If the IRQ handler's address is zero, it means that driver was
//! not present in the link (because the IRQ handlers are marked as weak). This would
//! actually be a program error, because it means the master/slave config for the IRQ
//! was set incorrectly.
static void lpspi_handle_shared_irq(uint32_t instance)
{
    if (g_lpspiSharedIrqConfig[instance].isMaster)
    {
        // Master mode.
        if (&lpspi_master_irq_handler)
        {
            lpspi_master_irq_handler(instance);
        }
    }
    else
    {
        // Slave mode.
        if (&lpspi_slave_irq_handler)
        {
            lpspi_slave_irq_handler(instance);
        }
    }
}

//! @brief Implementation of LPSPI0 handler named in startup code.
//!
//! Passes instance to generic LPSPI IRQ handler.
void LPSPI0_IRQHandler(void)
{
    lpspi_handle_shared_irq(0);
}

//! @brief Implementation of LPSPI1 handler named in startup code.
//!
//! Passes instance to generic LPSPI IRQ handler.
void LPSPI1_IRQHandler(void)
{
    lpspi_handle_shared_irq(1);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
