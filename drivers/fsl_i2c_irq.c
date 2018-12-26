/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "utilities/fsl_assert.h"
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup i2c_irq
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
void I2C_DRV_MasterIRQHandler(uint32_t instance);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Implementation of I2C0 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C0_IRQHandler(void)
{
    I2C_DRV_MasterIRQHandler(I2C0_IDX);
}
#if !defined(KV46F15_SERIES)
/*!
 * @brief Implementation of I2C1 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C1_IRQHandler(void)
{
    I2C_DRV_MasterIRQHandler(I2C1_IDX);
}
#endif

#if defined(K64F12_SERIES)
/*!
 * @brief Implementation of I2C2 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C2_IRQHandler(void)
{
    I2C_DRV_MasterIRQHandler(I2C2_IDX);
}
#endif /* K64F12_SERIES */

/*! @} */

/*******************************************************************************
 * EOF
 ******************************************************************************/
