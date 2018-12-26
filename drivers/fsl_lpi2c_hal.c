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

#include "fsl_lpi2c_hal.h"
#include "bootloader_common.h" /* For ARRAY_SIZE*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : lpi2c_hal_reset_slave
 * Description   : Initialize LPI2C slave peripheral to reset state.
 *
 *END**************************************************************************/
void lpi2c_hal_reset_slave(LPI2C_Type *baseAddr)
{
    // Soft Reset can reset all registers, except the Slave Control Register,
    //  So we need to clear SCR manually.
    LPI2C_WR_SCR_RST(baseAddr, 0x1U);
    LPI2C_WR_SCR(baseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lpi2c_hal_set_slave_7bit_address0
 * Description   : Sets the primary 7-bit slave address 0.
 *
 *END**************************************************************************/
void lpi2c_hal_set_slave_7bit_address0(LPI2C_Type *baseAddr, uint8_t address)
{
    // Set 7-bit slave address 0.
    LPI2C_WR_SAMR_ADDR0(baseAddr, address);

    // Set the address configuration, selecting address match 0 (7-bit).
    LPI2C_WR_SCFGR1_ADDRCFG(baseAddr, 0);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
