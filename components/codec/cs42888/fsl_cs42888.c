/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#include "fsl_cs42888.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void CS42888_Init(cs42888_handle_t *handle, cs42888_config_t *config)
{
    uint32_t i = 0;

    /* Set CS42888 I2C address */
    handle->xfer.slaveAddress = CS42888_I2C_ADDR;

    /*PDN and PDN for each ADC and DAC are set as 1*/
    CS42888_WriteReg(handle, CS42888_POWER_CONTROL, 0x7F);

    /* NULL pointer means default setting. */
    if (config == NULL)
    {
        /*Register Configurations*/
        /* set as slave */
        CS42888_SetFuncMode(handle, kCS42888_ModeSlave);
        CS42888_SetProtocol(handle, kCS42888_BusLeftJustified, 16);
    }
    else
    {
        CS42888_ModifyReg(handle, CS42888_FUNCTIONAL_MODE, CS42888_FUNCTIONAL_MODE_DAC_FM_MASK,
                          CS42888_FUNCTIONAL_MODE_DAC_FM(config->DACMode));
        CS42888_ModifyReg(handle, CS42888_FUNCTIONAL_MODE, CS42888_FUNCTIONAL_MODE_ADC_FM_MASK,
                          CS42888_FUNCTIONAL_MODE_ADC_FM(config->ADCMode));
        CS42888_SetProtocol(handle, config->bus, 16U);
    }
    /*Mute all DACs*/
    CS42888_WriteReg(handle, CS42888_CHANNEL_MUTE, 0xFF);

    /*Set PDN as 0*/
    CS42888_WriteReg(handle, CS42888_POWER_CONTROL, 0x0);
    CS42888_WriteReg(handle, CS42888_TRANSITION_CONTROL, 0x10);

    /* Configure the codec AIN volume to 8db */
    for (i = 0; i < 8; i++)
    {
        CS42888_SetAINVolume(handle, i, 16);
    }

    /*Delay and unmute*/
    CS42888_WriteReg(handle, CS42888_CHANNEL_MUTE, 0x0);
}

void CS42888_SetFuncMode(cs42888_handle_t *handle, cs42888_func_mode mode)
{
    CS42888_ModifyReg(handle, CS42888_FUNCTIONAL_MODE, CS42888_FUNCTIONAL_MODE_DAC_FM_MASK,
                      CS42888_FUNCTIONAL_MODE_DAC_FM(mode));
    CS42888_ModifyReg(handle, CS42888_FUNCTIONAL_MODE, CS42888_FUNCTIONAL_MODE_ADC_FM_MASK,
                      CS42888_FUNCTIONAL_MODE_ADC_FM(mode));
}

void CS42888_Deinit(cs42888_handle_t *handle)
{
    /* Disable all modules making CS42888 enter a low power mode */
    CS42888_WriteReg(handle, CS42888_FUNCTIONAL_MODE, 0U);
}

status_t CS42888_SetProtocol(cs42888_handle_t *handle, cs42888_bus_t protocol, uint32_t bitWidth)
{
    status_t ret = kStatus_Success;

    if ((protocol == kCS42888_BusLeftJustified) && (bitWidth <= 24))
    {
        CS42888_ModifyReg(handle, CS42888_INTERFACE_FORMATS, 0x3F, 0);
    }
    else if ((protocol == kCS42888_BusI2S) && (bitWidth <= 24))
        CS42888_ModifyReg(handle, CS42888_INTERFACE_FORMATS, 0x3F, 0x9);
    else if ((protocol == kCS42888_BusRightJustified) && (bitWidth == 24))
        CS42888_ModifyReg(handle, CS42888_INTERFACE_FORMATS, 0x3F, 0x12);
    else if ((protocol == kCS42888_BusRightJustified) && (bitWidth == 16))
        CS42888_ModifyReg(handle, CS42888_INTERFACE_FORMATS, 0x3F, 0x1B);
    else
        ret = kStatus_Fail;

    return ret;
}

status_t CS42888_ConfigDataFormat(cs42888_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint8_t bits)
{
    status_t retval = kStatus_Success;
    uint8_t val = 0;

    CS42888_ReadReg(handle, CS42888_FUNCTIONAL_MODE, &val);
    val &= ~0x7U;
    if (((val & 0xC0) == 0xC0) || ((val & 0x30) == 0x30))
    {
        retval = kStatus_Success;
    }
    else
    {
        /* Set the mclk */
        if (mclk < 12800000)
        {
            val &= 0xF0U;
        }
        else if (mclk < 25600000)
        {
            val |= 0x4U;
        }
        else if (mclk < 51200000)
        {
            val |= 0x8U;
        }
        else
        {
            return kStatus_Fail;
        }
        retval = CS42888_WriteReg(handle, CS42888_FUNCTIONAL_MODE, val);
    }
    return retval;
}

status_t CS42888_SetModule(cs42888_handle_t *handle, cs42888_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    uint8_t val = 0;

    /* Read Power control register value */
    CS42888_ReadReg(handle, CS42888_POWER_CONTROL, &val);
    if (isEnabled)
    {
        val |= module;
    }
    else
    {
        val &= ~module;
    }
    CS42888_WriteReg(handle, CS42888_POWER_CONTROL, val);
    return ret;
}

status_t CS42888_SetAOUTVolume(cs42888_handle_t *handle, uint8_t channel, uint8_t volume)
{
    status_t ret = kStatus_Success;
    uint8_t reg = CS42888_VOL_CONTROL_AOUT1 + (channel - 1U);

    if ((channel < 1) || (channel > 8))
    {
        ret = kStatus_Fail;
    }
    else
    {
        ret = CS42888_WriteReg(handle, reg, volume);
    }
    return ret;
}

status_t CS42888_SetAINVolume(cs42888_handle_t *handle, uint8_t channel, uint8_t volume)
{
    status_t ret = kStatus_Success;
    uint8_t reg = CS42888_VOL_CONTROL_AIN1 + (channel - 1U);

    if ((channel < 1) || (channel > 4))
    {
        ret = kStatus_Fail;
    }
    else
    {
        ret = CS42888_WriteReg(handle, reg, volume);
    }
    return ret;
}

uint8_t CS42888_GetAOUTVolume(cs42888_handle_t *handle, uint8_t channel)
{
    uint8_t val = 0;
    uint8_t reg = CS42888_VOL_CONTROL_AOUT1 + (channel - 1U);
    if ((channel < 1) || (channel > 8))
    {
        val = 0;
    }
    else
    {
        CS42888_ReadReg(handle, reg, &val);
    }
    return val;
}

uint8_t CS42888_GetAINVolume(cs42888_handle_t *handle, uint8_t channel)
{
    uint8_t val = 0;
    uint8_t reg = CS42888_VOL_CONTROL_AIN1 + (channel - 1U);
    if ((channel < 1) || (channel > 4))
    {
        val = 0;
    }
    else
    {
        CS42888_ReadReg(handle, reg, &val);
    }
    return val;
}

status_t CS42888_SetMute(cs42888_handle_t *handle, uint8_t channelMask)
{
    status_t ret = kStatus_Success;

    ret = CS42888_WriteReg(handle, CS42888_CHANNEL_MUTE, channelMask);
    return ret;
}

status_t CS42888_WriteReg(cs42888_handle_t *handle, uint8_t reg, uint8_t val)
{
    status_t retval = kStatus_Success;

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    uint8_t data[2] = {0};
    data[0] = reg;
    data[1] = val;
    retval = LPI2C_MasterStart(handle->base, CS42888_I2C_ADDR, kLPI2C_Write);
    if (retval != kStatus_Success)
    {
        retval = kStatus_Success;
    }
    retval = LPI2C_MasterSend(handle->base, data, 2);
    if (retval != kStatus_Success)
    {
        retval = kStatus_Success;
    }
    retval = LPI2C_MasterStop(handle->base);
    if (retval != kStatus_Success)
    {
        retval = kStatus_Success;
    }
#else
    /* Set I2C xfer structure */
    handle->xfer.direction = kI2C_Write;
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = &val;
    handle->xfer.dataSize = 1U;

    retval = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return retval;
}

status_t CS42888_ReadReg(cs42888_handle_t *handle, uint8_t reg, uint8_t *val)
{
    status_t retval = kStatus_Success;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    retval = LPI2C_MasterStart(handle->base, CS42888_I2C_ADDR, kLPI2C_Write);
    retval = LPI2C_MasterSend(handle->base, &reg, 1);
    retval = LPI2C_MasterStart(handle->base, CS42888_I2C_ADDR, kLPI2C_Read);
    retval = LPI2C_MasterReceive(handle->base, val, 1);
    retval = LPI2C_MasterStop(handle->base);
#else
    /* Set I2C xfer structure */
    handle->xfer.direction = kI2C_Read;
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = val;
    handle->xfer.dataSize = 1U;

    retval = I2C_MasterTransferBlocking(handle->base, &handle->xfer);
#endif

    return retval;
}

status_t CS42888_ModifyReg(cs42888_handle_t *handle, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t retval = 0;
    uint8_t reg_val = 0;
    retval = CS42888_ReadReg(handle, reg, &reg_val);
    reg_val &= (uint8_t)~mask;
    reg_val |= val;
    retval = CS42888_WriteReg(handle, reg, reg_val);
    return retval;
}
