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

#include "fsl_mpl3115a2.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Read register data by I2C protocol.
 * @param handle I2C instance and transfer configure.
 * @param reg register address.
 * @param val data address need to be saved.
 * @param bytesNumber Number of bytes to be readout.
 */
static status_t MPL_ReadReg(mpl_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber);

/*!
 * @brief Write data to  register by I2C protocol.
 * @param handle I2C instance and transfer configure.
 * @param reg register address.
 * @param val data need to be write to register.
 */
static status_t MPL_WriteReg(mpl_handle_t *handle, uint8_t reg, uint8_t val);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* mpl mode falg for internal used. */
static mpl_mode_t currentMode = kMPL_modePressure;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 *@brief Initialize the MPL device.
 *@param mplHandle The handle for data transfer refer to@_mpl_handle struct.
 *@param mplSetting Setting infomation for MPL device.
 *@return status of MPL transfer.
 */
mpl_status_t MPL_Init(mpl_handle_t *handle, mpl_config_t *config)
{
    assert(handle);
    assert(config);

    mpl_status_t status = kStatus_MPL_Success;

    /* Initialize the I2C access function. */
    handle->I2C_SendFunc = config->I2C_SendFunc;
    handle->I2C_ReceiveFunc = config->I2C_ReceiveFunc;

    /* Goto standby */
    status |= MPL_GotoStandby(handle);
    /* Configure the mode */
    status |= MPL_SetMode(handle, config->mode);
    /* Set over-sampling */
    status |= MPL_SetOversampleRate(handle, config->overSample);
    /* Enable the flags */
    status |= MPL_EnableEventFlags(handle);
    /* goto active state */
    status |= MPL_SetActive(handle);

    if (kStatus_MPL_Success != status)
    {
        return kStatus_MPL_Error;
    }
    return kStatus_MPL_Success;
}

static status_t MPL_ReadReg(mpl_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber)
{
    assert(handle);
    assert(val);

    if (!handle->I2C_ReceiveFunc)
    {
        return kStatus_Fail;
    }

    return handle->I2C_ReceiveFunc(MPL_I2C_ADDRESS, reg, 1, val, bytesNumber);
}

static status_t MPL_WriteReg(mpl_handle_t *handle, uint8_t reg, uint8_t val)
{
    assert(handle);

    if (!handle->I2C_SendFunc)
    {
        return kStatus_Fail;
    }

    return handle->I2C_SendFunc(MPL_I2C_ADDRESS, reg, 1, val);
}

/*
 * Software reset
 */
mpl_status_t MPL_SoftReset(mpl_handle_t *handle)
{
    assert(NULL != handle);
    uint8_t counter = 0U;
    uint8_t regValue = 0U;

    /* Read current sstting in register. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);
    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        /* Set RST bit and write it back. */
        regValue |= (1 << MPL_RST_SHIFT);
        status = MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
        else
        {
            /* By the end of the boot process the RST bit should be de-asserted to 0; check for it. */
            while (1)
            {
                status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);
                if (kStatus_Success != status)
                {
                    return kStatus_MPL_ProtocolError;
                }
                else
                {
                    if (!(regValue & (1 << MPL_RST_SHIFT)))
                    {
                        return kStatus_MPL_Success;
                    }
                    else
                    {
                        /* Error out after ~0.5 s for a read. */
                        if (++counter > 5)
                        {
                            return kStatus_MPL_TimeOut;
                        }
                    }
                }
            }
        }
    }
}

/*
 * Clears then sets the OST bit which causes the sensor to immediately take another reading,
 * necessary to sample faster than 1Hz
 */
mpl_status_t MPL_ToggleOneShot(mpl_handle_t *handle)
{
    uint8_t regValue = 0U;

    /* Read current settings. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);

    /* Clear OST bit and write it back. */
    regValue &= ~(1 << MPL_OST_SHIFT);
    status |= MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
    /* Read again settings, just to be safe. */
    status |= MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);
    /* Set OST bit and write it back. */
    regValue |= (1 << MPL_OST_SHIFT);
    status |= MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        return kStatus_MPL_Success;
    }
}

/*
 * Set the device to be barometer or altimeter.
 */
mpl_status_t MPL_SetMode(mpl_handle_t *handle, mpl_mode_t mode)
{
    uint8_t regValue = 0U;

    /* Read current settings form register. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);

    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }

    else
    {
        regValue = (mode) ? (regValue | (1 << 7)) : (regValue & ~(1 << 7));

        status = MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }

        else
        {
            /* update the static variable. */
            currentMode = mode;
            return kStatus_MPL_Success;
        }
    }
}

/*
 * @brief Put the sensor in stand-by mode
 * this is needed so that we can modify major control registers
 */
mpl_status_t MPL_GotoStandby(mpl_handle_t *handle)
{
    uint8_t regValue = 0U;

    /* Read current settings form register. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);

    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        /* Clear SBYB bit for entering stand-by mode. */
        regValue &= ~(1 << MPL_SBYB_SHIFT);
        status = MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
    }
    return kStatus_MPL_Success;
}

/*
 * Put the sensor in active mode
 */
mpl_status_t MPL_SetActive(mpl_handle_t *handle)
{
    uint8_t regValue = 0U;

    /* Read current settings form register. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);

    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        /* Set SBYB bit for entering active mode. */
        regValue |= (1 << MPL_SBYB_SHIFT);
        status = MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
        else
        {
            return kStatus_MPL_Success;
        }
    }
}

/*
 * Set the over-sample rate
 * datasheet calls for 128, but you can set it from 1 to 128 samples
 * The higher the oversample rate, the greater the time between data samples
 */
mpl_status_t MPL_SetOversampleRate(mpl_handle_t *handle, mpl_over_sample_t sampleRate)
{
    assert(sampleRate <= 7);
    if (sampleRate > 7)
    {
        /* Rate cannot be larger than 7. */
        return kStatus_MPL_ParameterError;
    }

    uint8_t regValue = 0U;

    /* Read current settings form register. */
    status_t status = MPL_ReadReg(handle, MPL_CTRL_REG1, &regValue, 1);

    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        /* Clear out old OS bits. */
        regValue &= ~MPL_OS_MASK;
        /* Mask in new OS bits */
        regValue |= (sampleRate << MPL_OS_SHIFT);

        status = MPL_WriteReg(handle, MPL_CTRL_REG1, regValue);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
        else
        {
            return kStatus_MPL_Success;
        }
    }
}

/*
 * Enables the pressure and temp measurement event flags so that we can test against them
 * this is recommended in datasheet during setup
 */
mpl_status_t MPL_EnableEventFlags(mpl_handle_t *handle)
{
    uint8_t regValue = 0x07U;

    status_t status = MPL_WriteReg(handle, PT_DATA_CFG, regValue);
    if (kStatus_Success != status)
    {
        return kStatus_MPL_ProtocolError;
    }
    else
    {
        return kStatus_MPL_Success;
    }
}

/*
 * Read sensor raw data.
 */
mpl_status_t MPL_ReadRawData(mpl_handle_t *handle, mpl_mode_t mode, int16_t *sensorData)
{
    mpl_status_t status = kStatus_MPL_Success;
    uint8_t regStatus = 0U;
    uint8_t dataReadyFlag = 0U;
    /* Buffer for pressure/altitude and temperature sensor. */
    mpl_data_type_t mplData = {0};
    uint32_t subaddress = 0;
    uint8_t dataSize = 0;
    uint8_t *data = NULL;
    /*
     * Set the new working mode, if given one
     */
    if ((mode > kMPL_modeTemperature) && (mode < kMPL_modeCurrent))
    {
        return kStatus_MPL_ParameterError;
    }
    else if ((kMPL_modeCurrent != mode) && (currentMode != mode))
    {
        /* Goto standby. */
        status |= MPL_GotoStandby(handle);
        /* Set the mode. */
        status |= MPL_SetMode(handle, mode);
        /* Goto active state. */
        status |= MPL_SetActive(handle);
        currentMode = mode;
    }
    if (kStatus_MPL_Success != status)
    {
        return kStatus_MPL_Error;
    }
    else
    {
        /*
         * Prepare for reading data.
         */
        switch (mode)
        {
            /* Pressure and altitude use the same setting. */
            case kMPL_modePressure:
            case kMPL_modeAltitude:
            {
                dataReadyFlag = 1 << MPL_PDR_SHIFT;
                subaddress = (uint32_t)OUT_P_MSB;
                data = (uint8_t *)&mplData.presAltDataMSB;
                dataSize = 3U;
                break;
            }
            case kMPL_modeTemperature:

            {
                dataReadyFlag = 1 << MPL_TDR_SHIFT;
                subaddress = (uint32_t)OUT_T_MSB;
                data = (uint8_t *)&mplData.tempDataMSB;
                dataSize = 2U;
                break;
            }
            default:
                break;
        }

        status_t status = MPL_ReadReg(handle, REG_STATUS, &regStatus, 1);

        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
        /* Check PDR/PTR bit; if it's not set, toggle OST. */
        else if (0 == (regStatus & dataReadyFlag))
        {
            /* Toggle the OST bit, causing the sensor to immediately take another reading. */
            MPL_ToggleOneShot(handle);
        }

        int16_t counter = 0;
        /* Wait for PDR/PTR bit, which indicates that we have new data. */
        while (1)
        {
            /* Delay for some time when wait for the translate complete.
             * This is useful when mode is changed.
             */
            Timer_Delay_ms(MPL_USER_DELAY_MS);
            status = MPL_ReadReg(handle, REG_STATUS, &regStatus, 1);
            if (kStatus_Success != status)
            {
                return kStatus_MPL_ProtocolError;
            }
            else
            {
                if (0 == (regStatus & dataReadyFlag))
                {
                    /* Error out after more than 0.5s for a read op. */
                    if (++counter > 5)
                    {
                        return kStatus_MPL_TimeOut;
                    }
                }
                else
                {
                    /* Continue with the program. */
                    break;
                }
            }
        }

        /*
         * Read sensor data
         */
        status = MPL_ReadReg(handle, subaddress, data, dataSize);
        if (kStatus_Success != status)
        {
            return kStatus_MPL_ProtocolError;
        }
        else
        {
            /*
             * save data
             */
            switch (mode)
            {
                /* Pressure and altitude use the same data.
                 * Pressure value is a Q18.2 right-aligned number in [Pa].
                 */
                case kMPL_modePressure:
                /* Altitude value is a Q16.4 right-aligned number in [m] */
                case kMPL_modeAltitude:
                {
                    /* Save pressure/altitude data. */
                    uint32_t temp32 =
                        ((mplData.presAltDataMSB << 16) | (mplData.presAltDataCSB << 8) | (mplData.presAltDataLSB)) >>
                        4;

                    /* Pack it into the first two integers in the output array. */
                    memcpy((void *)sensorData, (const void *)&temp32, sizeof(temp32));

                    break;
                }

                /* temperature value is a Q8.4 right-aligned number in [C] */
                case kMPL_modeTemperature:
                {
                    *sensorData = (int16_t)((mplData.tempDataMSB << 4) | (mplData.tempDataLSB >> 4));
                    break;
                }
                default:
                    break;
            }
        }
        return kStatus_MPL_Success;
    }
}

mpl_status_t MPL_ReadPressure(mpl_handle_t *handle, void *data)
{
    assert((NULL != handle) && (NULL != data));
    mpl_status_t status = kStatus_MPL_Success;
    float *pressure = data;
    uint32_t rawPressure = 0U;

    /* Read data from device. */
    status = MPL_ReadRawData(handle, kMPL_modePressure, (int16_t *)&rawPressure);
    if (kStatus_MPL_Success != status)
    {
        return kStatus_MPL_Error;
    }

    /* Translate raw data to float data.
     * Pressure value is a Q18.2 right-aligned number in [Pa].
     */
    *pressure = (float)rawPressure / 4;
    return kStatus_MPL_Success;
}

mpl_status_t MPL_ReadTemperature(mpl_handle_t *handle, void *data)
{
    assert((NULL != handle) && (NULL != data));
    mpl_status_t status = kStatus_MPL_Success;
    float *temperature = data;
    uint32_t rawTemperature = 0U;

    /* Read data from device. */
    status = MPL_ReadRawData(handle, kMPL_modeTemperature, (int16_t *)&rawTemperature);
    if (kStatus_MPL_Success != status)
    {
        return kStatus_MPL_Error;
    }

    /* Translate raw data to float data.
     * Temperature value is a Q8.4 right-aligned number in [C].
     */
    *temperature = (float)rawTemperature / 16;
    return kStatus_MPL_Success;
}
