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

#ifndef _FSL_MPL3115A2_H_
#define _FSL_MPL3115A2_H_

#include "fsl_timer.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MPL_I2C_ADDRESS (0x60)
/* For users to defined the delay time when read data from sensor. */
#define MPL_USER_DELAY_MS (100)

#define MPL_OS_SHIFT (3)
#define MPL_OS_MASK (0x7 << MPL_OS_SHIFT)

#define REG_STATUS (0x00)
#define OUT_P_MSB (0x01)
#define OUT_P_CSB (0x02)
#define OUT_P_LSB (0x03)
#define OUT_T_MSB (0x04)
#define OUT_T_LSB (0x05)
#define DR_STATUS (0x06)
#define OUT_P_DELTA_MSB (0x07)
#define OUT_P_DELTA_CSB (0x08)
#define OUT_P_DELTA_LSB (0x09)
#define OUT_T_DELTA_MSB (0x0A)
#define OUT_T_DELTA_LSB (0x0B)
#define WHO_AM_I (0x0C)
#define F_STATUS (0x0D)
#define F_DATA (0x0E)
#define F_SETUP (0x0F)
#define TIME_DLY (0x10)
#define SYSMOD (0x11)
#define INT_SOURCE (0x12)
#define PT_DATA_CFG (0x13)
#define BAR_IN_MSB (0x14)
#define BAR_IN_LSB (0x15)
#define P_TGT_MSB (0x16)
#define P_TGT_LSB (0x17)
#define T_TGT (0x18)
#define P_WND_MSB (0x19)
#define P_WND_LSB (0x1A)
#define T_WND (0x1B)
#define P_MIN_MSB (0x1C)
#define P_MIN_CSB (0x1D)
#define P_MIN_LSB (0x1E)
#define T_MIN_MSB (0x1F)
#define T_MIN_LSB (0x20)
#define P_MAX_MSB (0x21)
#define P_MAX_CSB (0x22)
#define P_MAX_LSB (0x23)
#define T_MAX_MSB (0x24)
#define T_MAX_LSB (0x25)
#define MPL_CTRL_REG1 (0x26)
#define MPL_CTRL_REG2 (0x27)
#define MPL_CTRL_REG3 (0x28)
#define MPL_CTRL_REG4 (0x29)
#define MPL_CTRL_REG5 (0x2A)
#define OFF_P (0x2B)
#define OFF_T (0x2C)
#define OFF_H (0x2D)

#define MPL_TDR_SHIFT (1)
#define MPL_PDR_SHIFT (2)
#define MPL_PTDR_SHIFT (3)
#define MPL_SBYB_SHIFT (0)
#define MPL_OST_SHIFT (1)
#define MPL_RST_SHIFT (2)

typedef enum _mpl_status
{
    kStatus_MPL_Success = 0,
    kStatus_MPL_Error = 0x01,
    kStatus_MPL_TimeOut = 0x02,
    kStatus_MPL_ParameterError = 0x04,
    kStatus_MPL_ProtocolError = 0x08,
    kStatus_MPL_InitError = 0x10
} mpl_status_t;

typedef enum _mpl_over_sample
{
    kMPL_overSample0,
    kMPL_overSample1,
    kMPL_overSample2,
    kMPL_overSample3,
    kMPL_overSample4,
    kMPL_overSample5,
    kMPL_overSample6,
    kMPL_overSample7,
} mpl_over_sample_t;

typedef enum _mpl_mode
{
    kMPL_modePressure = 0,
    kMPL_modeAltitude = 1,
    kMPL_modeTemperature = 2,
    kMPL_modeCurrent = 0xFF
} mpl_mode_t;

/*! @brief Define I2C access function. */
typedef status_t (*I2C_SendFunc_t)(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize,
                         uint32_t txBuff);
typedef status_t (*I2C_ReceiveFunc_t)(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize,
                            uint8_t *rxBuff, uint8_t rxBuffSize);

typedef struct _mpl_handle
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
} mpl_handle_t;

typedef struct _mpl_config
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
    mpl_mode_t mode;              /* device mode, altimeter or barometer. */
    mpl_over_sample_t overSample; /* oversampling ratio */
} mpl_config_t;

typedef struct _mpl_data_type
{
    /* Pressure/altitude data. */
    uint8_t presAltDataMSB;
    uint8_t presAltDataCSB;
    uint8_t presAltDataLSB;
    /* Temperature data. */
    uint8_t tempDataMSB;
    uint8_t tempDataLSB;
} mpl_data_type_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */

/*!
 * @brief Initialize the MPL device.
 * @param handle I2C instance and tenasfer config for transfer.
 * @param config mode and over sample for MPL device.
 * @return mpl status flag.
 */
mpl_status_t MPL_Init(mpl_handle_t *handle, mpl_config_t *config);

/*!
 * @brief MPL Software reset
 * The reset mechanism can be enabled in standby and active mode
 * When this bit is enabled, the reset mechanism resets all functional block registers and
 * loads the respective internal registers with default values, if the system was already
 * in standby mode, the reboot process will immediately begin; else if the system was in
 * active mode, the boot mechanism will automatically transition the system from active mode
 * to standby mode, and only then can the reboot process begin.
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @return mpl status flag.
 */
mpl_status_t MPL_SoftReset(mpl_handle_t *handle);

/*!
 * @brief Toggle one shot for read data immediately.
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @return mpl status flag.
 */
mpl_status_t MPL_ToggleOneShot(mpl_handle_t *handle);

/*!
 * @brief Set the device to be barometer or altimeter
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @param mode mode for barometer or altimeter.
 * @return mpl status flag.
 */
mpl_status_t MPL_SetMode(mpl_handle_t *handle, mpl_mode_t mode);

/*!
 * @brief Put the sensor in stand-by mode
 * this is needed so that we can modify the major control registers
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @return mpl status flag.
 */
mpl_status_t MPL_GotoStandby(mpl_handle_t *handle);

/*!
 * @brief Put the sensor in active mode
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @return mpl status flag.
 */
mpl_status_t MPL_SetActive(mpl_handle_t *handle);

/*!
 * @brief Set the over-sample rate
 * datasheet calls for 128, but you can set it from 1 to 128 samples
 * The higher the oversample rate, the greater the time between data samples
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @param sampleRate sample rate for MPL device.
 * @return mpl status flag.
 */
mpl_status_t MPL_SetOversampleRate(mpl_handle_t *handle, mpl_over_sample_t sampleRate);

/*!
 * @brief Enables the pressure and temp measurement event flags so that we can test against them
 * this is recommended in datasheet during setup
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @return mpl status flag.
 */
mpl_status_t MPL_EnableEventFlags(mpl_handle_t *handle);

/*
 * @brief Read sensor raw data
 * This data is raw data, and need to be translated by users.
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @param mode mode for barometer or altimeter.
 * @param sensorData pointer of sensor data address.
 * @return mpl status flag.
 */
mpl_status_t MPL_ReadRawData(mpl_handle_t *handle, mpl_mode_t mode, int16_t *sensorData);

/*
 * @brief Read pressure data[Pa].
 * API will read pressure data directly, and the data format is float.
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @param mode mode for barometer or altimeter.
 * @param data pointer of sensor data address(float).
 * @return mpl status flag.
 */
mpl_status_t MPL_ReadPressure(mpl_handle_t *handle, void *data);

/*
 * @brief Read Temperature data[C].
 * API will read temperature data directly, and the data format is float.
 *
 * @param handle I2C instance and tenasfer config for transfer.
 * @param mode mode for barometer or altimeter or temperature.
 * @param data pointer of sensor data address(float).
 * @return mpl status flag.
 */
mpl_status_t MPL_ReadTemperature(mpl_handle_t *handle, void *data);

#if defined(__cplusplus)
}
#endif /* _cplusplus */

#endif /* _FSL_MPL3115A2_H_ */
