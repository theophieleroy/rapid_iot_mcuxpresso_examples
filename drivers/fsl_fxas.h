/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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
#ifndef _FSL_FXAS_H_
#define _FSL_FXAS_H_

#include "fsl_common.h"

#define GYRO_I2C_ADDRESS   0x20
#define GYRO_FIFO_SIZE      32  /* FXAS21002 have 32 element FIFO */

/*
 *  STATUS Register
 */
#define STATUS_00_REG 0x00

/*
 *  XYZ Data Registers
 */
#define OUT_X_MSB_REG 0x01
#define OUT_X_LSB_REG 0x02
#define OUT_Y_MSB_REG 0x03
#define OUT_Y_LSB_REG 0x04
#define OUT_Z_MSB_REG 0x05
#define OUT_Z_LSB_REG 0x06

/*
 *  DR Status Register
 */
#define DR_STATUS_REG 0x07

/*
 *  DR Status Register
 */
#define F_STATUS_REG 0x08
#define F_CNT_MASK 0x3F
/*
 *  F_SETUP FIFO Setup Register
 */
#define F_SETUP_REG 0x09

#define F_MODE1_MASK 0x80
#define F_MODE0_MASK 0x40
#define F_WMRK5_MASK 0x20
#define F_WMRK4_MASK 0x10
#define F_WMRK3_MASK 0x08
#define F_WMRK2_MASK 0x04
#define F_WMRK1_MASK 0x02
#define F_WMRK0_MASK 0x01
#define F_MODE_MASK 0xC0
#define F_WMRK_MASK 0x3F
#define F_MODE_SHIFT 6

#define F_MODE_DISABLED 0x00

/*
 *  F_EVENT Register
 */
#define F_EVENT_REG 0x0A

/*
 *  INT_SOURCE System Interrupt Status Register
 */
#define INT_SOURCE_REG 0x0B

#define SRC_BOOTEND_MASK 0x08
#define SRC_FIFO_MASK 0x04
#define SRC_RT_MASK 0x02
#define SRC_DRDY_MASK 0x01

/*
 *  WHO_AM_I Device ID Register
 */
#define WHO_AM_I_REG 0x0C

/* Content */
#define kFXAS_WHO_AM_I_Device_ID 0xD7

/*
 *  CTRL_REG0 Register
 */
#define CTRL_REG0 0x0D

/* Rate Threshold configure Register */
#define RT_CFG_REG 0x0E

#define RT_XTEFE_MASK 0x01
#define RT_YTEFE_MASK 0x02
#define RT_ZTEFE_MASK 0x04
#define RT_ELE_MASK 0x08
#define RT_ALLAXES_MASK (RT_XTEFE_MASK | RT_YTEFE_MASK | RT_ZTEFE_MASK)

/* Rate threshold event source Register */
#define RT_SRC_REG 0x0F

#define RT_EA_MASK 0x40
#define RT_ZRT_MASK 0x20
#define RT_ZRT_POL_MASK 0x10
#define RT_YRT_MASK 0x08
#define RT_YRT_POL_MASK 0x04
#define RT_XRT_MASK 0x02
#define RT_XRT_POL_MASK 0x01

/*
 *  RT_THS Register
 */
#define RT_THS_REG 0x10


/*
 *  RT_COUNT Register
 */
#define RT_COUNT_REG 0x11

/*
 *  TEMP Register
 */
#define RT_TEMP_REG 0x12

/* CTRL_REG1 System Control 1 Register */
#define CTRL_REG1 0x13

#define RST_MASK 0x40
#define ST_MASK 0x20
#define DR2_MASK 0x10
#define DR1_MASK 0x08
#define DR0_MASK 0x04
#define OP_MASK 0x03
#define ACTIVE_MASK 0x02
#define DR_MASK 0x38

#define DATA_RATE_OFFSET (2)

#define ACTIVE (ACTIVE_MASK)
#define STANDBY 0x00

/* CTRL_REG2 System Control 2 Register */
#define CTRL_REG2 0x14

#define INT_CFG_FIFO_MASK 0x80
#define INT_EN_FIFO_MASK 0x40
#define INT_CFG_RT_MASK 0x20
#define INT_EN_RT_MASK 0x10
#define INT_CFG_DRDY_MASK 0x08
#define INT_EN_DRDY_MASK 0x04
#define IPOL_MASK 0x02
#define PP_OD_MASK 0x01

/* CTRL_REG3 System Control 3 Register */
#define CTRL_REG3 0x15

#define WRAPTDONE_MASK 0x08
#define EXTCTRLEN_MASK 0x04
#define FS_DOUBLE_MASK 0x01

/*! @brief Full-scale range definition.*/
typedef enum fxas_gfsr
{
    kFXAS_Gfsr_2000DPS,
    kFXAS_Gfsr_1000DPS,
    kFXAS_Gfsr_500DPS,
    kFXAS_Gfsr_250DPS
} fxas_gfsr_t;

typedef enum fxas_odr
{
    kFXAS_Godr_800Hz,
    kFXAS_Godr_400Hz,
    kFXAS_Godr_200Hz,
    kFXAS_Godr_100Hz,
    kFXAS_Godr_50Hz,
    kFXAS_Godr_25Hz,
    kFXAS_Godr_12_5Hz,
} fxas_odr_t;

typedef enum fxas_fifo
{
    kFXAS_FIFO_Disabled,
    kFXAS_FIFO_CircularMode,
    kFXAS_FIFO_StopMode
} fxas_fifo_t;

typedef struct fxas_data
{
    uint8_t gyroXMSB;
    uint8_t gyroXLSB;
    uint8_t gyroYMSB;
    uint8_t gyroYLSB;
    uint8_t gyroZMSB;
    uint8_t gyroZLSB;

} fxas_data_t;

/*! @brief Define I2C callback function. */
typedef status_t (*I2C_SendFunc_t)(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize,
                         uint32_t txBuff);
typedef status_t (*I2C_ReceiveFunc_t)(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize,
                            uint8_t *rxBuff, uint8_t rxBuffSize);

typedef struct fxas_config
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
    fxas_gfsr_t fsrdps;
    fxas_odr_t odr;
    fxas_fifo_t fifo;
} fxas_config_t;

/*! @brief fxas21002cq configure definition. This structure should be global.*/
typedef struct _fxas_handle
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
} fxas_handle_t;

typedef struct _fxas21002cq_data
{
    uint8_t accelXMSB;
    uint8_t accelXLSB;
    uint8_t accelYMSB;
    uint8_t accelYLSB;
    uint8_t accelZMSB;
    uint8_t accelZLSB;
} fxas21002cq_data_t;

/*!
 * @addtogroup fxos_common
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Verify and initialize fxas_handleice: Hybrid mode with ODR=50Hz, Mag OSR=32, Acc OSR=Normal.
 *
 * @param fxas_handle The pointer to accel driver handle.
 * @param configure The configuration structure.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXAS_Init(fxas_handle_t *fxas_handle, fxas_config_t *configure);

/*!
 * @brief Read data from sensors, assumes hyb_autoinc_mode is set in M_CTRL_REG2
 *
 * @param fxas_handle The pointer to accel driver handle.
 * @param sensorData The pointer to the buffer to hold sensor data
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXAS_ReadSensorData(fxas_handle_t *fxas_handle, fxas_data_t *sensorData);

/*!
 * @brief Write value to register of sensor.
 *
 * @param handle The pointer to fxas21002cq driver handle.
 * @param reg Register address.
 * @param val Data want to write.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXAS_WriteReg(fxas_handle_t *handle, uint8_t reg, uint8_t val);

/*!
 * @brief Read n bytes start at register from sensor.
 *
 * @param handle The pointer to fxas21002cq driver handle.
 * @param reg Register address.
 * @param val The pointer to address which store data.
 * @param bytesNumber Number of bytes receiver.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXAS_ReadReg(fxas_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber);

/*!
 * @brief Format to float.
 *
 * @param input The input integrate data.
 * @param fsrdps The FSR mode.
 *
 * @return The formated float.
 */
float FXAS_FormatFloat(int16_t input,  fxas_gfsr_t fsrdps);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _FSL_FXAS_H_ */
