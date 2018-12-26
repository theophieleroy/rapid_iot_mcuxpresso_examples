/******************************************************************************
Derivative Work of
SparkFunCCS811.cpp
CCS811 Arduino library
Marshall Taylor @ SparkFun Electronics
Nathan Seidle @ SparkFun Electronics

April 4, 2017

https://github.com/sparkfun/CCS811_Air_Quality_Breakout
https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library/blob/master/LICENSE.md .
If you have any questions or concerns with licensing, please contact techsupport@sparkfun.com.

The MIT License (MIT)

Copyright (c) 2015 SparkFun Electronics
Copyright (c) 2018 NXP

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Distributed as-is; no warranty is given.
******************************************************************************/

/*!
 * @file
 * This is the source file for the air quality sensor CCS811 driver.
 */

#include "CCS811.h"
#include <math.h>
#include <stdbool.h>
#include <assert.h>

/*****************************************************************************
 * Variables
 ****************************************************************************/
static CCS811_fct_t FCT_CCS811;
static bool initDriverDone = false;
static bool initHwDone = false;

/* air quality values obtained from the sensor */
static float refResistance;
static float resistance = 10000;
static uint16_t tVOC = 0;
static uint16_t CO2 = 0;
static uint16_t vrefCounts = 0;
static uint16_t ntcCounts = 0;
static float temperature = 0;

/*****************************************************************************
 * Static functions
 ****************************************************************************/

/*
 * @brief Read a CCS811 register.
 *
 * @param  Address of the register to read
 * @param  Pointer to the read value
 * @return Status value of I2C read function (0 for success)
 */
static uint8_t CCS811_readRegister(uint8_t offset, uint8_t* outputPointer)
{
    uint8_t cmd[2];

    cmd[0] = offset;
    return FCT_CCS811.I2C_Read(CCS811_I2C_ADDRESS, cmd, 1, outputPointer, 1);
}

/*
 * @brief Read several CCS811 registers.
 *
 * @param  Start address of the registers to read
 * @param  Pointer to the read values
 * @param  Number of consecutive registers to read
 * @return Status value of I2C read function (0 for success)
 */
static uint8_t CCS811_multiReadRegister(uint8_t offset, uint8_t *outputPointer, uint8_t length)
{
    uint8_t cmd[2];

    cmd[0] = offset;
    return FCT_CCS811.I2C_Read(CCS811_I2C_ADDRESS, cmd, 1, outputPointer, length);
}

/*
 * @brief Write to a CCS811 register.
 *
 * @param  Address of the register to write to
 * @param  Value to write
 * @return Status value of I2C read function (0 for success)
 */
static uint8_t CCS811_writeRegister(uint8_t offset, uint8_t dataToWrite)
{

    uint8_t cmd[2];

    cmd[0] = offset;
    cmd[1] = dataToWrite;

    return FCT_CCS811.I2C_Write(CCS811_I2C_ADDRESS, cmd, 2);
}

/*
 * @brief Write to several CCS811 registers.
 *
 * @param  Start address of the registers to write to
 * @param  Pointer to the values to be written
 * @param  Number of consecutive registers to write to
 * @return Status value of I2C read function (0 for success)
 */
static uint8_t CCS811_multiWriteRegister(uint8_t offset, uint8_t *inputPointer, uint8_t length)
{
    uint8_t cmd[length+1];
    cmd[0] = offset;
    for( uint32_t i = 0; i < length; i++ ) //This waits > 1ms @ 80MHz clock
    {
        cmd[1+i] = inputPointer[i];
    }
    return FCT_CCS811.I2C_Write(CCS811_I2C_ADDRESS, cmd, length+1);
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

void CCS811_Init_Driver(ptCCS811_fct_t FCT)
{
    assert((FCT != NULL) &&
            (FCT->connect_hw != NULL) &&
            (FCT->disconnect_hw != NULL) &&
            (FCT->I2C_Read != NULL) &&
            (FCT->I2C_Write != NULL) &&
            (FCT->WaitMsec != NULL));
    FCT_CCS811 = *FCT;
    initDriverDone = true;
}

void CCS811_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        FCT_CCS811.disconnect_hw();
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

CCS811_status CCS811_Init_Hw(void)
{
    uint8_t data[4] = {0x11,0xE5,0x72,0x8A}; /* reset key */
    uint8_t status;
    uint8_t val = 0;
    CCS811_status error_status = CCS811_SUCCESS;

    if (!initDriverDone) return CCS811_NOINIT_ERROR;

    FCT_CCS811.connect_hw();

    /* trick to allow calling internal public functions */
    initHwDone = true;

    /* check HW ID */
    if (CCS811_readRegister( CCS811_HW_ID,  &val) != 0)
    {
        error_status = CCS811_I2C_ERROR;
        goto return_status;
    }
    if (val != 0x81)
    {
        error_status = CCS811_INTERNAL_ERROR;
        goto return_status;
    }

    /* reset the device */
    if (CCS811_multiWriteRegister(CCS811_SW_RESET, data, 4)!=0)
    {
        error_status =  CCS811_I2C_ERROR;
        goto return_status;
    }

    /* wait 1ms */
    FCT_CCS811.WaitMsec(1);

    if (CCS811_checkForStatusError(&status) == CCS811_I2C_ERROR)
    {
        error_status = CCS811_I2C_ERROR;
        goto return_status;
    }
    if (status == true) return CCS811_INTERNAL_ERROR;

    error_status = CCS811_appValid(&val);
    if (error_status != CCS811_SUCCESS) goto return_status;
    if (val == 0)
    {
        error_status =  CCS811_INTERNAL_ERROR;
        goto return_status;
    }

    /* write 0 byte to this register to start app */
    //if (CCS811_writeRegister(CCS811_APP_START, 0)!=0) return CCS811_I2C_ERROR;
    val = CCS811_APP_START;
    if (FCT_CCS811.I2C_Write(CCS811_I2C_ADDRESS, &val, 1))
    {
        error_status = CCS811_I2C_ERROR;
        goto return_status;
    }

    /* read every second */
    if (CCS811_setDriveMode(1))
    {
        error_status = CCS811_I2C_ERROR;
        goto return_status;
    }

    return_status:
    if (error_status != CCS811_SUCCESS) initHwDone = false;
    return error_status;
}

CCS811_status CCS811_readAlgorithmResults(void)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    uint8_t data[4];
    if (CCS811_multiReadRegister(CCS811_ALG_RESULT_DATA, data, 4)!=0) return CCS811_I2C_ERROR;

    /* data ordered: co2MSB, co2LSB, tvocMSB, tvocLSB */
    CO2 = ((uint16_t)data[0] << 8) | data[1];
    tVOC = ((uint16_t)data[2] << 8) | data[3];
    return CCS811_SUCCESS;
}

CCS811_status CCS811_checkForStatusError(uint8_t* StatusError)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;
    assert(StatusError != NULL);

    /* read the status bit */
    if (CCS811_readRegister(CCS811_STATUS, StatusError )!=0) return CCS811_I2C_ERROR;
    *StatusError = *StatusError & 1 << 0; // bit ERROR
    return CCS811_SUCCESS;
}

CCS811_status CCS811_dataAvailable(uint8_t* value)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;
    assert(value != NULL);

    if (CCS811_readRegister(CCS811_STATUS, value )!=0) return CCS811_I2C_ERROR;
    *value = (*value &  1 << 3)>>3; // bit DATA_READY
    return CCS811_SUCCESS;
}

CCS811_status CCS811_appValid(uint8_t* value)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;
    assert(value != NULL);

    if (CCS811_readRegister(CCS811_STATUS, value )!=0) return CCS811_I2C_ERROR;
    *value = (*value &  1 << 4)>>4; // bit APP_VALID
    return CCS811_SUCCESS;
}

CCS811_status CCS811_getErrorRegister(uint8_t* value)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;
    assert(value != NULL);

    *value=0xFF;
    if (CCS811_readRegister(CCS811_ERROR_ID, value )!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

CCS811_status CCS811_getBaseline(unsigned int* baseline)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;
    assert(baseline != NULL);

    uint8_t data[2];
    if (CCS811_multiReadRegister(CCS811_BASELINE,data, 2)!=0) return CCS811_I2C_ERROR;

    *baseline = ((uint16_t)data[0] << 8) | data[1];
    return CCS811_SUCCESS;
}

CCS811_status CCS811_setBaseline(uint16_t input)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    uint8_t data[2];
    data[0] = (input >> 8) & 0x00FF;
    data[1] = input & 0x00FF;

    if (CCS811_multiWriteRegister(CCS811_BASELINE, data, 2)!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

CCS811_status CCS811_setInterrupts(uint8_t fct)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    uint8_t value;
    if (CCS811_readRegister(CCS811_MEAS_MODE, &value )!=0) return CCS811_I2C_ERROR;
    if (fct == 0)
    {
        value &= ~(CCS811_INTERRUPT_DRIVEN); /* clear INTERRUPT bit */
    }
    else
    {
        value |= (CCS811_INTERRUPT_DRIVEN); /* set INTERRUPT bit */
    }

    if (CCS811_writeRegister(CCS811_MEAS_MODE, value)!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

CCS811_status CCS811_setThresholds(uint8_t fct)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    uint8_t value;
    if (CCS811_readRegister(CCS811_MEAS_MODE, &value )!=0) return CCS811_I2C_ERROR;
    if (fct == 0)
    {
        value &= ~(CCS811_THRESHOLDS_ENABLED); /* clear THRESHOLDS bit */
    }
    else
    {
        value |= (CCS811_THRESHOLDS_ENABLED); /* set THRESHOLDS bit */
    }

    if (CCS811_writeRegister(CCS811_MEAS_MODE, value)!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

CCS811_status CCS811_setDriveMode(uint8_t mode)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    if (mode > 4) mode = 4; /* sanitize input */

    uint8_t value;
    if (CCS811_readRegister(CCS811_MEAS_MODE, &value)!=0) return CCS811_I2C_ERROR;
    value &= ~(0b00000111 << 4); /* clear DRIVE_MODE bits */
    value |= (mode << 4); /* mask in mode */
    if (CCS811_writeRegister(CCS811_MEAS_MODE, value)!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

CCS811_status CCS811_setEnvironmentalData(float relativeHumidity, float temperature)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    /* check for invalid temperatures */
    if ((temperature < -25) || (temperature > 50)) return CCS811_GENERIC_ERROR;

    /* check for invalid humidity */
    if ((relativeHumidity < 0) || (relativeHumidity > 100)) return CCS811_GENERIC_ERROR;

    uint32_t rH = relativeHumidity * 1000; /* 42.348 becomes 42348 */
    uint32_t temp = temperature * 1000; /* 23.2 becomes 23200 */

    uint8_t envData[4];

    /* split value into 7-bit integer and 9-bit fractional */
    envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
    envData[1] = 0; /* support only increments of 0.5 so bits 7-0 will always be zero */
    if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
    {
        envData[0] |= 1; /* set 9th bit of fractional to indicate 0.5% */
    }

    temp += 25000; /* add the 25C offset */
    /* split value into 7-bit integer and 9-bit fractional */
    envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
    envData[3] = 0;
    if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
    {
        envData[2] |= 1;  /* set 9th bit of fractional to indicate 0.5C */
    }
    if (CCS811_multiWriteRegister(CCS811_ENV_DATA, envData, 4)!=0) return CCS811_I2C_ERROR;
    return CCS811_SUCCESS;
}

void CCS811_setRefResistance(float input)
{
    refResistance = input;
}

CCS811_status CCS811_readNTC(void)
{
    if (!initHwDone) return CCS811_NOINIT_ERROR;

    uint8_t data[4];
    if (CCS811_multiReadRegister(CCS811_NTC, data, 4)!=0) return CCS811_I2C_ERROR;

    vrefCounts = ((uint16_t)data[0] << 8) | data[1];
    ntcCounts = ((uint16_t)data[2] << 8) | data[3];
    resistance = ((float)ntcCounts * refResistance / (float)vrefCounts);

    temperature = log((long)resistance);
    temperature = 1 / (0.001129148 + (0.000234125 * temperature) + (0.0000000876741 * temperature * temperature * temperature));
    temperature = temperature - 273.15;  /* convert from Kelvin to Celsius */

    return CCS811_SUCCESS;
}

uint16_t CCS811_getTVOC(void)
{
    return tVOC;
}

uint16_t CCS811_getCO2(void)
{
    return CO2;
}

float CCS811_getResistance(void)
{
    return resistance;
}

float CCS811_getTemperature(void)
{
    return temperature;
}

