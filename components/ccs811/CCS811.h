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
 * This is the header file for the air quality sensor CCS811 driver.
 */

#ifndef CCS811_H_
#define CCS811_H_

/*!
 * @addtogroup ccs811 CCS811 Air Quality Sensor
 * This module provides the API to control and monitor the CCS811 air quality sensor through an I2C interface.
 *
 * The basic steps to operate the CCS811 are as follows:
 * -# Initialize the driver with callback functions (#CCS811_Init_Driver)
 * -# Initialize the hardware (#CCS811_Init_Hw)
 * -# Check data availability (#CCS811_dataAvailable)
 * -# Trigger all data acquisition (#CCS811_readAlgorithmResults)
 * -# Read the relevant data (for example #CCS811_getCO2)
 * -# If the CCS811 is not needed anymore, de-initialize the driver (#CCS811_Deinit_Driver). The CCS811 will
 * be switched off. It allows to eventually release shared resources.
 *
 * Example - Sample application code to set CCS811 without error management
 * ------------------------------------------------------------------------
 * @code
 * #include "CCS811.h"
 *
 * CCS811_fct_t ccs811_fct;
 * uint8_t ready;
 * uint16_t uCO2;
 *
 * ccs811_fct.connect_hw = Ccs811_Connect;       // callback function to activate CCS811 hardware resource
 * ccs811_fct.disconnect_hw = Ccs811_Disconnect; // callback function to deactivate CCS811 hardware resource
 * ccs811_fct.I2C_Read = I2c_Read;               // callback function for I2C read
 * ccs811_fct.I2C_Write = I2c_Write;             // callback function for I2C write
 * ccs811_fct.WaitMsec = WaitMs;                 // wait callback function (in ms)
 *
 * CCS811_Init_Driver(&ccs811_fct);
 * CCS811_Init_Hw();
 * // check new data availability
 * CCS811_dataAvailable(&ready);
 * if (ready == 1) {
 *     CCS811_readAlgorithmResults();
 *     uCO2 = CCS811_getCO2();
 *     printf("The current CO2 value is: %d\n", uCO2);
 * }
 * [..]
 * // if the CCS811 is not needed anymore, the driver can be de-initialized
 * printf("Deinitialize CCS811 (it will be turned off)\n");
 * CCS811_Deinit_Driver();
 *
 * @endcode
 * @{
 */

#include <EmbeddedTypes.h>
#include <stdbool.h>

#define CCS811_I2C_ADDRESS              0x5A

/* Register addresses */
#define CCS811_STATUS                   0x00
#define CCS811_MEAS_MODE                0x01
#define CCS811_ALG_RESULT_DATA          0x02
#define CCS811_RAW_DATA                 0x03
#define CCS811_ENV_DATA                 0x05
#define CCS811_NTC                      0x06
#define CCS811_THRESHOLDS               0x10
#define CCS811_BASELINE                 0x11
#define CCS811_HW_ID                    0x20
#define CCS811_HW_VERSION               0x21
#define CCS811_FW_BOOT_VERSION          0x23
#define CCS811_FW_APP_VERSION           0x24
#define CCS811_ERROR_ID                 0xE0
#define CCS811_APP_START                0xF4
#define CCS811_SW_RESET                 0xFF
#define CCS811_INTERRUPT_DRIVEN         0x8
#define CCS811_THRESHOLDS_ENABLED       0x4

/*! @brief Status return codes. */
typedef enum _CCS811_status
{
    CCS811_SUCCESS,        /*!< Success */
    CCS811_ID_ERROR,       /*!< Bad hardware identifier */
    CCS811_I2C_ERROR,      /*!< I2C read/write error */
    CCS811_INTERNAL_ERROR, /*!< Internal hardware error */
    CCS811_NOINIT_ERROR,   /*!< Hardware or driver not initialized */
    CCS811_GENERIC_ERROR   /*!< Generic error (settings not allowed) */
} CCS811_status;

/*! @brief Structure of external functions or values. */
typedef struct _CCS811_fct_t
{
    void (*connect_hw)(void);     /*!< External function to activate the CCS811 hardware */
    void (*disconnect_hw)(void);  /*!< External function to deactivate the CCS811 hardware */
    uint8_t (*I2C_Read)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize); /*!< External I2C read function */
    uint8_t (*I2C_Write)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize); /*!< External I2C write function */
    void (*WaitMsec)(uint32_t tms); /*!< Wait function in milliseconds */
} CCS811_fct_t, *ptCCS811_fct_t;

/*!
 * @brief Initialize CCS811 driver.
 *
 * @param FCT Pointer to a structure of external functions or values
 */
void CCS811_Init_Driver(ptCCS811_fct_t FCT);

/*!
 * @brief De-initialize CCS811 driver.
 *
 */
void CCS811_Deinit_Driver();

/*!
 * @brief Initialize CCS811 hardware.
 *
 * @return Status value (0 for success)
 */
CCS811_status CCS811_Init_Hw(void);

/*!
 * @brief Read algorithm results.
 * @note Update the total volatile organic compounds (TVOC) in parts per billion (PPB) and the CO2 value.
 *
 * @return Status value (0 for success)
 */
CCS811_status CCS811_readAlgorithmResults(void);

/*!
 * @brief Check if error bit is set.
 *
 * @param  StatusError Pointer to error status bit value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_checkForStatusError(uint8_t* StatusError);

/*!
 * @brief Check if data is available.
 * @note Based on DATA_READ flag in the status register.
 *
 * @param  value Pointer to available data bit value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_dataAvailable(uint8_t* value);

/*!
 * @brief Check if APP_VALID is set.
 * @note Based on APP_VALID flag in the status register.
 *
 * @param  value Pointer to APP_VALID bit value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_appValid(uint8_t* value);

/*!
 * @brief Get the error register value.
 * @note Based on ERROR_ID register.
 *
 * @param  value Pointer to error value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_getErrorRegister(uint8_t* value);

/*!
 * @brief Get the baseline value.
 * @note It is used for telling sensor what 'clean' air is.
 * @note Put the sensor in clean air and record this value.
 *
 * @param  baseline Pointer to baseline value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_getBaseline(unsigned int* baseline);

/*!
 * @brief Set the baseline value.
 * @note It is used for telling sensor what 'clean' air is.
 * @note Put the sensor in clean air and record this value.
 *
 * @param  input Baseline value
 * @return Status value (0 for success)
 */
CCS811_status CCS811_setBaseline(uint16_t input);

/*!
 * @brief Enable or disable the interrupts.
 * @note It clears/sets the nINT signal.
 *
 * @param  fct Interrupt mode (0 for disable, 1 for enable)
 * @return Status value (0 for success)
 */
CCS811_status CCS811_setInterrupts(uint8_t fct);

/*!
 * @brief Enable or disable the interrupt thresholds.
 *
 * @param  fct Threshold mode (0 for disable, 1 for enable)
 * @return Status value (0 for success)
 */
CCS811_status CCS811_setThresholds(uint8_t fct);

/*!
 * @brief Set the drive mode.
 * @note
 * Mode 0 = Idle\n
 * Mode 1 = read every 1s\n
 * Mode 2 = every 10s\n
 * Mode 3 = every 60s\n
 * Mode 4 = RAW mode
 *
 * @param  mode Drive mode
 * @return Status value (0 for success)
 */
CCS811_status CCS811_setDriveMode(uint8_t mode);

/*!
 * @brief Set environmental data.
 * @note
 * Given a temperature and humidity, use these data for better compensation.
 *
 * @param  relativeHumidity Relative humidity, value within [0,100]
 * @param  temperature Temperature (Celsius), value within [-25,+50]
 * @return Status value (0 for success)
 */
CCS811_status CCS811_setEnvironmentalData(float relativeHumidity, float temperature);

/*!
 * @brief Set ref resistance.
 *
 * @param  input Ref resistance
 */
void CCS811_setRefResistance(float input);

/*!
 * @brief Read NTC.
 *
 * @return Status value (0 for success)
 */
CCS811_status CCS811_readNTC(void);

/*!
 * @brief Get Total Volatile Organic Compound (TVOC) value.
 *
 * @return TVOC value (0ppb to 1187ppb)
 */
uint16_t CCS811_getTVOC(void);

/*!
 * @brief Get the equivalent CO2 value.
 *
 * @return eCO2 value (400ppm to 8192ppm)
 */
uint16_t CCS811_getCO2(void);

/*!
 * @brief Get resistance value.
 *
 * @return Resistance value
 */
float CCS811_getResistance(void);

/*!
 * @brief Get temperature value.
 *
 * @return Temperature value (Celsius)
 */
float CCS811_getTemperature(void);

/*! @}*/

#endif /* CCS811_H_ */
