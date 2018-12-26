/*
*********************************************************************************
 * Copyright 2018 by ams AG                                                     *
 *                                                                              *
 * Redistribution and use in source and binary forms, with or without           *
 * modification, *are permitted provided that the following conditions are met: *
 *  1. Redistributions of source code must retain the above copyright notice,   *
 * this list of conditions and the following disclaimer.                        *
 *                                                                              *
 *  2. Redistributions in binary form must reproduce the above copyright notice,*
 *  this list of conditions and the following disclaimer in the documentation   *
 *  and/or other materials provided with the distribution.                      *
 *  3. Neither the name of the copyright holder nor the names of its            *
 * contributors may be used to endorse or promote products derived from this    *
 * software without specific prior written permission.                          *
 *                                                                              *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  *
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE    *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR          *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS     *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)      *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   *
 * POSSIBILITY OF SUCH DAMAGE.                                                  *
 ********************************************************************************
*/

/*!
 * @file
 * This is the header file for the temperature and humidity sensor ENS210 driver.
 */

#ifndef __ENS210_H_
#define __ENS210_H_

#include <stdbool.h>
#include <EmbeddedTypes.h>

/*! @addtogroup ens210   ENS210 Relative Humidity and Temperature Sensor
 * This module provides the API to operate an ENS210 relative humidity and temperature sensor with I2C interface.
 *
 * Basic steps to operate the sensor are as follows:
 * -# Set Run mode (#ENS210_SensRun_Set)
 * -# Start measurement (#ENS210_SensStart_Set)
 * -# Wait for measurement to complete
 * -# Read measurement (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get)
 *
 * Please refer to ENS210 Reference Driver and Porting Guide for more details on platform porting. In this module, names
 * T and H have been used to refer to temperature and relative humidity respectively to comply with ENS210 datasheet
 * naming convention.
 *
 * Example 1 - Sample application code to measure temperature and relative humidity without error checking
 * -------------------------------------------------------------------------------------------------------
 * @code
 * uint32_t T_Raw, H_Raw;
 * int32_t T_mCelsius, T_mFahrenheit, T_mKelvin, H_Percent;
 *
 * //Set runmode, start measurement, wait, read measurement (for both T and H)
 * ENS210_SensRun_Set(ENS210_SENSRUN_T_MODE_SINGLE_SHOT | ENS210_SENSRUN_H_MODE_SINGLE_SHOT);
 * ENS210_SensStart_Set(ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START);
 * WaitMsec(ENS210_T_H_CONVERSION_TIME_MS);
 * ENS210_THVal_Get(&T_Raw,&H_Raw);
 *
 * //Convert the raw temperature to milli Kelvin
 * T_mKelvin = ENS210_ConvertRawToKelvin(T_Raw, 1000);
 * //Convert the raw temperature to milli Celsius
 * T_mCelsius = ENS210_ConvertRawToCelsius(T_Raw, 1000);
 * //Convert the raw temperature to milli Fahrenheit
 * T_mFahrenheit = ENS210_ConvertRawToFahrenheit(T_Raw, 1000);
 * printf("T crc ok = %s\n", ENS210_IsCrcOk(T_Raw)  ? "yes" : "no");
 * printf("T valid = %s \n", ENS210_IsDataValid(T_Raw) ? "yes" : "no");
 * //Update the int32_t format specifier (%ld) based on platform word-size
 * printf("T = %ld mK %ld mC %ld mF \n", T_mKelvin, T_mCelsius, T_mFahrenheit);
 *
 * //Convert the raw relative humidity to milli %
 * H_Percent = ENS210_ConvertRawToPercentageH(H_Raw, 1000);
 * printf("H crc ok = %s\n", ENS210_IsCrcOk(H_Raw)  ? "yes" : "no");
 * printf("H valid = %s \n", ENS210_IsDataValid(H_Raw) ? "yes" : "no");
 * //Update the int32_t format specifier (%ld) based on platform word-size
 * printf("H = %ld m%%\n", H_Percent);
 *
 * @endcode
 *
 *
 * Example 2 - Sample application code to measure relative humidity with error checking
 * ------------------------------------------------------------------------------------
 * @code
 * uint32_t H_Raw;
 * int32_t H_Percent;
 * int status;
 * bool i2cOk;
 *
 * i2cOk = true; //Start accumulating I2C transaction errors
 *
 * status = ENS210_SensRun_Set(ENS210_SENSRUN_H_MODE_SINGLE_SHOT);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * status = ENS210_SensStart_Set(ENS210_SENSSTART_H_START);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * WaitMsec(ENS210_T_H_CONVERSION_TIME_MS);
 *
 * status = ENS210_HVal_Get(&H_Raw);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * if( !i2cOk ) {
 *     printf("H i2c error\n")
 * } else if( !ENS210_IsCrcOk(H_Raw) ) {
 *     printf("H crc error\n")
 * } else if( !ENS210_IsDataValid(H_Raw) ) {
 *     printf("H data invalid\n")
 * } else {
 *     //Convert the raw relative humidity to milli %
 *     H_Percent = ENS210_ConvertRawToPercentageH(H_Raw,1000);
 *     //Update the int32_t format specifier (%ld) based on platform word-size
 *     printf("H = %ld m%%\n", H_Percent);
 * }
 *
 * @endcode
 *
 * @{
 */
/*****************************************************************************
 * Types/enumerations/variables
 ****************************************************************************/

/*! @brief Status return codes. */
typedef enum ens210_status_ {
    ens210_success = 0,             /*!< Function returned successfully. */
    ens210_I2C_error = 1,           /*!< I2C Error. */
    ens210_invalid_ID = 2,          /*!< Invalid ID. */
    ens210_Tdata_CRC_error = 3,     /*!< CRC error for temperature data. */
    ens210_Hdata_CRC_error = 4,     /*!< CRC error for humidity data. */
    ens210_T_invalid_data = 5,      /*!< Temperature Data is invalid. */
    ens210_H_invalid_data = 6,      /*!< Humidity Data is invalid. */
    ens210_wrong_parameter = 7,     /*!< Wrong Parameter entered. */
    ens210_noinit = 8               /*!< ENS210 was not initialized. */
} ens210_status_t;

/*! @brief Measurement mode of Sensor */
enum measurement_mode {
    mode_TH = 0,        /*!< ENS210 set to measure both temperature and humidity. */
    mode_Tonly = 1,     /*!< ENS210 set to measure temperature only. */
    mode_Honly = 2      /*!< ENS210 set to measure humidity only. */
};

/*! ENS210 os-free driver version info */
#define ENS210_OSFREE_DRIVER_VERSION            2

/*! ENS210 T and H conversion time in milliseconds. Refer to ENS210 data sheet for timing information. */
#define ENS210_T_H_CONVERSION_TIME_MS           130

/*! ENS210 T conversion time in milliseconds */
#define ENS210_T_CONVERSION_TIME_MS             110

/*! ENS210 Booting time in milliseconds. */
#define ENS210_BOOTING_TIME_MS                  2

/*! ENS210 Reset time in milliseconds. */
#define ENS210_RESET_WAIT_TIME_MS               2

/*! ENS210 I2C slave address */
#define ENS210_I2C_SLAVE_ADDRESS                (uint8_t)0x43

/*! ENS210 SysCtrl register: Low power enable */
#define ENS210_SYSCTRL_LOWPOWER_ENABLE          (1 << 0)
/*! ENS210 SysCtrl register: Low power disable */
#define ENS210_SYSCTRL_LOWPOWER_DISABLE         (0 << 0)
/*! ENS210 SysCtrl register: Reset enable */
#define ENS210_SYSCTRL_RESET_ENABLE             (1 << 7)
/*! ENS210 SysCtrl register: Reset disable */
#define ENS210_SYSCTRL_RESET_DISABLE            (0 << 7)

/*! ENS210 SysStat register: Standby or Booting mode */
#define ENS210_SYSSTAT_MODE_STANDBY             (0 << 0)
/*! ENS210 SysStat register: Active mode */
#define ENS210_SYSSTAT_MODE_ACTIVE              (1 << 0)


/*! ENS210 SensRun register: temperature single shot mode */
#define ENS210_SENSRUN_T_MODE_SINGLE_SHOT       (0 << 0)
/*! ENS210 SensRun register: temperature continuous mode */
#define ENS210_SENSRUN_T_MODE_CONTINUOUS        (1 << 0)
/*! ENS210 SensRun register: relative humidity single shot mode */
#define ENS210_SENSRUN_H_MODE_SINGLE_SHOT       (0 << 1)
/*! ENS210 SensRun register: relative humidity continuous mode */
#define ENS210_SENSRUN_H_MODE_CONTINUOUS        (1 << 1)

/*! ENS210  SensStart register: T sensor start */
#define ENS210_SENSSTART_T_START                (1 << 0)
/*! ENS210  SensStart register: H sensor start */
#define ENS210_SENSSTART_H_START                (1 << 1)

/*! ENS210  SensStop register: T sensor stop */
#define ENS210_SENSSTOP_T_STOP                  (1 << 0)
/*! ENS210  SensStop register: H sensor stop */
#define ENS210_SENSSTOP_H_STOP                  (1 << 1)

/*! ENS210  SensStat register: T sensor idle */
#define ENS210_SENSSTAT_T_STAT_IDLE             (0 << 0)
/*! ENS210  SensStat register: T sensor active */
#define ENS210_SENSSTAT_T_STAT_ACTIVE           (1 << 0)
/*! ENS210  SensStat register: H sensor idle */
#define ENS210_SENSSTAT_H_STAT_IDLE             (0 << 1)
/*! ENS210  SensStat register: H sensor active */
#define ENS210_SENSSTAT_H_STAT_ACTIVE           (1 << 1)

/* wrapper for the I2C write, I2C read and wait functions needed by the sensor driver */
/* it is expected that the I2C_Read and I2C_Write functions return 0 if the I2C transaction is successful */
typedef struct _ens210_IoFunc_t
{
    uint8_t   (*I2C_Read)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize);  /*!< External I2C read function */
    uint8_t   (*I2C_Write)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize);  /*!< External I2C write function */
    void      (*WaitMsec)(uint32_t millisec); /*!< Wait function in milliseconds */
} ens210_IoFunc_t, *pens210_IoFunc_t;

/*! @brief    ENS210 ID block structure */
typedef struct ENS210_Ids_s
{
    uint16_t    partId;             /*!< Part ID */
    uint8_t     uId[8];             /*!< Unique Identifier 8 bytes */
} ENS210_Ids_t;

/*! @brief Structure of measurement data. */
typedef struct ens210_meas_data_s
{
    int32_t T_Celsius;              /*!< Temperature in Celsius */
    int32_t T_Fahrenheit;           /*!< Temperature in Fahrenheit */
    int32_t T_Kelvin;               /*!< Temperature in Kelvin */
    int32_t T_mCelsius;             /*!< Temperature in milliCelsius */
    int32_t T_mFahrenheit;          /*!< Temperature in milliFahrenheit */
    int32_t T_mKelvin;              /*!< Temperature in milliKelvin */
    int32_t H_Percent;              /*!< Relative Humidity to % */
    int32_t H_mPercent;             /*!< Relative Humidity to milli% */
} ens210_meas_data_t;

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/*!
 * @brief   Initialize ENS210 driver.
 * @param   pIoFunc     :   Pointer to a structure of external functions or values
 */
void ENS210_Init_Driver(ens210_IoFunc_t* pIoFunc);

/*!
 * @brief   De-initialize ENS210 driver.
 */
void ENS210_Deinit_Driver();

/*!
 * @brief   Initialize ENS210 hardware.
 * @return  The return status value (0 for success)
 */
ens210_status_t ENS210_Init_Hw(void);

/*!
 * @brief   Set ENS210 SysCtrl register; enabling reset and/or low power.
 * @param   sysCtrl     :   Mask composed of  ENS210_SYSCTRL_xxx macros.
 * @return  The return status value (0 for success)
 *  */
ens210_status_t ENS210_SysCtrl_Set(uint8_t sysCtrl);

/*!
 * @brief   Get ENS210 SysCtrl register.
 * @param   sysCtrl     :   Pointer to receive value of the register. Must not be null.
 * @return  The return status value (0 for success)
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_SysCtrl_Get(uint8_t *sysCtrl);

/*!
 * @brief   Get ENS210 SysStat register.
 * @param   sysStat     :   Pointer to receive value of the register. Must not be null.
 * @return  The return status value (0 for success)
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_SysStat_Get(uint8_t *sysStat);

/*!
 * @brief   Set ENS210 SensRun register; set the run mode single shot/continuous for T and H sensors.
 * @param   sensRun     :   Mask composed of ENS210_SENSRUN_xxx macros.
 * @return  The return status value (0 for success)
 */
ens210_status_t ENS210_SensRun_Set(uint8_t sensRun);

/*!
 * @brief   Get ENS210 SensRun register.
 * @param   sensRun     :   Pointer to receive value of the register. Must not be null.
 * @return  The return status value (0 for success)
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_SensRun_Get(uint8_t *sensRun);

/*!
 * @brief   Set ENS210 SensStart register; starts the measurement for T and/or H sensors.
 * @param   sensStart   :  Mask composed of ENS210_SENSSTART_xxx macros.
 * @return  The return status value (0 for success)
 */
ens210_status_t ENS210_SensStart_Set(uint8_t sensStart);

/*!
 * @brief   Set ENS210 SensStop register; stops the measurement for T and/or H sensors.
 * @param   sensStop    :   Mask composed of ENS210_SENSSTOP_xxx macros.
 * @return  The return status value (0 for success)
 */
ens210_status_t ENS210_SensStop_Set(uint8_t sensStop);

/*!
 * @brief   Get ENS210 SensStat register.
 * @param   sensStat    :   Pointer to receive value of the register. Must not be null.
 * @return  The return status value (0 for success)
 */
ens210_status_t ENS210_SensStat_Get(uint8_t *sensStat);

/*!
 * @brief   Get ENS210 TVal register; raw measurement data as well as CRC and valid indication.
 * @param   traw         :   Pointer to receive value of the register. Must not be null.
 * @return  The return status value (0 for success)
 * @note
 * Use ENS210_IsCrcOk and ENS210_IsDataValid before using the measurement data.\n
 * Use ENS210_ConvertRawToXXX to convert raw data to standard units.\n
 * If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_TVal_Get(uint32_t *traw);

/*!
 * @brief   Get ENS210 HVal register; raw measurement data as well as CRC and valid indication.
 * @param   hraw         :   Pointer to receive value of register. Must not be null.
 * @return  The return status value (0 for success)
 * @note
 * Use ENS210_IsCrcOk and ENS210_IsDataValid before using the measurement data.\n
 * If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_HVal_Get(uint32_t *hraw);

/*!
 * @brief   Get ENS210 TVal and HVal registers; raw measurement data as well as CRC and valid indication.
 * @param   traw         :   Pointer to receive value of TVal register. Must not be null.
 * @param   hraw         :   Pointer to receive value of HVal register. Must not be null.
 * @return  The return status value (0 for success)
 * @note
 * Use ENS210_IsCrcOk and ENS210_IsDataValid before using the measurement data.\n
 * If the return indicates I2C failure, the value of the out parameter is undefined.
 */
ens210_status_t ENS210_THVal_Get(uint32_t *traw, uint32_t *hraw);

/*!
 * @brief   Get ENS210 Part ID and UID.
 * @param   ids         :   Pointer to receive ids. Must not be null.
 * @return  The return status value (0 for success)
 * @note    If this function returns an error, it is suggested to reset the device to bring it to a known state.
 */
ens210_status_t ENS210_Ids_Get(ENS210_Ids_t *ids);

/*!
 * @brief   Get ENS210 temperature and humidity measurements
 * @param   meas_mode    :   Measurement mode
 * @param   results      :   Pointer to the result structure
 * @return  The return status value (0 for success)
 * @note
 * mode = 0: temperature and humidity\n
 * mode = 1: temperature only\n
 * mode = 2: humidity only
 */
ens210_status_t ENS210_Measure(uint8_t meas_mode, ens210_meas_data_t *results);

/*!
 * @}
 */

#endif /* __ENS210_H_ */

