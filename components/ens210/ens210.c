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

/**
 * @file
 * This is the source file for the temperature and humidity sensor ENS210 driver.
 */

#include "ens210.h"
#include <string.h>
#include <assert.h>

/*****************************************************************************
 * Private macros and functions
 ****************************************************************************/
/* Register addresses */
#define ENS210_REG_PART_ID     0x00
#define ENS210_REG_UID         0x04
#define ENS210_REG_SYS_CTRL    0x10
#define ENS210_REG_SYS_STAT    0x11
#define ENS210_REG_SENS_RUN    0x21
#define ENS210_REG_SENS_START  0x22
#define ENS210_REG_SENS_STOP   0x23
#define ENS210_REG_SENS_STAT   0x24
#define ENS210_REG_T_VAL       0x30
#define ENS210_REG_H_VAL       0x33

/** Mask to extract 16-bit data from raw T and H values */
#define ENS210_T_H_MASK        0xFFFFU

/** Simplification macro, implementing integer division with simple rounding to closest number
 *  It supports both positive and negative numbers, but ONLY positive divisors */
#define IDIV(n,d)              ((n)>0 ? ((n)+(d)/2)/(d) : ((n)-(d)/2)/(d))

#define CRC7WIDTH              7     //7 bits CRC has polynomial of 7th order (has 8 terms)
#define CRC7POLY               0x89  //The 8 coefficients of the polynomial
#define CRC7IVEC               0x7F  //Initial vector has all 7 bits high

#define DATA7WIDTH             17
#define DATA7MASK              ((1UL << DATA7WIDTH) - 1)  //0b 1 1111 1111 1111 1111
#define DATA7MSB               (1UL << (DATA7WIDTH - 1))  //0b 1 0000 0000 0000 0000

/** When the ENS210 is soldered a correction on T needs to be applied (see application note).
 *  Typically the correction is 50mK. Units for raw T is 1/64K. */
#define ENS210_TRAW_SOLDERCORRECTION  (50*64/1000)

/*****************************************************************************
 * Variables
 ****************************************************************************/

static ens210_IoFunc_t sENS210_Func;
static bool initDriverDone = false;
static bool initHwDone = false;

/*****************************************************************************
 * Private functions prototypes
 ****************************************************************************/

/*
 * @brief   Compute the CRC7 results.
 * @param   val   :   the value
 * @return  The CRC7 computed result
 */
static uint32_t ENS210_ComputeCRC7(uint32_t val);

/*
 * @brief   Verify the CRC of the raw temperature or relative humidity.
 * @param   raw         :  Raw temperature or relative humidity value to be verified (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get).
 * @return  True - Success,  False - Failure.
 * @note    This function can be used on raw T as well as raw H data (since they use the same format and CRC).
 */
static bool ENS210_IsCrcOk(uint32_t raw);

/*
 * @brief   Verify data validity of the raw temperature or relative humidity.
 * @param   raw         :  Raw  temperature or relative humidity value to be verified (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get).
 * @return  True - Valid,  False - Invalid.
 */
static bool ENS210_IsDataValid(uint32_t raw);

/*
 * @brief   Converts a raw temperature value into Kelvin.
 *          The output value is in Kelvin multiplied by parameter "multiplier".
 * @param   traw        :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier  :   The multiplication factor.
 * @return  The temperature value in 1/multiplier Kelvin.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
static int32_t ENS210_ConvertRawToKelvin(uint32_t traw, int multiplier);

/*
 * @brief   Converts a raw temperature value into Celsius.
 *          The output value is in Celsius multiplied by parameter "multiplier".
 * @param   traw         :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor.
 * @return  The temperature value in 1/multiplier Celsius.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
static int32_t ENS210_ConvertRawToCelsius(uint32_t traw, int multiplier);

/*
 * @brief   Converts a raw temperature value into Fahrenheit.
 *          The output value is in Fahrenheit multiplied by parameter "multiplier".
 * @param   traw         :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor of the converted temperature
 * @return  The temperature value in 1/multiplier Fahrenheit.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, or powers of 2.
 */
static int32_t ENS210_ConvertRawToFahrenheit(uint32_t traw, int multiplier);

/*
 * @brief   Converts a raw relative humidity value into human readable format.
 * @param   hraw         :   The relative humidity value in the raw format (#ENS210_HVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor of the converted relative humidity.
 * @return  The converted relative humidity value
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
static int32_t ENS210_ConvertRawToPercentageH(uint32_t hraw, int multiplier);

/*****************************************************************************
 * Private functions
 ****************************************************************************/

//Compute the CRC-7 of 'val' (which should only have 17 bits)
static uint32_t ENS210_ComputeCRC7(uint32_t val)
{
    //Setup polynomial
    uint32_t pol= CRC7POLY;

    //Align polynomial with data
    pol = pol << (DATA7WIDTH-CRC7WIDTH-1);

    //Loop variable (indicates which bit to test, start with highest)
    uint32_t bit = DATA7MSB;

    //Make room for CRC value
    val = val << CRC7WIDTH;
    bit = bit << CRC7WIDTH;
    pol = pol << CRC7WIDTH;

    //Insert initial vector
    val |= CRC7IVEC;

    //Apply division until all bits done
    while( bit & (DATA7MASK<<CRC7WIDTH) )
    {
        if( bit & val )
        {
            val ^= pol;
        }
        bit >>= 1;
        pol >>= 1;
    }
    return val;
}

//Verify the CRC
static bool ENS210_IsCrcOk(uint32_t raw)
{
    uint32_t crc, data;

    assert(raw <= 0xffffffUL);

    //Extract 7-bit CRC(Bit-17 to Bit-23)
    crc =  (raw >> 17) & 0x7F;

    //Get the raw T/H and data valid indication.
    data =  raw & 0x1ffff;

    return ENS210_ComputeCRC7(data) == crc;
}

//Check the Data Valid Bit
static bool ENS210_IsDataValid(uint32_t raw)
{
    assert(raw <= 0xffffffUL);

    //Bit-16 is data valid bit. It will be set if data is valid
    return (raw & (1UL << 16)) != 0;
}

//Convert raw temperature to Kelvin
//The output value is in Kelvin multiplied by parameter "multiplier"
static int32_t ENS210_ConvertRawToKelvin(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;

    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*K
    //where m is the multiplier, R the raw value and K is temperature in Kelvin.
    //K=R/64 (since raw has format 10.6).
    //m*K =  m*R/64
    return IDIV(t*multiplier, 64);
}

//Convert raw temperature to Celsius
//The output value is in Celsius multiplied by parameter "multiplier"
static int32_t ENS210_ConvertRawToCelsius(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;

    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*C
    //where m is the multiplier, R the raw value and K, C, F temperature in various units.
    //We use C=K-273.15 and K=R/64 (since raw has format 10.6).
    //m*C = m*(K-273.15) = m*K - 27315*m/100 = m*R/64 - 27315*m/100

    return IDIV(t*multiplier, 64) - IDIV(27315L*multiplier, 100);
}

//Convert raw temperature to Fahrenheit
//The output value is in Fahrenheit multiplied by parameter "multiplier"
static int32_t ENS210_ConvertRawToFahrenheit(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;

    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*F
    //where m is the multiplier, R the raw value and K, C, F temperature in various units.
    //We use F=1.8*(K-273.15)+32 and K=R/64 (since raw has format 10.6).

    //m*F = m*(1.8*(K-273.15)+32) = m*(1.8*K-273.15*1.8+32) = 1.8*m*K-459.67*m = 9*m*K/5 - 45967*m/100 = 9*m*R/320 - 45967*m/100
    return IDIV(9*multiplier*t, 320) - IDIV(45967L*multiplier, 100);

    //The first multiplication stays below 32 bits (tRaw:16, multiplier:11, 9:4)
    //The second  multiplication stays below 32 bits (multiplier:10, 45967:16)
}

//Convert raw relative humidity to readable format
//The output value is in % multiplied by parameter "multiplier"
static int32_t ENS210_ConvertRawToPercentageH(uint32_t hraw, int multiplier)
{
    int32_t h;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw relative humidity
    h = hraw & ENS210_T_H_MASK;

    //As raw format is 7.9, to obtain the relative humidity, it must be divided by 2^9
    return IDIV(h*multiplier, 512);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

// wrap the low level function (I2C write, I2C read, WaitMsec) required by ENS210 driver
// this function does not initialize the HW
void ENS210_Init_Driver(ens210_IoFunc_t* pIoFunc){
    assert((pIoFunc != NULL) &&
            (pIoFunc->I2C_Read != NULL) &&
            (pIoFunc->I2C_Write != NULL) &&
            (pIoFunc->WaitMsec != NULL));
    sENS210_Func = *pIoFunc;
    initDriverDone = true;
}

// De-initialize the driver
void ENS210_Deinit_Driver(){
    if (initHwDone)
    {
        /* Deinit HW */
        ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_ENABLE); // skip error management
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

// Initialize ENS210 hardware
ens210_status_t ENS210_Init_Hw(void)
{
    ens210_status_t status = ens210_success;

    if (!initDriverDone) return ens210_noinit;

    /* trick to allow calling internal public functions */
    initHwDone = true;

    //Reset the sensor
    status = ENS210_SysCtrl_Set(ENS210_SYSCTRL_RESET_ENABLE | ENS210_SYSCTRL_LOWPOWER_ENABLE);
    if (status != ens210_success) goto return_status;

    //Wait for ENS210 to complete reset
    sENS210_Func.WaitMsec(ENS210_RESET_WAIT_TIME_MS);

    //Set the run mode of sensors
    status = ENS210_SensRun_Set(ENS210_SENSRUN_T_MODE_SINGLE_SHOT | ENS210_SENSRUN_H_MODE_SINGLE_SHOT);
    if (status != ens210_success) goto return_status;

    return_status:
    if (status != ens210_success) initHwDone = false;
    return status;
}

//Set ENS210 SysCtrl register; enabling reset and/or low power
ens210_status_t ENS210_SysCtrl_Set(uint8_t sysCtrl)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_CTRL, sysCtrl};

    if (!initHwDone) return ens210_noinit;
    assert((sysCtrl & ~(ENS210_SYSCTRL_LOWPOWER_ENABLE | ENS210_SYSCTRL_RESET_ENABLE)) == 0);

    return (ens210_status_t)sENS210_Func.I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof (wBuf));
}

//Get ENS210 SysCtrl register
ens210_status_t ENS210_SysCtrl_Get(uint8_t *sysCtrl)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_CTRL};

    if (!initHwDone) return ens210_noinit;
    assert(sysCtrl != NULL);

    return (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof (wBuf), sysCtrl, sizeof (*sysCtrl));
}

//Get ENS210 SysStat register.
ens210_status_t ENS210_SysStat_Get(uint8_t *sysStat)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_STAT};

    if (!initHwDone) return ens210_noinit;
    assert(sysStat != NULL);

    return (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof (wBuf), sysStat, sizeof (*sysStat));
}

//Set ENS210 SensRun register; set the run mode single shot/continuous for T and H sensors.
ens210_status_t ENS210_SensRun_Set(uint8_t sensRun)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_RUN, sensRun};

    if (!initHwDone) return ens210_noinit;
    assert((sensRun & ~(ENS210_SENSRUN_T_MODE_CONTINUOUS | ENS210_SENSRUN_H_MODE_CONTINUOUS)) == 0);

    return (ens210_status_t)sENS210_Func.I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}

//Get ENS210 SensRun register
ens210_status_t ENS210_SensRun_Get(uint8_t *sensRun)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_RUN};

    if (!initHwDone) return ens210_noinit;
    assert(sensRun != NULL);

    return (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sensRun, sizeof *sensRun);
}

//Set ENS210 SensStart register; starts the measurement for T and/or H sensors.
ens210_status_t ENS210_SensStart_Set(uint8_t sensStart)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_START, sensStart};

    if (!initHwDone) return ens210_noinit;
    assert((sensStart & ~(ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START)) == 0);

    return (ens210_status_t)sENS210_Func.I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}

//Set ENS210 SensStop register; stops the measurement for T and/or H sensors.
ens210_status_t ENS210_SensStop_Set(uint8_t sensStop)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_STOP, sensStop};

    if (!initHwDone) return ens210_noinit;
    assert((sensStop & ~(ENS210_SENSSTOP_T_STOP | ENS210_SENSSTOP_H_STOP)) == 0);

    return (ens210_status_t)sENS210_Func.I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}

//Get ENS210 SensStat register.
ens210_status_t ENS210_SensStat_Get(uint8_t *sensStat)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_STAT};

    if (!initHwDone) return ens210_noinit;
    assert(sensStat != NULL);

    return (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sensStat, sizeof *sensStat);
}

//Get ENS210 TVal register; raw measurement data as well as CRC and valid indication
ens210_status_t ENS210_TVal_Get(uint32_t *traw)
{
    uint8_t rBuf[3];
    uint8_t wBuf[] = {ENS210_REG_T_VAL};
    ens210_status_t status;

    if (!initHwDone) return ens210_noinit;
    assert(traw != NULL);

    status = (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *traw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | (uint32_t)rBuf[0];

    return status;
}

//Get ENS210 HVal register; raw measurement data as well as CRC and valid indication
ens210_status_t ENS210_HVal_Get(uint32_t *hraw)
{
    uint8_t rBuf[3];
    uint8_t wBuf[] = {ENS210_REG_H_VAL};
    ens210_status_t status;

    if (!initHwDone) return ens210_noinit;
    assert(hraw != NULL);

    status = (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *hraw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | ((uint32_t)rBuf[0]) << 0;

    return status;
}

//Get ENS210 TVal and Hval registers; raw measurement data as well as CRC and valid indication
ens210_status_t ENS210_THVal_Get(uint32_t *traw, uint32_t *hraw)
{
    uint8_t rBuf[6];
    uint8_t wBuf[] = {ENS210_REG_T_VAL};
    ens210_status_t status;

    if (!initHwDone) return ens210_noinit;
    assert((traw != NULL) && (hraw != NULL));

    // Read 6 bytes starting from ENS210_REG_T_VAL
    status = (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *traw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | (uint32_t)rBuf[0];
    *hraw = ((uint32_t)rBuf[5]) << 16 | ((uint32_t)rBuf[4]) << 8 | (uint32_t)rBuf[3];

    return status;
}

// Get ENS210 Part ID and UID.
ens210_status_t ENS210_Ids_Get(ENS210_Ids_t *ids)
{
    uint8_t rBuf[12];
    uint8_t wBuf[] = {ENS210_REG_PART_ID};
    ens210_status_t status;

    if (!initHwDone) return ens210_noinit;
    assert(ids != NULL);

    // Special procedure needed to read ID's: put device in high power (see datasheet)
    // Set the system in Active mode
    status = ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_DISABLE);
    if (status != ens210_success) goto return_error_status;

    // Wait for sensor to go to active mode
    sENS210_Func.WaitMsec(ENS210_BOOTING_TIME_MS);

    // Get the id's
    status = (ens210_status_t)sENS210_Func.I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);
    if (status != ens210_success) goto return_error_status;

    // Copy id's (hw gives partid in little-endian format)
    ids->partId = ((uint32_t)rBuf[1]) << 8 | ((uint32_t)rBuf[0]) << 0;
    memcpy(&ids->uId[0], &rBuf[4], 8);

    // Go back to low power mode
    status = ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_ENABLE);
    if (status != ens210_success) goto return_error_status;

    // Signal success
    return status;

    return_error_status:
    // Make an attempt to restore low-power
    ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_ENABLE);
    // Return original I2C error
    return status;
}

ens210_status_t ENS210_Measure(uint8_t meas_mode, ens210_meas_data_t *results){
    ens210_status_t status;
    uint8_t meas_status;
    uint32_t T_Raw = 0, H_Raw = 0;
    uint32_t conversion_time_ms = 0;
    uint8_t start_meas = 0;

    if (!initHwDone) return ens210_noinit;
    assert(results != NULL);

    switch (meas_mode){
    case mode_TH :
        conversion_time_ms = ENS210_T_H_CONVERSION_TIME_MS;
        start_meas = ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START;
        break;
    case mode_Tonly :
        conversion_time_ms = ENS210_T_CONVERSION_TIME_MS;
        start_meas = ENS210_SENSSTART_T_START;
        break;
    case mode_Honly :
        conversion_time_ms = ENS210_T_H_CONVERSION_TIME_MS;
        start_meas = ENS210_SENSSTART_H_START;
        break;
    default :
        return ens210_wrong_parameter;
        break;
    }

    //check that the previous measurement is completed
    status = ENS210_SensStat_Get(&meas_status);
    if (status != ens210_success){
        return status;
    }
    if(meas_status != 0){
        // trying to start a measurement too early!
        sENS210_Func.WaitMsec(conversion_time_ms);
    }

    //Start the measurement
    status = ENS210_SensStart_Set(start_meas);
    if(status != ens210_success){
        //Start of measurement failed.
        return status;
    }

    // wait for the measurement to be completed
    sENS210_Func.WaitMsec(conversion_time_ms);

    if (meas_mode == mode_TH){
        //Get the temperature and humidity raw value
        status = ENS210_THVal_Get(&T_Raw, &H_Raw);
    }
    else if (meas_mode == mode_Tonly){
        status = ENS210_TVal_Get(&T_Raw);
    }
    else if (meas_mode == mode_Honly){
        status = ENS210_HVal_Get(&H_Raw);
    }
    if(status != ens210_success){
        //Getting T and/or H values failed
        return status;
    }

    if ((meas_mode == mode_TH) || (meas_mode == mode_Tonly)){
        // Verify the temperature raw value
        if(!ENS210_IsCrcOk(T_Raw))    {
            return ens210_Tdata_CRC_error;
        }
        else if(!ENS210_IsDataValid(T_Raw)){
            return ens210_T_invalid_data;
        }
        else {
            //Convert the raw temperature value to Kelvin
            results->T_Kelvin = ENS210_ConvertRawToKelvin(T_Raw, 1);
            //Convert the raw temperature value to Celsius
            results->T_Celsius = ENS210_ConvertRawToCelsius(T_Raw, 1);
            //Convert the raw temperature value to Fahrenheit
            results->T_Fahrenheit = ENS210_ConvertRawToFahrenheit(T_Raw, 1);
            //Convert the raw temperature value to milli Kelvin
            results->T_mKelvin = ENS210_ConvertRawToKelvin(T_Raw, 1000);
            //Convert the raw temperature value to milli Celsius
            results->T_mCelsius = ENS210_ConvertRawToCelsius(T_Raw, 1000);
            //Convert the raw temperature value to milli Fahrenheit
            results->T_mFahrenheit = ENS210_ConvertRawToFahrenheit(T_Raw, 1000);
        }
    }

    if ((meas_mode == mode_TH) || (meas_mode == mode_Honly)){
        //Verify the relative humidity raw value
        if(!ENS210_IsCrcOk(H_Raw)){
            return ens210_Hdata_CRC_error;
        }
        else if(!ENS210_IsDataValid(H_Raw)){
            return ens210_H_invalid_data;
        }
        else {
            //Convert the raw relative humidity to %
            results->H_Percent = ENS210_ConvertRawToPercentageH(H_Raw, 1);
            //Convert the raw relative humidity to milli%
            results->H_mPercent = ENS210_ConvertRawToPercentageH(H_Raw, 1000);
        }
    }
    return ens210_success;
}
