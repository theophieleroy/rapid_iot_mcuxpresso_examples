/*
 * Copyright (c) 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

/*!
 * @file
 * This is the source file for the atmospheric pressure sensor MPL3115 driver.
 */

#include "mpl3115.h"
#include <string.h>
#include <assert.h>

/* unshifted 7-bit I2C address */
#define MPL_I2C_ADDRESS  (0x60)

#define REG_STATUS      (0x00)
#define OUT_P_MSB       (0x01)
#define OUT_P_CSB       (0x02)
#define OUT_P_LSB       (0x03)
#define OUT_T_MSB       (0x04)
#define OUT_T_LSB       (0x05)
#define DR_STATUS       (0x06)
#define OUT_P_DELTA_MSB (0x07)
#define OUT_P_DELTA_CSB (0x08)
#define OUT_P_DELTA_LSB (0x09)
#define OUT_T_DELTA_MSB (0x0A)
#define OUT_T_DELTA_LSB (0x0B)
#define WHO_AM_I        (0x0C)
#define F_STATUS        (0x0D)
#define F_DATA          (0x0E)
#define F_SETUP         (0x0F)
#define TIME_DLY        (0x10)
#define SYSMOD          (0x11)
#define INT_SOURCE      (0x12)
#define PT_DATA_CFG     (0x13)
#define BAR_IN_MSB      (0x14)
#define BAR_IN_LSB      (0x15)
#define P_TGT_MSB       (0x16)
#define P_TGT_LSB       (0x17)
#define T_TGT           (0x18)
#define P_WND_MSB       (0x19)
#define P_WND_LSB       (0x1A)
#define T_WND           (0x1B)
#define P_MIN_MSB       (0x1C)
#define P_MIN_CSB       (0x1D)
#define P_MIN_LSB       (0x1E)
#define T_MIN_MSB       (0x1F)
#define T_MIN_LSB       (0x20)
#define P_MAX_MSB       (0x21)
#define P_MAX_CSB       (0x22)
#define P_MAX_LSB       (0x23)
#define T_MAX_MSB       (0x24)
#define T_MAX_LSB       (0x25)
#define MPL_CTRL_REG1   (0x26)
#define MPL_CTRL_REG2   (0x27)
#define MPL_CTRL_REG3   (0x28)
#define MPL_CTRL_REG4   (0x29)
#define MPL_CTRL_REG5   (0x2A)
#define OFF_P           (0x2B)
#define OFF_T           (0x2C)
#define OFF_H           (0x2D)

// SHIFTS
#define MPL_OS_SHIFT    (3)

#define MPL_TDR_SHIFT   (1)
#define MPL_PDR_SHIFT   (2)
#define MPL_PTDR_SHIFT  (3)

#define MPL_SBYB_SHIFT  (0)
#define MPL_OST_SHIFT   (1)
#define MPL_RST_SHIFT   (2)

#define INT_EN_FIFO_SHIFT   (6)
#define INT_CFG_FIFO_SHIFT  (6)

// MASKS
#define MPL_OS_MASK         (0x7 << MPL_OS_SHIFT)
#define MPL_F_WMRKK_MASK    0xF

/* MPL3115 CTRL_REG1 register */
#define MPL_RST             (1 << 2)
#define MPL_OST             (1 << 1)
#define MPL_SBYB            (1 << 0)

/***********************************************************************************/
/* variables                                                                       */
/***********************************************************************************/
static bool initDriverDone = false;
static bool initHwDone = false;
static mpl3115_IoFunc_t sIoFunc;

/* variable which denotes the sensor working as altimeter/barometer */
static modeMPL_t selectedMode = MPL_MODE_PRESSURE;

static uint16_t
selectedSampleRate  = 0xA5A5,
selectedDelay       = 0xA5A5;

/* oversample factor */
static uint16_t overSampleFactors[] = { 1,  2,  4,  8, 16,  32,  64, 128 };
/* min. time between data samples in [ms] */
static uint16_t overSampleDelays[]  = { 6, 10, 18, 34, 66, 130, 258, 512 };

static settingsMPL_t settings;


/*****************************************************************************
 * Static functions
 ****************************************************************************/

/*
 * @brief Initialize the internal structures.
 *
 * @param mplSettings Pointer to the internal settings structure
 * @return Status value (0 for success)
 */
mpl_status_t MPL_Init(const settingsMPL_t* mplSettings)
{
    mpl_status_t status = MPL_SUCCESS;

    memcpy( (void*)&settings, (void*)mplSettings, sizeof(settings) );

    // reset all registers to POR values
    if (MPL_SoftReset()!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set the mode
    else if (MPL_SetMode(settings.mode)!=MPL_SUCCESS) status =  MPL_INIT_ERROR;

    // set over-sampling
    else if (MPL_SetOversampleRate(settings.oversample)!=MPL_SUCCESS) status =  MPL_INIT_ERROR;

    // enable the flags
    else if (MPL_EnableEventFlags()!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set auto acquisition time step
    else if (MPL_SetAutoAcquisitionTime(settings.autoAcquisitionTime)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set offset pressure correction
    else if (MPL_SetOffsetPressure(settings.pressureOffset)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set offset altitude correction
    else if (MPL_SetOffsetAltitude(settings.altitudeOffset)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set offset temperature correction
    else if (MPL_SetOffsetTemperature(settings.tempOffset)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // setup FIFO register mode and event count to trigger interrupt
    else if (MPL_SetFifoMode(settings.fifoMode, settings.fifoWatermark)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // set FIFO interrupt and output pin (INT1 or INT2)
    else if (MPL_SetFifoInterrupt(settings.fifoINTpin)!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    // goto active state
    else if (MPL_SetActive()!=MPL_SUCCESS) status = MPL_INIT_ERROR;

    return status;
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

void MPL3115_Init_Driver(mpl3115_IoFunc_t* pIoFunc)
{
    assert((pIoFunc != NULL) &&
            (pIoFunc->I2C_Read != NULL) &&
            (pIoFunc->I2C_Write != NULL) &&
            (pIoFunc->WaitMsec != NULL));
    sIoFunc = *pIoFunc;
    initDriverDone = true;
}

void MPL3115_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        MPL_GotoStandby(); /* skip error management */
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

mpl_status_t MPL3115_Init_Hw()
{
    settingsMPL_t MPLsettings;
    mpl_status_t status = MPL_SUCCESS;

    if (!initDriverDone) return MPL_INIT_ERROR;

    /* trick to allow calling internal public functions */
    initHwDone = true;

    MPLsettings.mode = MPL_MODE_PRESSURE;
    MPLsettings.oversample = MPL_OS_0;              // oversampling = 1
    MPLsettings.autoAcquisitionTime = MPL_ST_0;     // Auto acquisition time = 1s
    MPLsettings.pressureOffset = 0;                 // Offset pressure correction (8 bits signed integer)
    MPLsettings.altitudeOffset = 127;               // Offset altitude correction = 128m (signed 8 bits integer)
    MPLsettings.tempOffset = 0;                     // Offset temperature correction (0.0625°C/LSB)
    MPLsettings.fifoMode = FIFO_DISABLED;           // FIFO mode disabled
    MPLsettings.fifoWatermark = 5;                  // 6 bits to set the number of FIFO samples required to trigger a watermark interrupt.
    MPLsettings.fifoINTpin = FIFO_INT1;             // set pin INT1 as output for FIFO interrupt

    /* initialize the sensor */
    status = MPL_Init(&MPLsettings);
    if (status != MPL_SUCCESS) initHwDone = false;
    return status;

}

mpl_status_t MPL_SoftReset()
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1, MPL_RST};

    if (!initHwDone) return MPL_INIT_ERROR;

    // Reset all registers to POR value
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!= MPL_SUCCESS) return MPL_ERROR;
    else
    {
        // by the end of the boot process the RST bit should be de-asserted to 0 => need to wait before next operation
        sIoFunc.WaitMsec(1); // 1ms delay
        return MPL_SUCCESS;
    }
}

mpl_status_t MPL_ToggleOneShot()
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // clear OST bit and write it back
    wBuf[1] = rBuf & ~(MPL_OST);
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    // read again settings, just to be safe
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // set OST bit and write it back
    wBuf[1] = rBuf | MPL_OST;
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_GetID(uint8_t* sensorID)
{
    uint8_t wBuf[1] = {WHO_AM_I};

    if (!initHwDone) return MPL_INIT_ERROR;
    assert(sensorID != NULL);

    // read device ID = WHO_AM_I register (address 0x0C)
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, sensorID, 1)!=MPL_SUCCESS) return MPL_ERROR;
    return MPL_SUCCESS;
}

mpl_status_t MPL_SetMode (modeMPL_t mode)
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // set/clear ALT bit
    switch (mode)
    {
    case MPL_MODE_PRESSURE: {
        wBuf[1] = rBuf & ~( 1 << 7 ); // Clear ALT bit
        break;
    }
    case MPL_MODE_ALTITUDE: {
        wBuf[1] = rBuf | ( 1 << 7 ); // Set ALT bit
        break;
    }
    case MPL_MODE_TEMPERATURE: {
        break;
    }
    default:
        return MPL_NOT_SUPPORTED;
    }

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;
    // update the static variable
    selectedMode = mode;
    return MPL_SUCCESS;
}

mpl_status_t MPL_GotoStandby()
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // clear SBYB bit for entering stand-by mode
    wBuf[1] = rBuf & ~MPL_SBYB;
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetActive()
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // set SBYB bit for entering active mode
    wBuf[1] = rBuf | MPL_SBYB;
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetOversampleRate (uint8_t sampleRate)
{
    uint8_t wBuf[2] = {MPL_CTRL_REG1};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // rate cannot be larger than 7
    if (sampleRate > 7) sampleRate = 7;

    selectedSampleRate = overSampleFactors[sampleRate];
    selectedDelay      = overSampleDelays[sampleRate];

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // clear out old OS bits
    wBuf[1] = rBuf & ~MPL_OS_MASK;
    // mask in new OS bits
    wBuf[1] |= ( sampleRate << MPL_OS_SHIFT );

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t  MPL_SetAutoAcquisitionTime (uint8_t sampleTime)
{
    uint8_t wBuf[2] = {MPL_CTRL_REG2};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read current settings
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // mask in new ST bits. sampleTime cannot be larger than 15 Giving a range of 1s to 9.1 hours
    wBuf[1] |= (sampleTime > 15) ? 15 : sampleTime;

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_EnableEventFlags()
{
    uint8_t wBuf[] = {PT_DATA_CFG, 0x07};

    if (!initHwDone) return MPL_INIT_ERROR;

    // write settings
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetOffsetPressure (int8_t pressOffset)
{
    // Convert input to 2's complement number
    uint8_t wBuf[] = {OFF_P, pressOffset};

    if (!initHwDone) return MPL_INIT_ERROR;

    // write settings
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetOffsetAltitude (int8_t altitudeOffset)
{
    uint8_t wBuf[] = {OFF_H, altitudeOffset};

    if (!initHwDone) return MPL_INIT_ERROR;

    // write settings
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetOffsetTemperature (int8_t temperatureOffset)
{
    // Convert input to 2's complement number
    uint8_t wBuf[] = {OFF_T, temperatureOffset};

    if (!initHwDone) return MPL_INIT_ERROR;

    // write settings
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetFifoMode (modeFIFO_t fMode, uint8_t fWmrk)
{
    uint8_t wBuf[2] = {F_SETUP};
    wBuf[1] = (fMode << 6) | (fWmrk & MPL_F_WMRKK_MASK);

    if (!initHwDone) return MPL_INIT_ERROR;

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_SetFifoInterrupt (pinINT_t pinINT)
{
    uint8_t wBuf[2] = {MPL_CTRL_REG4};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read CTRL4 register & set INT_EN_FIFO
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;
    wBuf[1] = rBuf | (1 << INT_EN_FIFO_SHIFT);

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    // read CTRL5 register & Route interrupt to either INT1 or INT2 pin
    wBuf[0] = MPL_CTRL_REG5;
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    wBuf[1] = (pinINT == FIFO_INT1) ? rBuf | (1 << INT_CFG_FIFO_SHIFT) : rBuf &~(1 << INT_CFG_FIFO_SHIFT);
    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_DisableFifoInterrupt ()
{
    uint8_t wBuf[2] = {MPL_CTRL_REG4};
    uint8_t rBuf;

    if (!initHwDone) return MPL_INIT_ERROR;

    // read CTRL4 register & disable INT_EN_FIFO
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;
    wBuf[1] = rBuf & ~(1 << INT_EN_FIFO_SHIFT);

    if (sIoFunc.I2C_Write(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 2)!=MPL_SUCCESS) return MPL_ERROR;

    return MPL_SUCCESS;
}

mpl_status_t MPL_GetFifoStatus(uint8_t* fifoStatus)
{
    uint8_t wBuf[1] = {F_STATUS};

    if (!initHwDone) return MPL_INIT_ERROR;
    assert(fifoStatus != NULL);

    // read device ID = WHO_AM_I register (address 0x0C)
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, fifoStatus, 1)!=MPL_SUCCESS) return MPL_ERROR;
    return MPL_SUCCESS;
}

mpl_status_t MPL_ReadRawData (modeMPL_t mode, int32_t* sensorData)
{
    uint8_t wBuf[2] = {REG_STATUS};
    uint8_t rBuf[5];
    uint8_t dataReadyFlag,
    dataReadyRegAddr,
    bytesToRead = 0;
    int16_t counter = 0;

    if (!initHwDone) return MPL_INIT_ERROR;
    assert(sensorData != NULL);

    /* set the new working mode, if given one */

    if  (( mode > MPL_MODE_TEMPERATURE ) && ( mode < MPL_MODE_CURRENT )) return MPL_INIT_ERROR;
    else if (( MPL_MODE_CURRENT != mode ) && ( selectedMode != mode ))
    {
        // goto standby
        if (MPL_GotoStandby()!=MPL_SUCCESS) return MPL_ERROR;
        // set the mode
        if (MPL_SetMode(mode)!=MPL_SUCCESS) return MPL_ERROR;
        // goto active state
        if (MPL_SetActive()!=MPL_SUCCESS) return MPL_ERROR;
        sIoFunc.WaitMsec(10);   // Wait 10ms
    }

    /* prepare for reading data */

    switch (selectedMode)
    {
    case MPL_MODE_PRESSURE:
    case MPL_MODE_ALTITUDE: {
        dataReadyFlag     = 1 << MPL_PDR_SHIFT;
        dataReadyRegAddr  = OUT_P_MSB;
        bytesToRead       = 3;
        break;
    }

    case MPL_MODE_TEMPERATURE: {
        dataReadyFlag     = 1 << MPL_TDR_SHIFT;
        dataReadyRegAddr  = OUT_T_MSB;
        bytesToRead       = 2;
        break;
    }

    default:  {}
    }

    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;

    // check PDR/PTR bit; if it's not set, toggle OST
    if (0 == (rBuf[0] & dataReadyFlag))
    {
        // toggle the OST bit, causing the sensor to immediately take another reading
        if (MPL_ToggleOneShot()!=MPL_SUCCESS) return MPL_ERROR;
        sIoFunc.WaitMsec(10);   // Wait 10ms
    }

    // wait for PDR/PTR bit, which indicates that we have new data
    while (1)
    {
        if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;
        if ((0==(rBuf[0] & dataReadyFlag)) & ( ++counter > 5 )) return MPL_TIMEOUT;
        else break;
        sIoFunc.WaitMsec(100);  // Wait 100ms
    }

    /* read sensor data */

    wBuf[0] = dataReadyRegAddr;
    if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, wBuf, 1, rBuf, bytesToRead)!=MPL_SUCCESS) return MPL_ERROR;

    switch ( selectedMode )
    {
    // pressure value is a Q18.2 right-aligned number in [Pa]
    case MPL_MODE_PRESSURE:
        // altitude value is a Q16.4 right-aligned number in [m]
    case MPL_MODE_ALTITUDE:     {
        *sensorData = (int32_t) ((rBuf[0] << 16) | (rBuf[1] << 8) | rBuf[2]) >> 4;
        break;
    }

    // temperature value is a Q8.4 right-aligned number in [C]
    case MPL_MODE_TEMPERATURE:  {
        *sensorData = (int32_t) (((rBuf[0] << 8) | rBuf[1]) >> 4);
        break;
    }
    default:  {}
    }

    return MPL_SUCCESS;
}

mpl_status_t MPL_Dump(uint8_t *sensorReg)
{
    uint8_t wBuf= 0;
    uint8_t rBuf;
    uint8_t i;

    if (!initHwDone) return MPL_INIT_ERROR;
    assert(sensorReg != NULL);

    // read all registers
    for (i=0; i<45; i++)
    {
        if (sIoFunc.I2C_Read(MPL3115_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, 1)!=MPL_SUCCESS) return MPL_ERROR;
        *sensorReg = rBuf;
        wBuf++;
        sensorReg++;
    }
    return MPL_SUCCESS;
}
