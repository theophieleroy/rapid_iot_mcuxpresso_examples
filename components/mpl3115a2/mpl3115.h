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
 * This is the header file for the atmospheric pressure sensor MPL3115 driver.
 */

#ifndef MPL3115_H_
#define MPL3115_H_

/*!
 * @addtogroup mpl3115 MPL3115 atmospheric pressure sensor
 * This module provides the API to operate the MPL3115 atmospheric pressure sensor through an I2C interface.
 *
 * The MPL3115A2 is a compact, piezoresistive, absolute pressure sensor with an I2C
 * digital interface. MPL3115A2 has a wide operating range of 20 kPa to 110 kPa, a range
 * that covers all surface elevations on earth. The MEMS is temperature compensated
 * utilizing an on-chip temperature sensor. The pressure and temperature data is fed into
 * a high resolution ADC to provide fully compensated and digitized outputs for pressure
 * in Pascals and temperature in °C.
 *
 * Usage
 *-----------------------------------------------------------------------------------------------------------
 *
 * Initialization:
 * @code
 *
 *  #include "mpl3115.h"
 *
 *  mpl3115_IoFunc_t MPL3115_sensor;
 *  MPL3115_sensor.I2C_Read = App_I2C1_Read;
 *  MPL3115_sensor.I2C_Write = App_I2C1_Write;
 *  MPL3115_sensor.WaitMsec = App_WaitMsec;
 *
 *  MPL3115_Init_Driver(&MPL3115_sensor);
 *  MPL3115_Init_Hw();
 * @endcode
 *
 * Basic Operation:
 * @code
 *
 *  int32_t data;
 *
 *  if (MPL_ReadRawData (MPL_MODE_PRESSURE, &data) == 0)
 *  {
 *        data /= 400; // in HPa (LSB = 0.25Pa)
 *  }
 *
 * @endcode
 *
 * @{
 */

#include <stdbool.h>
#include <EmbeddedTypes.h>

/*! @brief Structure of external functions or values. */
typedef struct _mpl3115_IoFunc_t
{
    uint8_t   (*I2C_Read)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize);     /*!< Function pointer to I2C Read function. */
    uint8_t   (*I2C_Write)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize);                                         /*!< Function pointer to I2C Write function. */
    void      (*WaitMsec)(uint32_t millisec);                                                                                   /*!< Function pointer to waitMsec function  */
} mpl3115_IoFunc_t, *pmpl3115_IoFunc_t;

typedef int16_t mE_t;

/*! @brief Status return codes. */
typedef enum
{
    MPL_SUCCESS,            /*!< Function ran successfully. */
    MPL_ERROR,              /*!< Error in running function. */
    MPL_PROTOCOL_ERROR,     /*!< Protocol error has occurred. */
    MPL_INIT_ERROR,         /*!< Initialization error has occurred. */
    MPL_TIMEOUT,            /*!< MPL function has timed out */
    MPL_NOT_SUPPORTED       /*!< Not Supported */
} mpl_status_t;

/*! @brief Oversampling factor */
typedef enum {
    MPL_OS_0,               /*!< Oversample Ratio = 1    */
    MPL_OS_1,               /*!< Oversample Ratio = 2    */
    MPL_OS_2,               /*!< Oversample Ratio = 4    */
    MPL_OS_3,               /*!< Oversample Ratio = 8    */
    MPL_OS_4,               /*!< Oversample Ratio = 16   */
    MPL_OS_5,               /*!< Oversample Ratio = 32   */
    MPL_OS_6,               /*!< Oversample Ratio = 64   */
    MPL_OS_7                /*!< Oversample Ratio = 128  */
} overSampleMPL_t;

/*! @brief Auto acquisition time step : power(2; MPL_ST_X) */
typedef enum {
    MPL_ST_0,               /*!< Auto acquistion time step = 1 second       */
    MPL_ST_1,               /*!< Auto acquistion time step = 2 seconds      */
    MPL_ST_2,               /*!< Auto acquistion time step = 4 seconds      */
    MPL_ST_3,               /*!< Auto acquistion time step = 8 seconds      */
    MPL_ST_4,               /*!< Auto acquistion time step = 16 seconds     */
    MPL_ST_5,               /*!< Auto acquistion time step = 32 seconds     */
    MPL_ST_6,               /*!< Auto acquistion time step = 64 seconds     */
    MPL_ST_7,               /*!< Auto acquistion time step = 128 seconds    */
    MPL_ST_8,               /*!< Auto acquistion time step = 256 seconds    */
    MPL_ST_9,               /*!< Auto acquistion time step = 512 seconds    */
    MPL_ST_10,              /*!< Auto acquistion time step = 1024 seconds   */
    MPL_ST_11,              /*!< Auto acquistion time step = 2048 seconds   */
    MPL_ST_12,              /*!< Auto acquistion time step = 4096 seconds   */
    MPL_ST_13,              /*!< Auto acquistion time step = 8192 seconds   */
    MPL_ST_14,              /*!< Auto acquistion time step = 16384 seconds  */
    MPL_ST_15               /*!< Auto acquistion time step = 32768 seconds  */
} autoAcquisitionTime_t;

/*! @brief Device Mode */
typedef enum {
    MPL_MODE_PRESSURE    = 0,   /*!< Device is in barometer mode. It reports an absolute pressure.  */
    MPL_MODE_ALTITUDE    = 1,   /*!< Device is in altimeter mode. The pressure data is converted to equivalent altitude based on US standard atmosphere */
    MPL_MODE_TEMPERATURE = 2,   /*!< This mode provides temperature from a high resolution temperature sensor. */
    MPL_MODE_CURRENT     = 0xFF
} modeMPL_t;

/*! @brief Fifo Mode */
typedef enum {
    FIFO_DISABLED,        /*!< FIFO is disabled (reset value) */
    FIFO_CIRCULAR,        /*!< FIFO contains the most recent samples when overflowed (circular buffer). Oldest sample is discarded to be replaced by new sample*/
    FIFO_STOP_OVERFLOW    /*!< FIFO stops accepting new samples when overflowed */
} modeFIFO_t;

/*! @brief Pin to route FIFO interrupt */
typedef enum {
    FIFO_INT1    = 1,   /*!< FIFO Interrupt routed to INT1 pin */
    FIFO_INT2    = 0    /*!< FIFO Interrupt routed to INT2 pin */
} pinINT_t;

/* structure that contains MPL settings */
typedef struct {
    modeMPL_t                 mode;                    /*!< device mode, altimeter or barometer */
    overSampleMPL_t           oversample;              /*!< oversampling ratio */
    autoAcquisitionTime_t     autoAcquisitionTime;     /*!< Auto acquisition time step */
    int8_t                    pressureOffset;          /*!< Offset pressure correction (signed: 4 Pa/LSB) */
    int8_t                    altitudeOffset;          /*!< Offset altitude correction (signed: 1m/LSB) */
    int8_t                    tempOffset;              /*!< Offset temperature correction -8°C (signed: 0.0625°C/LSB) */
    modeFIFO_t                fifoMode;                /*!< FIFO mode */
    uint8_t                   fifoWatermark;           /*!< These (6) bits set the number of FIFO samples required to trigger a watermark interrupt. */
    pinINT_t                  fifoINTpin;              /*!< Pin to route FIFO interrupt */
} settingsMPL_t;

/* MPL3115 I2C slave address */
#define MPL3115_I2C_SLAVE_ADDRESS                (uint8_t)0x60

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * @brief Initialize MPL3115 driver.
 * @note  Wrap the low level functions (I2C write, I2C read, WaitMsec).
 * @param pIoFunc  Pointer to a structure with external functions
 */
void MPL3115_Init_Driver(mpl3115_IoFunc_t* pIoFunc);

/*!
 * @brief De-initialize MPL3115 driver.
 *
 */
void MPL3115_Deinit_Driver();

/*!
 * @brief Initialize MPL3115 hardware.
 * @return Status value (0 for success)
 */
mpl_status_t MPL3115_Init_Hw();

/*!
 * @brief Soft reset.
 * @note
 * The reset mechanism can be enabled in standby and active mode.\n
 * When this bit is enabled, the reset mechanism resets all functional block
 * registers and loads the respective internal registers with default values
 * If the system was already in standby mode, the reboot process will\n
 * immediately begin; else if the system was in active mode, the boot mechanism
 * will automatically transition the system from active mode to standby mode,
 * and only then can the reboot process begin.
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SoftReset();

/*!
 * @brief Toggle the OST bit.
 * @note Clears then sets the OST bit which causes the sensor to immediately
 * take another reading, necessary to sample faster than 1Hz.
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_ToggleOneShot();

/*!
 * @brief Read the chip ID.
 *
 * @param sensorID Chip ID value
 * @return Status value (0 for success)
 */
mpl_status_t MPL_GetID(uint8_t* sensorID);

/*!
 * @brief Set the device mode (barometer or altimeter).
 *
 * @param mode 1 - barometer
 *             0 - altimeter
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetMode (modeMPL_t mode);

/*!
 * @brief Put the sensor in stand-by mode.
 * @note It is needed to modify major control registers.
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_GotoStandby ();

/*!
 * @brief Put the sensor in active mode.
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetActive();

/*!
 * @brief Set the over-sample rate.
 * @note
 * Datasheet calls for 128, but you can set it from 1 to 128 samples.
 * The higher the oversample rate, the greater the time between data samples.
 *
 * @param sampleRate Over-sample rate value
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetOversampleRate (uint8_t sampleRate);

/*!
 * @brief Set the auto-acquisition time step.
 * @note
 * Reset value = 0.\n
 * Step is power(2; sampleTime).
 *
 * @param sampleTime Sample time value
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetAutoAcquisitionTime (uint8_t sampleTime);

/*!
 * @brief Enable pressure and temperature measurement event flags.
 * @note This is recommended in datasheet during setup.
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_EnableEventFlags();

/*!
 * @brief Set the offset pressure correction.
 * @note
 * Pressure user accessible offset trim value number.\n
 * The user offset registers may be adjusted to enhance accuracy and optimize
 * the system performance.\n
 * Range is from −512 to +508 Pa, 4 Pa/LSB.
 *
 * @param pressOffset Pressure offset correction value.
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetOffsetPressure (int8_t pressOffset);

/*!
 * @brief Set the offset altitude correction.
 * @note
 * Altitude user accessible offset trim value number.\n
 * The user offset register provides user adjustment to the vertical height of
 * the altitude output.\n
 * The range of values are from −128 to +127 meters.
 *
 * @param altitudeOffset Altitude offset correction value.
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetOffsetAltitude (int8_t altitudeOffset);

/*!
 * @brief Set the offset temperature correction.
 * @note
 * Temperature user accessible offset trim value number.\n
 * The range of values is from −8 to +7.9375 °C, 0.0625 °C/LSB.
 *
 * @param temperatureOffset Temperature offset correction value.
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetOffsetTemperature (int8_t temperatureOffset);

/*!
 * @brief Set the FIFO mode
 * @note
 * It can be configured in either circular buffer or in overflow mode.\n
 * In circular buffer mode, a watermark can be set to trigger a flag event.
 * Exceeding the watermark, count does not stop the FIFO from accepting new
 * data, the oldest data is overwritten.
 *
 * @param fMode FIFO mode
 * @param fWmrk Watermark
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetFifoMode (modeFIFO_t fMode, uint8_t fWmrk);

/*!
 * @brief Setup the FIFO interrupt and route it to pin INT1 or INT2
 *
 * @param pinINT Interrupt pin selection (INT1 or INT2)
 * @return Status value (0 for success)
 */
mpl_status_t MPL_SetFifoInterrupt (pinINT_t pinINT);

/*!
 * @brief Disable the FIFO interrupt
 *
 * @return Status value (0 for success)
 */
mpl_status_t MPL_DisableFifoInterrupt();

/*!
 * @brief Read the FIFO status register.
 *
 * @param fifoStatus Pointer to FIFO status
 * @return Status value (0 for success)
 */
mpl_status_t MPL_GetFifoStatus(uint8_t* fifoStatus);

/*!
 * @brief Read sensor raw data
 *
 * @param  mode Sensor mode (pressure, altitude...)
 * @param  sensorData Pointer to the sensor data
 * @return Status value (0 for success)
 */
mpl_status_t MPL_ReadRawData (modeMPL_t mode, int32_t* sensorData);

/*!
 * @brief Read all the chip registers
 *
 * @param  sensorReg Pointer to the register dump
 * @return Status value (0 for success)
 */
mpl_status_t MPL_Dump(uint8_t* sensorReg);

/*! @}*/

#endif /* MPL3115_H_ */
