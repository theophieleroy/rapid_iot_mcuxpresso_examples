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
 * This is the header file for the ambient light sensing (ALS) TSL2572 driver.
 */

#ifndef TSL2572_H_
#define TSL2572_H_

/*!
 * @addtogroup tsl2572 TSL2572 Ambient Light Sensor
 * This module provides the API to operate the TSL2572 ambient light sensor through an I2C interface.
 *
 * The basic steps to operate the TSL2572 are as follows:
 * -# Initialize the driver with callback functions (#TSL2572_Init_Driver)
 * -# Initialize the hardware (#TSL2572_Init_Hw)
 * -# Eventually wait for an interrupt
 * -# Read data (#TSL2572_ReadAmbientLight)
 * -# Eventually clear the interrupt
 * -# If the TSL2572 is not needed anymore, de-initialize the driver (#TSL2572_Deinit_Driver). The TSL2572 will
 * be switched off.
 *
 * Example - Sample application code to set TSL2572 without error management
 * -------------------------------------------------------------------------
 * @code
 * #include "tsl2572.h"
 *
 * tsl2572_IoFunc_t tsl2572_fct;
 * float fAmbLight;
 *
 * tsl2572_fct.I2C_Read = I2c_Read;    // callback function for I2C read
 * tsl2572_fct.I2C_Write = I2c_Write;  // callback function for I2C write
 * tsl2572_fct.WaitMsec = WaitMs;      // wait callback function (in ms)
 *
 * TSL2572_Init_Driver(&tsl2572_fct);
 * // Initialize TSL2572 with default settings
 * TSL2572_Init_Hw();
 * // read data
 * TSL2572_ReadAmbientLight(&fAmbLight);
 * printf("The current value is: %d lux\n", fAmbLight);
 * // clear interrupt
 * TSL2572_ClearALSInterrupt();
 * [..]
 * // if the TSL2572 is not needed anymore, the driver can be de-initialized
 * printf("Deinitialize TSL2572 (it will be turned off)\n");
 * TSL2572_Deinit_Driver();
 *
 * @endcode
 * @{
 */

#include <EmbeddedTypes.h>
#include <stdbool.h>

/*! @brief Status return codes. */
typedef enum _tsl2572_status {
    tsl2572_success = 0,         /*!< Success */
    tsl2572_I2C_error = 1,       /*!< I2C read/write error */
    tsl2572_invalid_ID = 2,      /*!< Bad hardware identifier */
    tsl2572_wrong_parameter = 3, /*!< Parameter not allowed */
    tsl2572_not_initialized = 4  /*!< Hardware or driver not initialized */
} tsl2572_status_t;

/*! @brief Structure of external functions or values. */
typedef struct _tsl2572_IoFunc_t
{
    uint8_t   (*I2C_Read)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize); /*!< External I2C read function */
    uint8_t   (*I2C_Write)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize); /*!< External I2C write function */
    void      (*WaitMsec)(uint32_t millisec); /*!< Wait function in milliseconds */
} tsl2572_IoFunc_t, *ptsl2572_IoFunc_t;

/*!
 * @brief Internal structure for TSL2x7x settings
 * @note
 * struct tsl2x7x_default_settings - power on defaults unless
 *                                   overridden by platform data.
 */
typedef struct tsl2x7x_settings_ {
    uint8_t als_time;           /*!< ALS Integration time - multiple of 50ms */
    uint8_t als_gain;           /*!< Index into the ALS gain table */
    bool als_gain_level;        /*!< ALS gain level (when asserted, the 1× and 8× ALS gain (AGAIN) modes are scaled by 0.16) */
    uint8_t wait_time;          /*!< Time between PRX and ALS cycles in 2.7 periods */
    bool wlong;                 /*!< When asserted, the wait cycles are increased by a factor 12× from that programmed in the WTIME register */
    bool interrupts_enable;     /*!< Enable/Disable als interrupts */
    uint8_t persistence;        /*!< H/W Filters, number of 'out of limits' ADC readings ALS */
    uint16_t als_thresh_low;    /*!< CH0 'low' count to trigger interrupt */
    uint16_t als_thresh_high;   /*!< CH0 'high' count to trigger interrupt */
    bool als_enable;            /*!< This bit activates the two ADC channels */
    bool wait_enable;           /*!< This bit activates the wait feature */
    bool power_on;              /*!< This bit activates the internal oscillator to permit the timers and ADC channels to operate */
    float glass_attenuation;    /*!< Scaling factor referred to as glass attenuation (GA), can be used to compensate for attenuation */
} tsl2x7x_settings_t;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*!
 * @brief Initialize TSL2572 driver.
 * @note  Wrap the low level functions (I2C write, I2C read, WaitMsec)
 * @param pIoFunc  Pointer to a structure with external functions */
void TSL2572_Init_Driver(tsl2572_IoFunc_t* pIoFunc);

/*!
 * @brief De-initialize TSL2572 driver.
 *
 */
void TSL2572_Deinit_Driver();

/*!
 * @brief Initialize TSL2572 hardware.
 *
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_Init_Hw(void);

/*!
 * @brief Read ambient light.
 * @note  Sample CH0 and CH1 photodiodes and compute the human eye response to light intensity (in lux)
 *
 * @param lux Ambient light value (in lux)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_ReadAmbientLight(float *lux);

/*!
 * @brief Set ALS gain.
 * @note
 * AGAIN = 0   ALS Gain value = 1 * gain\n
 * AGAIN = 1   ALS Gain value = 8 * gain\n
 * AGAIN = 2   ALS Gain value = 16 * gain\n
 * AGAIN = 3   ALS Gain value = 120 * gain\n
 * AGL = 0     AGAIN = 0 or 1 or 2 or 3 -> scaling by 1\n
 * AGL = 1     AGAIN = 0 or 1 -> scaling by 0.16\n
 * Do not use AGL = 1 with AGAIN = 2 or 3
 *
 * @param  AGAIN ALS gain control value (AGAIN)
 * @param  AGL ALS gain level value (AGL)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_SetALSGain(uint8_t AGAIN, bool AGL);

/*!
 * @brief Set ALS ADC integration time time.
 * @note
 * The ALS time is the ALS ADC integration time.\n
 * ATIME = 0xFF     ALS integration cycles = 1,   time = 2.73ms\n
 * ATIME = 0xF6     ALS integration cycles = 10,  time = 27.3ms\n
 * ATIME = 0xDB     ALS integration cycles = 37,  time = 101ms\n
 * ATIME = 0xC0     ALS integration cycles = 64,  time = 175ms\n
 * ATIME = 0x00     ALS integration cycles = 256, time = 699ms\n
 *
 * @param  ATIME ALS time (ATIME)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_SetALSTime(uint8_t ATIME);

/*!
 * @brief Set ALS interrupt threshold low and threshold high.
 * @note
 * The thresholds refer to C0 photodiode only
 * (C1 is not used to trigger interrupts).
 *
 * @param  ALS_interrupt_Low_Threshold ALS interrupt Low Threshold
 * @param  ALS_interrupt_High_Threshold ALS interrupt High Threshold
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_SetALSThresholds(uint16_t ALS_interrupt_Low_Threshold, uint16_t ALS_interrupt_High_Threshold);

/*!
 * @brief Get ALS interrupt threshold low and threshold high.
 *
 * @param  ALS_interrupt_Low_Threshold Pointer to ALS interrupt Low Threshold
 * @param  ALS_interrupt_High_Threshold Pointer to ALS interrupt High Threshold
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_GetALSThresholds(uint16_t *ALS_interrupt_Low_Threshold, uint16_t *ALS_interrupt_High_Threshold);

/*!
 * @brief Set ALS interrupt persistence filter.
 * @note
 * APERS = 0     every ALS cycle generates an interrupt\n
 * APERS = 1     1 value outside of threshold range generates an interrupt\n
 * APERS = 2     2 consecutive values out of range generates an interrupt\n
 * APERS = 3     3 consecutive values out of range generates an interrupt\n
 * APERS = 4     5 consecutive values out of range generates an interrupt\n
 * APERS = 5     10 consecutive values out of range generates an interrupt\n
 * APERS = 6     15 consecutive values out of range generates an interrupt\n
 * APERS = 7     20 consecutive values out of range generates an interrupt\n
 * APERS = 8     25 consecutive values out of range generates an interrupt\n
 * APERS = 9     30 consecutive values out of range generates an interrupt\n
 * APERS = 10    35 consecutive values out of range generates an interrupt\n
 * APERS = 11    40 consecutive values out of range generates an interrupt\n
 * APERS = 12    45 consecutive values out of range generates an interrupt\n
 * APERS = 13    50 consecutive values out of range generates an interrupt\n
 * APERS = 14    55 consecutive values out of range generates an interrupt\n
 * APERS = 15    60 consecutive values out of range generates an interrupt
 *
 * @param  APERS ALS persistence value (APERS)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_SetALSPersistence(uint8_t APERS);

/*!
 * @brief Set wait time.
 * @note
 * WTIME = 0xFF     Wait time = 2.73ms (WLONG = 0), 0.033s (WLONG = 1)\n
 * WTIME = 0xB6     Wait time = 202ms (WLONG = 0), 2.4s (WLONG = 1)\n
 * WTIME = 0x00     Wait time = 699ms (WLONG = 0), 8.4s (WLONG = 1)\n
 * The Wait time register should be configured before TSL2572_Enable_ALS(true)
 *
 * @param  WTIME Wait time value (WTIME)
 * @param  WLONG Wait time extension by a factor of 12 (WLONG)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_SetWaitTime(uint8_t WTIME, bool WLONG);

/*!
 * @brief Enable ALS interrupts.
 *
 * @param  AIEN ALS interrupts enable (AIEN)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_EnableALSInterrupts(bool AIEN);

/*!
 * @brief Clear ALS interrupts.
 *
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_ClearALSInterrupt(void);

/*!
 * @brief Enable ALS.
 *
 * @param  AEN ALS enable (AEN)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_Enable_ALS(bool AEN);

/*!
 * @brief Enable Wait.
 *
 * @param  WEN Wait enable (WEN)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_Enable_Wait(bool WEN);

/*!
 * @brief Power ON.
 *
 * @param  PON Power ON enable (PON)
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_Power_ON(bool PON);

/*!
 * @brief Read all registers.
 *
 * @param  RegData Pointer to an array of 16 * uint8_t
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_ReadAllRegisters(uint8_t *RegData);

/*!
 * @brief Read CH0.
 *
 * @param  RegData Pointer to an array of 2 * uint8_t
 * @return Status value (0 for success)
 */
tsl2572_status_t TSL2572_ReadCH0(uint8_t *RegData);

/*! @}*/

#endif /* TSL2572_H_ */
