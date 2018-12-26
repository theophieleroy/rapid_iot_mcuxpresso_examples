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
 * @file battery.c
 * This is the source file for the battery sensor driver.
 */

#include "battery.h"
#include "peripherals.h"
#include "board.h"
#include "fsl_vref.h"
#include <assert.h>

/*****************************************************************************
 * Variables
 ****************************************************************************/

static bool initDriverDone = false;
static bool initHwDone = false;
static battery_fct_t FCT_BATTERY;

/*****************************************************************************
 * Static functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void BatterySensor_Init_Driver(ptbattery_fct_t FCT)
{
    assert((FCT != NULL) && (FCT->WaitMsec != NULL));
    FCT_BATTERY = *FCT;
    initDriverDone = true;
}

void BatterySensor_Deinit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        // nothing to be done
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

battery_status_t BatterySensor_Init_Hw()
{
    if (!initDriverDone) return battery_status_noinit;

    /* VREF HW init */
    vref_config_t vrefUserConfig;
    VREF_GetDefaultConfig(&vrefUserConfig);
    vrefUserConfig.bufferMode = kVREF_ModeHighPowerBuffer;
    VREF_Init(VREF, &vrefUserConfig); /* initializes and configures the VREF module */

    BAT_SENS_OFF;
    initHwDone = true;
    return battery_status_success;
}

battery_status_t BatterySensor_GetState(uint8_t *batteryPercentLevel, uint8_t *batteryChargingState)
{
    adc16_channel_config_t adc16ChannelConfigStruct = {
            .channelNumber = BAT_SENS_CHANNEL,
            .enableInterruptOnConversionCompleted = false,
            .enableDifferentialConversion = false
    };
    uint32_t batsens_meas;
    uint32_t vbat_mv;

    if (!initHwDone) return battery_status_noinit;
    assert((batteryPercentLevel != NULL) && (batteryChargingState != NULL));

    BAT_SENS_ON;
    FCT_BATTERY.WaitMsec(100); /* 100ms is based on the RC filter on BAT_SENS, 100Kohms+0.1uF (10ms time constant) */
    BOARD_Init_BATSENS();
    ADC16_EnableHardwareTrigger(BAT_SENS_BASE, false); /* Make sure the software trigger is used. */

    ADC16_SetChannelConfig(BAT_SENS_BASE, BAT_SENS_GROUP, &adc16ChannelConfigStruct); // Software Trigger measurement
    while (0U == (kADC16_ChannelConversionDoneFlag & ADC16_GetChannelStatusFlags(BAT_SENS_BASE, BAT_SENS_GROUP))) {}
    batsens_meas = ADC16_GetChannelConversionValue(BAT_SENS_BASE, BAT_SENS_GROUP);

    /* conversion from batsens_meas based on resistive divider values */
    vbat_mv = (batsens_meas * 12)/10;
    /* apply a hysteresis to get rid of the instantaneous variation (noise on power supply) */
    vbat_mv /= HYSTERESIS_MV;
    vbat_mv *= HYSTERESIS_MV;

    if (vbat_mv >= VBAT_MV_MAX)
    {
        *batteryPercentLevel = 100; /* max 100% */
    }
    else if (vbat_mv > VBAT_MV_MIN)
    {
        /* linearization of the charging curve */
        *batteryPercentLevel = (uint8_t)(((vbat_mv-VBAT_MV_MIN)*100)/(VBAT_MV_MAX-VBAT_MV_MIN));
        if (*batteryPercentLevel == 0) *batteryPercentLevel = 1; /* min 1% */
    }
    else /* (vbat_mv <= VBAT_MV_MIN) */
    {
        *batteryPercentLevel = 1; /* min 1% */
    }

    /* (*batteryChargingState) is not modified to reflect the actual hardware behavior (whatever the battery level) */
    *batteryChargingState = (uint8_t)BAT_CHARGING;
    BAT_SENS_OFF;

    return battery_status_success;
}
