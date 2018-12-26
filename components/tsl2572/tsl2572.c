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
 * This is the source file for the ambient light sensing (ALS) TSL2572 driver.
 */

#include "tsl2572.h"
#include <assert.h>

/* TSL2572 register addresses */
#define TSL2572_I2C_SLAVE_ADDRESS        0x39
#define TSL2572_DEVICE_ID                0x34    /* TSL25721 */
#define TSL2572_REG_ENABLE               0x00
#define TSL2572_REG_ATIME                0x01
#define TSL2572_REG_WTIME                0x03
#define TSL2572_REG_AILTL                0x04
#define TSL2572_REG_AILTH                0x05
#define TSL2572_REG_AIHTL                0x06
#define TSL2572_REG_AIHTH                0x07
#define TSL2572_REG_PERS                 0x0C
#define TSL2572_REG_CONFIG               0x0D
#define TSL2572_REG_CONTROL              0x0F
#define TSL2572_REG_ID                   0x12
#define TSL2572_REG_STATUS               0x13
#define TSL2572_REG_C0DATA               0x14
#define TSL2572_REG_C0DATAH              0x15
#define TSL2572_REG_C1DATA               0x16
#define TSL2572_REG_C1DATAH              0x17

/* TSL2572 COMMAND register masks */
#define TSL2572_COMMAND_MSK              0x80
#define TSL2572_TYPE_REPEAT_MSK          0x00
#define TSL2572_TYPE_AUTO_INC_MSK        0x20
#define TSL2572_TYPE_SPL_FN_MSK          0x60
#define TSL2572_ADD_ALS_INT_CLR_MSK      0x06

/* TSL2572 CONTROL register masks */
#define TSL2572_AGAIN_MASK               0x03

/* TSL2572 CONFIG register masks */
#define TSL2572_AGL_MSK                  0x04
#define TSL2572_WLONG_MSK                0x02

/* TSL2572 PERS register masks */
#define TSL2572_APERS_MSK                0x0F

/* TSL2572 ENABLE register masks */
#define TSL2572_AIEN_MSK                 0x10
#define TSL2572_AEN_MSK                  0x02
#define TSL2572_WEN_MSK                  0x08
#define TSL2572_PON_MSK                  0x01

static const tsl2x7x_settings_t tsl2x7x_default_settings = {
        .als_time = 0xDB,
        .als_gain = 0,
        .als_gain_level = true,
        .wait_time = 74,
        .wlong = false,
        .interrupts_enable = true,
        .persistence = 1,
        .als_thresh_low = 100,
        .als_thresh_high = 300,
        .als_enable = true,
        .wait_enable = true,
        .power_on = true,
        .glass_attenuation = 1
};

/*****************************************************************************
 * Variables
 ****************************************************************************/
static tsl2572_IoFunc_t sTSL2572_Func;
static bool initDriverDone = false;
static bool initHwDone = false;
static uint8_t gain_val = 0;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void TSL2572_Init_Driver(tsl2572_IoFunc_t* pIoFunc){
    assert((pIoFunc != NULL) &&
            (pIoFunc->I2C_Read != NULL) &&
            (pIoFunc->I2C_Write != NULL) &&
            (pIoFunc->WaitMsec != NULL));
    sTSL2572_Func = *pIoFunc;
    initDriverDone = true;
}

void TSL2572_Deinit_Driver(){
    if (initHwDone)
    {
        /* Deinit HW */
        TSL2572_Power_ON(false); /* skip error management */
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

tsl2572_status_t TSL2572_Init_Hw(void){
    uint8_t wBuf[2];
    uint8_t rBuf;
    tsl2572_status_t status;

    if (!initDriverDone) return tsl2572_not_initialized;

    /* trick to allow calling internal public functions */
    initHwDone = true;

    /* check device ID */
    wBuf[0] = TSL2572_REG_ID | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        status = tsl2572_I2C_error;
        goto return_status;
    }
    if (rBuf != TSL2572_DEVICE_ID){
        status = tsl2572_invalid_ID;
        goto return_status;
    }

    /* set ALS gain */
    status = TSL2572_SetALSGain(tsl2x7x_default_settings.als_gain,tsl2x7x_default_settings.als_gain_level);
    if (status != tsl2572_success) goto return_status;

    /* set ALS time */
    status = TSL2572_SetALSTime(tsl2x7x_default_settings.als_time);
    if (status != tsl2572_success) goto return_status;

    /* configure the wait time */
    status = TSL2572_SetWaitTime(tsl2x7x_default_settings.wait_time,tsl2x7x_default_settings.wlong);
    if (status != tsl2572_success) goto return_status;

    /* set the ALS interrupt thresholds */
    status = TSL2572_SetALSThresholds(tsl2x7x_default_settings.als_thresh_low,tsl2x7x_default_settings.als_thresh_high);
    if (status != tsl2572_success) goto return_status;

    /* set the ALS interrupt persistence */
    status = TSL2572_SetALSPersistence(tsl2x7x_default_settings.persistence);
    if (status != tsl2572_success) goto return_status;

    /* ALS Enable */
    status = TSL2572_Enable_ALS(tsl2x7x_default_settings.als_enable);
    if (status != tsl2572_success) goto return_status;

    /* wait enable */
    status = TSL2572_Enable_Wait(tsl2x7x_default_settings.wait_enable);
    if (status != tsl2572_success) goto return_status;

    /* power ON  */
    status = TSL2572_Power_ON(tsl2x7x_default_settings.power_on);
    if (status != tsl2572_success) goto return_status;

    /* clear the sensor IRQ status */
    status = TSL2572_ClearALSInterrupt();
    if (status != tsl2572_success) goto return_status;

    status = TSL2572_EnableALSInterrupts(tsl2x7x_default_settings.interrupts_enable);
    if (status != tsl2572_success) goto return_status;

    return_status:
    if (status != tsl2572_success) initHwDone = false;
    return status;
}

tsl2572_status_t TSL2572_ReadAmbientLight(float *lux){
    uint8_t wBuf[2];
    uint8_t rBuf[4];
    int c0,c1;
    float lux1,lux2,cpl;

    if (!initHwDone) return tsl2572_not_initialized;
    assert(lux != NULL);

    /* read CH0 low data register, CH0 high data register, CH1 low data register and CH1 high data register */
    wBuf[0] = TSL2572_REG_C0DATA | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf[0], sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    c0 = rBuf[1]<<8 | rBuf[0];
    c1 = rBuf[3]<<8 | rBuf[2];

    /* see TSL2572 datasheet */
    cpl = (2.73 * (256-tsl2x7x_default_settings.als_time)) * gain_val / (tsl2x7x_default_settings.glass_attenuation * 60);
    if (tsl2x7x_default_settings.als_gain_level){
        cpl /= 6;
    }
    lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
    lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
    cpl = lux1 >= lux2 ? lux1 : lux2; /* max(lux1, lux2) */
    *lux = ((cpl >= 0.0) ? cpl : 0.0); /* max(cpl, 0.0) */
    return tsl2572_success;
}

tsl2572_status_t TSL2572_SetALSGain(uint8_t AGAIN, bool AGL){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    if ((AGAIN > 1) && (AGL)){
        return tsl2572_wrong_parameter;
    }

    wBuf[0] = TSL2572_REG_CONTROL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    wBuf[1] = AGAIN & TSL2572_AGAIN_MASK;

    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }

    /* read CONFIG register */
    wBuf[0] = TSL2572_REG_CONFIG | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask AGL bit */
    if (AGL){
        wBuf[1] = rBuf | TSL2572_AGL_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_AGL_MSK);
    }

    /* write CONFIG register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }

    if ((AGAIN & TSL2572_AGAIN_MASK) == 0) gain_val = 1;
    else if ((AGAIN & TSL2572_AGAIN_MASK) == 1) gain_val = 8;
    else if ((AGAIN & TSL2572_AGAIN_MASK) == 2) gain_val = 16;
    else if ((AGAIN & TSL2572_AGAIN_MASK) == 3) gain_val = 120;

    return tsl2572_success;
}

tsl2572_status_t TSL2572_SetALSTime(uint8_t ATIME){
    uint8_t wBuf[2];

    if (!initHwDone) return tsl2572_not_initialized;

    wBuf[0] = TSL2572_REG_ATIME | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    wBuf[1] = ATIME;

    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_SetALSThresholds(uint16_t ALS_interrupt_Low_Threshold, uint16_t ALS_interrupt_High_Threshold){
    uint8_t wBuf[5];

    if (!initHwDone) return tsl2572_not_initialized;

    wBuf[0] = TSL2572_REG_AILTL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    wBuf[1] = (uint8_t)(ALS_interrupt_Low_Threshold & 0x00FF);
    wBuf[2] = (uint8_t)((ALS_interrupt_Low_Threshold & 0xFF00) >> 8);
    wBuf[3] = (uint8_t)(ALS_interrupt_High_Threshold & 0x00FF);
    wBuf[4] = (uint8_t)((ALS_interrupt_High_Threshold & 0xFF00) >> 8);

    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_GetALSThresholds(uint16_t *ALS_interrupt_Low_Threshold, uint16_t *ALS_interrupt_High_Threshold){
    uint8_t wBuf[2];
    uint8_t rBuf[4];

    if (!initHwDone) return tsl2572_not_initialized;
    assert((ALS_interrupt_Low_Threshold != NULL) && (ALS_interrupt_High_Threshold != NULL));

    /* read CH0 low data register, CH0 high data register, CH1 low data register and CH1 high data register */
    wBuf[0] = TSL2572_REG_AILTL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf[0], sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    *ALS_interrupt_Low_Threshold = (uint16_t) (rBuf[1]<<8 | rBuf[0]);
    *ALS_interrupt_High_Threshold = (uint16_t) (rBuf[3]<<8 | rBuf[2]);
    return tsl2572_success;
}

tsl2572_status_t TSL2572_SetALSPersistence(uint8_t APERS){
    uint8_t wBuf[2];

    if (!initHwDone) return tsl2572_not_initialized;

    wBuf[0] = TSL2572_REG_PERS | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    wBuf[1] = APERS & TSL2572_APERS_MSK;

    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_SetWaitTime(uint8_t WTIME, bool WLONG){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    wBuf[0] = TSL2572_REG_WTIME | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    wBuf[1] = WTIME;

    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }

    /* read CONFIG register */
    wBuf[0] = TSL2572_REG_CONFIG | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask WLONG bit */
    if (WLONG){
        wBuf[1] = rBuf | TSL2572_WLONG_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_WLONG_MSK);
    }

    /* write CONFIG register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_EnableALSInterrupts(bool AIEN){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    /* read ENABLE register */
    wBuf[0] = TSL2572_REG_ENABLE | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask AIEN bit */
    if (AIEN){
        wBuf[1] = rBuf | TSL2572_AIEN_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_AIEN_MSK);
    }

    /* write ENABLE register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_ClearALSInterrupt(void){
    uint8_t wBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    wBuf = TSL2572_COMMAND_MSK | TSL2572_TYPE_SPL_FN_MSK | TSL2572_ADD_ALS_INT_CLR_MSK;
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_Enable_ALS(bool AEN){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    /* read ENABLE register */
    wBuf[0] = TSL2572_REG_ENABLE | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask AEN bit */
    if (AEN){
        wBuf[1] = rBuf | TSL2572_AEN_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_AEN_MSK);
    }

    /* write ENABLE register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_Enable_Wait(bool WEN){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    /* read ENABLE register */
    wBuf[0] = TSL2572_REG_ENABLE | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask WEN bit */
    if (WEN){
        wBuf[1] = rBuf | TSL2572_WEN_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_WEN_MSK);
    }

    /* write ENABLE register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_Power_ON(bool PON){
    uint8_t wBuf[2];
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;

    /* read ENABLE register */
    wBuf[0] = TSL2572_REG_ENABLE | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }

    /* mask PON bit */
    if (PON){
        wBuf[1] = rBuf | TSL2572_PON_MSK;
    }
    else {
        wBuf[1] = rBuf & ~(TSL2572_PON_MSK);
    }

    /* write ENABLE register */
    if (sTSL2572_Func.I2C_Write(TSL2572_I2C_SLAVE_ADDRESS, &wBuf[0], sizeof (wBuf) )){
        return tsl2572_I2C_error;
    }
    return tsl2572_success;
}

tsl2572_status_t TSL2572_ReadAllRegisters(uint8_t *RegData){
    uint8_t wBuf;
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;
    assert(RegData != NULL);

    /* Read ENABLE register */
    wBuf = TSL2572_REG_ENABLE | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    *RegData = (uint8_t)(rBuf);

    /* Read ATIME register */
    wBuf = TSL2572_REG_ATIME | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read WTIME register */
    wBuf = TSL2572_REG_WTIME | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read AILTL register */
    wBuf = TSL2572_REG_AILTL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read AILTH register */
    wBuf = TSL2572_REG_AILTH | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read AIHTL register */
    wBuf = TSL2572_REG_AIHTL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read AIHTH register */
    wBuf = TSL2572_REG_AIHTH | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read PERS register */
    wBuf = TSL2572_REG_PERS | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read CONFIG register */
    wBuf = TSL2572_REG_CONFIG | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read CONTROL register */
    wBuf = TSL2572_REG_CONTROL | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read ID register */
    wBuf = TSL2572_REG_ID | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read STATUS register */
    wBuf = TSL2572_REG_STATUS | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read C0DATA register */
    wBuf = TSL2572_REG_C0DATA | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read C0DATAH register */
    wBuf = TSL2572_REG_C0DATAH | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read C1DATA register */
    wBuf = TSL2572_REG_C1DATA | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    /* Read C1DATAH register */
    wBuf = TSL2572_REG_C1DATAH | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    return tsl2572_success;
}

tsl2572_status_t TSL2572_ReadCH0(uint8_t *RegData){
    uint8_t wBuf;
    uint8_t rBuf;

    if (!initHwDone) return tsl2572_not_initialized;
    assert(RegData != NULL);

    /* read C0DATA register */
    wBuf = TSL2572_REG_C0DATA | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    *RegData = (uint8_t)(rBuf);

    /* read C0DATAH register */
    wBuf = TSL2572_REG_C0DATAH | TSL2572_COMMAND_MSK | TSL2572_TYPE_AUTO_INC_MSK;
    if (sTSL2572_Func.I2C_Read(TSL2572_I2C_SLAVE_ADDRESS, &wBuf, 1, &rBuf, sizeof (rBuf))){
        return tsl2572_I2C_error;
    }
    RegData++;
    *RegData = (uint8_t)(rBuf);

    return tsl2572_success;
}
