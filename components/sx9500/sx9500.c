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
 * @file sx9500.c
 * This is the source file for the SX9500 touch controller driver.
 */

#include "sx9500.h"
#include "fsl_debug_console.h"
#include <assert.h>

/*****************************************************************************
 * Variables
 ****************************************************************************/
static bool initDriverDone = false;
static bool initHwDone = false;
static sx9500_fct_t FCT_SX9500;
static bool m_txen;

/* Define registers that need to be initialized to values different than
 * default
 */
typedef struct smtc_reg_data {
    unsigned char reg;
    unsigned char val;
}smtc_reg_data_t;

static smtc_reg_data_t sx9500_i2c_reg_setup[] = {
    {
        .reg = SX9500_REG_IRQMSK,
        .val = 0x60,  /* CLOSEIRQEN[6]=1 (close interrupt is on),            */
                      /* FARIRQEN[5]=1 (far interrupt is on),                */
                      /* COMPDONEIRQEN[4]=0 (compensation interrupt is off), */
                      /* CONVDONEIRQEN[3]=0 (conversion interrupt is off)    */
    },
    {
        .reg = SX9500_REG_PROXCTRL1,
        .val = 0x03,  /* SHIELDEN[7:6]=0 (no shield),              */
                      /* RANGE[1:0]=3 (small, +/-2.5pF Full Scale) */
    },
    {
        .reg = SX9500_REG_PROXCTRL2,
        .val = 0x27,  /* GAIN[6:5]=1 (digital gain x2),          */
                      /* FREQ[4:3]=0 (83kHz sampling frequency), */
                      /* RESOLUTION[2:0]=0 (finest resolution)   */
    },
    {
        .reg = SX9500_REG_PROXCTRL3,
        .val = 0x41,  /* DOZEEN[6]=1 (enables doze mode),   */
                      /* DOZEPERIOD[5:4]=0 (2*scan period), */
                      /* RAWFILT[1:0]=1 (Low)               */
    },
    {
        .reg = SX9500_REG_PROXCTRL4,
        .val = 0x80,  /* AVGTHRESH[7:0]=0x80 (threshold triggering compensation = +/-128*value (typ between 16384 and 24576) */
    },
    {
        .reg = SX9500_REG_PROXCTRL5,
        .val = 0x0F,  /* AVGDEB[7:6]=0 (debounce=off),               */
                      /* AVGNEGFILT[5:3]=1 (lowest negative filter), */
                      /* AVGPOSFILT[2:0]=7 (highest positive filter) */
    },
    {
        .reg = SX9500_REG_PROXCTRL6,
        .val = 0x06,  /* PROXTHRESH[4:0]=6 (sensitivity=120) */
    },
    {
        .reg = SX9500_REG_PROXCTRL7,
        .val = 0x00,  /* AVGCOMPDIS[7]=0 (compensation enabled),      */
                      /* COMPMETHOD[6]=0 (separate CSx compensation), */
                      /* HYST[5:4]=0 (hysteresis=32),                 */
                      /* CLOSEDEB[3:2]=0 (close debouncer=off),       */
                      /* FARDEB[1:0]=0 (far debouncer=off)            */
    },
    {
        .reg = SX9500_REG_PROXCTRL8,
        .val = 0x08,  /* STUCK[7:4]=0 (stuck timeout=off),                          */
                      /* COMPPRD[3:0]=8 (periodic compensation every 8*128 samples) */
    },
    {
        .reg = SX9500_REG_PROXCTRL0,
        .val = 0x0F,  /* SCANPERIOD[6:4]=0 (scan every 30ms),  */
                      /* SENSOREN[3:0]=15 (enable all sensors) */
    },
};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static SX9500_status SX9500_write(uint8_t addr, uint8_t data)
{
    uint8_t cmd[2];

    cmd[0] = addr;
    cmd[1] = data;

    return (SX9500_status)FCT_SX9500.I2C_Write(SX9500_I2C_ADDRESS, cmd, 2);
}

static SX9500_status SX9500_read(uint8_t addr, uint8_t *dst_buf, uint32_t length)
{
    uint8_t cmd[2];

    cmd[0] = addr;
    return (SX9500_status)FCT_SX9500.I2C_Read(SX9500_I2C_ADDRESS, cmd, 1, dst_buf, length);
}

static SX9500_status SX9500_read_single(uint8_t addr, uint8_t* val)
{
    return (SX9500_status)FCT_SX9500.I2C_Read(SX9500_I2C_ADDRESS, &addr, 1, val, 1);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void SX9500_Init_Driver(ptsx9500_fct_t  FCT)
{
    assert((FCT != NULL) &&
            (FCT->connect_hw != NULL) &&
            (FCT->disconnect_hw != NULL) &&
            (FCT->I2C_Read != NULL) &&
            (FCT->I2C_Write != NULL) &&
            (FCT->WaitMs != NULL));
    FCT_SX9500 = *FCT;
    initDriverDone = true;
}

void SX9500_Denit_Driver()
{
    if (initHwDone)
    {
        /* Deinit HW */
        SX9500_set_active(false); /* skip error management */
        FCT_SX9500.disconnect_hw();
    }

    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

SX9500_status SX9500_Init_Hw()
{
    int i = 0;
    uint8_t val;
    SX9500_status status;

    if (!initDriverDone)
    {
        return SX9500_NOINIT_ERROR;
    }

    FCT_SX9500.connect_hw();

    /* perform a soft reset */
    status = SX9500_write(SX9500_REG_RESET, SX9500_RESET_CMD);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    FCT_SX9500.WaitMs(300); /* wait until the reset has finished */

    /* read IRQSRC to release NIRQ pin */
    status = SX9500_read(SX9500_REG_IRQSRC, &val,1);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    /* init I2C registers */
    int lenRegTable = sizeof(sx9500_i2c_reg_setup) / sizeof(smtc_reg_data_t);
    while (i < lenRegTable)
    {
        /* Write all registers/values contained in i2c_reg */
        status = SX9500_write(sx9500_i2c_reg_setup[i].reg, sx9500_i2c_reg_setup[i].val);
        if (status != SX9500_SUCCESS)
        {
            return status;
        }

        /* Read back value from register and verify write */
        status = SX9500_read_single(sx9500_i2c_reg_setup[i].reg, &val);
        if (status != SX9500_SUCCESS)
        {
            return status;
        }

        if (val != sx9500_i2c_reg_setup[i].val)
        {
            return SX9500_INTERNAL_ERROR;
        }

        i++;
    }

    initHwDone = true;
    return SX9500_SUCCESS;
}

SX9500_status SX9500_GetInfo_sensor(char CSn, uint8_t* buf)
{
    SX9500_status status;

    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }
    assert(buf != NULL);

    status = SX9500_write(SX9500_REG_SENSORSEL, CSn);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    status = SX9500_read(SX9500_REG_USEMSB, buf, 2);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    status = SX9500_read(SX9500_REG_AVGMSB, &buf[2], 2);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    status = SX9500_read(SX9500_REG_DIFFMSB, &buf[4], 2);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    return SX9500_SUCCESS;
}

/* get power mode (active or low power) */
bool SX9500_get_active()
{
    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }

    return m_txen;
}

/* set active mode or low power mode */
SX9500_status SX9500_set_active(bool en)
{
    SX9500_status status = SX9500_SUCCESS;

    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }

    m_txen = en;

    uint8_t val = en ? sx9500_i2c_reg_setup[10].val : 0x00U;

    status = SX9500_write(SX9500_REG_PROXCTRL0, val);

    return status;
}

/* returns the triggered capacitive sensing interface (CS0..CS3) */
SX9500_status SX9500_CSi_Detected(uint8_t* CSi)
{
    SX9500_status status = SX9500_SUCCESS;
    RegIrqSrc_t regirq;
    RegStat_t prox;

    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }
    assert(CSi != NULL);

    *CSi = 0xFF;

    status = SX9500_read_single(SX9500_REG_IRQSRC, &regirq.octet);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    status = SX9500_read_single(SX9500_REG_STAT, &prox.octet);
    if (status != SX9500_SUCCESS)
    {
        return status;
    }

    if (regirq.bits.close)
    {
        if (prox.bits.proxstat0)
        {
            *CSi = 0;
        }
        else if (prox.bits.proxstat1)
        {
            *CSi = 1;
        }
        else if (prox.bits.proxstat2)
        {
            *CSi = 2;
        }
        else if (prox.bits.proxstat3)
        {
            *CSi = 3;
        }
    }
    return SX9500_SUCCESS;
}

SX9500_status SX9500_Read_Irq(uint8_t* irqReg)
{
    SX9500_status status = SX9500_SUCCESS;

    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }

    if (NULL != irqReg)
    {
        RegIrqSrc_t regirq;
        status = SX9500_read_single(SX9500_REG_IRQSRC, &regirq.octet);
        if (status != SX9500_SUCCESS)
        {
            return status;
        }
        else
        {
            *irqReg = regirq.octet;
            return SX9500_SUCCESS;
        }
    }
    else
    {
        return SX9500_INTERNAL_ERROR;
    }
}

SX9500_status SX9500_Read_Proximity_Sensors(uint8_t* data)
{
    SX9500_status status = SX9500_SUCCESS;

    if (!initHwDone)
    {
        return SX9500_NOINIT_ERROR;
    }

    if (NULL != data)
    {
        RegStat_t prox;
        status = SX9500_read_single(SX9500_REG_STAT, &prox.octet);
        if (status != SX9500_SUCCESS)
        {
            *data = 0xFF;
            return status;
        }
        else
        {
            *data = prox.octet;
            return SX9500_SUCCESS;
        }
    }
    else
    {
        return SX9500_INTERNAL_ERROR;
    }
}

