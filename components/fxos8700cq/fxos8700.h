/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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
#ifndef _FSL_FXOS_H_
#define _FSL_FXOS_H_

/*!
 * @addtogroup fxos8700 FXOS8700 Combination Accelerometer & Magnetometer
 * This module provides the API to operate the FXOS8700 combination accelerometer/magnetometer sensor through an I2C interface.
 *
 * The FXOS8700CQ is a small, low-power, 3-axis, linear accelerometer and 3-axis,
 * magnetometer combined into a single package. The device features a 14-bit accelerometer and 16-bit magnetometer ADC
 * resolution along with smart-embedded functions. FXOS8700CQ has dynamically
 * selectable acceleration full-scale ranges of ±2 g/±4 g/±8 g and a fixed magnetic
 * measurement range of ±1200 μT. Output data rates (ODR) from 1.563 Hz to 800 Hz are
 * selectable by the user for each sensor. Interleaved magnetic and acceleration data is
 * available at ODR rates of up to 400 Hz.
 *
 * Usage
 * ------------------------------------------------------------------------------------------------------------------------------
 *
 * Initialization:
 * @code
 *   #include "fxos8700.h"
 *   fxos8700_handle_t g_fxosHandle;
 *   fxos8700_data_t fxos8700_data;
 *
 *   g_fxosHandle.xfer.slaveAddress = FXOS8700_I2C_SLAVE_ADDRESS;
 *   g_fxosHandle.base = I2C1;
 *   g_fxosHandle.i2cHandle = &g_mi2c_handle;
 *
 *   uint8_t g_sensorRange = 0;
 *   float g_dataScale =0;
 *
 *   // Init sensor
 *  if (FXOS8700_Init(&g_fxosHandle) != kStatus_Success)
 *  {
 *    return kStatus_Fail;
 *  }
 *
 *   // Get sensor range
 *   if (FXOS8700_ReadReg(&g_fxosHandle, FXOS8700_XYZ_DATA_CFG_REG, &g_sensorRange, 1) != kStatus_Success)
 *   {
 *       return -1;
 *   }
 *   if(g_sensorRange == 0x00)
 *   {
 *       // 0.244 mg/LSB
 *       g_dataScale = 0.000244;
 *   }
 *   else if(g_sensorRange == 0x01)
 *   {
 *       // 0.488 mg/LSB
 *       g_dataScale = 0.000488;
 *   }
 *  else if(g_sensorRange == 0x10)
 *  {
 *      // 0.976 mg/LSB
 *      g_dataScale = 0.000976;
 *  }
 * @endcode
 *
 * Basic Operation:
 * @code
 *
 *  float accel[3]= {0};
 *  float mag [3] = {0};
 *
 *  if (FXOS8700_ReadSensorData(&g_fxosHandle, &fxos8700_data) == kStatus_Success)
 *  {
 *
 *
 *      //converting to units of  g
 *      accel[0] =  ((float)((int16_t)(((fxos8700_data.accelXMSB*256) + (fxos8700_data.accelXLSB)))>> 2));
 *      accel[1] =  ((float)((int16_t)(((fxos8700_data.accelYMSB*256) + (fxos8700_data.accelYLSB)))>> 2));
 *      accel[2] =  ((float)((int16_t)(((fxos8700_data.accelZMSB*256) + (fxos8700_data.accelZLSB)))>> 2));
 *
 *      //apply scaling
 *      accel[0] *= g_dataScale;
 *      accel[1] *= g_dataScale;
 *      accel[2] *= g_dataScale;
 *
 *      //Converting to units of uT
 *      mag[0] =  (float)((int16_t)((fxos8700_data.magXMSB*256) + (fxos8700_data.magXLSB))) * 0.1;
 *      mag[1] =  (float)((int16_t)((fxos8700_data.magYMSB*256) + (fxos8700_data.magYLSB))) * 0.1;
 *      mag[2] =  (float)((int16_t)((fxos8700_data.magZMSB*256) + (fxos8700_data.magZLSB))) * 0.1;
 *
 * }
 *
 * @endcode
 *
 * @{
 */

#include "fsl_common.h"
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT > 0)
#include "fsl_lpi2c.h"
#else
#include "fsl_i2c.h"
#endif


#define SINGLE_TAP 1
#define DOUBLE_TAP 0
/*
 *  STATUS Register
 */
#define FXOS8700_STATUS_00_REG 0x00

#define FXOS8700_ZYXOW_MASK 0x80
#define FXOS8700_ZOW_MASK 0x40
#define FXOS8700_YOW_MASK 0x20
#define FXOS8700_XOW_MASK 0x10
#define FXOS8700_ZYXDR_MASK 0x08
#define FXOS8700_ZDR_MASK 0x04
#define FXOS8700_YDR_MASK 0x02
#define FXOS8700_XDR_MASK 0x01

/*
 *  F_STATUS FIFO Status Register
 */
#define FXOS8700_F_STATUS_REG 0x00

#define FXOS8700_F_OVF_MASK 0x80
#define FXOS8700_F_WMRK_FLAG_MASK 0x40
#define FXOS8700_F_CNT5_MASK 0x20
#define FXOS8700_F_CNT4_MASK 0x10
#define FXOS8700_F_CNT3_MASK 0x08
#define FXOS8700_F_CNT2_MASK 0x04
#define FXOS8700_F_CNT1_MASK 0x02
#define FXOS8700_F_CNT0_MASK 0x01
#define FXOS8700_F_CNT_MASK 0x3F

/*
 *  XYZ Data Registers
 */
#define FXOS8700_OUT_X_MSB_REG 0x01
#define FXOS8700_OUT_X_LSB_REG 0x02
#define FXOS8700_OUT_Y_MSB_REG 0x03
#define FXOS8700_OUT_Y_LSB_REG 0x04
#define FXOS8700_OUT_Z_MSB_REG 0x05
#define FXOS8700_OUT_Z_LSB_REG 0x06

/*
 *  F_SETUP FIFO Setup Register
 */
#define FXOS8700_F_SETUP_REG 0x09

#define FXOS8700_F_MODE1_MASK 0x80
#define FXOS8700_F_MODE0_MASK 0x40
#define FXOS8700_F_WMRK5_MASK 0x20
#define FXOS8700_F_WMRK4_MASK 0x10
#define FXOS8700_F_WMRK3_MASK 0x08
#define FXOS8700_F_WMRK2_MASK 0x04
#define FXOS8700_F_WMRK1_MASK 0x02
#define FXOS8700_F_WMRK0_MASK 0x01
#define FXOS8700_F_MODE_MASK 0xC0
#define FXOS8700_F_WMRK_MASK 0x3F

#define FXOS8700_F_MODE_DISABLED 0x00
#define FXOS8700_F_MODE_CIRCULAR (F_MODE0_MASK)
#define FXOS8700_F_MODE_FILL (F_MODE1_MASK)
#define FXOS8700_F_MODE_TRIGGER (F_MODE1_MASK + F_MODE0_MASK)

/*
 *  TRIG_CFG FIFO Trigger Configuration Register
 */
#define FXOS8700_TRIG_CFG_REG 0x0A

#define FXOS8700_TRIG_TRANS_MASK 0x20
#define FXOS8700_TRIG_LNDPRT_MASK 0x10
#define FXOS8700_TRIG_PULSE_MASK 0x08
#define FXOS8700_TRIG_FF_MT_MASK 0x04

/*
 *  SYSMOD System Mode Register
 */
#define FXOS8700_SYSMOD_REG 0x0B

#define FXOS8700_FGERR_MASK 0x80 /* MMA8451 only */
#define FXOS8700_FGT_4_MASK 0x40 /* MMA8451 only */
#define FXOS8700_FGT_3_MASK 0x20 /* MMA8451 only */
#define FXOS8700_FGT_2_MASK 0x10 /* MMA8451 only */
#define FXOS8700_FGT_1_MASK 0x08 /* MMA8451 only */
#define FXOS8700_FGT_0_MASK 0x04 /* MMA8451 only */
#define FXOS8700_FGT_MASK 0x7C   /* MMA8451 only */
#define FXOS8700_SYSMOD1_MASK 0x02
#define FXOS8700_SYSMOD0_MASK 0x01
#define FXOS8700_SYSMOD_MASK 0x03

#define FXOS8700_SYSMOD_STANDBY 0x00
#define FXOS8700_SYSMOD_WAKE (SYSMOD0_MASK)
#define FXOS8700_SYSMOD_SLEEP (SYSMOD1_MASK)

/*
 *  INT_SOURCE System Interrupt Status Register
 */
#define FXOS8700_INT_SOURCE_REG 0x0C

#define FXOS8700_SRC_ASLP_MASK 0x80
#define FXOS8700_SRC_FIFO_MASK 0x40
#define FXOS8700_SRC_TRANS_MASK 0x20
#define FXOS8700_SRC_LNDPRT_MASK 0x10
#define FXOS8700_SRC_PULSE_MASK 0x08
#define FXOS8700_SRC_FF_MT_MASK 0x04
#define FXOS8700_SRC_DRDY_MASK 0x01

/*
 *  WHO_AM_I Device ID Register
 */
#define FXOS8700_WHO_AM_I_REG 0x0D

/* Content */
#define FXOS8700_kFXOS_WHO_AM_I_Device_ID 0xC7

/* XYZ_DATA_CFG Sensor Data Configuration Register */
#define FXOS8700_XYZ_DATA_CFG_REG 0x0E

#define FXOS8700_HPF_OUT_MASK 0x10
#define FXOS8700_FS1_MASK 0x02
#define FXOS8700_FS0_MASK 0x01
#define FXOS8700_FS_MASK 0x03

#define FXOS8700_FULL_SCALE_2G 0x00
#define FXOS8700_FULL_SCALE_4G (FXOS8700_FS0_MASK)
#define FXOS8700_FULL_SCALE_8G (FXOS8700_FS1_MASK)

/* HP_FILTER_CUTOFF High Pass Filter Register */
#define FXOS8700_HP_FILTER_CUTOFF_REG 0x0F

#define FXOS8700_PULSE_HPF_BYP_MASK 0x20
#define FXOS8700_PULSE_LPF_EN_MASK 0x10
#define FXOS8700_SEL1_MASK 0x02
#define FXOS8700_SEL0_MASK 0x01
#define FXOS8700_SEL_MASK 0x03

/*
 *  PL_STATUS Portrait/Landscape Status Register
 */
#define FXOS8700_PL_STATUS_REG 0x10

#define FXOS8700_NEWLP_MASK 0x80
#define FXOS8700_LO_MASK 0x40
#define FXOS8700_LAPO1_MASK 0x04
#define FXOS8700_LAPO0_MASK 0x02
#define FXOS8700_BAFRO_MASK 0x01
#define FXOS8700_LAPO_MASK 0x06

/*
 *  PL_CFG Portrait/Landscape Configuration Register
 */
#define FXOS8700_PL_CFG_REG 0x11

#define FXOS8700_DBCNTM_MASK 0x80
#define FXOS8700_PL_EN_MASK 0x40

/*
 *  PL_COUNT Portrait/Landscape Debounce Register
 */
#define FXOS8700_PL_COUNT_REG 0x12

/*
 *  PL_BF_ZCMP Back/Front and Z Compensation Register
 */
#define FXOS8700_PL_BF_ZCOMP_REG 0x13

#define FXOS8700_BKFR1_MASK 0x80
#define FXOS8700_BKFR0_MASK 0x40
#define FXOS8700_ZLOCK2_MASK 0x04
#define FXOS8700_ZLOCK1_MASK 0x02
#define FXOS8700_ZLOCK0_MASK 0x01
#define FXOS8700_BKFR_MASK 0xC0
#define FXOS8700_ZLOCK_MASK 0x07

/*
 *  PL_P_L_THS Portrait to Landscape Threshold Register
 */
#define FXOS8700_PL_P_L_THS_REG 0x14

#define FXOS8700_P_L_THS4_MASK 0x80
#define FXOS8700_P_L_THS3_MASK 0x40
#define FXOS8700_P_L_THS2_MASK 0x20
#define FXOS8700_P_L_THS1_MASK 0x10
#define FXOS8700_P_L_THS0_MASK 0x08
#define FXOS8700_HYS2_MASK 0x04
#define FXOS8700_HYS1_MASK 0x02
#define FXOS8700_HYS0_MASK 0x01
#define FXOS8700_P_L_THS_MASK 0xF8
#define FXOS8700_HYS_MASK 0x07

/*
 *  FF_MT_CFG Freefall and Motion Configuration Register
 */
#define FXOS8700_FF_MT_CFG_REG 0x15

#define FXOS8700_ELE_MASK 0x80
#define FXOS8700_OAE_MASK 0x40
#define FXOS8700_ZEFE_MASK 0x20
#define FXOS8700_YEFE_MASK 0x10
#define FXOS8700_XEFE_MASK 0x08

/*
 *  FF_MT_SRC Freefall and Motion Source Registers
 */
#define FXOS8700_FF_MT_SRC_REG 0x16

#define FXOS8700_EA_MASK 0x80
#define FXOS8700_ZHE_MASK 0x20
#define FXOS8700_ZHP_MASK 0x10
#define FXOS8700_YHE_MASK 0x08
#define FXOS8700_YHP_MASK 0x04
#define FXOS8700_XHE_MASK 0x02
#define FXOS8700_XHP_MASK 0x01

/*
 *  FF_MT_THS Freefall and Motion Threshold Registers
 *  TRANSIENT_THS Transient Threshold Register
 */
#define FXOS8700_FT_MT_THS_REG 0x17
#define FXOS8700_TRANSIENT_THS_REG 0x1F

#define FXOS8700_DBCNTM_MASK 0x80
#define FXOS8700_THS6_MASK 0x40
#define FXOS8700_THS5_MASK 0x20
#define FXOS8700_THS4_MASK 0x10
#define FXOS8700_THS3_MASK 0x08
#define FXOS8700_THS2_MASK 0x04
#define FXOS8700_TXS1_MASK 0x02
#define FXOS8700_THS0_MASK 0x01
#define FXOS8700_THS_MASK 0x7F

/* FF_MT_COUNT Freefall Motion Count Registers */
#define FXOS8700_FF_MT_COUNT_REG 0x18

/* TRANSIENT_CFG Transient Configuration Register */
#define FXOS8700_TRANSIENT_CFG_REG 0x1D

#define FXOS8700_TELE_MASK 0x10
#define FXOS8700_ZTEFE_MASK 0x08
#define FXOS8700_YTEFE_MASK 0x04
#define FXOS8700_XTEFE_MASK 0x02
#define FXOS8700_HPF_BYP_MASK 0x01

/* TRANSIENT_SRC Transient Source Register */
#define FXOS8700_TRANSIENT_SRC_REG 0x1E

#define FXOS8700_TEA_MASK 0x40
#define FXOS8700_ZTRANSE_MASK 0x20
#define FXOS8700_Z_TRANS_POL_MASK 0x10
#define FXOS8700_YTRANSE_MASK 0x08
#define FXOS8700_Y_TRANS_POL_MASK 0x04
#define FXOS8700_XTRANSE_MASK 0x02
#define FXOS8700_X_TRANS_POL_MASK 0x01

/* TRANSIENT_COUNT Transient Debounce Register */
#define FXOS8700_TRANSIENT_COUNT_REG 0x20

/* PULSE_CFG Pulse Configuration Register */
#define FXOS8700_PULSE_CFG_REG 0x21

#define FXOS8700_DPA_MASK 0x80
#define FXOS8700_PELE_MASK 0x40
#define FXOS8700_ZDPEFE_MASK 0x20
#define FXOS8700_ZSPEFE_MASK 0x10
#define FXOS8700_YDPEFE_MASK 0x08
#define FXOS8700_YSPEFE_MASK 0x04
#define FXOS8700_XDPEFE_MASK 0x02
#define FXOS8700_XSPEFE_MASK 0x01

/* PULSE_SRC Pulse Source Register */
#define FXOS8700_PULSE_SRC_REG 0x22

#define FXOS8700_PEA_MASK 0x80
#define FXOS8700_AXZ_MASK 0x40
#define FXOS8700_AXY_MASK 0x20
#define FXOS8700_AXX_MASK 0x10
#define FXOS8700_DPE_MASK 0x08
#define FXOS8700_POLZ_MASK 0x04
#define FXOS8700_POLY_MASK 0x02
#define FXOS8700_POLX_MASK 0x01

/* PULSE_THS XYZ Pulse Threshold Registers */
#define FXOS8700_PULSE_THSX_REG 0x23
#define FXOS8700_PULSE_THSY_REG 0x24
#define FXOS8700_PULSE_THSZ_REG 0x25

#define FXOS8700_PTHS_MASK 0x7F

/* PULSE_TMLT Pulse Time Window Register */
#define FXOS8700_PULSE_TMLT_REG 0x26

/* PULSE_LTCY Pulse Latency Timer Register */
#define FXOS8700_PULSE_LTCY_REG 0x27

/* PULSE_WIND Second Pulse Time Window Register */
#define FXOS8700_PULSE_WIND_REG 0x28

/* ASLP_COUNT Auto Sleep Inactivity Timer Register */
#define FXOS8700_ASLP_COUNT_REG 0x29

/* CTRL_REG1 System Control 1 Register */
#define FXOS8700_CTRL_REG1 0x2A

#define FXOS8700_ASLP_RATE1_MASK 0x80
#define FXOS8700_ASLP_RATE0_MASK 0x40
#define FXOS8700_DR2_MASK 0x20
#define FXOS8700_DR1_MASK 0x10
#define FXOS8700_DR0_MASK 0x08
#define FXOS8700_LNOISE_MASK 0x04
#define FXOS8700_FREAD_MASK 0x02
#define FXOS8700_ACTIVE_MASK 0x01
#define FXOS8700_ASLP_RATE_MASK 0xC0
#define FXOS8700_DR_MASK 0x38

#define FXOS8700_ASLP_RATE_20MS 0x00
#define FXOS8700_ASLP_RATE_80MS (FXOS8700_ASLP_RATE0_MASK)
#define FXOS8700_ASLP_RATE_160MS (FXOS8700_ASLP_RATE1_MASK)
#define FXOS8700_ASLP_RATE_640MS (FXOS8700_ASLP_RATE1_MASK + FXOS8700_ASLP_RATE0_MASK)

#define FXOS8700_ASLP_RATE_50HZ (FXOS8700_ASLP_RATE_20MS)
#define FXOS8700_ASLP_RATE_12_5HZ (FXOS8700_ASLP_RATE_80MS)
#define FXOS8700_ASLP_RATE_6_25HZ (FXOS8700_ASLP_RATE_160MS)
#define FXOS8700_ASLP_RATE_1_56HZ (FXOS8700_ASLP_RATE_640MS)

#define FXOS8700_HYB_ASLP_RATE_25HZ (FXOS8700_ASLP_RATE_20MS)
#define FXOS8700_HYB_ASLP_RATE_6_25HZ (FXOS8700_ASLP_RATE_80MS)
#define FXOS8700_HYB_ASLP_RATE_1_56HZ (FXOS8700_ASLP_RATE_160MS)
#define FXOS8700_HYB_ASLP_RATE_0_8HZ (FXOS8700_ASLP_RATE_640MS)

#define FXOS8700_DATA_RATE_1250US 0x00
#define FXOS8700_DATA_RATE_2500US (FXOS8700_DR0_MASK)
#define FXOS8700_DATA_RATE_5MS (FXOS8700_DR1_MASK)
#define FXOS8700_DATA_RATE_10MS (FXOS8700_DR1_MASK + FXOS8700_DR0_MASK)
#define FXOS8700_DATA_RATE_20MS (FXOS8700_DR2_MASK)
#define FXOS8700_DATA_RATE_80MS (FXOS8700_DR2_MASK + FXOS8700_DR0_MASK)
#define FXOS8700_DATA_RATE_160MS (FXOS8700_DR2_MASK + FXOS8700_DR1_MASK)
#define FXOS8700_DATA_RATE_640MS (FXOS8700_DR2_MASK + FXOS8700_DR1_MASK + FXOS8700_DR0_MASK)

#define FXOS8700_DATA_RATE_800HZ (FXOS8700_DATA_RATE_1250US)
#define FXOS8700_DATA_RATE_400HZ (FXOS8700_DATA_RATE_2500US)
#define FXOS8700_DATA_RATE_200HZ (FXOS8700_DATA_RATE_5MS)
#define FXOS8700_DATA_RATE_100HZ (FXOS8700_DATA_RATE_10MS)
#define FXOS8700_DATA_RATE_50HZ (FXOS8700_DATA_RATE_20MS)
#define FXOS8700_DATA_RATE_12_5HZ (FXOS8700_DATA_RATE_80MS)
#define FXOS8700_DATA_RATE_6_25HZ (FXOS8700_DATA_RATE_160MS)
#define FXOS8700_DATA_RATE_1_56HZ (FXOS8700_DATA_RATE_640MS)

/* for hybrid (TO, Aug 2012) */
#define FXOS8700_HYB_DATA_RATE_400HZ (FXOS8700_DATA_RATE_1250US)
#define FXOS8700_HYB_DATA_RATE_200HZ (FXOS8700_DATA_RATE_2500US)
#define FXOS8700_HYB_DATA_RATE_100HZ (FXOS8700_DATA_RATE_5MS)
#define FXOS8700_HYB_DATA_RATE_50HZ (FXOS8700_DATA_RATE_10MS)
#define FXOS8700_HYB_DATA_RATE_25HZ (FXOS8700_DATA_RATE_20MS)
#define FXOS8700_HYB_DATA_RATE_6_25HZ (FXOS8700_DATA_RATE_80MS)
#define FXOS8700_HYB_DATA_RATE_3_15HZ (FXOS8700_DATA_RATE_160MS)
#define FXOS8700_HYB_DATA_RATE_0_8HZ (FXOS8700_DATA_RATE_640MS)

#define FXOS8700_ACTIVE (FXOS8700_ACTIVE_MASK)
#define FXOS8700_STANDBY 0x00

/* CTRL_REG2 System Control 2 Register */
#define FXOS8700_CTRL_REG2 0x2B

#define FXOS8700_ST_MASK 0x80
#define FXOS8700_RST_MASK 0x40
#define FXOS8700_SMODS1_MASK 0x10
#define FXOS8700_SMODS0_MASK 0x08
#define FXOS8700_SLPE_MASK 0x04
#define FXOS8700_MODS1_MASK 0x02
#define FXOS8700_MODS0_MASK 0x01
#define FXOS8700_SMODS_MASK 0x18
#define FXOS8700_MODS_MASK 0x03

#define FXOS8700_SMOD_NORMAL 0x00
#define FXOS8700_SMOD_LOW_NOISE (FXOS8700_SMODS0_MASK)
#define FXOS8700_SMOD_HIGH_RES (FXOS8700_SMODS1_MASK)
#define FXOS8700_SMOD_LOW_POWER (FXOS8700_SMODS1_MASK + FXOS8700_SMODS0_MASK)

#define FXOS8700_MOD_NORMAL 0x00
#define FXOS8700_MOD_LOW_NOISE (FXOS8700_MODS0_MASK)
#define FXOS8700_MOD_HIGH_RES (FXOS8700_MODS1_MASK)
#define FXOS8700_MOD_LOW_POWER (FXOS8700_MODS1_MASK + FXOS8700_MODS0_MASK)

/* CTRL_REG3 Interrupt Control Register */
#define FXOS8700_CTRL_REG3 0x2C

#define FXOS8700_FIFO_GATE_MASK 0x80
#define FXOS8700_WAKE_TRANS_MASK 0x40
#define FXOS8700_WAKE_LNDPRT_MASK 0x20
#define FXOS8700_WAKE_PULSE_MASK 0x10
#define FXOS8700_WAKE_FF_MT_MASK 0x08
#define FXOS8700_IPOL_MASK 0x02
#define FXOS8700_PP_OD_MASK 0x01

/* CTRL_REG4 Interrupt Enable Register */
#define FXOS8700_CTRL_REG4 0x2D

#define FXOS8700_INT_EN_ASLP_MASK 0x80
#define FXOS8700_INT_EN_FIFO_MASK 0x40
#define FXOS8700_INT_EN_TRANS_MASK 0x20
#define FXOS8700_INT_EN_LNDPRT_MASK 0x10
#define FXOS8700_INT_EN_PULSE_MASK 0x08
#define FXOS8700_INT_EN_FF_MT_MASK 0x04
#define FXOS8700_INT_EN_DRDY_MASK 0x01

/* CTRL_REG5 Interrupt Configuration Register */
#define FXOS8700_CTRL_REG5 0x2E

#define FXOS8700_INT_CFG_ASLP_MASK 0x80
#define FXOS8700_INT_CFG_FIFO_MASK 0x40
#define FXOS8700_INT_CFG_TRANS_MASK 0x20
#define FXOS8700_INT_CFG_LNDPRT_MASK 0x10
#define FXOS8700_INT_CFG_PULSE_MASK 0x08
#define FXOS8700_INT_CFG_FF_MT_MASK 0x04
#define FXOS8700_INT_CFG_DRDY_MASK 0x01

/* XYZ Offset Correction Registers */
#define FXOS8700_OFF_X_REG 0x2F
#define FXOS8700_OFF_Y_REG 0x30
#define FXOS8700_OFF_Z_REG 0x31

/* M_DR_STATUS Register */
#define FXOS8700_M_DR_STATUS_REG 0x32

#define FXOS8700_ZYXOW_MASK 0x80
#define FXOS8700_ZOW_MASK 0x40
#define FXOS8700_YOW_MASK 0x20
#define FXOS8700_XOW_MASK 0x10
#define FXOS8700_ZYXDR_MASK 0x08
#define FXOS8700_ZDR_MASK 0x04
#define FXOS8700_YDR_MASK 0x02
#define FXOS8700_XDR_MASK 0x01

/* MAG XYZ Data Registers */
#define FXOS8700_M_OUT_X_MSB_REG 0x33
#define FXOS8700_M_OUT_X_LSB_REG 0x34
#define FXOS8700_M_OUT_Y_MSB_REG 0x35
#define FXOS8700_M_OUT_Y_LSB_REG 0x36
#define FXOS8700_M_OUT_Z_MSB_REG 0x37
#define FXOS8700_M_OUT_Z_LSB_REG 0x38

/* MAG CMP Data Registers */
#define FXOS8700_CMP_X_MSB_REG 0x39
#define FXOS8700_CMP_X_LSB_REG 0x3A
#define FXOS8700_CMP_Y_MSB_REG 0x3B
#define FXOS8700_CMP_Y_LSB_REG 0x3C
#define FXOS8700_CMP_Z_MSB_REG 0x3D
#define FXOS8700_CMP_Z_LSB_REG 0x3E

/* MAG XYZ Offset Correction Registers */
#define FXOS8700_M_OFF_X_MSB_REG 0x3F
#define FXOS8700_M_OFF_X_LSB_REG 0x40
#define FXOS8700_M_OFF_Y_MSB_REG 0x41
#define FXOS8700_M_OFF_Y_LSB_REG 0x42
#define FXOS8700_M_OFF_Z_MSB_REG 0x43
#define FXOS8700_M_OFF_Z_LSB_REG 0x44

/* MAG MAX XYZ Registers */
#define FXOS8700_MAX_X_MSB_REG 0x45
#define FXOS8700_MAX_X_LSB_REG 0x46
#define FXOS8700_MAX_Y_MSB_REG 0x47
#define FXOS8700_MAX_Y_LSB_REG 0x48
#define FXOS8700_MAX_Z_MSB_REG 0x49
#define FXOS8700_MAX_Z_LSB_REG 0x4A

/* MAG MIN XYZ Registers */
#define FXOS8700_MIN_X_MSB_REG 0x4B
#define FXOS8700_MIN_X_LSB_REG 0x4C
#define FXOS8700_MIN_Y_MSB_REG 0x4D
#define FXOS8700_MIN_Y_LSB_REG 0x4E
#define FXOS8700_MIN_Z_MSB_REG 0x4F
#define FXOS8700_MIN_Z_LSB_REG 0x50

/* TEMP Registers */
#define FXOS8700_TEMP_REG 0x51

/* M_THS CONFIG Registers */
#define FXOS8700_M_THS_CFG_REG 0x52

/* M_THS SRC Registers */
#define FXOS8700_M_THS_SRC_REG 0x53

/* MAG THRESHOLD XYZ Registers */
#define FXOS8700_M_THS_X_MSB_REG 0x54
#define FXOS8700_M_THS_X_LSB_REG 0x55
#define FXOS8700_M_THS_Y_MSB_REG 0x56
#define FXOS8700_M_THS_Y_LSB_REG 0x57
#define FXOS8700_M_THS_Z_MSB_REG 0x58
#define FXOS8700_M_THS_Z_LSB_REG 0x59

/* M_THS COUNT Registers */
#define FXOS8700_M_THS_COUNT 0x5A

/* MAG CTRL_REG1 System Control 1 Register */
#define FXOS8700_M_CTRL_REG1 0x5B

#define FXOS8700_M_ACAL_MASK 0x80
#define FXOS8700_M_RST_MASK 0x40
#define FXOS8700_M_OST_MASK 0x20
#define FXOS8700_M_OSR2_MASK 0x10
#define FXOS8700_M_OSR1_MASK 0x08
#define FXOS8700_M_OSR0_MASK 0x04
#define FXOS8700_M_HMS1_MASK 0x02
#define FXOS8700_M_HMS0_MASK 0x01
#define FXOS8700_M_OSR_MASK 0x1C
#define FXOS8700_M_HMS_MASK 0x03

/* OSR Selections */
#define FXOS8700_M_OSR_1_56_HZ 0x00
#define FXOS8700_M_OSR_6_25_HZ FXOS8700_M_OSR0_MASK
#define FXOS8700_M_OSR_12_5_HZ FXOS8700_M_OSR1_MASK
#define FXOS8700_M_OSR_50_HZ   FXOS8700_M_OSR1_MASK + FXOS8700_M_OSR0_MASK
#define FXOS8700_M_OSR_100_HZ  FXOS8700_M_OSR2_MASK
#define FXOS8700_M_OSR_200_HZ  FXOS8700_M_OSR2_MASK + FXOS8700_M_OSR0_MASK
#define FXOS8700_M_OSR_400_HZ  FXOS8700_M_OSR2_MASK + FXOS8700_M_OSR1_MASK
#define FXOS8700_M_OSR_800_HZ  FXOS8700_M_OSR2_MASK + FXOS8700_M_OSR1_MASK + M_OSR0_MASK

/* Hybrid Mode Selection */
#define FXOS8700_ACCEL_ACTIVE 0x00
#define FXOS8700_MAG_ACTIVE FXOS8700_M_HMS0_MASK
#define FXOS8700_HYBRID_ACTIVE (FXOS8700_M_HMS1_MASK | FXOS8700_M_HMS0_MASK)

/* MAG CTRL_REG2 System Control 2 Register */
#define FXOS8700_M_CTRL_REG2 0x5C

#define FXOS8700_M_HYB_AUTOINC_MASK 0x20
#define FXOS8700_M_MAXMIN_DIS_MASK 0x10
#define FXOS8700_M_MAXMIN_DIS_THS_MASK 0x08
#define FXOS8700_M_MAXMIN_RST_MASK 0x04
#define FXOS8700_M_RST_CNT1_MASK 0x02
#define FXOS8700_M_RST_CNT0_MASK 0x01

/* Mag Auto-Reset De-Gauss Frequency */
#define FXOS8700_RST_ODR_CYCLE 0x00
#define FXOS8700_RST_16_ODR_CYCLE FXOS8700_M_RST_CNT0_MASK
#define FXOS8700_RST_512_ODR_CYCLE FXOS8700_M_RST_CNT1_MASK
#define FXOS8700_RST_DISABLED FXOS8700_M_RST_CNT1_MASK + FXOS8700_M_RST_CNT0_MASK

/* MAG CTRL_REG3 System Control 3 Register */
#define FXOS8700_M_CTRL_REG3 0x5D

#define FXOS8700_M_RAW_MASK 0x80
#define FXOS8700_M_ASLP_OS_2_MASK 0x40
#define FXOS8700_M_ASLP_OS_1_MASK 0x20
#define FXOS8700_M_ASLP_OS_0_MASK 0x10
#define FXOS8700_M_THS_XYZ_MASK 0x08
#define FXOS8700_M_ST_Z_MASK 0x04
#define FXOS8700_M_ST_XY1_MASK 0x02
#define FXOS8700_M_ST_XY0_MASK 0x01
#define FXOS8700_M_ASLP_OSR_MASK 0x70
#define FXOS8700_M_ST_XY_MASK 0x03

/* OSR Selections */
#define FXOS8700_M_ASLP_OSR_1_56_HZ 0x00
#define FXOS8700_M_ASLP_OSR_6_25_HZ FXOS8700_M_ASLP_OS_0_MASK
#define FXOS8700_M_ASLP_OSR_12_5_HZ FXOS8700_M_ASLP_OS_1_MASK
#define FXOS8700_M_ASLP_OSR_50_HZ  FXOS8700_M_ASLP_OS_1_MASK + FXOS8700_M_ASLP_OS_0_MASK
#define FXOS8700_M_ASLP_OSR_100_HZ FXOS8700_M_ASLP_OS_2_MASK
#define FXOS8700_M_ASLP_OSR_200_HZ FXOS8700_M_ASLP_OS_2_MASK + FXOS8700_M_ASLP_OS_0_MASK
#define FXOS8700_M_ASLP_OSR_400_HZ FXOS8700_M_ASLP_OS_2_MASK + FXOS8700_M_ASLP_OS_1_MASK
#define FXOS8700_M_ASLP_OSR_800_HZ FXOS8700_M_ASLP_OS_2_MASK + FXOS8700_M_ASLP_OS_1_MASK + FXOS8700_M_ASLP_OS_0_MASK

/* MAG INT SOURCE Register */
#define FXOS8700_M_INT_SOURCE 0x5E

#define FXOS8700_SRC_M_DRDY_MASK 0x04
#define FXOS8700_SRC_M_VECM_MASK 0x02
#define FXOS8700_SRC_M_THS_MASK 0x01

/* ACCEL VECTOR CONFIG Register */
#define FXOS8700_A_VECM_CFG 0x5F

#define FXOS8700_A_VECM_INIT_CFG_MASK 0x40
#define FXOS8700_A_VECM_INIT_EN_MASK 0x20
#define FXOS8700_A_VECM_WAKE_EN_MASK 0x10
#define FXOS8700_A_VECM_EN_MASK 0x08
#define FXOS8700_A_VECM_UPDM_MASK 0x04
#define FXOS8700_A_VECM_INITM_MASK 0x02
#define FXOS8700_A_VECM_ELE_MASK 0x01

/* ACCEL VECTOR THS MSB AND LSB Register */
#define FXOS8700_A_VECM_THS_MSB 0x60

#define FXOS8700_A_VECM_DBCNTM_MASK 0x80

#define FXOS8700_A_VECM_THS_LSB 0x61

/* ACCEL VECTOR CNT Register */
#define FXOS8700_A_VECM_CNT 0x62

/* ACCEL INI8700TIAL XYZ VECTORS Register */
#define FXOS8700_A_VECM_INITX_MSB 0x63
#define FXOS8700_A_VECM_INITX_LSB 0x64
#define FXOS8700_A_VECM_INITY_MSB 0x65
#define FXOS8700_A_VECM_INITY_LSB 0x66
#define FXOS8700_A_VECM_INITZ_MSB 0x67
#define FXOS8700_A_VECM_INITZ_LSB 0x68

/* MAG VECTOR CONFIG Register */
#define FXOS8700_M_VECM_CFG 0x69

#define FXOS8700_M_VECM_INIT_CFG_MASK 0x40
#define FXOS8700_M_VECM_INIT_EN_MASK 0x20
#define FXOS8700_M_VECM_WAKE_EN_MASK 0x10
#define FXOS8700_M_VECM_EN_MASK 0x08
#define FXOS8700_M_VECM_UPDM_MASK 0x04
#define FXOS8700_M_VECM_INITM_MASK 0x02
#define FXOS8700_M_VECM_ELE_MASK 0x01

/* MAG VECTOR THS MSB AND LSB Register */
#define FXOS8700_M_VECM_THS_MSB 0x6A

#define FXOS8700_M_VECM_DBCNTM_MASK 0x80

#define FXOS8700_M_VECM_THS_LSB 0x6B

/* MAG VECTOR CNT Register */
#define FXOS8700_M_VECM_CNT 0x6C

/* MAG INITIAL XYZ VECTORS Register */
#define FXOS8700_M_VECM_INITX_MSB 0x6D
#define FXOS8700_M_VECM_INITX_LSB 0x6E
#define FXOS8700_M_VECM_INITY_MSB 0x6F
#define FXOS8700_M_VECM_INITY_LSB 0x70
#define FXOS8700_M_VECM_INITZ_MSB 0x71
#define FXOS8700_M_VECM_INITZ_LSB 0x72

/* ACCEL FFMT THS X MSB AND LSB Register */
#define FXOS8700_A_FFMT_THS_X_MSB 0x73

#define FXOS8700_A_FFMT_THS_XYZ_EN_MASK 0x80

#define FXOS8700_A_FFMT_THS_X_LSB 0x74

#define FXOS8700_A_FFMT_THS_X_LSB_MASK 0xFC

/* ACCEL FFMT THS Y MSB AND LSB Register */
#define FXOS8700_A_FFMT_THS_Y_MSB 0x75

#define FXOS8700_A_FFMT_THS_Y_EN_MASK 0x80

#define FXOS8700_A_FFMT_THS_Y_LSB 0x76

#define FXOS8700_A_FFMT_THS_Y_LSB_MASK 0xFC

/* ACCEL FFMT THS Z MSB AND LSB Register */
#define FXOS8700_A_FFMT_THS_Z_MSB 0x77

#define FXOS8700_A_FFMT_THS_Z_EN_MASK 0x80

#define FXOS8700_A_FFMT_THS_Z_LSB 0x78

#define FXOS8700_A_FFMT_THS_Z_LSB_MASK 0xFC

/* ACCEL TRANSIENT INIT Register */
#define FXOS8700_A_TRAN_INIT_XYZ_MSB 0x79
#define FXOS8700_A_TRAN_INIT_X_LSB 0x7A
#define FXOS8700_A_TRAN_INIT_Y_LSB 0x7B
#define FXOS8700_A_TRAN_INIT_Z_LSB 0x7C

/*! @brief fxos8700cq configure definition. This structure should be global.*/
typedef struct _fxos8700_handle
{
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    LPI2C_Type *base;                       /*!< I2C base. */
    lpi2c_master_transfer_t xfer;           /*!< I2C master transfer context */
    lpi2c_master_handle_t *i2cHandle;       /*!< I2C master xfer */
#else
    I2C_Type *base;                 /*!< I2C base. */
    i2c_master_handle_t *i2cHandle; /*!< I2C master transfer context */
    i2c_master_transfer_t xfer;     /*!< I2C master xfer */
#endif
} fxos8700_handle_t;

typedef struct _fxos8700cq_data
{
    uint8_t accelXMSB;              /*!< X-axis Accelerometer Reading: [7:0] are 8 MSBs of 14-bit sample. */
    uint8_t accelXLSB;              /*!< X-axis Accelerometer Reading: [7:2] are 6 LSBs of 14-bit sample. */
    uint8_t accelYMSB;              /*!< Y-axis Accelerometer Reading: [7:0] are 8 MSBs of 14-bit sample. */
    uint8_t accelYLSB;              /*!< Y-axis Accelerometer Reading: [7:2] are 6 LSBs of 14-bit sample. */
    uint8_t accelZMSB;              /*!< Z-axis Accelerometer Reading: [7:0] are 8 MSBs of 14-bit sample. */
    uint8_t accelZLSB;              /*!< Z-axis Accelerometer Reading: [7:2] are 6 LSBs of 14-bit sample. */
    uint8_t magXMSB;                /*!< X-axis Magnetometer Reading:  [7:0] are 8 MSBs of 16-bit sample. */
    uint8_t magXLSB;                /*!< X-axis Magnetometer Reading:  [7:0] are 8 LSBs of 16-bit sample. */
    uint8_t magYMSB;                /*!< Y-axis Magnetometer Reading:  [7:0] are 8 MSBs of 16-bit sample. */
    uint8_t magYLSB;                /*!< Y-axis Magnetometer Reading:  [7:0] are 8 LSBs of 16-bit sample. */
    uint8_t magZMSB;                /*!< Z-axis Magnetometer Reading:  [7:0] are 8 MSBs of 16-bit sample. */
    uint8_t magZLSB;                /*!< Z-axis Magnetometer Reading:  [7:0] are 8 LSBs of 16-bit sample. */
} fxos8700_data_t;



#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Verify and initialize fxos8700_handle ice: Hybrid mode with ODR=200Hz, Mag OSR=32, Acc OSR=Normal.
 * Interrupt for data ready can be set using \#define EN_FXOS_DATAREADY_INTERRUPT
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_Init(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Read data from sensors, assumes hyb_autoinc_mode is set in M_CTRL_REG2
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 * @param sensorData The pointer to the buffer to hold sensor data
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_ReadSensorData(fxos8700_handle_t *fxos8700_handle, fxos8700_data_t *sensorData);

/*!
 * @brief Write value to register of sensor.
 *
 * @param handle The pointer to fxos8700cq driver handle.
 * @param reg Register address.
 * @param val Data want to write.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_WriteReg(fxos8700_handle_t *handle, uint8_t reg, uint8_t val);

/*!
 * @brief Read n bytes start at register from sensor.
 *
 * @param handle The pointer to fxos8700cq driver handle.
 * @param reg Register address.
 * @param val The pointer to address which store data.
 * @param bytesNumber Number of bytes receiver.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_ReadReg(fxos8700_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber);

/*!
 * @brief Puts the FXOS8700CQ into Standby Mode
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_SetStandby(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Puts the FXOS8700CQ into Active Mode
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_SetActive(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Sets up Motion Detection on x and y axis with 0.5g Threshold.
 * Motion Detection detects both dynamic acceleration and static acceleration(gravity). Use Transient Detection
 * if only interested in detecting dynamic acceleration changes.
 *
 * NOTE: Motion Detect and FreeFall Detect shares the same hardware block so only one function can be initialized/used at a time.
 * Please refer to NXP Application Note AN4070: Motion and Freefall Detection Using the MMA8451, 2, 3Q
 * on how to configure the Motion Detection parameters. FXOS8700CQ shares the same registers and hardware blocks
 * as the MMA8451 so AN4070 is applicable for this sensor.
 *
 * Interrupts can be set with \#define EN_FFMT_INTERRUPT.
 * Interrupts when enabled goes to INT2 pin, set to INT1 pin using \#define EN_FFMT_INT1_PIN
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_MotionDetect_Init(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Sets up Freefall Detection on X and Y and Z axis with 0.25g threshold. Combined X,Y,Z magnitude must be less than 0.25g to trigger.
 * NOTE: Motion Detect and FreeFall Detect shares the same hardware block so only one function can be initialized/used at a time.
 * Please refer to NXP Application Note AN4070: Motion and Freefall Detection Using the MMA8451, 2, 3Q
 * on how to configure the Freefall Detection parameters. FXOS8700CQ shares the same registers and hardware blocks
 * as the MMA8451 so AN4070 is applicable for this sensor.
 *
 * Interrupts can be set with \#define EN_FFMT_INTERRUPT.
 * Interrupts when enabled goes to INT2 pin, set to INT1 pin using \#define EN_FFMT_INT1_PIN
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_FreefallDetect_Init(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Sets up Tap Detection (also called Pulse Detection)
 * Pulse Thresholds: X:2g Y:2g Z:3g
 * Pulse Time Window: 6 counts
 * Pulse Latency Timer: 40 counts
 * Pulse 2nd Time Window: 15 counts
 *
 * Please refer to NXP Application Note AN4072: MMA8451, 2, 3Q Single/Double and Directional Tap Detection
 * on how to configure the Tap Detection parameters. FXOS8700CQ shares the same registers and hardware blocks
 * as the MMA8451 so AN4072 is applicable for this sensor.
 *
 * Interrupts can be set with \#define EN_TAP_INTERRUPT.
 * Interrupts when enabled goes to INT2 pin, set to INT1 pin using \#define EN_TAP_INT1_PIN
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_TapDetect_Init(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Disables either freefall or motion detection.
 * Disables the associated interrupts if enabled
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_FreefallMotion_DeInit(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Disables Tap Detection.
 * Disables the associated interrupts if enabled
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_TapDetect_DeInit(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Sets up Transient Detection on x,y,z axis with 0.25g Threshold and High-Pass Filter to 2 Hz (OSR= 200Hz,High Res)
 * Transient Detection is similar to Motion Detection except it only detects dynamic acceleration. It will not register
 * static acceleration such as gravity.
 *
 * Please refer to NXP Application Note AN4071: High-Pass Filtered Data and Transient Detection Using the MMA8451, 2, 3Q
 * on how to configure the Transient Detection parameters. It also lists when to use transient detection and when to use motion detection.
 * FXOS8700CQ shares the same registers and hardware blocks as the MMA8451 so AN4071 is applicable for this sensor.
 *
 * Interrupts can be set with \#define EN_TRANS_INTERRUPT.
 * Interrupts when enabled goes to INT2 pin, set to INT1 pin using \#define EN_TRANS_INT1_PIN
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_TransientDetect_Init(fxos8700_handle_t *fxos8700_handle);

/*!
 * @brief Disables Transient detection.
 * Disables the associated interrupts if enabled
 *
 * @param fxos8700_handle The pointer to fxos8700cq driver handle.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t FXOS8700_TransientDetect_DeInit(fxos8700_handle_t *fxos8700_handle);



#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @}*/

#endif /* _FSL_FXOS_H_ */
