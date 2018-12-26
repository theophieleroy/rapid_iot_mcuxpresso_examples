/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_wm8904.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t WM8904_UpdateFormat(codec_handle_t *handle, wm8904_audio_format_t *format);

static status_t WM8904_WaitOnWriteSequencer(codec_handle_t *handle);

static status_t WM8904_WriteRegister(codec_handle_t *handle, uint8_t reg, uint16_t value);

static status_t WM8904_ReadRegister(codec_handle_t *handle, uint8_t reg, uint16_t *value);

static status_t WM8904_ModifyRegister(codec_handle_t *handle, uint8_t reg, uint16_t mask, uint16_t value);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const uint8_t allRegisters[] = {
    0x00, 0x04, 0x05, 0x06, 0x07, 0x0A, 0x0C, 0x0E, 0x0F, 0x12, 0x14, 0x15, 0x16, 0x18, 0x19, 0x1A, 0x1B,
    0x1E, 0x1F, 0x20, 0x21, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x39,
    0x3A, 0x3B, 0x3C, 0x3D, 0x43, 0x44, 0x45, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x5A, 0x5E, 0x62,
    0x68, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7E, 0x7F,
    0x80, 0x81, 0x82, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0xC6, 0xF7, 0xF8};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t WM8904_Init(codec_handle_t *handle, void *wm8904_config)
{
    status_t result;
    wm8904_config_t *config = (wm8904_config_t *)wm8904_config;

    assert(config->mclk_HZ != 0U);

    handle->slaveAddress = WM8904_I2C_ADDRESS;

    /* reset */
    result = WM8904_WriteRegister(handle, WM8904_RESET, 0x0000);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* MCLK_INV=0, SYSCLK_SRC=0, TOCLK_RATE=0, OPCLK_ENA=1,
     * CLK_SYS_ENA=1, CLK_DSP_ENA=1, TOCLK_ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_2, 0x000F);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* WSEQ_ENA=1, WSEQ_WRITE_INDEX=0_0000 */
    result = WM8904_WriteRegister(handle, WM8904_WRT_SEQUENCER_0, 0x0100);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* WSEQ_ABORT=0, WSEQ_START=1, WSEQ_START_INDEX=00_0000 */
    result = WM8904_WriteRegister(handle, WM8904_WRT_SEQUENCER_3, 0x0100);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    result = WM8904_WaitOnWriteSequencer(handle);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* TOCLK_RATE_DIV16=0, TOCLK_RATE_x4=1, SR_MODE=0, MCLK_DIV=1
     * (Required for MMCs: SGY, KRT see erratum CE000546) */
    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_0, 0xA45F);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* INL_ENA=1, INR ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_POWER_MGMT_0, 0x0003);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* HPL_PGA_ENA=1, HPR_PGA_ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_POWER_MGMT_2, 0x0003);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* DACL_ENA=1, DACR_ENA=1, ADCL_ENA=1, ADCR_ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_POWER_MGMT_6, 0x000F);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* ADC_OSR128=1 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_ADC_0, 0x0001);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* CLK_SYS_RAT=0101 (512/fs) SAMPLE_RATE=101 (44.1kHz /48kHz) */
    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_1, 0x1405);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* DACL_DATINV=0, DACR_DATINV=0, DAC_BOOST=00, LOOPBACK=0, AIFADCL_SRC=0,
     * AIFADCR_SRC=1, AIFDACL_SRC=0, AIFDACR_SRC=1, ADC_COMP=0, ADC_COMPMODE=0,
     * DAC_COMP=0, DAC_COMPMODE=0 */
    result = WM8904_WriteRegister(handle, WM8904_AUDIO_IF_0, 0x0050);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* BCLK_DIR=0 (input), AIF_WL=00 (16-bits) */
    result = WM8904_WriteRegister(handle, WM8904_AUDIO_IF_1, 0x0002);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* OPCLK_DIV=0 (sysclk), BCLK_DIV=0c (sysclk/16) */
    result = WM8904_WriteRegister(handle, WM8904_AUDIO_IF_2, 0x000c);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* LRCLK_DIR=0 (input), LRCLK_RATE=0010_0000_0000 (BCLK / 32) */
    result = WM8904_WriteRegister(handle, WM8904_AUDIO_IF_3, 0x0020);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* DAC_MONO=0, DAC_SB_FILT-0, DAC_MUTERATE=0, DAC_UNMUTE RAMP=0,
     * DAC_OSR128=1, DAC_MUTE=0, DEEMPH=0 (none) */
    result = WM8904_WriteRegister(handle, WM8904_DAC_DIG_1, 0x0040);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* INL_CM_ENA=0, L_IP_SEL_N=10, L_IP_SEL_P=01, L_MODE=00 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_LEFT_IN_1, 0x0014);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* INR CM_ENA=0, R_IP_SEL_N=10, R_IP_SEL_P=01, R_MODE=00 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_RIGHT_IN_1, 0x0014);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* LINMUTE=0, LIN_VOL=0_0101 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_LEFT_IN_0, 0x0005);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* RINMUTE=0, RIN VOL=0_0101 LINEOUTL RMV SHORT-1, LINEOUTL ENA_OUTP=1,
     * LINEOUTL_ENA_DLY=1, LINEOUTL_ENA=1, LINEOUTR_RMV_SHORT-1,
     * LINEOUTR_ENA_OUTP=1 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_RIGHT_IN_0, 0x0005);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* HPL_BYP_ENA=0, HPR_BYP_ENA=0, LINEOUTL_BYP ENA=0, LINEOUTR_BYP ENA=0 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_OUT12_ZC, 0x0000);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* HPOUTL_MUTE=0, HPOUT_VU=0, HPOUTLZC=0, HPOUTL_VOL=11_1001 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_OUT1_LEFT, 0x0039);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* HPOUTR_MUTE=0, HPOUT_VU=0, HPOUTRZC=0, HPOUTR_VOL=11_1001 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_OUT1_RIGHT, 0x0039);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* Enable DC servos for headphone out */
    result = WM8904_WriteRegister(handle, WM8904_DC_SERVO_0, 0x0003);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* HPL_RMV_SHORT=1, HPL_ENA_OUTP=1, HPL_ENA_DLY=1, HPL_ENA=1,
     * HPR_RMV_SHORT=1, HPR_ENA_OUTP=1, HPR_ENA_DLY=1, HPR_ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_ANALOG_HP_0, 0x00FF);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* CP_DYN_PWR=1 */
    result = WM8904_WriteRegister(handle, WM8904_CLS_W_0, 0x0001);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* CP_ENA=1 */
    result = WM8904_WriteRegister(handle, WM8904_CHRG_PUMP_0, 0x0001);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    result = WM8904_SetMasterSlave(handle, config->master);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    result = WM8904_SetProtocol(handle, config->protocol);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    return WM8904_CheckAudioFormat(handle, &config->format, config->mclk_HZ);
}

status_t WM8904_Deinit(codec_handle_t *handle)
{
    /* reset */
    return WM8904_WriteRegister(handle, WM8904_RESET, 0x0000);
}

void WM8904_GetDefaultConfig(wm8904_config_t *config)
{
    memset(config, 0, sizeof(wm8904_config_t));

    config->master = false;
    config->protocol = kWM8904_ProtocolI2S;
    config->format.sampleRate = kWM8904_SampleRate48kHz;
    config->format.bitWidth = kWM8904_BitWidth16;
}

status_t WM8904_SetMasterSlave(codec_handle_t *handle, bool master)
{
    if (master)
    {
        /* only slave currently supported */
        return kStatus_WM8904_Fail;
    }

    return kStatus_WM8904_Success;
}

status_t WM8904_SetProtocol(codec_handle_t *handle, wm8904_protocol_t protocol)
{
    return WM8904_ModifyRegister(handle, WM8904_AUDIO_IF_1, 0x0003, (uint16_t)protocol);
}

status_t WM8904_CheckAudioFormat(codec_handle_t *handle, wm8904_audio_format_t *format, uint32_t mclkFreq)
{
    assert(handle && format);

    status_t result;
    uint16_t mclkDiv = 0U;
    uint32_t sampleRate = 0U;
    uint32_t fsRatio = 0U;

    result = WM8904_ReadRegister(handle, WM8904_CLK_RATES_0, &mclkDiv);
    if (kStatus_WM8904_Success != result)
    {
        return result;
    }

    switch (format->sampleRate)
    {
        case kWM8904_SampleRate8kHz:
            sampleRate = 8000;
            break;
        case kWM8904_SampleRate12kHz:
            sampleRate = 12000;
            break;
        case kWM8904_SampleRate16kHz:
            sampleRate = 16000;
            break;
        case kWM8904_SampleRate24kHz:
            sampleRate = 24000;
            break;
        case kWM8904_SampleRate32kHz:
            sampleRate = 32000;
            break;
        case kWM8904_SampleRate48kHz:
            sampleRate = 48000;
            break;
        default:
            break;
    }

    fsRatio = (mclkFreq >> (mclkDiv & 0x1U)) / sampleRate;

    switch (fsRatio)
    {
        case 64:
            format->fsRatio = kWM8904_FsRatio64X;
            break;
        case 128:
            format->fsRatio = kWM8904_FsRatio128X;
            break;
        case 192:
            format->fsRatio = kWM8904_FsRatio192X;
            break;
        case 256:
            format->fsRatio = kWM8904_FsRatio256X;
            break;
        case 384:
            format->fsRatio = kWM8904_FsRatio384X;
            break;
        case 512:
            format->fsRatio = kWM8904_FsRatio512X;
            break;
        case 768:
            format->fsRatio = kWM8904_FsRatio768X;
            break;
        case 1024:
            format->fsRatio = kWM8904_FsRatio1024X;
            break;
        case 1408:
            format->fsRatio = kWM8904_FsRatio1408X;
            break;
        case 1536:
            format->fsRatio = kWM8904_FsRatio1536X;
            break;
        default:
            break;
    }

    return WM8904_UpdateFormat(handle, format);
}

status_t WM8904_SetAudioFormat(codec_handle_t *handle, uint32_t sysclk, uint32_t sampleRate, uint32_t bitWidth)
{
    status_t result;
    wm8904_audio_format_t format;
    uint32_t ratio = 0;

    switch (sampleRate)
    {
        case 8000:
            format.sampleRate = kWM8904_SampleRate8kHz;
            break;
        case 11025:
        case 12000:
            format.sampleRate = kWM8904_SampleRate12kHz;
            break;
        case 16000:
            format.sampleRate = kWM8904_SampleRate16kHz;
            break;
        case 22050:
        case 24000:
            format.sampleRate = kWM8904_SampleRate24kHz;
            break;
        case 32000:
            format.sampleRate = kWM8904_SampleRate32kHz;
            break;
        case 44100:
        case 48000:
            format.sampleRate = kWM8904_SampleRate48kHz;
            break;
        default:
            break;
    }

    switch (bitWidth)
    {
        case 16:
            format.bitWidth = kWM8904_BitWidth16;
            break;
        case 20:
            format.bitWidth = kWM8904_BitWidth20;
            break;
        case 24:
            format.bitWidth = kWM8904_BitWidth24;
            break;
        case 32:
            format.bitWidth = kWM8904_BitWidth32;
            break;
        default:
            break;
    }

    ratio = sysclk / sampleRate;
    switch (ratio)
    {
        case 64:
            format.fsRatio = kWM8904_FsRatio64X;
            break;
        case 128:
            format.fsRatio = kWM8904_FsRatio128X;
            break;
        case 192:
            format.fsRatio = kWM8904_FsRatio192X;
            break;
        case 256:
            format.fsRatio = kWM8904_FsRatio256X;
            break;
        case 384:
            format.fsRatio = kWM8904_FsRatio384X;
            break;
        case 512:
            format.fsRatio = kWM8904_FsRatio512X;
            break;
        case 768:
            format.fsRatio = kWM8904_FsRatio768X;
            break;
        case 1024:
            format.fsRatio = kWM8904_FsRatio1024X;
            break;
        case 1408:
            format.fsRatio = kWM8904_FsRatio1408X;
            break;
        case 1536:
            format.fsRatio = kWM8904_FsRatio1536X;
            break;
        default:
            break;
    }

    result = WM8904_UpdateFormat(handle, &format);

    return result;
}

status_t WM8904_SetVolume(codec_handle_t *handle, uint16_t volumeLeft, uint16_t volumeRight)
{
    status_t result;

    result = WM8904_ModifyRegister(handle, WM8904_ANALOG_OUT1_LEFT, 0x3F, volumeLeft);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    result = WM8904_ModifyRegister(handle, WM8904_ANALOG_OUT1_RIGHT, 0xBF, volumeRight | 0x0080);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    return kStatus_WM8904_Success;
}

status_t WM8904_SetMute(codec_handle_t *handle, bool muteLeft, bool muteRight)
{
    status_t result;
    uint16_t left = muteLeft ? 0x0100 : 0x0000;
    uint16_t right = muteRight ? 0x0100 : 0x0000;

    result = WM8904_ModifyRegister(handle, WM8904_ANALOG_OUT1_LEFT, 0x0100, left);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    result = WM8904_ModifyRegister(handle, WM8904_ANALOG_OUT1_RIGHT, 0x0180, right | 0x0080);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    return kStatus_WM8904_Success;
}

status_t WM8904_PrintRegisters(codec_handle_t *handle)
{
    status_t result;
    uint16_t value;
    uint32_t i;

    for (i = 0; i < sizeof(allRegisters); i++)
    {
        result = WM8904_ReadRegister(handle, allRegisters[i], &value);
        if (result != kStatus_WM8904_Success)
        {
            PRINTF("\r\n");
            return result;
        }
        PRINTF("%s", ((i % 8) == 0) ? "\r\n" : "\t");
        PRINTF("%02X:%04X", allRegisters[i], value);
    }

    PRINTF("\r\n");
    return result;
}

static status_t WM8904_UpdateFormat(codec_handle_t *handle, wm8904_audio_format_t *format)
{
    status_t result;

    /* Disable SYSCLK */
    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_2, 0x00);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* Set Clock ratio and sample rate */

    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_1, (format->fsRatio << 10) | format->sampleRate);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* Set bit resolution */
    result = WM8904_ModifyRegister(handle, WM8904_AUDIO_IF_1, 0x000C, format->bitWidth);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    /* Enable SYSCLK */
    result = WM8904_WriteRegister(handle, WM8904_CLK_RATES_2, 0x1007);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    return kStatus_WM8904_Success;
}

static status_t WM8904_WaitOnWriteSequencer(codec_handle_t *handle)
{
    status_t result;
    uint16_t value;

    do
    {
        result = WM8904_ReadRegister(handle, WM8904_WRT_SEQUENCER_4, &value);
    } while ((result == kStatus_WM8904_Success) && (value & 1));

    return result;
}

static status_t WM8904_WriteRegister(codec_handle_t *handle, uint8_t reg, uint16_t value)
{
    uint8_t retval = 0;

    retval = CODEC_I2C_WriteReg(handle->slaveAddress, kCODEC_RegAddr8Bit, reg, kCODEC_RegWidth16Bit, value,
                                handle->I2C_SendFunc);

    return retval;
}

static status_t WM8904_ReadRegister(codec_handle_t *handle, uint8_t reg, uint16_t *value)
{
    uint8_t retval = 0;

    retval = CODEC_I2C_ReadReg(handle->slaveAddress, kCODEC_RegAddr8Bit, reg, kCODEC_RegWidth16Bit, value,
                               handle->I2C_ReceiveFunc);

    return retval;
}

static status_t WM8904_ModifyRegister(codec_handle_t *handle, uint8_t reg, uint16_t mask, uint16_t value)
{
    status_t result;
    uint16_t regValue;

    result = WM8904_ReadRegister(handle, reg, &regValue);
    if (result != kStatus_WM8904_Success)
    {
        return result;
    }

    regValue &= (uint16_t)~mask;
    regValue |= value;

    return WM8904_WriteRegister(handle, reg, regValue);
}
