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

#ifndef _FSL_WM8904_H_
#define _FSL_WM8904_H_

#include "fsl_codec_common.h"

/*!
 * @addtogroup wm8904
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief CLOCK driver version 2.0.1. */
#define FSL_WM8904_DRIVER_VERSION (MAKE_VERSION(2, 0, 1))
/*@}*/

#define WM8904_RESET (0x00)
#define WM8904_ANALOG_ADC_0 (0x0A)
#define WM8904_POWER_MGMT_0 (0x0C)
#define WM8904_POWER_MGMT_2 (0x0E)
#define WM8904_POWER_MGMT_6 (0x12)
#define WM8904_CLK_RATES_0 (0x14)
#define WM8904_CLK_RATES_1 (0x15)
#define WM8904_CLK_RATES_2 (0x16)
#define WM8904_AUDIO_IF_0 (0x18)
#define WM8904_AUDIO_IF_1 (0x19)
#define WM8904_AUDIO_IF_2 (0x1A)
#define WM8904_AUDIO_IF_3 (0x1B)
#define WM8904_DAC_DIG_1 (0x21)
#define WM8904_ANALOG_LEFT_IN_0 (0x2C)
#define WM8904_ANALOG_RIGHT_IN_0 (0x2D)
#define WM8904_ANALOG_LEFT_IN_1 (0x2E)
#define WM8904_ANALOG_RIGHT_IN_1 (0x2F)
#define WM8904_ANALOG_OUT1_LEFT (0x39)
#define WM8904_ANALOG_OUT1_RIGHT (0x3A)
#define WM8904_ANALOG_OUT12_ZC (0x3D)
#define WM8904_DC_SERVO_0 (0x43)
#define WM8904_ANALOG_HP_0 (0x5A)
#define WM8904_CHRG_PUMP_0 (0x62)
#define WM8904_CLS_W_0 (0x68)
#define WM8904_WRT_SEQUENCER_0 (0x6C)
#define WM8904_WRT_SEQUENCER_3 (0x6F)
#define WM8904_WRT_SEQUENCER_4 (0x70)

/*! @brief WM8904 I2C address. */
#define WM8904_I2C_ADDRESS (0x1A)

/*! @brief WM8904 I2C bit rate. */
#define WM8904_I2C_BITRATE (400000U)

/*! @brief WM8904 status return codes. */
enum _wm8904_status
{
    kStatus_WM8904_Success = 0x0, /*!< Success */
    kStatus_WM8904_Fail = 0x1     /*!< Failure */
};

/*! @brief The audio data transfer protocol. */
typedef enum _wm8904_protocol
{
    kWM8904_ProtocolRightJustified = 0x0, /*!< Right justified mode */
    kWM8904_ProtocolLeftJustified = 0x1,  /*!< Left justified mode */
    kWM8904_ProtocolI2S = 0x2,            /*!< I2S mode */
    kWM8904_ProtocolDSP = 0x3             /*!< DSP mode */
} wm8904_protocol_t;

/*! @brief The SYSCLK / fs ratio. */
typedef enum _wm8904_fs_ratio
{
    kWM8904_FsRatio64X = 0x0,   /*!< SYSCLK is   64 * sample rate * frame width */
    kWM8904_FsRatio128X = 0x1,  /*!< SYSCLK is  128 * sample rate * frame width */
    kWM8904_FsRatio192X = 0x2,  /*!< SYSCLK is  192 * sample rate * frame width */
    kWM8904_FsRatio256X = 0x3,  /*!< SYSCLK is  256 * sample rate * frame width */
    kWM8904_FsRatio384X = 0x4,  /*!< SYSCLK is  384 * sample rate * frame width */
    kWM8904_FsRatio512X = 0x5,  /*!< SYSCLK is  512 * sample rate * frame width */
    kWM8904_FsRatio768X = 0x6,  /*!< SYSCLK is  768 * sample rate * frame width */
    kWM8904_FsRatio1024X = 0x7, /*!< SYSCLK is 1024 * sample rate * frame width */
    kWM8904_FsRatio1408X = 0x8, /*!< SYSCLK is 1408 * sample rate * frame width */
    kWM8904_FsRatio1536X = 0x9  /*!< SYSCLK is 1536 * sample rate * frame width */
} wm8904_fs_ratio_t;

/*! @brief Sample rate. */
typedef enum _wm8904_sample_rate
{
    kWM8904_SampleRate8kHz = 0x0,  /*!< 8 kHz */
    kWM8904_SampleRate12kHz = 0x1, /*!< 11.025kHz, 12kHz */
    kWM8904_SampleRate16kHz = 0x2, /*!< 16kHz */
    kWM8904_SampleRate24kHz = 0x3, /*!< 22.05kHz, 24kHz */
    kWM8904_SampleRate32kHz = 0x4, /*!< 32kHz */
    kWM8904_SampleRate48kHz = 0x5  /*!< 44.1kHz, 48kHz */
} wm8904_sample_rate_t;

/*! @brief Bit width. */
typedef enum _wm8904_bit_width
{
    kWM8904_BitWidth16 = 0x0, /*!< 16 bits */
    kWM8904_BitWidth20 = 0x1, /*!< 20 bits */
    kWM8904_BitWidth24 = 0x2, /*!< 24 bits */
    kWM8904_BitWidth32 = 0x3  /*!< 32 bits */
} wm8904_bit_width_t;

/*! @brief Audio format configuration. */
typedef struct _wm8904_audio_format
{
    wm8904_fs_ratio_t fsRatio;       /*!< SYSCLK / fs ratio */
    wm8904_sample_rate_t sampleRate; /*!< Sample rate */
    wm8904_bit_width_t bitWidth;     /*!< Bit width */
} wm8904_audio_format_t;

/*! @brief Configuration structure of WM8904. */
typedef struct _wm8904_config
{
    bool master;                  /*!< Master or slave */
    wm8904_protocol_t protocol;   /*!< Audio transfer protocol */
    wm8904_audio_format_t format; /*!< Audio format */
    uint32_t mclk_HZ;             /*!< MCLK frequency value */
} wm8904_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes WM8904.
 *
 * @param handle WM8904 handle structure.
 * @param codec_config WM8904 configuration structure.
 */
status_t WM8904_Init(codec_handle_t *handle, void *wm8904_config);

/*!
 * @brief Deinitializes the WM8904 codec.
 *
 * This function resets WM8904.
 *
 * @param handle WM8904 handle structure.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_Deinit(codec_handle_t *handle);

/*!
 * @brief Fills the configuration structure with default values.
 *
 * The default values are:
 *
 *   master = false;
 *   protocol = kWM8904_ProtocolI2S;
 *   format.fsRatio = kWM8904_FsRatio64X;
 *   format.sampleRate = kWM8904_SampleRate48kHz;
 *   format.bitWidth = kWM8904_BitWidth16;
 *
 * @param handle WM8904 handle structure to be filled with default values.
 */
void WM8904_GetDefaultConfig(wm8904_config_t *config);

/*!
 * @brief Sets WM8904 as master or slave.
 *
 * @param handle WM8904 handle structure.
 * @param master true for master, false for slave.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_SetMasterSlave(codec_handle_t *handle, bool master);

/*!
 * @brief Sets the audio data transfer protocol.
 *
 * @param handle WM8904 handle structure.
 * @param protocol Audio transfer protocol.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_SetProtocol(codec_handle_t *handle, wm8904_protocol_t protocol);

/*!
 * @brief Sets the audio data format.
 *
 * @param handle WM8904 handle structure.
 * @param sysclk System clock frequency for codec, user should pay attention to this parater, sysclk is caculate as
 * SYSCLK = MCLK / MCLKDIV, MCLKDIV is bit0 of WM8904_CLK_RATES_0.
 * @param sampleRate Sample rate frequency in Hz.
 * @param bitWidth Audio data bit width.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_SetAudioFormat(codec_handle_t *handle, uint32_t sysclk, uint32_t sampleRate, uint32_t bitWidth);

/*!
 * @brief check and update the audio data format.
 * This api is used check the fsRatio setting based on the mclk and sample rate, if fsRatio setting
 * is not correct, it will correct it according to mclk and sample rate.
 * @param handle WM8904 handle structure.
 * @param format audio data format
 * @param mclkFreq mclk frequency
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_CheckAudioFormat(codec_handle_t *handle, wm8904_audio_format_t *format, uint32_t mclkFreq);

/*!
 * @brief Sets the headphone output volume.
 *
 * The parameter should be from 0 to 63.
 * The resulting volume will be (parameter - 57 dB).
 * 0 for -57 dB, 57 for 0 dB, 63 for +6 dB etc.
 *
 * @param handle WM8904 handle structure.
 * @param volumeLeft Volume of the left channel.
 * @param volumeRight Volume of the right channel.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_SetVolume(codec_handle_t *handle, uint16_t volumeLeft, uint16_t volumeRight);

/*!
 * @brief Sets the headphone output mute.
 *
 * @param handle WM8904 handle structure.
 * @param muteLeft true to mute left channel, false to unmute.
 * @param muteRight true to mute right channel, false to unmute.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_SetMute(codec_handle_t *handle, bool muteLeft, bool muteRight);

/*!
 * @brief Reads content of all WM8904 registers and prints it to debug console.
 *
 * @param handle WM8904 handle structure.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise.
 */
status_t WM8904_PrintRegisters(codec_handle_t *handle);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_WM8904_H_ */
