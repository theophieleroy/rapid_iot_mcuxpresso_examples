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

#include "fsl_isi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.isi"
#endif

/* The macros for color space convertion. */
#define ISI_CSC_COEFF_FRAC_BITS 8U
#define ISI_CSC_COEFF_SIGN_SHIFT 10U
#define ISI_CSC_COEFF_MAX 3.99609375 /* 11.11111111b */

/* The number of output buffer. */
#define ISI_OUT_BUFFER_CNT 2

typedef union _u32_f32
{
    float f32;
    uint32_t u32;
} u32_f32_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get instance number for ISI module.
 *
 * @param base ISI peripheral base address.
 */
static uint32_t ISI_GetInstance(ISI_Type *base);

/*!
 * @brief Convert IEEE 754 float value to the value could be written to registers.
 *
 * This function converts the float value to integer value to set the scaler
 * and CSC parameters.
 *
 * @param floatValue The float value to convert.
 * @param intBits Bits number of integer part in result.
 * @param fracBits Bits number of fractional part in result.
 * @return The value to set to register.
 */
static uint32_t ISI_ConvertFloat(float floatValue, uint8_t intBits, uint8_t fracBits);

/*!
 * @brief Convert the desired scale fact to pre-decimation (DEC) and SCALE_FACTO.
 *
 * @param inputDimension Input dimension.
 * @param outputDimension Output dimension.
 * @param dec The decimation value.
 * @param scale The scale value set to register SCALE_FACTOR.
 */
static void ISI_GetScalerParam(uint16_t inputDimension, uint16_t outputDimension, uint8_t *dec, uint32_t *scale);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Pointers to ISI bases for each instance. */
static ISI_Type *const s_isiBases[] = ISI_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to isi clocks for each instance. */
static const clock_ip_name_t s_isiClocks[] = ISI_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
static void ISI_GetScalerParam(uint16_t inputDimension, uint16_t outputDimension, uint8_t *dec, uint32_t *scale)
{
    uint32_t scaleFact = ((uint32_t)inputDimension << 12U) / outputDimension;

    if (scaleFact >= (16U << 12U))
    {
        /* Desired fact is two large, use the largest support value. */
        *dec = 3U;
        *scale = 0x2000U;
    }
    else
    {
        if (scaleFact > (8U << 12U))
        {
            *dec = 3U;
        }
        else if (scaleFact > (4U << 12U))
        {
            *dec = 2U;
        }
        else if (scaleFact > (2U << 12U))
        {
            *dec = 1U;
        }
        else
        {
            *dec = 0U;
        }

        *scale = scaleFact >> (*dec);

        if (0U == *scale)
        {
            *scale = 1U;
        }
    }
}

static uint32_t ISI_GetInstance(ISI_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_isiBases); instance++)
    {
        if (s_isiBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_isiBases));

    return instance;
}

static uint32_t ISI_ConvertFloat(float floatValue, uint8_t intBits, uint8_t fracBits)
{
    /* One bit reserved for sign bit. */
    assert(intBits + fracBits < 32);

    u32_f32_t u32_f32;
    uint32_t ret;

    u32_f32.f32 = floatValue;
    uint32_t floatBits = u32_f32.u32;
    int32_t expValue = (int32_t)((floatBits & 0x7F800000U) >> 23U) - 127;

    ret = (floatBits & 0x007FFFFFU) | 0x00800000U;
    expValue += fracBits;

    if (expValue < 0)
    {
        return 0U;
    }
    else if (expValue > 23)
    {
        /* should not exceed 31-bit when left shift. */
        assert((expValue - 23) <= 7);
        ret <<= (expValue - 23);
    }
    else
    {
        ret >>= (23 - expValue);
    }

    /* Set the sign bit. */
    if (floatBits & 0x80000000U)
    {
        ret = ((~ret) + 1U) & ~(((uint32_t)-1) << (intBits + fracBits + 1));
    }

    return ret;
}

void ISI_Init(ISI_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_isiClocks[ISI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Enable channel clock. */
    ISI_Reset(base);
    base->CHNL_CTRL = ISI_CHNL_CTRL_CLK_EN_MASK;
}

void ISI_Deinit(ISI_Type *base)
{
    ISI_Reset(base);
    /* Stop channel, disable the channel clock. */
    base->CHNL_CTRL = 0U;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_isiClocks[ISI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void ISI_Reset(ISI_Type *base)
{
    base->CHNL_CTRL |= ISI_CHNL_CTRL_SW_RST_MASK;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    base->CHNL_CTRL &= ~ISI_CHNL_CTRL_SW_RST_MASK;
}

void ISI_SetConfig(ISI_Type *base, const isi_config_t *config)
{
    assert(config);

    uint32_t reg;

    /* Set control bit fields in register CHNL_CTRL. */
    reg = base->CHNL_CTRL;
    reg &= ~(ISI_CHNL_CTRL_CHNL_BYPASS_MASK | ISI_CHNL_CTRL_CHAIN_BUF_MASK | ISI_CHNL_CTRL_BLANK_PXL_MASK |
             ISI_CHNL_CTRL_MIPI_VC_ID_MASK | ISI_CHNL_CTRL_SRC_TYPE_MASK | ISI_CHNL_CTRL_SRC_MASK);
    reg |= ISI_CHNL_CTRL_CHNL_BYPASS(config->isChannelBypassed) | ISI_CHNL_CTRL_CHAIN_BUF(config->chainMode) |
           ISI_CHNL_CTRL_BLANK_PXL(config->blankPixel) | ISI_CHNL_CTRL_MIPI_VC_ID(config->mipiChannel) |
           ISI_CHNL_CTRL_SRC_TYPE(config->isSourceMemory) | ISI_CHNL_CTRL_SRC(config->sourcePort);
    base->CHNL_CTRL = reg;

    /* Set control bit fields in register CHNL_IMG_CTRL. */
    reg = base->CHNL_IMG_CTRL;
    reg &= ~(ISI_CHNL_IMG_CTRL_FORMAT_MASK | ISI_CHNL_IMG_CTRL_DEINT_MASK | ISI_CHNL_IMG_CTRL_YCBCR_MODE_MASK);
    reg |= ISI_CHNL_IMG_CTRL_FORMAT(config->outputFormat) | ISI_CHNL_IMG_CTRL_DEINT(config->deintMode) |
           ISI_CHNL_IMG_CTRL_YCBCR_MODE(config->isYCbCr);
    base->CHNL_IMG_CTRL = reg;

    base->CHNL_IMG_CFG = ((uint32_t)(config->inputHeight) << ISI_CHNL_IMG_CFG_HEIGHT_SHIFT) |
                         ((uint32_t)(config->inputWidth) << ISI_CHNL_IMG_CFG_WIDTH_SHIFT);

    /* Set output buffer configuration. */
    base->CHNL_OUT_BUF_PITCH = config->outputLinePitchBytes;

    /* Set channel buffer panic threshold. */
    reg = base->CHNL_OUT_BUF_CTRL;
    reg &= ~(ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_MASK | ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_MASK |
             ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_MASK);
    reg |= ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V(config->thresholdV) |
           ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U(config->thresholdU) |
           ISI_CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y(config->thresholdY);
    base->CHNL_OUT_BUF_CTRL = reg;
}

void ISI_GetDefaultConfig(isi_config_t *config)
{
    assert(config);

    config->isChannelBypassed = false;
    config->isSourceMemory = false;
    config->isYCbCr = false;
    config->chainMode = kISI_ChainDisable;
    config->deintMode = kISI_DeintDisable;
    config->blankPixel = 0xFFU;
    config->sourcePort = 0U;
    config->mipiChannel = 0U;
    config->inputHeight = 1080U;
    config->inputWidth = 1920U;
    config->outputFormat = kISI_OutputRGBA8888;
    config->outputLinePitchBytes = 0U;
    config->thresholdY = kISI_ThresholdDisable;
    config->thresholdU = kISI_ThresholdDisable;
    config->thresholdV = kISI_ThresholdDisable;
}

void ISI_SetScalerConfig(
    ISI_Type *base, uint16_t inputWidth, uint16_t inputHeight, uint16_t outputWidth, uint16_t outputHeight)
{
    uint8_t decX, decY;
    uint32_t scaleX, scaleY;

    ISI_GetScalerParam(inputWidth, outputWidth, &decX, &scaleX);
    ISI_GetScalerParam(inputHeight, outputHeight, &decY, &scaleY);

    /* Set the pre-decimation configuration. */
    base->CHNL_IMG_CTRL = (base->CHNL_IMG_CTRL & ~(ISI_CHNL_IMG_CTRL_DEC_X_MASK | ISI_CHNL_IMG_CTRL_DEC_Y_MASK)) |
                          ISI_CHNL_IMG_CTRL_DEC_X(decX) | ISI_CHNL_IMG_CTRL_DEC_Y(decY);

    /* Set the bilinear scaler engine configuration. */
    /* The scaler factor is represented as ##.####_####_#### in register. */
    base->CHNL_SCALE_FACTOR = ISI_CHNL_SCALE_FACTOR_X_SCALE(scaleX) | ISI_CHNL_SCALE_FACTOR_Y_SCALE(scaleY);
}

void ISI_SetColorSpaceConversionConfig(ISI_Type *base, const isi_csc_config_t *config)
{
    assert(config);

    /*
     * The CSC coefficient has a sign bit, 2 bits integer, and 8 bits of fraction as ###.####_####.
     * This function converts the float value to the register format.
     */
    base->CHNL_CSC_COEFF0 = (ISI_ConvertFloat(config->A1, 2, 8) << ISI_CHNL_CSC_COEFF0_A1_SHIFT) |
                            (ISI_ConvertFloat(config->A2, 2, 8) << ISI_CHNL_CSC_COEFF0_A2_SHIFT);
    base->CHNL_CSC_COEFF1 = (ISI_ConvertFloat(config->A3, 2, 8) << ISI_CHNL_CSC_COEFF1_A3_SHIFT) |
                            (ISI_ConvertFloat(config->B1, 2, 8) << ISI_CHNL_CSC_COEFF1_B1_SHIFT);
    base->CHNL_CSC_COEFF2 = (ISI_ConvertFloat(config->B2, 2, 8) << ISI_CHNL_CSC_COEFF2_B2_SHIFT) |
                            (ISI_ConvertFloat(config->B3, 2, 8) << ISI_CHNL_CSC_COEFF2_B3_SHIFT);
    base->CHNL_CSC_COEFF3 = (ISI_ConvertFloat(config->C1, 2, 8) << ISI_CHNL_CSC_COEFF3_C1_SHIFT) |
                            (ISI_ConvertFloat(config->C2, 2, 8) << ISI_CHNL_CSC_COEFF3_C2_SHIFT);
    base->CHNL_CSC_COEFF4 =
        (ISI_ConvertFloat(config->C3, 2, 8) << ISI_CHNL_CSC_COEFF4_C3_SHIFT) | ISI_CHNL_CSC_COEFF4_D1(config->D1);
    base->CHNL_CSC_COEFF5 = ISI_CHNL_CSC_COEFF5_D2(config->D2) | ISI_CHNL_CSC_COEFF5_D3(config->D3);

    base->CHNL_IMG_CTRL =
        (base->CHNL_IMG_CTRL & ~ISI_CHNL_IMG_CTRL_CSC_MODE_MASK) | ISI_CHNL_IMG_CTRL_CSC_MODE(config->mode);
}

void ISI_ColorSpaceConversionGetDefaultConfig(isi_csc_config_t *config)
{
    assert(config);

    config->mode = kISI_CscYUV2RGB;
    config->A1 = 0.0f;
    config->A2 = 0.0f;
    config->A3 = 0.0f;
    config->B1 = 0.0f;
    config->B2 = 0.0f;
    config->B3 = 0.0f;
    config->C1 = 0.0f;
    config->C2 = 0.0f;
    config->C3 = 0.0f;
    config->D1 = 0;
    config->D2 = 0;
    config->D3 = 0;
}

void ISI_SetCropConfig(ISI_Type *base, const isi_crop_config_t *config)
{
    assert(config);

    base->CHNL_CROP_ULC = ISI_CHNL_CROP_ULC_X(config->upperLeftX) | ISI_CHNL_CROP_ULC_Y(config->upperLeftY);
    base->CHNL_CROP_LRC = ISI_CHNL_CROP_LRC_X(config->lowerRightX) | ISI_CHNL_CROP_LRC_Y(config->lowerRightY);
}

void ISI_CropGetDefaultConfig(isi_crop_config_t *config)
{
    assert(config);

    config->upperLeftX = 0U;
    config->upperLeftY = 0U;
    config->lowerRightX = 0U;
    config->lowerRightY = 0U;
}

void ISI_SetRegionAlphaConfig(ISI_Type *base, uint8_t index, const isi_region_alpha_config_t *config)
{
    assert(config);
    assert(index < ISI_ROI_NUM);

    uint32_t reg = base->ROI[index].CHNL_ROI_ALPHA & ~ISI_CHNL_ROI_ALPHA_ALPHA_MASK;
    base->ROI[index].CHNL_ROI_ALPHA = reg | ISI_CHNL_ROI_ALPHA_ALPHA(config->alpha);

    base->ROI[index].CHNL_ROI_ULC = ISI_CHNL_ROI_ULC_X(config->upperLeftX) | ISI_CHNL_ROI_ULC_Y(config->upperLeftY);
    base->ROI[index].CHNL_ROI_LRC = ISI_CHNL_ROI_LRC_X(config->lowerRightX) | ISI_CHNL_ROI_LRC_Y(config->lowerRightY);
}

void ISI_RegionAlphaGetDefaultConfig(isi_region_alpha_config_t *config)
{
    assert(config);

    config->upperLeftX = 0U;
    config->upperLeftY = 0U;
    config->lowerRightX = 0U;
    config->lowerRightY = 0U;
    config->alpha = 0U;
}

void ISI_EnableRegionAlpha(ISI_Type *base, uint8_t index, bool enable)
{
    assert(index < ISI_ROI_NUM);

    if (enable)
    {
        base->ROI[index].CHNL_ROI_ALPHA |= ISI_CHNL_ROI_ALPHA_ALPHA_EN_MASK;
    }
    else
    {
        base->ROI[index].CHNL_ROI_ALPHA &= ~ISI_CHNL_ROI_ALPHA_ALPHA_EN_MASK;
    }
}

void ISI_SetInputMemConfig(ISI_Type *base, const isi_input_mem_config_t *config)
{
    assert(config);

    uint32_t reg;

    base->CHNL_IN_BUF_ADDR = config->adddr;
    base->CHNL_IN_BUF_PITCH = ISI_CHNL_IN_BUF_PITCH_FRM_PITCH(config->framePitchBytes) |
                              ISI_CHNL_IN_BUF_PITCH_LINE_PITCH(config->linePitchBytes);

    reg = base->CHNL_MEM_RD_CTRL;
    reg &= ~ISI_CHNL_MEM_RD_CTRL_IMG_TYPE_MASK;
    reg |= ISI_CHNL_MEM_RD_CTRL_IMG_TYPE(config->format);
    base->CHNL_MEM_RD_CTRL = reg;
}

void ISI_InputMemGetDefaultConfig(isi_input_mem_config_t *config)
{
    assert(config);

    config->adddr = 0U;
    config->linePitchBytes = 0U;
    config->framePitchBytes = 0U;
    config->format = kISI_InputMemBGR888;
}

void ISI_TriggerInputMemRead(ISI_Type *base)
{
    uint32_t reg;

    reg = base->CHNL_MEM_RD_CTRL;
    /* Clear CHNL_MEM_RD_CTRL[READ_MEM]. */
    base->CHNL_MEM_RD_CTRL = reg & ~ISI_CHNL_MEM_RD_CTRL_READ_MEM_MASK;
    /* Set CHNL_MEM_RD_CTRL[READ_MEM]. */
    base->CHNL_MEM_RD_CTRL = reg | ISI_CHNL_MEM_RD_CTRL_READ_MEM_MASK;
}

void ISI_SetOutputBufferAddr(ISI_Type *base, uint8_t index, uint32_t addrY, uint32_t addrU, uint32_t addrV)
{
    assert(index < ISI_OUT_BUFFER_CNT);

    if (0 == index)
    {
        base->CHNL_OUT_BUF1_ADDR_Y = addrY;
        base->CHNL_OUT_BUF1_ADDR_U = addrU;
        base->CHNL_OUT_BUF1_ADDR_V = addrV;
        base->CHNL_OUT_BUF_CTRL ^= ISI_CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR_MASK;
    }
    else
    {
        base->CHNL_OUT_BUF2_ADDR_Y = addrY;
        base->CHNL_OUT_BUF2_ADDR_U = addrU;
        base->CHNL_OUT_BUF2_ADDR_V = addrV;
        base->CHNL_OUT_BUF_CTRL ^= ISI_CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR_MASK;
    }
}
