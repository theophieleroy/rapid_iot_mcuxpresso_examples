/*
 * The Clear BSD License
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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
#include "fsl_dryice.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dryice"
#endif

/* all bits defined in the Write Access Control Register. Same positions as Lock Register. */
#define DRYICE_ALL_WAC_MASK 0x00FF3FFFu

/* all bits defined in the Interrupt Enable Register. */
#define DRYICE_ALL_IER_MASK 0x00FF03FDu

/* all bits defined in the Tamper Enable Register. */
#define DRYICE_ALL_TER_MASK 0x00FF03FCu

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool dryice_IsRegisterWriteAllowed(DRY_Type *base, uint32_t mask)
{
    bool retval;

    retval = false;
    mask = mask & DRYICE_ALL_WAC_MASK;

    /* specified LR and WAC bit(s) must be set */
    if ((mask == (mask & base->LR)) && (mask == (mask & base->WAC)))
    {
        retval = true;
    }
    return retval;
}

static bool dryice_IsRegisterReadAllowed(DRY_Type *base, uint32_t mask)
{
    bool retval;

    retval = false;
    mask = mask & DRYICE_ALL_WAC_MASK;

    /* specified RAC bit(s) must be set */
    if (mask == (mask & base->RAC))
    {
        retval = true;
    }
    return retval;
}

static bool dryice_IsKeyReadAllowed(DRY_Type *base, uint32_t keyRegisters)
{
    bool retval;
    uint32_t mask;
    uint32_t drySrac;
    uint32_t drySkrlr;
    uint32_t i;

    retval = true;
    mask = 1u;

    /* limit argument to maximum */
    if (keyRegisters > ARRAY_SIZE(base->SKR))
    {
        keyRegisters = ARRAY_SIZE(base->SKR);
    }

    /* specified SRAC and SKRLR bit(s) must be set */
    drySrac = base->SRAC;
    drySkrlr = base->SKRLR;
    for (i = 0; i < keyRegisters; i++)
    {
        if (((mask & drySrac) == 0) || ((mask & drySkrlr) == 0))
        {
            retval = false;
            break;
        }
    }

    return retval;
}

static bool dryice_IsKeyWriteAllowed(DRY_Type *base, uint32_t keyRegisters)
{
    bool retval;
    uint32_t mask;
    uint32_t drySwac;
    uint32_t drySkwlr;
    uint32_t i;

    retval = true;
    mask = 1u;

    /* limit argument to maximum */
    if (keyRegisters > ARRAY_SIZE(base->SKR))
    {
        keyRegisters = ARRAY_SIZE(base->SKR);
    }

    /* specified SWAC and SKWLR bit(s) must be set */
    drySwac = base->SWAC;
    drySkwlr = base->SKWLR;
    for (i = 0; i < keyRegisters; i++)
    {
        if (((mask & drySwac) == 0) || ((mask & drySkwlr) == 0))
        {
            retval = false;
            break;
        }
    }

    /* if the DryIce tamper flag is set the secure key storage is held in reset and so cannot be written */
    if (retval && (base->SR & DRY_SR_DTF_MASK))
    {
        retval = false;
    }

    return retval;
}

static status_t dryice_PinConfigure(DRY_Type *base, const dryice_pin_config_t *pinConfig, uint32_t pin)
{
    uint32_t temp;
    uint32_t mask;
    status_t status;

    if ((dryice_IsRegisterWriteAllowed(base, DRY_LR_PDL_MASK | DRY_LR_PPL_MASK | ((1u << DRY_LR_GFL_SHIFT) << pin))) &&
        (pinConfig != NULL))
    {
        /* pin 0 to 7 selects bit0 to bit7 */
        mask = (1u << pin);

        /* Pin Direction Register */
        temp = base->PDR;
        temp &= ~mask; /* clear the bit */
        if (kDRYICE_TamperPinDirectionOut == pinConfig->pinDirection)
        {
            temp |= mask; /* set the bit, if configured */
        }
        base->PDR = temp;

        /* Pin Polarity Register */
        temp = base->PPR;
        temp &= ~mask; /* clear the bit */
        if (kDRYICE_TamperPinPolarityExpectInverted == pinConfig->pinPolarity)
        {
            temp |= mask; /* set the bit, if configured */
        }
        base->PPR = temp;

        /* compute and set the configured value to the glitch filter register */
        temp = 0;
        temp |= DRY_PGFR_GFW(pinConfig->glitchFilterWidth);
        temp |= DRY_PGFR_GFP(pinConfig->glitchFilterPrescaler);
#if defined(FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_SAMPLING) && (FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_SAMPLING > 0)
        temp |= DRY_PGFR_TPSW(pinConfig->tamperPinSampleWidth);
        temp |= DRY_PGFR_TPSF(pinConfig->tamperPinSampleFrequency);
#endif
        temp |= DRY_PGFR_TPEX(pinConfig->tamperPinExpected);
        temp |= DRY_PGFR_TPE(pinConfig->tamperPullEnable);
#if defined(FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_PULL_SELECT) && (FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_PULL_SELECT > 0)
        temp |= DRY_PGFR_TPS(pinConfig->tamperPullSelect);
#endif
        /* make sure the glitch filter is disabled when we configure glitch filter width */
        base->PGFR[pin] = temp;
        /* add glitch filter enabled */
        if (pinConfig->glitchFilterEnable)
        {
            temp |= DRY_PGFR_GFE(1u);
            base->PGFR[pin] = temp;
        }
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

static status_t dryice_ActiveTamperConfigure(DRY_Type *base,
                                             const dryice_active_tamper_config_t *activeTamperConfig,
                                             uint32_t activeTamperRegister)
{
    uint32_t temp;
    status_t status;

    /* check if writing to active tamper register is allowed */
    if ((dryice_IsRegisterWriteAllowed(base, (1u << DRY_LR_ATL_SHIFT) << activeTamperRegister)) &&
        (activeTamperConfig != NULL))
    {
        /* compute and set the configured value to the active tamper register */
        temp = 0;
        temp |= DRY_ATR_ATSR(activeTamperConfig->activeTamperShift);
        temp |= DRY_ATR_ATP(activeTamperConfig->activeTamperPolynomial);
        base->ATR[activeTamperRegister] = temp;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

void DRYICE_Init(DRY_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* ungate clock */
    CLOCK_EnableClock(kCLOCK_Dryice0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    CLOCK_EnableClock(kCLOCK_Secreg0);
}

void DRYICE_Deinit(DRY_Type *base)
{
    uint32_t i;

    /* disable all glitch filters and active tampers */
    for (i = 0; i < ARRAY_SIZE(base->PGFR); i++)
    {
        base->PGFR[i] = 0;
    }
    for (i = 0; i < ARRAY_SIZE(base->ATR); i++)
    {
        base->ATR[i] = 0;
    }

    /* disable innter Dryice clock and prescaler */
    base->CR &= ~DRY_CR_DEN_MASK;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* gate clock */
    CLOCK_DisableClock(kCLOCK_Dryice0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    CLOCK_DisableClock(kCLOCK_Secreg0);
}

void DRYICE_GetDefaultConfig(DRY_Type *base, dryice_config_t *defaultConfig)
{
    struct _dryice_config myDefaultConfig =
    {
        true,                         /* innerClockAndPrescalerEnable */
        false,                        /* tamperForceSystemResetEnable */
        kDRYICE_StatusLockWithTamper, /* updateMode */
#if defined(FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT) && \
    (FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT > 0)
        kDRYICE_ClockType1Hz, /* clockSourceActiveTamper0 */
        kDRYICE_ClockType1Hz, /* clockSourceActiveTamper1 */
#endif
        kDRYICE_Hysteresis305mV,  /* tamperHysteresisSelect */
        false,                    /* tamperPassiveFilterEnable */
        kDRYICE_DriveStrengthLow, /* tamperDriveStrength */
        kDRYICE_SlewRateSlow,     /* tamperSlewRate */
#if defined(FSL_FEATURE_DRYICE_HAS_SECURE_REGISTER_FILE_SELECT) && \
    (FSL_FEATURE_DRYICE_HAS_SECURE_REGISTER_FILE_SELECT > 0)
        kDRYICE_VbatRegisterResetWithTamperOrInterrupt, /* secureRegisterFile */
#endif
        0, /* prescaler */
    };

    *defaultConfig = myDefaultConfig;
}

status_t DRYICE_SetConfig(DRY_Type *base, const dryice_config_t *config)
{
    uint32_t dryCR;
#if defined(FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT) && \
    (FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT > 0)
    uint32_t crATCS;
#endif
    status_t retval;

    /* check if writing to CR is allowed */
    if ((dryice_IsRegisterWriteAllowed(base, DRY_LR_CRL_MASK)) && (config != NULL))
    {
        /* compute CR value */
        dryCR = 0;
        dryCR |= DRY_CR_TFSR(config->tamperForceSystemResetEnable);
        dryCR |= DRY_CR_UM(config->updateMode);
#if defined(FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT) && \
    (FSL_FEATURE_DRYICE_HAS_ACTIVE_TAMPER_CLOCK_SOURCE_SELECT > 0)
        crATCS = (uint32_t)config->clockSourceActiveTamper0;
        crATCS |= ((uint32_t)config->clockSourceActiveTamper1) << 1u;
        dryCR |= DRY_CR_ATCS(crATCS);
#endif
        dryCR |= DRY_CR_THYS(config->tamperHysteresisSelect);
        dryCR |= DRY_CR_TPFE(config->tamperPassiveFilterEnable);
        dryCR |= DRY_CR_TDSE(config->tamperDriveStrength);
        dryCR |= DRY_CR_TSRE(config->tamperSlewRate);
#if defined(FSL_FEATURE_DRYICE_HAS_SECURE_REGISTER_FILE_SELECT) && \
    (FSL_FEATURE_DRYICE_HAS_SECURE_REGISTER_FILE_SELECT > 0)
        dryCR |= DRY_CR_SRF(config->secureRegisterFile);
#endif
        dryCR |= DRY_CR_DPR(config->prescaler);
        /* write the computed value to the CR register */
        base->CR = dryCR;
        /* after the prescaler is written to CR register, enable the inner dryice clock and prescaler */
        if (config->innerClockAndPrescalerEnable)
        {
            base->CR = dryCR | DRY_CR_DEN_MASK;
        }
        retval = kStatus_Success;
    }
    else
    {
        retval = kStatus_Fail;
    }

    return retval;
}

status_t DRYICE_SoftwareReset(DRY_Type *base)
{
    status_t retval;

    /* check if writing to CR is allowed */
    if (dryice_IsRegisterWriteAllowed(base, DRY_LR_CRL_MASK))
    {
        /* set the CR[SWR] */
        base->CR = DRY_CR_SWR_MASK;
        retval = kStatus_Success;
    }
    else
    {
        retval = kStatus_Fail;
    }

    return retval;
}

status_t DRYICE_ActiveTamperSetConfig(DRY_Type *base,
                                      const dryice_active_tamper_config_t *activeTamperConfig,
                                      uint32_t activeTamperRegisterSelect)
{
    uint32_t mask;
    status_t status;
    uint32_t i;

    mask = 1u;
    status = kStatus_Success;

    /* configure active tamper register by active tamper register, by moving through all active tamper registers */
    for (i = 0; i < ARRAY_SIZE(base->ATR); i++)
    {
        if (activeTamperRegisterSelect & mask)
        {
            /* configure this active tamper register */
            status = dryice_ActiveTamperConfigure(base, activeTamperConfig, i);
            if (status != kStatus_Success)
            {
                break;
            }
        }
        mask = mask << 1u;
    }

    return status;
}

void DRYICE_PinGetDefaultConfig(DRY_Type *base, dryice_pin_config_t *pinConfig)
{
    struct _dryice_pin_config myPinDefaultConfig =
    {
        kDRYICE_TamperPinDirectionIn,          /* pinDirection */
        kDRYICE_TamperPinPolarityExpectNormal, /* pinPolarity */
        0,                                     /* glitchFilterWidth */
        kDRYICE_GlitchFilterClock512Hz,        /* glitchFilterPrescaler */
        false,                                 /* glitchFilterEnable */
#if defined(FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_SAMPLING) && (FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_SAMPLING > 0)
        kDRYICE_GlitchFilterSampleDisable,       /* tamperPinSampleWidth */
        kDRYICE_GlitchFilterSamplingEveryCycle8, /* tamperPinSampleFrequency */
#endif
        kDRYICE_GlitchFilterExpectedLogicZero, /* tamperPinExpected */
        false,                                 /* tamperPullEnable */
#if defined(FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_PULL_SELECT) && (FSL_FEATURE_DRYICE_HAS_TAMPER_PIN_PULL_SELECT > 0)
        kDRYICE_GlitchFilterPullTypeAssert, /* tamperPullSelect */
#endif
    };

    *pinConfig = myPinDefaultConfig;
}

status_t DRYICE_PinSetConfig(DRY_Type *base, const dryice_pin_config_t *pinConfig, uint32_t pinSelect)
{
    uint32_t mask;
    status_t status;
    uint32_t i;

    mask = 1u;
    status = kStatus_Success;

    /* configure pin by pin, by moving through all selected pins */
    for (i = 0; i < ARRAY_SIZE(base->PGFR); i++)
    {
        if (pinSelect & mask)
        {
            /* clear this pin from pinSelect */
            pinSelect &= ~mask;

            /* configure this pin */
            status = dryice_PinConfigure(base, pinConfig, i);

            /* if pinSelect is zero, we have configured all pins selected by pinSelect, so skip */
            if ((status != kStatus_Success) || (0 == pinSelect))
            {
                break;
            }
        }
        mask = mask << 1u;
    }

    return status;
}

status_t DRYICE_GetKey(DRY_Type *base, uint8_t *key, size_t keySize)
{
    status_t status;
    uint32_t keyRegisters;
    uint32_t readKey[ARRAY_SIZE(base->SKR)] = {0};
    void *pKey;
    uint32_t i;

    /* compute number of 32-bit secure key registers to cover keySize bytes */
    keyRegisters = keySize / sizeof(uint32_t);
    if (keySize % sizeof(uint32_t))
    {
        keyRegisters++;
    }
    if (keyRegisters > ARRAY_SIZE(base->SKR))
    {
        keyRegisters = ARRAY_SIZE(base->SKR);
    }

    if ((dryice_IsKeyReadAllowed(base, keyRegisters)) && (key != NULL))
    {
        /* read locally the secure key registers */
        for (i = 0; i < keyRegisters; i++)
        {
            readKey[i] = base->SKR[i];
        }

        /* memcpy to user buffer */
        pKey = readKey;
        for (i = 0; i < keySize; i++)
        {
            key[i] = ((uint8_t *)pKey)[i];
        }

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_WriteKey(DRY_Type *base, const uint8_t *key, size_t keySize)
{
    status_t status = kStatus_Fail;
    uint32_t keyRegisters;
    uint32_t writeKey[ARRAY_SIZE(base->SKR)] = {0};
    void *pKey;
    uint32_t i;

    /* compute number of 32-bit secure key registers to cover keySize bytes */
    keyRegisters = keySize / sizeof(uint32_t);
    if (keySize % sizeof(uint32_t))
    {
        keyRegisters++;
    }
    if (keyRegisters > ARRAY_SIZE(base->SKR))
    {
        keyRegisters = ARRAY_SIZE(base->SKR);
    }

    /* invalidate secure key registers */
    if (key && keyRegisters)
    {
        base->SKVR = 0xFFu;
        status = kStatus_Success;
    }

    if ((kStatus_Success == status) && (dryice_IsKeyWriteAllowed(base, keyRegisters)))
    {
        /* memcpy to local buffer */
        pKey = writeKey;
        for (i = 0; i < keySize; i++)
        {
            ((uint8_t *)pKey)[i] = key[i];
        }

        /* write the secure key registers */
        for (i = 0; i < keyRegisters; i++)
        {
            base->SKR[i] = writeKey[i];
        }
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_GetStatusFlags(DRY_Type *base, uint32_t *result)
{
    status_t status;

    if ((dryice_IsRegisterReadAllowed(base, DRY_RAC_SRR_MASK)) && (result != NULL))
    {
        *result = base->SR;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_ClearStatusFlags(DRY_Type *base, uint32_t mask)
{
    status_t status;

    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_SRW_MASK))
    {
        base->SR = mask;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_EnableInterrupts(DRY_Type *base, uint32_t mask)
{
    status_t status;

    mask = mask & DRYICE_ALL_IER_MASK; /* only set the bits documented in Reference Manual. */
    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_IEW_MASK))
    {
        base->IER |= mask;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_DisableInterrupts(DRY_Type *base, uint32_t mask)
{
    status_t status;

    mask = mask & DRYICE_ALL_IER_MASK; /* only clear the bits documented in Reference Manual.  */
    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_IEW_MASK))
    {
        base->IER &= ~mask;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_EnableTampers(DRY_Type *base, uint32_t mask)
{
    status_t status;

    mask = mask & DRYICE_ALL_TER_MASK; /* only set the bits documented in Reference Manual */
    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_TEW_MASK))
    {
        base->TER |= mask;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_DisableTampers(DRY_Type *base, uint32_t mask)
{
    status_t status;

    mask = mask & DRYICE_ALL_TER_MASK; /* only clear the bits documented in Reference Manual */
    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_TEW_MASK))
    {
        base->TER &= ~mask;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_ForceTamper(DRY_Type *base)
{
    status_t status;

    if (dryice_IsRegisterWriteAllowed(base, DRY_WAC_TSRW_MASK))
    {
        base->TSR = 0;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t DRYICE_GetTamperTimeSeconds(DRY_Type *base, uint32_t *tamperTimeSeconds)
{
    status_t status;

    if ((dryice_IsRegisterReadAllowed(base, DRY_RAC_TSRR_MASK)) && (tamperTimeSeconds != NULL))
    {
        *tamperTimeSeconds = base->TSR;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

void DRYICE_LockRegisters(DRY_Type *base, uint32_t mask)
{
    mask &= kDRYICE_AllRegisters; /* make sure only documented registers are selected by the mask */
    base->LR &= ~mask;            /* clear the selected bits */
}

void DRYICE_LockSecureKeys(DRY_Type *base, uint32_t mask, dryice_readwrite_t readWrite)
{
    mask &= kDRYICE_AllSecureKeys; /* make sure only documented secure key registers are selected by the mask */
    if ((readWrite == kDRYICE_Read) || (readWrite == kDRYICE_ReadWrite))
    {
        base->SKRLR &= ~mask;
    }
    if ((readWrite == kDRYICE_Write) || (readWrite == kDRYICE_ReadWrite))
    {
        base->SKWLR &= ~mask;
    }
}

void DRYICE_DisableAccess(DRY_Type *base, uint32_t maskRegisters, uint32_t maskKeys, dryice_readwrite_t readWrite)
{
    maskRegisters &= kDRYICE_AllRegisters; /* make sure only documented registers are selected by the mask */
    maskKeys &= kDRYICE_AllSecureKeys;     /* make sure only documented secure key registers are selected by the mask */

    if ((readWrite == kDRYICE_Read) || (readWrite == kDRYICE_ReadWrite))
    {
        base->RAC &= ~maskRegisters;
        base->SRAC &= ~maskKeys;
    }
    if ((readWrite == kDRYICE_Write) || (readWrite == kDRYICE_ReadWrite))
    {
        base->WAC &= ~maskRegisters;
        base->SWAC &= ~maskKeys;
    }
}
