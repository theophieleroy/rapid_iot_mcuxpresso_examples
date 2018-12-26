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

/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v4.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VMD12
mcu_data: ksdk2_0
processor_version: 3.0.1
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define OSC_CAP0P                                         0U  /*!< Oscillator 0pF capacitor load */
#define OSC_ER_CLK_DISABLE                                0U  /*!< Disable external reference clock */
#define SIM_OSC32KSEL_OSC32KCLK_CLK                       0U  /*!< OSC32KSEL select: OSC32KCLK clock */
#define SIM_PLLFLLSEL_IRC48MCLK_CLK                       3U  /*!< PLLFLL select: IRC48MCLK clock */
#define SIM_USB_CLK_48000000HZ                     48000000U  /*!< Input SIM frequency for USB: 48000000Hz */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_EnableIrc48MOsc
 * Description   : This function is used to enabling IRC48M oscillator for K60_1M
 * as workaround because there is not enabled the oscillator automatically.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_EnableIrc48MOsc()
{
  /* USB clock gate enable */
  CLOCK_EnableClock(kCLOCK_Usbfs0);
  /* IRC48M oscillator enable */
  USB0->CLK_RECOVER_IRC_EN = USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK | USB_CLK_RECOVER_IRC_EN_REG_EN_MASK;
  /* USB clock gate disable */
  CLOCK_DisableClock(kCLOCK_Usbfs0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetFllExtRefDiv
 * Description   : Configure FLL external reference divider (FRDIV).
 * Param frdiv   : The value to set FRDIV.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetFllExtRefDiv(uint8_t frdiv)
{
    MCG->C1 = ((MCG->C1 & ~MCG_C1_FRDIV_MASK) | MCG_C1_FRDIV(frdiv));
}

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_Boot_Clock_RUN();
}

/*******************************************************************************
 ********************* Configuration BOARD_Boot_Clock_RUN **********************
 ******************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_Boot_Clock_RUN
called_from_default_init: true
outputs:
- {id: Bus_clock.outFreq, value: 60 MHz, locked: true, accuracy: '0.001'}
- {id: Core_clock.outFreq, value: 120 MHz, locked: true, accuracy: '0.001'}
- {id: Flash_clock.outFreq, value: 24 MHz, locked: true, accuracy: '0.001'}
- {id: FlexBus_clock.outFreq, value: 24 MHz, locked: true, accuracy: '0.001'}
- {id: IRC48MCLK.outFreq, value: 48 MHz}
- {id: LPO_clock.outFreq, value: 1 kHz}
- {id: MCGIRCLK.outFreq, value: 2 MHz}
- {id: PLLFLLCLK.outFreq, value: 48 MHz}
- {id: System_clock.outFreq, value: 120 MHz, locked: true, accuracy: '0.001'}
- {id: USB48MCLK.outFreq, value: 48 MHz, locked: true, accuracy: '0.001'}
settings:
- {id: MCGMode, value: PEE}
- {id: MCG.IRCS.sel, value: MCG.FCRDIV}
- {id: MCG.IREFS.sel, value: MCG.FRDIV}
- {id: MCG.OSCSEL.sel, value: SIM.IRC48MCLK}
- {id: MCG.PLLS.sel, value: MCG.PLL}
- {id: MCG.PRDIV.scale, value: '12'}
- {id: MCG.VDIV.scale, value: '30'}
- {id: MCG_C1_IRCLKEN_CFG, value: Enabled}
- {id: MCG_C5_PLLCLKEN0_CFG, value: Enabled}
- {id: MCG_CG_PLLSTEN0_CFG, value: Enabled}
- {id: MCRFFCLKAllowConfig, value: 'no'}
- {id: SIM.OUTDIV2.scale, value: '2', locked: true}
- {id: SIM.OUTDIV3.scale, value: '5', locked: true}
- {id: SIM.OUTDIV4.scale, value: '5', locked: true}
- {id: SIM.PLLFLLSEL.sel, value: IRC48M.IRC48MCLK}
- {id: SIM.USBSRCSEL.sel, value: SIM.USBDIV}
- {id: USBClkConfig, value: 'yes'}
sources:
- {id: IRC48M.IRC48M.outFreq, value: 48 MHz}
- {id: OSC.OSC.outFreq, value: 12 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_Boot_Clock_RUN configuration
 ******************************************************************************/
const mcg_config_t mcgConfig_BOARD_Boot_Clock_RUN =
    {
        .mcgMode = kMCG_ModePEE,                  /* PEE - PLL Engaged External */
        .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
        .ircs = kMCG_IrcFast,                     /* Fast internal reference clock selected */
        .fcrdiv = 0x1U,                           /* Fast IRC divider: divided by 2 */
        .frdiv = 0x0U,                            /* FLL reference clock divider: divided by 1 */
        .drs = kMCG_DrsLow,                       /* Low frequency range */
        .dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
        .oscsel = kMCG_OscselIrc,                 /* Selects 48 MHz IRC Oscillator */
        .pll0Config =
            {
                .enableMode = kMCG_PllEnableIndependent | kMCG_PllEnableInStop,/* MCGPLLCLK enabled independently of MCG clock mode as well as in STOP mode */
                .prdiv = 0xbU,                    /* PLL Reference divider: divided by 12 */
                .vdiv = 0x6U,                     /* VCO divider: multiplied by 30 */
            },
    };
const sim_clock_config_t simConfig_BOARD_Boot_Clock_RUN =
    {
        .pllFllSel = SIM_PLLFLLSEL_IRC48MCLK_CLK, /* PLLFLL select: IRC48MCLK clock */
        .er32kSrc = SIM_OSC32KSEL_OSC32KCLK_CLK,  /* OSC32KSEL select: OSC32KCLK clock */
        .clkdiv1 = 0x1440000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /5, OUTDIV4: /5 */
    };
const osc_config_t oscConfig_BOARD_Boot_Clock_RUN =
    {
        .freq = 0U,                               /* Oscillator frequency: 0Hz */
        .capLoad = (OSC_CAP0P),                   /* Oscillator capacity load: 0pF */
        .workMode = kOSC_ModeExt,                 /* Use external clock */
        .oscerConfig =
            {
                .enableMode = OSC_ER_CLK_DISABLE, /* Disable external reference clock */
            }
    };

/*******************************************************************************
 * Code for BOARD_Boot_Clock_RUN configuration
 ******************************************************************************/
void BOARD_Boot_Clock_RUN(void)
{
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Enable IRC48M oscillator for K60_1M as workaround because there is not enabled the oscillator automatically. */
    CLOCK_CONFIG_EnableIrc48MOsc();
    /* Configure the Internal Reference clock (MCGIRCLK). */
    CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_Boot_Clock_RUN.irclkEnableMode,
                                  mcgConfig_BOARD_Boot_Clock_RUN.ircs, 
                                  mcgConfig_BOARD_Boot_Clock_RUN.fcrdiv);
    /* Configure FLL external reference divider (FRDIV). */
    CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_Boot_Clock_RUN.frdiv);
    /* Set MCG to PEE mode. */
    CLOCK_BootToPeeMode(mcgConfig_BOARD_Boot_Clock_RUN.oscsel,
                        kMCG_PllClkSelPll0,
                        &mcgConfig_BOARD_Boot_Clock_RUN.pll0Config);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_Boot_Clock_RUN);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOT_CLOCK_RUN_CORE_CLOCK;
    /* Enable USB FS clock. */
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, SIM_USB_CLK_48000000HZ);
}

