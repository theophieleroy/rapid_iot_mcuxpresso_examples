/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
 * All rights reserved.
 *
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "clock_config.h" /* for BOARD_XTAL0_CLK_HZ */
#include "power_manager.h"
#include "timer_manager.h"
#include "timer_adapter.h"
#if defined(CFG_BLE_PRJ)
#include "rwip.h"
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
power_manager_env_t g_pmEnv = {
    .req[kPmActive] = 0U, .req[kPmSleep] = 0U, .req[kPmPowerDown0] = 0U, .req[kPmPowerDown1] = 0U};
#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
static tm_timer_id_t s_xtal32KTimerID;
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(CFG_BLE_PRJ)
extern void APP_SaveBleReg(void);
extern void APP_RestoreBleReg(void);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
static void PM_Xtal32KReadyTimerHandler(void *param)
{
    CLOCK_AttachClk(kXTAL32K_to_32K_CLK);
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(1U));
    SYSCON->ANA_CTRL1 = SYSCON->ANA_CTRL1 & ~SYSCON_ANA_CTRL1_X32_SMT_EN_MASK;

#if defined(CFG_BLE_PRJ)
    /* Allow BLE sleep */
    enable_ble_sleep(true);
#endif
}

static inline void PM_Switch32kOff(void)
{
#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK, SYSCON_PMU_CTRL1_XTAL32K_DIS(1U));
#else
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(1U));
#endif
}

static inline void PM_Switch32kOn(void)
{
#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
    SYSCON->ANA_CTRL1 = SYSCON->ANA_CTRL1 | SYSCON_ANA_CTRL1_X32_SMT_EN_MASK;
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK | SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK,
                        SYSCON_PMU_CTRL1_RCO32K_DIS(0U) | SYSCON_PMU_CTRL1_XTAL32K_DIS(0U));

    CLOCK_AttachClk(kRCO32K_to_32K_CLK);

    TM_SetTimer(s_xtal32KTimerID, kTM_SingleShotTimer, TA_SECOND(1), (tm_callback_t)PM_Xtal32KReadyTimerHandler, NULL);

#if defined(CFG_BLE_PRJ)
    /* Prevent BLE sleep */
    enable_ble_sleep(false);
#endif
#else
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(0U));
#endif
}

static inline void PM_CheckSysPowerMode(void)
{
    /* Check user policy and user events */
    if (g_pmEnv.req[kPmActive] & ~(kPmReqGpio))
    {
        /* Stay active */
        return;
    }

    /* Check GPIO wakeup source */
    if (g_pmEnv.req[kPmSleep] || (POWER_GpioActiveRequest() == false))
    {
        /* Clear Gpio active request */
        PM_ClrReq(kPmReqGpio);
    }
    else
    {
        /* Stay active */
        PM_SetReq(kPmActive, kPmReqGpio);
    }
}

#if defined(CFG_BLE_PRJ)
/*!
 * @brief Wait until ble core is in power down mode.
 *
 * Should be used together with rwip_sleep(), i.e. PM_CheckBlePowerMode().
 * If rwip_sleep() returns kPmPowerDown0/kPmPowerDown1, ble core will enter power down in 3~4 32k cycles.
 * After that, High frequency clock source can be switched to OSC32M. So use this function to make the chip
 * to stay in sleep mode until ble core is power down, which is indicated by SYSCON_SYS_STAT_OSC_EN bit.
 */
static inline void PM_WaitBlePd(void)
{
    NVIC_ClearPendingIRQ(OSC_INT_LOW_IRQn);
    NVIC_EnableIRQ(OSC_INT_LOW_IRQn);
    while (SYSCON_SYS_STAT_OSC_EN_MASK & SYSCON->SYS_STAT)
    {
        POWER_EnterSleep();
    }
    NVIC_DisableIRQ(OSC_INT_LOW_IRQn);
}

static inline void PM_BlePreWorkaround(void)
{
    /* Power down affects calibration's FSM, turn it off before power down.
     * Turn off Ble core's high frequency clock before power down.
     */
    SYSCON->CLK_DIS = SYSCON_CLK_DIS_CLK_CAL_DIS_MASK | SYSCON_CLK_DIS_CLK_BLE_DIS_MASK;
}

static inline void PM_BlePostWorkaround(void)
{
    /* Power down affects calibration's FSM, turn it on again after waking from power down.
     * After waking up from power down, XTAL will be turned on automatically. Ble core's high clock source
       can be turn on as soon as possible, so do it in POWER_EnterPowerDown(). */
    SYSCON->CLK_EN = SYSCON_CLK_EN_CLK_CAL_EN_MASK /* | SYSCON_CLK_DIS_CLK_BLE_DIS_MASK */;
    /* imr = 1 */
    CALIB->VCO_MOD_CFG |= CALIB_VCO_MOD_CFG_IMR_MASK;
    /* AA error */
    BLEDP->DP_AA_ERROR_CTRL = 0x0000000EU;
}

/*!
 * @brief Check ble core's status and set power mode request for ble core.
 *
 * If rwip_sleep returns kPmPowerDown0/1, PM_WaitBlePd() must be called before
 * switching high frequency clock source form crystal to OSC32M.
 */
static inline void PM_CheckBlePowerMode(void)
{
    /*
     * rwip_sleep() returns ble core's status as follows.
     * kPmActive:     there are still some kernel events to be processed by KE_Schedule()
     * kPmSleep:      no kernel event is pending, but next ble event/kernel timer will come up soon
                          (time shorter than crystal wakeup delay).
     * kPmPowerDown0: next ble event/kernel timer is quite far away(time longer than crystal wakeup delay),
     *                    32k clock source is needed for timing.
     * kPmPowerDown1: no ble event/kernel timer is ongoing, 32k clock source can be shut down.
     */
    uint8_t pm = rwip_sleep();

    PM_SetReq((power_mode_t)pm, kPmReqBle);
}
#endif

/*!
 * @brief Power down workflow of the chip, when using ble.
 */
static void PM_SetPowerDownMode(power_mode_t pm)
{
    uint32_t backWakeupEn[2U];
    uint32_t backWakeupLvl[2U];
    uint8_t xtalDivRestore = 0;

#if defined(CFG_BLE_PRJ)
    PM_WaitBlePd();

    /* Enable OSC_EN wakeup */
    NVIC_ClearPendingIRQ(OSC_IRQn);
    NVIC_EnableIRQ(OSC_IRQn);
    APP_SaveBleReg();
#endif

    /* power on OSC32M and switch to it */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(0U));
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) | SYSCON_CLK_CTRL_SYS_CLK_SEL(0U);

    /* Set Xtal divider before the crystal is ready */
    if ((SYSCON->XTAL_CTRL & SYSCON_XTAL_CTRL_XTAL_DIV_MASK) == 0)
    {
        xtalDivRestore = 1;
        SYSCON->XTAL_CTRL |= SYSCON_XTAL_CTRL_XTAL_DIV_MASK;
    }

    if (pm == kPmPowerDown1)
    {
        PM_Switch32kOff();
    }

#if defined(CFG_SWD_WAKEUP)
    POWER_EnableSwdWakeup();
#endif

    /* Enable GPIO wakeup */
    NVIC_ClearPendingIRQ(EXT_GPIO_WAKEUP_IRQn);
    NVIC_EnableIRQ(EXT_GPIO_WAKEUP_IRQn);

    /* Backup GPIO wakeup setting for GPIO wakeup source interrupt generating */
    backWakeupEn[0U] = SYSCON->PIO_WAKEUP_EN0;
    backWakeupEn[1U] = SYSCON->PIO_WAKEUP_EN1 &
                       (SYSCON_PIO_WAKEUP_EN1_PB00_WAKEUP_EN_MASK | SYSCON_PIO_WAKEUP_EN1_PB01_WAKEUP_EN_MASK |
                        SYSCON_PIO_WAKEUP_EN1_PB02_WAKEUP_EN_MASK);
    backWakeupLvl[0U] = SYSCON->PIO_WAKEUP_LVL0;
    backWakeupLvl[1U] = SYSCON->PIO_WAKEUP_LVL1;

    POWER_LatchIO();

#if defined(CFG_BLE_PRJ)
    PM_BlePreWorkaround();
#endif

    POWER_EnterPowerDown(0);

#if defined(CFG_BLE_PRJ)
    PM_BlePostWorkaround();
#endif

    POWER_RestoreIO();

    /* To generate a pending interrupt for GPIO wakeup source */
    if (backWakeupEn[0U])
    {
        GPIOA->INTTYPECLR = backWakeupEn[0U];
        GPIOA->INTPOLCLR = backWakeupEn[0U] & backWakeupLvl[0U];  /* low level interrupt (wakeup) */
        GPIOA->INTPOLSET = backWakeupEn[0U] & ~backWakeupLvl[0U]; /* high level interrupt (wakeup) */
        GPIOA->INTENSET = backWakeupEn[0U];
    }
    if (backWakeupEn[1U])
    {
        GPIOB->INTTYPECLR = backWakeupEn[1U];
        GPIOB->INTPOLCLR = backWakeupEn[1U] & backWakeupLvl[1U];  /* low level interrupt (wakeup) */
        GPIOB->INTPOLSET = backWakeupEn[1U] & ~backWakeupLvl[1U]; /* high level interrupt (wakeup) */
        GPIOB->INTENSET = backWakeupEn[1U];
    }
    NVIC_DisableIRQ(EXT_GPIO_WAKEUP_IRQn);
    NVIC_ClearPendingIRQ(EXT_GPIO_WAKEUP_IRQn);

#if defined(CFG_SWD_WAKEUP)
    POWER_RestoreSwd();
    if (!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
    {
        /* if waked up by SWDIO */
        if (NVIC_GetPendingIRQ(GPIOA_IRQn) && (GPIOA->INTSTATUS & SWDIO_GPIO_PIN_MASK))
        {
            GPIOA->INTENCLR = SWDIO_GPIO_PIN_MASK;
            GPIOA->INTSTATUS = SWDIO_GPIO_PIN_MASK;
            NVIC_ClearPendingIRQ(GPIOA_IRQn);
            /* prevent MCU enter into PD */
            PM_SetReq(kPmActive, kPmReqUserPolicy);
        }
    }
#endif

    if (pm == kPmPowerDown1)
    {
        PM_Switch32kOn();
    }

#if defined(CFG_BLE_PRJ)
    APP_RestoreBleReg();
    NVIC_DisableIRQ(OSC_IRQn);
    if (NVIC_GetPendingIRQ(OSC_IRQn))
    {
        NVIC_ClearPendingIRQ(OSC_IRQn);
        /* BLE wakeup is onging (moved from OSC interrupt handler) */
        rwip_prevent_sleep_set(RW_WAKE_UP_ONGOING);
    }
#endif

    /* wait until XTAL is ready, and switch to it */
    NVIC_ClearPendingIRQ(XTAL_READY_IRQn);
    NVIC_EnableIRQ(XTAL_READY_IRQn);

    while (!(SYSCON_SYS_MODE_CTRL_XTAL_RDY_MASK & SYSCON->SYS_MODE_CTRL))
    {
        POWER_EnterSleep();
    }

    NVIC_DisableIRQ(XTAL_READY_IRQn);
    NVIC_ClearPendingIRQ(XTAL_READY_IRQn);

    if (xtalDivRestore)
    {
        SYSCON->XTAL_CTRL &= ~SYSCON_XTAL_CTRL_XTAL_DIV_MASK;
    }

    /* switch to XTAL and power off OSC32M */
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) | SYSCON_CLK_CTRL_SYS_CLK_SEL(1U);
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(1U));

    BOARD_WakeupRestore();
}

void PM_Init(void)
{
    uint32_t msk, val;

    POWER_Init();

    POWER_EnablePD(kPDRUNCFG_PD_FIR);
    POWER_EnablePD(kPDRUNCFG_PD_FSP);
    POWER_EnablePD(kPDRUNCFG_PD_OSC32M);

#if defined(CFG_BLE_PRJ)
    /* Enable OSC_EN as interrupt and wakeup source */
    SYSCON->PMU_CTRL0 = SYSCON->PMU_CTRL0 | SYSCON_PMU_CTRL0_OSC_INT_EN_MASK;
#endif

#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
    s_xtal32KTimerID = TM_AllocateTimer();

    val = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS(0U)  /* switch on XTAL32K during power down */
          | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS(1U) /* switch off RCO32K during power down */
          | SYSCON_PMU_CTRL1_XTAL32K_DIS(0U)    /* switch on XTAL32K */
          | SYSCON_PMU_CTRL1_RCO32K_DIS(1U);    /* switch off RCO32K at all time */
#else
    val = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS(1U)  /* switch off XTAL32K during power down */
          | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS(0U) /* switch on RCO32K during power down */
          | SYSCON_PMU_CTRL1_XTAL32K_DIS(1U)    /* switch off XTAL32K at all time */
          | SYSCON_PMU_CTRL1_RCO32K_DIS(0U);    /* switch on RCO32K */
#endif

    msk = SYSCON_PMU_CTRL1_XTAL32K_PDM_DIS_MASK | SYSCON_PMU_CTRL1_RCO32K_PDM_DIS_MASK |
          SYSCON_PMU_CTRL1_XTAL32K_DIS_MASK | SYSCON_PMU_CTRL1_RCO32K_DIS_MASK;

    /* The default setting of capacitive sensor, DAC, ADC and USB PLL's power are disabled.
       User should power on these peripherals when using them. */
    POWER_WritePmuCtrl1(SYSCON, msk, val);
}

void PM_PowerManagement(void)
{
    uint32_t primask = DisableGlobalIRQ();
    uint8_t exitFromPD = 0;

    PM_CheckSysPowerMode();

    if (g_pmEnv.req[kPmActive] != 0U)
    {
        /* Stay in active mode */
    }
    else
    {
#if defined(CFG_BLE_PRJ)
        PM_CheckBlePowerMode();
#endif
        if (g_pmEnv.req[kPmActive] != 0U)
        {
            /* Stay in active mode */
        }
        else if (g_pmEnv.req[kPmSleep] != 0U)
        {
            /* Enter sleep */
            POWER_EnterSleep();
        }
        else if (g_pmEnv.req[kPmPowerDown0] != 0U)
        {
            /* Enter power down 0 */
            PM_SetPowerDownMode(kPmPowerDown0);
            exitFromPD = 1;
        }
        else
        {
            /* Enter power down 1 */
            PM_SetPowerDownMode(kPmPowerDown1);
            exitFromPD = 1;
        }
        PM_ClrReq(kPmReqBle);
    }

    EnableGlobalIRQ(primask);

    if (exitFromPD)
    {
        /* To restore something for user's application if needed */
        APP_WakeupRestore();
    }
}
