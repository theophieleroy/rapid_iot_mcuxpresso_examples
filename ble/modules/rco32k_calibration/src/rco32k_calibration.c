#include "clock_config.h" /* for BOARD_XTAL0_CLK_HZ */
#include "fsl_common.h"

#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ != CLK_XTAL_32KHZ))

#include "fsl_ctimer.h"
#include "rco32k_calibration.h"
#include "power_manager.h"
#include "timer_manager.h"
#include "timer_adapter.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RCO32K_CaptureEdgeCb(uint32_t flag);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static rco32k_calib_env_t s_rco32kEnv = {
    .ctimer = CTIMER3, /* use CTIMER3 for RCO32K's software calibration by default */
    .interval = 1000U, /* RCO32K software calibration interval 1000ms */
    .hwCodeFlag = 0};

/* when using kCTIMER_SingleCallback, this array has only one element */
static ctimer_callback_t s_CtimerCbArray[] = {
    RCO32K_CaptureEdgeCb,
};

static tm_timer_id_t s_rco32KTimerID;

/*******************************************************************************
 * Code
 ******************************************************************************/
int32_t RCO32K_FilterPpm(int32_t raw)
{
    static int32_t last_cooked = 0xFFFF;

    int32_t cooked;

    if (last_cooked != 0xFFFF)
    {
        cooked = last_cooked + raw;
        if (cooked < 0)
            cooked = -(-cooked >> 1U);
        else
            cooked = cooked >> 1U;
        last_cooked = cooked;
    }
    else
    {
        cooked = raw;
        last_cooked = raw;
    }

    return cooked;
}

/*!
 * @brief Calculate the ppm value of RCO32K and a) set it to RTC calibration register,
 *  b) pass it to ble stack.
 *
 *  ppm = (0x100000ull * 32000(Hz) * cnt) / (apb_clock(Hz) * ncycle(32K cycle count))
 * If apb clock is  8MHz, ppm = (0x8000 / 1) * cnt / 125
 * If apb clock is 16MHz, ppm = (0x8000 / 2) * cnt / 125
 * If apb clock is 32MHz, ppm = (0x8000 / 4) * cnt / 125
 *
 * @param cnt  delta of ctimer counter value during actual 16 32k cycles.
 *             During 16 precise 32k cycles(500us), if APB is 16M, cnt equals 8000.
 */
static inline void RCO32K_CalcPpm(uint32_t cnt)
{
    uint32_t dir = 0U;
    int32_t real_ppm = 0;
    uint32_t ppm;
    uint32_t tmp;

    /* suppress warning */
    dir = dir;
    real_ppm = real_ppm;

    tmp = (CLOCK_GetFreq(kCLOCK_ApbClk) / 8000000U);
    tmp = 0x8000U / tmp * cnt;

    /* 1/125 ~= 2^(-7) + 2^(-13) + 2^(-14) + 2^(-18) + 2^(-21) + 2^(-24) + 2^(-25) + 2^(-26)
     * deviation: ~1.3e-9 */
    ppm = (tmp >> 7U) + (tmp >> 13U) + (tmp >> 14U) + (tmp >> 18U) + (tmp >> 21U) + (tmp >> 24U) + (tmp >> 25U) +
          (tmp >> 26U);

    if (ppm > 0x100000U)
    {
        ppm = ppm - 0x100000U;
        dir = 0U;
    }
    else
    {
        ppm = 0x100000U - ppm;
        dir = 1U;
    }

    if (ppm > 0xBFFFU)
    {
        s_rco32kEnv.hwCodeFlag = (dir ? 1 : -1);
    }

    /* write to rtc calibration register */
    RTC_Calibration(RTC, dir ? kRTC_BackwardCalibration : kRTC_ForwardCalibration, ppm);

    real_ppm = (dir) ? -ppm : ppm;

    set_32k_ppm(real_ppm);
}

/*!
 * @brief Deal with 32k clock capture interrupt of ctimer.
 *
 * 32k's rise edge interrupt happens every ~31.25us, if any critical section blocks the interrupt
 * for long period, interrupt may get lost. So if delta of current capture value and last capture
 * value is multiple of ~31.25us(250 apb ticks in case of 8M apb), we need to compensate software
 * capture counter. And what's more, the accuracy of 32k rco is +/-6%, worst case value
 * 250 * (1 - 6%) = 235 is used as divisor.
 *
 * @param flag Interrupt flag of ctimer.
 */
static void RCO32K_CaptureEdgeCb(uint32_t flag)
{
    uint32_t cap = 0U;
    uint32_t delta = 0U;
    uint32_t cnt = 0U;

    if (flag & CTIMER_IR_CR2INT_MASK)
    {
        cap = s_rco32kEnv.ctimer->CR[kCTIMER_Capture_2];
        s_rco32kEnv.cnt++;

        if (s_rco32kEnv.cnt == RCO32K_1ST_EDGE)
        {
            s_rco32kEnv.firstVal = cap;
        }
        else
        {
            /* compensate software capture counter */
            delta = cap - s_rco32kEnv.lastVal;
            s_rco32kEnv.cnt += ((delta / (235U * (CLOCK_GetFreq(kCLOCK_ApbClk) / 8000000U))) - 1U);

            if (s_rco32kEnv.cnt >= RCO32K_17TH_EDGE)
            {
                s_rco32kEnv.ctimer->CCR &= ~(CTIMER_CCR_CAP2RE_MASK | CTIMER_CCR_CAP2FE_MASK | CTIMER_CCR_CAP2I_MASK);

                if (CTIMER_GetStatusFlags(s_rco32kEnv.ctimer) & CTIMER_IR_CR2INT_MASK)
                {
                    CTIMER_ClearStatusFlags(s_rco32kEnv.ctimer, CTIMER_IR_CR2INT_MASK);
                }
                CTIMER_Deinit(s_rco32kEnv.ctimer);

                /* pass apb cycle count during 16 32k cycles to RCO32K_CalcPpm() */
                delta = cap - s_rco32kEnv.firstVal;
                cnt = delta - ((s_rco32kEnv.cnt - RCO32K_17TH_EDGE) * delta / (s_rco32kEnv.cnt - 1U));
                RCO32K_CalcPpm(cnt);

                if (CTIMER0 == s_rco32kEnv.ctimer)
                {
                    PM_ClrReq(kPmReqCTIMER0);
                    NVIC_ClearPendingIRQ(CTIMER0_IRQn);
                }
                else if (CTIMER1 == s_rco32kEnv.ctimer)
                {
                    PM_ClrReq(kPmReqCTIMER1);
                    NVIC_ClearPendingIRQ(CTIMER1_IRQn);
                }
                else if (CTIMER2 == s_rco32kEnv.ctimer)
                {
                    PM_ClrReq(kPmReqCTIMER2);
                    NVIC_ClearPendingIRQ(CTIMER2_IRQn);
                }
                else if (CTIMER3 == s_rco32kEnv.ctimer)
                {
                    PM_ClrReq(kPmReqCTIMER3);
                    NVIC_ClearPendingIRQ(CTIMER3_IRQn);
                }
            }
        }

        s_rco32kEnv.lastVal = cap;
    }
}

/*!
 * @brief Configure ctimer to capture RCO32K waveform's rising edge.
 *
 * Ctimer uses APB as clock source, to capture 17 32k clock's rising edge.
 * The capture interrupt happens every 32k rise edge (~31.25us).
 *
 * @param none
 */
static void RCO32K_CfgCtimer(void *param)
{
    ctimer_config_t config;
    uint32_t code;

    if (s_rco32kEnv.hwCodeFlag != 0)
    {
        code = CALIB->RCO_RC_REF_OSC_CFG & CALIB_RCO_RC_REF_OSC_CFG_CAU_RCO_CAP_CFG_MASK;

        /* When temperature rises to aoround 100 Celsius degree, hardware calibration code becomes 0.
           If the temperature goes higher, the ppm value calculated by RCO32K_CalcPpm() will be larger than 0xFFFF,
           and exceeds the limit of RTC_CAL_PPM field in RTC's CAL register. */
        assert((code != 0U) && (code != 0xFU));

        if (s_rco32kEnv.hwCodeFlag == 1)
            code++;
        else
            code--;

        CALIB->RCO_RC_REF_OSC_CFG = (CALIB->RCO_RC_REF_OSC_CFG & ~CALIB_RCO_RC_REF_OSC_CFG_CAU_RCO_CAP_CFG_MASK) | code;
        s_rco32kEnv.hwCodeFlag = 0;
    }

    s_rco32kEnv.firstVal = 0U;
    s_rco32kEnv.cnt = 0U;

    config.mode = kCTIMER_TimerMode;
    config.input = kCTIMER_Capture_2;
    config.prescale = 0U;

    CTIMER_Init(s_rco32kEnv.ctimer, &config);
    CTIMER_StartTimer(s_rco32kEnv.ctimer);
    CTIMER_RegisterCallBack(s_rco32kEnv.ctimer, s_CtimerCbArray, kCTIMER_SingleCallback);
    CTIMER_SetupCapture(s_rco32kEnv.ctimer, kCTIMER_Capture_2, kCTIMER_Capture_RiseEdge, true);

    if (CTIMER0 == s_rco32kEnv.ctimer)
    {
        PM_SetReq(kPmSleep, kPmReqCTIMER0);
    }
    else if (CTIMER1 == s_rco32kEnv.ctimer)
    {
        PM_SetReq(kPmSleep, kPmReqCTIMER1);
    }
    else if (CTIMER2 == s_rco32kEnv.ctimer)
    {
        PM_SetReq(kPmSleep, kPmReqCTIMER2);
    }
    else if (CTIMER3 == s_rco32kEnv.ctimer)
    {
        PM_SetReq(kPmSleep, kPmReqCTIMER3);
    }
}

void RCO32K_InitSwCalib(uint32_t interval)
{
    RCO32K_CfgCtimer(NULL);
    s_rco32kEnv.interval = interval;
    if (s_rco32kEnv.interval)
    {
        s_rco32KTimerID = TM_AllocateTimer();
        TM_SetTimer(s_rco32KTimerID, kTM_IntervalTimer, TA_MILLISECOND(s_rco32kEnv.interval),
                    (tm_callback_t)RCO32K_CfgCtimer, NULL);
    }
}

void RCO32K_UpdateSwCalibInterval(uint32_t interval)
{
    s_rco32kEnv.interval = interval;
    if (s_rco32kEnv.interval)
    {
        TM_UpdateInterval(s_rco32KTimerID, TA_MILLISECOND(s_rco32kEnv.interval));
    }
    else
    {
        TM_ClearTimer(s_rco32KTimerID);
    }
}
#endif
