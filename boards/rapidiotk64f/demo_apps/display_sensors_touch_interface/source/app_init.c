/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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

/*!=================================================================================================
 \file       app_init.c
 \brief      This is a public source file for the initial system startup module. It contains
 the implementation of the interface functions.
 ==================================================================================================*/

/*==================================================================================================
 Include Files
 ==================================================================================================*/
#include <stdio.h>
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
//#include "fsl_mpu.h"
#include "fsl_sysmpu.h"
#include "fsl_port.h"
#include "pin_mux.h"

#if WDOG_ENABLE
    #include "fsl_wdog.h"
#endif

#ifndef FRDM_K64F_KW41Z
    #include "PhyInterface.h"
    #include "MacInterface.h"
#endif

/* FSL Framework */
#include "LED.h"
#include "rgb_led.h"
#include "nvm_adapter.h"
#include "TimersManager.h"
#include "Keyboard.h"
#include "SerialManager.h"
#include "Panic.h"

#include "fsl_os_abstraction.h"
#include "app_init.h"
#include "app_config.h"
#include "thread_network.h"
#include "thread_cfg.h"
#include "debug_log.h"
#include "sensors.h"

#if THREAD_USE_THCI
    #include "thci.h"
    #include "FsciInterface.h"
#endif

#if THREAD_USE_SHELL
    #include "shell_ip.h"
#endif

#ifdef FRDM_K64F_KW41Z
    #include "FsciInterface.h"
    #include "cmd_threadip.h"
    #include "shell_ip.h"

    #if gHybridApp_d
        #include "cmd_ble.h"
        #include "ble_shell.h"
    #endif

    #if SHELL_DEBUGGER
        #include "shell_peripherals.h"
    #endif

    #if LWIP_IPV6
        #include "lwip/opt.h"
        #include "lwip/tcpip.h"
        #include "lwip/ethip6.h"
        #include "lwip/dhcp.h"
        #include "lwip/mld6.h"
        #include "lwip/prot/dhcp.h"
        #include "netif/ethernet.h"
        #include "ethernetif.h"
    #endif
#endif

/* Display */
#include "BUTTON.h"
#include "CHECKBOX.h"
#include "DROPDOWN.h"
#include "GUI.h"
#include "MULTIPAGE.h"
#include "RADIO.h"
#include "SLIDER.h"
#include "emwin_support.h"

/* graphics */
#include "NXP_logo.h"
#include "test_img.h"
#include "img_bmp_callbacks.h"

/* user interface*/
#include "ui_manager.h"

/* Port interrupts */
#include "port_interrupts.h"

/* Flash */
#include "img_program_ext.h"
#include "spi_bus_share.h"

uint8_t gImgBuf[0x200];

//osaMutexId_t gExtMemMutex;

extern void APP_Init(void);
extern void APP_Handler(void);

#if gLpmIncluded_d
    extern void App_SedWakeUpFromKeyBoard(void);
#endif

#if THCI_USB_ENABLE && THR_SOFTWARE_RESET_ENABLE
    extern void THCI_ResetCpuEvent(resetCpuStatus_t resetStatus, uint32_t timeoutUs);
#endif

/*==================================================================================================
 Private macros
 ==================================================================================================*/
#define gFSCI_IpStackOpGCnf_c                   0xCFU
/*! FSCI utility Confirmations/Indications              */
#define gFSCI_CnfOpcodeGroup_c                  0xA4
/*! FSCI operation group for GATT Database (application) */
#define gFsciBleL2capOpcodeGroup_c              0x41
/*! FSCI operation group for GATT */
#define gFsciBleGattOpcodeGroup_c               0x44
/*! FSCI operation group for GATT Database (application) */
#define gFsciBleGattDbAppOpcodeGroup_c          0x45
/*! FSCI operation group for GAP */
#define gFsciBleGapOpcodeGroup_c                0x47

/* ATTENTION: the following static configuration is not used, as DHCP is used instead */
/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 0
#define configIP_ADDR3 142

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 0
#define configGW_ADDR3 1

// TODO: Sync on where this should be placed in the project
#define SW_VERSION                "2.00"

#define GyroscopeInterval (1.0)
#define AccelerometerInterval (0.3)
#define RPK_DEMO_TIMEOUT             300

/*==================================================================================================
 Private type definitions
 ==================================================================================================*/

/*==================================================================================================
 Private prototypes
 ==================================================================================================*/
static void fsciThciRegister(uint32_t fsciInterfaceId);
static void fsciBleRegister(uint32_t fsciInterfaceId);
static void THCI_RxCb(void *pData, void *param, uint32_t interfaceId);
static void BLE_FSCI_RxCb(void *pData, void *param, uint32_t interfaceId);

#if gLpmIncluded_d
    static void APP_HandleLowPowerOnIdle(void);
#endif
#if WDOG_ENABLE
    static void APP_WDOG_Init(void);
    static void APP_WDOG_Refresh(void);
#endif
/*==================================================================================================
 Private global variables declarations
 ==================================================================================================*/

void (*pfAppKeyboardHandler)(void *) = NULL;

#if WDOG_ENABLE
/* Configure watchdog. */
const wdog_config_t wdogConfig =
{
    .enableWdog = TRUE, /* Watchdog mode */
    .timeoutValue = 0x4096U, /* Watchdog overflow time is about 4s*/
    .enableWindowMode = FALSE, /* Disable window function */
    .windowValue = 0, /* Watchdog window value */
    .prescaler = kWDOG_ClockPrescalerDivide1, /* Watchdog clock prescaler */
    .enableUpdate = TRUE, /* Update register enabled */
    .clockSource = kWDOG_LpoClockSource, /* Watchdog clock source is LPO 1KHz */
#if defined(FSL_FEATURE_WDOG_HAS_WAITEN) && FSL_FEATURE_WDOG_HAS_WAITEN
    .workMode.enableWait = TRUE, /* Enable watchdog in wait mode */
#endif
    .workMode.enableStop = FALSE, /* Enable watchdog in stop mode */
    .workMode.enableDebug = FALSE, /* Disable watchdog in debug mode */
};

static WDOG_Type *wdog_base = WDOG;
#endif

/*!< reset MCU timestamp <microseconds> */
static uint64_t gSwResetTimestamp = 0;
/*!< boolean -  nvm format */
static bool_t gResetToFactory = FALSE;

#if 0
static struct netif fsl_netif0;
/* IPv6 multicast group FF03::3EAD */
static const ip_addr_t realmlocal_mcast_3ead =
    IPADDR6_INIT(PP_HTONL(0xFF030000UL),
                 PP_HTONL(0x00000000UL),
                 PP_HTONL(0x00000000UL),
                 PP_HTONL(0x00003EADUL)
                );
#endif

/*==================================================================================================
 Public global variables declarations
 ==================================================================================================*/

taskMsgQueue_t appThreadMsgQueue;

osaSemaphoreId_t gOtaSem;

static tmrTimerID_t sensorTimer = gTmrInvalidTimerID_c;

/* Starting arrow, used for generating other arrows*/
const GUI_POINT sourceArrow[3] = {{20, 10}, {0, 10}, {10, 0}};

/*==================================================================================================
 Public functions
 ==================================================================================================*/
/*==================================================================================================
 ==================================================================================================*/

void Init_Display()
{
    Display_Connect(); /* triggers GUI_Init() */
    Backlight_SetLevel(BLIGHT_LEVEL_LOW);

    GUI_SetBkColor(GUI_BLACK);
    GUI_SetColor(GUI_WHITE);
}

#define GUI_FONT_TITLE  GUI_Font8x18
#define GUI_FONT_NORMAL GUI_Font8x16

void DisplayArrows()
{
	/* If on the right screen */
	if(sensorToDisplay == 0)
	{
		/* Draw the left arrow */
		GUI_POINT leftArrowPoints[3];
		GUI_RotatePolygon(leftArrowPoints, sourceArrow, 3, 1.5707963268);
		GUI_FillPolygon(leftArrowPoints, 3, 5, 100);
	}

	/* If on the left screen */
	if(sensorToDisplay == 1)
	{
		/* Draw the right arrow */
		GUI_POINT rightArrowPoints[3];
		GUI_RotatePolygon(rightArrowPoints, sourceArrow, 3, -1.5707963268);
		GUI_FillPolygon(rightArrowPoints, 3, 170, 80);
	}
}

static void UpdateSensors(void *param)
{
	float interval;
	char buff[50]               = {0};

	float AccelerometerValue[3] = {0};
	uint8_t AccelerometerSize   = 3 * sizeof(float);

	int16_t GyroscopeValue[3]   = {0};
	uint8_t GyroscopeSize       = 6;

	/* Clear screen if screen to be displayed has changed */
	if(displayHasChanged == 1)
	{
		GUI_Clear();
		displayHasChanged = 0;
	}

	/* Accelerometer data screen */
	if(sensorToDisplay == 0)
	{
		if (get_acceleration((uint8_t *)AccelerometerValue, &AccelerometerSize) == 0)
		{
			sprintf(buff, " X: %+.3f\n\n Y: %+.3f\n\n Z: %+.3f", AccelerometerValue[0], AccelerometerValue[1], AccelerometerValue[2]);

			/* If the screen is On, display sensor data*/
			if(isDisplayOn == 1)
			{
				GUI_DispStringHCenterAt("Accelerometer", 88, 18);
				GUI_DispStringHCenterAt(buff, 88, 48);
				DisplayArrows();
			}
		}

		interval = AccelerometerInterval;
	}

	/* Gyroscope data screen */
	if(sensorToDisplay == 1)
	{
		if (get_rotation_speed((uint8_t *)GyroscopeValue, &GyroscopeSize) == 0)
		{
			sprintf(buff, " X: %+5d\n\n Y: %+5d\n\n Z: %+5d", GyroscopeValue[0], GyroscopeValue[1], GyroscopeValue[2]);

			/* If the screen is On, display sensor data*/
			if(isDisplayOn == 1)
			{
				GUI_DispStringHCenterAt("Gyroscope", 88, 18);
				GUI_DispStringHCenterAt(buff, 88, 48);
				DisplayArrows();
			}
		}

		interval = GyroscopeInterval;
	}

	if (sensorTimer == gTmrInvalidTimerID_c)
	{
		sensorTimer = TMR_AllocateTimer();
	}

	TMR_StartSingleShotTimer(sensorTimer, interval * 1000, UpdateSensors, NULL);
}

void main_task(uint32_t param)
{
    static uint8_t mainInitialized = FALSE;

    if (!mainInitialized)
    {
        mainInitialized = TRUE;

        /* Init memory blocks manager */
        MEM_Init();

        /* Init  timers module */
        TMR_Init();
        TMR_TimeStampInit();

        /* Init Led module */
        LED_Init();

        /* Init I2C. */
		I2C1_init();
		I2C2_init();

        RGB_Led_Set_State(3,1);
        App_WaitMsec(1000);
        RGB_Led_Set_State(3,2);
        App_WaitMsec(1000);
        RGB_Led_Set_State(0,3);

        SerialManager_Init();
    }

    /* Create semaphore to disable (lock) UI manager while running OTA */
    gOtaSem = OSA_SemaphoreCreate(0U);

    if (NULL == gOtaSem)
    {
        panic(0,0,0,0);
    }

    Init_Display();
    GUI_SetFont(&GUI_Font8x18);

    /* Initialize UI Manager task */
    UiManager_Init();

    /* Init Touchpad. */
	if (Init_touchpad())
	{
		GUI_DispStringHCenterAt("Error while initializing touchpad!", 88, 68);
		App_WaitMsec(500);
	}

	/* Init Accelerometer. */
	if (Init_accel_mag())
	{
		GUI_DispStringHCenterAt("Error while initializing Accel & Mag!", 88, 68);
		App_WaitMsec(500);
	}

	/* Init Gyroscope. */
	if (Init_rotation_speed())
	{
		GUI_DispStringHCenterAt("Error while initializing gyroscope!", 88, 68);
		App_WaitMsec(500);
	}

    if (sensorTimer == gTmrInvalidTimerID_c)
	{
		sensorTimer = TMR_AllocateTimer();
	}

	TMR_StartSingleShotTimer(sensorTimer, AccelerometerInterval * 1000, UpdateSensors, NULL);
}

bool_t SERIAL_TAP_IP6_SEND(struct pbuf *p, struct netif *inp)
{
    struct ip6_hdr *ip6hdr = (struct ip6_hdr *)p->payload;

    SerialTun_IPPacketSendRequest_t req =
    {
        .Size = SIZEOF_ETH_HDR + p->tot_len,
        .Data = (uint8_t *)p->payload - SIZEOF_ETH_HDR
    };

    // TCP traffic is not sent to the black-box
    if (IP6H_NEXTH(ip6hdr) != IP6_NEXTH_TCP)
    {
        SerialTun_IPPacketSendRequest(&req, 0);

        // UDP traffic is not processed by LwIP
        if (IP6H_NEXTH(ip6hdr) == IP6_NEXTH_UDP)
        {
            pbuf_free(p);
            return TRUE;
        }

    }

    // not eaten traffic to be processed by LwIP
    return FALSE;
}

/*!*************************************************************************************************
 \fn     APP_ResetMcuOnTimeout
 \brief  Reset the MCU on timeout
 \param  [in]    timeoutMs  timeout in milliseconds
 \param  [in]    resetToFactory
 \return         None
 ***************************************************************************************************/
void APP_ResetMcuOnTimeout(uint32_t timeoutMs, bool_t resetToFactory)
{
    gResetToFactory = resetToFactory;
    gSwResetTimestamp = TMR_GetTimestamp();
    gSwResetTimestamp += (timeoutMs * 1000); /* microseconds*/
}

/*!*************************************************************************************************
 \fn     APP_GetResetMcuTimeout
 \brief  Return the interval time until a MCU reset occurs
 \return  the time interval; 0 means that no Mcu reset was programmed
 ***************************************************************************************************/
uint32_t APP_GetResetMcuTimeout(void)
{
    uint32_t timeInterval = 0;

    if (gSwResetTimestamp > TMR_GetTimestamp())
    {
        timeInterval = (uint32_t)((gSwResetTimestamp - TMR_GetTimestamp())
                                  / 1000);
    }

    return timeInterval;
}

/*==================================================================================================
 Private functions
 ==================================================================================================*/
static void fsciThciRegister(uint32_t fsciInterfaceId)
{
    if (FSCI_RegisterOpGroup(gFSCI_IpStackOpGCnf_c,
                             gFsciMonitorMode_c,
                             THCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciThciRegister, 0, 0);
    }
}

static void fsciBleRegister(uint32_t fsciInterfaceId)
{
    /* Register Generic FSCI */
    if (FSCI_RegisterOpGroup(gFSCI_CnfOpcodeGroup_c,
                             gFsciMonitorMode_c,
                             BLE_FSCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciBleRegister, 0, 0);
    }

    /* Register L2CAP command handler */
    if (FSCI_RegisterOpGroup(gFsciBleL2capOpcodeGroup_c,
                             gFsciMonitorMode_c,
                             BLE_FSCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciBleRegister, 0, 0);
    }

    /* Register GATT command handler */
    if (FSCI_RegisterOpGroup(gFsciBleGattOpcodeGroup_c,
                             gFsciMonitorMode_c,
                             BLE_FSCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciBleRegister, 0, 0);
    }

    /* Register GATT Database (application) command handler */
    if (FSCI_RegisterOpGroup(gFsciBleGattDbAppOpcodeGroup_c,
                             gFsciMonitorMode_c,
                             BLE_FSCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciBleRegister, 0, 0);
    }

    /* Register GAP command handler */
    if (FSCI_RegisterOpGroup(gFsciBleGapOpcodeGroup_c,
                             gFsciMonitorMode_c,
                             BLE_FSCI_RxCb,
                             NULL,
                             fsciInterfaceId) != gFsciSuccess_c)
    {
        panic(0, (uint32_t)fsciBleRegister, 0, 0);
    }
}

static void THCI_RxCb
(
    void *pData,
    void *param,
    uint32_t interfaceId
)
{
    thrEvtContainer_t container; // this could be allocated instead
    KHC_ThreadIP_RX_MsgHandler(pData, &container, interfaceId);

//    SHELL_ThrEventNotify(&container);
}

static void BLE_FSCI_RxCb
(
    void *pData,
    void *param,
    uint32_t interfaceId
)
{
#if gHybridApp_d
    bleEvtContainer_t container; // this could be allocated instead
    KHC_BLE_RX_MsgHandler(pData, &container, interfaceId);
    SHELL_BleEventNotify(&container);
#endif
}

/*!*************************************************************************************************
 \fn     APP_HandleLowPowerOnIdle
 \brief  Handle low power on idle
 \param  [in]
 \return         None
 ***************************************************************************************************/
#if gLpmIncluded_d
static void APP_HandleLowPowerOnIdle(void)
{
    if (PWR_CheckIfDeviceCanGoToSleep())
    {
        PWRLib_WakeupReason_t wakeupReason;
        wakeupReason = PWR_EnterLowPower();

        if (wakeupReason.Bits.FromKeyBoard)
        {
            /* Protection to the LLWD pin enabled on both edges */
            static bool_t wakeUpFlag = FALSE;

            if (TRUE == wakeUpFlag)
            {
                wakeUpFlag = FALSE;
                App_SedWakeUpFromKeyBoard();
            }
            else
            {
                wakeUpFlag = TRUE;
            }

            PWR_AllowDeviceToSleep();
        }
    }
}
#endif

/*!*************************************************************************************************
 \fn     static void APP_WDOG_Init(void)
 \brief  Init watch dog if enabled
 ***************************************************************************************************/
#if WDOG_ENABLE
static void APP_WDOG_Init(void)
{

    uint32_t i = 0;

    WDOG_Init(wdog_base, &wdogConfig);

    /* Accessing register by bus clock */
    for (i = 0; i < 256; i++)
    {
        (void)WDOG->RSTCNT;
    }
}

/*!*************************************************************************************************
 \fn     static void APP_WDOG_Refresh(void)
 \brief  Refresh watch dog if enabled
 ***************************************************************************************************/

static void APP_WDOG_Refresh(void)
{
    uint32_t wdogTimer = (uint32_t)((((uint32_t)wdog_base->TMROUTH) << 16U) | (wdog_base->TMROUTL));

    /* Restart the watchdog so it doesn't reset */
    if (wdogTimer > (wdogConfig.timeoutValue >> 3U))
    {
        WDOG_Refresh(wdog_base);
    }
}
#endif
/*==================================================================================================
 Private debug functions
 ==================================================================================================*/
