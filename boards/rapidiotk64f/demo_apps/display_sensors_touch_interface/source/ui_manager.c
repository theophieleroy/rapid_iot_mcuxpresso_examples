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
 * @file ui_manager.c
 * This is the source file for the UI manager
 */

#include <stdio.h>

#include "board.h"
#include "ui_manager.h"
#include "fsl_os_abstraction.h"
#include "FreeRTOS.h"
#include "fsl_port.h"
#include "Panic.h"
#include "sx9500.h"
#include "port_interrupts.h"

#ifndef NDEBUG
#include "shell.h"
#endif

#include "GUI.h"

#include "spi_bus_share.h"

/*****************************************************************************
 * Variables
 ****************************************************************************/
osaTaskId_t  gUiManTaskId;
static osaEventId_t sUiManTaskEventId;
static bool_t sIsProcessingEvent = FALSE;

int sensorToDisplay			= 0;
int isDisplayOn				= 1;
int displayHasChanged		= 1;

extern osaSemaphoreId_t gOtaSem;

static const uint32_t kDebounceCount = 10U;
/*****************************************************************************
 * Private functions
 ****************************************************************************/
/*!
 * @brief Capture input from switch events
 *
 * @param[in, out] events Pointer to event flags from interrupt, outputs input after debounce
 *
 */
static void UiManagerCaptureInput(uint32_t *events)
{
    *events = 0U;

    /* Wait for 5 ms to help debounce */
    OSA_TimeDelay(5);

    volatile uint32_t count = kDebounceCount;

    /*
     * Verify we have captured all intended inputs by reading the value of each gpio.
     *
     * If the GPIO is asserted, we OR the event back into the event flags and clear any pending interrupt.
     *
     * If the GPIO is not longer asserted, at any time in the loop it is cleared from the event flags.
     *
     */
    while(count--)
    {
        if (0U == GPIO_PinRead(BOARD_INITPINS_USER_SW1_GPIO, BOARD_INITPINS_USER_SW1_GPIO_PIN))
        {
            *events |= kUiSwOne;
            GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW1_GPIO, 1U << BOARD_INITPINS_USER_SW1_GPIO_PIN);
        }
        else
        {
            *events &= ~kUiSwOne;
        }

        if (0U == GPIO_PinRead(BOARD_INITPINS_USER_SW2_GPIO, BOARD_INITPINS_USER_SW2_GPIO_PIN))
        {
            *events |= kUiSwTwo;
            GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW2_GPIO, 1U << BOARD_INITPINS_USER_SW2_GPIO_PIN);
        }
        else
        {
            *events &= ~kUiSwTwo;
        }

        if (0U == GPIO_PinRead(BOARD_INITPINS_USER_SW3_GPIO, BOARD_INITPINS_USER_SW3_GPIO_PIN))
        {
            *events |= kUiSwThree;
            GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW3_GPIO, 1U << BOARD_INITPINS_USER_SW3_GPIO_PIN);
        }
        else
        {
            *events &= ~kUiSwThree;
        }

        if (0U == GPIO_PinRead(BOARD_INITPINS_USER_SW4_GPIO, BOARD_INITPINS_USER_SW4_GPIO_PIN))
        {
            *events |= kUiSwFour;
            GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW4_GPIO, 1U << BOARD_INITPINS_USER_SW4_GPIO_PIN);
        }
        else
        {
            *events &= ~kUiSwFour;
        }
    }
}

/*!
 * @brief Handler for UI switch input events, displays event counts through GUI
 *
 * @param[in] events Event flags set from input capture
 *
 */
static void UiManagerSwitchHandler(uint32_t events)
{
    /* Process the captured switch events */

    /* Check for SW1 event */
    if (events & kUiSwOne)
    {
    	isDisplayOn = 1 - isDisplayOn;
    	displayHasChanged = 1;
    }

    /* Sleep for a while to debounce inputs */
    OSA_TimeDelay(60);
}

/*!
 * @brief UI Manager task function, handles touch and switch input events
 *
 * @param[in] argument Task parameter for RTOS. Not used, but necessary for RTOS compatibility
 *
 */
static void UiManagerTask(osaTaskParam_t argument)
{
    osaEventFlags_t  uiManTaskEventFlags;

    /* Let user know that UI is ready */
    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    GUI_SetColor(GUI_LIGHTBLUE);
    GUI_DispString("\n\n\n UI Manager Ready");

    while(1)
    {
        /* Clear any pending touch interrupt */
        uint8_t dummy;
        (void)SX9500_Read_Irq(&dummy);

        /* Allow callback to process interrupts */
        sIsProcessingEvent = FALSE;

        /* Wait for an event. The task will block here. */
        (void)OSA_EventWait(sUiManTaskEventId, osaEventFlagsAll_c, FALSE, osaWaitForever_c ,&uiManTaskEventFlags);

        if (uiManTaskEventFlags & kUiLock)
        {
            /* Disable interrupts */
            PORT_IRQ_UiDisableIrq();

            GUI_SetBkColor(GUI_WHITE);
            GUI_Clear();
            GUI_SetColor(GUI_BLUE);
            GUI_DispString("\n\n\n UI Locked for OTA ");

            /* Wait for signal from OTA process */
            OSA_SemaphoreWait(gOtaSem, osaWaitForever_c);

            GUI_SetBkColor(GUI_BLACK);
            GUI_Clear();
            GUI_SetColor(GUI_LIGHTBLUE);
            GUI_DispString("\n\n\n UI Unlocked ");

            /* Enable interrupts again */
            PORT_IRQ_UiEnableIrq();
        }
        else if (uiManTaskEventFlags & kUiTouchEv)
        {
            /* IRQ Event from Touch, read out source of event */
            RegStat_t dir;
            RegIrqSrc_t irq;
            if (SX9500_Read_Irq(&irq.octet) == SX9500_SUCCESS)
            {
                if (irq.bits.close || irq.bits.far)
                {
                    SX9500_Read_Proximity_Sensors(&dir.octet);

                    /* Clear out compensation value (not needed for this task) */
                    dir.bits.compstat = 0;

                    /*
                     * NOTE: Only clear if we have something new to display,
                     *       removing finger does not necessarily mean something new
                     */

                    /* Change sensor to display according to the detected touch pad */
                    switch(dir.octet)
                    {

                        case kUiTouchRt:
#ifndef NDEBUG
                            shell_write("Touch Right\n\r");
#endif

                            sensorToDisplay = 0;
                            displayHasChanged = 1;
                            break;

                        case kUiTouchLt:
#ifndef NDEBUG
                            shell_write("Touch Left\n\r");
#endif

                            sensorToDisplay = 1;
							displayHasChanged = 1;
                            break;

                        default:
                            /* NOTE: Touching multiple pads will result in no change to display */
                            break;
                    };
                }
            }
        }
        else if (uiManTaskEventFlags & kUiSwMask)
        {
            /* IRQ event from switches */

            /* Check for additional input */
            UiManagerCaptureInput(&uiManTaskEventFlags);

            /* Process event */
            UiManagerSwitchHandler(uiManTaskEventFlags);
        }
    }
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
OSA_TASK_DEFINE(UiManagerTask, UI_TASK_PRIO, 1, UI_TASK_STACK_SIZE, FALSE);
void UiManager_SetEvent(uint32_t event)
{
    /* If we are currently processing an event, ignore this interrupt (part of debounce solution) */
    if (!sIsProcessingEvent)
    {
        /* Set event and set flag to block additional interrupts */
        sIsProcessingEvent = TRUE;
        (void)OSA_EventSet(sUiManTaskEventId, event);
    }
}

void UiManager_Init(void)
{
    static uint8_t initialized = FALSE;

    /* Check if UI Manager is already initialized */
    if(FALSE == initialized)
    {
        initialized = TRUE;

        sUiManTaskEventId = OSA_EventCreate(TRUE);
        if(NULL == sUiManTaskEventId)
        {
            panic(0,0,0,0);
        }
        else
        {
            gUiManTaskId = OSA_TaskCreate(OSA_TASK(UiManagerTask), NULL);
            if(NULL == gUiManTaskId)
            {
                panic(0,0,0,0);
            }
        }
    }
}
