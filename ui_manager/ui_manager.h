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
 * @file ui_manager.h
 * This is the header file for the UI manager
 */

#ifndef _UI_MANAGER_H_
#define _UI_MANAGER_H_

/*!
 * @addtogroup ui_manager UI Manager
 *
 * The ui_manager manages user interactions with the Rapid IoT device.
 *
 * This includes event handling and display updates for the following:
 *      - Touch Pads
 *      - Push Buttons
 *      - UI Lock for BLE OTA Updates
 *
 * The ui_manager is intended to be used with the port_interrupts module. If not used with
 * port_interrupts module, the developer must create their own interrupt enablement and interrupt
 * handlers.
 *
 * Dependencies
 * -------------------------------------------------------------------------------------------------
 * The following modules are necessary for the ui_manager:
 *      - emWin GUI Library
 *      - port_interrupts for handling touch and push button interrupts
 *      - osaSemaphoreId_t gOtaSem (defined globaly)
 *      - If LOAD_EXT_IMG is set to 1:
 *          + img_program_ext module for saving/loading BMP data from External SPI Flash
 *          + img_bmp_callbacks module for loading images from External SPI Flash to display
 *          + up_arrow.h, down_arrow.h, right_arrow.h, and left_arrow.h
 *
 * Behavior
 * -------------------------------------------------------------------------------------------------
 * See local README.md
 *
 * Usage
 * -------------------------------------------------------------------------------------------------
 *
 * Initialization:
 * @code
 *
 *      // Define semaphore globaly
 *      osaSemaphoreId_t gOtaSem;
 *
 *      Display_Connect(); // Triggers GUI_Init()
 *      Backlight_SetLevel(BLIGHT_LEVEL_LOW);
 *
 *      // Create semaphore to disable (lock) UI manager while running OTA
 *      gOtaSem = OSA_SemaphoreCreate(0U);
 *
 *      if (NULL == gOtaSem)
 *      {
 *          panic(0,0,0,0);
 *      }
 *
 *      UiManager_Init();
 *
 *      PORT_IRQ_EnablePortAIrq();
 *      PORT_IRQ_EnablePortEIrq();
 *
 * @endcode
 *
 * Triggering UI Lock:
 * @code
 *
 *      // Trigger UI Lock
 *      UiManager_SetEvent(kUiLock);
 *
 *      // Perform critical task, i.e., BLE OTA
 *
 *      // Unlock UI with semaphore
 *      OSA_SemaphorePost(gOtaSem);
 *
 * @endcode
 *
 * Example Interrupt Handler:
 * @code
 *
 *      void PORTA_IRQHandler(void)
 *      {
 *          uint32_t pin_nb;
 *          pin_nb = PORT_GetPinsInterruptFlags(PORTA);
 *
 *          if (pin_nb & (1 << BOARD_INITPINS_USER_SW1_GPIO_PIN))
 *          {
 *              UiManager_SetEvent(kUiSwOne);
 *              GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW1_GPIO, 1U << BOARD_INITPINS_USER_SW1_GPIO_PIN);
 *          }
 *      }
 *
 * @endcode
 *
 * @{
 * @brief User Interface manager for demonstration purposes
 */

#define UI_TASK_STACK_SIZE      (3072U)
#define UI_TASK_PRIO            (3)

/*!
 * @brief UI Event Flags and Masks
 *
 * Event flags to be used in IRQ handlers to wake UI Manager task.
 *
 */
typedef enum _ui_events
{
    kUiTouchEv    = (1U << 0U),    /*!< Touch event flag */
    kUiSwOne      = (1U << 1U),    /*!< Switch one event flag */
    kUiSwTwo      = (1U << 2U),    /*!< Switch two event flag */
    kUiSwThree    = (1U << 3U),    /*!< Switch three event flag */
    kUiSwFour     = (1U << 4U),    /*!< Switch four event flag */
    kUiSwMask     = (0x1EU),       /*!< Switch event mask */
    kUiAllHmiMask = (0x1FU),       /*!< All UI HMI events mask */
    kUiLock       = (0x40U),       /*!< Lock UI to prevent user interaction */

}ui_events_t;

/*!
 * @brief UI Touch states
 *
 * States read from touch controller (SX9500), processed by UI Manager.
 *
 */
typedef enum _ui_touch
{
    kUiTouchDn = (1U << 4U),                    /*!< Single touch on down pad */
    kUiTouchRt = (1U << 5U),                    /*!< Single touch on right pad */
    kUiTouchUp = (1U << 6U),                    /*!< Single touch on up pad */
    kUiTouchLt = (1U << 7U),                    /*!< Single touch on left pad */
    kUiTouchDnRt = (1U << 4U) | (1U << 5U),     /*!< Dual touch on down and right pads */
    kUiTouchDnUp = (1U << 4U) | (1U << 6U),     /*!< Dual touch on down and up pads */
    kUiTouchDnLt = (1U << 4U) | (1U << 7U),     /*!< Dual touch on down and left pads */
    kUiTouchUpRt = (1U << 6U) | (1U << 5U),     /*!< Dual touch on up and right pads */
    kUiTouchUpLt = (1U << 6U) | (1U << 7U),     /*!< Dual touch on up and left pads */
    kUiTouchRtLt = (1U << 5U) | (1U << 7U),     /*!< Dual touch on right and left pads */

}ui_touch_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Function to set event, can be called from ISR or task. Will ignore new events until manager processes previous event.
 *
 * @param[in] event  Event to trigger in UI manager
 *
 * @return None
 *
 */
void UiManager_SetEvent(uint32_t event);

/*!
 * @brief Initialization function for creating UI Manager task and event group
 *
 * @return None
 *
 */
void UiManager_Init(void);

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif // _UI_MANAGER_H_
