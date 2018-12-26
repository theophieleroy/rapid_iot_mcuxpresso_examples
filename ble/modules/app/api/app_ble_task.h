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

#ifndef _APP_BLE_TASK_H_
#define _APP_BLE_TASK_H_

/*!
 * @addtogroup APP_BLE_API
 * @{
 */

#include "rwip_config.h" /* SW configuration */

#if (BLE_APP_PRESENT)

#include "ke_msg.h"       /* Kernel Message */
#include "ke_task.h"      /* Kernel Task */
#include <stdint.h>       /* Standard Integer */
#include "clock_config.h" /* for CFG_32K_RCO */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Number of APP Task Instances */
#define APP_IDX_MAX (1)

/*! @brief States of APP task */
enum APP_STATE
{
    APP_STATE_INIT = 1,
    APP_STATE_ADV,
    APP_STATE_IDLE,

    APP_STATE_MAX /*!< Number of defined states. */
};

/*! @brief APP Task messages */
enum APP_MSG
{
    APP_MSG_DEFAULT = TASK_FIRST_MSG(TASK_ID_APP),
    APP_MSG_TIMER_CFG,
    APP_MSG_BUTTON_DOWN,

#if defined(CFG_PER_TEST)
    APP_MSG_PER_TEST_TIMER,
#endif

#if defined(CFG_DEMO_MENU)
    APP_MSG_MENU_UART_DATA_IND,
#endif

#if (BLE_BATT_SERVER)
    APP_MSG_BASS_BATT_LVL_CHK_TIMER, /*!< Check battery level */
#endif

    APP_MSG_MAX, /*!< Number of defined message. */
    APP_MSG_NUM = APP_MSG_MAX - APP_MSG_DEFAULT
};

/*! @brief Configure timer structure */
struct app_timer_cfg_cmd
{
    ke_msg_id_t timer_id; /*!< Timer message ID */
    uint32_t duration;    /*!< Duration == 0 means clear */
};

/*! @brief Button press down command structure */
struct app_button_down_cmd
{
    uint32_t pin; /*!< Buttom GPIO pin */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Default State handlers definition. */
extern struct ke_msg_handler g_AppDefaultState[APP_MSG_NUM];
/*! @brief Table of message handlers */
extern const struct ke_state_handler app_default_handler;
/*! @brief Defines the place holder for the states of all the task instances. */
extern ke_state_t app_state[APP_IDX_MAX];

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Handles reception of all messages sent from the lower layers to the application.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_MsgHandler(ke_msg_id_t const msgid, void *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

/*!
 * @brief To configure the timer.
 * @param[in] msgid Id of the message received.
 * @param[in] param The timer details to be configured.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return Returns KE_MSG_CONSUMED.
 */
int APP_TimerCfgHandler(ke_msg_id_t const msgid,
                        struct app_timer_cfg_cmd const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id);

/*!
 * @brief A generic periodical software timer for per test.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_PerTestTimerHandler(ke_msg_id_t const msgid,
                            void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

/*!
 * @brief Handles reception of the button down message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_ButtonDownHandler(ke_msg_id_t const msgid,
                          struct app_button_down_cmd const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

#endif /*(BLE_APP_PRESENT) */

/*! @brief @} APP_BLE_API */

#endif /* _APP_BLE_TASK_H_ */
