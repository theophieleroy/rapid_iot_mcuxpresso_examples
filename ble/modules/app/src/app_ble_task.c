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
#include "clock_config.h"
#include "app_ble.h"
#include "fsl_gpio.h"
#include "rco32k_calibration.h"
#include "fsl_power.h"

#if (BLE_APP_PRESENT)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
    ke_task_id_t task_id;
    const struct ke_state_handler *handler;
} app_table_handler_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct ke_msg_handler g_AppDefaultState[APP_MSG_NUM] = {
    {KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t)APP_MsgHandler}, {APP_MSG_TIMER_CFG, (ke_msg_func_t)APP_TimerCfgHandler},
};

const struct ke_state_handler app_default_handler = KE_STATE_HANDLER(g_AppDefaultState);

ke_state_t app_state[APP_IDX_MAX];

const app_table_handler_t s_appTableHandlers[] = {
    {TASK_ID_GAPM, &g_AppGapTableHandler},

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    {TASK_ID_GAPC, &g_AppGapTableHandler},      {TASK_ID_GATTC, &g_AppGattTableHandler},
#endif

/* Server */
#if (BLE_DIS_SERVER)
    {TASK_ID_DISS, &g_AppDissTableHandler},
#endif /*(BLE_DIS_SERVER) */

#if (BLE_BATT_SERVER)
    {TASK_ID_BASS, &g_AppBassTableHandler},
#endif /*(BLE_BATT_SERVER) */

#if (BLE_QPP_SERVER)
    {TASK_ID_QPPS, &g_AppQppsTableHandler},
#endif /*(BLE_QPP_SERVER) */

#if (BLE_YELL_SERVER)
    {TASK_ID_YELLS, &g_AppYellsTableHandler},
#endif /*(BLE_YELL_SERVER) */

#if (BLE_GL_SENSOR)
    {TASK_ID_GLPS, &g_AppGlpsTableHandler},
#endif /*(BLE_GL_SENSOR) */

#if (BLE_FINDME_TARGET)
    {TASK_ID_FINDT, &g_AppFindtTableHandler},
#endif /*(BLE_FINDME_TARGET) */

#if (BLE_PROX_REPORTER)
    {TASK_ID_PROXR, &g_AppProxrTableHandler},
#endif /*(BLE_PROX_REPORTER) */

#if (BLE_BP_SENSOR)
    {TASK_ID_BLPS, &g_AppBlpsTableHandler},
#endif /*(BLE_BLPS_SENSOR) */

#if (BLE_HT_THERMOM)
    {TASK_ID_HTPT, &g_AppHtptTableHandler},
#endif /*(BLE_HT_THERMOM) */

#if (BLE_CSC_SENSOR)
    {TASK_ID_CSCPS, &g_AppCscpsTableHandler},
#endif /*(BLE_CSCPS_SENSOR) */

#if (BLE_CP_SENSOR)
    {TASK_ID_CPPS, &g_AppCppsTableHandler},
#endif /*(BLE_CP_SENSOR) */

#if (BLE_HR_SENSOR)
    {TASK_ID_HRPS, &g_AppHrpsTableHandler},
#endif /*(BLE_HR_SENSOR) */

#if (BLE_LN_SENSOR)
    {TASK_ID_LANS, &g_AppLansTableHandler},
#endif /*(BLE_LN_SENSOR) */

#if (BLE_AN_SERVER)
    {TASK_ID_ANPS, &g_AppAnpsTableHandler},
#endif /*(BLE_AN_CLIENT) */
#if (BLE_SP_SERVER)
    {TASK_ID_SCPPS, &g_AppScppsTableHandler},
#endif /*(BLE_SP_SERVER) */

#if (BLE_HID_DEVICE)
    {TASK_ID_HOGPD, &g_AppHogpdTableHandler},
#endif /*(BLE_HID_DEVICE)*/

#if (BLE_TIP_SERVER)
    {TASK_ID_TIPS, &g_AppTipsTableHandler},
#endif /*(BLE_TIP_SERVER)*/

/* Client */
#if (BLE_DIS_CLIENT)
    {TASK_ID_DISC, &g_AppDiscTableHandler},
#endif /*(BLE_DIS_CLIENT) */

#if (BLE_BATT_CLIENT)
    {TASK_ID_BASC, &g_AppBascTableHandler},
#endif /*(BLE_BATT_CLIENT) */

#if (BLE_FINDME_LOCATOR)
    {TASK_ID_FINDL, &g_AppFindlTableHandler},
#endif /*(BLE_FINDME_LOCATOR) */

#if (BLE_QPP_CLIENT)
    {TASK_ID_QPPC, &g_AppQppcTableHandler},
#endif /*(BLE_QPP_CLIENT) */

#if (BLE_PROX_MONITOR)
    {TASK_ID_PROXM, &g_AppProxmTableHandler},
#endif /*(BLE_PROX_MONITOR) */

#if (BLE_GL_COLLECTOR)
    {TASK_ID_GLPC, &g_AppGlpcTableHandler},
#endif /*(BLE_GL_COLLECTOR) */

#if (BLE_HT_COLLECTOR)
    {TASK_ID_HTPC, &g_AppHtpcTableHandler},
#endif /*(BLE_HT_COLLECTOR) */

#if (BLE_CP_COLLECTOR)
    {TASK_ID_CPPC, &g_AppCppcTableHandler},
#endif /*(BLE_CSC_COLLECTOR) */
#if (BLE_CSC_COLLECTOR)
    {TASK_ID_CSCPC, &g_AppCscpcTableHandler},
#endif /*(BLE_CP_COLLECTOR) */

#if (BLE_LN_COLLECTOR)
    {TASK_ID_LANC, &g_AppLancTableHandler},
#endif /*(BLE_LN_COLLECTOR)*/

#if (BLE_SP_CLIENT)
    {TASK_ID_SCPPC, &g_AppScppcTableHandler},
#endif /*(BLE_SP_CLIENT)*/

#if (BLE_RSC_SENSOR)
    {TASK_ID_RSCPS, &g_AppRscpsTableHandler},
#endif /*(BLE_RSCPS_SENSOR) */

#if (BLE_PAS_SERVER)
    {TASK_ID_PASPS, &g_AppPaspsTableHandler},
#endif /*(BLE_PAS_SERVER) */

#if (BLE_PAS_CLIENT)
    {TASK_ID_PASPC, &g_AppPaspcTableHandler},
#endif /*(BLE_PAS_CLIENT) */

#if (BLE_HID_BOOT_HOST)
    {TASK_ID_HOGPBH, &g_AppHogpbhTableHandler},
#endif /*(BLE_HID_BOOT_HOST)*/

#if (BLE_HID_REPORT_HOST)
    {TASK_ID_HOGPRH, &g_AppHogprhTableHandler},
#endif /*(BLE_HID_REPORT_HOST)*/

#if (BLE_AN_CLIENT)
    {TASK_ID_ANPC, &g_AppAnpcTableHandler},
#endif /*(BLE_AN_CLIENT) */

#if (BLE_TIP_CLIENT)
    {TASK_ID_TIPC, &g_AppTipcTableHandler},
#endif /*(BLE_TIP_CLIENT)*/

#if (BLE_HR_COLLECTOR)
    {TASK_ID_HRPC, &g_AppHrpcTableHandler},
#endif /*(BLE_HR_COLLECTOR)*/

#if (BLE_BP_COLLECTOR)
    {TASK_ID_BLPC, &g_AppBlpcTableHandler},
#endif /*(BLE_BP_COLLECTOR)*/

#if (BLE_RSC_COLLECTOR)
    {TASK_ID_RSCPC, &g_AppRscpcTableHandler},
#endif /*(BLE_RSC_COLLECTOR) */

#if (BLE_OTA_CLIENT)
    {TASK_ID_OTAC, &g_AppOtacTableHandler},
#endif /*(BLE_OTA_CLIENT)*/

#if (BLE_OTA_SERVER)
    {TASK_ID_OTAS, &g_AppOtasTableHandler},
#endif /*(BLE_OTA_SERVER)*/
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Handles the specified message.
 * @param[in] handler_list   Pointer to handler function list
 * @param[in] msgid          Id of the message received.
 * @param[in] param          Pointer to the parameters of the message.
 * @param[in] src_id         ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
static uint8_t APP_GetHandler(const struct ke_state_handler *handler_list,
                              ke_msg_id_t msgid,
                              void *param,
                              ke_task_id_t src_id)
{
    /* Counter */
    uint8_t counter;

    /* Get the message handler function by parsing the message table */
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler *handler = (struct ke_msg_handler *)(handler_list->msg_table + counter - 1);

        if ((handler->id == msgid) || (handler->id == KE_MSG_DEFAULT_HANDLER))
        {
            /* If handler is NULL, message should not have been received in this state */
            ASSERT_ERR(handler->func);

            return (uint8_t)(handler->func(msgid, param, TASK_APP, src_id));
        }
    }

    /* If we are here no handler has been found, drop the message */
    return (KE_MSG_CONSUMED);
}

int APP_MsgHandler(ke_msg_id_t const msgid, void *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    /* Message policy */
    uint8_t msg_pol = KE_MSG_CONSUMED;

    /* Retrieve identifier of the task from received message */
    ke_task_id_t src_task_id = MSG_T(msgid);

    /* Find and execute message handler */
    for (uint32_t idx = 0; idx < sizeof(s_appTableHandlers) / sizeof(app_table_handler_t); idx++)
    {
        if (s_appTableHandlers[idx].task_id == src_task_id)
        {
            msg_pol = APP_GetHandler(s_appTableHandlers[idx].handler, msgid, param, src_id);
            break;
        }
    }

    return (msg_pol);
}

int APP_TimerCfgHandler(ke_msg_id_t const msgid,
                        struct app_timer_cfg_cmd const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id)
{
    if (param->duration)
    {
        ke_timer_set(param->timer_id, TASK_APP, param->duration);
    }
    else
    {
        ke_timer_clear(param->timer_id, TASK_APP);
    }

    return (KE_MSG_CONSUMED);
}

#if defined(CFG_PER_TEST)
int APP_PerTestTimerHandler(ke_msg_id_t const msgid,
                            void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    /*! @brief activate the 5s timer again */
    APP_TIMER_SET(APP_MSG_PER_TEST_TIMER, 500U, APP_PerTestTimerHandler);

    static uint8_t cnt = 0U;

    cnt++;

    if (cnt % 2U == 0U)
    { /* 5s * 2 = 10s */
        APP_ShowStatistic();
    }

    return (KE_MSG_CONSUMED);
}
#endif

int APP_ButtonDownHandler(ke_msg_id_t const msgid,
                          struct app_button_down_cmd const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    APP_ButtonDownCallback(param->pin);
    return (KE_MSG_CONSUMED);
}

#endif /*(BLE_APP_PRESENT) */
