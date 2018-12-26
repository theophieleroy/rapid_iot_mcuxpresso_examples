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
#ifndef _APP_OTAC_H_
#define _APP_OTAC_H_

/*!
 * @addtogroup APP_OTAS_API QBlue Private Profile Client
 * @{
 */

#if BLE_OTA_SERVER
#include "otas_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Otas Service server environment variable */
struct app_otas_env_tag
{
    uint16_t conhdl; /*!<  Connection handle */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppOtasTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * Add a QBlue Private Profile Client instance.
 */
void APP_OtasAddProfileTask(void);


/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  OTAS_ENABLE_RSP
 * @description
 *  This API is used for enabling the Client role of the OTA. Profile will discovery all
 *  the attributes related to OTAS. 
 */
void APP_OtasEnableReq(uint16_t conhdl);

/*!
 * @brief Send data to server.
 * @param[in] conhdl        Connection handle 
 * @param[in] len           Length of data to be sent
 * @param[in] val           Pointer to data to be sent
 * @response  OTAS_SEND_DATA_RSP
 * @description
 * This function is used by the application to send a raw data to server.
 */
void app_otas_host_send_cmd(uint16_t conhdl, uint16_t len, uint8_t *val);

void iapServerSend(uint16_t len, uint8_t *data);


/*!
 * @brief Handles the enable confirmation from the OTAS.
 *
 * @param[in] msgid     OTAS_ENABLE_RSP
 * @param[in] param     Pointer to struct otas_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAS
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client to either send the discovery results of OTAS and 
 * confirm enabling of the Client role, or to simply confirm enabling of Client role.
 */
int APP_OtasEnableRspHandler(ke_msg_id_t const msgid,
                             struct otas_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the generic message for read responses for APP.
 *
 * @param[in] msgid     OTAS_RD_CHAR_RSP
 * @param[in] param     Pointer to struct otas_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAS
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received read response. This message in OTAS contains
 * OTAS' version.
 */
int APP_OtasRdCharRspHandler(ke_msg_id_t const msgid,
                             struct otas_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the generic message for write characteristic response status to APP.
 *
 * @param[in] msgid     OTAS_SEND_DATA_RSP
 * @param[in] param     Pointer to struct otas_send_data_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAS
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received write response.  
 */
int APP_OtasSendDataRspHandler(ke_msg_id_t const msgid,
                               struct otas_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Handles the value send to APP.
 *
 * @param[in] msgid     OTAS_RECV_DATA_IND
 * @param[in] param     Pointer to struct otas_recv_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAS
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received value by 
 * notification. The application will do what it needs to do with the received value. 
 */
int APP_OtasRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct otas_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
uint16_t otapServerGetMtu(uint16_t conidx);

#endif /* BLE_OTA_SERVER */
/*! @brief @} APP_OTAS_API */

#endif /* APP_QPPC_H_ */
