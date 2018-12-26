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

#ifndef _APP_MENU_H_
#define _APP_MENU_H_

#include "ke_msg.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief menu id */
typedef enum
{
    menu_main,
    menu_gap,
    menu_gap_create_connection,
    menu_gap_disconnection,
    menu_gap_pair,
    menu_gap_encrypt,
    menu_gap_security,
    menu_gap_unbond,
    menu_gatt,
    menu_gatt_exc_mtu,
    menu_smp_tk_input,
    menu_smp_nc,
    menu_htpc,
    menu_htpc_enable,
    menu_blpc,
    menu_blpc_enable,
    menu_hrpc,
    menu_hrpc_enable,
    menu_glpc,
    menu_glpc_enable,
    menu_findl,
    menu_findl_enable,
    menu_proxm,
    menu_proxm_enable,
    menu_tipc,
    menu_tipc_enable,
    menu_hogph,
    menu_hogpbh_enable,
    menu_hogprh_enable,
    menu_scppc,
    menu_scppc_enable,
    menu_disc,
    menu_disc_enable,
    menu_basc,
    menu_basc_enable,
    menu_cscpc,
    menu_cscpc_enable,
    menu_rscpc,
    menu_rscpc_enable,
    menu_anpc,
    menu_anpc_enable,
    menu_anpc_alert,
    menu_paspc,
    menu_paspc_enable,
    menu_l2cc,
    menu_l2cc_enable,
    menu_l2cc_select,
    menu_l2cc_chnl,
    menu_qppc,
    menu_qppc_enable,
    menu_cppc,
    menu_cppc_enable,
    menu_lanc,
    menu_lanc_enable,
    menu_otac,
    menu_otac_enable,

    menu_max
} MENU_ID;

#define QN_UART_RX_LEN 0x10

/*! @brief App menu handle message */
struct app_menu_uart_data_ind
{
    uint16_t len;
    uint8_t data[1];
};

/*! @brief QN demo environment context structure */
struct app_menu_uart_env_tag
{
    uint16_t len;
    uint8_t buf_rx[QN_UART_RX_LEN];
};

/*! @brief message of data request send to application */
struct app_menu_uart_data_req
{
    uint16_t len;
    uint8_t data[1];
};

/*!
 * @brief Initialize USRT to receive command line.
 */
void APP_MenuUartInit(void);

/*!
 * @brief Show start menu.
 */
void APP_MenuShowStart(void);

/*!
 * @brief Show TK input menu.
 */
void APP_MenuShowTkInput(void);

/*!
 * @brief Show numeric compare menu.
 */
void APP_MenuShowNumericCompare(void);

/*!
 * @brief Handles reception of the command line.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_MenuUartDataIndHandler(ke_msg_id_t const msgid,
                               const void *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#endif /* _APP_MENU_H_ */
