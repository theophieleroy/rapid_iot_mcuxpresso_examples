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
#ifndef _APP_BLPS_H_
#define _APP_BLPS_H_

/*!
 * @addtogroup APP_BLPS_API
 * @{
 */

#if BLE_BP_SENSOR
#include "blps_task.h"
#include "blps.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppBlpsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add blood pressure profile.
 */
void APP_BlpsAddProfileTask(void);

/*!
 * @brief Restore the bond data in Blood Pressure Sensor role of the Blood Pressure profile
 *
 * @param[in] conhdl             Connection handle
 * @param[in] bp_meas_ind_en     Value stored for Blood Pressure indication Client Configuration Char
 * @param[in] interm_cp_ntf_en   Value stored for intermediate cuff pressure notification Client Configuration
 * Char
 *
 * @response BLPS_ENABLE_RSP
 * @description
 * This function is used for restoring the bond data in Blood Pressure Sensor role of the Blood Pressure
 * profile.
 */
void APP_BlpsEnableReq(uint16_t conhdl, uint16_t bp_meas_ind_en, uint16_t interm_cp_ntf_en);

/*!
 * @brief Send a notification containing raw data - at connection.
 *
 * @param[in] conhdl                 Connection handle
 * @param[in] flag_interm_cp         Own code for differentiating between Blood Pressure Measurement and Intermediate
 * Cuff Pressure Measurement characteristics
 * @param[in] bps_bp_meas meas_val   Blood Pressure Measurement Structure
 *
 * @response BLPS_MEAS_SEND_RSP
 * @description
 * This function is used by the application (which handles the blood pressure device driver and measurements)to send a
 *blood
 * pressure measurement through the blood pressure sensor. The flag_interm_cp determines if measurement is an
 *intermediate
 * cuff pressure that will be notified or blood pressure full value that will be indicated.
 * Upon reception of this request, BLPS task will check if the necessary action (indication/notification) is possible
 *with
 * the current configuration set by the Collector in the BTS attributes:
 * If no, an error status is sent to the application.
 * If action is possible, blood pressure value is packed into a correct format in appropriate attribute value.
 * Notification/indication request is sent to GATT to generate the required PDU for the peer.
 * If it's an indication, the confirmation will come to the application directly from GATT.
 * If it's a notification, a confirmation does not come from peer, but through BLPS_MEAS_SEND_CFM right after sending
 * GATT message to notify the Intermediate Cuff Pressure Characteristic.
 */
void BLPS_MeasSendReq(uint16_t conhdl, uint16_t flag_interm_cp, struct bps_bp_meas meas_val);

#endif

/*! @brief @} APP_BLPS_API */

#endif /* _APP_BLPS_H_ */
