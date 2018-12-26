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
#ifndef _APP_ANPS_H_
#define _APP_ANPS_H_

/*!
 * @addtogroup APP_ANPS_API
 * @{
 */

#if (BLE_AN_SERVER)

#include "anps.h"
#include "anps_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppAnpsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief This API message shall be used to add one instance of the Alert Notification Service.
 * 
 */
void APP_AnpsAddProfileTask(void);

/*!
 * @brief This API message shall be used after the connection with a peer device has been established in order to
 * restore the bond data used for that connection.
 *
 * @param[in] conhdl                        Connection handle
 * @param[in] new_alert_ntf_cfg             New  Alert  Characteristic  -  Saved  Client Characteristic Configuration Descriptor Value for a bonded device.
 * @param[in] unread_alert_status_ntf_cfg   Unread Alert Status Characteristic  -  Saved  Client Characteristic Configuration Descriptor Value for a bonded device.
 * @response ANPS_ENABLE_RSP
 * @description
 * This API message shall be used after the connection with a peer device has been established in order to
 * restore the bond data used for that connection.
 */
void APP_AnpsEnableReq(uint16_t conhdl, uint16_t rsc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg);

/*!
 * @brief This API message shall be used by the application to send a New Alert or an Unread Alert Status
 * notification to a specific device.
 * @param[in] conhdl             Connection handle
 * @param[in] str_len            Information String Length
 * @param[in] num                Number of alerts
 * @param[in] cat_id             Category ID
 * @param[in] str_info           Text String Information  
 * @response  ANPS_CMP_EVT
 * @description
 * This API message shall be used by the application to send a New Alert or an Unread Alert Status
 * notification to a specific device.
 * The implementation of the profile takes care of the provided value (category id,...), if one of these is not within the
 * ranges defined by the profile specification, an ANPS_CMP_EVT message with an PRF_ERR_INVALID_PARAM status is
 * sent to the application.
 * It also checks whether the provided category has been set as supported and if the peer device has enable sending of
 * notifications for this category and for the characteristic to update. If no notification can be sent, the received status
 * will be PRF_ERR_NTF_DISABLED.
 *
 */
void APP_AnpsNtfNewAlertCmd(uint16_t conhdl, uint8_t str_len, uint8_t num, uint8_t cat_id, uint8_t* str_info);

/*!
 * @brief This API message shall be used by the application to send a New Alert or an Unread Alert Status
 * notification to a specific device.
 * @param[in] conhdl             Connection handle
 * @param[in] num                Number of alerts
 * @param[in] cat_id             Category ID
 * @response  ANPS_CMP_EVT
 * @description
 * This API message shall be used by the application to send a New Alert or an Unread Alert Status
 * notification to a specific device.
 * The implementation of the profile takes care of the provided value (category id, ...), if one of these is not within the
 * ranges defined by the profile specification, an ANPS_CMP_EVT message with an PRF_ERR_INVALID_PARAM status is
 * sent to the application.
 * It also checks whether the provided category has been set as supported and if the peer device has enable sending of
 * notifications for this category and for the characteristic to update. If no notification can be sent, the received status
 * will be PRF_ERR_NTF_DISABLED.
 *
 */
void APP_AnpsNtfUnreadAlertCmd(uint16_t conhdl, uint8_t cat_id, uint8_t num);

#endif 

/*! @brief @} APP_ANPS_API */
                                
#endif /* _APP_ANPS_H_ */

