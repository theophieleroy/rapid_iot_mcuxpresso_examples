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
#ifndef _APP_PASPS_H_
#define _APP_PASPS_H_

/*!
 * @addtogroup APP_PASPS_API
 * @{
 */

#if (BLE_PAS_SERVER)

#include "pasps.h"
#include "pasps_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppPaspsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Phone Alert Status Profile Server.
 * 
 */
void APP_PaspsAddProfileTask(void);

/*!
 * @brief Restore the bond data in Phone Alert Status Server role of the Phone Alert Status profile.
 *
 * @param[in] conhdl                   Connection handle
 * @param[in] alert_status_ntf_cfg     Stored Alert Status notification configuration for a bonded device
 * @param[in] ringer_setting_ntf_cfg   Stored Ringer Setting notification configuration for bonded device
 * @response PASPS_ENABLE_RSP
 * @description
 * This API message shall be used after the connection with a peer device has been established in order to
 * set the PASP Server bond data for the connection.
 */
void APP_PaspsEnableReq(uint16_t conhdl, uint16_t rsc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg);

/*!
 * @brief This API message shall be used by the application to update the value of the Alert Status characteristic or
 * the Ringer Setting characteristic stored in the database.
 * @param[in] conhdl             Connection handle
 * @param[in] operation          Operation code, indicate which characteristic value need to be updated
 * @param[in] value              Instantaneous Speed
 * @response  PASPS_CMP_EVT
 * @description
 * This API message shall be used by the application to update the value of the Alert Status characteristic or
 * the Ringer Setting characteristic stored in the database. The interpretation of the value depends of the operation
 * value which is used to detect which one of these two characteristic shall be modified.
 *
 */
void APP_PaspsUpdateCharValCmd(uint16_t conhdl, uint8_t operation, uint8_t value);

#endif 

/*! @brief @} APP_PASPS_API */
                                
#endif /* _APP_PASPS_H_ */

