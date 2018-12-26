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
#ifndef _APP_RSCPS_H_
#define _APP_RSCPS_H_

/*!
 * @addtogroup APP_CSCPC_API
 * @{
 */

#if (BLE_RSC_SENSOR)

#include "rscps.h"
#include "rscps_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppRscpsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Running Speed and Cadence Profile Sensor.  
 * 
 */
void APP_RscpsAddProfileTask(void);

/*!
 * @brief Restore the bond data in Runing Speed and Cadence Sensor role of the Runing Speed and Cadence profile
 *
 * @param[in] conhdl               Connection handle
 * @param[in] rsc_meas_ntf_cfg     RSC Measurement Characteristic - Saved Client Characteristic Configuration Descriptor Value for a bonded device.
 * @param[in] sc_ctnl_pt_ntf_cfg   SC Control Point Characteristic - Saved Client Characteristic Configuration Descriptor Value for a bonded device.
 * @response RSCPS_ENABLE_RSP
 * @description
 * This API message shall be used after the connection with a peer device has been bonded in order to set the bonding data.
 */
void APP_RscpsEnableReq(uint16_t conhdl, uint16_t rsc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg);

/*!
 * @brief Request to send RSC measurement notification to peer decice.
 * @param[in] flags                Flags. Indicate which parameters are included in the value
 * @param[in] inst_cad             Instantaneous Cadence
 * @param[in] inst_speed           Instantaneous Speed
 * @param[in] inst_stride_len      Instantaneous Stride Length
 * @param[in] total_dist           Total Distance
 * @response  RSCPS_NTF_RSC_MEAS_RSP
 * @description
 * This API message shall be used by the application to send a RSC Measurement notification to every connected device.
 *
 */
void APP_RscpsNtfRscMeasReq(uint16_t conhdl,
                            uint8_t flags,
                            uint16_t inst_cad,
                            uint16_t inst_speed,
                            uint16_t inst_stride_len,
                            uint16_t total_dist);

#endif 

/*! @brief @} APP_RSCPS_API */
                                
#endif /* _APP_RSCPS_H_ */

