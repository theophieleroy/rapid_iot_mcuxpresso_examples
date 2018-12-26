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
#ifndef APP_CSCPS_H_
#define APP_CSCPS_H_

/*!
 * @addtogroup APP_CSCPS_API
 * @{
 */

#if BLE_CSC_SENSOR
#include "cscps_task.h"
#include "cscps.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppCscpsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Cycling Speed and Cadence profile.
 */
void APP_CscpsAddProfileTask(void);

/*!
 * @brief Restore the bond data in Cycling Speed and Cadence Sensor role of the Cycling Speed and Cadence profile
 *
 * @param[in] conhdl               Connection handle
 * @param[in] csc_meas_ntf_cfg     CSC Measurement Characteristic - Saved Client Characteristic Configuration Descriptor
 * Value for a bonded device
 * @param[in] sc_ctnl_pt_ntf_cfg   SC Control Point Characteristic - Saved Client Characteristic Configuration
 * Descriptor Value for a bonded device
 *
 *
 * @response CSCPS_ENABLE_RSP
 * @description
 * This API message shall be used after the connection with a peer device has been bonded in order to set the bonding
 * data.
 */
void APP_CscpsEnableReq(uint16_t conhdl, uint16_t csc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg);

/*!
 * @brief  This API message shall be used by the application to send a CSC Measurement notification to every
 * connected device.
 *
 * @param[in] flags                 Flags
 * @param[in] cumul_crank_rev       Cumulative Crank Revolution
 * @param[in] last_crank_evt_time   Last Crank Event Time
 * @param[in] last_wheel_evt_time   Last Wheel Event Time
 * @param[in] wheel_rev             Wheel Revolution since the last wheel event time.
 *
 * @Response CSCPS_NTF_CSC_MEAS_RSP
 * @description
 * This API message shall be used by the application to send a CSC Measurement notification to every
 * connected device. This profile checks whether the peer device has enable sending of notifications for the
 * characteristic and sends them according to its value.
 * The wheel_rev value is added to the total wheel revolution value stored in the environment. The total value is then
 * sent to the peer device.
 */
void APP_CscpsNtfCscMeasReq(uint16_t conhdl,
                            uint8_t flags,
                            uint16_t cumul_crank_rev,
                            uint16_t last_crank_evt_time,
                            uint16_t last_wheel_evt_time,
                            uint16_t wheel_rev);

#endif
/*! @brief @} APP_CSCPS_API */

#endif /* APP_CSCPS_H_ */
