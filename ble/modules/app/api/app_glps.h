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

#ifndef APP_GLPS_H_
#define APP_GLPS_H_

/*!
 * @addtogroup APP_GLPS_API
 * @{
 */

#if BLE_GL_SENSOR
#include "glps.h"
#include "glps_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
struct app_glps_env_tag
{
    void* cur_record_offset;
    uint32_t addr_of_record_to_send; /* The meas idx found by the max seq_num */
    struct glp_racp_req racp_req;
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppGlpsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Create glucose service database.
 */
void APP_GlpsAddProfileTask(void);

/*!
 * @brief Start the glucose profile - at connection
 *
 * @param[in] conhdl   Connection handle
 * @param[in] evt_cfg  Glucose sensor event configuration (notification, indication) configured by
 * peer device during another connection (Bonded information)
 * - bit 0: Glucose measurement notifications enabled
 * - bit 1: Glucose measurement context notifications enabled
 * - bit 4: Record Access Control Point (RACP) indications enabled
 *
 * @response GLPS_ENABLE_RSP
 * @description
 * This function is used for enabling the Glucose Sensor role.
 * Before calling this function, a BLE connection shall exist with peer device.
 */
void APP_GlpsEnableReq(uint16_t conhdl, uint8_t evt_cfg);

void GRM_Init(void);

bool GRM_GlpsAddRecord(struct glp_meas meas_add, struct glp_meas_ctx meas_ctx_add);

#endif /* BLE_GL_SENSOR */

/*! @brief @} APP_GLPS_API */

#endif /* APP_GLPS_H_ */
