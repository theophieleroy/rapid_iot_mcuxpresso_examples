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

#ifndef _EIF_MANAGER_H_
#define _EIF_MANAGER_H_

#include "fsl_common.h"
#include "app_ble.h"

#ifndef CFG_BLE_EIF
#define CFG_BLE_EIF kEIF_Uart
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define kEIF_Uart (0U)
#define kEIF_Usb_Vcom (1U)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Initialize BLE stack external interface.
 *
 * This function initializes the BLE stack external interface depending on user
 * definition in "system_config.h", Including:
 *  - USB
 *  - UART
 */
void EIF_Init(void);

/*!
 * @brief Start a data reception.
 *
 * This function receives data from the external interface, Including:
 *  - USB
 *  - UART
 *
 * @param bufptr Pointer to the receive buffer.
 * @param size Size of the expected reception.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when reception is finished.
 */
void EIF_Read(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

/*!
 * @brief Starts a data transmission.
 *
 * This function transmit data to the external interface, Including:
 *  - USB
 *  - UART
 *
 * @param bufptr Pointer to the transmit buffer.
 * @param size Size of the transmission.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when transmission is finished.
 */
void EIF_Write(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

/*! @brief Enable Interface flow.*/
void EIF_FlowOn(void);

/*! @brief Disable Interface flow.
 *
 * @retval true Flow has been disabled.
 * @retval false Failed to disable the flow.
 */
bool EIF_FlowOff(void);

/*! @brief BLE stack external interface receive callback. */
void EIF_RxCallback(void);

/*! @brief BLE stack external interface transmit callback.*/
void EIF_TxCallback(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _EIF_MANAGER_H_ */
