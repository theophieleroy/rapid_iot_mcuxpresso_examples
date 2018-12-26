/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

#ifndef __SRTM_SERVICE_STRUCT_H__
#define __SRTM_SERVICE_STRUCT_H__

#include "srtm_defs.h"
#include "srtm_list.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM service struct
*/
struct _srtm_service
{
    srtm_list_t node;  /*!< SRTM service list node to link to a list */
    srtm_dispatcher_t dispatcher;
    uint8_t category;

    void (*destroy)(srtm_service_t service);
    srtm_status_t (*request)(srtm_service_t service, srtm_request_t request);
    srtm_status_t (*notify)(srtm_service_t service, srtm_notification_t notification);
};

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Compare message and service version to see if the service can serve.
 *
 * @param service SRTM service handle.
 * @param msg The message to check.
 * @param svcVer The service version to check.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Service_CheckVersion(srtm_service_t service, srtm_message_t msg,
                                        uint16_t svcVer);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_SERVICE_STRUCT_H__ */
