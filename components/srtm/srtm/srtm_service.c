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

#include <assert.h>

#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_message.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void SRTM_Service_Destroy(srtm_service_t service)
{
    assert(service);
    assert(service->destroy);
    service->destroy(service);
}
   
srtm_status_t SRTM_Service_Request(srtm_service_t service, srtm_request_t request)
{
    assert(service);
    assert(service->request);

    return service->request(service, request);
}

srtm_status_t SRTM_Service_Notify(srtm_service_t service, srtm_notification_t notification)
{
    assert(service);
    assert(service->notify);

    return service->notify(service, notification);
}

srtm_status_t SRTM_Service_CheckVersion(srtm_service_t service, srtm_message_t msg,
                                        uint16_t svcVer)
{
    uint16_t msgVer = SRTM_CommMessage_GetVersion(msg);

    if (SRTM_MESSAGE_MAJOR_VERSION(msgVer) != SRTM_MESSAGE_MAJOR_VERSION(svcVer))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                           "SRTM_WARN: SRTM_Service_CheckVersion mismatch req %d, service %d!\r\n", msgVer, svcVer);
        return SRTM_Status_ServiceVerMismatch;
    }

    return SRTM_Status_Success;
}
