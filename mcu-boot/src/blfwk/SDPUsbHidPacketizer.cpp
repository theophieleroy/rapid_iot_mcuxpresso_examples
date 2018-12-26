/*
* The Clear BSD License
* Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "blfwk/SDPUsbHidPacketizer.h"
#include "blfwk/Logging.h"

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

status_t SDPUsbHidPacketizer::writePacket(const uint8_t *packet, uint32_t byteCount, packet_type_t packetType)
{
    assert(m_peripheral);
    assert(packetType == kPacketType_Command || packetType == kPacketType_Data);
    assert(byteCount);
    assert(packet);

    uint32_t numBytes = 0;

    // Construct report contents.
    memset(&m_report, 0, sizeof(m_report));
    if (packetType == kPacketType_Command)
    {
        m_report[0] = kIdReport1;       // Report 1 is used to send a command to the device.
        numBytes = kReport1SizeBytes;
    }
    else
    {
        m_report[0] = kIdReport2;       // Report 2 is used to send data to the device.
        numBytes = kReport2SizeBytes;
    }

    memcpy(&m_report[1], packet, byteCount);

    return getPeripheral()->write((uint8_t *)&m_report, numBytes, m_packetTimeoutMs);
}

status_t SDPUsbHidPacketizer::readPacket(uint8_t **packet, uint32_t *packetLength, packet_type_t packetType)
{
    assert(m_peripheral);
    assert(packet);
    assert(packetLength);
    *packet = NULL;
    *packetLength = 0;
    // packetType is not used.

    // Read report.
    uint32_t actualBytes = 0;
    uint32_t retryCnt = 0;
    do
    {
        status_t retVal =
            m_peripheral->read((uint8_t *)&m_report, sizeof(m_report), &actualBytes, m_packetTimeoutMs);
        if (retVal != kStatus_Success)
        {
            return retVal;
        }

        if (actualBytes)
        {
            // Check the report ID.
            uint8_t reportId = m_report[0];
            if (reportId == kIdReport3)
            {
                if (actualBytes != kReport3SizeBytes)
                {
                    Log::error("usbhid: received unexpected number of bytes=%x\n", actualBytes);
                    return kStatus_Fail;
                }
            }
            else if (reportId == kIdReport4)
            {
                if (actualBytes != kReport4SizeBytes)
                {
                    Log::error("usbhid: received unexpected number of bytes=%x\n", actualBytes);
                    return kStatus_Fail;
                }
            }
            else
            {
                Log::error("usbhid: received unexpected report=%x\n", reportId);
                return kStatus_Fail;
            }
        }

    } while (!actualBytes && (++retryCnt < kPollPacketMaxRetryCnt));

    // Return results.
    *packet = &m_report[1];
    *packetLength = actualBytes - 1;

    return kStatus_Success;
}
