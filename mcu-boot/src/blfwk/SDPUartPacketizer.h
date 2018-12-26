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

#ifndef _sdp_uart_packetizer_h_
#define _sdp_uart_packetizer_h_

#include "Packetizer.h"
#include "UartPeripheral.h"
#include "serial.h"

#pragma once

//! @addtogroup sdp_uart_packetizer
//! @{

namespace blfwk
{

/*!
* @brief Provides source and sink for SDP packets that go over UART.
*/
class SDPUartPacketizer : public Packetizer
{
public:
    //! @brief Constants.
    enum _constants
    {
        kMaxReadSizeBytes = 64,         //!< Max bytes returned by single read.
        kNumBytesCommandResponse = 4,   //!< Number of bytes in a kPacketTypeCommand read.
    };

public:
    //! @brief Default Constructor.
    SDPUartPacketizer(UartPeripheral *peripheral, uint32_t readPacketTimeoutMs)
        : Packetizer(peripheral, readPacketTimeoutMs)
        , m_readBuf()
    {
    }

    //! @brief Destructor.
    virtual ~SDPUartPacketizer() {}

    //! @brief Read a packet.
    //!
    //! Provides the address of a buffer containing the packet.
    //!
    //! @param packet Pointer location to write packet pointer
    //! @param packetLength Number of bytes in returned packet
    virtual status_t readPacket(uint8_t **packet, uint32_t *packetLength, packet_type_t packetType)
    {
        uint32_t count = ((packetType == kPacketType_Command) || (m_readCount == 0)) ? kNumBytesCommandResponse : m_readCount;
        if (count > sizeof(m_readBuf))
        {
            count = sizeof(m_readBuf);
        }
        status_t status = getPeripheral()->read(m_readBuf, count, packetLength, m_packetTimeoutMs);
        *packet = m_readBuf;
        return status;
    }

    //! @brief Write a packet.
    //!
    //! @param packet Pointer to packet to write
    //! @param byteCount Number of bytes in packet
    virtual status_t writePacket(const uint8_t *packet, uint32_t byteCount, packet_type_t packetType)
    {
        return getPeripheral()->write(packet, byteCount);
    }

    //! @brief Finalize.
    virtual void finalize() {};

    //! @brief Peripheral accessor.
    virtual UartPeripheral *getPeripheral() { return (UartPeripheral *)m_peripheral; }

    //! @brief Abort data phase.
    virtual void abortPacket() {};

    //! @brief Send framing packet ack.
    virtual void sync() {};

    //! @brief Enable simulator command processor pump.
    virtual void enableSimulatorPump() {};

    //! @brief Pump simulator command processor.
    virtual void pumpSimulator() {};

    //! @brief Set aborted flag.
    //!
    //! Used for out-of-band flow control for simulator.
    virtual void setAborted(bool aborted) {};

    //! @brief Return the max packet size.
    virtual uint32_t getMaxPacketSize() { return 0; }

protected:
    uint8_t m_readBuf[kMaxReadSizeBytes];   //!< Buffer for read data.
};

} // namespace blfwk

//! @}

#endif // _sdp_uart_packetizer_h_
