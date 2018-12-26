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

#ifndef _Progress_h_
#define _Progress_h_

#pragma once

#include "blfwk/host_types.h"

/*!
* @brief Contains the callback function for progress and abort phase.
*
*/
class Progress
{
public:
    //! @brief Default constructor.
    Progress()
        : m_segmentIndex(1)
        , m_segmentCount(1)
        , m_progressCallback(NULL)
        , m_abortPhase(NULL)
    {
    }

    //! @brief Constructor with initial callback.
    Progress(void(*callback)(int, int, int), bool *abortPhase)
        : m_segmentIndex(1)
        , m_segmentCount(1)
        , m_progressCallback(callback)
        , m_abortPhase(abortPhase)
    {
    }

    //! @brief Default destructor.
    ~Progress() {}
    //! @brief execute the progress callback function.
    //!
    //! @param percentage the percentage of current executed progress.
    void progressCallback(int percentage)
    {
        if (m_progressCallback != NULL)
        {
            m_progressCallback(percentage, m_segmentIndex, m_segmentCount);
        }
    }

    //! @brief Check whether the data phase is canceled.
    bool abortPhase(void)
    {
        if ((m_abortPhase) && (*m_abortPhase == true))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //! @biref initialized the progress callback function and the variable of controlling data phase.
    //!
    //! @param callback The progress callback function.
    //!
    //! @param abortPhase The pointer pointing to a variable controlling whether abort current data phase.
    void registerCallback(void(*callback)(int, int, int), bool *abortPhase)
    {
        m_progressCallback = callback;
        m_abortPhase = abortPhase;
    }

public:
    int m_segmentIndex; //!< Index of data segment in progressing.
    int m_segmentCount; //!< The number of data segments.

private:
    void(*m_progressCallback)(int percentage, int segmentIndex, int segmentCount); //!< The progress callback function.
    bool *m_abortPhase; //!< The pointer pointing to a variable controlling whether abort current data phase.
};

#endif // _Progress_h_
