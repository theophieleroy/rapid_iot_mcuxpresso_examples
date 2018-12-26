/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <stdarg.h>
#include "fsl_test_error.h"

#if defined(DETAILED_ERROR_MSG_ENABLE)
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum
{
    kErrorInfo_MaxCount = 10,  //!< Max count of Error Info
    kErrorInfo_MaxLength = 60, //!< Max Length of Error Info
};

//! @brief Error List
typedef struct _ERROR_TRACE_list
{
    uint32_t m_count;                                              //!< Valid count of error info.
    char m_content[kErrorInfo_MaxCount][kErrorInfo_MaxLength + 1]; //!< error info buffers.
} ERROR_TRACE_list_t;

ERROR_TRACE_list_t s_errorInfoList;

// See fsl_test_error.h for detailed information.
void fsl_test_error_init(void)
{
    s_errorInfoList.m_count = 0;
}

// See fsl_test_error.h for detailed information.
void fsl_test_error_add(const char *format, ...)
{
    if (s_errorInfoList.m_count < kErrorInfo_MaxCount)
    {
        char *errorBuffer = s_errorInfoList.m_content[s_errorInfoList.m_count];
        va_list args;
        va_start(args, format);
        vsnprintf(errorBuffer, kErrorInfo_MaxLength, format, args);
        va_end(args);
        s_errorInfoList.m_count++;
    }
}

// See fsl_test_error.h for detailed information.
void fsl_test_error_traverse(void)
{
    if (s_errorInfoList.m_count > 0)
    {
        printf(" -Detailed Information-\r\n");

        for (uint32_t i = 0; i < s_errorInfoList.m_count; i++)
        {
            printf("    %lu:%s\r\n", i + 1, s_errorInfoList.m_content[i]);
        }

        // Clear error list.
        fsl_test_error_init();
    }
}

#endif // DETAILED_ERROR_MSG_ENABLE

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
