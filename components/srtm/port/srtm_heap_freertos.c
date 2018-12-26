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

#include <FreeRTOS.h>

#include "srtm_heap.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SRTM_HEAP_STAT_COUNT_MASK (0x3F)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef SRTM_DEBUG_MESSAGE_FUNC
static uint32_t count;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void *SRTM_Heap_Malloc(uint32_t size)
{
#ifdef SRTM_DEBUG_MESSAGE_FUNC
    size_t freeSize, minFreeSize;

    if (((++count) & SRTM_HEAP_STAT_COUNT_MASK) == 0)
    {
        freeSize = xPortGetFreeHeapSize();
        minFreeSize = xPortGetMinimumEverFreeHeapSize();
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO,
                           "#### Heap free space 0x%x, min 0x%x ####\r\n",
                           freeSize, minFreeSize);
    }    
#endif
    return pvPortMalloc(size);
}

void SRTM_Heap_Free(void *buf)
{
    vPortFree(buf);
}
