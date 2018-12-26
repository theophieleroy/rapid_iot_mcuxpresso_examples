/**HEADER********************************************************************
*
* Copyright (c) 2015 Freescale Semiconductor;
* All Rights Reserved
*
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName:soc_usb.c
* $Version :
* $Date    :
*
* Comments:
*
*
*END************************************************************************/
#include "types.h"
#include "soc.h"
#include "adapter.h"

uint32_t soc_get_usb_base_address(uint8_t controller_id)
{
    if (controller_id == 0)
    {
        return (uint32_t)USB0_BASE_PTR;
    }
    else
    {
        return (uint32_t)0;
    }
}

uint8_t soc_get_usb_vector_number(uint8_t controller_id)
{
    if (controller_id == 0)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
        return INT_USB0;
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
        return USB0_IRQn + 16;
#endif
    }
    else
    {
        return 0xFF;
    }
}
/* EOF */
