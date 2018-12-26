/**HEADER********************************************************************
*
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
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

uint32_t soc_get_usb_base_address(uint8_t controller_id)
{
    if (controller_id == 0)
    {
        return (uint32_t)USB0_BASE_PTR;
    }
    else
    {
        return (uint32_t)USBHS_BASE_PTR;
    }
}

uint8_t soc_get_usb_vector_number(uint8_t controller_id)
{
    if (controller_id == 0)
    {
        return INT_USB0;
    }
    else
    {
        return INT_USBHS;
    }
}
/* EOF */
