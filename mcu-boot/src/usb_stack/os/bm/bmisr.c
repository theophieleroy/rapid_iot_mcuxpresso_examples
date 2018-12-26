/**HEADER********************************************************************
*
* Copyright (c) 2010, 2013- 2014 Freescale Semiconductor;
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
* $FileName: bmisr.c$
* $Version :
* $Date    :
*
* Comments:
*
*   This file implements ISR related functionality.
*
*END************************************************************************/

#include "types.h"
#include "soc_isr.h"
#include "nvic.h"
#include "bootloader_common.h"

int32_t bm_install_isr(uint32_t vector, int_isr_fptr_t isr_ptr, void *isr_data)
{
    return soc_install_isr(vector, isr_ptr, isr_data);
}

int32_t bm_int_init(uint8_t num, uint8_t prior, bool enable)
{
    return nvic_int_init(num, prior, enable);
}
