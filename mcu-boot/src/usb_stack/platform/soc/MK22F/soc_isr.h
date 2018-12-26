/**HEADER*********************************************************************
*
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
*
******************************************************************************
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
******************************************************************************
*
* $FileName: soc_isr.h
* $Version :
* $Date    :
*
* Comments:
*
*  
*
*END*************************************************************************/
#ifndef SOC_ISR_H
#define SOC_ISR_H
#include "types.h"
#include "soc.h"
#include "derivative.h"
typedef void (*int_isr_fptr_t)(void*);

#ifdef __cplusplus
extern "C" {
#endif

int32_t soc_install_isr( uint32_t vector, int_isr_fptr_t isr_ptr, void* isr_data);

#ifdef __cplusplus
}
#endif

#endif
/* EOF */
