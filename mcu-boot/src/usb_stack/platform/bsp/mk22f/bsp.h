/**HEADER*********************************************************************
*
* Copyright (c) 2011, 2013 Freescale Semiconductor;
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
* $FileName: bsp.h
* $Version :
* $Date    :
*
* Comments:
*
*   This file includes all include files specific to this target system.
*
*END*************************************************************************/
#ifndef __bsp_h__
#define __bsp_h__   1

#include "derivative.h"
#include "soc.h"

#define SYS_CLK         120000000
#define BUS_CLK         60000000

#define SCI_CLK         SYS_CLK
#define SCI_CHANNEL     1

#define _bsp_int_init nvic_int_init
#define _int_install_isr install_isr
#ifdef __cplusplus
extern "C" {
#endif

void bsp_serial_io_init(uint8_t dev_num);

#ifdef __cplusplus
}
#endif

#endif  /* __bsp_h__ */
