/**HEADER*********************************************************************
*
* Copyright (c) 2004-2010, 2013 - 2014 Freescale Semiconductor;
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
* $FileName: soc_pit.h
* $Version :
* $Date    :
*
* Comments:
*
*  
*
*END*************************************************************************/
 #ifndef _SOC_PIT_H_
 #define _SOC_PIT_H_

/*INCLUDES*-----------------------------------------------------------------*/

/*MACROS*-------------------------------------------------------------------*/
#define PIT_OK                          (0)
#define PIT_INVALID_PARAM               (-1)
#define PIT_CHANNEL_MAX                 4

/*GLOBAL FUNCTION PROTOTYPES*-----------------------------------------------*/
void* pit_get_base_address(uint8_t);
uint32_t pit_get_vector(uint8_t);
int8_t pit_init_freq(uint8_t,uint8_t,uint32_t,uint32_t,bool);
int8_t pit_timer_install(uint8_t,uint8_t,uint32_t,uint32_t,bool,uint32_t,void (* isr_ptr)(void *));
void pit_mask_int(uint8_t,uint8_t);
void pit_unmask_int(uint8_t,uint8_t);
bool pit_check_int_flag(uint8_t,uint8_t);
void pit_clear_int(uint8_t,uint8_t);

#endif
/* EOF */
