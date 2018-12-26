/**HEADER********************************************************************
*
* Copyright (c) 2013- 2014 Freescale Semiconductor;
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
* $FileName: adapter_bm.h$
* $Version :
* $Date    :
*
* Comments:
*
* @brief The file contains the definition of BM of OS adapter header.
*
*****************************************************************************/

#ifndef _USB_OSADAPTER_BM_H
#define _USB_OSADAPTER_BM_H 1
#include "types.h"
#include "adapter_types.h"
#include "compiler.h"
#include "soc.h"
#include "bootloader_common.h"

extern void sys_lock(void);
extern void sys_unlock(void);
extern void *memcpy(void *, const void *, uint32_t);
extern void *memset(void *s, int c, uint32_t n);
// extern int32_t printf_kinetis (const char *fmt, ...);
extern void *BM_mem_alloc_zero(uint32_t size);
extern void *BM_mem_alloc_word_aligned(uint32_t size);
extern void BM_mem_free(void *buffer);
extern int32_t bm_install_isr(uint32_t vector, osa_int_isr_fptr isr_ptr, void *isr_data);
extern int32_t bm_int_init(uint8_t num, uint8_t prior, bool enable);
extern void OSA_TimeDelay(uint32_t delay);
extern uint32_t OS_Event_get_value(os_event_handle handle);

//#define USB_PRINTF                                         printf_kinetis
#define OS_install_isr bm_install_isr
#define OS_intr_init(num, prior, subprior, enable) bm_int_init(num, prior, enable)

#define TICKS_PER_SEC 1000
/* memory operation wrapper */
#define OS_Lock() sys_lock()
#define OS_Unlock() sys_unlock()

/* Based on the targets it should be modified, for ColdFire it is MBYTES */
#define OS_dcache_invalidate_mlines(p, n)
#define OS_dcache_flush_mlines(p, n)

#ifndef OS_mem_alloc_uncached
#define OS_Mem_alloc_uncached BM_mem_alloc_word_aligned
#endif

#ifndef OS_mem_alloc_uncached_zero
#define OS_Mem_alloc_uncached_zero BM_mem_alloc_zero
#endif

#define OS_Mem_alloc_zero(n) BM_mem_alloc_zero(n)
#define OS_Mem_alloc(n) BM_mem_alloc_zero(n)
#define OS_Mem_free(ptr) BM_mem_free(ptr)
#define OS_Mem_zero(ptr, n) memset((ptr), (0), (n))
#define OS_Mem_copy(src, dst, n) memcpy((dst), (src), (n))

#define OS_Event_set _bm_event_set
#define OS_Event_clear _bm_event_clear

#define OS_MsgQ_create _bm_msgq_init
#define OS_MsgQ_destroy _bm_msgq_deinit
#define OS_Time_delay time_delay
#define OS_Sem_post _bm_sem_post
#endif
