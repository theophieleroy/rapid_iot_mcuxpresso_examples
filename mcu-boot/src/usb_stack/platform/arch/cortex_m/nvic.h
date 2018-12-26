/**HEADER********************************************************************
 *
 * Copyright (c) 2010, 2013 Freescale Semiconductor;
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
 * $FileName: nvic.h
 * $Version :
 * $Date    :
 *
 * Comments:
 *
 *
 *
 *
 *
 *
 *
 *END************************************************************************/
#ifndef _NVIC_H_
#define _NVIC_H_
/******************************************************************************
 * Includes
 *****************************************************************************/
#include "bootloader_common.h"
/******************************************************************************
* Macro
*****************************************************************************/
/* minimal implemented priority required by Cortex core */
#ifndef CORTEX_PRIOR_IMPL
#if CPU_IS_ARM_CORTEX_M0P
#define CORTEX_PRIOR_IMPL (1)
#elif CPU_IS_ARM_CORTEX_M4
#define CORTEX_PRIOR_IMPL (3)
#endif
#endif /* CORTEX_PRIOR_IMPL */
#define CORTEX_PRIOR_SHIFT (8 - CORTEX_PRIOR_IMPL)
#define CORTEX_PRIOR_MASK ((0xff << CORTEX_PRIOR_SHIFT) & 0xff)
#define CORTEX_PRIOR(x) (((x) << CORTEX_PRIOR_SHIFT) & CORTEX_PRIOR_MASK)
#define CORTEX_PRI_PERIPH_IN_BASE (0xE000E100u)

#define INT_FIRST_INTERNAL (0)

#ifndef INT_LAST_INTERNAL
#if CPU_IS_ARM_CORTEX_M0P
#define INT_LAST_INTERNAL (32)
#elif CPU_IS_ARM_CORTEX_M4
#define INT_LAST_INTERNAL (250)
#endif
#endif

#define NVIC_OK (0)
#define NVIC_INVALID_PARAM (-1)
/*
** CORTEX_NVIC_STRUCT
** Reset and Clock Control
*/
#if CPU_IS_ARM_CORTEX_M4
typedef struct cortex_nvic_struct
{
    uint32_t ENABLE[32];
    uint32_t DISABLE[32];
    uint32_t SET[32];
    uint32_t CLR[32];
    uint32_t ACTIVE[32];
    uint32_t rsvd[32];
    uint32_t PRIORITY[32];
} CORTEX_NVIC_STRUCT, *CORTEX_NVIC_STRUCT_PTR;
#elif CPU_IS_ARM_CORTEX_M0P
typedef struct cortex_nvic_struct
{
    uint32_t ENABLE;
    uint8_t RESERVED_0[124];
    uint32_t DISABLE;
    uint8_t RESERVED_1[124];
    uint32_t SET;
    uint8_t RESERVED_2[124];
    uint32_t CLR;
    uint8_t RESERVED_3[380];
    uint32_t PRIORITY[8];
} CORTEX_NVIC_STRUCT, *CORTEX_NVIC_STRUCT_PTR;
#endif

typedef volatile struct cortex_nvic_struct *VCORTEX_NVIC_STRUCT_PTR;
extern int8_t nvic_int_init(uint8_t, uint8_t, bool);
extern int8_t nvic_int_enable(uint8_t);
extern int8_t nvic_int_disable(uint8_t);
extern void sys_lock();
extern void sys_unlock();
#endif
/*EOF*/
