/**HEADER********************************************************************
*
* Copyright (c) 2012, 2013 - 2014 Freescale Semiconductor;
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
* $FileName:nvic.c
* $Version :
* $Date    :
*
* Comments:
* ARM Nested Vectored Interrupt Controller (NVIC)
*
*END************************************************************************/
#include "types.h"
#include "soc.h"
#include "nvic.h"

uint32_t g_usb_lock_level;

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name   : nvic_int_init
* Returned Value  : int8_t
*       NVIC_OK or error code
* Comments        :
*  	Initialize a specific interrupt in the cortex core nvic
*
*END*---------------------------------------------------------------------*/
int8_t nvic_int_init
(
    /* [IN] Interrupt number */
    uint8_t irq,
    /* [IN] Interrupt priority */
    uint8_t prior,
    /* [IN] enable the interrupt now? */
    bool enable
   )
{
    VCORTEX_NVIC_STRUCT_PTR nvic = (VCORTEX_NVIC_STRUCT_PTR)CORTEX_PRI_PERIPH_IN_BASE;
    uint8_t ext_irq_no = irq - 16;

    /* check priority value, must be below maximal enabled/set value */
    if (prior >= (1 << CORTEX_PRIOR_IMPL))
    {
        return NVIC_INVALID_PARAM;
    }

    if (ext_irq_no <= (uint8_t)INT_LAST_INTERNAL)
    {
        nvic->PRIORITY[ext_irq_no >> 2] = (nvic->PRIORITY[ext_irq_no >> 2] & ~(0xff << ((ext_irq_no & 3) * 8))) | (((prior << CORTEX_PRIOR_SHIFT) & CORTEX_PRIOR_MASK) << ((ext_irq_no & 3) * 8));
        if (enable)
        {
            nvic_int_enable(irq);
        }
        else
        {
            nvic_int_disable(irq);
        }
    }
    else
    {
        return NVIC_INVALID_PARAM;
    }

    return NVIC_OK;
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name   : nvic_int_enable
* Returned Value  : int8_t
*       NVIC_OK or error code
* Comments        :
*     Enable interrupt on cortex core NVIC
*
*END*---------------------------------------------------------------------*/
int8_t nvic_int_enable
(
    /* [IN] Interrupt number */
    uint8_t  irq
)
{
    VCORTEX_NVIC_STRUCT_PTR nvic = (VCORTEX_NVIC_STRUCT_PTR)CORTEX_PRI_PERIPH_IN_BASE;
    uint8_t ext_irq_no = irq - 16;

    if (ext_irq_no <= INT_LAST_INTERNAL)
    {
#if CPU_IS_ARM_CORTEX_M0P
        nvic->ENABLE = 1 << ext_irq_no;
#elif CPU_IS_ARM_CORTEX_M4
        nvic->ENABLE[ext_irq_no >> 5] = 1 << (ext_irq_no & 0x1f);
#endif
    }
    else
    {
        return NVIC_INVALID_PARAM;
    }
    return NVIC_OK;
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name   : nvic_int_disable
* Returned Value  : int8_t
*       NVIC_OK or error code
* Comments        :
*     Disable interrupt on cortex core NVIC
*
*END*---------------------------------------------------------------------*/
int8_t nvic_int_disable
(
    /* [IN] Interrupt number */
    uint8_t irq
)
{
    VCORTEX_NVIC_STRUCT_PTR nvic = (VCORTEX_NVIC_STRUCT_PTR)CORTEX_PRI_PERIPH_IN_BASE;
    uint8_t ext_irq_no = irq - 16;

    if (ext_irq_no <= INT_LAST_INTERNAL)
    {
#if CPU_IS_ARM_CORTEX_M0P
        nvic->DISABLE = 1 << ext_irq_no;
#elif CPU_IS_ARM_CORTEX_M4
        nvic->DISABLE[ext_irq_no >> 5] = 1 << (ext_irq_no & 0x1f);
#endif
    }
    else
    {
        return NVIC_INVALID_PARAM;
    }

    return NVIC_OK;
}

#ifdef __CC_ARM
#define _SYS_LOCK_()   __disable_irq()
#define _SYS_UNLOCK_() __enable_irq()
#else
#define _SYS_LOCK_()   asm(" CPSID i")
#define _SYS_UNLOCK_() asm(" CPSIE i")
#endif

void sys_lock()
{
    if (0 == g_usb_lock_level)
    {
        _SYS_LOCK_();
    }
    g_usb_lock_level++;
}

void sys_unlock()
{
    if (g_usb_lock_level)
    {
        g_usb_lock_level--;
    }

    if (0 == g_usb_lock_level)
    {
       _SYS_UNLOCK_();
    }
}

/* EOF */

