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
* $FileName: soc_isr.c
* $Version :
* $Date    :
*
* Comments:
*
*  
*
*END*************************************************************************/
#include "types.h"
#include "soc.h"
#include "bsp.h"
#include "derivative.h"
#include "user_config.h"
#if (DELAY_ENABLE) || (MAX_TIMER_OBJECTS)
#include "soc_pit.h"
#endif
#if (PRINTF_ENABLE)
#include "soc_sci.h"
#endif
#if I2S_ENABLE
#include "soc_i2s.h"
#endif

#if BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID

typedef void (*int_isr_fptr_t)(void*);
static int_isr_fptr_t lowlevel_usb_isr = NULL;
static void* isr_param_usb_isr = NULL;

#if (PRINTF_ENABLE)
static int_isr_fptr_t lowlevel_uart_isr = NULL;
static void* isr_param_uart_isr = NULL;
#endif

#if PORTD_ENABLE
static int_isr_fptr_t lowlevel_portd_isr = NULL;
static void* isr_param_portd_isr = NULL;
#endif /* PORTD_ENABLE */

#if (DELAY_ENABLE) || (MAX_TIMER_OBJECTS)
static int_isr_fptr_t lowlevel_pit0_isr = NULL;
static int_isr_fptr_t lowlevel_pit1_isr = NULL;
#endif /* (DELAY_ENABLE) || (MAX_TIMER_OBJECTS) */

#if (I2S_ENABLE)
static int_isr_fptr_t lowlevel_i2s0_tx_isr = NULL;
#endif /* I2S_ENABLE */


int32_t soc_install_isr( uint32_t vector, int_isr_fptr_t isr_ptr, void* isr_data)
{
    if(vector == soc_get_usb_vector_number(0))
    {
        lowlevel_usb_isr = isr_ptr;
        isr_param_usb_isr = isr_data;
    }

#if PORTD_ENABLE
    if(vector == (PORTD_IRQn + 16))
    {
        lowlevel_portd_isr = isr_ptr;
        isr_param_portd_isr = isr_data;
    }
#endif /* PORTD_ENABLE */

#if (DELAY_ENABLE) || (MAX_TIMER_OBJECTS)
    if(vector == pit_get_vector(0))
    {
        lowlevel_pit0_isr = isr_ptr;
    }
    
    if(vector == pit_get_vector(1))
    {
        lowlevel_pit1_isr = isr_ptr;
    }
#endif /* (DELAY_ENABLE) || (MAX_TIMER_OBJECTS) */

#if I2S_ENABLE
    if(vector == i2s_get_vector(0))
    {
        lowlevel_i2s0_tx_isr = isr_ptr;
    }
#endif /* I2S_ENABLE */

#if PRINTF_ENABLE
    if(vector == bsp_get_serial_vector(SCI_CHANNEL))
    {
        lowlevel_uart_isr = isr_ptr;
        isr_param_uart_isr = isr_data;
    }
#endif
    return 1;
}

void USB0_IRQHandler(void)
{
    if(lowlevel_usb_isr != NULL)
    {
        lowlevel_usb_isr(isr_param_usb_isr);
    }
}

void PIT0_ISR(void)
{
#if (DELAY_ENABLE) || (MAX_TIMER_OBJECTS)
    if(lowlevel_pit0_isr != NULL)
    {
        lowlevel_pit0_isr(NULL);
    }
#endif /* (DELAY_ENABLE) || (MAX_TIMER_OBJECTS) */
}

void PIT1_ISR(void)
{
#if (DELAY_ENABLE) || (MAX_TIMER_OBJECTS)
    if(lowlevel_pit1_isr != NULL)
    {
        lowlevel_pit1_isr(NULL);
    }
#endif /* (DELAY_ENABLE) || (MAX_TIMER_OBJECTS) */
}


void PORTD_ISR(void)
{
#if PORTD_ENABLE
    if(lowlevel_portd_isr != NULL)
    {
        lowlevel_portd_isr(isr_param_portd_isr);
    }
#endif /* PORTA_ENABLE */
}

void I2S0_TX_ISR(void)
{
#if I2S_ENABLE
    if(lowlevel_i2s0_tx_isr != NULL)
    {
        lowlevel_i2s0_tx_isr(NULL);
    }
#endif /* I2S_ENABLE */
}

void UART_ISR(void)
{
#if (PRINTF_ENABLE)	
  if(lowlevel_uart_isr != NULL)
      lowlevel_uart_isr(isr_param_uart_isr);
#endif
}

#endif // BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID

/* EOF */

