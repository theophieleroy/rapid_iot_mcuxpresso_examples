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
* Comments:  
*
*END************************************************************************/
#include "adapter.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
    #if (defined(CPU_MK64F12))
#include "fsl_device_registers.h"
    #endif
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
#include "MK64F12.h"
#endif
#define SIM_SOPT2_IRC48MSEL_MASK                 0x30000u
extern uint8_t soc_get_usb_vector_number(uint8_t controller_id);

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#define BSP_USB_INT_LEVEL                (4)
#define USB_CLK_RECOVER_IRC_EN (*(volatile unsigned char *)0x40072144)
#define BSPCFG_USB_USE_IRC48M            (1)
static int32_t bsp_usb_dev_io_init
(
    int32_t i
)
{
    int32_t ret = 0;
    if (i == 0)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#if BSPCFG_USB_USE_IRC48M
    
    SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;
    
#else
    /* Configure USB to be clocked from PLL0 */
    SIM_SOPT2_REG(SIM_BASE_PTR) |= SIM_SOPT2_USBSRC_MASK;
    SIM_SOPT2_REG(SIM_BASE_PTR) |= SIM_SOPT2_PLLFLLSEL_MASK;
    /* Configure USB divider to be 120MHz * 2 / 5 = 48 MHz */
     
    SIM_WR_CLKDIV2_USBFRAC(SIM_BASE_PTR, 1);
    SIM_WR_CLKDIV2_USBDIV(SIM_BASE_PTR, 4);

    /* Enable USB-OTG IP clocking */
    SIM_SCGC4_REG(SIM_BASE_PTR) |= SIM_SCGC4_USBOTG_MASK;
#endif
#endif
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#if BSPCFG_USB_USE_IRC48M
    /* USB clock divider */
    CLOCK_SYS_SetUsbfsDiv(i, 0U, 0U);

    /* PLL/FLL selected as CLK source */
    CLOCK_SYS_SetUsbfsSrc(i, kClockUsbfsSrcPllFllSel);
    CLOCK_SYS_SetPllfllSel(kClockPllFllSelIrc48M);

    /* USB Clock Gating */
    CLOCK_SYS_EnableUsbfsClock(i);
    /* Enable IRC 48MHz for USB module */
    USB_CLK_RECOVER_IRC_EN = 0x03;
#else
    /* PLL/FLL selected as CLK source */
    CLOCK_SYS_SetUsbfsSrc(i, kClockUsbfsSrcPllFllSel);
    CLOCK_SYS_SetPllfllSel(kClockPllFllSelPll);
    
    /* USB Clock Gating */
    CLOCK_SYS_EnableUsbfsClock(i);
    
    /* Enable clock gating to all ports, A, B, C, D, E*/
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);
    CLOCK_SYS_EnablePortClock(2);
    CLOCK_SYS_EnablePortClock(3);
    CLOCK_SYS_EnablePortClock(4);

    /* Weak pull downs */
    HW_USB_USBCTRL_WR(0x40);
#endif
#endif
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
#if BSPCFG_USB_USE_IRC48M
    /*
    * Configure SIM_CLKDIV2: USBDIV = 0, USBFRAC = 0
    */
    SIM_CLKDIV2 &= ~(SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK);
    /* Configure USB to be clocked from IRC 48MHz */
    SIM_SOPT2 |= (SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_IRC48MSEL_MASK);
    /* Enable USB-OTG IP clocking */
    SIM_SCGC4 |= SIM_SCGC4_USBOTG_MASK;
    /* Enable IRC 48MHz for USB module */
    USB0_CLK_RECOVER_IRC_EN = 0x03;
#else
    /* USB Clock Gating */
    SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);       
                                            

              /* Enable clock gating to all ports */
    SIM_SCGC5 |=   SIM_SCGC5_PORTA_MASK \
                             | SIM_SCGC5_PORTB_MASK \
                             | SIM_SCGC5_PORTC_MASK \
                             | SIM_SCGC5_PORTD_MASK \
                             | SIM_SCGC5_PORTE_MASK;


    /* PLL/FLL selected as CLK source */
    SIM_SOPT2 |= (SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL(0x01));

    /* Weak pull downs */
    USB0_USBCTRL = (0x40);
#endif
#endif
    }
    else
    {
        ret = -1; //unknow controller
    }

    return ret;
}

int32_t bsp_usb_dev_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_dev_io_init(controller_id);
    if (result != 0)
    {
        return result;
    }

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
    /* SYSMPU is disabled. All accesses from all bus masters are allowed */
    MPU_CESR=0;
    if (0 == controller_id)
    {
        /* Configure enable USB regulator for device */
        HW_SIM_SOPT1CFG_SET(SIM_BASE, SIM_SOPT1CFG_URWE_MASK);
        HW_SIM_SOPT1_SET(SIM_BASE, SIM_SOPT1_USBREGEN_MASK);

        /* reset USB CTRL register */
        HW_USB_USBCTRL_WR(USB0_BASE, 0);

        /* Enable internal pull-up resistor */
        HW_USB_CONTROL_WR(USB0_BASE, USB_CONTROL_DPPULLUPNONOTG_MASK);
        HW_USB_USBTRC0_SET(USB0_BASE, 0x40); /* Software must set this bit to 1 */
        /* setup interrupt */
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
        OS_intr_init(USB0_IRQn + 16, BSP_USB_INT_LEVEL, 0, TRUE);
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
        OS_intr_init(USB0_IRQn, BSP_USB_INT_LEVEL, 0, TRUE);
#endif

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
    /* SYSMPU is disabled. All accesses from all bus masters are allowed */
    MPU_CESR=0;
    if (0 == controller_id)
    {
        /* Configure enable USB regulator for device */
        SIM_SOPT1CFG |= (SIM_SOPT1CFG_URWE_MASK);
        SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

        /* reset USB CTRL register */
        USB0_USBCTRL = 0;

        /* Enable internal pull-up resistor */
        USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG_MASK;
        USB0_USBTRC0 = 0x40; /* Software must set this bit to 1 */
        /* setup interrupt */
        OS_intr_init(INT_USB0, BSP_USB_INT_LEVEL, 0, TRUE);
#else
    /* SYSMPU is disabled. All accesses from all bus masters are allowed */
    MPU_CESR=0;    
//    HW_MPU_CESR_WR(0);
    if (0 == controller_id)
    {

        /* Configure enable USB regulator for device */
        SIM_SOPT1_REG(SIM_BASE_PTR) |= SIM_SOPT1_USBREGEN_MASK;

        /* reset USB CTRL register */
        USB_CONTROL_REG(USB0_BASE_PTR) = (0);
        
        /* Enable internal pull-up resistor */
        USB_CONTROL_REG(USB0_BASE_PTR) = USB_CONTROL_DPPULLUPNONOTG_MASK;
        /* setup interrupt */
        OS_intr_init((IRQn_Type)soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
#endif
    }
    else
    {
        /* unknown controller */
        result = -1;
    }

    return result;
}
#endif
/* EOF */
