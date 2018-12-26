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
* $FileName: soc_pit.c
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
#include "soc_pit.h"
#define ELEMENTS_OF(x) ( sizeof(x)/sizeof(x[0]) )
static const void* pit_address[] =
{
    (void*)PIT_BASE_PTR
};

IRQInterruptIndex pit_vectors[] =
{
    INT_PIT0,
    INT_PIT1,
    INT_PIT2,
    INT_PIT3
};

/*GLOBAL FUNCTIONS*---------------------------------------------------------*/
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_get_base_address
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the base register address of the corresponding PIT
*    module.
*
*END*----------------------------------------------------------------------*/
void* pit_get_base_address
(
    /* [IN] PIT index */
    uint8_t dev_num
)
{
    if (dev_num < ELEMENTS_OF(pit_address)) 
    {
        return (void*)pit_address[dev_num];
    }
    return NULL;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_get_vector
* Returned Value   : Number of vectors associated with the peripheral
* Comments         :
*    This function returns desired interrupt vector for specified PIT module.
*
*END*----------------------------------------------------------------------*/
uint32_t pit_get_vector
(
    /* [IN] PIT channel */
    uint8_t channel
)
{
    if (channel < ELEMENTS_OF(pit_vectors)) 
    {
        return pit_vectors[channel];
    } 
    else 
    {
        return 0;
    }
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_init_freq
* Returned Value   : None
* Comments         : This function initializes the PIT frequence
*
*
*END*----------------------------------------------------------------------*/
int8_t pit_init_freq
(
    /* [IN] PIT index */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel,
    /* [IN] Ticks per second */
    uint32_t tickfreq,
    /* [IN] Clock speed in Hz */
    uint32_t clk,
    /* [IN] Unmask the timer after initializing */
    bool unmask_timer
)
{
    PIT_MemMapPtr pit_ptr = pit_get_base_address(timer);
    if ((NULL == pit_ptr)||(channel > PIT_CHANNEL_MAX)||(0 == tickfreq)||(0 == clk))
    {
        return PIT_INVALID_PARAM;
    }

    /* Enable PIT Module Clock */
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    /* Enable PIT module */
    pit_ptr->MCR &= ~(PIT_MCR_FRZ_MASK |PIT_MCR_MDIS_MASK);
    /* Set counter reload value and counter value */
    pit_ptr->CHANNEL[channel].LDVAL = (uint16_t)(clk/tickfreq);
    /* Clear interrupt flag */
    pit_ptr->CHANNEL[channel].TFLG |= PIT_TFLG_TIF_MASK;
    if(unmask_timer)
    {
        /* Enable Timer Interrupt */
        pit_unmask_int(timer,channel);
    }
    return PIT_OK;
}

#if 0
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_init_freq
* Returned Value   : None
* Comments         : This function initializes the PIT frequence
*
*
*END*----------------------------------------------------------------------*/
int8_t pit_timer_install
(
    /* [IN] PIT index */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel,
    /* [IN] Ticks per second */
    uint32_t tickfreq,
    /* [IN] Clock speed in Hz */
    uint32_t clk,
    /* [IN] Unmask the timer after initializing */
    bool unmask_timer,
    /* [IN] interrupt priority */
    uint32_t priority,
    /* [IN] Pointer to PIT ISR */
    void(* isr_ptr)(void*)
)
{
    IRQInterruptIndex vector   = pit_get_vector(channel);
    if ((PIT_OK != pit_init_freq(timer,channel,tickfreq,clk,unmask_timer))||(0 == vector))
    {
        return PIT_INVALID_PARAM;
    }
    /* Install the timer interrupt handler */
    _int_install_isr(vector,isr_ptr);
    _bsp_int_init(vector,priority,TRUE);
}
#endif
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_mask_int
* Returned Value   : None
* Comments         : This function mask interrupt of given timer
*
*
*END*----------------------------------------------------------------------*/
void pit_mask_int
(
    /* [IN] Timer to use */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel
)
{
    PIT_MemMapPtr pit_ptr = pit_get_base_address(timer);
    if (NULL != pit_ptr)
    {
        /* Disable Timer Interrupt */
        pit_ptr->CHANNEL[channel].TCTRL &= ~PIT_TCTRL_TEN_MASK;
        pit_ptr->CHANNEL[channel].TCTRL &= ~PIT_TCTRL_TIE_MASK;
    }
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_unmask_int
* Returned Value   : None
* Comments         : This function unmask interrupt of given timer
*
*
*END*----------------------------------------------------------------------*/
void pit_unmask_int
(
    /* [IN] Timer to use */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel
)
{
    PIT_MemMapPtr pit_ptr = pit_get_base_address(timer);
    if (NULL != pit_ptr)
    {
        /* Enable Timer Interrupt */
        pit_ptr->CHANNEL[channel].TCTRL |= PIT_TCTRL_TEN_MASK;
        pit_ptr->CHANNEL[channel].TCTRL |= PIT_TCTRL_TIE_MASK;
    }
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_check_int_flag
* Returned Value   : TRUE or FALSE
* Comments         : This function checks timer interrupt flag
*
*
*END*----------------------------------------------------------------------*/
bool pit_check_int_flag
(
    /* [IN] Timer to use */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel
)
{
    /* Get SPI module address */
    PIT_MemMapPtr pit_ptr = pit_get_base_address(timer);
    if (NULL == pit_ptr)
    {
        return FALSE;
    }
    return (((uint32_t)(pit_ptr->CHANNEL[channel].TFLG & PIT_TFLG_TIF_MASK)) ? TRUE : FALSE);
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : pit_clear_int
* Returned Value   : None
* Comments         : This function clear flag interrupt
*
*
*END*----------------------------------------------------------------------*/
void pit_clear_int
(
    /* [IN] Timer to use */
    uint8_t timer,
    /* [IN] Timer channel */
    uint8_t channel
)
{
    /* Get SPI module address */
    PIT_MemMapPtr pit_ptr = pit_get_base_address(timer);
    if (NULL == pit_ptr)
    {
        return;
    }
    /* Clear RTC Interrupt */
    pit_ptr->CHANNEL[channel].TFLG |= PIT_TFLG_TIF_MASK;
}
/* EOF */
