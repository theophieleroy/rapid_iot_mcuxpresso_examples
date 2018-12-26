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
* $FileName: bmevent.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*   
*
*
*END************************************************************************/
#include "types.h"
#include "bmevent.h"

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_event_init
* Returned Value   :
* Comments         : Allocate event pointer, clear value and set event in valid state.
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_event_init
(
    bm_event_struct_t* event
)
{
    event->valid = BM_EVENT_VALID;
    event->value = 0;
    return  BM_EVENT_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_event_set
* Returned Value   : 
* Comments         : Set value of event pointer. 
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_event_set
(
  bm_event_struct_t* event,
  uint32_t value
)
{
	sys_lock();
	if(event->valid == BM_EVENT_VALID)
	{
		event->value |= value;
		sys_unlock();
		return BM_EVENT_VALID;
	}
	sys_unlock();	
    return BM_EVENT_INVALID;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_event_clear
* Returned Value   : 
* Comments         : Clear value of event pointer.
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_event_clear
(
  bm_event_struct_t* event,
  uint32_t bitmask
)
{
	sys_lock(); 
	if((uint8_t)(event->valid) == BM_EVENT_VALID)
	{
		event->value &= ~bitmask;
		sys_unlock();
		return BM_EVENT_VALID;
	}
	sys_unlock();
    return BM_EVENT_INVALID;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_event_wait_ticks
* Returned Value   : 
* Comments         : This function returns the value USB_EVENT_SET when an event occurs, but timeout not support.
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_event_wait_ticks
(
    bm_event_struct_t* event,
    uint32_t bitmask,
    uint8_t  all,
    uint16_t ticks
)
{
    UNUSED(all)
    UNUSED(ticks)

    if(event->valid == BM_EVENT_VALID) 
    {
        if(0x00 != (event->value & bitmask))
        {
            return BM_EVENT_SET;
        } 
        else 
        {
            return BM_EVENT_NOT_SET;
        }
    } 
    return (uint16_t)BM_EVENT_INVALID; 
}
