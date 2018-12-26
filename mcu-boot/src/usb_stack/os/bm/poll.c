/**HEADER********************************************************************
* 
* Copyright (c) 2010, 2013 - 2014 Freescale Semiconductor;
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
* $FileName: poll.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*   This file implements polling functionality.
*
*END************************************************************************/
#include "types.h"
#include "user_config.h"

#include "poll.h"

/* poll global variale */
poll_struct_t g_poll;

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : POLL_init
* Returned Value   : void
* Comments         : Init POLL_STRUCT object
*   
*END*----------------------------------------------------------------------*/
void POLL_init() 
{
    uint8_t i;

    g_poll.registered_no = 0; 
    for (i = 0; i <POLL_MAX_NUM; i++)
    {
        g_poll.p_slot[i].p_func = NULL;
        g_poll.p_slot[i].param = NULL;
    }
}
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : POLL_register
* Returned Value   : POLL_OK: register successfully
*                    POLL_REGISTER_FAIL: can NOT register more function to poll
* Comments         : Register funtion to poll
*   
*END*----------------------------------------------------------------------*/
uint8_t POLL_register(poll_pointer_t func, void* param) 
{
    uint8_t ret;
    
    if (POLL_MAX_NUM <= g_poll.registered_no)
    {
        return (uint8_t)POLL_REGISTER_FAIL;
    }
    else
    {
        ret = g_poll.registered_no;
        g_poll.p_slot[g_poll.registered_no].p_func = func;
        g_poll.p_slot[g_poll.registered_no].param = param;
        g_poll. registered_no ++;
        return ret;
    }
}
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : Poll
* Returned Value   : void
* Comments         : Poll function to call in while loop
*   
*END*----------------------------------------------------------------------*/
void Poll() 
{
    uint8_t i;

    for (i = 0; i < g_poll.registered_no; i++)
    {
        g_poll.p_slot[i].p_func(g_poll.p_slot[i].param);
    }
}
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : Poll
* Returned Value   : POLL_OK: register successfully
*                    POLL_NOT_FOUND: Nothing to unregister
* Comments         : Unregister polling function
*   
*END*----------------------------------------------------------------------*/
uint8_t POLL_unregister(poll_pointer_t func, void * param) 
{
    uint8_t i;
    uint8_t j =0;
    uint8_t ret = (uint8_t)POLL_NOT_FOUND;

    poll_slot_struct_t temp[POLL_MAX_NUM] = {NULL};

    /* find and unregister func */
    for(i = 0; i < POLL_MAX_NUM; i++) 
    {
        if ((g_poll.p_slot[i].p_func == func) && (g_poll.p_slot[i].param == param))
        {
            g_poll.p_slot[i].p_func = NULL;
            g_poll.p_slot[i].param = NULL;
            g_poll.registered_no--;
            ret = POLL_OK;
        }
        else
        {
            temp[j].p_func = g_poll.p_slot[i].p_func;
            temp[j].param = g_poll.p_slot[i].param;
            j++;
        }
    }

    /* sort the function pointer erray */
    for(i = 0; i < POLL_MAX_NUM; i++) 
    {
        g_poll.p_slot[i].p_func = temp [i].p_func;
        g_poll.p_slot[i].param = temp [i].param;
    }
    return ret;
}

