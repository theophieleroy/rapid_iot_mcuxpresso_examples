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
* $FileName: osadapter_bm.c$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file includes the implementation of BM of OS adapter.
* 
*****************************************************************************/ 
#include "adapter_cfg.h"
#include "adapter_types.h"

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#include "types.h"
#include "adapter_bm.h"
#include "bmevent.h"
#include "bmmsgq.h"
#include "poll.h"
#include "bmsem.h"

#pragma weak delay
extern void delay(uint32_t delay);

typedef struct registered_info_struct
{
   poll_pointer_t   func;
   void*            param;
} registered_info_struct_t;

registered_info_struct_t   registered_function[POLL_MAX_NUM];

uint32_t OS_Task_create(task_start_t pstart, void* param, uint32_t pri, uint32_t stack_size, char* task_name, void* opt)
{
    uint8_t index = POLL_register(pstart, param);
    if (index != (uint8_t)POLL_REGISTER_FAIL)
    {
        registered_function[index].func = pstart;
        registered_function[index].param = param;
        return index;
    }
    return (uint32_t)OS_TASK_ERROR;
}

uint32_t OS_Task_delete(uint32_t task_id)
{
    uint8_t index = task_id;
    poll_pointer_t p_func = NULL;
    void*          param  = NULL;
    
    if(index >= POLL_MAX_NUM)
    {
        return (uint32_t)OS_TASK_ERROR;
    }
    
    p_func = registered_function[index].func;
    param = registered_function[index].param;

    if (POLL_unregister(p_func, param) != (uint8_t)POLL_NOT_FOUND)
    {
        registered_function[index].param = NULL;
        registered_function[index].func = NULL;
        return OS_TASK_OK;
    }
    else
    {
        return (uint32_t)OS_TASK_ERROR;
    }
}

uint32_t OS_Task_suspend(uint32_t task_id)
{
    return (uint32_t)OS_TASK_OK;
}

uint32_t OS_Task_resume(uint32_t task_id)
{
    return (uint32_t)OS_TASK_OK;
}

os_event_handle OS_Event_create(uint32_t flag)
{
    bm_event_struct_t *event;
    event = (bm_event_struct_t*)BM_mem_alloc_word_aligned(sizeof(bm_event_struct_t));
    if (event == NULL)
    {
        return NULL;
    }
    
    if (_bm_event_init(event) != BM_EVENT_OK)
    {
        BM_mem_free((void*)event);
        return NULL;
    }
    return (os_event_handle)event;
}

uint32_t OS_Event_destroy(os_event_handle handle)
{
    if (NULL == handle)
        return (uint32_t)OS_EVENT_ERROR;
    BM_mem_free((void*)handle);
    return (uint32_t)OS_EVENT_OK;
}

uint32_t OS_Event_check_bit(os_event_handle handle, uint32_t bitmask)
{
    return (((bm_event_struct_t*)handle)->value & bitmask);
}

uint32_t OS_Event_get_value(os_event_handle handle)
{
        return ((bm_event_struct_t*)handle)->value;
}
#if 0
uint32_t OS_Event_set(os_event_handle handle, uint32_t bitmask)
{
    LWEVENT_STRUCT *event = (LWEVENT_STRUCT*)handle;
    if (_lwevent_set(event, bitmask) != MQX_OK)
    {
        return OS_EVENT_ERROR;
    }
    return OS_EVENT_OK;
}


uint32_t OS_Event_clear(os_event_handle handle, uint32_t bitmask)
{
    LWEVENT_STRUCT *event = (LWEVENT_STRUCT*)handle;
    if(_lwevent_clear(event, bitmask) != MQX_OK)
    {
        return OS_EVENT_ERROR;
    }
    return OS_EVENT_OK;
}

#endif
uint32_t OS_Event_wait(os_event_handle handle, uint32_t bitmask, uint32_t flag, uint32_t timeout)
{
    bm_event_struct_t *event = (bm_event_struct_t*)handle;
    uint32_t ret;
    uint32_t wait_ticket = timeout * 10;
    
    ret = _bm_event_wait_ticks(event, bitmask, flag, timeout);
    
    if(BM_EVENT_SET == ret)
    {
        return OS_EVENT_OK;
    }

    while (wait_ticket--)
    {
        delay(1);
        ret = _bm_event_wait_ticks(event, bitmask, flag, timeout);
            
        if(BM_EVENT_SET == ret)
        {
            return (uint32_t)OS_EVENT_OK;
        }
    }
    return (uint32_t)OS_EVENT_TIMEOUT;
}

#if 0
os_msgq_handle OS_MsgQ_create(uint32_t max_msg_number, uint32_t msg_size)
{
    void* msgq;
    uint32_t size = sizeof(LWMSGQ_STRUCT) + max_msg_number * msg_size + 4;
    
    msgq = _mem_alloc_system_zero(size);
    if (msgq == NULL)
    {
        return NULL;
    }

    size = (msg_size - 1) / sizeof(uint32_t) + 1;
    if (_lwmsgq_init(msgq, max_msg_number, size) != MQX_OK)
    {
        _mem_free(msgq);
        return NULL;
    }
       
    return (os_msgq_handle)msgq;
}
#endif

uint32_t OS_MsgQ_send(os_msgq_handle msgq, void* msg, uint32_t flag)
{
    if (BM_MSGQ_OK != _bm_msgq_send((bm_msgq_handle)msgq, (int32_t *) msg))
    {
        return (uint32_t)OS_MSGQ_ERROR;
    }
    return (uint32_t)OS_MSGQ_OK;
}

uint32_t OS_MsgQ_recv(os_msgq_handle msgq, void* msg, uint32_t flag, uint32_t timeout)
{
    if (BM_MSGQ_OK != _bm_msgq_receive((bm_msgq_handle)msgq, (int32_t *) msg))
    {
        return (uint32_t)OS_MSGQ_ERROR;
    }
    return (uint32_t)OS_MSGQ_OK;
}

#if 0
uint32_t OS_MsgQ_destroy(os_msgq_handle msgq)
{
    _mem_free(msgq);
    return OS_MSGQ_OK;
}
#endif
uint32_t OS_MsgQ_Is_Empty(os_msgq_handle msgq, void* msg)
{
  uint32_t ret;

    ret = BMMSGQ_IS_EMPTY(msgq);
  if(!ret)
  {
    if (BM_MSGQ_OK != _bm_msgq_receive((bm_msgq_handle)msgq, (int32_t *) msg))
    {
            return (uint32_t)OS_MSGQ_ERROR;
    }
  }
  return ret;
}

os_mutex_handle OS_Mutex_create()
{
    return (os_mutex_handle)(0x0000FFFF);
}

uint32_t OS_Mutex_lock(os_mutex_handle mutex)
{
    return OS_MUTEX_OK;
}

uint32_t OS_Mutex_unlock(os_mutex_handle mutex)
{
    return OS_MUTEX_OK;
}

uint32_t OS_Mutex_destroy(os_mutex_handle mutex)
{
    return OS_MUTEX_OK;
}

os_gpio_handle OS_Gpio_init(uint32_t id, uint32_t dir, uint32_t value)
{
    return (os_gpio_handle)(0x0000FFFF);
}

uint32_t OS_Gpio_set_functionality(os_gpio_handle handle, uint32_t function)
{
    return OS_GPIO_OK;
}

uint32_t OS_Gpio_set_value(os_gpio_handle handle, uint32_t value)
{
    return OS_GPIO_OK;
}

uint32_t OS_Gpio_deinit(os_gpio_handle handle)
{
    return OS_GPIO_OK;
}

os_sem_handle OS_Sem_create(int32_t initial_number)
{
    bm_sem_struct_t* sem = NULL;
    sem = (bm_sem_struct_t*)BM_mem_alloc_word_aligned(sizeof(bm_sem_struct_t));
    if (sem == NULL)
    {
        return NULL;
    }
    if (_bm_sem_create(sem, initial_number) != BM_SEM_OK)
    {
        BM_mem_free(sem);
        return NULL;
    }
    return (os_sem_handle)sem;
}

uint32_t OS_Sem_wait(os_sem_handle handle, uint32_t timeout)
{
    bm_sem_struct_t* sem = (bm_sem_struct_t*)handle;
    uint32_t ret;
    uint32_t wait_ticket = timeout * 10;
    
    ret = _bm_sem_wait_ticks(sem, timeout);
    if (BM_SEM_OK == ret)
    {
        return (uint32_t)OS_SEM_OK;
    }
    else if (BM_SEM_INVALID == ret)
    {
        return (uint32_t)OS_SEM_ERROR;
    }

    while (wait_ticket--)
    {
        delay(1);
        ret = _bm_sem_wait_ticks(sem, timeout);
            
        if(BM_SEM_OK == ret)
        {
            return (uint32_t)OS_SEM_OK;
        }
    }
    return (uint32_t)OS_SEM_TIMEOUT;
}

uint32_t OS_Sem_destroy(os_sem_handle handle)
{
    uint32_t result = _bm_sem_destroy((bm_sem_struct_t*)handle);
    if (result == BM_SEM_OK)
    {
        BM_mem_free(handle);
        return (uint32_t)OS_SEM_OK;
    }
    else
    {
        return (uint32_t)OS_SEM_ERROR;
    }
}

#endif



