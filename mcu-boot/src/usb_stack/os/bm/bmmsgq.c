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
* $FileName: bmmsgq.c$
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
#include "bmmsgq.h"
#include "adapter_bm.h"

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_msgq_init
* Returned Value   :
* Comments         : Initialize message queue. Allocate message pointer. Make queue in QUEUE_EMPTY status.
*    
*
*END*----------------------------------------------------------------------*/
bm_msgq_handle _bm_msgq_init
(
    /* Number of messages created after the header */
    uint32_t num_messages,

    /* The size of the messages in _mqx_max_type's */
    uint32_t msg_size
)
{
    uint32_t i;
    bm_msgq_struct_t* p_msgq = NULL;

    p_msgq = (bm_msgq_struct_t*)malloc(sizeof(bm_msgq_struct_t));
    if (p_msgq == NULL)
    {
        return NULL;
    }
    
    p_msgq->size = msg_size;
    p_msgq->max_size = num_messages;
    p_msgq->element_array = (bm_msgq_element_t*)malloc(num_messages * sizeof (bm_msgq_element_t));
    if (p_msgq->element_array == NULL)
    {
        _bm_msgq_deinit(p_msgq);
        return NULL;
    }
    
    for (i = 0; i < num_messages; i++)
    {
        p_msgq->element_array[i].message_data = (void*)malloc(msg_size*sizeof(uint32_t));
        if (p_msgq->element_array[i].message_data == NULL)
        {
            _bm_msgq_deinit(p_msgq);
            return NULL;
        }
        
    }
    /* Set head pointer and tail pointer are NULL */
    p_msgq->head = NULL;
    p_msgq->tail = NULL;

    /* set information for queue */
    p_msgq->queue_status = BM_MSGQ_EMPTY;
    p_msgq->current_size = 0;
    p_msgq->index = 0;

    return (bm_msgq_handle)p_msgq;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_msgq_send
* Returned Value   :
* Comments         : Send a message to the message pool.
*    
*
*END*----------------------------------------------------------------------*/
uint32_t _bm_msgq_send
  (
      bm_msgq_handle handle,
      /* Number of messages created after the header */
      int32_t*   message
  )
{
    bm_msgq_struct_t*  p_msgq = (bm_msgq_struct_t*)handle;
    bm_msgq_element_t* new_element_ptr;
 
    int32_t *to_ptr;
    int32_t *from_ptr;
    int32_t i;
  
    if (p_msgq == NULL)
    {
        return BM_MSGQ_SEND_ERROR;
    }
   /* check status of queue */
    if (BM_MSGQ_FULL == p_msgq->queue_status)
    {
        return BM_MSGQ_SEND_ERROR;
    }
 
    p_msgq->index++;
    if (p_msgq->max_size == p_msgq->index) 
    {
        p_msgq->index = 0;
    }
    new_element_ptr = &p_msgq->element_array[p_msgq->index];
  
    /* copy data from message to new element */
    to_ptr = (int32_t *)(new_element_ptr->message_data);
    from_ptr = message;
    /* copy data */
    i = (int32_t)p_msgq->size+1;
    while (--i) 
    {
       *to_ptr++ = *from_ptr++;
    } /* Endwhile */

    /* Link new element to link list */
    /* Check queue status */
    if (BM_MSGQ_EMPTY == p_msgq->queue_status){ /* this is the first element */
    /*
        head   new_element
       ___________________
       |NULL|   1   |NULL|
       |____|_______|____|

        tail          
     */
        new_element_ptr->next = NULL;
        new_element_ptr->prev = NULL;
        p_msgq->head = new_element_ptr; 
        p_msgq->tail = new_element_ptr;
    } 
    else 
    { /* this is NOT the first element */
    
        new_element_ptr->next = NULL;
    /*
                                        new_element
       ___________________      ___________________
    -->|n-1 |   n   |NULL|      |  n |  n+1  |NULL|
    <--|____|_______|____|      |____|_______|____|
        tail                                          */
        
        new_element_ptr->prev = p_msgq->tail;
    /*
                                        new_element
       ___________________      ___________________
    -->|n-1 |   n   |NULL|      |  n |  n+1  |NULL|
    <--|____|_______|____|<-----|____|_______|____|
        tail                                          */
                                            
        p_msgq->tail->next = new_element_ptr;
    /*                                  new_element
       ___________________      ___________________
    -->|n-1 |   n   |n+1 |----->|  n |  n+1  |NULL|
    <--|____|_______|____|<-----|____|_______|____|
        tail
                                                  */
        p_msgq->tail = new_element_ptr;
    /*                                  new_element
       ___________________      ___________________
    -->|n-1 |   n   |n+1 |----->|  n |  n+1  |NULL|
    <--|____|_______|____|<-----|____|_______|____|
                                 tail
                                                  */
    }
  
    /* update new current size */
    p_msgq->current_size++;
  
    /* Set status of queue after send an element */
    if (p_msgq->current_size == p_msgq->max_size)
    {
        p_msgq->queue_status = BM_MSGQ_FULL;
    }
    else
    {
        p_msgq->queue_status = BM_MSGQ_VALID;
    }

    return BM_MSGQ_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_msgq_receive
* Returned Value   :
* Comments         : Receive a message from the message pool.
*    
*
*END*----------------------------------------------------------------------*/
uint32_t _bm_msgq_receive
  (
      bm_msgq_handle handle,
      /* Number of messages created after the header */
      int32_t*   message
  )
{
    bm_msgq_struct_t* p_msgq = (bm_msgq_struct_t*)handle;
    bm_msgq_element_t* receive_element_ptr;
    int32_t *to_ptr;
    int32_t *from_ptr;
    int32_t i;
 
    if (p_msgq == NULL)
    {
        return BM_MSGQ_RECEIVE_ERROR;
    }
    /* check status of queue */
    if (BM_MSGQ_EMPTY == p_msgq->queue_status)
    {
        return BM_MSGQ_RECEIVE_ERROR;
    }
  
    /* Get receive element in the link list (it is the first element) */
    receive_element_ptr = p_msgq->head;
  
    /* Copy data from receive element to message */
    /* Set address of source and destination */
    to_ptr = message;
    from_ptr = (int32_t *)(receive_element_ptr->message_data);
    /* copy data */
    i = (int32_t)(p_msgq->size+1);
    while (--i) 
    {
        *to_ptr++ = *from_ptr++;
    } /* Endwhile */
  
    /* Remove receive element from link list */
    /* Check queue status */
    if (1 == p_msgq->current_size)
    { /* there are only 1 element in queue */
      /*
        head   
       ___________________
       |NULL|   1   |NULL|
       |____|_______|____|

        tail          
     */
        p_msgq->head = NULL;
        p_msgq->tail = NULL;
    
    } 
    else 
    {
    /*
        head receive_element            
       ___________________      ___________________
       |NULL|   1   | 2  |----->|  1 |   2   | 3  |--->
       |____|_______|____|<-----|____|_______|____|<---

     */
        p_msgq->head = receive_element_ptr->next; 
    /*
       receive_element           head   
       ___________________      ___________________
       |NULL|   1   | 2  |----->|  1 |   2   | 3  |--->
       |____|_______|____|<-----|____|_______|____|<---

     */
        p_msgq->head->prev = NULL;
    /*
       receive_element           head   
       ___________________      ___________________
       |NULL|   1   | 2  |----->|NULL|   2   | 3  |--->
       |____|_______|____|<--x--|____|_______|____|<---

     */
    }
  
    /* Free receive element */
    p_msgq->current_size--;
    /* update index of last element */
    if (0 == p_msgq->current_size)
    {
        p_msgq->queue_status = BM_MSGQ_EMPTY;
    }
    else 
    {
        p_msgq->queue_status = BM_MSGQ_VALID;
    }
 
    return BM_MSGQ_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_msgq_deinit
* Returned Value   :
* Comments         : deinit the message pool.
*    
*
*END*----------------------------------------------------------------------*/
uint32_t _bm_msgq_deinit(bm_msgq_handle handle)
{
    uint32_t i = 0;
    bm_msgq_struct_t* p_msgq = (bm_msgq_struct_t*)handle;
    if (p_msgq == NULL)
        return BM_MSGQ_INIT_ERROR;

    if (p_msgq->element_array != NULL)
    {
        for (i = 0; i < p_msgq->max_size; i++)
        {
            if (p_msgq->element_array[i].message_data != NULL)
            {
                free(p_msgq->element_array[i].message_data);
                p_msgq->element_array[i].message_data = NULL;
            }
        }    
    }

    free(p_msgq->element_array);
    p_msgq->element_array = NULL;
    free(p_msgq);
    p_msgq = NULL;
    return (uint32_t)BM_MSGQ_OK;
}
