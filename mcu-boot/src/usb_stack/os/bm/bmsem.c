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
* $FileName: bmsem.c$
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
#include "bmsem.h"

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_sem_create
* Returned Value   :
* Comments         : Create a semaphore pointer, set valid field to BM_SEM_VALID and VAULE to initial_number
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_sem_create
(
    bm_sem_struct_t*      sem_ptr,
    int32_t               initial_number
)
{
    sem_ptr->valid = BM_SEM_VALID;
    sem_ptr->value = (uint32_t)initial_number;
    return  BM_SEM_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_sem_post
* Returned Value   : 
* Comments         : Increate VALUE field of semaphore pointer.
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_sem_post
(
    bm_sem_struct_t*    sem_ptr
)
{
    if(sem_ptr->valid == BM_SEM_VALID) 
    {
        sem_ptr->value++;
        return BM_SEM_OK;
    }
    return BM_SEM_INVALID;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_sem_wait
* Returned Value   : 
* Comments         : Wait for a lightweight semaphore.
* 
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_sem_wait
(
    bm_sem_struct_t* sem_ptr
)
{
    if(sem_ptr->valid == BM_SEM_VALID) 
    {
        sem_ptr->value--;
        return BM_SEM_OK;
    }
    return BM_SEM_INVALID;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_sem_wait_ticks
* Returned Value   : 
* Comments         :
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_sem_wait_ticks
(
    bm_sem_struct_t* sem_ptr,
    uint32_t ticks
)
{
    UNUSED(ticks)

    if(sem_ptr->valid == BM_SEM_VALID) 
    {
        if(sem_ptr->value <= 0)
        {
            return BM_SEM_FREE;
        }
        else 
        {
            sem_ptr->value--;
            return BM_SEM_OK;
        }
    }
    return BM_SEM_INVALID;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bm_sem_wait_ticks
* Returned Value   : 
* Comments         : Set valid field of semaphore pointer to BM_SEM_INVALID
*    
*
*END*----------------------------------------------------------------------*/
uint16_t _bm_sem_destroy
(
    bm_sem_struct_t* sem_ptr
)
{
    if(sem_ptr->valid == BM_SEM_VALID) 
    {
        sem_ptr->value--;
        return (uint16_t)BM_SEM_OK;
    }
    return (uint16_t)BM_SEM_INVALID;
}
