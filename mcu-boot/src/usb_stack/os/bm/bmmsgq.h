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
* $FileName: bmmsgq.h$
* $Version :
* $Date    :
*
* Comments:
*
*   This file containts definitions for use with light weight message queues
*
*END************************************************************************/
#ifndef __bm_msq_h__
#define __bm_msq_h__
#include "types.h"
#include "bootloader_common.h"
/*--------------------------------------------------------------------------*/
/*
**                            MACRO DEFINITIONS
*/

/* control bits for lwmsgq_send */

/* Error messages */
#define BM_MSGQ_OK 0x00
#define BM_MSGQ_SEND_ERROR 0x01
#define BM_MSGQ_RECEIVE_ERROR 0x02
#define BM_MSGQ_INIT_ERROR 0x03

/* QUEUE status */
#define BM_MSGQ_VALID 0x00
#define BM_MSGQ_FULL 0x01
#define BM_MSGQ_EMPTY 0x02
#define BM_MSGQ_TIMEOUT 0x03

//#define BM_MSGQ_MAX_ELEMENT_NUMBER           10
/*--------------------------------------------------------------------------*/
/*
**                            DATATYPE DECLARATIONS
*/

/*---------------------------------------------------------------------
**
** LWMSGQ STRUCTURE
**
** This structure used to store a circular long word queue.
** The structure must be the LAST if it is included into another
** data structure, as the queue falls off of the end of
** this structure.
*/
typedef void *bm_msgq_handle;

typedef struct bm_msgq_element
{
    void *message_data;
    struct bm_msgq_element *next;
    struct bm_msgq_element *prev;
} bm_msgq_element_t;

typedef struct bm_msgq_struct
{
    /* A Queue of task descriptors waiting to write */
    bm_msgq_element_t *head;

    /* A Queue of task descriptors waiting to read */
    bm_msgq_element_t *tail;

    /* The validity check field */
    uint32_t queue_status;

    /* The size of the message chunk in the queue in _mqx_max_type's */
    uint32_t size;

    /* The maximum number of msgs for the queue, as specified in
     * initialization of the queue.
     */
    uint32_t max_size;

    /* The current number of messages in the queue. */
    uint32_t current_size;

    /* The current index of the message element */
    uint32_t index;

    /* The array to hold the message element in the queue. */
    bm_msgq_element_t *element_array;

} bm_msgq_struct_t;

#define BMMSGQ_IS_EMPTY(lwq) (((bm_msgq_struct_t *)(lwq))->current_size == 0)

/*--------------------------------------------------------------------------*/
/*
** FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif

bm_msgq_handle _bm_msgq_init(uint32_t, uint32_t);
uint32_t _bm_msgq_send(bm_msgq_handle, int32_t *);
uint32_t _bm_msgq_receive(bm_msgq_handle, int32_t *);
uint32_t _bm_msgq_deinit(bm_msgq_handle);

#ifdef __cplusplus
}
#endif

#endif
