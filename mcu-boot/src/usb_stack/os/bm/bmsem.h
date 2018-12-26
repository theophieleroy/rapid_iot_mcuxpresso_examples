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
* $FileName: bmsem.h$
* $Version :
* $Date    :
*
* Comments:
*
*   This file containts definitions for use with light weight message queues
*
*END************************************************************************/
#ifndef __bm_sem_h__
#define __bm_sem_h__
#include "types.h"
//#include "khci.h"
#include "bootloader_common.h"

/*--------------------------------------------------------------------------*/
/*
**                            MACRO DEFINITIONS
*/

#define BM_SEM_OK 0x00
#define BM_SEM_ERROR 0x01
#define BM_SEM_FREE 0x02
#define BM_SEM_VALID 0x04
#define BM_SEM_INVALID 0x05

/*--------------------------------------------------------------------------*/
/*
**                            DATATYPE DECLARATIONS
*/

typedef struct bm_sem
{
    uint8_t valid;
    //    bool        valid;
    uint32_t value;
} bm_sem_struct_t;
/*---------------------------------------------------------------------
**
**
*/

/*--------------------------------------------------------------------------*/
/*
** FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t _bm_sem_create(bm_sem_struct_t *, int32_t);
extern uint16_t _bm_sem_post(bm_sem_struct_t *);
extern uint16_t _bm_sem_wait(bm_sem_struct_t *);
extern uint16_t _bm_sem_wait_ticks(bm_sem_struct_t *, uint32_t);
extern uint16_t _bm_sem_destroy(bm_sem_struct_t *);

#ifdef __cplusplus
}
#endif

#endif
