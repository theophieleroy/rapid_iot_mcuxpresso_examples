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
* $FileName: bmevent.h$
* $Version :
* $Date    :
*
* Comments:
*
*
*
*END************************************************************************/
#ifndef __usb_event_h__
#define __usb_event_h__
#include "bootloader_common.h"

/*--------------------------------------------------------------------------*/
/*
**                            MACRO DEFINITIONS
*/

#define BM_EVENT_OK 0x00
#define BM_EVENT_ERROR 0x01
#define BM_EVENT_SET 0x02
#define BM_EVENT_NOT_SET 0x03
#define BM_EVENT_VALID 0x04
#define BM_EVENT_INVALID 0x05

/*--------------------------------------------------------------------------*/
/*
**                            DATATYPE DECLARATIONS
*/
#if defined __CC_ARM
#pragma push
#pragma pack(1)
#endif
#ifdef __GNUC__
/* << EST pushing options */
#pragma pack(push)
#pragma pack(1)
#endif
typedef struct bm_event
{
    uint8_t valid;
    //    bool     valid;
    uint32_t value;
} bm_event_struct_t;

#ifdef __CC_ARM
#pragma pop
#endif
#ifdef __GNUC__
/* << EST restoring previous packing */
#pragma pack(pop)
#endif
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

extern uint16_t _bm_event_init(bm_event_struct_t *);
extern uint16_t _bm_event_clear(bm_event_struct_t *, uint32_t);
extern uint16_t _bm_event_set(bm_event_struct_t *, uint32_t);
extern uint16_t _bm_event_wait_ticks(bm_event_struct_t *, uint32_t, uint8_t, uint16_t);
extern void sys_lock();
extern void sys_unlock();

#ifdef __cplusplus
}
#endif

#endif
/* EOF */
