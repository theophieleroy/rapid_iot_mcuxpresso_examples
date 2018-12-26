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
* $FileName: mem_util.c$
* $Version : 
* $Date    : 
*
* Comments:
*
*
*
*END************************************************************************/
#include "adapter_bm.h"

#define  BM_MEM4_ALIGN(n)          ((n) + (-(n) & 3))

void* BM_mem_alloc_word_aligned(uint32_t size)
{
    void *p = (void*)malloc(size + 4);
    return((void*)(BM_MEM4_ALIGN((uint32_t)p)));
}
void* BM_mem_alloc_zero(uint32_t size)
{
    void* p;
    p = BM_mem_alloc_word_aligned(size);
    if (p != NULL)
    {
        memset( (p) , (0) , (size));
    }
    return p;
}

void BM_mem_free(void* buffer)
{
    free(buffer);
    return ;
}
