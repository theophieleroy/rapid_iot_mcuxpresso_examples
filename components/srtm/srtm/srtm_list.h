/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SRTM_LIST_H__
#define __SRTM_LIST_H__

#include <assert.h>

/*!
 * @addtogroup srtm
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief Get SRTM list object structure pointer.
*/
#define SRTM_LIST_OBJ(type, field, list) (type)((uint32_t)list - (uint32_t)(&((type)0)->field))

/**
* @brief SRTM list fields
*/
typedef struct _srtm_list
{
    struct _srtm_list *prev;  /*!< previous list node */
    struct _srtm_list *next;  /*!< next list node */
} srtm_list_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialize SRTM list head.
 *
 * @param list SRTM list head pointer.
 */
static inline void SRTM_List_Init(srtm_list_t *list)
{
    assert(list);

    list->prev = list;
    list->next = list;
}

/*!
 * @brief Check whether SRTM list is empty.
 *
 * @param list SRTM list head pointer.
 * @return TRUE when list is empty, FALSE otherwise.
 */
static inline bool SRTM_List_IsEmpty(srtm_list_t *list)
{
    assert(list);

    return list->next == list;
}

/*!
 * @brief Add list node at list head.
 *
 * @param list SRTM list head pointer.
 * @param node SRTM list node pointer to add.
 */
static inline void SRTM_List_AddHead(srtm_list_t *list, srtm_list_t *node)
{
    assert(list);
    assert(node);

    node->next = list->next;
    node->prev = list;
    list->next->prev = node;
    list->next = node;
}

/*!
 * @brief Add list node at list tail.
 *
 * @param list SRTM list head pointer.
 * @param node SRTM list node pointer to add.
 */
static inline void SRTM_List_AddTail(srtm_list_t *list, srtm_list_t *node)
{
    assert(list);
    assert(node);

    node->prev = list->prev;
    node->next = list;
    list->prev->next = node;
    list->prev = node;
}

/*!
 * @brief Insert list node before another.
 *
 * @param anchor SRTM list anchor node pointer.
 * @param node SRTM list node pointer to insert.
 */
static inline void SRTM_List_InsertBefore(srtm_list_t *anchor, srtm_list_t *node)
{
    SRTM_List_AddTail(anchor, node);
}

/*!
 * @brief Insert list node after another.
 *
 * @param anchor SRTM list anchor node pointer.
 * @param node SRTM list node pointer to insert.
 */
static inline void SRTM_List_InsertAfter(srtm_list_t *anchor, srtm_list_t *node)
{
    SRTM_List_AddHead(anchor, node);
}

/*!
 * @brief Remove list node from list.
 *
 * @param node SRTM list node pointer to remove.
 */
static inline void SRTM_List_Remove(srtm_list_t *node)
{
    assert(node);

    node->prev->next = node->next;
    node->next->prev = node->prev;
    /* clear node */
    SRTM_List_Init(node);
}

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_LIST_H__ */
