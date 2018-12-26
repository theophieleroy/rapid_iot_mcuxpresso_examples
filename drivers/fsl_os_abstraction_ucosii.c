/*!
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* ile uCOSII_Adapter.c
* This is the source file for the OS Abstraction layer for uCOSII.
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
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction_ucosii.h"
#include "fsl_os_abstraction.h"
#include <string.h>
#include "GenericList.h"
#include "fsl_common.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */

#ifdef DEBUG_ASSERT
#define OS_ASSERT(condition) \
    if (!(condition))        \
        while (1)            \
            ;
#else
#define OS_ASSERT(condition) (void)(condition);
#endif

#if (osNumberOfMessageQs || osNumberOfEvents)
#define osObjectAlloc_c 1
#else
#define osObjectAlloc_c 0
#endif
/*! @brief Converts milliseconds to ticks*/
#define MSEC_TO_TICK(msec) (((INT32U)(msec) + 500uL / OS_TICKS_PER_SEC) * OS_TICKS_PER_SEC / 1000uL)
#define TICKS_TO_MSEC(tick) ((tick)*1000uL / OS_TICKS_PER_SEC)

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

typedef struct osEventStruct_tag
{
    uint32_t inUse;
    event_t event;
} osEventStruct_t;

typedef struct osMsgQStruct_tag
{
    uint32_t inUse;
    OS_EVENT *pQueue;
    void *msgQ[osNumberOfMessages];
} osMsgQStruct_t;

typedef struct osObjStruct_tag
{
    uint32_t inUse;
    uint32_t osObj;
} osObjStruct_t;

typedef struct osObjectInfo_tag
{
    void *pHeap;
    uint32_t objectStructSize;
    uint32_t objNo;
} osObjectInfo_t;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if osObjectAlloc_c
static void *osObjectAlloc(const osObjectInfo_t *pOsObjectInfo);
static bool_t osObjectIsAllocated(const osObjectInfo_t *pOsObjectInfo, void *pObjectStruct);
static void osObjectFree(const osObjectInfo_t *pOsObjectInfo, void *pObjectStruct);
#endif
extern void main_task(void const *argument);
void startup_task(void *argument);
static uint32_t wait_timeout_msec_to_tick(uint32_t timeout);
/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
const uint8_t gUseRtos_c = USE_RTOS; // USE_RTOS = 0 for BareMetal and 1 for OS
extern uint32_t SystemCoreClock;
uint32_t gInterruptDisableCount = 0;
/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if gTaskMultipleInstancesManagement_c
list_t threadList;
#endif
#if osNumberOfEvents
osEventStruct_t osEventHeap[osNumberOfEvents];
const osObjectInfo_t osEventInfo = {osEventHeap, sizeof(osEventStruct_t), osNumberOfEvents};
#endif

#if osNumberOfMessageQs
osMsgQStruct_t osMsgQHeap[osNumberOfMessageQs];
const osObjectInfo_t osMsgQInfo = {osMsgQHeap, sizeof(osMsgQStruct_t), osNumberOfMessageQs};
#endif

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*FUNCTION**********************************************************************
 *
 * Function Name : startup_task
 * Description   : Wrapper over main_task..
 *
 *END**************************************************************************/
void startup_task(void *argument)
{
    main_task(argument);
    while (1)
        ;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetId
 * Description   : This function is used to get current active task's handler.
 *
 *END**************************************************************************/
osaTaskId_t OSA_TaskGetId(void)
{
    return (osaTaskId_t)OSTCBCur;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskYield
 * Description   : When a task calls this function, it will give up CPU and put
 * itself to the tail of ready list.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskYield(void)
{
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetPriority
 * Description   : This function returns task's priority by task handler.
 *
 *END**************************************************************************/
osaTaskPriority_t OSA_TaskGetPriority(osaTaskId_t taskId)
{
    task_handler_t handler = (task_handler_t)taskId;
    return (osaTaskPriority_t)PRIORITY_RTOS_TO_OSA(handler->OSTCBPrio);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskSetPriority
 * Description   : This function sets task's priority by task handler.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskSetPriority(osaTaskId_t taskId, osaTaskPriority_t taskPriority)
{
    osaStatus_t status = osaStatus_Error;
    task_handler_t handler = (task_handler_t)taskId;
    if (OS_ERR_NONE == OSTaskChangePrio(handler->OSTCBPrio, PRIORITY_OSA_TO_RTOS(taskPriority)))
    {
        status = osaStatus_Success;
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskCreate
 * Description   : This function is used to create a task and make it ready.
 * Param[in]     :  threadDef  - Definition of the thread.
 *                  task_param - Parameter to pass to the new thread.
 * Return Thread handle of the new thread, or NULL if failed.
  *
 *END**************************************************************************/
osaTaskId_t OSA_TaskCreate(osaThreadDef_t *thread_def, osaTaskParam_t task_param)
{
#if gTaskMultipleInstancesManagement_c
    osaThreadLinkHandle_t threadLinkHandle;
#endif
    task_stack_t *threadStackPtr;
    osaTaskId_t taskId = NULL;
    INT8U err;
    INT8U localPriority;
    uint32_t stackSizeLocal;
#if OS_CRITICAL_METHOD == 3U
    OS_CPU_SR cpu_sr = 0U;
#endif
    if (thread_def == NULL)
    {
        return taskId;
    }
#if !gTaskMultipleInstancesManagement_c
    if (thread_def->instances != 1)
    {
        return taskId;
    }
#endif
    stackSizeLocal = SIZE_IN_UINT32_UNITS(thread_def->stacksize);
    /*
    * To provide unified task piority for upper layer, OSA layer makes conversion.
    * uC/OS-II's highest 4 priorities are reserved for system, so add 4 here.
    */
    localPriority = PRIORITY_OSA_TO_RTOS(thread_def->tpriority);
    threadStackPtr = (task_stack_t *)thread_def->tstack;

#if gTaskMultipleInstancesManagement_c
    threadLinkHandle = thread_def->tlink;
    OS_ENTER_CRITICAL();
    if (thread_def->instances == 0)
    {
        OS_EXIT_CRITICAL();
        return NULL;
    }
    thread_def->instances--;
    while (ListGetList((listElementHandle_t)&threadLinkHandle->link) == &threadList)
    {
        threadLinkHandle += 1; /*Go to next link element*/
        threadStackPtr += stackSizeLocal;
    }
    threadLinkHandle->osThreadId = NULL;
    ListAddTail(&threadList, (listElementHandle_t)&threadLinkHandle->link);
    OS_EXIT_CRITICAL();
#endif

    err = OSTaskCreateExt((task_t)thread_def->pthread,         /* Task pointer*/
                          (task_param_t)task_param,            /* Task input parameter*/
                          &threadStackPtr[stackSizeLocal - 1], /* Pointer to top of stack*/
                          localPriority,                       /* Task priority*/
                          localPriority,                       /* Task id. should be the same as priority*/
                          threadStackPtr,                      /* Pointer to the bottom of the stack*/
                          stackSizeLocal,                      /* Size of the stack*/
                          (void *)0,                           /* TCB extension not supported by this abstraction*/
                          thread_def->useFloat ? OS_TASK_OPT_SAVE_FP : OS_TASK_OPT_NONE);

    if (OS_ERR_NONE != err)
    {
#if gTaskMultipleInstancesManagement_c
        OS_ENTER_CRITICAL();
        thread_def->instances++;
        ListRemoveElement((listElementHandle_t)&threadLinkHandle->link);
        OS_EXIT_CRITICAL();
#endif
        return taskId;
    }
#if OS_TASK_NAME_EN > 0u
    OSTaskNameSet(localPriority, thread_def->tname, &err);
    if (OS_ERR_NONE != err)
    {
        OSTaskDel(localPriority);
#if gTaskMultipleInstancesManagement_c
        OS_ENTER_CRITICAL();
        thread_def->instances++;
        ListRemoveElement((listElementHandle_t)&threadLinkHandle->link);
        OS_EXIT_CRITICAL();
#endif
        return taskId;
    }
#endif
    OS_ENTER_CRITICAL();
    taskId = (osaTaskId_t)OSTCBPrioTbl[localPriority];
    OS_EXIT_CRITICAL();
#if gTaskMultipleInstancesManagement_c
    threadLinkHandle->osThreadDefHandle = thread_def;
    threadLinkHandle->osThreadStackHandle = threadStackPtr;
    threadLinkHandle->osThreadId = taskId;
#endif
    return taskId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskDestroy
 * Description   : This function destroy a task.
 * Param[in]     :taskId - Thread handle.
 * Return osaStatus_Success if the task is destroied, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskDestroy(osaTaskId_t taskId)
{
#if gTaskMultipleInstancesManagement_c
    osaThreadLinkHandle_t threadLinkHandle;
#endif
    task_handler_t handler = (task_handler_t)taskId;
#if OS_CRITICAL_METHOD == 3U
    OS_CPU_SR cpu_sr = 0U;
    (void)cpu_sr;
#endif
#if gTaskMultipleInstancesManagement_c
    threadLinkHandle = (osaThreadLinkHandle_t)ListGetHead(&threadList);
    while (threadLinkHandle->osThreadId != taskId)
    {
        threadLinkHandle =
            (osaThreadLinkHandle_t)ListGetNext((listElementHandle_t)&threadLinkHandle->link); /*Check next thread*/
        if (threadLinkHandle == NULL)
        {
            return osaStatus_Error;
        }
    }
#endif
    if (OS_ERR_NONE != OSTaskDel(handler->OSTCBPrio))
    {
        return osaStatus_Error;
    }
#if gTaskMultipleInstancesManagement_c
    OS_ENTER_CRITICAL();
    ListRemoveElement((listElementHandle_t)&threadLinkHandle->link);
    threadLinkHandle->osThreadDefHandle->instances++;
    OS_EXIT_CRITICAL();
#endif
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeDelay
 * Description   : This function is used to suspend the active thread for the given number of milliseconds.
 *
 *END**************************************************************************/
void OSA_TimeDelay(uint32_t millisec)
{
    uint32_t ticks;
    ticks = MSEC_TO_TICK(millisec);
    /*
    * If delay is too short that changed to 0 tick, reset it to 1 tick
    * in case of no delay.
    */
    if (!ticks)
    {
        ticks = 1U;
    }
    OSTimeDly(ticks);
}
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeGetMsec
 * Description   : This function gets current time in milliseconds.
 *
 *END**************************************************************************/
uint32_t OSA_TimeGetMsec(void)
{
    INT32U timeTick = OSTimeGet();
    return TICKS_TO_MSEC(timeTick);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreCreate
 * Description   : This function is used to create a semaphore.
 * Return         : Semaphore handle of the new semaphore, or NULL if failed.
  *
 *END**************************************************************************/
osaSemaphoreId_t OSA_SemaphoreCreate(uint32_t initValue)
{
#if osNumberOfSemaphores
    semaphore_t sem;
    sem = OSSemCreate(initValue);
    return (osaSemaphoreId_t)sem;
#else
    (void)initValue;
    return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreDestroy
 * Description   : This function is used to destroy a semaphore.
 * Return        : osaStatus_Success if the semaphore is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreDestroy(osaSemaphoreId_t semId)
{
#if osNumberOfSemaphores
    osaStatus_t status;
    INT8U err;
    semaphore_t sem = (semaphore_t)semId;
    OSSemDel(sem, OS_DEL_ALWAYS, &err);
    if (OS_ERR_NONE == err)
    {
        status = osaStatus_Success;
    }
    else
    {
        status = osaStatus_Error;
    }
    return status;
#else
    (void)semId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreWait
 * Description   : This function checks the semaphore's counting value, if it is
 * positive, decreases it and returns osaStatus_Success, otherwise, timeout
 * will be used for wait. The parameter timeout indicates how long should wait
 * in milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will
 * return osaStatus_Timeout immediately if semaphore is not positive.
 * This function returns osaStatus_Success if the semaphore is received, returns
 * osaStatus_Timeout if the semaphore is not received within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreWait(osaSemaphoreId_t semId, uint32_t millisec)
{
#if osNumberOfSemaphores
    osaStatus_t status;
    INT8U err;
    uint32_t ticks;
    semaphore_t sem = (semaphore_t)semId;
    /* If timeout is 0, try to get the semaphore. */
    if (!millisec)
    {
        if (OSSemAccept(sem) > 0)
        {
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Timeout;
        }
    }
    else
    {
        ticks = wait_timeout_msec_to_tick(millisec);

        OSSemPend(sem, ticks, &err);

        if (OS_ERR_NONE == err)
        {
            status = osaStatus_Success;
        }
        else if (OS_ERR_TIMEOUT == err)
        {
            status = osaStatus_Timeout;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)semId;
    (void)millisec;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphorePost
 * Description   : This function is used to wake up one task that wating on the
 * semaphore. If no task is waiting, increase the semaphore. The function returns
 * osaStatus_Success if the semaphre is post successfully, otherwise returns
 * osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphorePost(osaSemaphoreId_t semId)
{
#if osNumberOfSemaphores
    osaStatus_t status;
    semaphore_t sem = (semaphore_t)semId;
    if (OS_ERR_NONE == OSSemPost(sem))
    {
        status = osaStatus_Success;
    }
    else
    {
        status = osaStatus_Error;
    }
    return status;
#else
    (void)semId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexCreate
 * Description   : This function is used to create a mutex.
 * Return        : Mutex handle of the new mutex, or NULL if failed.
 *
 *END**************************************************************************/
osaMutexId_t OSA_MutexCreate(void)
{
#if osNumberOfMutexes
    mutex_t mutex;
    INT8U err;
    /* NOTE: uC/OS-II does not support priority inherit but only priority protect */
    mutex = OSMutexCreate(OS_PRIO_MUTEX_CEIL_DIS, &err);

    if (OS_ERR_NONE != err)
    {
        mutex = NULL;
    }
    return (osaMutexId_t)mutex;
#else
    return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexLock
 * Description   : This function checks the mutex's status, if it is unlocked,
 * lock it and returns osaStatus_Success, otherwise, wait for the mutex.
 * This function returns osaStatus_Success if the mutex is obtained, returns
 * osaStatus_Error if any errors occur during waiting. If the mutex has been
 * locked, pass 0 as timeout will return osaStatus_Timeout immediately.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexLock(osaMutexId_t mutexId, uint32_t millisec)
{
#if osNumberOfMutexes
    osaStatus_t status;
    INT8U err;
    uint32_t ticks;
    mutex_t mutex = (mutex_t)mutexId;

    /* Avoid to get the mutex current task has already got. */
    if (mutex->OSEventPtr == (void *)OSTCBCur)
    {
        status = osaStatus_Error;
    }
    else
    {
        if (!millisec) /* If timeout is 0, try to lock the mutex. */
        {
            BOOLEAN pollResult = OSMutexAccept(mutex, &err);

            if (OS_ERR_NONE == err)
            {
                if (OS_TRUE == pollResult)
                {
                    status = osaStatus_Success;
                }
                else
                {
                    status = osaStatus_Timeout;
                }
            }
            else
            {
                status = osaStatus_Error;
            }
        }
        else
        {
            ticks = wait_timeout_msec_to_tick(millisec);

            OSMutexPend(mutex, ticks, &err);

            if (OS_ERR_NONE == err)
            {
                status = osaStatus_Success;
            }
            else if (OS_ERR_TIMEOUT == err)
            {
                status = osaStatus_Timeout;
            }
            else
            {
                status = osaStatus_Error;
            }
        }
        /* If timeout is not 0, convert it to tickes. */
    }
    return status;
#else
    (void)mutexId;
    (void)millisec;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexUnlock
 * Description   : This function is used to unlock a mutex.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexUnlock(osaMutexId_t mutexId)
{
#if osNumberOfMutexes
    osaStatus_t status;
    mutex_t mutex = (mutex_t)mutexId;
    {
        if (OS_ERR_NONE == OSMutexPost(mutex))
        {
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)mutexId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexDestroy
 * Description   : This function is used to destroy a mutex.
 * Return        : osaStatus_Success if the lock object is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexDestroy(osaMutexId_t mutexId)
{
#if osNumberOfMutexes
    osaStatus_t status;
    mutex_t mutex = (mutex_t)mutexId;
    INT8U err;
    OSMutexDel(mutex, OS_DEL_ALWAYS, &err);

    if (OS_ERR_NONE == err)
    {
        status = osaStatus_Success;
    }
    else
    {
        status = osaStatus_Error;
    }
    return status;
#else
    (void)mutexId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventCreate
 * Description   : This function is used to create a event object.
 * Return        : Event handle of the new event, or NULL if failed.
 *
 *END**************************************************************************/
osaEventId_t OSA_EventCreate(bool_t autoClear)
{
#if osNumberOfEvents
    osaEventId_t eventId;
    osEventStruct_t *pEventStruct;
    INT8U err;
    OSA_InterruptDisable();
    eventId = pEventStruct = osObjectAlloc(&osEventInfo);
    OSA_InterruptEnable();
    if (eventId)
    {
        pEventStruct->event.pGroup = OSFlagCreate(0u, &err);
        if (OS_ERR_NONE == err)
        {
            pEventStruct->event.autoClear = autoClear;
        }
        else
        {
            OSA_InterruptDisable();
            osObjectFree(&osEventInfo, eventId);
            OSA_InterruptEnable();
            eventId = NULL;
        }
    }
    return eventId;
#else
    (void)autoClear;
    return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventSet
 * Description   : Set one or more event flags of an event object.
 * Return        : osaStatus_Success if set successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventSet(osaEventId_t eventId, osaEventFlags_t flagsToSet)
{
#if osNumberOfEvents
    osaStatus_t status;
    osEventStruct_t *pEventStruct;
    INT8U err;
    if (osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pEventStruct = (osEventStruct_t *)eventId;
        OSFlagPost(pEventStruct->event.pGroup, (event_flags_t)flagsToSet, OS_FLAG_SET, &err);

        if (OS_ERR_NONE == err)
        {
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)eventId;
    (void)flagsToSet;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventClear
 * Description   : Clear one or more event flags of an event object.
 * Return        :osaStatus_Success if clear successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventClear(osaEventId_t eventId, osaEventFlags_t flagsToClear)
{
#if osNumberOfEvents
    osaStatus_t status;
    osEventStruct_t *pEventStruct;
    INT8U err;
    if (osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pEventStruct = (osEventStruct_t *)eventId;
        OSFlagPost(pEventStruct->event.pGroup, (event_flags_t)flagsToClear, OS_FLAG_CLR, &err);

        if (OS_ERR_NONE == err)
        {
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)eventId;
    (void)flagsToClear;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventWait
 * Description   : This function checks the event's status, if it meets the wait
 * condition, return osaStatus_Success, otherwise, timeout will be used for
 * wait. The parameter timeout indicates how long should wait in milliseconds.
 * Pass osaWaitForever_c to wait indefinitely, pass 0 will return the value
 * osaStatus_Timeout immediately if wait condition is not met. The event flags
 * will be cleared if the event is auto clear mode. Flags that wakeup waiting
 * task could be obtained from the parameter setFlags.
 * This function returns osaStatus_Success if wait condition is met, returns
 * osaStatus_Timeout if wait condition is not met within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventWait(
    osaEventId_t eventId, osaEventFlags_t flagsToWait, bool_t waitAll, uint32_t millisec, osaEventFlags_t *pSetFlags)
{
#if osNumberOfEvents
    osaStatus_t status;
    osEventStruct_t *pEventStruct;
    INT8U err, opt;
    uint32_t ticks;
    if (osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pEventStruct = (osEventStruct_t *)eventId;
        /* Prepare wait options base on wait type and clear type. */
        if (waitAll)
        {
            opt = OS_FLAG_WAIT_SET_ALL;
        }
        else
        {
            opt = OS_FLAG_WAIT_SET_ANY;
        }

        if (TRUE == pEventStruct->event.autoClear)
        {
            opt |= OS_FLAG_CONSUME;
        }

        /* If timeout is 0, try to wait. */
        if (0U == millisec)
        {
            *pSetFlags =
                (osaEventFlags_t)OSFlagAccept(pEventStruct->event.pGroup, (event_flags_t)flagsToWait, opt, &err);
        }
        else
        {
            /* If timeout is not 0, convert it to tickes. */
            ticks = wait_timeout_msec_to_tick(millisec);
            *pSetFlags =
                (osaEventFlags_t)OSFlagPend(pEventStruct->event.pGroup, (event_flags_t)flagsToWait, opt, ticks, &err);
        }

        if (OS_ERR_NONE == err)
        {
            status = osaStatus_Success;
        }
        else if ((OS_ERR_TIMEOUT == err) || (OS_ERR_FLAG_NOT_RDY == err))
        {
            status = osaStatus_Timeout;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)eventId;
    (void)flagsToWait;
    (void)waitAll;
    (void)millisec;
    (void)pSetFlags;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventDestroy
 * Description   : This function is used to destroy a event object. Return
 * osaStatus_Success if the event object is destroyed successfully, otherwise
 * return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventDestroy(osaEventId_t eventId)
{
#if osNumberOfEvents
    osaStatus_t status;
    osEventStruct_t *pEventStruct;
    INT8U err;
    if (osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pEventStruct = (osEventStruct_t *)eventId;
        OSFlagDel(pEventStruct->event.pGroup, OS_DEL_ALWAYS, &err);
        if (OS_ERR_NONE == err)
        {
            pEventStruct->event.pGroup = (OS_FLAG_GRP *)0;
            status = osaStatus_Success;
            OSA_InterruptDisable();
            osObjectFree(&osEventInfo, eventId);
            OSA_InterruptEnable();
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)eventId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQCreate
 * Description   : This function is used to create a message queue.
 * Return        : the handle to the message queue if create successfully, otherwise
 * return NULL.
 *
 *END**************************************************************************/
osaMsgQId_t OSA_MsgQCreate(uint32_t msgNo)
{
#if osNumberOfMessageQs
    osaMsgQId_t msgQId = NULL;
    osMsgQStruct_t *pMsgQStruct;
    if (msgNo <= osNumberOfMessages)
    {
        OSA_InterruptDisable();
        msgQId = pMsgQStruct = osObjectAlloc(&osMsgQInfo);
        OSA_InterruptEnable();
        if (msgQId)
        {
            pMsgQStruct->pQueue = OSQCreate(pMsgQStruct->msgQ, msgNo);
            if (!pMsgQStruct->pQueue)
            {
                OSA_InterruptDisable();
                osObjectFree(&osMsgQInfo, msgQId);
                OSA_InterruptEnable();
                msgQId = NULL;
            }
        }
    }
    return msgQId;
#else
    (void)msgNo;
    return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQPut
 * Description   : This function is used to put a message to a message queue.
* Return         : osaStatus_Success if the message is put successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQPut(osaMsgQId_t msgQId, void *pMessage)
{
#if osNumberOfMessageQs
    osaStatus_t status;
    osMsgQStruct_t *pMsgQStruct;
    osaMsg_t msg;
    if (osObjectIsAllocated(&osMsgQInfo, msgQId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pMsgQStruct = (osMsgQStruct_t *)msgQId;
        msg = *((osaMsg_t *)pMessage);

        if (OS_ERR_NONE == OSQPost(pMsgQStruct->pQueue, msg))
        {
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)msgQId;
    (void)pMessage;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQGet
 * Description   : This function checks the queue's status, if it is not empty,
 * get message from it and return osaStatus_Success, otherwise, timeout will
 * be used for wait. The parameter timeout indicates how long should wait in
 * milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will return
 * osaStatus_Timeout immediately if queue is empty.
 * This function returns osaStatus_Success if message is got successfully,
 * returns osaStatus_Timeout if message queue is empty within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQGet(osaMsgQId_t msgQId, void *pMessage, uint32_t millisec)
{
#if osNumberOfMessageQs
    osaStatus_t status;
    osMsgQStruct_t *pMsgQStruct;
    uint32_t ticks;
    osaMsg_t *pMsg;
    INT8U err;
    if (osObjectIsAllocated(&osMsgQInfo, msgQId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pMsgQStruct = (osMsgQStruct_t *)msgQId;
        pMsg = (osaMsg_t *)pMessage;
        if (0U != millisec)
        {
            /* If timeout is not 0, convert it to tickes. */
            ticks = wait_timeout_msec_to_tick(millisec);
            *pMsg = OSQPend(pMsgQStruct->pQueue, ticks, &err);
        }
        else
        {
            *pMsg = OSQAccept(pMsgQStruct->pQueue, &err);
        }

        if (OS_ERR_NONE == err)
        {
            status = osaStatus_Success;
        }
        else if ((OS_ERR_Q_EMPTY == err) || (OS_ERR_TIMEOUT == err))
        {
            status = osaStatus_Timeout;
        }
        else
        {
            status = osaStatus_Error;
        }
    }

    return status;
#else
    (void)msgQId;
    (void)pMessage;
    (void)millisec;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQDestroy
 * Description   : This function is used to destroy the message queue.
 * Return        : osaStatus_Success if the message queue is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQDestroy(osaMsgQId_t msgQId)
{
#if osNumberOfMessageQs
    osaStatus_t status;
    osMsgQStruct_t *pMsgQStruct;
    INT8U err;
    if (osObjectIsAllocated(&osMsgQInfo, msgQId) == FALSE)
    {
        status = osaStatus_Error;
    }
    else
    {
        pMsgQStruct = (osMsgQStruct_t *)msgQId;
        OSQDel(pMsgQStruct->pQueue, OS_DEL_ALWAYS, &err);

        if (OS_ERR_NONE == err)
        {
            OSA_InterruptDisable();
            osObjectFree(&osMsgQInfo, msgQId);
            OSA_InterruptEnable();
            status = osaStatus_Success;
        }
        else
        {
            status = osaStatus_Error;
        }
    }
    return status;
#else
    (void)msgQId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptEnable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptEnable(void)
{
    OS_CPU_SR cpu_sr = 0u;
    if (gInterruptDisableCount > 0)
    {
        gInterruptDisableCount--;

        if (gInterruptDisableCount == 0)
        {
            OS_EXIT_CRITICAL();
        }
        /* call core API to enable the global interrupt*/
    }
    (void)cpu_sr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptDisable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptDisable(void)
{
    OS_CPU_SR cpu_sr = 0u;
    OS_ENTER_CRITICAL();
    gInterruptDisableCount++;
    (void)cpu_sr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InstallIntHandler
 * Description   : This function is used to install interrupt handler.
 *
 *END**************************************************************************/
void OSA_InstallIntHandler(uint32_t IRQNumber, void (*handler)(void))
{
#if defined(__IAR_SYSTEMS_ICC__)
    _Pragma("diag_suppress = Pm138")
#endif
        InstallIRQHandler((IRQn_Type)IRQNumber, (uint32_t)handler);
#if defined(__IAR_SYSTEMS_ICC__)
    _Pragma("diag_remark = PM138")
#endif
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */

OSA_TASK_DEFINE(startup_task, gMainThreadPriority_c, 1, gMainThreadStackSize_c, FALSE);
int main(void)
{
    extern void hardware_init(void);
#if gTaskMultipleInstancesManagement_c
    ListInit(&threadList, 0);
#endif
    OSInit();
    /* Initialize MCU clock */
    hardware_init();
    OS_CPU_SysTickInit(SystemCoreClock / (uint32_t)OS_TICKS_PER_SEC);
    OSA_TaskCreate(OSA_TASK(startup_task), NULL);
    OSStart();
    return 0;
}

/*! *********************************************************************************
* \brief     Allocates a osObjectStruct_t block in the osObjectHeap array.
* \param[in] pointer to the object info struct.
* Object can be semaphore, mutex, message Queue, event
* \return Pointer to the allocated osObjectStruct_t, NULL if failed.
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
#if osObjectAlloc_c
static void *osObjectAlloc(const osObjectInfo_t *pOsObjectInfo)
{
    uint32_t i;
    uint8_t *pObj = (uint8_t *)pOsObjectInfo->pHeap;
    for (i = 0; i < pOsObjectInfo->objNo; i++, pObj += pOsObjectInfo->objectStructSize)
    {
        if (((osObjStruct_t *)pObj)->inUse == 0)
        {
            ((osObjStruct_t *)pObj)->inUse = 1;
            return (void *)pObj;
        }
    }
    return NULL;
}
#endif

/*! *********************************************************************************
* \brief     Verifies the object is valid and allocated in the osObjectHeap array.
* \param[in] the pointer to the object info struct.
* \param[in] the pointer to the object struct.
* Object can be semaphore, mutex, message Queue, event
* \return TRUE if the object is valid and allocated, FALSE otherwise
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
#if osObjectAlloc_c
static bool_t osObjectIsAllocated(const osObjectInfo_t *pOsObjectInfo, void *pObjectStruct)
{
    uint32_t i;
    uint8_t *pObj = (uint8_t *)pOsObjectInfo->pHeap;
    for (i = 0; i < pOsObjectInfo->objNo; i++, pObj += pOsObjectInfo->objectStructSize)
    {
        if (pObj == pObjectStruct)
        {
            if (((osObjStruct_t *)pObj)->inUse)
            {
                return TRUE;
            }
            break;
        }
    }
    return FALSE;
}
#endif

/*! *********************************************************************************
* \brief     Frees an osObjectStruct_t block from the osObjectHeap array.
* \param[in] pointer to the object info struct.
* \param[in] Pointer to the allocated osObjectStruct_t to free.
* Object can be semaphore, mutex, message Queue, event
* \return none.
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
#if osObjectAlloc_c
static void osObjectFree(const osObjectInfo_t *pOsObjectInfo, void *pObjectStruct)
{
    uint32_t i;
    uint8_t *pObj = (uint8_t *)pOsObjectInfo->pHeap;
    for (i = 0; i < pOsObjectInfo->objNo; i++, pObj += pOsObjectInfo->objectStructSize)
    {
        if (pObj == pObjectStruct)
        {
            ((osObjStruct_t *)pObj)->inUse = 0;
            break;
        }
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : wait_timeout_msec_to_tick
 * Description   : This function converts timeout from millisecond to tick for
 * wait functions.
 *
 *END**************************************************************************/
static uint32_t wait_timeout_msec_to_tick(uint32_t timeout)
{
    uint32_t ticks;
    if (osaWaitForever_c == timeout) /* Wait forever. */
    {
        return 0U; /* Timeout 0 means wait forever for uC/OS-II. */
    }
    else
    {
        ticks = MSEC_TO_TICK(timeout); /* Change timeout to tick. */
                                       /*
                                       * If timeout is too short that changed to 0 tick, reset it to 1 tick
                                       * in case of infinitely wait.
                                       */
        if (!ticks)
        {
            ticks = 1U;
        }
        return ticks;
    }
}
