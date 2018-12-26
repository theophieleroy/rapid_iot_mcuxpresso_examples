/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "flexcan/fsl_flexcan_driver.h"
#include "bootloader_common.h"

#if BL_CONFIG_CAN

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//! Place to store application callbacks for each of the FLEXCAN modules.
static flexcan_info_t s_applicationInfo[2] = {{0}};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to runtime state structure.*/
void *g_flexcanStatePtr[CAN_INSTANCE_COUNT] = {NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetBitrate
 * Description   : Set FlexCAN baudrate.
 * This function will set up all the time segment values. Those time segment
 * values are passed in by the user and are based on the required baudrate.
 *
 *END**************************************************************************/
void FLEXCAN_DRV_SetBitrate(uint8_t instance, const flexcan_time_segment_t *bitrate)
{
    assert(instance < CAN_INSTANCE_COUNT);

    /* Set time segments*/
    FLEXCAN_HAL_SetTimeSegments((CAN_Type *)g_flexcanBaseAddr[instance], bitrate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_GetBitrate
 * Description   : Get FlexCAN baudrate.
 * This function will be return the current bit rate settings
 *
 *END**************************************************************************/
void FLEXCAN_DRV_GetBitrate(uint8_t instance, flexcan_time_segment_t *bitrate)
{
    assert(instance < CAN_INSTANCE_COUNT);

    /* Get the time segments*/
    FLEXCAN_HAL_GetTimeSegments((CAN_Type *)g_flexcanBaseAddr[instance], bitrate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetMasktype
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 *END**************************************************************************/
void FLEXCAN_DRV_SetMaskType(uint8_t instance, flexcan_rx_mask_type_t type)
{
    assert(instance < CAN_INSTANCE_COUNT);

    FLEXCAN_HAL_SetMaskType((CAN_Type *)g_flexcanBaseAddr[instance], type);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxFifoGlobalMask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_DRV_SetRxFifoGlobalMask(uint8_t instance, flexcan_mb_id_type_t id_type, uint32_t mask)
{
    assert(instance < CAN_INSTANCE_COUNT);

    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    if (id_type == kFlexCanMbId_Std)
    {
        /* Set standard global mask for RX FIOF*/
        FLEXCAN_HAL_SetRxFifoGlobalStdMask(canBaseAddr, mask);
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended global mask for RX FIFO*/
        FLEXCAN_HAL_SetRxFifoGlobalExtMask(canBaseAddr, mask);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxMbGlobalMask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask
 * or the 29-bit extended mask.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetRxMbGlobalMask(uint8_t instance, flexcan_mb_id_type_t id_type, uint32_t mask)
{
    assert(instance < CAN_INSTANCE_COUNT);

    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
#if FLEXCAN_USE_EXT_ID
    if (id_type == kFlexCanMbId_Std)
    {
#endif
        /* Set standard global mask for RX MB*/
        FLEXCAN_HAL_SetRxMbGlobalStdMask(canBaseAddr, mask);
#if FLEXCAN_USE_EXT_ID
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended global mask for RX MB*/
        FLEXCAN_HAL_SetRxMbGlobalExtMask(canBaseAddr, mask);
    }
    else
    {
        return kStatus_FLEXCAN_InvalidArgument;
    }
#endif
    return kStatus_FLEXCAN_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxIndividualMask
 * Description   : Set Rx individual mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_DRV_SetRxIndividualMask(uint8_t instance, flexcan_mb_id_type_t id_type, uint32_t mb_idx, uint32_t mask)
{
    assert(instance < CAN_INSTANCE_COUNT);

    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    if (id_type == kFlexCanMbId_Std)
    {
        /* Set standard individual mask*/
        FLEXCAN_HAL_SetRxIndividualStdMask(canBaseAddr, mb_idx, mask);
    }
    else if (id_type == kFlexCanMbId_Ext)
    {
        /* Set extended individual mask*/
        FLEXCAN_HAL_SetRxIndividualExtMask(canBaseAddr, mb_idx, mask);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Init
 * Description   : Initialize FlexCAN driver.
 * This function will select a source clock, reset FlexCAN module, set maximum
 * number of message buffers, initialize all message buffers as inactive, enable
 * RX FIFO if needed, mask all mask bits, disable all MB interrupts, enable
 * FlexCAN normal mode, and enable all the error interrupts if needed.
 *
 *END**************************************************************************/
void FLEXCAN_DRV_Init(uint8_t instance,
                      const flexcan_user_config_t *data,
                      bool enable_err_interrupts,
                      flexcan_state_t *state,
                      flexcan_info_t *appInfo,
                      flexcan_clk_source_t clk_source)
{
    assert(instance < CAN_INSTANCE_COUNT);
    assert(state);

    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    s_applicationInfo[instance] = *appInfo;

/* Enable clock gate to FlexCAN module */
#if defined(PKE18F15_SERIES)
    PCC_WR_CLKCFG_CGC(PCC_FLEXCAN0_INDEX, 1);
#else
    SIM_SET_SCGC6(SIM, SIM_SCGC6_FLEXCAN0_MASK);
#endif

    /* Select a source clock for FlexCAN*/
    FLEXCAN_HAL_SelectClock(canBaseAddr, clk_source);

    /* Enable the CAN clock */
    FLEXCAN_HAL_Enable(canBaseAddr);

    /* Initialize FLEXCAN device */
    FLEXCAN_HAL_Init(canBaseAddr);

    FLEXCAN_HAL_SetMaxMbNumber(canBaseAddr, data->max_num_mb);

#if FLEXCAN_USE_FIFO
    if (data->is_rx_fifo_needed)
    {
        FLEXCAN_HAL_EnableRxFifo(canBaseAddr, data->num_id_filters, data->max_num_mb);
    }
#endif

    /* Select mode */
    FLEXCAN_HAL_EnableOperationMode(canBaseAddr, kFlexCanNormalMode);

    /* Init the interrupt sync object.*/
    OSA_SemaCreate(&state->txIrqSync, 0);
    OSA_SemaCreate(&state->rxIrqSync, 0);

    /* Enable FlexCAN interrupts.*/
    //    NVIC_EnableIRQ(g_flexcanWakeUpIrqId[instance]);
    NVIC_EnableIRQ(g_flexcanErrorIrqId[instance]);
    NVIC_EnableIRQ(g_flexcanBusOffIrqId[instance]);
    NVIC_EnableIRQ(g_flexcanOredMessageBufferIrqId[instance]);

    /* Enable error interrupts */
    if (enable_err_interrupts)
    {
        FLEXCAN_HAL_EnableErrInt(canBaseAddr);
    }

    state->fifo_message = NULL;
    state->rx_mb_idx = 0;

    /* Save runtime structure pointers so irq handler can point to the correct state structure */
    g_flexcanStatePtr[instance] = state;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigTxMb
 * Description   : Configure a Tx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Tx buffer as INACTIVE, and enable the
 * Message Buffer interrupt.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigTxMb(uint8_t instance,
                                        uint32_t mb_idx,
                                        flexcan_data_info_t *tx_info,
                                        uint32_t msg_id)
{
    assert(instance < CAN_INSTANCE_COUNT);

    flexcan_mb_code_status_t cs;
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    flexcan_state_t *state = g_flexcanStatePtr[instance];

    state->tx_mb_idx = mb_idx;
    /* Initialize transmit mb*/
    cs.data_length = tx_info->data_length;
    cs.msg_id_type = tx_info->msg_id_type;
    cs.code = kFlexCanTX_Inactive;
    return FLEXCAN_HAL_SetMbTx(canBaseAddr, mb_idx, &cs, msg_id, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Send
 * Description   : Set up FlexCAN Message buffer for transmitting data.
 * This function will set the MB CODE field as DATA for Tx buffer. Then this
 * function will copy user's buffer into the message buffer data area, and wait
 * for the Message Buffer interrupt.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_Send(uint8_t instance,
                                  uint32_t mb_idx,
                                  flexcan_data_info_t *tx_info,
                                  uint32_t msg_id,
                                  uint8_t *mb_data,
                                  uint32_t timeout_ms)
{
    assert(instance < CAN_INSTANCE_COUNT);

    flexcan_status_t result = kStatus_FLEXCAN_Success;
    flexcan_mb_code_status_t cs;
    flexcan_state_t *state = g_flexcanStatePtr[instance];
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    osa_status_t syncStatus;

    cs.data_length = tx_info->data_length;
    cs.msg_id_type = tx_info->msg_id_type;

    /* Set up FlexCAN message buffer for transmitting data*/
    cs.code = kFlexCanTX_Data;
    result = FLEXCAN_HAL_SetMbTx(canBaseAddr, mb_idx, &cs, msg_id, mb_data);

    if (result == kStatus_FLEXCAN_Success)
    {
        /* Enable message buffer interrupt*/
        FLEXCAN_HAL_EnableMbInt(canBaseAddr, mb_idx);
        do
        {
            syncStatus = OSA_SemaWait(&state->txIrqSync, timeout_ms);
        } while (syncStatus == kStatus_OSA_Idle);

        /* Disable message buffer interrupt*/
        FLEXCAN_HAL_DisableMbInt(canBaseAddr, mb_idx);

        /* Wait for the interrupt*/
        if (syncStatus != kStatus_OSA_Success)
        {
            result = kStatus_FLEXCAN_TimeOut;
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigMb
 * Description   : Configure a Rx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Rx message buffer as NOT_USED, enable
 * the Message Buffer interrupt, configure the message buffer code for Rx
 * message buffer as INACTIVE, copy user's buffer into the message buffer data
 * area, and configure the message buffer code for Rx message buffer as EMPTY.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigRxMb(uint8_t instance,
                                        uint32_t mb_idx,
                                        flexcan_data_info_t *rx_info,
                                        uint32_t msg_id)
{
    assert(instance < CAN_INSTANCE_COUNT);

    flexcan_status_t result;
    flexcan_mb_code_status_t cs;
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    flexcan_state_t *state = g_flexcanStatePtr[instance];

    state->rx_mb_idx = mb_idx;
    cs.data_length = rx_info->data_length;
    cs.msg_id_type = rx_info->msg_id_type;

    /* Initialize rx mb*/
    cs.code = kFlexCanRX_NotUsed;
    result = FLEXCAN_HAL_SetMbRx(canBaseAddr, mb_idx, &cs, msg_id);
    if (result)
    {
        return result;
    }

    /* Initialize receive MB*/
    cs.code = kFlexCanRX_Inactive;
    result = FLEXCAN_HAL_SetMbRx(canBaseAddr, mb_idx, &cs, msg_id);
    if (result)
    {
        return result;
    }

    /* Set up FlexCAN message buffer fields for receiving data*/
    cs.code = kFlexCanRX_Empty;
    return FLEXCAN_HAL_SetMbRx(canBaseAddr, mb_idx, &cs, msg_id);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigRxFifo
 * Description   : Confgure RX FIFO ID filter table elements.
 * This function will confgure RX FIFO ID filter table elements, and enable RX
 * FIFO interrupts.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigRxFifo(uint8_t instance,
                                          flexcan_rx_fifo_id_element_format_t id_format,
                                          flexcan_id_table_t *id_filter_table)
{
    assert(instance < CAN_INSTANCE_COUNT);

    flexcan_status_t result;
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    /* Initialize rx fifo*/
    result = FLEXCAN_HAL_SetRxFifo(canBaseAddr, id_format, id_filter_table);
    if (result)
    {
        return result;
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_StartRx
 * Description   : Start receive data after a Rx MB interrupt occurs.
 * This function will lock Rx MB after a Rx MB interrupt occurs, get the Rx MB
 * field values, and unlock the Rx MB.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_RxMessageBuffer(uint8_t instance, uint32_t mb_idx, flexcan_mb_t *data, uint32_t timeout_ms)
{
    assert(instance < CAN_INSTANCE_COUNT);
    assert(data);

    flexcan_status_t result;
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    flexcan_state_t *state = g_flexcanStatePtr[instance];
    osa_status_t syncStatus;

    /* Enable MB interrupt*/
    FLEXCAN_HAL_EnableMbInt(canBaseAddr, mb_idx);

    do
    {
        syncStatus = OSA_SemaWait(&state->rxIrqSync, timeout_ms);
    } while (syncStatus == kStatus_OSA_Idle);

    /* Wait for the interrupt*/
    if (syncStatus != kStatus_OSA_Success)
    {
        return kStatus_FLEXCAN_TimeOut;
    }

    FLEXCAN_HAL_LockRxMb(canBaseAddr, mb_idx);

    /* Get RX MB field values*/
    result = FLEXCAN_HAL_GetMb(canBaseAddr, mb_idx, data);
    if (result)
    {
        return result;
    }

    /* Unlock RX message buffer and RX FIFO*/
    FLEXCAN_HAL_UnlockRxMb(canBaseAddr);

    FLEXCAN_HAL_DisableMbInt(canBaseAddr, mb_idx);
    return kStatus_FLEXCAN_Success;
}

flexcan_status_t FLEXCAN_DRV_RxFifo(uint8_t instance, flexcan_mb_t *data, uint32_t timeout_ms)
{
    assert(instance < CAN_INSTANCE_COUNT);
    assert(data);

    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    osa_status_t syncStatus;
    flexcan_state_t *state = g_flexcanStatePtr[instance];

    /* This will get filled by the interrupt handler */
    state->fifo_message = data;

    /* Enable RX FIFO interrupts*/
    for (uint8_t i = 5; i <= 7; i++)
    {
        FLEXCAN_HAL_EnableMbInt(canBaseAddr, i);
    }

    do
    {
        syncStatus = OSA_SemaWait(&state->rxIrqSync, timeout_ms);
    } while (syncStatus == kStatus_OSA_Idle);

    /* Wait for the interrupt*/
    if (syncStatus != kStatus_OSA_Success)
    {
        return kStatus_FLEXCAN_TimeOut;
    }

    /* Disable RX FIFO interrupts*/
    for (uint8_t i = 5; i <= 7; i++)
    {
        FLEXCAN_HAL_DisableMbInt(canBaseAddr, i);
    }
    state->fifo_message = NULL;

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Deinit
 * Description   : Shutdown a FlexCAN module.
 * This function will disable all FlexCAN interrupts, and disable the FlexCAN.
 *
 *END**************************************************************************/
uint32_t FLEXCAN_DRV_Deinit(uint8_t instance)
{
    assert(instance < CAN_INSTANCE_COUNT);
    flexcan_state_t *state = g_flexcanStatePtr[instance];

#if defined(PKE18F15_SERIES)
    if (!PCC_RD_CLKCFG_CGC(PCC_FLEXCAN0_INDEX))
    {
        return 0;
    }
#else
    if (!(SIM_RD_SCGC6(SIM) & SIM_SCGC6_FLEXCAN0_MASK))
    {
        return 0;
    }
#endif

    /* Destroy I2C sema. */
    OSA_SemaDestroy(&state->txIrqSync);
    OSA_SemaDestroy(&state->rxIrqSync);
    /* Disable FlexCAN interrupts.*/
    NVIC_DisableIRQ(g_flexcanWakeUpIrqId[instance]);
    NVIC_DisableIRQ(g_flexcanErrorIrqId[instance]);
    NVIC_DisableIRQ(g_flexcanBusOffIrqId[instance]);
    NVIC_DisableIRQ(g_flexcanOredMessageBufferIrqId[instance]);

    /* De-initialize FLEXCAN device */
    FLEXCAN_HAL_Deinit((CAN_Type *)g_flexcanBaseAddr[instance]);

    /* Clear the state pointer */
    g_flexcanStatePtr[instance] = NULL;

/* Disable clock gate to FlexCAN module */
#if defined(PKE18F15_SERIES)
    PCC_WR_CLKCFG_CGC(PCC_FLEXCAN0_INDEX, 0);
#else
    SIM_CLR_SCGC6(SIM, SIM_SCGC6_FLEXCAN0_MASK);
#endif

    return 0;
}

void FLEXCAN_DRV_IRQHandler(uint8_t instance)
{
    volatile uint32_t flag_reg;
    uint32_t temp;
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];
    flexcan_state_t *state = g_flexcanStatePtr[instance];

    flexcan_info_t *appInfo = &s_applicationInfo[instance];

    /* Get the interrupts that are enabled and ready */
    flag_reg = ((FLEXCAN_HAL_GetAllMbIntFlags(canBaseAddr)) & CAN_IMASK1_BUFLM_MASK) & CAN_RD_IMASK1(canBaseAddr);

    /* Check Tx/Rx interrupt flag and clear the interrupt */
    if (flag_reg)
    {
#if FLEXCAN_USE_FIFO
        if ((flag_reg & 0x20) && CAN_RD_MCR_RFEN(canBaseAddr))
        {
            if (state->fifo_message != NULL)
            {
                /* Get RX FIFO field values */
                FLEXCAN_HAL_ReadFifo(canBaseAddr, state->fifo_message);
                OSA_SemaPost(&state->rxIrqSync);
            }
        }
        else
        {
#endif
            temp = (1 << state->rx_mb_idx);
            if (flag_reg & temp)
            {
                OSA_SemaPost(&state->rxIrqSync);

                /* Lock RX MB*/
                FLEXCAN_HAL_LockRxMb(canBaseAddr, state->rx_mb_idx);
                flexcan_mb_t rx_mb;
                /* Get RX MB field values*/
                if (!FLEXCAN_HAL_GetMb(canBaseAddr, state->rx_mb_idx, &rx_mb))
                {
                    uint8_t i;
                    uint8_t sink_byte = 0;
                    for (i = 0; i < rx_mb.length; i++)
                    {
                        sink_byte = rx_mb.data[i];
                        appInfo->data_sink(sink_byte, instance);
                    }
                }

                /* Unlock RX message buffer and RX FIFO*/
                FLEXCAN_HAL_UnlockRxMb(canBaseAddr);
            }
#if FLEXCAN_USE_FIFO
        }
#endif

        temp = (1 << state->tx_mb_idx);
        if (flag_reg & temp)
        {
            OSA_SemaPost(&state->txIrqSync);
        }

        FLEXCAN_HAL_ClearMbIntFlag(canBaseAddr, flag_reg);
    }

    /* Clear all other interrupts in ERRSTAT register (Error, Busoff, Wakeup) */
    FLEXCAN_HAL_ClearErrIntStatus(canBaseAddr);

    return;
}

void FLEXCAN_DRV_EnableMbInt(uint8_t instance, uint32_t mb_idx)
{
    CAN_Type *canBaseAddr = (CAN_Type *)g_flexcanBaseAddr[instance];

    FLEXCAN_HAL_EnableMbInt(canBaseAddr, mb_idx);
}

void FLEXCAN_DRV_EnableOperationMode(uint8_t instance, flexcan_operation_modes_t mode)
{
    uint32_t canBaseAddr = g_flexcanBaseAddr[instance];

    FLEXCAN_HAL_EnableOperationMode((CAN_Type *)canBaseAddr, mode);
}

void FLEXCAN_DRV_SoftReset(uint8_t instance)
{
    uint32_t canBaseAddr = g_flexcanBaseAddr[instance];

    FLEXCAN_HAL_SoftReset(canBaseAddr);
}

void FLEXCAN_DRV_SetDataSinkFunc(uint32_t instance, void (*data_sink)(uint8_t, uint32_t))
{
    s_applicationInfo[instance].data_sink = data_sink;
}

#endif // BL_CONFIG_CAN

/*******************************************************************************
 * EOF
 ******************************************************************************/
