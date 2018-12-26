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
#include "fsl_flexcan_hal.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK (0x3FFFFFFFU) /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format A extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT (1U)         /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format A extended shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK (0x3FF80000U) /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format A standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT (19U)        /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format A standard shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK (0x3FFFU)     /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1 (16U)       /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2 (0U)        /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK (0x3FF8U)     /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1 (19U)       /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B standard shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2 (3U)        /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format B standard shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK (0xFFU)           /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format C mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1 (24U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format C shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2 (16U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format C shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3 (8U)            /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format C shift3.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4 (0U)            /*!< FlexCAN RX FIFO ID filter*/
                                                                 /*! format C shift4.*/
#define FLEXCAN_ALL_INT (0x0007U)                                /*!< Masks for wakeup, error, bus off*/
                                                                 /*! interrupts*/
#define FLEXCAN_BYTE_DATA_FIELD_MASK (0xFFU)                     /*!< Masks for byte data field.*/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Enable
 * Description   : Enable FlexCAN module.
 * This function will enable FlexCAN module clock.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_Enable(CAN_Type *canBaseAddr)
{
    /* Check for low power mode*/
    if (CAN_RD_MCR_LPMACK(canBaseAddr))
    {
        /* Enable clock*/
        CAN_CLR_MCR(canBaseAddr, CAN_MCR_MDIS_MASK);
        /* Wait until enabled*/
        while (CAN_RD_MCR_LPMACK(canBaseAddr))
        {
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Disable
 * Description   : Disable FlexCAN module.
 * This function will disable FlexCAN module clock.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_Disable(CAN_Type *canBaseAddr)
{
    /* To access the memory mapped registers*/
    /* Entre disable mode (hard reset).*/
    if (CAN_RD_MCR_MDIS(canBaseAddr) == 0x0)
    {
        /* Clock disable (module)*/
        CAN_BWR_MCR_MDIS(canBaseAddr, 0x1);

        /* Wait until disable mode acknowledged*/
        while (!(CAN_RD_MCR_LPMACK(canBaseAddr)))
        {
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SelectClock
 * Description   : Select FlexCAN clock source.
 * This function will select either internal bus clock or external clock as
 * FlexCAN clock source.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SelectClock(CAN_Type *canBaseAddr, flexcan_clk_source_t clk)
{
    assert(clk < 2);

    CAN_BWR_CTRL1_CLKSRC(canBaseAddr, clk);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Reset
 * Description   : Reset FlexCAN module.
 * This function will reset FlexCAN module.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_Reset(CAN_Type *canBaseAddr)
{
    /* Make sure that module is not in low power mode*/
    if (!(CAN_RD_MCR_LPMACK(canBaseAddr)))
    {
        /* Reset the FLEXCAN*/
        CAN_BWR_MCR_SOFTRST(canBaseAddr, 0x1);

        /* Wait for reset cycle to complete*/
        while (CAN_RD_MCR_SOFTRST(canBaseAddr))
        {
        };
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Init
 * Description   : Initialize FlexCAN module.
 * This function will reset FlexCAN module, set maximum number of message
 * buffers, initialize all message buffers as inactive, enable RX FIFO
 * if needed, mask all mask bits, and disable all MB interrupts.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_Init(CAN_Type *canBaseAddr)
{
    /* Reset the FLEXCAN*/
    FLEXCAN_HAL_Reset(canBaseAddr);

    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Rx global mask*/
    CAN_WR_RXMGMASK(canBaseAddr, CAN_ID_EXT(CAN_RXMGMASK_MG_MASK));

    /* Rx reg 14 mask*/
    CAN_WR_RX14MASK(canBaseAddr, CAN_ID_EXT(CAN_RX14MASK_RX14M_MASK));

    /* Rx reg 15 mask*/
    CAN_WR_RX15MASK(canBaseAddr, CAN_ID_EXT(CAN_RX15MASK_RX15M_MASK));

    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);

    /* Disable all MB interrupts*/
    CAN_WR_IMASK1(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Deinit
 * Description   : De-initialize FlexCAN module.
 * This function will reset FlexCAN module, disable FlexCAN module clock.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_Deinit(CAN_Type *canBaseAddr)
{
    /* Soft reset the FLEXCAN*/
    FLEXCAN_HAL_Reset(canBaseAddr);

/* Manually reset those registers which are not affected by soft reset*/
#if FSL_FEATURE_FLEXCAN_HAS_EXTENDED_TIMING_REGISTER
    CAN_WR_CBT(canBaseAddr, 0x0);
#endif
    CAN_WR_CTRL1(canBaseAddr, 0x0);

    /* Disable FlexCAN.*/
    FLEXCAN_HAL_Disable(canBaseAddr);

    /* Need to reset CTRL1 again, since some bits can be written only in disable/freeze mode*/
    CAN_WR_CTRL1(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTimeSegments
 * Description   : Set FlexCAN time segments.
 * This function will set all FlexCAN time segments which define the length of
 * Propagation Segment in the bit time, the length of Phase Buffer Segment 2 in
 * the bit time, the length of Phase Buffer Segment 1 in the bit time, the ratio
 * between the PE clock frequency and the Serial Clock (Sclock) frequency, and
 * the maximum number of time quanta that a bit time can be changed by one
 * resynchronization. (One time quantum is equal to the Sclock period.)
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetTimeSegments(CAN_Type *canBaseAddr, const flexcan_time_segment_t *time_seg)
{
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Set FlexCAN time segments*/
    CAN_CLR_CTRL1(canBaseAddr,
                  (CAN_CTRL1_PROPSEG_MASK | CAN_CTRL1_PSEG2_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PRESDIV_MASK) |
                      CAN_CTRL1_RJW_MASK);
    CAN_SET_CTRL1(canBaseAddr, (CAN_CTRL1_PROPSEG(time_seg->propseg) | CAN_CTRL1_PSEG2(time_seg->pseg2) |
                                CAN_CTRL1_PSEG1(time_seg->pseg1) | CAN_CTRL1_PRESDIV(time_seg->pre_divider) |
                                CAN_CTRL1_RJW(time_seg->rjw)));

    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetTimeSegments
 * Description   : Get FlexCAN time segments.
 * This function will get all FlexCAN time segments defined.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_GetTimeSegments(CAN_Type *canBaseAddr, flexcan_time_segment_t *time_seg)
{
    time_seg->pre_divider = CAN_RD_CTRL1_PRESDIV(canBaseAddr);
    time_seg->propseg = CAN_RD_CTRL1_PROPSEG(canBaseAddr);
    time_seg->pseg1 = CAN_RD_CTRL1_PSEG1(canBaseAddr);
    time_seg->pseg2 = CAN_RD_CTRL1_PSEG2(canBaseAddr);
    time_seg->rjw = CAN_RD_CTRL1_RJW(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMbTx
 * Description   : Configure a message buffer for transmission.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will copy user's buffer into the
 * message buffer data area and configure the message buffer as required for
 * transmission.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetMbTx(
    CAN_Type *canBaseAddr, uint32_t mb_idx, flexcan_mb_code_status_t *cs, uint32_t msg_id, uint8_t *mb_data)
{
    uint32_t i;
    uint32_t temp, temp1;
    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);

    assert(flexcan_reg_ptr);
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

#if FLEXCAN_USE_FIFO
    /* Check if RX FIFO is enabled*/
    if (CAN_RD_MCR_RFEN(canBaseAddr))
    {
        /* Get the number of RX FIFO Filters*/
        temp1 = (CAN_RD_CTRL2_RFFN(canBaseAddr));
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        temp = 6 + (temp1 + 1) * 8 / 4;

        if (mb_idx <= (temp - 1))
        {
            return (kStatus_FLEXCAN_InvalidArgument);
        }
    }
#endif

    /* Copy user's buffer into the message buffer data area*/
    if (mb_data != NULL)
    {
        flexcan_reg_ptr->MB[mb_idx].WORD0 = 0x0;
        flexcan_reg_ptr->MB[mb_idx].WORD1 = 0x0;

        for (i = 0; i < cs->data_length; i++)
        {
            temp1 = (*(mb_data + i));
            if (i < 4)
            {
                temp = temp1 << ((3 - i) * 8);
                flexcan_reg_ptr->MB[mb_idx].WORD0 |= temp;
            }
            else
            {
                temp = temp1 << ((7 - i) * 8);
                flexcan_reg_ptr->MB[mb_idx].WORD1 |= temp;
            }
        }
    }

/* Set the ID according the format structure*/
#if FLEXCAN_USE_EXT_ID
    if (cs->msg_id_type == kFlexCanMbId_Ext)
    {
        /* ID [28-0]*/
        flexcan_reg_ptr->MB[mb_idx].ID &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        flexcan_reg_ptr->MB[mb_idx].ID |= (msg_id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

        /* Set IDE*/
        flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_IDE_MASK;

        /* Clear SRR bit*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_SRR_MASK;

        /* Set the length of data in bytes*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_DLC_MASK;
        flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_DLC(cs->data_length);

        /* Set MB CODE*/
        /* Reset the code*/
        if (cs->code != kFlexCanTX_NotUsed)
        {
            if (cs->code == kFlexCanTX_Remote)
            {
                /* Set RTR bit*/
                flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_RTR_MASK;
                cs->code = kFlexCanTX_Data;
            }

            /* Reset the code*/
            flexcan_reg_ptr->MB[mb_idx].CS &= ~(CAN_CS_CODE_MASK);

            /* Activating message buffer*/
            flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_CODE(cs->code);
        }
    }
    else if (cs->msg_id_type == kFlexCanMbId_Std)
    {
#endif
        /* ID[28-18]*/
        flexcan_reg_ptr->MB[mb_idx].ID &= ~CAN_ID_STD_MASK;
        flexcan_reg_ptr->MB[mb_idx].ID |= CAN_ID_STD(msg_id);

        /* make sure IDE and SRR are not set*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

        /* Set the length of data in bytes*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_DLC_MASK;
        flexcan_reg_ptr->MB[mb_idx].CS |= (cs->data_length) << CAN_CS_DLC_SHIFT;

        /* Set MB CODE*/
        if (cs->code != kFlexCanTX_NotUsed)
        {
            if (cs->code == kFlexCanTX_Remote)
            {
                /* Set RTR bit*/
                flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_RTR_MASK;
                cs->code = kFlexCanTX_Data;
            }

            /* Reset the code*/
            flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_CODE_MASK;

            /* Set the code*/
            flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_CODE(cs->code);
        }
#if FLEXCAN_USE_EXT_ID
    }
    else
    {
        return (kStatus_FLEXCAN_InvalidArgument);
    }
#endif

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMbRx
 * Description   : Configure a message buffer for receiving.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will configure the message buffer as
 * required for receiving.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetMbRx(CAN_Type *canBaseAddr,
                                     uint32_t mb_idx,
                                     flexcan_mb_code_status_t *cs,
                                     uint32_t msg_id)
{
#if FLEXCAN_USE_FIFO
    uint32_t val1, val2 = 1;
#endif

    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);

    assert(flexcan_reg_ptr);
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

#if FLEXCAN_USE_FIFO
    /* Check if RX FIFO is enabled*/
    if (CAN_RD_MCR_RFEN(canBaseAddr))
    {
        /* Get the number of RX FIFO Filters*/
        val1 = CAN_RD_CTRL2_RFFN(canBaseAddr);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = 6 + (val1 + 1) * 8 / 4;

        if (mb_idx <= (val2 - 1))
        {
            return (kStatus_FLEXCAN_InvalidArgument);
        }
    }
#endif

#if FLEXCAN_USE_EXT_ID
    /* Set the ID according the format structure*/
    if (cs->msg_id_type == kFlexCanMbId_Ext)
    {
        /* Set IDE*/
        flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_IDE_MASK;

        /* Clear SRR bit*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_SRR_MASK;

        /* Set the length of data in bytes*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_DLC_MASK;
        flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_DLC(cs->data_length);

        /* ID [28-0]*/
        flexcan_reg_ptr->MB[mb_idx].ID &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        flexcan_reg_ptr->MB[mb_idx].ID |= (msg_id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

        /* Set MB CODE*/
        if (cs->code != kFlexCanRX_NotUsed)
        {
            flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_CODE_MASK;
            flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_CODE(cs->code);
        }
    }
    else if (cs->msg_id_type == kFlexCanMbId_Std)
    {
#endif
        /* Make sure IDE and SRR are not set*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

        /* Set the length of data in bytes*/
        flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_DLC_MASK;
        flexcan_reg_ptr->MB[mb_idx].CS |= (cs->data_length) << CAN_CS_DLC_SHIFT;

        /* ID[28-18]*/
        flexcan_reg_ptr->MB[mb_idx].ID &= ~CAN_ID_STD_MASK;
        flexcan_reg_ptr->MB[mb_idx].ID |= CAN_ID_STD(msg_id);

        /* Set MB CODE*/
        if (cs->code != kFlexCanRX_NotUsed)
        {
            flexcan_reg_ptr->MB[mb_idx].CS &= ~CAN_CS_CODE_MASK;
            flexcan_reg_ptr->MB[mb_idx].CS |= CAN_CS_CODE(cs->code);
        }
#if FLEXCAN_USE_EXT_ID
    }
    else
    {
        return (kStatus_FLEXCAN_InvalidArgument);
    }
#endif

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMb
 * Description   : Get a message buffer field values.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will get the message buffer field
 * values and copy the MB data field into user's buffer.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_GetMb(CAN_Type *canBaseAddr, uint32_t mb_idx, flexcan_mb_t *mb)
{
    uint32_t i;
#if FLEXCAN_USE_FIFO
    uint32_t val1, val2 = 1;
#endif

    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);
    assert(flexcan_reg_ptr);
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

#if FLEXCAN_USE_FIFO
    /* Check if RX FIFO is enabled*/
    if (CAN_RD_MCR_RFEN(canBaseAddr))
    {
        /* Get the number of RX FIFO Filters*/
        val1 = CAN_RD_CTRL2_RFFN(canBaseAddr);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = 6 + (val1 + 1) * 8 / 4;

        if (mb_idx <= (val2 - 1))
        {
            return (kStatus_FLEXCAN_InvalidArgument);
        }
    }
#endif

    /* Get a MB field values*/
    mb->cs = flexcan_reg_ptr->MB[mb_idx].CS;
    if ((mb->cs) & CAN_CS_IDE_MASK)
    {
        mb->msg_id = flexcan_reg_ptr->MB[mb_idx].ID;
    }
    else
    {
        mb->msg_id = (flexcan_reg_ptr->MB[mb_idx].ID) >> CAN_ID_STD_SHIFT;
    }

    /* Copy MB data field into user's buffer*/
    for (i = 0; i < kFlexCanMessageSize; i++)
    {
        if (i < 4)
        {
            mb->data[3 - i] = ((flexcan_reg_ptr->MB[mb_idx].WORD0) >> (i * 8)) & FLEXCAN_BYTE_DATA_FIELD_MASK;
        }
        else
        {
            mb->data[11 - i] = ((flexcan_reg_ptr->MB[mb_idx].WORD1) >> ((i - 4) * 8)) & FLEXCAN_BYTE_DATA_FIELD_MASK;
        }
    }

    mb->length = (flexcan_reg_ptr->MB[mb_idx].CS) >> 16;

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_LockRxMb
 * Description   : Lock the RX message buffer.
 * This function will the RX message buffer.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_LockRxMb(CAN_Type *canBaseAddr, uint32_t mb_idx)
{
    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);
    assert(flexcan_reg_ptr);
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

    /* Lock the mailbox*/
    flexcan_reg_ptr->MB[mb_idx].CS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableRxFifo
 * Description   : Enable Rx FIFO feature.
 * This function will enable the Rx FIFO feature.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableRxFifo(CAN_Type *canBaseAddr, uint32_t numOfFilters, uint32_t maxNumMb)
{
    uint32_t i;

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Enable RX FIFO*/
    CAN_BWR_MCR_RFEN(canBaseAddr, 0x1);
    /* Set the number of the RX FIFO filters needed*/
    CAN_BWR_CTRL2_RFFN(canBaseAddr, numOfFilters);
    /* RX FIFO global mask*/
    CAN_WR_RXFGMASK(canBaseAddr, CAN_ID_EXT(CAN_RXFGMASK_FGM_MASK));
    for (i = 0; i < maxNumMb; i++)
    {
        /* RX individual mask*/
        CAN_WR_RXIMR(canBaseAddr, i, CAN_ID_EXT(CAN_RXIMR_MI_MASK));
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableRxFifo
 * Description   : Disable Rx FIFO feature.
 * This function will disable the Rx FIFO feature.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableRxFifo(CAN_Type *canBaseAddr)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Disable RX FIFO*/
    CAN_BWR_MCR_RFEN(canBaseAddr, 0x0);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoFiltersNumber
 * Description   : Set the number of Rx FIFO filters.
 * This function will define the number of Rx FIFO filters.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoFiltersNumber(CAN_Type *canBaseAddr, uint32_t number)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Set the number of RX FIFO ID filters*/
    CAN_BWR_CTRL2_RFFN(canBaseAddr, number);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaxMbNumber
 * Description   : Set the number of the last Message Buffers.
 * This function will define the number of the last Message Buffers
 *
*END**************************************************************************/
void FLEXCAN_HAL_SetMaxMbNumber(CAN_Type *canBaseAddr, uint32_t maxNumMb)
{
    uint32_t i;
    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Set the maximum number of MBs*/
    CAN_BWR_MCR_MAXMB(canBaseAddr, maxNumMb);

    /* Initialize all message buffers as inactive*/
    for (i = 0; i < maxNumMb; i++)
    {
        flexcan_reg_ptr->MB[i].CS = 0x0;
        flexcan_reg_ptr->MB[i].ID = 0x0;
        flexcan_reg_ptr->MB[i].WORD0 = 0x0;
        flexcan_reg_ptr->MB[i].WORD1 = 0x0;
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetIdFilterTableElements
 * Description   : Set ID filter table elements.
 * This function will set up ID filter table elements.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetIdFilterTableElements(CAN_Type *canBaseAddr,
                                                      flexcan_rx_fifo_id_element_format_t id_format,
                                                      flexcan_id_table_t *id_filter_table)
{
    uint32_t i, j, numOfFilters;
    uint32_t val1 = 0, val2 = 0, val = 0;
    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);
    assert(flexcan_reg_ptr);

    numOfFilters = CAN_RD_CTRL2_RFFN(canBaseAddr);
    switch (id_format)
    {
        case (kFlexCanRxFifoIdElementFormat_A):
            /* One full ID (standard and extended) per ID Filter Table element.*/
            CAN_BWR_MCR_IDAM(canBaseAddr, kFlexCanRxFifoIdElementFormat_A);
            if (id_filter_table->is_remote_mb)
            {
                val = 1U << 31U;
            }
            if (id_filter_table->is_extended_mb)
            {
                val |= 1 << 30;
                j = 0;
                for (i = 0; i < (numOfFilters + 1) * 8; i += 4)
                {
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS =
                        val + ((*(id_filter_table->id_filter + i)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].ID =
                        val + ((*(id_filter_table->id_filter + i + 1)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 =
                        val + ((*(id_filter_table->id_filter + i + 2)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 =
                        val + ((*(id_filter_table->id_filter + i + 3)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                    j++;
                }
            }
            else
            {
                j = 0;
                for (i = 0; i < (numOfFilters + 1) * 8; i += 4)
                {
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS =
                        val + ((*(id_filter_table->id_filter + i)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].ID =
                        val + ((*(id_filter_table->id_filter + i + 1)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 =
                        val + ((*(id_filter_table->id_filter + i + 2)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 =
                        val + ((*(id_filter_table->id_filter + i + 3)) << FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT &
                               FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                    j++;
                }
            }
            break;
        case (kFlexCanRxFifoIdElementFormat_B):
            /* Two full standard IDs or two partial 14-bit (standard and extended) IDs*/
            /* per ID Filter Table element.*/
            CAN_BWR_MCR_IDAM(canBaseAddr, kFlexCanRxFifoIdElementFormat_B);
            if (id_filter_table->is_remote_mb)
            {
                val1 = 1U << 31U;
                val2 = 1 << 15;
            }
            if (id_filter_table->is_extended_mb)
            {
                val1 |= 1 << 30;
                val2 |= 1 << 14;
                j = 0;
                for (i = 0; i < (numOfFilters + 1) * 8; i += 8)
                {
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS =
                        val1 +
                        ((*(id_filter_table->id_filter + i)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS |=
                        val2 +
                        ((*(id_filter_table->id_filter + i + 1)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].ID =
                        val1 +
                        ((*(id_filter_table->id_filter + i + 2)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].ID |=
                        val2 +
                        ((*(id_filter_table->id_filter + i + 3)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 =
                        val1 +
                        ((*(id_filter_table->id_filter + i + 4)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 |=
                        val2 +
                        ((*(id_filter_table->id_filter + i + 5)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 =
                        val1 +
                        ((*(id_filter_table->id_filter + i + 6)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 |=
                        val2 +
                        ((*(id_filter_table->id_filter + i + 7)) &
                         FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);
                    j++;
                }
            }
            else
            {
                j = 0;
                for (i = 0; i < (numOfFilters + 1) * 8; i += 8)
                {
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS =
                        val1 + (((*(id_filter_table->id_filter + i)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].CS |=
                        val2 + (((*(id_filter_table->id_filter + i + 1)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].ID =
                        val1 + (((*(id_filter_table->id_filter + i + 2)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].ID |=
                        val2 + (((*(id_filter_table->id_filter + i + 3)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 =
                        val1 + (((*(id_filter_table->id_filter + i + 4)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 |=
                        val2 + (((*(id_filter_table->id_filter + i + 5)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);

                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 =
                        val1 + (((*(id_filter_table->id_filter + i + 6)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 |=
                        val2 + (((*(id_filter_table->id_filter + i + 7)) & FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK)
                                << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);
                    j++;
                }
            }
            break;
        case (kFlexCanRxFifoIdElementFormat_C):
            /* Four partial 8-bit Standard IDs per ID Filter Table element.*/
            CAN_BWR_MCR_IDAM(canBaseAddr, kFlexCanRxFifoIdElementFormat_C);
            j = 0;
            for (i = 0; i < (numOfFilters + 1) * 8; i += 16)
            {
                flexcan_reg_ptr->MB[6 + i - j * 3].CS =
                    ((*(id_filter_table->id_filter + i)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                flexcan_reg_ptr->MB[6 + i - j * 3].CS |=
                    ((*(id_filter_table->id_filter + i + 1)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                flexcan_reg_ptr->MB[6 + i - j * 3].CS |=
                    ((*(id_filter_table->id_filter + i + 2)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                flexcan_reg_ptr->MB[6 + i - j * 3].CS |=
                    ((*(id_filter_table->id_filter + i + 3)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);

                flexcan_reg_ptr->MB[6 + i - j * 3].ID =
                    ((*(id_filter_table->id_filter + i + 4)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                flexcan_reg_ptr->MB[6 + i - j * 3].ID |=
                    ((*(id_filter_table->id_filter + i + 5)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                flexcan_reg_ptr->MB[6 + i - j * 3].ID |=
                    ((*(id_filter_table->id_filter + i + 6)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                flexcan_reg_ptr->MB[6 + i - j * 3].ID |=
                    ((*(id_filter_table->id_filter + i + 7)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);

                flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 =
                    ((*(id_filter_table->id_filter + i + 8)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 |=
                    ((*(id_filter_table->id_filter + i + 9)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 |=
                    ((*(id_filter_table->id_filter + i + 10)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD0 |=
                    ((*(id_filter_table->id_filter + i + 11)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);

                flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 =
                    ((*(id_filter_table->id_filter + i + 12)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 |=
                    ((*(id_filter_table->id_filter + i + 13)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 |=
                    ((*(id_filter_table->id_filter + i + 14)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                flexcan_reg_ptr->MB[6 + i - j * 3].WORD1 |=
                    ((*(id_filter_table->id_filter + i + 15)) &
                     FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK << FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);

                j++;
            }
            break;
        case (kFlexCanRxFifoIdElementFormat_D):
            /* All frames rejected.*/
            CAN_BWR_MCR_IDAM(canBaseAddr, kFlexCanRxFifoIdElementFormat_D);
            break;
        default:
            return kStatus_FLEXCAN_InvalidArgument;
    }

    return (kStatus_FLEXCAN_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifo
 * Description   : Confgure RX FIFO ID filter table elements.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetRxFifo(CAN_Type *canBaseAddr,
                                       flexcan_rx_fifo_id_element_format_t id_format,
                                       flexcan_id_table_t *id_filter_table)
{
    /* Set RX FIFO ID filter table elements*/
    return FLEXCAN_HAL_SetIdFilterTableElements(canBaseAddr, id_format, id_filter_table);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableMbInt
 * Description   : Enable the corresponding Message Buffer interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableMbInt(CAN_Type *canBaseAddr, uint32_t mb_idx)
{
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

    /* Enable the corresponding message buffer Interrupt*/
    CAN_SET_IMASK1(canBaseAddr, 0x1 << mb_idx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableMbInt
 * Description   : Disable the corresponding Message Buffer interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableMbInt(CAN_Type *canBaseAddr, uint32_t mb_idx)
{
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

    /* Disable the corresponding message buffer Interrupt*/
    CAN_CLR_IMASK1(canBaseAddr, 0x1 << mb_idx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableErrInt
 * Description   : Enable the error interrupts.
 * This function will enable Error interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableErrInt(CAN_Type *canBaseAddr)
{
    /* Enable Error interrupt*/
    CAN_BWR_CTRL1_ERRMSK(canBaseAddr, 0x1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableErrorInt
 * Description   : Disable the error interrupts.
 * This function will disable Error interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableErrInt(CAN_Type *canBaseAddr)
{
    /* Disable Error interrupt*/
    CAN_BWR_CTRL1_ERRMSK(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableBusOffInt
 * Description   : Enable the Bus off interrupts.
 * This function will enable Bus Off interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableBusOffInt(CAN_Type *canBaseAddr)
{
    /* Enable Bus Off interrupt*/
    CAN_BWR_CTRL1_BOFFMSK(canBaseAddr, 0x1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableBusOffInt
 * Description   : Disable the Bus off interrupts.
 * This function will disable Bus Off interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableBusOffInt(CAN_Type *canBaseAddr)
{
    /* Disable Bus Off interrupt*/
    CAN_BWR_CTRL1_BOFFMSK(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableWakeupInt
 * Description   : Enable the wakeup interrupts.
 * This function will enable Wake up interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableWakeupInt(CAN_Type *canBaseAddr)
{
    /* Enable Wake Up interrupt*/
    CAN_BWR_MCR_WAKMSK(canBaseAddr, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableWakeupInt
 * Description   : Disable the wakeup interrupts.
 * This function will disable Wake up interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableWakeupInt(CAN_Type *canBaseAddr)
{
    /* Disable Wake Up interrupt*/
    CAN_BWR_MCR_WAKMSK(canBaseAddr, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableTxWarningInt
 * Description   : Enable the TX warning interrupts.
 * This function will enable TX warning interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableTxWarningInt(CAN_Type *canBaseAddr)
{
    /* Enable TX warning interrupt*/
    CAN_BWR_CTRL1_TWRNMSK(canBaseAddr, 0x1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableTxWarningInt
 * Description   : Disable the TX warning interrupts.
 * This function will disable TX warning interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableTxWarningInt(CAN_Type *canBaseAddr)
{
    /* Disable TX warning interrupt*/
    CAN_BWR_CTRL1_TWRNMSK(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableRxWarningInt
 * Description   : Enable the RX warning interrupts.
 * This function will enable RX warning interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableRxWarningInt(CAN_Type *canBaseAddr)
{
    /* Enable RX warning interrupt*/
    CAN_BWR_CTRL1_RWRNMSK(canBaseAddr, 0x1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableRxWarningInt
 * Description   : Disable the RX warning interrupts.
 * This function will disable RX warning interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableRxWarningInt(CAN_Type *canBaseAddr)
{
    /* Disable RX warning interrupt*/
    CAN_BWR_CTRL1_RWRNMSK(canBaseAddr, 0x0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ExitFreezeMode
 * Description   : Exit of freeze mode.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_ExitFreezeMode(CAN_Type *canBaseAddr)
{
    CAN_BWR_MCR_HALT(canBaseAddr, 0x0);
    CAN_BWR_MCR_FRZ(canBaseAddr, 0x0);

    /* Wait till exit freeze mode*/
    while (CAN_RD_MCR_FRZACK(canBaseAddr))
    {
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnterFreezeMode
 * Description   : Enter the freeze mode.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnterFreezeMode(CAN_Type *canBaseAddr)
{
    CAN_BWR_MCR_FRZ(canBaseAddr, 0x1);
    CAN_BWR_MCR_HALT(canBaseAddr, 0x1);

    /* Wait for entering the freeze mode*/
    while (!(CAN_RD_MCR_FRZACK(canBaseAddr)))
    {
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMbIntFlag
 * Description   : Get the corresponding message buffer interrupt flag.
 *
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_GetMbIntFlag(CAN_Type *canBaseAddr, uint32_t mb_idx)
{
    uint32_t temp;

    /* Get the corresponding message buffer interrupt flag*/
    temp = 0x1 << mb_idx;
    if (CAN_RD_IFLAG1(canBaseAddr) & temp)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetErrCounter
 * Description   : Get transmit error counter and receive error counter.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_GetErrCounter(CAN_Type *canBaseAddr, flexcan_berr_counter_t *err_cnt)
{
    /* Get transmit error counter and receive error counter*/
    err_cnt->rxerr = CAN_RD_ECR_RXERRCNT(canBaseAddr);
    err_cnt->txerr = CAN_RD_ECR_TXERRCNT(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ClearErrIntStatus
 * Description   : Clear all error interrupt status.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_ClearErrIntStatus(CAN_Type *canBaseAddr)
{
    if (CAN_RD_ESR1(canBaseAddr) & FLEXCAN_ALL_INT)
    {
        CAN_WR_ESR1(canBaseAddr, FLEXCAN_ALL_INT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ReadFifo
 * Description   : Read Rx FIFO data.
 * This function will copy MB[0] data field into user's buffer.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_ReadFifo(CAN_Type *canBaseAddr, flexcan_mb_t *rx_fifo)
{
    uint32_t i;
    volatile CAN_Type *flexcan_reg_ptr;

    flexcan_reg_ptr = ((CAN_Type *)canBaseAddr);

    assert(flexcan_reg_ptr);

    rx_fifo->cs = flexcan_reg_ptr->MB[0].CS;

    if ((rx_fifo->cs) & CAN_CS_IDE_MASK)
    {
        rx_fifo->msg_id = flexcan_reg_ptr->MB[0].ID;
    }
    else
    {
        rx_fifo->msg_id = (flexcan_reg_ptr->MB[0].ID) >> CAN_ID_STD_SHIFT;
    }

    /* Copy MB[0] data field into user's buffer*/
    for (i = 0; i < kFlexCanMessageSize; i++)
    {
        if (i < 4)
        {
            rx_fifo->data[3 - i] = ((flexcan_reg_ptr->MB[0].WORD0) >> (i * 8)) & FLEXCAN_BYTE_DATA_FIELD_MASK;
        }
        else
        {
            rx_fifo->data[11 - i] = ((flexcan_reg_ptr->MB[0].WORD1) >> ((i - 4) * 8)) & FLEXCAN_BYTE_DATA_FIELD_MASK;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaskType
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetMaskType(CAN_Type *canBaseAddr, flexcan_rx_mask_type_t type)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* Set RX masking type (RX global mask or RX individual mask)*/
    if (type == kFlexCanRxMask_Global)
    {
        /* Enable Global RX masking*/
        CAN_BWR_MCR_IRMQ(canBaseAddr, 0x0);
    }
    else
    {
        /* Enable Individual Rx Masking and Queue*/
        CAN_BWR_MCR_IRMQ(canBaseAddr, 0x1);
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalStdMask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalStdMask(CAN_Type *canBaseAddr, uint32_t std_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 11 bit standard mask*/
    CAN_WR_RXFGMASK(canBaseAddr, CAN_ID_STD(std_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalExtMask
 * Description   : Set Rx FIFO global mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalExtMask(CAN_Type *canBaseAddr, uint32_t ext_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 29-bit extended mask*/
    CAN_WR_RXFGMASK(canBaseAddr, CAN_ID_EXT(ext_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualStdMask
 * Description   : Set Rx individual mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxIndividualStdMask(CAN_Type *canBaseAddr, uint32_t mb_idx, uint32_t std_mask)
{
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 11 bit standard mask*/
    CAN_WR_RXIMR(canBaseAddr, mb_idx, CAN_ID_STD(std_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualExtMask
 * Description   : Set Rx individual mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxIndividualExtMask(CAN_Type *canBaseAddr, uint32_t mb_idx, uint32_t ext_mask)
{
    assert(mb_idx < CAN_RD_MCR_MAXMB(canBaseAddr));

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 29-bit extended mask*/
    CAN_WR_RXIMR(canBaseAddr, mb_idx, CAN_ID_EXT(ext_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbGlobalStdMask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbGlobalStdMask(CAN_Type *canBaseAddr, uint32_t std_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 11 bit standard mask*/
    CAN_WR_RXMGMASK(canBaseAddr, CAN_ID_STD(std_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf14StdMask
 * Description   : Set Rx Message Buffer 14 mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbBuf14StdMask(CAN_Type *canBaseAddr, uint32_t std_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 11 bit standard mask*/
    CAN_WR_RX14MASK(canBaseAddr, CAN_ID_STD(std_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf15StdMask
 * Description   : Set Rx Message Buffer 15 mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbBuf15StdMask(CAN_Type *canBaseAddr, uint32_t std_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 11 bit standard mask*/
    CAN_WR_RX15MASK(canBaseAddr, CAN_ID_STD(std_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbGlobalExtMask
 * Description   : Set Rx Message Buffer global mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbGlobalExtMask(CAN_Type *canBaseAddr, uint32_t ext_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 29-bit extended mask*/
    CAN_WR_RXMGMASK(canBaseAddr, CAN_ID_EXT(ext_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf14ExtMask
 * Description   : Set Rx Message Buffer 14 mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbBuf14ExtMask(CAN_Type *canBaseAddr, uint32_t ext_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 29-bit extended mask*/
    CAN_WR_RX14MASK(canBaseAddr, CAN_ID_EXT(ext_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf15ExtMask
 * Description   : Set Rx Message Buffer 15 mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMbBuf15ExtMask(CAN_Type *canBaseAddr, uint32_t ext_mask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    /* 29-bit extended mask*/
    CAN_WR_RX15MASK(canBaseAddr, CAN_ID_EXT(ext_mask));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableOperationMode
 * Description   : Enable a FlexCAN operation mode.
 * This function will enable one of the modes listed in flexcan_operation_modes_t.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableOperationMode(CAN_Type *canBaseAddr, flexcan_operation_modes_t mode)
{
    if (mode == kFlexCanFreezeMode)
    {
        /* Debug mode, Halt and Freeze*/
        FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);
    }
    else if (mode == kFlexCanDisableMode)
    {
        /* Debug mode, Halt and Freeze*/
        CAN_BWR_MCR_MDIS(canBaseAddr, 0x1);
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    if (mode == kFlexCanNormalMode)
    {
        CAN_BWR_MCR_SUPV(canBaseAddr, 0x0);
    }
    else if (mode == kFlexCanListenOnlyMode)
    {
        CAN_BWR_CTRL1_LOM(canBaseAddr, 0x1);
    }
    else if (mode == kFlexCanLoopBackMode)
    {
        CAN_BWR_CTRL1_LPB(canBaseAddr, 0x1);
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableOperationMode
 * Description   : Disable a FlexCAN operation mode.
 * This function will disable one of the modes listed in flexcan_operation_modes_t.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableOperationMode(CAN_Type *canBaseAddr, flexcan_operation_modes_t mode)
{
    if (mode == kFlexCanFreezeMode)
    {
        /* De-assert Freeze Mode*/
        FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
    }
    else if (mode == kFlexCanDisableMode)
    {
        /* Disable module mode*/
        CAN_WR_MCR_MDIS(canBaseAddr, 0x0);
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(canBaseAddr);

    if (mode == kFlexCanNormalMode)
    {
        CAN_BWR_MCR_SUPV(canBaseAddr, 0x1);
    }
    else if (mode == kFlexCanListenOnlyMode)
    {
        CAN_BWR_CTRL1_LOM(canBaseAddr, 0x0);
    }
    else if (mode == kFlexCanLoopBackMode)
    {
        CAN_BWR_CTRL1_LPB(canBaseAddr, 0x0);
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(canBaseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SoftReset
 * Description   : Perform a soft reset to FexCAN.
 * This function will perfomr a soft reset to FlexCAN.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SoftReset(uint32_t canBaseAddr)
{
    CAN_WR_MCR_SOFTRST((CAN_Type *)canBaseAddr, 0x1);
    /* Wait till exit of reset*/
    while (CAN_RD_MCR_SOFTRST((CAN_Type *)canBaseAddr))
    {
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
