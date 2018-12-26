/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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

#include "fsl_mipi_dsi_cmd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t MIPI_DSI_DCS_SoftReset(mipi_dsi_device_t *device)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData = kMIPI_DCS_SoftReset;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_SetDisplayOn(mipi_dsi_device_t *device, bool on)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    if (on)
    {
        txData = kMIPI_DCS_SetDisplayOn;
    }
    else
    {
        txData = kMIPI_DCS_SetDisplayOff;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_EnterSleepMode(mipi_dsi_device_t *device, bool enter)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    if (enter)
    {
        txData = kMIPI_DCS_EnterSleepMode;
    }
    else
    {
        txData = kMIPI_DCS_ExitSleepMode;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_EnterPartialMode(mipi_dsi_device_t *device, bool enter)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    if (enter)
    {
        txData = kMIPI_DCS_EnterPartialMode;
    }
    else
    {
        txData = kMIPI_DCS_EnterNormalMode;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_EnterInvertMode(mipi_dsi_device_t *device, bool enter)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    if (enter)
    {
        txData = kMIPI_DCS_EnterInvertMode;
    }
    else
    {
        txData = kMIPI_DCS_ExitInvertMode;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_EnterIdleMode(mipi_dsi_device_t *device, bool enter)
{
    dsi_transfer_t dsiXfer = {0};
    uint8_t txData;

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    dsiXfer.txDataSize = 1;
    dsiXfer.txData = &txData;

    if (enter)
    {
        txData = kMIPI_DCS_EnterIdleMode;
    }
    else
    {
        txData = kMIPI_DCS_ExitIdleMode;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_DCS_Write(mipi_dsi_device_t *device, const uint8_t *txData, int32_t txDataSize)
{
    dsi_transfer_t dsiXfer = {0};

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataSize = txDataSize;
    dsiXfer.txData = txData;

    if (0 == txDataSize)
    {
        /* For DSC command, the data size should not be 0. */
        return kStatus_InvalidArgument;
    }
    else if (1 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataDcsShortWrNoParam;
    }
    else if (2 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataDcsShortWrOneParam;
    }
    else
    {
        dsiXfer.txDataType = kDSI_TxDataDcsLongWr;
    }

    return device->xferFunc(&dsiXfer);
}

status_t MIPI_DSI_GenericWrite(mipi_dsi_device_t *device, const uint8_t *txData, int32_t txDataSize)
{
    dsi_transfer_t dsiXfer = {0};

    dsiXfer.virtualChannel = device->virtualChannel;
    dsiXfer.txDataSize = txDataSize;
    dsiXfer.txData = txData;

    if (0 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrNoParam;
    }
    else if (1 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrOneParam;
    }
    else if (2 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrTwoParam;
    }
    else
    {
        dsiXfer.txDataType = kDSI_TxDataGenLongWr;
    }

    return device->xferFunc(&dsiXfer);
}
