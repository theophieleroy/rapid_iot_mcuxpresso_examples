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

#include "utilities/fsl_assert.h"
#include <string.h>
#include "i2c/master/fsl_i2c_master_driver.h"
#include "microseconds/microseconds.h"
#include "utilities/fsl_rtos_abstraction.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
uint32_t get_bus_clock(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for I2C instances. */
const uint32_t g_i2cBaseAddr[] = I2C_BASE_ADDRS;

/* Table to save i2c IRQ enum numbers defined in CMSIS header file. */
const IRQn_Type g_i2cIrqId[I2C_INSTANCE_COUNT] = {I2C0_IRQn,
#if !defined(KV46F15_SERIES)
                                                  I2C1_IRQn,
#endif
#if defined(K64F12_SERIES)
                                                  I2C2_IRQn
#endif
};

/* Pointer to runtime state structure.*/
void *g_i2cStatePtr[I2C_INSTANCE_COUNT] = {{NULL}};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterInit
 * Description   : initializes the I2C master mode driver.
 * This function will initialize the I2C master mode driver, enable I2C clock,
 * and enable I2C interrupt.
 *
 *END**************************************************************************/
void I2C_DRV_MasterInit(uint32_t instance, i2c_master_state_t *master)
{
    assert(master);
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];

    /* Exit if current instance is already initilized. */
    if (g_i2cStatePtr[instance])
    {
        return;
    }

    /* Init driver instance struct.*/
    memset(master, 0, sizeof(i2c_master_state_t));

    /* Create sem object for transfer. */
    OSA_SemaCreate(&master->irqSync, 0);

    /* Enable clock for I2C.*/
    if (instance == I2C0_IDX)
    {
        SIM_SET_SCGC4(SIM, SIM_SCGC4_I2C0_MASK);
    }
#if (I2C_INSTANCE_COUNT > 1)
    else if (instance == I2C1_IDX)
    {
        SIM_SET_SCGC4(SIM, SIM_SCGC4_I2C1_MASK);
    }
#endif
    /* Init peripheral to known state.*/
    I2C_HAL_Init(baseAddr);

    /* Save runtime structure poniter.*/
    g_i2cStatePtr[instance] = master;

    /* Enable I2C interrupt in NVIC level.*/
    if (instance == I2C0_IDX)
    {
        NVIC_EnableIRQ(I2C0_IRQn);
    }
#if (I2C_INSTANCE_COUNT > 1)
    else if (instance == I2C1_IDX)
    {
        NVIC_EnableIRQ(I2C1_IRQn);
    }
#endif

    /* Indicate I2C bus is idle. */
    master->i2cIdle = true;

    /* Enable module.*/
    I2C_HAL_Enable(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterDeinit
 * Description   : Deinit the I2C master mode driver.
 * This function will deinit the I2C master mode driver, disable I2C clock,
 * and disable I2C interrupt.
 *
 *END**************************************************************************/
void I2C_DRV_MasterDeinit(uint32_t instance)
{
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];

    /* Disable module.*/
    I2C_HAL_Disable(baseAddr);

    /* Disable clock for I2C.*/
    /* Disable I2C NVIC interrupt*/
    if (instance == I2C0_IDX)
    {
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_I2C0_MASK);
        NVIC_DisableIRQ(I2C0_IRQn);
    }
#if (I2C_INSTANCE_COUNT > 1)
    else if (instance == I2C1_IDX)
    {
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_I2C1_MASK);
        NVIC_DisableIRQ(I2C1_IRQn);
    }
#endif
    /* Cleared state pointer. */
    g_i2cStatePtr[instance] = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSetBaud
 * Description   : configures the I2C bus to access a device.
 * This function will set baud rate.
 *
 *END**************************************************************************/
void I2C_DRV_MasterSetBaud(uint32_t instance, const i2c_device_t *device)
{
    assert(device);
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];

    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    /* Set baud rate if different.*/
    if (device->baudRate_kbps != master->lastBaudRate_kbps)
    {
        /* Get the current bus clock.*/
        I2C_HAL_SetBaudRate(baseAddr, get_bus_clock(), device->baudRate_kbps, NULL);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterWait
 * Description   : Wait transfer to finish.
 * This function is a static function which will be called by other data
 * transaction APIs.
 *
 *END**************************************************************************/
static i2c_status_t I2C_DRV_MasterWait(uint32_t instance, uint32_t timeout_ms)
{
    assert(instance < I2C_INSTANCE_COUNT);

    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    osa_status_t syncStatus;

    do
    {
        syncStatus = OSA_SemaWait(&master->irqSync, timeout_ms);
    } while (syncStatus == kStatus_OSA_Idle);

    if (syncStatus != kStatus_OSA_Success)
    {
        master->status = kStatus_I2C_Timeout;
    }

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterWait
 * Description   : Wait transfer to finish.
 * This function is a static function which will be called by other data
 * transaction APIs.
 *
 *END**************************************************************************/
static i2c_status_t I2C_DRV_MasterWaitForBusyFree(uint32_t instance, uint32_t timeout_ms)
{
    assert(instance < I2C_INSTANCE_COUNT);

    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];
    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];

    uint64_t startTicks = microseconds_get_ticks();
    uint64_t timeOutTicks = microseconds_convert_to_ticks(timeout_ms * 1000);

    while (I2C_HAL_GetStatusFlag(baseAddr, kI2CBusBusy))
    {
        if (timeOutTicks && ((microseconds_get_ticks() - startTicks) >= timeOutTicks))
        {
            master->status = kStatus_I2C_BusBusy;
            break;
        }
    }

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SendAddress
 * Description   : Prepare and send out address buffer with interrupt.
 * This function is a static function which will be called by other data
 * transaction APIs.
 *
 *END**************************************************************************/
static i2c_status_t I2C_DRV_SendAddress(uint32_t instance,
                                        const i2c_device_t *device,
                                        uint8_t *cmdBuff,
                                        uint32_t cmdSize,
                                        i2c_direction_t direction,
                                        uint32_t timeout_ms)
{
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];
    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    uint8_t addrByte1, addrByte2, directionBit;
    bool is10bitAddr;
    uint8_t addrBuff[2] = {0};
    uint8_t addrSize = 0;

    /*--------------- Prepare Address Buffer ------------------*/
    /* Get r/w bit according to required direction.
     * read is 1, write is 0. */
    directionBit = (direction == kI2CReceive) ? 0x1U : 0x0U;

    /* Check to see if slave address is 10 bits or not. */
    is10bitAddr = ((device->address >> 10U) == 0x1EU) ? true : false;

    /* Get address byte 1 and byte 2 according address bit number. */
    if (is10bitAddr)
    {
        addrByte1 = (uint8_t)device->address >> 8U;
        addrByte2 = (uint8_t)device->address;
    }
    else
    {
        addrByte1 = (uint8_t)device->address;
    }

    /* Get the device address with r/w direction. If we have a sub-address,
      then that is always done as a write transfer prior to transferring
      the actual data.*/
    addrByte1 = addrByte1 << 1U;

    /* First need to write if 10-bit address or has cmd buffer. */
    addrByte1 |= (uint8_t)((is10bitAddr || cmdBuff) ? 0U : directionBit);

    /* Put slave address byte 1 into address buffer. */
    addrBuff[addrSize++] = addrByte1;

    if (is10bitAddr)
    {
        /* Put address byte 2 into address buffer. */
        addrBuff[addrSize++] = addrByte2;
    }

    /*--------------- Send Address Buffer ------------------*/
    master->txBuff = addrBuff;
    master->txSize = addrSize;

    /* Send first byte in address buffer to trigger interrupt.*/
    I2C_HAL_WriteByte(baseAddr, addrBuff[0]);

    /* Wait for the transfer to finish.*/
    I2C_DRV_MasterWait(instance, timeout_ms);

    /*--------------------- Send CMD -----------------------*/
    if ((master->status == kStatus_I2C_Success) && cmdBuff)
    {
        master->txBuff = cmdBuff;
        master->txSize = cmdSize;

        /* Send first byte in address buffer to trigger interrupt.*/
        I2C_HAL_WriteByte(baseAddr, *cmdBuff);

        /* Wait for the transfer to finish.*/
        I2C_DRV_MasterWait(instance, timeout_ms);
    }

    /* Send slave address again if receiving data from 10-bit address slave,*/
    /* OR conducting a cmd receive. */
    if ((master->status == kStatus_I2C_Success) && (direction == kI2CReceive) && (is10bitAddr || cmdBuff))
    {
        /* Need to send slave address again. */
        master->txSize = 1U;
        master->txBuff = NULL;

        /* Need to generate a repeat start before changing to receive. */
        I2C_HAL_SendStart(baseAddr);

        /* Send address byte 1 again. */
        I2C_HAL_WriteByte(baseAddr, (uint8_t)(addrByte1 | 1U));

        /* Wait for the transfer to finish.*/
        I2C_DRV_MasterWait(instance, timeout_ms);
    }

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSendDataBlocking
 * Description   : performs a blocking send transaction on the I2C bus.
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                            const i2c_device_t *device,
                                            uint8_t *cmdBuff,
                                            uint32_t cmdSize,
                                            uint8_t *txBuff,
                                            uint32_t txSize,
                                            uint32_t timeout_ms)
{
    assert(instance < I2C_INSTANCE_COUNT);
    // uint32_t i = 0;
    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];
    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    /* Return if there is already a trasction */
    if (!master->i2cIdle)
    {
        return master->status = kStatus_I2C_Busy;
    }
    else
    {
        master->i2cIdle = false;
    }

    master->txBuff = NULL;
    master->txSize = 0;
    master->rxBuff = NULL;
    master->rxBuff = 0;
    master->status = kStatus_I2C_Success;

    I2C_DRV_MasterSetBaud(instance, device);

    /* Set direction to send for sending of address and data. */
    I2C_HAL_SetDirMode(baseAddr, kI2CSend);

    /* Enable i2c interrupt.*/
    I2C_HAL_ClearInt(baseAddr);
    I2C_HAL_SetIntCmd(baseAddr, true);

    /* Generate start signal. */
    I2C_HAL_SendStart(baseAddr);

    /* Send out slave address. */
    I2C_DRV_SendAddress(instance, device, cmdBuff, cmdSize, kI2CSend, timeout_ms);

    /* Send out data in transmit buffer. */
    if (master->status == kStatus_I2C_Success)
    {
        /* Fill tx buffer and size to run-time structure. */
        master->txBuff = txBuff;
        master->txSize = txSize;

        /* Send first byte in transmit buffer to trigger interrupt.*/
        I2C_HAL_WriteByte(baseAddr, master->txBuff[0]);

        /* Wait for the transfer to finish.*/
        I2C_DRV_MasterWait(instance, timeout_ms);
    }

    /* Generate stop signal. */
    I2C_HAL_SendStop(baseAddr);

    /* Disable interrupt. */
    I2C_HAL_SetIntCmd(baseAddr, false);

    /* Indicate I2C bus is idle. */
    master->i2cIdle = true;

    /* Wait for the STOP signal finish. */
    I2C_DRV_MasterWaitForBusyFree(instance, timeout_ms);

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveDataBlocking
 * Description   : Performs a blocking receive transaction on the I2C bus.
 *
 *END**************************************************************************/
i2c_status_t I2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                               const i2c_device_t *device,
                                               uint8_t *cmdBuff,
                                               uint32_t cmdSize,
                                               uint8_t *rxBuff,
                                               uint32_t rxSize,
                                               uint32_t timeout_ms)
{
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];

    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    /* Return if there is already a trasction */
    if (!master->i2cIdle)
    {
        return master->status = kStatus_I2C_Busy;
    }

    master->rxBuff = rxBuff;
    master->rxSize = rxSize;
    master->txBuff = NULL;
    master->txSize = 0;
    master->status = kStatus_I2C_Success;

    I2C_DRV_MasterSetBaud(instance, device);

    /* Set direction to send for sending of address. */
    I2C_HAL_SetDirMode(baseAddr, kI2CSend);

    /* Enable i2c interrupt.*/
    I2C_HAL_ClearInt(baseAddr);
    I2C_HAL_SetIntCmd(baseAddr, true);

    /* Generate start signal. */
    I2C_HAL_SendStart(baseAddr);

    /* Send out slave address. */
    I2C_DRV_SendAddress(instance, device, cmdBuff, cmdSize, kI2CReceive, timeout_ms);

    /* Start to receive data. */
    if (master->status == kStatus_I2C_Success)
    {
        /* Change direction to receive. */
        I2C_HAL_SetDirMode(baseAddr, kI2CReceive);

        /* Send NAK if only one byte to read. */
        if (rxSize == 0x1U)
        {
            I2C_HAL_SendNak(baseAddr);
        }
        else
        {
            I2C_HAL_SendAck(baseAddr);
        }

        /* Dummy read to trigger receive of next byte in interrupt. */
        I2C_HAL_ReadByte(baseAddr);

        /* Wait for the transfer to finish.*/
        I2C_DRV_MasterWait(instance, timeout_ms);
    }

    /* The stop signal is send inside irq before reading the last byte. */
    /* Disable interrupt. */
    I2C_HAL_SetIntCmd(baseAddr, false);

    /* Indicate I2C bus is idle. */
    master->i2cIdle = true;

    /* Wait for the STOP signal finish. */
    I2C_DRV_MasterWaitForBusyFree(instance, timeout_ms);

    return master->status;
}

/*!
 * @brief I2C master IRQ handler.
 * This handler uses the buffers stored in the i2c_master_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void I2C_DRV_MasterIRQHandler(uint32_t instance)
{
    assert(instance < I2C_INSTANCE_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_i2cBaseAddr[instance];
    /* Go ahead and clear the interrupt flag.*/
    I2C_HAL_ClearInt(baseAddr);

    bool signalCompletion = false;

    /* Get current runtime structure. */
    i2c_master_state_t *master = (i2c_master_state_t *)g_i2cStatePtr[instance];

    /* Get current master transfer direction. */
    i2c_direction_t direction = I2C_HAL_GetDirMode(baseAddr);

    /* Exit immediately if there is no transfer in progress OR not in master mode.*/
    if ((!I2C_HAL_GetStatusFlag(baseAddr, kI2CBusBusy)) || (!I2C_HAL_IsMaster(baseAddr)))
    {
        return;
    }

    /* Handle send.*/
    if (direction == kI2CSend)
    {
        /* Check whether we got an ACK or NAK from the former byte we sent.*/
        if (I2C_HAL_GetStatusFlag(baseAddr, kI2CReceivedNak))
        {
            /* Record that we got a NAK.*/
            master->status = kStatus_I2C_ReceivedNak;

            /* Got a NAK, so we're done with this transfer.*/
            signalCompletion = true;
        }
        else
        {
            /* Continue send if still have data. txSize and txBuff index need an
             * increment first because one byte is already sent to trigger interrupt. */
            if (--master->txSize > 0)
            {
                /* Transmit next byte and update buffer index.*/
                I2C_HAL_WriteByte(baseAddr, *(++master->txBuff));
            }
            else
            {
                /* No data to send, finish. */
                signalCompletion = true;
            }
        }
    }

    /* Handle receive.*/
    if (direction == kI2CReceive)
    {
        switch (--master->rxSize)
        {
            case 0x0U:
                /* Generate stop signal. */
                I2C_HAL_SendStop(baseAddr);
                signalCompletion = true;
                break;
            case 0x1U:
                /* For the byte before last, we need to set NAK.*/
                I2C_HAL_SendNak(baseAddr);
                break;
            default:
                I2C_HAL_SendAck(baseAddr);
                break;
        }

        /* Read recently received byte into buffer and update buffer index.*/
        *(master->rxBuff++) = I2C_HAL_ReadByte(baseAddr);
    }

    /* Signal the sync object.*/
    if (signalCompletion)
    {
        OSA_SemaPost(&master->irqSync);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
