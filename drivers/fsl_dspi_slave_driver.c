/*
 * The Clear BSD License
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "bootloader/bl_peripheral_interface.h"
#include "bootloader/bl_irq_common.h"
#include "fsl_device_registers.h"
#include "dspi/fsl_dspi_slave_driver.h"
#include "dspi/hal/fsl_dspi_hal.h"
#include "fsl_dspi_shared_irqs.h"
#include <string.h>
#include "utilities/fsl_assert.h"

#if BL_CONFIG_DSPI

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! SPI slave constants */
enum _spi_slave_constants
{
    kEmptyChar = 0, /*!< Empty character */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
const uint32_t g_dspiBaseAddr[] = SPI_BASE_ADDRS;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief DSPI Slave Generic IRQ handler.
 *
 * This handler uses the callbacks stored in the dspi_slave_state_t struct to transfer data
 * either from the data source or to the data sink functions.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void dspi_slave_irq_handler(void *state)
{
    /* instantiate local variable of type dspi_slave_state_t and equate it to the
     * pointer to state
     */
    dspi_slave_state_t *dspiState = (dspi_slave_state_t *)state;

    uint32_t instance = dspiState->instance;
    uint32_t baseAddr = g_dspiBaseAddr[instance];

    /* Get the callback pointers from the run-time state structure */
    dspi_slave_callbacks_t *callbacks = &dspiState->callbacks;

    /* catch tx fifo underflow conditions */
    if (dspi_hal_get_status_flag((SPI_Type *)baseAddr, kDspiTxFifoUnderflow))
    {
        /* Report SPI slave  transmit underrun error */
        if (callbacks->onError)
        {
            callbacks->onError(kStatus_DSPI_SlaveTxUnderrun, instance);
        }
    }

    /* Fill the tx fifo, where the fifo can be 1 entry or more */
    while (dspi_hal_get_status_flag((SPI_Type *)baseAddr, kDspiTxFifoFillRequest))
    {
        /* SPI transmit interrupt */
        uint32_t sourceWord = kEmptyChar;
        uint8_t sourceWordTemp;

        /* get the first 8-bits of data */
        callbacks->dataSource(&sourceWordTemp, instance);
        sourceWord = sourceWordTemp;

        /* See if the bits/frame is greater than one byte */
        if (dspiState->bitsPerFrame > 8)
        {
            callbacks->dataSource(&sourceWordTemp, instance);
            sourceWord |= (uint32_t)sourceWordTemp << 8U;
        }
        /* See if the bits/frame is greater than two bytes */
        if (dspiState->bitsPerFrame > 16)
        {
            callbacks->dataSource(&sourceWordTemp, instance);
            sourceWord |= (uint32_t)sourceWordTemp << 16U;
        }
        /* See if the bits/frame is greater than three bytes */
        if (dspiState->bitsPerFrame > 24)
        {
            callbacks->dataSource(&sourceWordTemp, instance);
            sourceWord |= (uint32_t)sourceWordTemp << 24U;
        }

        /* Finally, write the data to the DSPI data register */
        dspi_hal_write_data_slave_mode((SPI_Type *)baseAddr, sourceWord);

        /* try to clear TFFF by writing a one to it; it will not clear if TX FIFO not full */
        dspi_hal_clear_status_flag((SPI_Type *)baseAddr, kDspiTxFifoFillRequest);
    }

    /* Fill the rx fifo, where the fifo can be 1 entry or more */
    while (dspi_hal_get_status_flag((SPI_Type *)baseAddr, kDspiRxFifoDrainRequest))
    {
        /* SPI receive interrupt, read the data from the DSPI data register */
        uint32_t readData = dspi_hal_read_data((SPI_Type *)baseAddr);

        /* clear the rx fifo drain request, needed for non-DMA applications as this flag
         * will remain set even if the rx fifo is empty. By manually clearing this flag, it
         * either remain clear if no more data is in the fifo, or it will set if there is
         * more data in the fifo.
         */
        dspi_hal_clear_status_flag((SPI_Type *)baseAddr, kDspiRxFifoDrainRequest);

        /* Sink the first 8-bits */
        callbacks->dataSink((uint8_t)readData, instance);

        /* See if the bits/frame is greater than one byte */
        if (dspiState->bitsPerFrame > 8)
        {
            /* Sink the next 8-bits */
            callbacks->dataSink((uint8_t)(readData >> 8), instance);
        }
        /* See if the bits/frame is greater than two bytes */
        if (dspiState->bitsPerFrame > 16)
        {
            /* Sink the next 8-bits */
            callbacks->dataSink((uint8_t)(readData >> 16), instance);
        }
        /* See if the bits/frame is greater than three bytes */
        if (dspiState->bitsPerFrame > 24)
        {
            /* Sink the next 8-bits */
            callbacks->dataSink((uint8_t)(readData >> 24), instance);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_slave_init
 * Description   : Initialize a DSPI instance for slave mode operation.
 * This function saves the callbacks to the run-time state structure for later use in the
 * interrupt handler. It also ungates the clock to the DSPI module, initializes the DSPI
 * for slave mode, enables the module and corresponding interrupts. Once initialized, the
 * DSPI module is configured in slave mode and ready to receive data from a SPI master. The
 * following is an example of how to set up the dspi_slave_state_t and the dspi_slave_user_config_t
 * parameters and how to call the dspi_slave_init function by passing in these parameters:
 *   dspi_slave_state_t dspiSlaveState; <- the user simply allocates memory for this struct
 *   dspi_slave_user_config_t slaveUserConfig;
 *   slaveUserConfig.callbacks.dataSink = data_sink; <- set to user implementation of function
 *   slaveUserConfig.callbacks.dataSource = data_source; <- set to user implementation of function
 *   slaveUserConfig.callbacks.onError = on_error; <- set to user implementation of function
 *   slaveUserConfig.dataConfig.bitsPerFrame = 16;
 *   slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
 *   slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
 *   dspi_slave_init(slaveInstance, &slaveUserConfig, &dspiSlaveState);
 *
 *END**************************************************************************/
status_t dspi_slave_init(uint32_t instance, const dspi_slave_user_config_t *slaveConfig, dspi_slave_state_t *dspiState)
{
    assert(slaveConfig);
    assert(instance < SPI_INSTANCE_COUNT);

    uint32_t baseAddr = g_dspiBaseAddr[instance];
    status_t errorCode = kStatus_Success;

    /* DSPI config struct in hal, fill out it's members below */
    dspi_slave_config_t dspiConfig;

    /* Clear the run-time state struct for this instance. */
    memset(dspiState, 0, sizeof(*dspiState));

    /* Save the application info. */
    dspiState->callbacks = slaveConfig->callbacks;

    /* configure the run-time state struct with the instance number */
    dspiState->instance = instance;

    /* configure the run-time state struct with the nubmer of bits/frame */
    dspiState->bitsPerFrame = slaveConfig->dataConfig.bitsPerFrame;

    /* Enable clock for DSPI */
    switch (instance)
    {
        case 0:
            SIM_SET_SCGC6(SIM, SIM_SCGC6_SPI0_MASK);
            break;
#if (SPI_INSTANCE_COUNT > 1U)
        case 1:
            SIM_SET_SCGC6(SIM, SIM_SCGC6_SPI1_MASK);
            break;
#if (SPI_INSTANCE_COUNT > 2U)
        case 2:
            SIM_SET_SCGC3(SIM, SIM_SCGC3_SPI2_MASK);
            break;
#endif // (SPI_INSTANCE_COUNT > 2U)
#endif // (SPI_INSTANCE_COUNT > 1U)
    }

    /* Reset the DSPI module */
    dspi_hal_reset((SPI_Type *)baseAddr);

    /* Initialize the parameters of the hal DSPI config structure with desired data
     * members of the user config struct, then the hal init function will be called
     */
    dspiConfig.isEnabled = false;        /* disable the DSPI module while we're setting it up */
    dspiConfig.isTxFifoDisabled = false; /* enable tx fifo */
    dspiConfig.isRxFifoDisabled = false; /* enable rx fifo */
    /* data format field config */
    dspiConfig.dataConfig.bitsPerFrame = slaveConfig->dataConfig.bitsPerFrame;
    dspiConfig.dataConfig.clkPolarity = slaveConfig->dataConfig.clkPolarity;
    dspiConfig.dataConfig.clkPhase = slaveConfig->dataConfig.clkPhase;

    errorCode = dspi_hal_slave_init((SPI_Type *)baseAddr, &dspiConfig);
    if (errorCode != kStatus_Success)
    {
        return errorCode; /* return immediately if there's a problem with the init */
    }

    /* DSPI system enable */
    dspi_hal_enable((SPI_Type *)baseAddr);

    /* flush the fifos*/
    dspi_hal_flush_fifos((SPI_Type *)baseAddr, true, true);

    /* Configure IRQ state structure, so irq handler can point to the correct state structure */
    dspi_set_shared_irq_state(instance, dspiState, false);

    /* TX FIFO Fill Flag (TFFF) request enable*/
    dspi_hal_configure_interrupt((SPI_Type *)baseAddr, kDspiTxFifoFillRequest, true);
    /* RX FIFO Drain request: RFDF_RE to enable RFDF interrupt */
    dspi_hal_configure_interrupt((SPI_Type *)baseAddr, kDspiRxFifoDrainRequest, true);
    /* TX FIFO underflow request enable*/
    dspi_hal_configure_interrupt((SPI_Type *)baseAddr, kDspiTxFifoUnderflow, true);

    /* Write 0 to tx shift register so a master will receive a known first word (0). */
    dspi_hal_write_data_slave_mode((SPI_Type *)baseAddr, 0);

    /* Clear the Tx FIFO Fill Flag (TFFF) status bit
     * This should be done after writing the data to the PUSHR register
     */
    dspi_hal_clear_status_flag((SPI_Type *)baseAddr, kDspiTxFifoFillRequest);

    /* Enable the interrupt */
    dspi_set_system_IRQ_gate(instance, kPeripheralEnableIRQ);

    /* Start DSPI transfers, set to running state */
    dspi_hal_start_transfer((SPI_Type *)baseAddr);

    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_slave_shutdown
 * Description   : Shutdown a DSPI instance.
 * Resets the DSPI peripheral, disables the interrupt to the core, and gates its clock.
 *
 *END**************************************************************************/
void dspi_slave_shutdown(dspi_slave_state_t *dspiState)
{
    uint32_t instance = dspiState->instance;

    assert(instance < SPI_INSTANCE_COUNT);

    uint32_t baseAddr = g_dspiBaseAddr[instance];

    /* if SPI is not clocked exit, if so disable the interrupt and proceed*/
    switch (instance)
    {
        case 0:
            if (!(SIM_RD_SCGC6(SIM) & SIM_SCGC6_SPI0_MASK))
            {
                return;
            }
            dspi_set_system_IRQ_gate(0, kPeripheralDisableIRQ);
            break;
#if (SPI_INSTANCE_COUNT > 1U)
        case 1:
            if (!(SIM_RD_SCGC6(SIM) & SIM_SCGC6_SPI1_MASK))
            {
                return;
            }
            dspi_set_system_IRQ_gate(1, kPeripheralDisableIRQ);
            break;
#if (SPI_INSTANCE_COUNT > 2U)
        case 2:
            if (!(SIM_RD_SCGC3(SIM) & SIM_SCGC3_SPI2_MASK))
            {
                return;
            }
            dspi_set_system_IRQ_gate(2, kPeripheralDisableIRQ);
            break;
#endif // (SPI_INSTANCE_COUNT > 2U)
#endif // (SPI_INSTANCE_COUNT > 1U)
    }

    /* Restore the module to defaults then power it down. This also disables the DSPI module. */
    dspi_hal_reset((SPI_Type *)baseAddr);

    /* Gate the clock for DSPI. */
    switch (instance)
    {
        case 0:
            SIM_CLR_SCGC6(SIM, SIM_SCGC6_SPI0_MASK);
            break;
#if (SPI_INSTANCE_COUNT > 1U)
        case 1:
            SIM_CLR_SCGC6(SIM, SIM_SCGC6_SPI1_MASK);
            break;
#if (SPI_INSTANCE_COUNT > 2U)
        case 2:
            SIM_CLR_SCGC3(SIM, SIM_SCGC3_SPI2_MASK);
            break;
#endif // (SPI_INSTANCE_COUNT > 2U)
#endif // (SPI_INSTANCE_COUNT > 1U)
    }
}

#endif // BL_CONFIG_DSPI

/*******************************************************************************
 * EOF
 ******************************************************************************/
