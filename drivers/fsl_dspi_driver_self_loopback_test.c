/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#include <stdlib.h>
#include "utilities/fsl_assert.h"
#include <string.h>
#include "dspi/fsl_dspi_types.h"
#include "dspi/fsl_dspi_master_driver.h"
#include "fsl_dspi_driver_unit_test.h"
#include "clock/fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TRANSFER_COUNT 32

/* for K64 tower board, use SPI1 and SPI2*/
/* SPI1 is available on header J3, short J3.10 to J3.9*/
/* SPI2 is available on R105-R109, short R108 to R109*/
#if defined(CPU_MK64FN1M0VMD12)
#define INSTANCE_START 1
#define INSTANCE_END 2
#endif

/* for K70 tower board, only SPI2 is available*/
/* SPI2 is available on header J5, short J5.10 to J5.9*/
#if defined(CPU_MK70FN1M0VMJ12)
#define INSTANCE_START 2
#define INSTANCE_END 2
#endif

/* for K22 tower board, only SPI1 is available*/
/* SPI1 is available on header J5, short J5.10 to J5.9*/
#if defined(CPU_MK22FN512VMC12)
#define INSTANCE_START 1
#define INSTANCE_END 1
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Test function for dspi peripheral driver test.
 *        "self" loopback test, mainly testing SOUT-to-SIN, master mode only
 *        On the board, be sure to conect SOUT-to-SIN
 *        For board specific connections, see above
 *
 *
 */
uint32_t dspi_self_loopback_test(void)
{
    uint32_t failCount = 0;
    uint32_t i;
    dspi_master_user_config_t userConfig;
    dspi_device_t spiDevice;
    dspi_master_state_t dspiMasterState;
    uint32_t calculatedBaudRate;
    uint32_t instance;
    uint8_t sendBuffer[TRANSFER_COUNT] = {0};
    uint8_t receiveBuffer[TRANSFER_COUNT] = {0};
    uint32_t timeout = 1;
    uint32_t transferByteCount;

    /* fill up the send buffer*/
    for (i = 0; i < TRANSFER_COUNT; i++)
    {
        sendBuffer[i] = i;
    }

    /* test various instances if available*/
    for (instance = INSTANCE_START; instance <= INSTANCE_END; instance++)
    {
        /* configure dspi pin mux*/
        dspi_test_pinmux_setup(instance);

        if (instance == 1)
        {
            spiDevice.bitsPerSec = 500000;
        }

        if (instance == 2)
        {
            spiDevice.bitsPerSec = 5000000;
        }

        /* configure the members of the user config*/
        userConfig.isChipSelectContinuous = true;
        userConfig.isSckContinuous = false;
        userConfig.pcsPolarity = kDspiPcs_ActiveLow;
        userConfig.whichCtar = kDspiCtar0;
        userConfig.whichPcs = kDspiPcs0;

        /* config member of the spi device config*/
        spiDevice.dataBusConfig.bitsPerFrame = 8;
        spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
        spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
        spiDevice.dataBusConfig.direction = kDspiMsbFirst;

        /* init the dspi module*/
        dspi_master_init(instance, &dspiMasterState, &userConfig, &calculatedBaudRate);

        /* configure the spi bus*/
        if (dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate) != kStatus_Success)
        {
            failCount++;
        }
        if (calculatedBaudRate > spiDevice.bitsPerSec)
        {
            failCount++;
        }

        /***************************************************************************
         * Sync test
         **************************************************************************/
        /* test various transfer counts sizes to cover various cases*/
        for (transferByteCount = 1; transferByteCount <= 32; transferByteCount++)
        {
            /* now transfer data, using blocking method*/
            if (dspi_master_transfer(&dspiMasterState, &spiDevice, sendBuffer, receiveBuffer, transferByteCount,
                                     timeout) != kStatus_Success)
            {
                failCount++;
            }

            /* now check the read buffer*/
            for (i = 0; i < transferByteCount; i++)
            {
                if (receiveBuffer[i] != sendBuffer[i])
                {
                    failCount++;
                }
            }
        }

        /***************************************************************************
         * Async test
         **************************************************************************/
        /* test various transfer counts sizes to cover various cases*/
        for (transferByteCount = 1; transferByteCount <= 32; transferByteCount++)
        {
            /* now transfer data, using blocking method*/
            if (dspi_master_transfer_async(&dspiMasterState, &spiDevice, sendBuffer, receiveBuffer,
                                           transferByteCount) != kStatus_Success)
            {
                failCount++;
            }

            /* Because of aync mode, we have to ping the master driver to
             * see when the transfer is complete, put NULL for transfer count since we don't care
             */
            while (dspi_master_get_transfer_status(&dspiMasterState, NULL) == kStatus_DSPI_Busy)
            {
            }

            /* now check the read buffer*/
            for (i = 0; i < transferByteCount; i++)
            {
                if (receiveBuffer[i] != sendBuffer[i])
                {
                    failCount++;
                }
            }
        }

        /***************************************************************************
         * Change bit count per frame test, blocking test
         **************************************************************************/
        spiDevice.dataBusConfig.bitsPerFrame = 16;
        transferByteCount = 32; // 32 bytes or (16) 16-bit words

        /* now transfer data, using blocking method*/
        if (dspi_master_transfer(&dspiMasterState, &spiDevice, sendBuffer, receiveBuffer, transferByteCount, timeout) !=
            kStatus_Success)
        {
            failCount++;
        }

        /* now check the read buffer*/
        for (i = 0; i < transferByteCount; i++)
        {
            if (receiveBuffer[i] != sendBuffer[i])
            {
                failCount++;
            }
        }

        /***************************************************************************
         * Change bit count per frame test, non-blocking (aysnc) test
         **************************************************************************/
        spiDevice.dataBusConfig.bitsPerFrame = 11; /* set bit/frame to something odd */

        /* fill up the send buffer, where every other byte with "bit/frame-8" bits */
        for (i = 0; i < TRANSFER_COUNT; i += 2)
        {
            sendBuffer[i] = i;
            sendBuffer[i + 1] = (i + 1) & (0x7);
        }

        /* now transfer data, using blocking method*/
        if (dspi_master_transfer_async(&dspiMasterState, &spiDevice, sendBuffer, receiveBuffer, transferByteCount) !=
            kStatus_Success)
        {
            failCount++;
        }

        /* Because of aync mode, we have to ping the master driver to
         * see when the transfer is complete, put NULL for transfer count since we don't care
         */
        while (dspi_master_get_transfer_status(&dspiMasterState, NULL) == kStatus_DSPI_Busy)
        {
        }

        /* now check the read buffer*/
        for (i = 0; i < transferByteCount; i++)
        {
            if (receiveBuffer[i] != sendBuffer[i])
            {
                failCount++;
            }
        }
    }

    return failCount;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
