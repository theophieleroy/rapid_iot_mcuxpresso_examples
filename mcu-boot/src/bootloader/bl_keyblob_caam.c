/*
 * The Clear BSD License
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
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
#include <stdint.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "bootloader/bl_keyblob.h"

#define TRACE debug_printf
#define HAB_TRACE debug_printf

/* These defines could be customized */

#define INPUT_RING_ADDR (0x26000100)
#define OUTPUT_RING0_ADDR (0x26000200)
#define OUTPUT_RING1_ADDR (0x26000204)
#define JOB_DSC_ADDR (0x26000300)
#define DEK_BLOB_ADDR (0x26000400)

/* Data Encryption Key Example - i.e. the key will be wrapped in the blob */
static const uint8_t dek[16] = { 0xCC, 0x20, 0x02, 0x73, 0xFE, 0xED, 0x33, 0x2D,
                                 0x6B, 0xCB, 0x35, 0xB2, 0xAE, 0x09, 0x76, 0x84 };

/* This is the required header added to the DEK blob for encrypted boot */
uint8_t wrapped_key_hdr[8] = { 0x81, 0x00, 0x48, 0x41, 0x66, 0x55, 0x10, 0x00 };

uint32_t encap_dsc[] = { ENCAP_BLOB_DESC1, ENCAP_BLOB_DESC2, ENCAP_BLOB_DESC3, ENCAP_BLOB_DESC4, ENCAP_BLOB_DESC5,
                         ENCAP_BLOB_DESC6, ENCAP_BLOB_DESC7, ENCAP_BLOB_DESC8, ENCAP_BLOB_DESC9 };

uint32_t rng_inst_dsc[] = { RNG_INST_DESC1, RNG_INST_DESC2, RNG_INST_DESC3, RNG_INST_DESC4, RNG_INST_DESC5,
                            RNG_INST_DESC6, RNG_INST_DESC7, RNG_INST_DESC8, RNG_INST_DESC9 };

uint32_t hab_hal_read_register(uint32_t reg)
{
    uint32_t val = 0;

    val = *((const volatile uint32_t *)reg);

    return val;
}

void hab_hal_write_register(uint32_t reg, uint32_t val)
{
    __DSB();
    __ISB();

    *((volatile uint32_t *)reg) = (uint32_t)val;

    __DSB();
    __ISB();
}

/*!
 * Secure memory run command.
 *
 * @param   sec_mem_cmd  Secure memory command register
 * @return  cmd_status  Secure memory command status register
 */
uint32_t secmem_set_cmd(uint32_t sec_mem_cmd)
{
    uint32_t temp_reg;
    HAB_ENG_CAAM_WRITE(CAAM_SMCJR0, sec_mem_cmd);
    do
    {
        temp_reg = HAB_ENG_CAAM_READ(CAAM_SMCSJR0);
    } while (temp_reg & CMD_COMPLETE);

    return temp_reg;
}

/*!
 * CAAM page allocation.
 *
 * @param   page  Number of the page to allocate.
 * @param   partition  Number of the partition to allocate.
 */
static uint32_t caam_page_alloc(uint8_t page_num, uint8_t partition_num)
{
    uint32_t temp_reg;

    /*
     * De-Allocate partition_num if already allocated to ARM core
     */
    if (HAB_ENG_CAAM_READ(CAAM_SMPO_0) & PARTITION_OWNER(partition_num))
    {
        temp_reg = secmem_set_cmd(PARTITION(partition_num) | CMD_PART_DEALLOC);
        if (temp_reg & SMCSJR_AERR)
        {
            TRACE("Error: De-allocation status 0x%X\n", temp_reg);
            return ERROR_IN_PAGE_ALLOC;
        }
    }

    /* set the access rights to allow full access */
    HAB_ENG_CAAM_WRITE(CAAM_SMAG1JR0(partition_num), 0xF);
    HAB_ENG_CAAM_WRITE(CAAM_SMAG2JR0(partition_num), 0xF);
    HAB_ENG_CAAM_WRITE(CAAM_SMAPJR0(partition_num), 0xFF);

    /* Now need to allocate partition_num of secure RAM. */
    /* De-Allocate page_num by starting with a page inquiry command */
    temp_reg = secmem_set_cmd(PAGE(page_num) | CMD_INQUIRY);
    /* if the page is owned, de-allocate it */
    if ((temp_reg & SMCSJR_PO) == PAGE_OWNED)
    {
        temp_reg = secmem_set_cmd(PAGE(page_num) | CMD_PAGE_DEALLOC);
        if (temp_reg & SMCSJR_AERR)
        {
            TRACE("Error: Allocation status 0x%X\n", temp_reg);
            return ERROR_IN_PAGE_ALLOC;
        }
    }

    /* Allocate page_num to partition_num */
    temp_reg = secmem_set_cmd(PAGE(page_num) | PARTITION(partition_num) | CMD_PAGE_ALLOC);
    if (temp_reg & SMCSJR_AERR)
    {
        TRACE("Error: Allocation status 0x%X\n", temp_reg);
        return ERROR_IN_PAGE_ALLOC;
    }
    /* page inquiry command to ensure that the page was allocated */
    temp_reg = secmem_set_cmd(PAGE(page_num) | CMD_INQUIRY);
    /* if the page is not owned => problem */
    if ((temp_reg & SMCSJR_PO) != PAGE_OWNED)
    {
        TRACE("Error: Allocation of page %d in partition %d failed 0x%X\n", temp_reg, page_num, partition_num);

        return ERROR_IN_PAGE_ALLOC;
    }

    return SUCCESS;
}

/*!
 * Use CAAM to generate a blob.
 *
 * @param   plain_data_addr  Location address of the plain data.
 * @param   blob_addr  Location address of the blob.
 *
 * @return  SUCCESS or ERROR_XXX
 */
uint32_t caam_gen_blob(uint32_t plain_data_addr, uint32_t blob_addr)
{
    uint32_t ret = SUCCESS;

    uint32_t *pEncapDsc = (uint32_t *)0x26000300;
    uint32_t i;

    for (i = 0; i < 9; i++)
    {
        *pEncapDsc++ = encap_dsc[i];
    }

    pEncapDsc = (uint32_t *)0x26000300;

    /* Buffer to hold the resulting blob */
    uint8_t *blob = (uint8_t *)blob_addr;

    /* initialize the blob array */
    memset(blob, 0, 64);

    HAB_ENG_CAAM_READ(CAAM_STA);

    /**** Prepare partition and page, and start the job to create the blob ***/
    caam_page_alloc(PAGE_1, PARTITION_1);
    if (ret != SUCCESS)
        return ret;

    /* Write the DEK to the partition. */
    memcpy((uint32_t *)SEC_MEM_PAGE1, (uint32_t *)plain_data_addr, 16);

    /* Now configure the access rights of the partition */
    HAB_ENG_CAAM_WRITE(CAAM_SMAG1JR0(PARTITION_1), KS_G1); // set group 1
    HAB_ENG_CAAM_WRITE(CAAM_SMAG2JR0(PARTITION_1), 0);     // clear group 2
    HAB_ENG_CAAM_WRITE(CAAM_SMAPJR0(PARTITION_1), PERM);   // set permissions & locks

    /* Fill in the address where the DEK resides */
    *(pEncapDsc + 5) = (uint32_t)SEC_MEM_PAGE1;
    /* Fill in output blob addr in encap_dsc */
    *(pEncapDsc + 7) = (uint32_t)blob;

    /* Run descriptor with result written to blob buffer */
    /* Add job to input ring */
    *((uint32_t *)0x26000100) = (uint32_t)pEncapDsc;

    for (i = 0; i < 9; i++)
    {
        TRACE("BLOB DSC i=%d DEC[%d] = 0x%x\n", i, i, *(uint32_t *)(0x26000300 + i * 4));
    }

    /* Increment jobs added */
    HAB_ENG_CAAM_WRITE(CAAM_IRJAR0, 1);

    /* Wait for job ring to complete the job: 1 completed job expected */
    while (HAB_ENG_CAAM_READ(CAAM_ORSFR0) != 1)
        ;
#if 0
    uint32_t val;
    val = *((const volatile UINT32*)(CAAM_ORSFR0));
    while(val != 1)
    {
        val = *((const volatile UINT32*)(CAAM_ORSFR0));
    }
#endif

    /* check that descriptor address is the one expected in the output ring */
    if (*((uint32_t *)0x26000200) == (uint32_t)pEncapDsc)
    {
        /* check if any error is reported in the output ring */
        if ((*((uint32_t *)0x26000204) & JOB_RING_STS) != 0)
        {
            TRACE("Error: blob encap job completed with errors 0x%X\n", *((uint32_t *)0x26000204));
        }
    }
    else
    {
        TRACE(
            "Error: blob encap job output ring descriptor address does"
            " not match\n");
    }

    /* Remove job from Job Ring Output Queue */
    HAB_ENG_CAAM_WRITE(CAAM_ORJRR0, 1);

    return ret;
}

/*!
 * Initialize the CAAM.
 */
void caam_open(void)
{
    uint32_t temp_reg;
    uint32_t *pRngDsc = (uint32_t *)0x26000300;
    uint32_t i;

    for (i = 0; i < 9; i++)
    {
        *pRngDsc++ = rng_inst_dsc[i];
    }

    /* MID for CAAM - already done by HAB in ROM during preconfigure,
     * That is JROWN for JR0/1 = 1 (TZ, Secure World, ARM)
     * JRNSMID and JRSMID for JR0/1 = 2 (TZ, Secure World, CAAM)
     *
     * However, still need to initialize Job Rings as these are torn
     * down by HAB for each command
     */

    /* Initialize job ring addresses */

    HAB_ENG_CAAM_WRITE(CAAM_IRBAR0, (uint32_t)(0x26000100)); /* input ring address */
    HAB_ENG_CAAM_WRITE(CAAM_ORBAR0, (uint32_t)(0x26000200)); /* output ring address */

    /* Initialize job ring sizes to 1 */
    HAB_ENG_CAAM_WRITE(CAAM_IRSR0, JOB_RING_ENTRIES);
    HAB_ENG_CAAM_WRITE(CAAM_ORSR0, JOB_RING_ENTRIES);

    /* HAB disables interrupts for JR0 so do the same here */
    temp_reg = HAB_ENG_CAAM_READ(CAAM_JRCFGR0_LS) | JRCFG_LS_IMSK;
    HAB_ENG_CAAM_WRITE(CAAM_JRCFGR0_LS, temp_reg);

    /********* Initialize and instantiate the RNG *******************/
    /* if RNG already instantiated then skip it */
    if ((HAB_ENG_CAAM_READ(CAAM_RDSTA) & RDSTA_IF0) != RDSTA_IF0)
    {
        HAB_ENG_CAAM_READ(CAAM_STA);

        /* Enter TRNG Program mode */
        temp_reg = HAB_ENG_CAAM_READ(CAAM_RTMCTL) | RTMCTL_PGM | 0x0040;
        HAB_ENG_CAAM_WRITE(CAAM_RTMCTL, temp_reg);
#if 0    
        /* Set OSC_DIV field to TRNG */
        temp_reg = HAB_ENG_CAAM_READ(CAAM_RTMCTL) | (RNG_TRIM_OSC_DIV << 2);
        HAB_ENG_CAAM_WRITE(CAAM_RTMCTL,temp_reg);
    
        /* Set delay */
        HAB_ENG_CAAM_WRITE(CAAM_RTSDCTL, ((RNG_TRIM_ENT_DLY << 16) | 0x09C4));   
        HAB_ENG_CAAM_WRITE(CAAM_RTFRQMIN, (RNG_TRIM_ENT_DLY >> 1));
        HAB_ENG_CAAM_WRITE(CAAM_RTFRQMAX, (RNG_TRIM_ENT_DLY << 3));
#endif
        /* Resume TRNG Run mode */
        temp_reg = HAB_ENG_CAAM_READ(CAAM_RTMCTL) ^ RTMCTL_PGM;
        HAB_ENG_CAAM_WRITE(CAAM_RTMCTL, temp_reg);

        /* Clear the ERR bit in RTMCTL if set. The TRNG error can occur when the
        * RNG clock is not within 1/2x to 8x the system clock.
        * This error is possible if ROM code does not initialize the system PLLs
        * immediately after PoR.
        */
        temp_reg = HAB_ENG_CAAM_READ(CAAM_RTMCTL) | RTMCTL_ERR;
        HAB_ENG_CAAM_WRITE(CAAM_RTMCTL, temp_reg);

        for (i = 0; i < 9; i++)
        {
            TRACE("RNG DSC i=%d DEC[%d] = 0x%x\n", i, i, *(uint32_t *)(0x26000300 + i * 4));
        }

        /* Run descriptor to instantiate the RNG */
        /* Add job to input ring */
        *((uint32_t *)0x26000100) = (uint32_t)0x26000300;
        /* Increment jobs added */
        HAB_ENG_CAAM_WRITE(CAAM_IRJAR0, 1);

        /* Wait for job ring to complete the job: 1 completed job expected */
        while (HAB_ENG_CAAM_READ(CAAM_ORSFR0) != 1)
            ;

        /* check that descriptor address is the one expected in the out ring */
        if (*((uint32_t *)0x26000200) == (uint32_t)(0x26000300))
        {
            /* check if any error is reported in the output ring */
            if ((*((uint32_t *)0x26000204) & JOB_RING_STS) != 0)
            {
                TRACE("Error: RNG instantiation job completed with errors 0x%X\n", *((uint32_t *)0x26000204));
            }
            else
            {
                HAB_TRACE("RNG instantiate SUCCESS !!!\n");
            }
        }
        else
        {
            TRACE("Error: RNG job output ring descriptor address does not match\n");
        }

        /* ensure that the RNG was correctly instantiated */
        temp_reg = HAB_ENG_CAAM_READ(CAAM_RDSTA);
        if (temp_reg != (RDSTA_IF0 | RDSTA_SKVN))
        {
            TRACE("Error: RNG instantiation failed 0x%X\n", temp_reg);
        }

        /* Remove job from Job Ring Output Queue */
        HAB_ENG_CAAM_WRITE(CAAM_ORJRR0, 1);
    }

    return;
}

void generate_key_blob_test(void)
{
    generate_key_blob((uint32_t *)&dek[0], (uint8_t *)0x8000);
}

/*!
 * CAAM blob test for encrypted boot.
 *
 * @return  0
 */
int32_t generate_key_blob(uint32_t *key_addr, uint8_t *key_blob_addr)
{
    uint8_t i;
    uint8_t *ptr;
    uint32_t ret = SUCCESS;

    debug_printf("Start CAAM blob generation:\n");

    caam_open();

    /* Print the DEK that is going to be used */
    for (i = 0; i < 16; i++)
    {
        TRACE("%02X ", ((uint8_t *)key_addr)[i]);
    }
    TRACE("\n\n");

    /* Generate the blob to encapsulate the DEK */
    ret = caam_gen_blob((uint32_t)key_addr, (uint32_t)(DEK_BLOB_ADDR + 8));
    if (ret != SUCCESS)
        TRACE("Error during blob decap operation: 0x%d\n", ret);

    /* Copy the header into the DEK blob buffer */
    memcpy((uint32_t *)DEK_BLOB_ADDR, wrapped_key_hdr, sizeof(wrapped_key_hdr));

    /* Print the generated key blob */
    HAB_TRACE("DEK blob is available at 0x%08X and equals:\n", (int)DEK_BLOB_ADDR);
    for (i = 0; i < (8 + 64); i++)
    {
        HAB_TRACE("%02X ", ((uint8_t *)DEK_BLOB_ADDR)[i]);
    }

    TRACE("\n\n");
    TRACE(
        "Note: the first 8 Bytes are the required header and wrp_dat of"
        " the wrapped key structure!\n\n");

    /* Write out the key blob */
    ptr = key_blob_addr;
    for (i = 0; i < (8 + 64); i++)
    {
        *ptr++ = ((uint8_t *)DEK_BLOB_ADDR)[i];
    }

    return ret;
}
