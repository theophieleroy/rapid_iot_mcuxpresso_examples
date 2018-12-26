/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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

#include "fsl_caam.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.caam"
#endif


/*! Compile time sizeof() check */
#define BUILD_ASSURE(condition, msg) extern int msg[1 - 2 * (!(condition))] __attribute__((unused))

/*! AESA XCBC-MAC or CMAC request CLASS 1 (default) or CLASS 2 CHA */
#ifndef CAAM_AES_MAC_CLASS
#define CAAM_AES_MAC_CLASS_1 0x02000000u
#define CAAM_AES_MAC_CLASS_2 0x04000000u
#define CAAM_AES_MAC_CLASS CAAM_AES_MAC_CLASS_1
#endif

/*! IRBAR and ORBAR job ring registers are 64-bit. these macros access least significant address 32-bit word. */
#define IRBAR0 *(((volatile uint32_t *)&(base->JOBRING[0].IRBAR_JR)) + 1)
#define ORBAR0 *(((volatile uint32_t *)&(base->JOBRING[0].ORBAR_JR)) + 1)
#define IRBAR1 *(((volatile uint32_t *)&(base->JOBRING[1].IRBAR_JR)) + 1)
#define ORBAR1 *(((volatile uint32_t *)&(base->JOBRING[1].ORBAR_JR)) + 1)

typedef enum _caam_algorithm
{
    kCAAM_AlgorithmAES = 0x10u << 16,
    kCAAM_AlgorithmDES = 0x20u << 16,
    kCAAM_Algorithm3DES = 0x21u << 16,
    kCAAM_AlgorithmSHA1 = 0x41u << 16,
    kCAAM_AlgorithmSHA224 = 0x42u << 16,
    kCAAM_AlgorithmSHA256 = 0x43u << 16,
    kCAAM_AlgorithmSHA384 = 0x44u << 16,
    kCAAM_AlgorithmSHA512 = 0x45u << 16,
} caam_algorithm_t;

typedef enum _caam_aai_symmetric_alg
{
    kCAAM_ModeCTR = 0x00U << 4,
    kCAAM_ModeCBC = 0x10U << 4,
    kCAAM_ModeECB = 0x20U << 4,
    kCAAM_ModeCFB = 0x30U << 4,
    kCAAM_ModeOFB = 0x40U << 4,
    kCAAM_ModeCMAC = 0x60U << 4,
    kCAAM_ModeXCBCMAC = 0x70U << 4,
    kCAAM_ModeCCM = 0x80U << 4,
    kCAAM_ModeGCM = 0x90U << 4,
} caam_aai_symmetric_alg_t;

typedef enum _caam_algorithm_state
{
    kCAAM_AlgStateUpdate = 0u,
    kCAAM_AlgStateInit = 1u,
    kCAAM_AlgStateFinal = 2u,
    kCAAM_AlgStateInitFinal = 3u,
} caam_algorithm_state_t;

/*******************************************************************************
 * HASH Definitions
 ******************************************************************************/
enum _caam_sha_digest_len
{
    kCAAM_RunLenSha1 = 28u,
    kCAAM_OutLenSha1 = 20u,
    kCAAM_RunLenSha224 = 40u,
    kCAAM_OutLenSha224 = 28u,
    kCAAM_RunLenSha256 = 40u,
    kCAAM_OutLenSha256 = 32u,
    kCAAM_RunLenSha384 = 64u,
    kCAAM_OutLenSha384 = 48u,
    kCAAM_RunLenSha512 = 64u,
    kCAAM_OutLenSha512 = 64u,
};

/*! Internal states of the HASH creation process */
typedef enum _caam_hash_algo_state
{
    kCAAM_HashInit = 1u, /*!< Key in the HASH context is the input key. */
    kCAAM_HashUpdate,    /*!< HASH context has algorithm specific context: MAC, K2 and K3 (XCBC-MAC), MAC and L (CMAC),
                             running digest (MDHA). Key in the HASH context is the derived key. */
} caam_hash_algo_state_t;

/*! 64-byte block represented as byte array or 16 32-bit words */
typedef union _caam_hash_block
{
    uint32_t w[CAAM_HASH_BLOCK_SIZE / 4]; /*!< array of 32-bit words */
    uint8_t b[CAAM_HASH_BLOCK_SIZE];      /*!< byte array */
} caam_hash_block_t;

/*! Definitions of indexes into hash context array */
typedef enum _caam_hash_ctx_indexes
{
    kCAAM_HashCtxKeyStartIdx = 12, /*!< context word array index where key is stored */
    kCAAM_HashCtxKeySize = 20,     /*!< context word array index where key size is stored */
    kCAAM_HashCtxNumWords = 21,    /*!< number of context array 32-bit words  */
} caam_hash_ctx_indexes;

typedef struct _caam_hash_ctx_internal
{
    caam_hash_block_t blk; /*!< memory buffer. only full 64-byte blocks are written to CAAM during hash updates */
    uint32_t word[kCAAM_HashCtxNumWords]; /*!< CAAM module context that needs to be saved/restored between CAAM jobs */
    uint32_t blksz;                       /*!< number of valid bytes in memory buffer */
    CAAM_Type *base;                      /*!< CAAM peripheral base address */
    caam_handle_t *handle;                /*!< CAAM handle (specifies jobRing and optional callback function) */
    caam_hash_algo_t algo;        /*!< selected algorithm from the set of supported algorithms in caam_hash_algo_t */
    caam_hash_algo_state_t state; /*!< finite machine state of the hash software process */
} caam_hash_ctx_internal_t;

/*! Definitions of indexes into hash job descriptor */
enum _caam_hash_sgt_index
{
    kCAAM_HashDescriptorSgtIdx = 13u, /*!< Index of the hash job descriptor[] where the two entry SGT starts. */
};

/*! One entry in the SGT */
typedef struct _caam_sgt_entry
{
    /* 64-bit address. */
    uint32_t address_h;
    uint32_t address_l;
    uint32_t length;
    uint32_t offset;
} caam_sgt_entry_t;

/*! Definitions SGT entry type */
typedef enum _caam_hash_sgt_entry_type
{
    kCAAM_HashSgtEntryNotLast = 0u, /*!< Do not set the Final Bit in SGT entries */
    kCAAM_HashSgtEntryLast = 1u,    /*!< Sets Final Bit in the last SGT entry */
} caam_hash_sgt_entry_type_t;

/*! Two entry SGT, embedded in the hash job descriptor */
typedef caam_sgt_entry_t caam_hash_internal_sgt_t[2];

/*! Definitions of SGT type */
typedef enum _caam_hash_sgt_type
{
    kCAAM_HashSgtInternal = 0u, /*!< Two entry SGT is copied into the hash job descriptor. */
    kCAAM_HashSgtExternal = 1u, /*!< Use external SGT. */
} caam_hash_sgt_type_t;

enum _caam_hash_non_blocking_sgt_entries
{
    kCAAM_HashSgtMaxCtxEntries =
        (sizeof(caam_hash_block_t) + sizeof(uint32_t) * kCAAM_HashCtxKeyStartIdx) / sizeof(caam_sgt_entry_t),
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

static caam_job_ring_interface_t *s_jr0 = NULL; /*!< Pointer to job ring interface 0. */
static int32_t s_jrIndex0 = 0;                  /*!< Current index in the input job ring 0. */

static caam_job_ring_interface_t *s_jr1 = NULL; /*!< Pointer to job ring interface 1. */
static int32_t s_jrIndex1 = 0;                  /*!< Current index in the input job ring 1. */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * CAAM Common code static
 ******************************************************************************/
#if 0
/* for build without string.h memcpy() */
static void caam_memcpy(void *dst, const void *src, size_t size)
{
    register uint8_t *to = (uint8_t *)(uintptr_t)dst;
    register const uint8_t *from = (const uint8_t *)(uintptr_t)src;

    /* if it is possible to move data with 32-bit aligned access, do it so */
    if ((size >= sizeof(uint32_t)) && (0u == ((uintptr_t)dst & 0x3u)) && (0u == ((uintptr_t)src & 0x3u)))
    {
        register uint32_t *to32 = (uint32_t *)(uintptr_t)dst;
        register const uint32_t *from32 = (const uint32_t *)(uintptr_t)src;
        while (size >= sizeof(uint32_t))
        {
            *to32 = *from32;
            size -= sizeof(uint32_t);
            to32++;
            from32++;
        }

        to = (uint8_t *)(uintptr_t)to32;
        from = (const uint8_t *)(uintptr_t)from32;
    }

    while (size)
    {
        *to = *from;
        size--;
        to++;
        from++;
    }
}
#else
#include <string.h>
#define caam_memcpy memcpy
#endif

static void caam_job_ring_set_base_address_and_size(CAAM_Type *base,
                                                    caam_job_ring_t jr,
                                                    uint32_t *inputRingBase,
                                                    size_t inputRingSize,
                                                    uint32_t *outputRingBase,
                                                    uint32_t outputRingSize)
{
    if (kCAAM_JobRing0 == jr)
    {
        IRBAR0 = (uintptr_t)inputRingBase;
        base->JOBRING[0].IRSR_JR = inputRingSize;
        ORBAR0 = (uintptr_t)outputRingBase;
        base->JOBRING[0].ORSR_JR = outputRingSize;
    }

    if (kCAAM_JobRing1 == jr)
    {
        IRBAR1 = (uintptr_t)inputRingBase;
        base->JOBRING[1].IRSR_JR = inputRingSize;
        ORBAR1 = (uintptr_t)outputRingBase;
        base->JOBRING[1].ORSR_JR = outputRingSize;
    }
}

static inline void caam_input_ring_set_jobs_added(CAAM_Type *base, caam_job_ring_t jr, uint32_t numJobsAdded)
{
    if (kCAAM_JobRing0 == jr)
    {
        base->JOBRING[0].IRJAR_JR = numJobsAdded;
    }

    if (kCAAM_JobRing1 == jr)
    {
        base->JOBRING[1].IRJAR_JR = numJobsAdded;
    }
}

static inline void caam_output_ring_set_jobs_removed(CAAM_Type *base, caam_job_ring_t jr, uint32_t numJobsRemoved)
{
    if (kCAAM_JobRing0 == jr)
    {
        base->JOBRING[0].ORJRR_JR = numJobsRemoved;
    }

    if (kCAAM_JobRing1 == jr)
    {
        base->JOBRING[1].ORJRR_JR = numJobsRemoved;
    }
}

static uint32_t caam_output_ring_get_slots_full(CAAM_Type *base, caam_job_ring_t jr)
{
    uint32_t retVal = 0;

    if (kCAAM_JobRing0 == jr)
    {
        retVal = base->JOBRING[0].ORSFR_JR;
    }

    if (kCAAM_JobRing1 == jr)
    {
        retVal = base->JOBRING[1].ORSFR_JR;
    }

    return retVal;
}

/*!
 * @brief Tests the correct key size.
 *
 * This function tests the correct key size.
 * @param keySize Input key length in bytes.
 * @return True if the key length is supported, false if not.
 */
bool caam_check_key_size(const uint32_t keySize)
{
    return ((keySize == 16u) || ((keySize == 24u)) || ((keySize == 32u)));
}

static status_t caam_in_job_ring_add(CAAM_Type *base, caam_job_ring_t jobRing, void *descaddr)
{
    /* adding new job to the s_inJobRing[] must be atomic
     * as this is global variable
     */
    uint32_t currPriMask = DisableGlobalIRQ();
    if (kCAAM_JobRing0 == jobRing)
    {
        s_jr0->inputJobRing[s_jrIndex0] = (uintptr_t)descaddr;
        s_jrIndex0++;
        if (s_jrIndex0 >= ARRAY_SIZE(s_jr0->inputJobRing))
        {
            s_jrIndex0 = 0;
        }
    }
    else
    {
        s_jr1->inputJobRing[s_jrIndex1] = (uintptr_t)descaddr;
        s_jrIndex1++;
        if (s_jrIndex1 >= ARRAY_SIZE(s_jr1->inputJobRing))
        {
            s_jrIndex1 = 0;
        }
    }

    caam_input_ring_set_jobs_added(base, jobRing, 1);
    EnableGlobalIRQ(currPriMask);
    return kStatus_Success;
}

/* this function shall be only called inside CAAM driver critical section
 * because it accesses global variables.
 */
static status_t caam_out_job_ring_remove(CAAM_Type *base, caam_job_ring_t jobRing, int outIndex)
{
    if (kCAAM_JobRing0 == jobRing)
    {
        s_jr0->outputJobRing[outIndex++] = 0; /* clear descriptor address */
        s_jr0->outputJobRing[outIndex] = 0;   /* clear status */
    }
    else
    {
        s_jr1->outputJobRing[outIndex++] = 0; /* clear descriptor address */
        s_jr1->outputJobRing[outIndex] = 0;   /* clear status */
    }

    caam_output_ring_set_jobs_removed(base, jobRing, 1);
    return 0;
}

static status_t caam_out_job_ring_test_and_remove(
    CAAM_Type *base, caam_job_ring_t jobRing, void *descriptor, bool *wait, bool *found)
{
    uint32_t currPriMask = DisableGlobalIRQ();
    int i;
    status_t status;

    *found = false;
    *wait = true;
    status = kStatus_Success;

    uint32_t *jr = (kCAAM_JobRing0 == jobRing) ? s_jr0->outputJobRing : s_jr1->outputJobRing;
    int jrEntries = (kCAAM_JobRing0 == jobRing) ? ARRAY_SIZE(s_jr0->outputJobRing) : ARRAY_SIZE(s_jr1->outputJobRing);

    /* check if an interrupt or other thread consumed the result that we just saw */
    if (caam_output_ring_get_slots_full(base, jobRing))
    {
        /* check if our descriptor is in the output job ring
         * look from the beginning of the out job ring
         */
        i = 0;

        while ((!*found) && (i < jrEntries))
        {
            if (jr[i] == (uint32_t)descriptor)
            {
                *found = true;
                *wait = false;
                /* check for error in status word */
                if (jr[i + 1])
                {
                    /* printf("Error 0x%lx\r\n", jr[i + 1]); */

                    /* for JMP/HALT commands with User specified status, return the user status, just to allow the
                     * software to test for user status termination status words */
                    /* This is used by PKHA PrimalityTest to report a candidate is believed not being prime */
                    if (0x30000000u == (jr[i + 1] & 0xff000000u))
                    {
                        status = jr[i + 1];
                    }
                    else
                    {
                        status = kStatus_Fail;
                    }
                }
                caam_out_job_ring_remove(base, jobRing, i);
            }
            else
            {
                /* try next result */
                i += 2u;
            }
        }
    }
    EnableGlobalIRQ(currPriMask);
    return status;
}

typedef union _caam_xcm_block_t
{
    uint32_t w[4]; /*!< CAAM context register is 16 bytes written as four 32-bit words */
    uint8_t b[16]; /*!< 16 octets block for CCM B0 and CTR0 and for GCM */
} caam_xcm_block_t;

static uint32_t swap_bytes(uint32_t in)
{
    return (((in & 0x000000ffu) << 24) | ((in & 0x0000ff00u) << 8) | ((in & 0x00ff0000u) >> 8) |
            ((in & 0xff000000u) >> 24));
}

/*!
 * @brief AES GCM check validity of input arguments.
 *
 * This function checks the validity of input arguments.
 *
 * @param base LTC peripheral base address.
 * @param src Source address (plaintext or ciphertext).
 * @param iv Initialization vector address.
 * @param aad Additional authenticated data address.
 * @param dst Destination address (plaintext or ciphertext).
 * @param inputSize Size of input (same size as output) data in bytes.
 * @param ivSize Size of Initialization vector in bytes.
 * @param aadSize Size of additional data in bytes.
 * @param tagSize Size of the GCM tag in bytes.
 */
static status_t caam_aes_gcm_check_input_args(CAAM_Type *base,
                                              const uint8_t *src,
                                              const uint8_t *iv,
                                              const uint8_t *aad,
                                              uint8_t *dst,
                                              size_t inputSize,
                                              size_t ivSize,
                                              size_t aadSize,
                                              size_t tagSize)
{
    if (!base)
    {
        return kStatus_InvalidArgument;
    }

    /* tag can be NULL to skip tag processing */
    if ((ivSize && (!iv)) || (aadSize && (!aad)) || (inputSize && ((!src) || (!dst))))
    {
        return kStatus_InvalidArgument;
    }

    /* octet length of tag (tagSize) must be element of 4,8,12,13,14,15,16 */
    if (((tagSize > 16u) || (tagSize < 12u)) && (tagSize != 4u) && (tagSize != 8u))
    {
        return kStatus_InvalidArgument;
    }

    /* no IV AAD DATA makes no sense */
    if (0 == (inputSize + ivSize + aadSize))
    {
        return kStatus_InvalidArgument;
    }

    /* check length of input strings. This is more strict than the GCM specificaiton due to 32-bit architecture.
     * The API interface would work on 64-bit architecture as well, but as it has not been tested, let it limit to
     * 32-bits.
     */
    if (!((ivSize >= 1u) && (sizeof(size_t) <= 4u)))
    {
        return kStatus_InvalidArgument;
    }

    return kStatus_Success;
}

static const uint32_t templateAesGcm[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */

    /* 03 */ 0x82100908u, /* OPERATION: AES GCM Decrypt Finalize */

    /* 04 */ 0x12830004u, /* LOAD Class 1 ICV Size Register by IMM data */
    /* 05 */ 0x00000000u, /* place: received ICV size */

    /* 06 */ 0x22210000u, /* FIFO LOAD IV (flush) */
    /* 07 */ 0x00000000u, /* place: IV address */

    /* 08 */ 0x22310000u, /* FIFO LOAD AAD (flush) */
    /* 09 */ 0x00000000u, /* place: AAD address */

    /* 10 */ 0x22130000u, /* FIFO LOAD message */
    /* 11 */ 0x00000000u, /* place: message address */

    /* 12 */ 0x60300000u, /* FIFO STORE Message */
    /* 13 */ 0x00000000u, /* place: destination address */

    /* 14 */ 0xA3001201u, /* JMP always to next command. Load/store checkpoint. Class 1 done checkpoint. */

    /* For encryption, write the computed and encrypted MAC to user buffer */
    /* For decryption, compare the computed tag with the received tag, CICV-only job. */
    /* 15 */ 0x10880004u, /* LOAD Immediate to Clear Written Register. */
    /* 16 */ 0x08000004u, /* value for Clear Written Register: C1D and C1DS bits are set */
    /* 17 */ 0x12820004u, /* LOAD Immediate to C1DS Register. */
    /* 18 */ 0x00000000u, /* zero data size */
    /* 19 */ 0x12830004u, /* LOAD Class 1 ICV Size Register by IMM data */
    /* 20 */ 0x00000000u, /* place: received ICV size */
    /* 21 */ 0x82100902u, /* OPERATION: AES GCM Decrypt Update ICV_TEST */
    /* 22 */ 0x223B0000u, /* FIFO LOAD ICV */
    /* 23 */ 0x00000000u, /* place: received ICV address */
};

status_t caam_aes_gcm_non_blocking(CAAM_Type *base,
                                   caam_handle_t *handle,
                                   caam_desc_aes_gcm_t descriptor,
                                   const uint8_t *input,
                                   uint8_t *output,
                                   size_t size,
                                   const uint8_t *iv,
                                   size_t ivSize,
                                   const uint8_t *aad,
                                   size_t aadSize,
                                   const uint8_t *key,
                                   size_t keySize,
                                   uint32_t tag,
                                   size_t tagSize,
                                   int encrypt)
{
    BUILD_ASSURE(sizeof(templateAesGcm) <= sizeof(caam_desc_aes_gcm_t), caam_desc_aes_gcm_t_size);
    status_t status;

    status = caam_aes_gcm_check_input_args(base, input, iv, aad, output, size, ivSize, aadSize, tagSize);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesGcm);
    caam_memcpy(descriptor, templateAesGcm, sizeof(templateAesGcm));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* Encrypt decrypt */
    if (encrypt)
    {
        descriptor[3] |= 1u; /* ENC */
    }

    /* ICV Size */
    descriptor[5] = tagSize;

    bool ivLast = (aadSize == 0) && (size == 0);
    bool aadLast = (size == 0);

    /* IV address and size */
    descriptor[6] |= (ivSize & 0x0000FFFFu);
    descriptor[7] = (uint32_t)iv;
    if (ivLast)
    {
        descriptor[6] |= 0x00020000u; /* LC1 */
    }

    /* AAD address and size */
    descriptor[8] |= (aadSize & 0x0000FFFFu);
    descriptor[9] = (uint32_t)aad;
    if ((!ivLast) && aadLast)
    {
        descriptor[8] |= 0x00020000u; /* LC1 */
    }

    /* Message source address and size */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)input;

    /* Message destination address and size */
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)output;

    if (tag)
    {
        if (!encrypt)
        {
            descriptor[20] = tagSize;
            descriptor[22] |= (tagSize & 0xFFu);
            descriptor[23] = (uint32_t)tag;
        }
        else
        {
            /* For encryption change the command to FIFO STORE, as tag data needs to be put into tag output */
            descriptor[14] = 0x52200000u | (tagSize & 0xFFu); /* STORE from Class 1 context to tag */
            descriptor[15] = (uint32_t)tag;                   /* place: tag address */
            ;
            descriptor[16] = 0xA0C00000u; /* always halt with status 0x0 (normal) */
        }
    }
    else
    {
        /* tag is NULL, skip tag processing */
        descriptor[14] = 0xA0C00000u; /* always halt with status 0x0 (normal) */
    }

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static status_t caam_aes_ccm_check_input_args(CAAM_Type *base,
                                              const uint8_t *src,
                                              const uint8_t *iv,
                                              const uint8_t *key,
                                              uint8_t *dst,
                                              size_t ivSize,
                                              size_t aadSize,
                                              size_t keySize,
                                              size_t tagSize)
{
    if (!base)
    {
        return kStatus_InvalidArgument;
    }

    /* tag can be NULL to skip tag processing */
    if ((!src) || (!iv) || (!key) || (!dst))
    {
        return kStatus_InvalidArgument;
    }

    /* size of Nonce (ivSize) must be element of 7,8,9,10,11,12,13 */
    if ((ivSize < 7u) || (ivSize > 13u))
    {
        return kStatus_InvalidArgument;
    }
    /* octet length of MAC (tagSize) must be element of 4,6,8,10,12,14,16 for tag processing or zero to skip tag
     * processing */
    if (((tagSize > 0) && (tagSize < 4u)) || (tagSize > 16u) || (tagSize & 1u))
    {
        return kStatus_InvalidArgument;
    }

    /* check if keySize is supported */
    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }

    /* AESA does not support more AAD than this */
    if (aadSize >= 65280u)
    {
        return kStatus_InvalidArgument;
    }
    return kStatus_Success;
}

static void caam_aes_ccm_context_init(
    uint32_t inputSize, const uint8_t *iv, uint32_t ivSize, uint32_t aadSize, uint32_t tagSize, void *b0, void *ctr0)
{
    caam_xcm_block_t blk;
    caam_xcm_block_t blkZero = {{0x0u, 0x0u, 0x0u, 0x0u}};

    int q; /* octet length of binary representation of the octet length of the payload. computed as (15 - n), where n is
              length of nonce(=ivSize) */
    uint8_t flags; /* flags field in B0 and CTR0 */

    /* compute B0 */
    caam_memcpy(&blk, &blkZero, sizeof(blk));
    /* tagSize - size of output MAC */
    q = 15 - ivSize;
    flags = (uint8_t)(8 * ((tagSize - 2) / 2) + q - 1); /* 8*M' + L' */
    if (aadSize)
    {
        flags |= 0x40; /* Adata */
    }
    blk.b[0] = flags;                   /* flags field */
    blk.w[3] = swap_bytes(inputSize);   /* message size, most significant byte first */
    caam_memcpy(&blk.b[1], iv, ivSize); /* nonce field */

    /* Write B0 data to the context register.
     */
    caam_memcpy(b0, &blk.b[0], 16);

    /* Write CTR0 to the context register.
     */
    caam_memcpy(&blk, &blkZero, sizeof(blk)); /* ctr(0) field = zero */
    blk.b[0] = q - 1;                         /* flags field */
    caam_memcpy(&blk.b[1], iv, ivSize);       /* nonce field */
    caam_memcpy(ctr0, &blk.b[0], 16);
}

static const uint32_t templateAesCcm[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */

    /* 03 */ 0x12A00010u, /* LOAD 16 immediate bytes of B0 to Class 1 Context Register. Offset 0 bytes. */
    /* 04 */ 0x00000000u, /* place: B0[0-3] */
    /* 05 */ 0x00000000u, /* place: B0[4-7] */
    /* 06 */ 0x00000000u, /* place: B0[8-11] */
    /* 07 */ 0x00000000u, /* place: B0[12-15] */

    /* 08 */ 0x12A01010u, /* LOAD 16 immediate bytes of CTR0 to Class 1 Context Register. Offset 16 bytes. */
    /* 09 */ 0x00000000u, /* place: CTR0[0-3] */
    /* 10 */ 0x00000000u, /* place: CTR0[4-7] */
    /* 11 */ 0x00000000u, /* place: CTR0[8-11] */
    /* 12 */ 0x00000000u, /* place: CTR0[12-15] */

    /* 13 */ 0x8210080Cu, /* OPERATION: AES CCM Decrypt Initialize/Finalize */

    /* 14 */ 0x22B00004u, /* FIFO LOAD additional authentication data. Immediate 32-bit word with aadSize encoded */
    /* 15 */ 0x00000000u, /* place: encoded aadSize followed by first byte(s) of authentication data */
    /* 16 */ 0x22310000u, /* FIFO LOAD additional authentication data. Flush as this is last data of AAD type. */
    /* 17 */ 0x00000000u, /* place: AAD address */

    /* 18 */ 0x22130000u, /* FIFO LOAD message */
    /* 19 */ 0x00000000u, /* place: message address */

    /* 20 */ 0x60300000u, /* FIFO STORE Message */
    /* 21 */ 0x00000000u, /* place: destination address */

    /* For encryption, write the computed and encrypted MAC to user buffer */
    /* 22 */ 0x52202000u, /* STORE from Class 1 context to tag */
    /* 23 */ 0x00000000u, /* place: tag address */

    /* For decryption, compare the computed tag with the received tag */

};

status_t caam_aes_ccm_non_blocking(CAAM_Type *base,
                                   caam_handle_t *handle,
                                   caam_desc_aes_ccm_t descriptor,
                                   const uint8_t *input,
                                   uint8_t *output,
                                   size_t size,
                                   const uint8_t *iv,
                                   size_t ivSize,
                                   const uint8_t *aad,
                                   size_t aadSize,
                                   const uint8_t *key,
                                   size_t keySize,
                                   uint32_t tag,
                                   size_t tagSize,
                                   int encrypt)
{
    BUILD_ASSURE(sizeof(templateAesCcm) <= sizeof(caam_desc_aes_ccm_t), caam_desc_aes_ccm_t_size);
    status_t status;

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesCcm);
    caam_memcpy(descriptor, templateAesCcm, sizeof(templateAesCcm));

    status = caam_aes_ccm_check_input_args(base, input, iv, key, output, ivSize, aadSize, keySize, tagSize);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* B0 and CTR0 */
    caam_aes_ccm_context_init(size, iv, ivSize, aadSize, tagSize, &descriptor[4], &descriptor[9]);

    /* Encrypt decrypt */
    if (encrypt)
    {
        descriptor[13] |= 1u; /* ENC */
    }
    else if (tag)
    {
        descriptor[13] |= 2u; /* ICV_TEST */
    }
    else
    {
        /* decrypt with tag NULL (skip tag processing). nothing needs to be changed in descriptor[13] for this case */
    }

    /* AAD address and size */
    /* encoding is two octets, msbyte first */
    if (aadSize)
    {
        uint32_t swapped = swap_bytes(aadSize);
        uint32_t sz;
        caam_memcpy(&descriptor[15], ((uint8_t *)&swapped) + sizeof(uint16_t), sizeof(uint16_t));
        sz = aadSize > 2u ? 2u : aadSize;                       /* limit aad to the end of 16 bytes blk */
        caam_memcpy(((uint8_t *)&descriptor[15]) + 2, aad, sz); /* fill B1 with aad */
        /* track consumed AAD. sz bytes have been moved to fifo. */
        aadSize -= sz;
        aad += sz;

        if (!aadSize)
        {
            /* in case aadSize is 1 or 2, we add Flush bit to the command and skip FIFO LOAD AAD */
            descriptor[14] |= 0x00010000; /* Flush (last AAD data) */
            descriptor[16] = 0xA0000002u; /* jump to current index + 2 (=18) */
        }
        else
        {
            descriptor[16] |= (aadSize & 0x0000FFFFu);
            descriptor[17] = (uint32_t)aad;
        }
    }
    else
    {
        /* no AAD, jump directly to message */
        descriptor[14] = 0xA0000004u; /* jump to current index + 4 (=18) */
    }

    /* Message source address and size */
    descriptor[18] |= (size & 0x0000FFFFu);
    descriptor[19] = (uint32_t)input;

    /* Message destination address and size */
    descriptor[20] |= (size & 0x0000FFFFu);
    descriptor[21] = (uint32_t)output;

    if (tag)
    {
        /* For decryption change the command to FIFO LOAD, as tag data needs to be put into input FIFO */
        if (!encrypt)
        {
            /* FIFO LOAD ICV */
            descriptor[22] = 0x223B0000u;
        }
        descriptor[22] |= (tagSize & 0xFFu);
        descriptor[23] = (uint32_t)tag;
    }
    else
    {
        /* tag is NULL, skip tag processing */
        descriptor[22] = 0xA0C00000u; /* always halt with status 0x0 (normal) */
    }

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_EncryptTagCcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_ccm_t descriptor,
                                           const uint8_t *plaintext,
                                           uint8_t *ciphertext,
                                           size_t size,
                                           const uint8_t *iv,
                                           size_t ivSize,
                                           const uint8_t *aad,
                                           size_t aadSize,
                                           const uint8_t *key,
                                           size_t keySize,
                                           uint8_t *tag,
                                           size_t tagSize)
{
    return caam_aes_ccm_non_blocking(base, handle, descriptor, plaintext, ciphertext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 1);
}

status_t CAAM_AES_DecryptTagCcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_ccm_t descriptor,
                                           const uint8_t *ciphertext,
                                           uint8_t *plaintext,
                                           size_t size,
                                           const uint8_t *iv,
                                           size_t ivSize,
                                           const uint8_t *aad,
                                           size_t aadSize,
                                           const uint8_t *key,
                                           size_t keySize,
                                           const uint8_t *tag,
                                           size_t tagSize)
{
    return caam_aes_ccm_non_blocking(base, handle, descriptor, ciphertext, plaintext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 0);
}

static const uint32_t templateAesCtr[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x12201010u, /* LOAD 16 bytes of CTR0 to Class 1 Context Register. Offset 16 bytes. */
    /* 04 */ 0x00000000u, /* place: CTR0 address */

    /* 05 */ 0x82100000u, /* OPERATION: AES CTR (de)crypt in Update mode */
    /* 06 */ 0x22130000u, /* FIFO LOAD Message */
    /* 07 */ 0x00000000u, /* place: source address */
    /* 08 */ 0x60300000u, /* FIFO STORE Message */
    /* 09 */ 0x00000000u, /* place: destination address */

    /* 10 */ 0xA2000001u, /* JMP always to next command. Done checkpoint (wait for Class 1 Done) */
    /* 11 */ 0x10880004u, /* LOAD Immediate to Clear Written Register. */
    /* 12 */ 0x08000004u, /* value for Clear Written Register: C1D and C1DS bits are set */
    /* 13 */ 0x22930010u, /* FIFO LOAD Message Immediate 16 bytes */
    /* 14 */ 0x00000000u, /* all zeroes 0-3 */

    /* 15 */ 0x00000000u, /* all zeroes 4-7 */
    /* 16 */ 0x00000000u, /* all zeroes 8-11 */
    /* 17 */ 0x00000000u, /* all zeroes 12-15 */
    /* 18 */ 0x60300010u, /* FIFO STORE Message 16 bytes */
    /* 19 */ 0x00000000u, /* place: counterlast[] block address */

    /* 20 */ 0x82100000u, /* OPERATION: AES CTR (de)crypt in Update mode */
    /* 21 */ 0x52201010u, /* STORE 16 bytes of CTRi from Class 1 Context Register offset 16 bytes. */
    /* 22 */ 0x00000000u, /* place: CTRi address */
};

status_t CAAM_AES_CryptCtrNonBlocking(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      caam_desc_aes_ctr_t descriptor,
                                      const uint8_t *input,
                                      uint8_t *output,
                                      size_t size,
                                      uint8_t *counter,
                                      const uint8_t *key,
                                      size_t keySize,
                                      uint8_t *counterlast,
                                      size_t *szLeft)
{
    BUILD_ASSURE(sizeof(templateAesCtr) <= sizeof(caam_desc_aes_ctr_t), caam_desc_aes_ctr_t_size);
    uint32_t descriptorSize;

    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }

    /* get template descriptor and it's size */
    descriptorSize = ARRAY_SIZE(templateAesCtr);
    caam_memcpy(descriptor, templateAesCtr, sizeof(templateAesCtr));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for CTR0 in templateAesCtr */
    descriptor[4] = (uint32_t)counter;

    /* source address and size */
    descriptor[6] |= (size & 0x0000FFFFu);
    descriptor[7] = (uint32_t)input;

    /* destination address and size */
    descriptor[8] |= (size & 0x0000FFFFu);
    descriptor[9] = (uint32_t)output;

    /* AES CTR Crypt OPERATION in descriptor[5]
     * Algorithm State (AS) in template is Update (0h)
     * Only in case we are chaining the AES CTR calls (counterlast[] != NULL),
     * we have to change the algorithm state to Finalize (2h)
     * and so the CTRi for the last message block is not written to Class 1 Context.
     * This allows us to repeat AES CTR of the last CTRi, with destination to counterlast[],
     * and with using all zeroes in message data, the counterlast[] gets ECB of the last CTRi.
     */

    /* if counterlast or szLeft is NULL, the caller is not interested in AES of last counter
     * Thus, we can skip the counterlast processing
     * and only read the last CTRi from context.
     * So, we replace descriptor[10] with a jump command to STORE
     */
    if ((counterlast == NULL) || (szLeft == NULL))
    {
        /*  To create an unconditional jump, use TEST TYPE = 00 (all specified conditions true) and
            clear all TEST CONDITION bits because the tested condition is considered to be true if
            no test condition bits are set. */
        descriptor[10] = 0xA000000Bu; /* jump to current index + 11 (=21) */
    }
    else
    {
        uint32_t lastSize;

        descriptor[5] |= 0x08u; /* finalize */
        descriptor[19] = (uint32_t)counterlast;

        lastSize = size % 16u;
        if (lastSize)
        {
            *szLeft = 16u - lastSize;
        }
        else
        {
            *szLeft = 0;
            /* descriptor[10] = 0xA000000Bu; */ /* jump to current index + 11 (=21) */
        }
    }

    /* read last CTRi from AES back to caller */
    descriptor[22] = (uint32_t)counter;

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static const uint32_t templateAesEcb[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x22130000u, /* FIFO LOAD Message */
    /* 04 */ 0x00000000u, /* place: source address */
    /* 05 */ 0x60300000u, /* FIFO STORE Message */
    /* 06 */ 0x00000000u, /* place: destination address */
    /* 07 */ 0x82100200u, /* OPERATION: AES ECB Decrypt */
};

status_t CAAM_AES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t *key,
                                        size_t keySize)
{
    BUILD_ASSURE(sizeof(templateAesEcb) <= sizeof(caam_desc_aes_ecb_t), caam_desc_aes_ecb_t_size);
    uint32_t descriptorSize;

    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }
    /* ECB mode, size must be non-zero 16-byte multiple */
    if (size % 16u)
    {
        return kStatus_InvalidArgument;
    }

    descriptorSize = ARRAY_SIZE(templateAesEcb);
    caam_memcpy(descriptor, templateAesEcb, sizeof(templateAesEcb));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;
    descriptor[3] |= (size & 0x0000FFFFu);
    descriptor[4] = (uint32_t)plaintext;
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)ciphertext;
    descriptor[7] |= 1u; /* add ENC bit to specify Encrypt OPERATION */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t *key,
                                        size_t keySize)
{
    uint32_t descriptorSize;

    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }
    /* ECB mode, size must be non-zero 16-byte multiple */
    if (size % 16u)
    {
        return kStatus_InvalidArgument;
    }

    descriptorSize = ARRAY_SIZE(templateAesEcb);
    caam_memcpy(descriptor, templateAesEcb, sizeof(templateAesEcb));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;
    descriptor[3] |= (size & 0x0000FFFFu);
    descriptor[4] = (uint32_t)ciphertext;
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)plaintext;

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static const uint32_t templateAesCbc[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x12200010u, /* LOAD 16 bytes of iv to Class 1 Context Register */
    /* 04 */ 0x00000000u, /* place: iv address */
    /* 05 */ 0x22130000u, /* FIFO LOAD Message */
    /* 06 */ 0x00000000u, /* place: source address */
    /* 07 */ 0x60300000u, /* FIFO STORE Message */
    /* 08 */ 0x00000000u, /* place: destination address */
    /* 09 */ 0x82100100u, /* OPERATION: AES CBC Decrypt */
};

status_t CAAM_AES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        size_t keySize)
{
    BUILD_ASSURE(sizeof(templateAesCbc) <= sizeof(caam_desc_aes_cbc_t), caam_desc_aes_cbc_t_size);
    uint32_t descriptorSize;

    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }

    /* CBC mode, size must be non-zero 16-byte multiple */
    if (size % 16u)
    {
        return kStatus_InvalidArgument;
    }

    /* get template descriptor and it's size */
    descriptorSize = ARRAY_SIZE(templateAesCbc);
    caam_memcpy(descriptor, templateAesCbc, sizeof(templateAesCbc));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for IV in templateAesCbc */
    descriptor[4] = (uint32_t)iv;

    /* source address and size */
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)plaintext;

    /* destination address and size */
    descriptor[7] |= (size & 0x0000FFFFu);
    descriptor[8] = (uint32_t)ciphertext;

    /* AES CBC */
    descriptor[9] |= 1u; /* add ENC bit to specify Encrypt OPERATION */

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        size_t keySize)
{
    uint32_t descriptorSize;

    if (!caam_check_key_size(keySize))
    {
        return kStatus_InvalidArgument;
    }

    /* CBC mode, size must be non-zero 16-byte multiple */
    if (size % 16u)
    {
        return kStatus_InvalidArgument;
    }

    /* get template descriptor and it's size */
    descriptorSize = ARRAY_SIZE(templateAesCbc);
    caam_memcpy(descriptor, templateAesCbc, sizeof(templateAesCbc));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for IV in templateAesCbc */
    descriptor[4] = (uint32_t)iv;

    /* source address and size */
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)ciphertext;

    /* destination address and size */
    descriptor[7] |= (size & 0x0000FFFFu);
    descriptor[8] = (uint32_t)plaintext;

    /* AES CBC Decrypt OPERATION in descriptor[9] */

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_EncryptTagGcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_gcm_t descriptor,
                                           const uint8_t *plaintext,
                                           uint8_t *ciphertext,
                                           size_t size,
                                           const uint8_t *iv,
                                           size_t ivSize,
                                           const uint8_t *aad,
                                           size_t aadSize,
                                           const uint8_t *key,
                                           size_t keySize,
                                           uint8_t *tag,
                                           size_t tagSize)
{
    return caam_aes_gcm_non_blocking(base, handle, descriptor, plaintext, ciphertext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 1);
}

status_t CAAM_AES_DecryptTagGcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_gcm_t descriptor,
                                           const uint8_t *ciphertext,
                                           uint8_t *plaintext,
                                           size_t size,
                                           const uint8_t *iv,
                                           size_t ivSize,
                                           const uint8_t *aad,
                                           size_t aadSize,
                                           const uint8_t *key,
                                           size_t keySize,
                                           const uint8_t *tag,
                                           size_t tagSize)
{
    return caam_aes_gcm_non_blocking(base, handle, descriptor, ciphertext, plaintext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 0);
}

void CAAM_GetDefaultConfig(caam_config_t *config)
{
    caam_config_t userConfig = {
        {NULL, NULL}, kCAAM_RNG_SampleModeVonNeumann, kCAAM_RNG_RingOscDiv4, true, true, true,
        true,         kCAAM_NormalOperationBlobs,
    };

    *config = userConfig;
}

void CAAM_Init(CAAM_Type *base, const caam_config_t *config)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Caam0);
#endif

    base->MCFGR = 0x80000000u; /* reset */
    base->MCFGR = 0x90000000u; /* reset DMA */
    base->MCFGR = 0x00082300u; /* (reset value) */

    /* job ring interface 0 is mandatory */
    assert(config->jobRingInterface[0]);
    if (NULL == config->jobRingInterface[0])
    {
        return; /* return if user wishes to use the job ring but does not configure it. */
    }

    /* Job Rings Configuration
     * number of entries in both input and output ring is equal.
     * Note the size of an entry is different. an entry in the input ring is a 32-bit word.
     * an entry in the output ring is two 32-bit words. (descriptor pointer followed by termination status word)
     */
    s_jr0 = config->jobRingInterface[0];
    memset(s_jr0, 0, sizeof(*s_jr0));
    s_jrIndex0 = 0;
    caam_job_ring_set_base_address_and_size(base, kCAAM_JobRing0, s_jr0->inputJobRing, ARRAY_SIZE(s_jr0->inputJobRing),
                                            s_jr0->outputJobRing, ARRAY_SIZE(s_jr0->outputJobRing) / 2);

    if (config->jobRingInterface[1])
    {
        s_jr1 = config->jobRingInterface[1];
        memset(s_jr1, 0, sizeof(*s_jr1));
        s_jrIndex1 = 0;
        caam_job_ring_set_base_address_and_size(base, kCAAM_JobRing1, s_jr1->inputJobRing,
                                                ARRAY_SIZE(s_jr1->inputJobRing), s_jr1->outputJobRing,
                                                ARRAY_SIZE(s_jr1->outputJobRing) / 2);
    }

    /*
     * Instantiate RNG in normal (non-deterministic) mode and load the JDKEK, TDKEK and TDSK registers
     * this step is required for example
     * for FIFO STORE command to be able to store Key register as Black key
     * for example during AES XCBC-MAC context switch (need to store derived key K1 to memory)
     */
    caam_rng_config_t rngConfig;
    CAAM_RNG_GetDefaultConfig(&rngConfig);

    /* reset RNG */
    base->RTMCTL = CAAM_RTMCTL_PRGM_MASK | CAAM_RTMCTL_ERR_MASK | CAAM_RTMCTL_RST_DEF_MASK |
                   CAAM_RTMCTL_SAMP_MODE(kCAAM_RNG_SampleModeRaw);
    base->RTMCTL = CAAM_RTMCTL_ERR_MASK | CAAM_RTMCTL_SAMP_MODE(config->rngSampleMode) |
                   CAAM_RTMCTL_OSC_DIV(config->rngRingOscDiv);

    caam_handle_t handle;
    handle.jobRing = kCAAM_JobRing0;
    CAAM_RNG_Init(base, &handle, kCAAM_RngStateHandle0, &rngConfig);
    CAAM_RNG_GenerateSecureKey(base, &handle, NULL);

    /* set RANDDPAR bit for the AESA to reseed its DPA mask using data from kCAAM_RngStateHandle0 */
    /* also set other bits to 1 for security */
    base->SCFGR = CAAM_SCFGR_LCK_TRNG(config->scfgrLockTrngProgramMode) |
                  CAAM_SCFGR_RDB(config->scfgrEnableRandomDataBuffer) |
                  CAAM_SCFGR_RNGSH0(config->scfgrRandomRngStateHandle0) |
                  CAAM_SCFGR_RANDDPAR(config->scfgrRandomDpaResistance) | CAAM_SCFGR_PRIBLOB(config->scfgrPriblob);
}

status_t CAAM_Wait(CAAM_Type *base, caam_handle_t *handle, void *descriptor, caam_wait_mode_t mode)
{
    /* poll output ring for the specified job descriptor */
    status_t status;
    bool wait;
    bool found;

    wait = true;
    status = kStatus_Success;
    found = false;

    while (wait)
    {
        /* any result available on this job ring? */
        if (caam_output_ring_get_slots_full(base, handle->jobRing))
        {
            status = caam_out_job_ring_test_and_remove(base, handle->jobRing, descriptor, &wait, &found);
        }

        /* non-blocking mode polls output ring once */
        if (mode == kCAAM_Nonblocking)
        {
            wait = false; /* exit the while() */
            if (!found)
            {
                status = kStatus_CAAM_Again; /* job not in the tested ring */
            }
        }
    }
    return status;
}

void CAAM_Deinit(CAAM_Type *base)
{
    caam_handle_t handle;
    handle.jobRing = kCAAM_JobRing0;
    CAAM_RNG_Deinit(base, &handle, kCAAM_RngStateHandle0);

    base->JOBRING[0].JRCR_JR = CAAM_JRCR_JR_RESET_MASK;
    base->JOBRING[0].JRCR_JR = CAAM_JRCR_JR_RESET_MASK;
    base->JOBRING[1].JRCR_JR = CAAM_JRCR_JR_RESET_MASK;
    base->JOBRING[1].JRCR_JR = CAAM_JRCR_JR_RESET_MASK;
    base->DRR = CAAM_DRR_RST0_MASK;
    base->MCFGR = CAAM_MCFGR_SWRST_MASK;                           /* reset */
    base->MCFGR = CAAM_MCFGR_SWRST_MASK | CAAM_MCFGR_DMA_RST_MASK; /* reset DMA */
    base->MCFGR = 0x00082300u;                                     /* (reset value) */

    while (0 == (base->RTMCTL & CAAM_RTMCTL_TSTOP_OK_MASK))
    {
    }

    /* reset RNG */
    base->RTMCTL = CAAM_RTMCTL_PRGM_MASK | CAAM_RTMCTL_ERR_MASK | CAAM_RTMCTL_RST_DEF_MASK |
                   CAAM_RTMCTL_SAMP_MODE(kCAAM_RNG_SampleModeRaw);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(kCLOCK_Caam0);
#endif
}

status_t CAAM_ExternalKeyTransfer(CAAM_Type *base,
                                  caam_handle_t *handle,
                                  caam_ext_key_xfr_source_t keySource,
                                  size_t keySize)
{
    caam_desc_aes_ecb_t descBuf = {0};
    status_t status;

    descBuf[0] = 0xB0800002u; /* HEADER */
    descBuf[1] = 0x40000000u; /* EXT KEY XFR command. */

    switch (keySource)
    {
        case kCAAM_ExtKeyXfr_KeyRegisterClass1:
        case kCAAM_ExtKeyXfr_KeyRegisterClass2:
        case kCAAM_ExtKeyXfr_PkhaRamE:
            descBuf[1] |= ((uint32_t)keySource << 16) | keySize;
            status = kStatus_Success;
            break;

        default:
            status = kStatus_InvalidArgument;
            break;
    }

    if (status != kStatus_Success)
    {
        return status;
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &descBuf[0]);
    } while (status != kStatus_Success);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t *key,
                             size_t keySize)
{
    caam_desc_aes_ecb_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key, keySize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t *key,
                             size_t keySize)
{
    caam_desc_aes_ecb_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key, keySize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t iv[16],
                             const uint8_t *key,
                             size_t keySize)
{
    caam_desc_aes_cbc_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key, keySize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t iv[16],
                             const uint8_t *key,
                             size_t keySize)
{
    caam_desc_aes_cbc_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key, keySize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_CryptCtr(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *input,
                           uint8_t *output,
                           size_t size,
                           uint8_t counter[16],
                           const uint8_t *key,
                           size_t keySize,
                           uint8_t counterlast[16],
                           size_t *szLeft)
{
    caam_desc_aes_ctr_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_CryptCtrNonBlocking(base, handle, descBuf, input, output, size, counter, key, keySize,
                                              counterlast, szLeft);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_EncryptTagCcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *plaintext,
                                uint8_t *ciphertext,
                                size_t size,
                                const uint8_t *iv,
                                size_t ivSize,
                                const uint8_t *aad,
                                size_t aadSize,
                                const uint8_t *key,
                                size_t keySize,
                                uint8_t *tag,
                                size_t tagSize)
{
    caam_desc_aes_ccm_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_EncryptTagCcmNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, ivSize, aad,
                                                   aadSize, key, keySize, tag, tagSize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptTagCcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *ciphertext,
                                uint8_t *plaintext,
                                size_t size,
                                const uint8_t *iv,
                                size_t ivSize,
                                const uint8_t *aad,
                                size_t aadSize,
                                const uint8_t *key,
                                size_t keySize,
                                const uint8_t *tag,
                                size_t tagSize)
{
    caam_desc_aes_ccm_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_DecryptTagCcmNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, ivSize, aad,
                                                   aadSize, key, keySize, tag, tagSize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_EncryptTagGcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *plaintext,
                                uint8_t *ciphertext,
                                size_t size,
                                const uint8_t *iv,
                                size_t ivSize,
                                const uint8_t *aad,
                                size_t aadSize,
                                const uint8_t *key,
                                size_t keySize,
                                uint8_t *tag,
                                size_t tagSize)
{
    caam_desc_aes_gcm_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_EncryptTagGcmNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, ivSize, aad,
                                                   aadSize, key, keySize, tag, tagSize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptTagGcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *ciphertext,
                                uint8_t *plaintext,
                                size_t size,
                                const uint8_t *iv,
                                size_t ivSize,
                                const uint8_t *aad,
                                size_t aadSize,
                                const uint8_t *key,
                                size_t keySize,
                                const uint8_t *tag,
                                size_t tagSize)
{
    caam_desc_aes_gcm_t descBuf;
    status_t status;

    do
    {
        status = CAAM_AES_DecryptTagGcmNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, ivSize, aad,
                                                   aadSize, key, keySize, tag, tagSize);
    } while (status == kStatus_CAAM_Again);

    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

/*******************************************************************************
 * HASH Code static
 ******************************************************************************/
static status_t caam_hash_check_input_alg(caam_hash_algo_t algo)
{
    if ((algo != kCAAM_XcbcMac) && (algo != kCAAM_Cmac) && (algo != kCAAM_Sha1) && (algo != kCAAM_Sha224) &&
        (algo != kCAAM_Sha256) && (algo != kCAAM_Sha384) && (algo != kCAAM_Sha512))
    {
        return kStatus_InvalidArgument;
    }
    return kStatus_Success;
}

static inline bool caam_hash_alg_is_cmac(caam_hash_algo_t algo)
{
    return ((algo == kCAAM_XcbcMac) || (algo == kCAAM_Cmac));
}

static inline bool caam_hash_alg_is_sha(caam_hash_algo_t algo)
{
    return ((algo == kCAAM_Sha1) || (algo == kCAAM_Sha224) || (algo == kCAAM_Sha256) || (algo == kCAAM_Sha384) ||
            (algo == kCAAM_Sha512));
}

static status_t caam_hash_check_input_args(
    CAAM_Type *base, caam_hash_ctx_t *ctx, caam_hash_algo_t algo, const uint8_t *key, uint32_t keySize)
{
    /* Check validity of input algorithm */
    if (kStatus_Success != caam_hash_check_input_alg(algo))
    {
        return kStatus_InvalidArgument;
    }

    if ((NULL == ctx) || (NULL == base))
    {
        return kStatus_InvalidArgument;
    }

    if (caam_hash_alg_is_cmac(algo))
    {
        if ((NULL == key) || (!caam_check_key_size(keySize)))
        {
            return kStatus_InvalidArgument;
        }
    }

    return kStatus_Success;
}

static status_t caam_hash_check_context(caam_hash_ctx_internal_t *ctxInternal, const uint8_t *data)
{
    if ((NULL == data) || (NULL == ctxInternal) || (NULL == ctxInternal->base) ||
        (kStatus_Success != caam_hash_check_input_alg(ctxInternal->algo)))
    {
        return kStatus_InvalidArgument;
    }
    return kStatus_Success;
}

static uint32_t caam_hash_algo2mode(caam_hash_algo_t algo, uint32_t algState, uint32_t *algOutSize)
{
    uint32_t modeReg = 0u;
    uint32_t outSize = 0u;

    /* Set CAAM algorithm */
    switch (algo)
    {
        case kCAAM_XcbcMac:
            modeReg = (uint32_t)kCAAM_AlgorithmAES | (uint32_t)kCAAM_ModeXCBCMAC;
            outSize = 16u;
            break;
        case kCAAM_Cmac:
            modeReg = (uint32_t)kCAAM_AlgorithmAES | (uint32_t)kCAAM_ModeCMAC;
            outSize = 16u;
            break;
        case kCAAM_Sha1:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA1;
            outSize = kCAAM_OutLenSha1;
            break;
        case kCAAM_Sha224:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA224;
            outSize = kCAAM_OutLenSha224;
            break;
        case kCAAM_Sha256:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA256;
            outSize = kCAAM_OutLenSha256;
            break;
        case kCAAM_Sha384:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA384;
            outSize = kCAAM_OutLenSha384;
            break;
        case kCAAM_Sha512:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA512;
            outSize = kCAAM_OutLenSha512;
            break;
        default:
            break;
    }

    modeReg |= algState;
    if (algOutSize)
    {
        *algOutSize = outSize;
    }

    return modeReg;
}

static uint32_t caam_hash_algo2ctx_size(caam_hash_algo_t algo, uint32_t how)
{
    uint32_t ctxSize = 0u;

    /* Size of context in bytes for context switching */
    switch (algo)
    {
        case kCAAM_XcbcMac:
            if (how == 0)
            {
                ctxSize = 48u; /* add K3 and K2 */
            }
            else
            {
                ctxSize = 16u; /* only running or final MAC during UPDATE or FINALIZE or INITIALIZE/FINALIZE */
            }
            break;
        case kCAAM_Cmac:
            if (how == 0)
            {
                ctxSize = 32u; /* add L */
            }
            else
            {
                ctxSize = 16u; /* only running or final MAC during UPDATE or FINALIZE or INITIALIZE/FINALIZE */
            }
            break;
        /* MDHA use of the Context Register
        The Context Register stores the current digest and running message length. The running
        message length will be 8 bytes immediately following the active digest. The digest size is
        defined as follows:
        MD5: 16 bytes
        SHA-1: 20 bytes
        SHA-224: 28 bytes final digest; 32 bytes running digest
        SHA-256: 32 bytes
        SHA-384: 48 bytes final digest; 64 bytes running digest
        SHA-512: 64 bytes */
        case kCAAM_Sha1:
            ctxSize = 28u; /* 8 + 20 */
            break;
        case kCAAM_Sha224:
        case kCAAM_Sha256:
            ctxSize = 40u; /* 8 + 32 */
            break;
        case kCAAM_Sha384:
        case kCAAM_Sha512:
            ctxSize = 72u; /* 8 + 64 */
            break;
        default:
            break;
    }
    return ctxSize;
}

static const uint32_t templateHash[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x00000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x10200000u, /* LOAD bytes to Class Context Register. Offset 0 bytes. */
    /* 04 */ 0x00000000u, /* place: context address */

    /* 05 */ 0x80000000u, /* OPERATION (place either AES MAC or MDHA SHA) */
    /* 06 */ 0x21170000u, /* FIFO LOAD Class Message via SGT */
    /* 07 */ 0x00000000u, /* place: SGT address */
    /* 08 */ 0x50200000u, /* STORE bytes from Class Context Register offset 0 bytes. */
    /* 09 */ 0x00000000u, /* place: context address */

    /* 10 */ 0x60240000u, /* FIFO STORE from KEY to memory. */
    /* 11 */ 0x00000000u, /* place: derived key address ECB encrypted */
    /* 12 */ 0xA0C00000u, /* halt always with status 0 */

    /* 13 */ 0x00000000u, /* SGT entry 0 word 0 */
    /* 14 */ 0x00000000u, /* SGT entry 0 word 1 */
    /* 15 */ 0x00000000u, /* SGT entry 0 word 2 */
    /* 16 */ 0x00000000u, /* SGT entry 0 word 3 */

    /* 17 */ 0x00000000u, /* SGT entry 1 word 0 */
    /* 18 */ 0x00000000u, /* SGT entry 1 word 1 */
    /* 19 */ 0x00000000u, /* SGT entry 1 word 2 */
    /* 20 */ 0x00000000u, /* SGT entry 1 word 3 */
};

/*!
 * @brief Add data chunk to SGT table. Append after uncomplete block in ctxInternal if there is any.
 *
 * @param ctxInternal uncomplete block in the hash context - to be inserted before new data chunk
 * @param input new data chunk to insert
 * @param inputSize size in bytes of new data chunk to insert
 * @param numRemain number of bytes that remain in the last uncomplete block
 * @param algState in FINALIZE or INITIALIZE/FINALIZE we add also last uncomplete block bytes
 * @param sgt address of the SGT
 * @param last last call to this function adds Final Bit
 */
static uint32_t caam_hash_sgt_insert(caam_hash_ctx_internal_t *ctxInternal,
                                     const uint8_t *input,
                                     size_t inputSize,
                                     size_t *numRemain,
                                     caam_algorithm_state_t algState,
                                     caam_sgt_entry_t *sgt,
                                     caam_hash_sgt_entry_type_t last)
{
    /* configure SGT
     * *64 bytes multiple in kCAAM_HashInit or kCAAM_HashUpdate
     * *arbitrary amount of data in kCAAM_HashInitFinal or kCAAM_HashFinal
     * min 1 and max 2 SGT entries
     * 1) if there is any data in the context buffer, use it as one entry
     * 2) input as one entry
     */
    uint32_t numBlocks;
    uint32_t remain;
    uint32_t num;
    uint32_t currSgtEntry;

    uint32_t ctxBlksz = ctxInternal ? ctxInternal->blksz : 0;
    uint32_t ctxBlkAddr = ctxInternal ? (uint32_t)&ctxInternal->blk.b[0] : 0;

    currSgtEntry = 0;
    numBlocks = (inputSize + ctxBlksz) / CAAM_HASH_BLOCK_SIZE;
    remain = (inputSize + ctxBlksz) % CAAM_HASH_BLOCK_SIZE;

    /* number of bytes for processing
     * only full block multiple in INITIALIZE or UPDATE
     * any size in INITIALIZE/FINALIZE or FINALIZE
     */
    num = (CAAM_HASH_BLOCK_SIZE * numBlocks);
    if ((algState == kCAAM_AlgStateInitFinal) || (algState == kCAAM_AlgStateFinal))
    {
        num += remain; /* add also uncomplete bytes from last block in one of FINALIZE states */
        remain = 0;
    }
    if (numRemain)
    {
        *numRemain = remain;
    }

    if (ctxBlksz || (0 == ctxBlksz + inputSize))
    {
        sgt[currSgtEntry].address_l = ctxBlkAddr;
        sgt[currSgtEntry].length = ctxBlksz;
        if ((kCAAM_HashSgtEntryLast == last) && (!inputSize))
        {
            sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */
        }
        currSgtEntry++;
    }

    if (inputSize)
    {
        /* number of bytes for processing
         * only full block multiple in INITIALIZE or UPDATE
         * any size in INITIALIZE/FINALIZE or FINALIZE
         */
        sgt[currSgtEntry].address_l = (uint32_t)input;
        sgt[currSgtEntry].length = inputSize - remain;
        if (kCAAM_HashSgtEntryLast == last)
        {
            sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */
            sgt[currSgtEntry].offset = 0x80000000u;
        }
    }
    return num; /* no of bytes processed in total by these 1 or 2 SGT entries */
}

/*!
 * @brief Create job descriptor for the HASH request and schedule at CAAM job ring
 *
 *
 */
static status_t caam_hash_schedule_input_data(CAAM_Type *base,
                                              caam_handle_t *handle,
                                              caam_hash_algo_t algo,
                                              caam_sgt_entry_t *sgt,
                                              uint32_t dataSize,
                                              caam_hash_sgt_type_t sgtType,
                                              caam_algorithm_state_t algState,
                                              caam_desc_hash_t descriptor,
                                              size_t *outputSize,
                                              void *output,
                                              void *context,
                                              uint32_t keyAddr,
                                              uint32_t keySize)
{
    BUILD_ASSURE(sizeof(templateHash) <= sizeof(caam_desc_hash_t), caam_desc_hash_t_size);
    uint32_t descriptorSize = ARRAY_SIZE(templateHash);
    uint32_t algOutSize = 0;

    bool isSha = caam_hash_alg_is_sha(algo); /* MDHA engine */
                                             /* how many bytes to read from context register
                                              * we need caam_hash_algo2ctx_size() to return
                                              * full context size (to be used for context restore in descriptor[3])
                                              */
    uint32_t caamCtxSz = caam_hash_algo2ctx_size(algo, 0 /* full context */);

    caam_memcpy(descriptor, templateHash, sizeof(templateHash));

    /* MDHA is always Class 2 CHA, AESA configured at build time as Class 1 CHA */
    uint32_t class = isSha ? 0x04000000u : CAAM_AES_MAC_CLASS;

    /* add class to all commands that need it */
    descriptor[1] |= class;
    descriptor[3] |= class;
    descriptor[5] |= class;
    descriptor[6] |= class;
    descriptor[8] |= class;
    descriptor[10] |= class;

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* kCAAM_AlgStateInit or kCAAM_AlgStateInitFinal needs to skip context load as there is no context */
    if ((algState == kCAAM_AlgStateInit) || (algState == kCAAM_AlgStateInitFinal))
    {
        if (isSha)
        {
            /* HEADER can jump directly to MDHA operation */
            descriptor[0] |= 0x00050000; /* JUMP to descriptor[5] */
        }
        else
        {
            /* load KEY, then directly to AESA MAC operation */
            descriptor[1] |= (keySize & 0x3FFu);
            descriptor[2] = keyAddr;
            descriptor[3] = 0xA0000002u; /* JUMP to descriptor[5] */
        }
    }
    else
    {
        if (isSha)
        {
            /* MDHA SHA in Update state skips loading the KEY, as MDHA SHA has no configurable key
             * HEADER can jump directly to context restore
             */
            descriptor[0] |= 0x00030000;       /* JUMP to descriptor[3] */
            /* descriptor[1] = 0xA0000002u; */ /* JUMP to descriptor[3] */
        }
        else
        {
            /* load KEY */
            descriptor[1] |= (keySize & 0x3FFu);
            descriptor[2] = keyAddr;

            /* XCBC-MAC K1 derived key has been ECB encrypted (black key)
             * so it needs decrypt
             */
            if (kCAAM_XcbcMac == algo)
            {
                descriptor[1] |= 1u << 22; /* ENC */
            }
        }

        /* context restore */
        descriptor[3] |= caamCtxSz;
        descriptor[4] = (uint32_t)context;
    }

    /* OPERATION:
     * alg MDHA or AESA
     * mode INITIALIZE or UPDATE or FINALIZE or INITIALIZE/FINALIZE in algState argument
     */

    /* ALGORITHM OPERATION | CLASS | alg | aai | algState */
    descriptor[5] |= caam_hash_algo2mode(algo, (uint32_t)algState << 2, &algOutSize);

    /* configure SGT */
    descriptor[6] |= (0xFFFFu & dataSize);
    if (kCAAM_HashSgtInternal == sgtType)
    {
        descriptor[7] = (uint32_t)&descriptor[kCAAM_HashDescriptorSgtIdx]; /* use SGT embedded in the job descriptor */
        caam_memcpy(&descriptor[kCAAM_HashDescriptorSgtIdx], sgt, sizeof(caam_hash_internal_sgt_t));
    }
    else
    {
        descriptor[7] = (uint32_t)sgt;
    }

    /* save context: context switch init or running or result */
    if ((kCAAM_AlgStateFinal == algState) || (kCAAM_AlgStateInitFinal == algState))
    {
        if (outputSize)
        {
            if (algOutSize < *outputSize)
            {
                *outputSize = algOutSize;
            }
            else
            {
                algOutSize = *outputSize;
            }
        }
        caamCtxSz = algOutSize;
    }
    else
    {
        uint32_t how = (algState == kCAAM_AlgStateInit) ? 0 : 1; /* context switch needs full, then running */
        caamCtxSz = caam_hash_algo2ctx_size(algo, how);
    }
    descriptor[8] |= caamCtxSz;
    if ((kCAAM_AlgStateFinal == algState) || (kCAAM_AlgStateInitFinal == algState))
    {
        /* final result write to output */
        descriptor[9] = (uint32_t)output;
    }
    else
    {
        /* context switch write to ctxInternal */
        descriptor[9] = (uint32_t)context;
    }

    /* save the derived key K1 in XCBC-MAC. only if context switch. */
    if ((kCAAM_AlgStateInit == algState) && (kCAAM_XcbcMac == algo))
    {
        descriptor[10] |= (keySize & 0x3FFu);
        descriptor[11] = keyAddr;
    }
    else
    {
        descriptor[10] = descriptor[12]; /* always halt with status 0x0 (normal) */
    }

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

/*!
 * @brief Add uncomplete block (ctxInternal), then append new data (to current hash).
 *
 *
 */
static status_t caam_hash_append_data(caam_hash_ctx_internal_t *ctxInternal,
                                      const uint8_t *input,
                                      size_t inputSize,
                                      caam_algorithm_state_t algState,
                                      caam_desc_hash_t descriptor,
                                      size_t *numRemain,
                                      void *output,
                                      size_t *outputSize)
{
    caam_hash_internal_sgt_t sgt;
    memset(&sgt, 0, sizeof(sgt));
    size_t num = caam_hash_sgt_insert(ctxInternal, input, inputSize, numRemain, algState, sgt, kCAAM_HashSgtEntryLast);
    return caam_hash_schedule_input_data(ctxInternal->base, ctxInternal->handle, ctxInternal->algo, sgt, num,
                                         kCAAM_HashSgtInternal, algState, descriptor, outputSize, output,
                                         &ctxInternal->word[0], (uint32_t)&ctxInternal->word[kCAAM_HashCtxKeyStartIdx],
                                         ctxInternal->word[kCAAM_HashCtxKeySize]);
}

status_t CAAM_HASH_Init(CAAM_Type *base,
                        caam_handle_t *handle,
                        caam_hash_ctx_t *ctx,
                        caam_hash_algo_t algo,
                        const uint8_t *key,
                        size_t keySize)
{
    status_t ret;
    caam_hash_ctx_internal_t *ctxInternal;
    uint32_t i;

    ret = caam_hash_check_input_args(base, ctx, algo, key, keySize);
    if (ret != kStatus_Success)
    {
        return ret;
    }

    /* set algorithm in context struct for later use */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    ctxInternal->algo = algo;
    for (i = 0; i < kCAAM_HashCtxNumWords; i++)
    {
        ctxInternal->word[i] = 0u;
    }

    /* Steps required only using AES engine */
    if (caam_hash_alg_is_cmac(algo))
    {
        /* store input key and key length in context struct for later use */
        ctxInternal->word[kCAAM_HashCtxKeySize] = keySize;
        caam_memcpy(&ctxInternal->word[kCAAM_HashCtxKeyStartIdx], key, keySize);
    }

    ctxInternal->blksz = 0u;
    for (i = 0; i < ARRAY_SIZE(ctxInternal->blk.w); i++)
    {
        ctxInternal->blk.w[0] = 0u;
    }
    ctxInternal->state = kCAAM_HashInit;
    ctxInternal->base = base;
    ctxInternal->handle = handle;

    return kStatus_Success;
}

status_t CAAM_HASH_Update(caam_hash_ctx_t *ctx, const uint8_t *input, size_t inputSize)
{
    caam_desc_hash_t descBuf;
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;
    bool isUpdateState;
    size_t numRemain = 0;

    /* compile time check for the correct structure size */
    BUILD_ASSURE(sizeof(caam_hash_ctx_internal_t) <= sizeof(caam_hash_ctx_t), caam_hash_ctx_internal_t_size);

    if (0 == inputSize)
    {
        return kStatus_Success;
    }

    /* we do caam_memcpy() input stream, up to buffer size
     * of 64 bytes. then if I have more I have to
     * 1) load Class 2 context
     * 2) schedule CAAM job with INITIALIZE or UPDATE mode (simple if only 64 bytes block is processed. SG table for 2
     * and more)
     * 3) in step 2 process all full 64 bytes blocks
     * 4) copy last not-full buffer size data to buffer.
     * 5) save Class 2 context
     */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, input);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* if we are still less than 64 bytes, keep only in context */
    if ((ctxInternal->blksz + inputSize) <= CAAM_HASH_BLOCK_SIZE)
    {
        caam_memcpy((&ctxInternal->blk.b[0]) + ctxInternal->blksz, input, inputSize);
        ctxInternal->blksz += inputSize;
        return status;
    }
    else
    {
        isUpdateState = ctxInternal->state == kCAAM_HashUpdate;
        if (!isUpdateState)
        {
            /* Step 2: schedule CAAM job in INITIALIZE mode.
             */
            ctxInternal->state = kCAAM_HashUpdate;
            /* skip load context as there is no running context yet. */
            status = caam_hash_append_data(ctxInternal, input, inputSize, kCAAM_AlgStateInit, descBuf, &numRemain, NULL,
                                           NULL);
        }
    }

    if (kStatus_Success != status)
    {
        return status;
    }

    if (isUpdateState)
    {
        /* Step 2: schedule CAAM job in UPDATE mode.
         */

        /* process input data and save CAAM context to context structure */
        status =
            caam_hash_append_data(ctxInternal, input, inputSize, kCAAM_AlgStateUpdate, descBuf, &numRemain, NULL, NULL);
    }

    /* blocking wait? */
    status = CAAM_Wait(ctxInternal->base, ctxInternal->handle, descBuf, kCAAM_Blocking);

    /* after job is complete, copy numRemain bytes at the end of the input[] to the context */
    caam_memcpy((&ctxInternal->blk.b[0]), input + inputSize - numRemain, numRemain);
    ctxInternal->blksz = numRemain;

    return status;
}

status_t CAAM_HASH_UpdateNonBlocking(caam_hash_ctx_t *ctx, const uint8_t *input, size_t inputSize)
{
    status_t status;

    caam_hash_ctx_internal_t *ctxInternal;

    if (0 == inputSize)
    {
        return kStatus_Success;
    }

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, input);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* Add input data chunk to SGT */
    uint32_t currSgtEntry = ctxInternal->blksz;
    if (currSgtEntry >= kCAAM_HashSgtMaxCtxEntries)
    {
        return kStatus_InvalidArgument;
    }

    caam_sgt_entry_t *sgt = &((caam_sgt_entry_t *)ctxInternal)[currSgtEntry];
    caam_hash_sgt_insert(NULL, input, inputSize, NULL, kCAAM_AlgStateInitFinal, sgt,
                         kCAAM_HashSgtEntryNotLast /* not last. we don't know if this is the last chunk */);
    if (inputSize)
    {
        ctxInternal->blksz++;
    }

    return status;
}

status_t CAAM_HASH_Finish(caam_hash_ctx_t *ctx, uint8_t *output, size_t *outputSize)
{
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;
    caam_desc_hash_t descBuf;
    caam_algorithm_state_t algState;

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, output);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* determine algorithm state to configure
     * based on prior processing.
     * If at least one full block has been processed during HASH_Update() then the state in ctxInternal
     * will be set to kCAAM_HashUpdate and so we will configure FINALIZE algorithm state.
     * Otherwise there is data only in the ctxInternal that we can process in INITIALIZE/FINALIZE.
     */
    if (ctxInternal->state == kCAAM_HashInit)
    {
        algState = kCAAM_AlgStateInitFinal;
    }
    else
    {
        algState = kCAAM_AlgStateFinal;
    }

    status = caam_hash_append_data(
        ctxInternal, NULL, 0, /* we process only blksz bytes in ctxInternal, so giving NULL and zero size here */
        algState, descBuf, NULL, output, outputSize);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* blocking wait */
    status = CAAM_Wait(ctxInternal->base, ctxInternal->handle, descBuf, kCAAM_Blocking);
    memset(ctx, 0, sizeof(caam_hash_ctx_t));
    return status;
}

status_t CAAM_HASH_FinishNonBlocking(caam_hash_ctx_t *ctx,
                                     caam_desc_hash_t descriptor,
                                     uint8_t *output,
                                     size_t *outputSize)
{
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, output);
    if (kStatus_Success != status)
    {
        return status;
    }

    uint32_t currSgtEntry = ctxInternal->blksz;
    if (currSgtEntry > kCAAM_HashSgtMaxCtxEntries)
    {
        return kStatus_InvalidArgument;
    }

    caam_sgt_entry_t *sgt = &((caam_sgt_entry_t *)ctxInternal)[0];

    /* mark currSgtEntry with Final Bit */
    int i;
    uint32_t totalLength = 0;
    for (i = 0; i < currSgtEntry; i++)
    {
        totalLength += sgt[i].length;
    }
    sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */

    status = caam_hash_schedule_input_data(ctxInternal->base, ctxInternal->handle, ctxInternal->algo, sgt, totalLength,
                                           kCAAM_HashSgtExternal, kCAAM_AlgStateInitFinal, descriptor, outputSize,
                                           output, NULL, (uint32_t)&ctxInternal->word[kCAAM_HashCtxKeyStartIdx],
                                           ctxInternal->word[kCAAM_HashCtxKeySize]);
    return kStatus_Success;
}

status_t CAAM_HASH(CAAM_Type *base,
                   caam_handle_t *handle,
                   caam_hash_algo_t algo,
                   const uint8_t *input,
                   size_t inputSize,
                   const uint8_t *key,
                   size_t keySize,
                   uint8_t *output,
                   size_t *outputSize)
{
    status_t status;
    caam_desc_hash_t descBuf;

    status = CAAM_HASH_NonBlocking(base, handle, descBuf, algo, input, inputSize, key, keySize, output, outputSize);
    if (kStatus_Success != status)
    {
        return status;
    }

    status = CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
    return status;
}

status_t CAAM_HASH_NonBlocking(CAAM_Type *base,
                               caam_handle_t *handle,
                               caam_desc_hash_t descriptor,
                               caam_hash_algo_t algo,
                               const uint8_t *input,
                               size_t inputSize,
                               const uint8_t *key,
                               size_t keySize,
                               uint8_t *output,
                               size_t *outputSize)
{
    status_t status;
    caam_algorithm_state_t algState;
    caam_hash_internal_sgt_t sgt;

    memset(&sgt, 0, sizeof(sgt));

    algState = kCAAM_AlgStateInitFinal;
    uint32_t num = caam_hash_sgt_insert(NULL,             /* no ctxInternal data to pre-pend before input data chunk */
                                        input, inputSize, /* data and size in bytes */
                                        NULL, /* all data is processed during kCAAM_AlgStateInitFinal, nothing remain */
                                        algState, sgt, kCAAM_HashSgtEntryLast); /* sgt table, entry 0 word 0 */

    /* schedule the request at CAAM */
    status = caam_hash_schedule_input_data(base, handle, algo, sgt, num, kCAAM_HashSgtInternal, algState, descriptor,
                                           outputSize, output, NULL, (uint32_t)key, keySize);
    return status;
}

/*******************************************************************************
 * RNG Code public
 ******************************************************************************/

status_t CAAM_RNG_GetDefaultConfig(caam_rng_config_t *config)
{
    status_t status;
    if (config)
    {
        config->autoReseedInterval = 0; /* zero means hardware default of 10.000.000 will be used */
        config->personalString = NULL;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_InvalidArgument;
    }

    return status;
}

status_t CAAM_RNG_Init(CAAM_Type *base,
                       caam_handle_t *handle,
                       caam_rng_state_handle_t stateHandle,
                       const caam_rng_config_t *config)
{
    status_t status;

    /* create job descriptor */
    caam_desc_rng_t rngInstantiate = {0};
    rngInstantiate[0] = 0xB0800006u;
    rngInstantiate[1] = 0x12200020u; /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
    rngInstantiate[2] = (uint32_t)config->personalString;
    rngInstantiate[3] = 0x12820004u;                /* LOAD Immediate 4 bytes to Class 1 Data Size Register. */
    rngInstantiate[4] = config->autoReseedInterval; /* value for the Class 1 Data Size Register */
    rngInstantiate[5] = 0x82500006u;                /* RNG instantiate state handle:
                                              *   TST=0 for normal non-deterministic mode
                                              *   PR=1 for prediction resistance
                                              */

    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngInstantiate[5] |= 1u << 4;
    }

    /* default auto reseed interval */
    if (config->autoReseedInterval == 0)
    {
        rngInstantiate[3] = 0xA0000002u; /* jump to current index + 2 (=5) */
    }

    /* optional personalization string present */
    if (config->personalString)
    {
        rngInstantiate[5] |= 1u << 10; /* set PS bit in ALG OPERATION (AS=01 Instantiate) */
    }
    else
    {
        rngInstantiate[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngInstantiate[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngInstantiate[0], kCAAM_Blocking);

    return status;
}

status_t CAAM_RNG_Deinit(CAAM_Type *base, caam_handle_t *handle, caam_rng_state_handle_t stateHandle)
{
    status_t status;

    /* create job descriptor */
    caam_desc_rng_t rngUninstantiate = {0};
    rngUninstantiate[0] = 0xB0800002u; /* HEADER */
    rngUninstantiate[1] = 0x8250000Cu; /* ALG OPERATION: RNG uninstantiate state handle (AS=11 Uninstantiate) */

    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngUninstantiate[1] |= 1u << 4;
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngUninstantiate[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngUninstantiate[0], kCAAM_Blocking);

    return status;
}

status_t CAAM_RNG_GenerateSecureKey(CAAM_Type *base, caam_handle_t *handle, caam_rng_generic256_t additionalEntropy)
{
    status_t status;

    /* create job descriptor */
    caam_desc_rng_t rngGenSeckey = {0};
    rngGenSeckey[0] = 0xB0800004u; /* HEADER */
    rngGenSeckey[1] = 0x12200020u; /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
    rngGenSeckey[2] = (uint32_t)additionalEntropy;
    rngGenSeckey[3] = 0x82501000u; /* set SK bit in ALG OPERATION (AS=00 Generate) */

    /* optional additional input included */
    if (additionalEntropy)
    {
        rngGenSeckey[3] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        rngGenSeckey[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngGenSeckey[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngGenSeckey[0], kCAAM_Blocking);
    return status;
}

status_t CAAM_RNG_Reseed(CAAM_Type *base,
                         caam_handle_t *handle,
                         caam_rng_state_handle_t stateHandle,
                         caam_rng_generic256_t additionalEntropy)
{
    status_t status;

    /* create job descriptor */
    caam_desc_rng_t rngReseed = {0};
    rngReseed[0] = 0xB0800004u; /* HEADER */
    rngReseed[1] = 0x12200020u; /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
    rngReseed[2] = (uint32_t)additionalEntropy;
    rngReseed[3] = 0x8250000Au; /* ALG OPERATION: RNG reseed state handle (AS=10 Reseed) */

    /* optional additional input included */
    if (additionalEntropy)
    {
        rngReseed[3] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        rngReseed[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* select state handle */
    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngReseed[3] |= 1u << 4;
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngReseed[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngReseed[0], kCAAM_Blocking);
    return status;
}

status_t CAAM_RNG_GetRandomData(CAAM_Type *base,
                                caam_handle_t *handle,
                                caam_rng_state_handle_t stateHandle,
                                void *data,
                                size_t dataSize,
                                caam_rng_random_type_t dataType,
                                caam_rng_generic256_t additionalEntropy)
{
    status_t status;
    caam_desc_rng_t descBuf;

    do
    {
        status = CAAM_RNG_GetRandomDataNonBlocking(base, handle, stateHandle, descBuf, data, dataSize, dataType,
                                                   additionalEntropy);
    } while (status == kStatus_CAAM_Again);

    if (kStatus_Success != status)
    {
        return status;
    }

    status = CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
    return status;
}

static const uint32_t templateRng[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x12200020u, /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
    /* 02 */ 0x00000000u, /* place: additional input address */
    /* 03 */ 0x12820004u, /* LOAD Class 1 Data Size Register by IMM data */
    /* 04 */ 0x00000000u, /* place: data size to generate */
    /* 05 */ 0x82500002u, /* RNG generate */
    /* 06 */ 0x60300000u, /* FIFO STORE message */
    /* 07 */ 0x00000000u, /* place: destination address */
};

status_t CAAM_RNG_GetRandomDataNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_rng_state_handle_t stateHandle,
                                           caam_desc_rng_t descriptor,
                                           void *data,
                                           size_t dataSize,
                                           caam_rng_random_type_t dataType,
                                           caam_rng_generic256_t additionalEntropy)
{
    /* create job descriptor */
    BUILD_ASSURE(sizeof(templateRng) <= sizeof(caam_desc_rng_t), caam_desc_rng_t_size);
    uint32_t descriptorSize = ARRAY_SIZE(templateRng);

    caam_memcpy(descriptor, templateRng, sizeof(templateRng));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* optional additional input included */
    if (additionalEntropy)
    {
        descriptor[2] = (uint32_t)additionalEntropy;
        descriptor[5] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        descriptor[0] |= 3u << 16; /* start at index 3 */
    }

    descriptor[4] = dataSize;
    descriptor[6] |= (dataSize & 0xFFFFu);
    descriptor[7] = (uint32_t)data;

    /* select state handle */
    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        descriptor[5] |= 1u << 4;
    }

    /* configure type of data */
    if (dataType == kCAAM_RngDataNonZero)
    {
        descriptor[5] |= 1u << 8; /* set NZB bit in ALG OPERATION */
    }

    if (dataType == kCAAM_RngDataOddParity)
    {
        descriptor[5] |= 1u << 9; /* set OBP bit in ALG OPERATION */
    }

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static const uint32_t templateCipherDes[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02800000u, /* KEY Class 1 IMM */
    /* 02 */ 0x00000000u, /* IMM key1 0-3 */
    /* 03 */ 0x00000000u, /* IMM key1 4-8 */
    /* 04 */ 0x00000000u, /* IMM key2 0-3 */
    /* 05 */ 0x00000000u, /* IMM key2 4-8 */
    /* 06 */ 0x00000000u, /* IMM key3 0-3 */
    /* 07 */ 0x00000000u, /* IMM key3 4-8 */
    /* 08 */ 0x12200008u, /* LOAD 8 bytes of iv to Class 1 Context Register */
    /* 09 */ 0x00000000u, /* place: iv address */
    /* 10 */ 0x22130000u, /* FIFO LOAD Message */
    /* 11 */ 0x00000000u, /* place: source address */
    /* 12 */ 0x60300000u, /* FIFO STORE Message */
    /* 13 */ 0x00000000u, /* place: destination address */
    /* 14 */ 0x82200000u, /* OPERATION: DES Decrypt, AS = zeroes, AAI = zeroes (CTR) */
};

/*******************************************************************************
 * DES Code public
 ******************************************************************************/

status_t CAAM_DES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_ASSURE(sizeof(caam_desc_cipher_des_t) >= sizeof(templateCipherDes), caam_desc_cipher_des_t_size);
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000006u; /* ECB has no context, jump to currIdx+6 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u; /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000006u; /* ECB has no context, jump to currIdx+6 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             size_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        size_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES2_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key1, key2);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000004u; /* ECB has no context, jump to currIdx+4 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u;  /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */
    descriptor[14] |= 0x10000; /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES2_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key1, key2);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000004u; /* ECB has no context, jump to currIdx+4 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */
    descriptor[14] |= 0x10000;    /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES3_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key1, key2, key3);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[8] = 0xA0000002u; /* ECB has no context, jump to currIdx+2 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u;  /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */
    descriptor[14] |= 0x10000; /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES3_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key1, key2, key3);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[8] = 0xA0000002u; /* ECB has no context, jump to currIdx+2 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */
    descriptor[14] |= 0x10000;    /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              size_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_CAAM_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         size_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    caam_memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    caam_memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    caam_memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

#define DESC_HEADER 0xB0800000u
#define DESC_HEADER_ADD_DESCLEN(cmdWord, len) (cmdWord |= len)

#define DESC_FLOADA 0x220C0000u
#define DESC_FLOADB 0x220D0000u
#define DESC_FLOADN 0x22080000u
#define DESC_KEY_E_ 0x02010000u
#define DESC_STOREB 0x600D0000u
#define DESC_STORE_ 0x52110004u
#define DESC_ADD_LEN(cmdWord, len) (cmdWord |= len)
#define DESC_SET_ADDR(cmdWord, addr) (cmdWord = (uint32_t)(addr))

#define DESC_FLOADA0 0x22000000u
#define DESC_FLOADA1 0x22010000u
#define DESC_FLOADA3 0x22030000u
#define DESC_FLOADB0 0x22040000u
#define DESC_FLOADB1 0x22050000u
#define DESC_FLOADB2 0x22060000u
#define DESC_FLOADB3 0x22070000u
#define DESC_STOREB1 0x60050000u
#define DESC_STOREB2 0x60060000u

static const uint32_t templateArithmeticPKHA[] = {
    /* 00 */ DESC_HEADER, /* HEADER */
    /* 01 */ 0x81800001u, /* OPERATION: PKHA OPERATION: clear memory function. */
    /* 02 */ 0xA2000001u, /* JMP always to next command. Done checkpoint (wait for Class 1 Done) */
    /* 03 */ DESC_FLOADN, /* FIFO LOAD PKHA N */
    /* 04 */ 0x00000000u, /* place: N address */
    /* 05 */ DESC_FLOADA, /* FIFO LOAD PKHA A */
    /* 06 */ 0x00000000u, /* place: A address */
    /* 07 */ DESC_FLOADB, /* FIFO LOAD PKHA B */
    /* 08 */ 0x00000000u, /* place: B address */
    /* 09 */ DESC_KEY_E_, /* KEY PKHA E */
    /* 10 */ 0x00000000u, /* place: E address */
    /* 11 */ 0x81800000u, /* OPERATION: PKHA OPERATION: Arithmetic Functions. */
    /* 12 */ DESC_STOREB, /* FIFO STORE PKHA B */
    /* 13 */ 0x00000000u, /* place: result address */
};

/*! @brief PKHA functions - arithmetic, copy/clear memory. */
typedef enum _caam_pkha_func_t
{
    kCAAM_PKHA_ClearMem = 1U,
    kCAAM_PKHA_ArithModAdd = 2U,         /*!< (A + B) mod N */
    kCAAM_PKHA_ArithModSub1 = 3U,        /*!< (A - B) mod N */
    kCAAM_PKHA_ArithModSub2 = 4U,        /*!< (B - A) mod N */
    kCAAM_PKHA_ArithModMul = 5U,         /*!< (A x B) mod N */
    kCAAM_PKHA_ArithModExp = 6U,         /*!< (A^E) mod N */
    kCAAM_PKHA_ArithModRed = 7U,         /*!< (A) mod N */
    kCAAM_PKHA_ArithModInv = 8U,         /*!< (A^-1) mod N */
    kCAAM_PKHA_ArithEccAdd = 9U,         /*!< (P1 + P2) */
    kCAAM_PKHA_ArithEccDouble = 10U,     /*!< (P2 + P2) */
    kCAAM_PKHA_ArithEccMul = 11U,        /*!< (E x P1) */
    kCAAM_PKHA_ArithModR2 = 12U,         /*!< (R^2 mod N) */
    kCAAM_PKHA_ArithGcd = 14U,           /*!< GCD (A, N) */
    kCAAM_PKHA_ArithPrimalityTest = 15U, /*!< Miller-Rabin */
    kCAAM_PKHA_CopyMemSizeN = 16U,
    kCAAM_PKHA_CopyMemSizeSrc = 17U,
} caam_pkha_func_t;

/*! @brief Register areas for PKHA clear memory operations. */
typedef enum _caam_pkha_reg_area
{
    kCAAM_PKHA_RegA = 8U,
    kCAAM_PKHA_RegB = 4U,
    kCAAM_PKHA_RegE = 2U,
    kCAAM_PKHA_RegN = 1U,
    kCAAM_PKHA_RegAll = kCAAM_PKHA_RegA | kCAAM_PKHA_RegB | kCAAM_PKHA_RegE | kCAAM_PKHA_RegN,
} caam_pkha_reg_area_t;

/*! @brief Quadrant areas for 4096-bit registers for PKHA copy memory
 * operations. */
typedef enum _caam_pkha_quad_area_t
{
    kCAAM_PKHA_Quad0 = 0U,
    kCAAM_PKHA_Quad1 = 1U,
    kCAAM_PKHA_Quad2 = 2U,
    kCAAM_PKHA_Quad3 = 3U,
} caam_pkha_quad_area_t;

/*! @brief User-supplied (R^2 mod N) input or CAAM should calculate. */
typedef enum _caam_pkha_r2_t
{
    kCAAM_PKHA_CalcR2 = 0U, /*!< Calculate (R^2 mod N) */
    kCAAM_PKHA_InputR2 = 1U /*!< (R^2 mod N) supplied as input */
} caam_pkha_r2_t;

/*! @brief CAAM PKHA parameters */
typedef struct _caam_pkha_mode_params_t
{
    caam_pkha_func_t func;
    caam_pkha_f2m_t arithType;
    caam_pkha_montgomery_form_t montFormIn;
    caam_pkha_montgomery_form_t montFormOut;
    caam_pkha_reg_area_t srcReg;
    caam_pkha_quad_area_t srcQuad;
    caam_pkha_reg_area_t dstReg;
    caam_pkha_quad_area_t dstQuad;
    caam_pkha_timing_t equalTime;
    caam_pkha_r2_t r2modn;
} caam_pkha_mode_params_t;

static void caam_pkha_default_parms(caam_pkha_mode_params_t *params)
{
    params->func = (caam_pkha_func_t)0;
    params->arithType = kCAAM_PKHA_IntegerArith;
    params->montFormIn = kCAAM_PKHA_NormalValue;
    params->montFormOut = kCAAM_PKHA_NormalValue;
    params->srcReg = kCAAM_PKHA_RegAll;
    params->srcQuad = kCAAM_PKHA_Quad0;
    params->dstReg = kCAAM_PKHA_RegAll;
    params->dstQuad = kCAAM_PKHA_Quad0;
    params->equalTime = kCAAM_PKHA_NoTimingEqualized;
    params->r2modn = kCAAM_PKHA_CalcR2;
}

static void caam_pkha_mode_set_src_reg_copy(uint32_t *outMode, caam_pkha_reg_area_t reg)
{
    int i = 0;

    do
    {
        reg = (caam_pkha_reg_area_t)(((uint32_t)reg) >> 1u);
        i++;
    } while (reg);

    i = 4 - i;
    /* Source register must not be E. */
    if (i != 2)
    {
        *outMode |= ((uint32_t)i << 17u);
    }
}

static void caam_pkha_mode_set_dst_reg_copy(uint32_t *outMode, caam_pkha_reg_area_t reg)
{
    int i = 0;

    do
    {
        reg = (caam_pkha_reg_area_t)(((uint32_t)reg) >> 1u);
        i++;
    } while (reg);

    i = 4 - i;
    *outMode |= ((uint32_t)i << 10u);
}

static void caam_pkha_mode_set_src_seg_copy(uint32_t *outMode, const caam_pkha_quad_area_t quad)
{
    *outMode |= ((uint32_t)quad << 8u);
}

static void caam_pkha_mode_set_dst_seg_copy(uint32_t *outMode, const caam_pkha_quad_area_t quad)
{
    *outMode |= ((uint32_t)quad << 6u);
}

static uint32_t caam_pkha_get_mode(const caam_pkha_mode_params_t *params)
{
    uint32_t modeReg;

    /* Set the PKHA algorithm and the appropriate function. */
    modeReg = (uint32_t)params->func;

    if ((params->func == kCAAM_PKHA_CopyMemSizeN) || (params->func == kCAAM_PKHA_CopyMemSizeSrc))
    {
        /* Set source and destination registers and quads. */
        caam_pkha_mode_set_src_reg_copy(&modeReg, params->srcReg);
        caam_pkha_mode_set_dst_reg_copy(&modeReg, params->dstReg);
        caam_pkha_mode_set_src_seg_copy(&modeReg, params->srcQuad);
        caam_pkha_mode_set_dst_seg_copy(&modeReg, params->dstQuad);
    }
    else
    {
        /* Set the arithmetic type - integer or binary polynomial (F2m). */
        modeReg |= ((uint32_t)params->arithType << 17u);

        /* Set to use Montgomery form of inputs and/or outputs. */
        modeReg |= ((uint32_t)params->montFormIn << 19u);
        modeReg |= ((uint32_t)params->montFormOut << 18u);

        /* Set to use pre-computed R2modN */
        modeReg |= ((uint32_t)params->r2modn << 16u);
    }

    modeReg |= ((uint32_t)params->equalTime << 10u);

    return modeReg;
}

enum _caam_user_specified_status
{
    /* the value below is used as LOCAL_OFFSET field for the JMP/HALT command, in which we test the PRM flag */
    kCAAM_UserSpecifiedStatus_NotPrime = 0x55u,

    /* the value below is returned in Job termination status word in case PrimalityTest result is NotPrime.
     */
    kCAAM_StatusNotPrime = 0x30000000u | kCAAM_UserSpecifiedStatus_NotPrime,
};

static status_t caam_pkha_algorithm_operation_command(CAAM_Type *base,
                                                      caam_handle_t *handle,
                                                      caam_desc_pkha_t descriptor,
                                                      const uint8_t *A,
                                                      size_t sizeA,
                                                      const uint8_t *B,
                                                      size_t sizeB,
                                                      const uint8_t *N,
                                                      size_t sizeN,
                                                      const uint8_t *E,
                                                      size_t sizeE,
                                                      caam_pkha_mode_params_t *params,
                                                      uint8_t *result,
                                                      size_t *resultSize)
{
    uint32_t clearMask = 0;
    uint32_t descriptorSize = ARRAY_SIZE(templateArithmeticPKHA);
    BUILD_ASSURE(sizeof(caam_desc_pkha_t) >= sizeof(templateArithmeticPKHA), caam_desc_pkha_t_size_too_low);

    /* initialize descriptor from template */
    caam_memcpy(descriptor, templateArithmeticPKHA, sizeof(templateArithmeticPKHA));

    /* add descriptor lenght in bytes to HEADER descriptor command */
    DESC_HEADER_ADD_DESCLEN(descriptor[0], descriptorSize);

    /* input data */
    if (N && sizeN)
    {
        clearMask |= 1u << 16; /* add Nram bit to PKHA_MODE_MS */
        DESC_ADD_LEN(descriptor[3], sizeN);
        DESC_SET_ADDR(descriptor[4], N);
    }
    else
    {
        /* jump to descriptor[4] */
        descriptor[3] = 0xA0000002u; /* jump to current index + 2 (=4) */
    }

    if (A && sizeA)
    {
        clearMask |= 1u << 19; /* add Aram bit to PKHA_MODE_MS */
        DESC_ADD_LEN(descriptor[5], sizeA);
        DESC_SET_ADDR(descriptor[6], A);
    }
    else
    {
        /* jump to descriptor[6] */
        descriptor[5] = 0xA0000002u; /* jump to current index + 2 (=6) */
    }

    if (B && sizeB)
    {
        clearMask |= 1u << 18; /* add Bram bit to PKHA_MODE_MS */
        DESC_ADD_LEN(descriptor[7], sizeB);
        DESC_SET_ADDR(descriptor[8], B);
    }
    else
    {
        /* jump to descriptor[8] */
        descriptor[7] = 0xA0000002u; /* jump to current index + 2 (=8) */
    }

    if (E && sizeE)
    {
        clearMask |= 1u << 17; /* add Eram bit to PKHA_MODE_MS */
        DESC_ADD_LEN(descriptor[9], sizeE);
        DESC_SET_ADDR(descriptor[10], E);
    }
    else
    {
        /* jump to descriptor[11] */
        descriptor[9] = 0xA0000002u; /* jump to current index + 2 (=11) */
    }

    /* add registers to clear into the pkha clear memory function */
    descriptor[1] |= clearMask;

    /* add functions details for pkha arithmetic functions */
    descriptor[11] |= caam_pkha_get_mode(params);

    /* RESULTS */
    if (result && resultSize)
    {
        /* We don't know the size of result at this point. But, we know it will be <= modulus. */
        DESC_ADD_LEN(descriptor[12], sizeN);
        DESC_SET_ADDR(descriptor[13], result);
        *resultSize = sizeN;
    }
    else
    {
        /* special case for Primality Test - instead of reading result, check PRM bit and return user-specified status
         * if it is set. */
        /* conditional HALT, return user-specificed status if condition evaluated is true. this condition checks if (PRM
         * is false). */
        descriptor[12] = 0xA0C12000u;
        descriptor[12] |= kCAAM_UserSpecifiedStatus_NotPrime;

        descriptor[13] = 0xA0C00000u; /* always halt with status 0x0 (normal) */
    }

    /* schedule the job */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static const uint32_t templateArithmeticECC[] = {
    /* 00 */ DESC_HEADER,  /* HEADER */
    /* 01 */ 0x818F0001u,  /* OPERATION: PKHA OPERATION: clear memory function. Clear ABNE. */
    /* 02 */ 0xA2000001u,  /* JMP always to next command. Done checkpoint (wait for Class 1 Done) */
    /* 03 */ DESC_FLOADN,  /* FIFO LOAD PKHA N */
    /* 04 */ 0x00000000u,  /* place: N address */
    /* 05 */ DESC_FLOADA0, /* FIFO LOAD A0 */
    /* 06 */ 0x00000000u,  /* place: A0 address */
    /* 07 */ DESC_FLOADA1, /* FIFO LOAD A1 */
    /* 08 */ 0x00000000u,  /* place: A1 address */
    /* 09 */ DESC_FLOADA3, /* FIFO LOAD PKHA A3 */
    /* 10 */ 0x00000000u,  /* place: A3 address */
    /* 11 */ DESC_FLOADB0, /* FIFO LOAD PKHA B0 */
    /* 12 */ 0x00000000u,  /* place: B0 address */
    /* 13 */ DESC_FLOADB1, /* FIFO LOAD PKHA B1 */
    /* 14 */ 0x00000000u,  /* place: B1 address */
    /* 15 */ DESC_FLOADB2, /* FIFO LOAD PKHA B2 */
    /* 16 */ 0x00000000u,  /* place: B2 address */
    /* 17 */ DESC_FLOADB3, /* FIFO LOAD PKHA B3 */
    /* 18 */ 0x00000000u,  /* place: B3 address */
    /* 19 */ DESC_KEY_E_,  /* KEY PKHA E */
    /* 20 */ 0x00000000u,  /* place: E address */
    /* 21 */ 0x81800000u,  /* OPERATION: PKHA OPERATION: Arithmetic Functions. */
    /* 22 */ DESC_STOREB1, /* FIFO STORE PKHA B1 */
    /* 23 */ 0x00000000u,  /* place: result X address */
    /* 24 */ DESC_STOREB2, /* FIFO STORE PKHA B2 */
    /* 25 */ 0x00000000u,  /* place: result Y address */
};

static status_t caam_pkha_ecc_algorithm_operation_command(CAAM_Type *base,
                                                          caam_handle_t *handle,
                                                          caam_desc_pkha_ecc_t descriptor,
                                                          const caam_pkha_ecc_point_t *A,
                                                          const caam_pkha_ecc_point_t *B,
                                                          const uint8_t *E,
                                                          size_t sizeE,
                                                          const uint8_t *N,
                                                          const uint8_t *R2modN_B1,
                                                          const uint8_t *R2modN_B3,
                                                          const uint8_t *aCurveParam,
                                                          const uint8_t *bCurveParam,
                                                          size_t size,
                                                          caam_pkha_ecc_point_t *result,
                                                          caam_pkha_mode_params_t *params)
{
    uint32_t descriptorSize = ARRAY_SIZE(templateArithmeticECC);
    BUILD_ASSURE(sizeof(caam_desc_pkha_ecc_t) >= sizeof(templateArithmeticECC), caam_desc_pkha_ecc_t_size_too_low);

    /* initialize descriptor from template */
    caam_memcpy(descriptor, templateArithmeticECC, sizeof(templateArithmeticECC));

    /* add descriptor lenght in bytes to HEADER descriptor command */
    DESC_HEADER_ADD_DESCLEN(descriptor[0], descriptorSize);

    /* N = modulus */
    DESC_ADD_LEN(descriptor[3], size);
    DESC_SET_ADDR(descriptor[4], N);

    /* [A0, A1] first point in affine coordinates */
    if (A)
    {
        DESC_ADD_LEN(descriptor[5], size);
        DESC_SET_ADDR(descriptor[6], A->X);
        DESC_ADD_LEN(descriptor[7], size);
        DESC_SET_ADDR(descriptor[8], A->Y);
    }
    else
    {
        /* jump to descriptor[9] */
        descriptor[5] = 0xA0000004u; /* jump to current index + 4 (=9) */
    }

    /* aCurveParam */
    DESC_ADD_LEN(descriptor[9], size);
    DESC_SET_ADDR(descriptor[10], aCurveParam);

    /* bCurveParam */
    DESC_ADD_LEN(descriptor[11], size);
    DESC_SET_ADDR(descriptor[12], bCurveParam);

    /* [B1, B2] second point in affine coordinates */
    if (B)
    {
        DESC_ADD_LEN(descriptor[13], size);
        DESC_SET_ADDR(descriptor[14], B->X);
        DESC_ADD_LEN(descriptor[15], size);
        DESC_SET_ADDR(descriptor[16], B->Y);
    }
    else if (R2modN_B1) /* R2modN for ECC_MOD_MUL goes to B1 */
    {
        DESC_ADD_LEN(descriptor[13], size);
        DESC_SET_ADDR(descriptor[14], R2modN_B1);
        /* jump to descriptor[17] */
        descriptor[15] = 0xA0000002u; /* jump to current index + 2 (=17) */
    }
    else
    {
        /* jump to descriptor[17] */
        descriptor[13] = 0xA0000004u; /* jump to current index + 4 (=17) */
    }

    if (R2modN_B3)
    {
        DESC_ADD_LEN(descriptor[17], size);
        DESC_SET_ADDR(descriptor[18], R2modN_B3);
    }
    else
    {
        /* jump to descriptor[19] */
        descriptor[17] = 0xA0000002u; /* jump to current index + 2 (=19) */
    }

    if (E && sizeE)
    {
        DESC_ADD_LEN(descriptor[19], sizeE);
        DESC_SET_ADDR(descriptor[20], E);
    }
    else
    {
        /* jump to descriptor[21] */
        descriptor[19] = 0xA0000002u; /* jump to current index + 2 (=21) */
    }

    /* add functions details for pkha arithmetic functions */
    descriptor[21] |= caam_pkha_get_mode(params);

    /* store [B1, B2] result point */
    DESC_ADD_LEN(descriptor[22], size);
    DESC_SET_ADDR(descriptor[23], result->X);
    DESC_ADD_LEN(descriptor[24], size);
    DESC_SET_ADDR(descriptor[25], result->Y);

    /* schedule the job */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

int CAAM_PKHA_CompareBigNum(const uint8_t *a, size_t sizeA, const uint8_t *b, size_t sizeB)
{
    int retval = 0;

    /* skip zero msbytes - integer a */
    while ((sizeA) && (0u == a[0]))
    {
        sizeA--;
        a++;
    }

    /* skip zero msbytes - integer b */
    while ((sizeB) && (0u == b[0]))
    {
        sizeB--;
        b++;
    }

    if (sizeA > sizeB)
    {
        retval = 1;
    } /* int a has more non-zero bytes, thus it is bigger than b */
    else if (sizeA < sizeB)
    {
        retval = -1;
    } /* int b has more non-zero bytes, thus it is bigger than a */
    else if (sizeA == 0)
    {
        retval = 0;
    } /* sizeA = sizeB = 0 */
    else
    {
        int n;
        uint32_t equal;
        int val;

        n = sizeA - 1;
        equal = 0;

        /* compare all bytes - does not leak (in time domain) how many bytes equal */
        /* move from lsbyte to msbyte */
        while (n >= 0)
        {
            uint32_t chXor = (a[n] ^ b[n]);

            equal |= chXor;
            val = (int)chXor * (a[n] - b[n]);

            if (val < 0)
            {
                retval = -1;
            }

            if (val > 0)
            {
                retval = 1;
            }

            if (val == 0)
            {
                val = 1;
            }

            if (val)
            {
                n--;
            }
        }

        if (0 == equal)
        {
            retval = 0;
        }
    }
    return (retval);
}

status_t CAAM_PKHA_ModAddNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *B,
                                     size_t sizeB,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType)
{
    caam_pkha_mode_params_t params;
    status_t status;

    if (arithType == kCAAM_PKHA_IntegerArith)
    {
        if (CAAM_PKHA_CompareBigNum(A, sizeA, N, sizeN) >= 0)
        {
            return (kStatus_InvalidArgument);
        }

        if (CAAM_PKHA_CompareBigNum(B, sizeB, N, sizeN) >= 0)
        {
            return (kStatus_InvalidArgument);
        }
    }

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModAdd;
    params.arithType = arithType;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, B, sizeB, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModAdd(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *B,
                          size_t sizeB,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModAddNonBlocking(base, handle, descBuf, A, sizeA, B, sizeB, N, sizeN, result, resultSize,
                                             arithType);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModSub1NonBlocking(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      caam_desc_pkha_t descriptor,
                                      const uint8_t *A,
                                      size_t sizeA,
                                      const uint8_t *B,
                                      size_t sizeB,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *result,
                                      size_t *resultSize)
{
    caam_pkha_mode_params_t params;
    status_t status;

    if (CAAM_PKHA_CompareBigNum(A, sizeA, N, sizeN) >= 0)
    {
        return (kStatus_InvalidArgument);
    }

    if (CAAM_PKHA_CompareBigNum(B, sizeB, N, sizeN) >= 0)
    {
        return (kStatus_InvalidArgument);
    }

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModSub1;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, B, sizeB, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModSub1(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *A,
                           size_t sizeA,
                           const uint8_t *B,
                           size_t sizeB,
                           const uint8_t *N,
                           size_t sizeN,
                           uint8_t *result,
                           size_t *resultSize)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModSub1NonBlocking(base, handle, descBuf, A, sizeA, B, sizeB, N, sizeN, result, resultSize);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModSub2NonBlocking(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      caam_desc_pkha_t descriptor,
                                      const uint8_t *A,
                                      size_t sizeA,
                                      const uint8_t *B,
                                      size_t sizeB,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *result,
                                      size_t *resultSize)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModSub2;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, B, sizeB, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModSub2(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *A,
                           size_t sizeA,
                           const uint8_t *B,
                           size_t sizeB,
                           const uint8_t *N,
                           size_t sizeN,
                           uint8_t *result,
                           size_t *resultSize)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModSub2NonBlocking(base, handle, descBuf, A, sizeA, B, sizeB, N, sizeN, result, resultSize);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModMulNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *B,
                                     size_t sizeB,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType,
                                     caam_pkha_montgomery_form_t montIn,
                                     caam_pkha_montgomery_form_t montOut,
                                     caam_pkha_timing_t equalTime)
{
    caam_pkha_mode_params_t params;
    status_t status;

    if (arithType == kCAAM_PKHA_IntegerArith)
    {
        if (CAAM_PKHA_CompareBigNum(A, sizeA, N, sizeN) >= 0)
        {
            return (kStatus_InvalidArgument);
        }

        if (CAAM_PKHA_CompareBigNum(B, sizeB, N, sizeN) >= 0)
        {
            return (kStatus_InvalidArgument);
        }
    }

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModMul;
    params.arithType = arithType;
    params.montFormIn = montIn;
    params.montFormOut = montOut;
    params.equalTime = equalTime;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, B, sizeB, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);

    return status;
}

status_t CAAM_PKHA_ModMul(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *B,
                          size_t sizeB,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType,
                          caam_pkha_montgomery_form_t montIn,
                          caam_pkha_montgomery_form_t montOut,
                          caam_pkha_timing_t equalTime)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModMulNonBlocking(base, handle, descBuf, A, sizeA, B, sizeB, N, sizeN, result, resultSize,
                                             arithType, montIn, montOut, equalTime);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModR2NonBlocking(CAAM_Type *base,
                                    caam_handle_t *handle,
                                    caam_desc_pkha_t descriptor,
                                    const uint8_t *N,
                                    size_t sizeN,
                                    uint8_t *result,
                                    size_t *resultSize,
                                    caam_pkha_f2m_t arithType)
{
    status_t status;
    caam_pkha_mode_params_t params;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModR2;
    params.arithType = arithType;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, NULL, 0, NULL, 0, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModR2(CAAM_Type *base,
                         caam_handle_t *handle,
                         const uint8_t *N,
                         size_t sizeN,
                         uint8_t *result,
                         size_t *resultSize,
                         caam_pkha_f2m_t arithType)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModR2NonBlocking(base, handle, descBuf, N, sizeN, result, resultSize, arithType);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModExpNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     const uint8_t *E,
                                     size_t sizeE,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType,
                                     caam_pkha_montgomery_form_t montIn,
                                     caam_pkha_timing_t equalTime)
{
    caam_pkha_mode_params_t params;
    status_t status;

    if (arithType == kCAAM_PKHA_IntegerArith)
    {
        if (CAAM_PKHA_CompareBigNum(A, sizeA, N, sizeN) >= 0)
        {
            return (kStatus_InvalidArgument);
        }
    }

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModExp;
    params.arithType = arithType;
    params.montFormIn = montIn;
    params.equalTime = equalTime;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, NULL, 0, N, sizeN, E, sizeE,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModExp(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          const uint8_t *E,
                          size_t sizeE,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType,
                          caam_pkha_montgomery_form_t montIn,
                          caam_pkha_timing_t equalTime)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModExpNonBlocking(base, handle, descBuf, A, sizeA, N, sizeN, E, sizeE, result, resultSize,
                                             arithType, montIn, equalTime);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModRedNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModRed;
    params.arithType = arithType;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, NULL, 0, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModRed(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModRedNonBlocking(base, handle, descBuf, A, sizeA, N, sizeN, result, resultSize, arithType);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModInvNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithModInv;
    params.arithType = arithType;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, NULL, 0, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModInv(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModInvNonBlocking(base, handle, descBuf, A, sizeA, N, sizeN, result, resultSize, arithType);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ModGcdNonBlocking(CAAM_Type *base,
                                     caam_handle_t *handle,
                                     caam_desc_pkha_t descriptor,
                                     const uint8_t *A,
                                     size_t sizeA,
                                     const uint8_t *N,
                                     size_t sizeN,
                                     uint8_t *result,
                                     size_t *resultSize,
                                     caam_pkha_f2m_t arithType)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithGcd;
    params.arithType = arithType;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, NULL, 0, N, sizeN, NULL, 0,
                                                   &params, result, resultSize);
    return status;
}

status_t CAAM_PKHA_ModGcd(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ModGcdNonBlocking(base, handle, descBuf, A, sizeA, N, sizeN, result, resultSize, arithType);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_PrimalityTestNonBlocking(CAAM_Type *base,
                                            caam_handle_t *handle,
                                            caam_desc_pkha_t descriptor,
                                            const uint8_t *A,
                                            size_t sizeA,
                                            const uint8_t *B,
                                            size_t sizeB,
                                            const uint8_t *N,
                                            size_t sizeN)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithPrimalityTest;

    status = caam_pkha_algorithm_operation_command(base, handle, descriptor, A, sizeA, B, sizeB, N, sizeN, NULL, 0,
                                                   &params, NULL, NULL);
    return status;
}

status_t CAAM_PKHA_PrimalityTest(CAAM_Type *base,
                                 caam_handle_t *handle,
                                 const uint8_t *A,
                                 size_t sizeA,
                                 const uint8_t *B,
                                 size_t sizeB,
                                 const uint8_t *N,
                                 size_t sizeN,
                                 bool *res)
{
    caam_desc_pkha_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_PrimalityTestNonBlocking(base, handle, descBuf, A, sizeA, B, sizeB, N, sizeN);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);

    if (status == kStatus_Success)
    {
        /* this return code means that the candidate is believed to be prime. */
        *res = true;
    }
    /* clear DESC INDEX field in the Job termination status word and check if it is our NotPrime user specified status
     */
    else if (((uint32_t)status & 0xffff00ffu) == kCAAM_StatusNotPrime)
    {
        /* change status to Ok to upper layer caller. this return code means that the candidate is believed to not being
         * prime. */
        status = kStatus_Success;
        *res = false;
    }
    else
    {
        *res = false;
    }

    return status;
}

status_t CAAM_PKHA_ECC_PointAddNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_pkha_ecc_t descriptor,
                                           const caam_pkha_ecc_point_t *A,
                                           const caam_pkha_ecc_point_t *B,
                                           const uint8_t *N,
                                           const uint8_t *R2modN,
                                           const uint8_t *aCurveParam,
                                           const uint8_t *bCurveParam,
                                           size_t size,
                                           caam_pkha_f2m_t arithType,
                                           caam_pkha_ecc_point_t *result)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithEccAdd;
    params.arithType = arithType;
    params.r2modn = R2modN ? kCAAM_PKHA_InputR2 : kCAAM_PKHA_CalcR2;

    status = caam_pkha_ecc_algorithm_operation_command(base, handle, descriptor, A, B, NULL, 0, N, NULL, R2modN,
                                                       aCurveParam, bCurveParam, size, result, &params);
    return status;
}

status_t CAAM_PKHA_ECC_PointAdd(CAAM_Type *base,
                                caam_handle_t *handle,
                                const caam_pkha_ecc_point_t *A,
                                const caam_pkha_ecc_point_t *B,
                                const uint8_t *N,
                                const uint8_t *R2modN,
                                const uint8_t *aCurveParam,
                                const uint8_t *bCurveParam,
                                size_t size,
                                caam_pkha_f2m_t arithType,
                                caam_pkha_ecc_point_t *result)
{
    caam_desc_pkha_ecc_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ECC_PointAddNonBlocking(base, handle, descBuf, A, B, N, R2modN, aCurveParam, bCurveParam,
                                                   size, arithType, result);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ECC_PointDoubleNonBlocking(CAAM_Type *base,
                                              caam_handle_t *handle,
                                              caam_desc_pkha_ecc_t descriptor,
                                              const caam_pkha_ecc_point_t *B,
                                              const uint8_t *N,
                                              const uint8_t *aCurveParam,
                                              const uint8_t *bCurveParam,
                                              size_t size,
                                              caam_pkha_f2m_t arithType,
                                              caam_pkha_ecc_point_t *result)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithEccDouble;
    params.arithType = arithType;

    status = caam_pkha_ecc_algorithm_operation_command(base, handle, descriptor, NULL, B, NULL, 0, N, NULL, NULL,
                                                       aCurveParam, bCurveParam, size, result, &params);
    return status;
}

status_t CAAM_PKHA_ECC_PointDouble(CAAM_Type *base,
                                   caam_handle_t *handle,
                                   const caam_pkha_ecc_point_t *B,
                                   const uint8_t *N,
                                   const uint8_t *aCurveParam,
                                   const uint8_t *bCurveParam,
                                   size_t size,
                                   caam_pkha_f2m_t arithType,
                                   caam_pkha_ecc_point_t *result)
{
    caam_desc_pkha_ecc_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ECC_PointDoubleNonBlocking(base, handle, descBuf, B, N, aCurveParam, bCurveParam, size,
                                                      arithType, result);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_ECC_PointMulNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_pkha_ecc_t descriptor,
                                           const caam_pkha_ecc_point_t *A,
                                           const uint8_t *E,
                                           size_t sizeE,
                                           const uint8_t *N,
                                           const uint8_t *R2modN,
                                           const uint8_t *aCurveParam,
                                           const uint8_t *bCurveParam,
                                           size_t size,
                                           caam_pkha_timing_t equalTime,
                                           caam_pkha_f2m_t arithType,
                                           caam_pkha_ecc_point_t *result)
{
    caam_pkha_mode_params_t params;
    status_t status;

    caam_pkha_default_parms(&params);
    params.func = kCAAM_PKHA_ArithEccMul;
    params.equalTime = equalTime;
    params.arithType = arithType;
    params.r2modn = R2modN ? kCAAM_PKHA_InputR2 : kCAAM_PKHA_CalcR2;

    status = caam_pkha_ecc_algorithm_operation_command(base, handle, descriptor, A, NULL, E, sizeE, N, R2modN, NULL,
                                                       aCurveParam, bCurveParam, size, result, &params);
    return status;
}

status_t CAAM_PKHA_ECC_PointMul(CAAM_Type *base,
                                caam_handle_t *handle,
                                const caam_pkha_ecc_point_t *A,
                                const uint8_t *E,
                                size_t sizeE,
                                const uint8_t *N,
                                const uint8_t *R2modN,
                                const uint8_t *aCurveParam,
                                const uint8_t *bCurveParam,
                                size_t size,
                                caam_pkha_timing_t equalTime,
                                caam_pkha_f2m_t arithType,
                                caam_pkha_ecc_point_t *result)
{
    caam_desc_pkha_ecc_t descBuf;
    status_t status;

    do
    {
        status = CAAM_PKHA_ECC_PointMulNonBlocking(base, handle, descBuf, A, E, sizeE, N, R2modN, aCurveParam,
                                                   bCurveParam, size, equalTime, arithType, result);
    } while (status == kStatus_CAAM_Again);

    if (status != kStatus_Success)
    {
        return status;
    }

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_PKHA_NormalToMontgomery(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *A,
                                      size_t *sizeA,
                                      uint8_t *B,
                                      size_t *sizeB,
                                      uint8_t *R2,
                                      size_t *sizeR2,
                                      caam_pkha_timing_t equalTime,
                                      caam_pkha_f2m_t arithType)
{
    status_t status;

    /* need to convert our Integer inputs into Montgomery format */
    if (N && sizeN && R2 && sizeR2)
    {
        /* 1. R2 = MOD_R2(N) */
        status = CAAM_PKHA_ModR2(base, handle, N, sizeN, R2, sizeR2, arithType);
        if (status != kStatus_Success)
        {
            return status;
        }

        /* 2. A(Montgomery) = MOD_MUL_IM_OM(A, R2, N) */
        if (A && sizeA)
        {
            status = CAAM_PKHA_ModMul(base, handle, A, *sizeA, R2, *sizeR2, N, sizeN, A, sizeA, arithType,
                                      kCAAM_PKHA_MontgomeryFormat, kCAAM_PKHA_MontgomeryFormat, equalTime);
            if (status != kStatus_Success)
            {
                return status;
            }
        }

        /* 2. B(Montgomery) = MOD_MUL_IM_OM(B, R2, N) */
        if (B && sizeB)
        {
            status = CAAM_PKHA_ModMul(base, handle, B, *sizeB, R2, *sizeR2, N, sizeN, B, sizeB, arithType,
                                      kCAAM_PKHA_MontgomeryFormat, kCAAM_PKHA_MontgomeryFormat, equalTime);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }
    else
    {
        status = kStatus_InvalidArgument;
    }

    return status;
}

status_t CAAM_PKHA_MontgomeryToNormal(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *A,
                                      size_t *sizeA,
                                      uint8_t *B,
                                      size_t *sizeB,
                                      caam_pkha_timing_t equalTime,
                                      caam_pkha_f2m_t arithType)
{
    uint8_t one = 1;
    status_t status = kStatus_InvalidArgument;

    /* A = MOD_MUL_IM_OM(A(Montgomery), 1, N) */
    if (A && sizeA)
    {
        status = CAAM_PKHA_ModMul(base, handle, A, *sizeA, &one, sizeof(one), N, sizeN, A, sizeA, arithType,
                                  kCAAM_PKHA_MontgomeryFormat, kCAAM_PKHA_MontgomeryFormat, equalTime);
        if (kStatus_Success != status)
        {
            return status;
        }
    }

    /* B = MOD_MUL_IM_OM(B(Montgomery), 1, N) */
    if (B && sizeB)
    {
        status = CAAM_PKHA_ModMul(base, handle, B, *sizeB, &one, sizeof(one), N, sizeN, B, sizeB, arithType,
                                  kCAAM_PKHA_MontgomeryFormat, kCAAM_PKHA_MontgomeryFormat, equalTime);
        if (kStatus_Success != status)
        {
            return status;
        }
    }
    return status;
}
