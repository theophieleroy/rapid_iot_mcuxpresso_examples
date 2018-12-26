/*
 * The Clear BSD License
 * Copyright 2018 NXP
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

#ifndef _FSL_CASPER_H_
#define _FSL_CASPER_H_

#include "fsl_common.h"

/*!
 * @addtogroup casper
 * @{
 */

/*! @file */


/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*!
 * @addtogroup casper_driver
 * @{
 */
/*! @name Driver version */
/*@{*/
/*! @brief CASPER driver version. Version 2.0.0.
 *
 * Current version: 2.0.0
 *
 * Change log:
 * - Version 2.0.0
 *   - Initial version
 */
#define FSL_CASPER_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief CASPER operation
 *
 */
typedef enum _casper_operation
{
    kCASPER_OpMul6464NoSum = 0x01, /*! Walking 1 or more of J loop, doing r=a*b using 64x64=128*/
    kCASPER_OpMul6464Sum =
        0x02, /*! Walking 1 or more of J loop, doing c,r=r+a*b using 64x64=128, but assume inner j loop*/
    kCASPER_OpMul6464FullSum =
        0x03, /*! Walking 1 or more of J loop, doing c,r=r+a*b using 64x64=128, but sum all of w. */
    kCASPER_OpMul6464Reduce =
        0x04,               /*! Walking 1 or more of J loop, doing c,r[-1]=r+a*b using 64x64=128, but skip 1st write*/
    kCASPER_OpAdd64 = 0x08, /*! Walking add with off_AB, and in/out off_RES doing c,r=r+a+c using 64+64=65*/
    kCASPER_OpSub64 = 0x09, /*! Walking subtract with off_AB, and in/out off_RES doing r=r-a uding 64-64=64, with last
                               borrow implicit if any*/
    kCASPER_OpDouble64 = 0x0A, /*! Walking add to self with off_RES doing c,r=r+r+c using 64+64=65*/
    kCASPER_OpXor64 = 0x0B,    /*! Walking XOR with off_AB, and in/out off_RES doing r=r^a using 64^64=64*/
    kCASPER_OpShiftLeft32 =
        0x10, /*! Walking shift left doing r1,r=(b*D)|r1, where D is 2^amt and is loaded by app (off_CD not used)*/
    kCASPER_OpShiftRight32 = 0x11, /*! Walking shift right doing r,r1=(b*D)|r1, where D is 2^(32-amt) and is loaded by
                                      app (off_CD not used) and off_RES starts at MSW*/
    kCASPER_OpCopy = 0x14,         /*! Copy from ABoff to resoff, 64b at a time*/
    kCASPER_OpRemask = 0x15,       /*! Copy and mask from ABoff to resoff, 64b at a time*/
    kCASPER_OpCompare = 0x16,      /*! Compare two arrays, running all the way to the end*/
    kCASPER_OpCompareFast = 0x17,  /*! Compare two arrays, stopping on 1st !=*/
} casper_operation_t;

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup casper_driver
 * @{
 */

/*!
 * @brief Enables clock and disables reset for CASPER peripheral.
 *
 * Enable clock and disable reset for CASPER.
 *
 * @param base CASPER base address
 */
void CASPER_Init(CASPER_Type *base);

/*!
 * @brief Disables clock for CASPER peripheral.
 *
 * Disable clock and enable reset.
 *
 * @param base CASPER base address
 */
void CASPER_Deinit(CASPER_Type *base);

/*!
 *@}
 */ /* end of casper_driver */

/*******************************************************************************
 * PKHA API
 ******************************************************************************/

/*!
 * @addtogroup casper_driver_pkha
 * @{
 */

/*!
 * @brief Performs modular exponentiation - (A^E) mod N.
 *
 * This function performs modular exponentiation.
 *
 * @param base CASPER base address
 * @param signature first addend (in little endian format)
 * @param pubN modulus (in little endian format)
 * @param wordLen Size of pubN in bytes
 * @param pubE exponent
 * @param[out] plaintext Output array to store result of operation (in little endian format)
 */
int CASPER_ModExp(CASPER_Type *base,
                  const uint8_t *signature,
                  const uint8_t *pubN,
                  size_t wordLen,
                  uint32_t pubE,
                  uint8_t *plaintext);

/*!
 *@}
 */ /* end of casper_driver_pkha */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_CASPER_H_ */
