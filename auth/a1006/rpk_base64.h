/*
 * Copyright 2018 NXP
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * NXP authentication products.  This software is supplied "AS IS" without any
 * warranties of any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontroller and/or
 * authentication products.  This copyright, permission, and disclaimer notice
 * must appear in all copies of this code.
 */

#ifndef _RPK_BASE64_H_
#define _RPK_BASE64_H_

/*!
 * @addtogroup rpk_base64 Base64 String Encoder/Decoder
 *
 * The rpk_base64 module provides an efficient method of encoding and decoding Base64 strings.
 *
 * Usage
 * -------------------------------------------------------------------------------------------------
 *
 * Encoding:
 * @code
 *
 *      const char msg = "Hello world";
 *
 *      uint32_t encLength = RPK_Base64_Get_Encoded_Len(strlen(msg));
 *
 *      uint8_t encBuffer = (uint8_t*)malloc(encLength);
 *
 *      uint32_t ret = RPK_Base64_Encode((uint8_t*)msg, strlen(msg), encBuffer, encLength);
 *
 *      if (encLength != ret)
 *      {
 *          // Something went wrong
 *      }
 *
 * @endcode
 *
 * Decoding:
 * @code
 *
 *      const char* msg = "SGVsbG8gd29ybGQ=";
 *
 *      uint32_t decLength = RPK_Base64_Get_Decoded_Len((uint8_t*)msg, strlen(msg));
 *
 *      uint8_t decBuffer = (uint8_t*)malloc(decLength);
 *
 *      uint32_t ret = RPK_Base64_Decode((uint8_t*)msg, strlen(msg), decBuffer, decLength);
 *
 *      if (decLength != ret)
 *      {
 *          // Something went wrong
 *      }
 *
 * @endcode
 *
 * Memory Considerations
 * -------------------------------------------------------------------------------------------------
 * It is the responsibility of the developer to allocate the necessary memory. See 'Usage' section
 * above for methods on obtaining accurate buffer lengths needed.
 *
 * @{
 * @brief Base64 Encoding/Decoding
 */

/*! @file */

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Get Encoded Length - return buffer length needed to store encoded version of data
 *
 * @note Call before dynamic allocation of memory for storing base64 encoded buffer
 *
 * @param len Length of raw data buffer in bytes
 *
 * @return Length in bytes of base64 encoded version of given data buffer
 */
uint32_t RPK_Base64_Get_Encoded_Len(uint32_t len);

/*!
 * @brief Get Decoded Length - return buffer length needed to store decoded version of data
 *
 * @note Call before dynamic allocation of memory for storing decoded buffer from base64 data
 *
 *  @param buf Pointer to encoded data buffer
 *  @param len Length of encoded data buffer in bytes
 *
 * @return Length in bytes of decoded version of given base64 data buffer; 0 if NULL pointer detected
 */
uint32_t RPK_Base64_Get_Decoded_Len(uint8_t *buf, uint32_t len);

/*!
 * @brief Encode data with base64 encoding
 *
 * @note RPK_Base64_Get_Encoded_Len should be called before hand to set valid numOutBytes
 *
 *  @param in          Pointer to raw data to encode
 *  @param numInBytes  Length of raw data to encode
 *  @param out         Pointer to encoded data that will be output
 *  @param numOutBytes Length of resulting encoded data
 *
 * @return Length in bytes of encoded data; 0 if NULL pointer detected
 */
uint32_t RPK_Base64_Encode(uint8_t *in, uint32_t numInBytes, uint8_t *out, uint32_t numOutBytes);

/*!
 * @brief Decode base64 into raw binary data
 *
 * @note RPK_Base64_Get_Decoded_Len should be called before hand to set valid numOutBytes
 *
 *  @param in          Pointer to encoded data
 *  @param numInBytes  Length of encoded data
 *  @param out         Pointer to raw data that will be output
 *  @param numOutBytes Length of resulting raw binary data
 *
 * @return Length in bytes of decoded data; 0 if NULL pointer detected
 */
uint32_t RPK_Base64_Decode(uint8_t *in, uint32_t numInBytes, uint8_t *out, uint32_t numOutBytes);

/*! @}*/

#ifdef __cplusplus
}
#endif

/*! @}*/


#endif // _RPK_BASE64_H_
