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

#include <stdint.h>
#include <stddef.h>
#include "rpk_base64.h"

const uint8_t pad = '=';

const uint8_t *encoderRing = (const uint8_t *)"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const uint8_t *decoderRing = (const uint8_t *)"...........................................\x3E...\x3F\x34\x35\x36\x37\x38\x39\x3A\x3B\x3C\x3D.......\x0\x1\x2\x3\x4\x5\x6\x7\x8\x9\xA\xB\xC\xD\xE\xF\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19......\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2A\x2B\x2C\x2D\x2E\x2F\x30\x31\x32\x33";

uint32_t RPK_Base64_Get_Encoded_Len(uint32_t len)
{
    uint32_t mod = len % 3;
    if (0 == mod)
    {
        return (4 * (len / 3));
    }
    else
    {
        return ((4 * ((len - mod) / 3)) + 4);
    }
}

uint32_t RPK_Base64_Get_Decoded_Len(uint8_t *buf, uint32_t len)
{
    if (NULL != buf)
    {
        if ('=' == buf[len - 1])
        {
            if ('=' == buf[len - 2])
            {
                return ((3 * ((len - 4) / 4)) + 1);
            }
            else
            {
                return ((3 * ((len - 4) / 4)) + 2);
            }
        }
        else
        {
            return (3 * (len / 4));
        }
    }
    else
    {
        return 0;
    }
}

uint32_t RPK_Base64_Encode(uint8_t *in, uint32_t numInBytes, uint8_t *out, uint32_t numOutBytes)
{
    if ((NULL != in) && (NULL != out))
    {
        uint32_t extra = (numInBytes % 3);
        uint32_t count = numInBytes - extra;
        uint32_t idx = 0;
        uint32_t odx = 0;
        uint32_t capture = 0;

        while(idx < count)
        {
            capture |= (in[idx++] << 16);
            capture |= (in[idx++] << 8);
            capture |= in[idx++];

            out[odx++] = encoderRing[(capture >> 18) & 0x3F];
            out[odx++] = encoderRing[(capture >> 12) & 0x3F];
            out[odx++] = encoderRing[(capture >> 6) & 0x3F];
            out[odx++] = encoderRing[capture & 0x3F];

            capture = 0;
        }

        if (0 < extra)
        {
            if (1 == extra)
            {
                capture |= (in[idx++] << 16);

                out[odx++] = encoderRing[(capture >> 18) & 0x3F];
                out[odx++] = encoderRing[(capture >> 12) & 0x3F];
                out[odx++] = '=';
                out[odx++] = '=';
            }
            else
            {
                capture |= (in[idx++] << 16);
                capture |= (in[idx++] << 8);

                out[odx++] = encoderRing[(capture >> 18) & 0x3F];
                out[odx++] = encoderRing[(capture >> 12) & 0x3F];
                out[odx++] = encoderRing[(capture >> 6) & 0x3F];
                out[odx++] = '=';
            }
        }

        return odx;
    }
    else
    {
        return 0;
    }
}

uint32_t RPK_Base64_Decode(uint8_t *in, uint32_t numInBytes, uint8_t *out, uint32_t numOutBytes)
{
    if ((NULL != in) && (NULL != out))
    {
        uint32_t padding = 0;
        uint32_t count = numInBytes;
        uint32_t idx = 0;
        uint32_t odx = 0;
        uint32_t capture = 0;

        if ('=' == in[numInBytes - 1])
        {
            if ('=' == in[numInBytes - 2])
            {
                padding = 2;
            }
            else
            {
                padding = 1;
            }
        }

        count -= padding;

        while (idx < count)
        {
            capture |= (decoderRing[in[idx++]] << 18);
            capture |= (decoderRing[in[idx++]] << 12);
            capture |= (decoderRing[in[idx++]] << 6);
            capture |= decoderRing[in[idx++]];

            out[odx++] = ((capture >> 16) & 0xFF);
            out[odx++] = ((capture >> 8) & 0xFF);
            out[odx++] = (capture & 0xFF);

            capture = 0;
        }

        if (0 < padding)
        {
            if (1 == padding)
            {
                capture |= ((decoderRing[in[idx++]] << 18) & 0x3F);
                capture |= ((decoderRing[in[idx++]] << 12) & 0x3F);
                capture |= ((decoderRing[in[idx++]] << 6) & 0x3F);

                out[odx++] = ((capture >> 16) & 0xFF);
                out[odx++] = ((capture >> 8) & 0xFF);
            }
            else
            {
                capture |= ((decoderRing[in[idx++]] << 18) & 0x3F);
                capture |= ((decoderRing[in[idx++]] << 12) & 0x3F);

                out[odx++] = ((capture >> 16) & 0xFF);
            }
        }

        return odx;
    }
    else
    {
        return 0;
    }
}
