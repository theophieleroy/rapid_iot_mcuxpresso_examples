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
#include "a100x_interface.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define A1006_I2C_GENERAL_CALL  0x00
#define A1006_I2C_SLAVE_ADDR    0xA0 /* 8-bit slave address of A1006 */

typedef void* a1006_instance_t;

static a1006_instance_t sInstance = NULL;

static a1006_IoFunc_t sIoFunc;

typedef enum _a1006_cmd
{
    kTemp
} a1006_cmd_t;

/*! @brief Wake-up values for A1006 powerDown mode. */
typedef enum
{
    a1006_wake_on_i2c = 0x01,
} A1006_powerDown_parameter;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void buffer_copy(void *dst, void *src, size_t bytes)
{
    uint8_t *d, *s, *end;
    d = (uint8_t *)dst;
    s = (uint8_t *)src;
    end = d + bytes;

    while(d != end)
    {
        *d++ = *s++;
    }
}
/*****************************************************************************
 * Public functions
 ****************************************************************************/

uint8_t A1006_Init_Driver(a1006_IoFunc_t* pIoFunc)
{
    if (NULL == sInstance)
    {
        sIoFunc = *pIoFunc;
        sInstance = (a1006_instance_t)&sIoFunc;
        return kInitSuccess;
    }
    else
    {
        return kAlreadyInitialized;
    }
}

uint8_t A1006_Deinit_Driver(void)
{
    if (NULL != sInstance)
    {
        sIoFunc.I2C_Read    = NULL;
        sIoFunc.I2C_Write   = NULL;
        sIoFunc.WaitMsec    = NULL;
        sIoFunc.i2c_Address = 0;
        sIoFunc.cert_slot = kCertificateInvalid;
        sInstance = NULL;
        return kDeinitSuccess;
    }
    else
    {
        return kNotInitialized;
    }
}

uint8_t A1006_Soft_Reset(void)
{
    uint8_t c = 0x06;

    uint8_t ret = kTransferFail;

    ret = sIoFunc.I2C_Write(A1006_I2C_GENERAL_CALL, &c, 1);

    if (kTransferOkay == ret)
    {
        sIoFunc.WaitMsec(2);
    }

    return ret;
}

uint8_t A1006_Power_Down(void)
{
    uint8_t sbuf[4];
    sbuf[0] = 0x09;
    sbuf[1] = 0x01;
    sbuf[2] = 0x01;
    sbuf[3] = 0x01;

    uint8_t ret = kTransferFail;

    ret = sIoFunc.I2C_Write(sIoFunc.i2c_Address, sbuf, 4);

    if (kTransferOkay == ret)
    {
        sIoFunc.WaitMsec(20);
    }

    return ret;
}

uint8_t A1006_Wake_Up(void)
{
    uint8_t sbuf[3];
    sbuf[0] = 0x09;
    sbuf[1] = 0x02;
    sbuf[2] = 0x00;

    uint8_t ret = kTransferFail;

    ret = sIoFunc.I2C_Write(sIoFunc.i2c_Address, sbuf, 3);

    if (kTransferOkay == ret)
    {
        sIoFunc.WaitMsec(5);
    }

    return ret;
}

uint8_t A1006_Get_Status(uint32_t * a1006_status)
{
    uint8_t rbuf[2];

    uint8_t ret = kTransferFail;
    uint8_t cmd[2] = { 0x09, 0x00 };
    ret = sIoFunc.I2C_Read(sIoFunc.i2c_Address, cmd, 2, rbuf, 2);

    if (rbuf[0] != 0x01)
    {
        ret = kTransferFail;
    }

    if (kTransferOkay == ret)
    {
        *a1006_status = (uint32_t)rbuf[1];
    }

    return ret;
}

uint8_t A1006_Get_Uid(uint8_t *uid)
{
    uint8_t rcvbuf[18];
    uint32_t numbytes = 18;

    uint8_t ret = kTransferFail;

    // Configure command for appropriate certificate slot
    uint8_t cmd[2] = {0};
    if (kCertificateOne == sIoFunc.cert_slot)
    {
        cmd[0] = 0x03;
        cmd[1] = 0x00;
    }
    else if (kCertificateTwo == sIoFunc.cert_slot)
    {
        cmd[0] = 0x03;
        cmd[1] = 0x10;
    }
    else
    {
        return kInvalidInput;
    }

    ret = sIoFunc.I2C_Read(sIoFunc.i2c_Address, cmd, 2, rcvbuf, numbytes);

    if (kTransferOkay == ret)
    {
        // Check for LEN error
        if (rcvbuf[0])
        {
            ret = rcvbuf[0];
        }
    }

    if (kTransferOkay == ret)
    {
        // Check for PCB error
        if (rcvbuf[1])
        {
            ret = rcvbuf[1];
        }
    }

    if (kTransferOkay == ret)
    {
        buffer_copy(uid, &rcvbuf[2], 16);
    }

    return ret;
}

uint8_t A1006_Get_Cert(uint8_t *cert)
{
    uint8_t rcvbuf[130];
    uint32_t numbytes = 130;

    uint8_t ret = kTransferFail;

    // Configure command for appropriate certificate slot
    uint8_t cmd[2] = {0};
    if (kCertificateOne == sIoFunc.cert_slot)
    {
        cmd[0] = 0x01;
        cmd[1] = 0x00;
    }
    else if (kCertificateTwo == sIoFunc.cert_slot)
    {
        cmd[0] = 0x02;
        cmd[1] = 0x00;
    }
    else
    {
        return kInvalidInput;
    }

    ret = sIoFunc.I2C_Read(sIoFunc.i2c_Address, cmd, 2, rcvbuf, numbytes);

    if (kTransferOkay == ret)
    {
        // Check for LEN error
        if (rcvbuf[0])
        {
            ret = rcvbuf[0];
        }
    }

    if (kTransferOkay == ret)
    {
        // Check for PCB error
        if (rcvbuf[1])
        {
            ret = rcvbuf[1];
        }
    }

    if (kTransferOkay == ret)
    {
        buffer_copy(cert, &rcvbuf[2], 128);
    }

    return ret;
}

uint8_t A1006_Set_Challenge(uint8_t *challenge)
{
    uint8_t ret = kTransferFail;

    uint8_t sbuf[47] = {0};
    sbuf[0] = 0x08;
    sbuf[1] = 0x00;
    sbuf[2] = 0x2C;

    buffer_copy(&sbuf[3], challenge, 44);

    ret = sIoFunc.I2C_Write(sIoFunc.i2c_Address, sbuf, 47);

    return ret;
}

uint8_t A1006_Get_Response(uint8_t *response)
{
    uint8_t ret = kTransferFail;

    uint8_t rbuf[46] = {0};
    uint8_t cmd[2] = { 0x08, 0x01 };
    ret = sIoFunc.I2C_Read(sIoFunc.i2c_Address, cmd, 2, rbuf, 46);

    if (kTransferOkay == ret)
    {
        if (0x01 == rbuf[0])
        {
            ret = 0x80 | rbuf[1];
        }
    }

    if (kTransferOkay == ret)
    {
        if (0x2d == rbuf[0])
        {
            buffer_copy(response, &rbuf[2], 44);
        }
    }

    return ret;
}
