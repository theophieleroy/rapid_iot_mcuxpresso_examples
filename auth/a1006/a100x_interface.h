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

#ifndef _A100X_INTERFACE_H_
#define _A100X_INTERFACE_H_

/*!
 * @addtogroup a100x_interface
 *
 * The a100x_interface module provides communication with the A1006 secure authenticator via I2C.
 *
 * This API facilitates authentication between the Client device w/A1006 on board and a remote Host.
 *
 * Usage
 * -------------------------------------------------------------------------------------------------
 * 
 * Initialization:
 * @code
 *
 *     a1006_IoFunc_t A1006_auth;
 *     A1006_auth.I2C_Read = App_I2C2_Read;
 *     A1006_auth.I2C_Write = App_I2C2_Write;
 *     A1006_auth.WaitMsec = App_WaitMsec;
 *     A1006_auth.i2c_Address = A1006_I2C_SLAVE_ADDRESS;
 *     A1006_auth.cert_slot = kCertificateOne;
 * 
 *     uint8_t ret = A1006_Init_Driver(&A1006_auth);
 * 
 * @endcode
 *
 * Basic Operation:
 * @code
 *
 *     // Obtaining UID
 *     uint8_t ret = 1;
 *     uint8_t uid[16] = {0};
 *
 *     if (NULL != uid)
 *     {
 *         ret = A1006_Get_Uid(uid);
 *     }
 *
 *     // Obtaining Certificate
 *     uint8_t cert[128] = {0};
 *
 *     if (NULL != cert)
 *     {
 *         ret = A1006_Get_Cert(cert);
 *     }
 *
 *     // Setting Challenge from Host
 *     uint8_t challenge[44] = {0};
 *
 *     if (NULL != challenge)
 *     {
 *         ret = A1006_Set_Challenge(challenge);
 *     }
 *
 *     // Getting channenge Response from A1006
 *     uint8_t response[44] = {0};
 *
 *     if (NULL != response)
 *     {
 *         ret = A1006_Get_Response(response);
 *     }
 *
 * @endcode
 *
 * Memory Considerations
 * -------------------------------------------------------------------------------------------------
 * It is the responsibility of the developer to allocate the necessary memory.
 *
 * The following buffer sizes are needed:
 *     - A1006_Get_Uid:
 *         + uid buffer must be 16 bytes
 *     - A1006_Get_Cert:
 *         + cert buffer must be 128 bytes
 *     - A1006_Set_Challenge:
 *         + challenge buffer must be 44 bytes
 *     - A1006_Get_Response:
 *         + response buffer must be 44 bytes
 * 
 * Certificate Options
 * -------------------------------------------------------------------------------------------------
 * The a100x_interface can be directed to use two different certificates for authentication.
 * 
 * These options are:
 *     -# User Certificate [kCertificateOne]
 *     -# NXP Certificate  [kCertificateTwo]
 *
 * @{
 * @brief A100x interface routines to communicate with the A100x over I2C.
 */

/*! @file */

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief A1006 Certificate Slots*/
typedef enum _a1006_cert_slot
{
    kCertificateOne = 0,              /*!< Directs a100x_interface to use User Certificate */
    kCertificateTwo,                  /*!< Directs a100x_interface to use NXP Certificate */
    kCertificateInvalid = 0xFFFFFFFF, /*!< Represent invalid certificate slot */
} a1006_cert_slot_t;

/*! @brief A1006 IO Functions*/
typedef struct _a1006_IoFunc_t
{
    uint8_t   (*I2C_Read)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize);   /*!< Function pointer to I2C Read function */
    uint8_t   (*I2C_Write)(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize);                                       /*!< Function pointer to I2C Write function */
    void      (*WaitMsec)(uint32_t millisec);                                                                                 /*!< Function pointer to WaitMsec function */
    uint8_t   i2c_Address;                                                                                                    /*!< 7-bit I2C slave address */
    a1006_cert_slot_t cert_slot;                                                                                              /*!< Enum for slot number to read out credentials */
} a1006_IoFunc_t, *pa1006_IoFunc_t;

/*! @brief A1006 Transfer Status */
typedef enum _a1006_transfer_status
{
    kTransferOkay = 0U,   /*!< Transfer Ok. */
    kTransferFail,        /*!< Transfer Fail. */
    kTransferTimeout,     /*!< Transfer Timeout. */
    kInvalidInput,        /*!< Invalid Input. */
} a1006_transfer_status_t;

/*! @brief A1006 Initialization Status */
typedef enum _a1006_init_status
{
    kInitSuccess,         /*!< A1006 Initialization Successful. */
    kAlreadyInitialized,  /*!< A1006 Already Initialized. */
    kDeinitSuccess,       /*!< A1006 Deinitialization Successful. */
    kNotInitialized       /*!< A1006 Not Initialized. */
} a1006_init_status_t;

/*!
 * @brief Initialize  interfaceA100x slave.
 *
 * @note This API should be called at the beginning of the application before using the driver.
 *
 * @param pIoFunc Input I/O function structure
 *
 * @return None
 */
uint8_t A1006_Init_Driver(a1006_IoFunc_t* pIoFunc);

/*!
 * @brief De-Initialize interface to A100x slave.
 *
 * @note This API should be called once application has finished using A100x
 *
 * @return None
 */
uint8_t A1006_Deinit_Driver(void);


/*!
 * @brief Soft reset using general call (0x00)
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Soft_Reset(void);

/*!
 * @brief Power down device with I2C command
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Power_Down(void);

/*!
 * @brief Wake-up device with I2C command
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Wake_Up(void);

/*!
 * @brief Get device statuc over I2C
 *
 * @param a1006_status Output 32-bit integer status value
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Get_Status(uint32_t * a1006_status);

/*!
 * @brief Get UID from device over I2C
 *
 * @param uid  Output 16 byte buffer containing uid from device
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Get_Uid(uint8_t *uid);

/*!
 * @brief Get raw compressed certificate from device over i2c
 *
 * @param cert Output 128 byte buffer for containing raw certificate data
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Get_Cert(uint8_t *cert);

/*!
 * @brief Set challenge on device over I2C
 *
 * @param challenge Input 44 byte buffer containing the challenge to place in device
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Set_Challenge(uint8_t *challenge);

/*!
 * @brief Get challenge response from device over I2C
 *
 * @param response Output 44 byte buffer containing the response from the device
 *
 * @return 0 on success\n
 *         != 0 on error
 */
uint8_t A1006_Get_Response(uint8_t *response);

/*! @}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif // _A100X_INTERFACE_H_
