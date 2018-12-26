/**
 ****************************************************************************************
 *
 * @file qpp_common.h
 *
 * @brief Header File - QBlue private profile common types.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _YELL_COMMON_H_
#define _YELL_COMMON_H_

/**
 ****************************************************************************************
 * @addtogroup QBlue private Profile
 * @ingroup PROFILE
 * @brief QBlue private Profile
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_YELL_CLIENT || BLE_YELL_SERVER)

#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define YELL_SVC_PRIVATE_UUID "\xFB\x34\x9B\x5F\x80\x00\x00\x80\x00\x10\x00\x00\xEA\xFE\x00\x00"
#define YELLS_RX_CHAR_UUID "\x00\x97\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"
#define YELLS_TX_CHAR_UUID "\x01\x97\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"
#define YELLS_ENUM_CHAR_UUID "\x02\x97\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"
//#define QPPS_TX_CHAR_UUID2 "\x03\x97\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"

// Used as max data length
#define YELL_DATA_MAX_LEN (512)

// error code
#define YELLS_ERR_RX_DATA_EXCEED_MAX_LENGTH (0x82)

enum
{
    YELLS_VALUE_NTF_OFF = 0x00,
    YELLS_VALUE_NTF_ON = 0x01,
};
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#endif /* #if (BLE_YELL_CLIENT || BLE_YELL_SERVER) */

/// @} yell_common

#endif /* _YELL_COMMON_H_ */
