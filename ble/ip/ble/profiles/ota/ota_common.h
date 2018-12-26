/**
 ****************************************************************************************
 *
 * @file ota_common.h
 *
 * @brief Header File - OTA profile common types.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTA_COMMON_H_
#define _OTA_COMMON_H_

/**
 ****************************************************************************************
 * @addtogroup OTA Profile
 * @ingroup PROFILE
 * @brief OTA Profile
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define ATT_SVC_OTA_UUID                    "\xE0\x1C\x4B\x5E\x1E\xEB\xA1\x5C\xEE\xF4\x5E\xBA\x50\x55\xFF\x01"
#define ATT_CHAR_OTA_TARGET_RSP_UUID        "\xE0\x1C\x4B\x5E\x1E\xEB\xA1\x5C\xEE\xF4\x5E\xBA\x51\x55\xFF\x01"
#define ATT_CHAR_OTA_HOST_CMD_UUID          "\xE0\x1C\x4B\x5E\x1E\xEB\xA1\x5C\xEE\xF4\x5E\xBA\x52\x55\xFF\x01"

/// Maximum block length
#define OTA_BLOCK_MAX_LEN               (512)
/*
 * ENUMS
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

/// @} ota_common

#endif /* _OTA_COMMON_H_ */
