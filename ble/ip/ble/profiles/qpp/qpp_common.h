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

#ifndef _QPP_COMMON_H_
#define _QPP_COMMON_H_

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
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define QPP_SVC_PRIVATE_UUID            "\xFB\x34\x9B\x5F\x80\x00\x00\x80\x00\x10\x00\x00\xE9\xFE\x00\x00"
#define QPPS_RX_CHAR_UUID               "\x00\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"
#define QPPS_TX_CHAR_UUID               "\x01\x96\x12\x16\x54\x92\x75\xB5\xA2\x45\xFD\xAB\x39\xC4\x4B\xD4"

// Used as max data length
#define QPP_DATA_MAX_LEN                (512)

// QPP throughput statistics timeout
#define QPP_THROUGHPUT_STATS_TIMEOUT    (1000)

enum
{   
    QPPS_VALUE_NTF_OFF = 0x00,
    QPPS_VALUE_NTF_ON  = 0x01,
};
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// QPP statistics 
struct qpp_throughput_stats
{
    uint32_t prev_tx_bytes;
    uint32_t prev_rx_bytes;
    uint32_t curr_tx_bytes;
    uint32_t curr_rx_bytes;
};

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

/// @} qpp_common

#endif /* _QPP_COMMON_H_ */
