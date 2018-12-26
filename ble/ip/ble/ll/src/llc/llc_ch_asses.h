/**
 ****************************************************************************************
 *
 * @file llc_ch_asses.h
 *
 * @brief Declaration of functions used for channel assessment
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef LLC_CH_ASSES_H_
#define LLC_CH_ASSES_H_

/**
 ****************************************************************************************
 * @addtogroup LLCCHASSES
 * @ingroup LLC
 * @brief Channel assessment functions
 *
 * This module implements the primitives used for channel assessment
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_CHNL_ASSESS)

#include <stdint.h>
#include <stdbool.h>
#include "rwip_config.h"
#include "co_bt.h"



/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * ENUMERATIONS
 ****************************************************************************************
 */

enum lld_ch_asses_ponderation
{
    // Noiser
    LLD_CH_ASSES_SYNC_ERR_HIGH_RSSI             = -3,
    // Packet error
    LLD_CH_ASSES_CRC_ERR                        = -3,
    // Sync missed
    LLD_CH_ASSES_SYNC_ERR_LOW_RSSI_NO_LATENCY   = -1,
    // Latency
    LLD_CH_ASSES_SYNC_ERR_LOW_RSSI_LATENCY      = 0,
    // Packet ok
    LLD_CH_ASSES_SYNC_FOUND_NO_CRC_ERR          = 3
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Structure describing the channel assessment environment
struct llc_ch_asses_tag
{
    /// Standard deviation
    int8_t  rcvd_quality[LE_DATA_FREQ_LEN];
    /// Latency enable
    bool latency_en;
    /// Re-assessment counter (nb of assessment timer expiration)
    uint8_t reassess_count;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Get the informations on the received packet (error, RSSI) to take the decision
 * to remove or not the channel
 *
 * @param[in] handle      Connection handle of the current link to be analyzed
 * @param[in] status      rx status information from rx descriptor
 * @param[in] rssi        rssi value of received pdu
 * @param[in] channel     channel number of RX packet
 *
 ****************************************************************************************
 */
void llc_ch_assess_local(uint16_t conhdl, uint16_t status, int8_t rssi, uint8_t channel);

/**
 ****************************************************************************************
 * @brief Get the local channel map
 *
 * @param[in] conhdl      Connection handle of the current link
 * @param[in] map pointer local channel map
 *
 ****************************************************************************************
 */

void llc_ch_assess_get_local_ch_map(uint16_t conhdl, struct le_chnl_map *map);
/**
 ****************************************************************************************
 * @brief Get the current channel map
 *
 * @param[in] conhdl      Connection handle of the current link
 *
 * @param[out] le_chnl_map   Current channel map
 *
 ****************************************************************************************
 */
struct le_chnl_map * llc_ch_assess_get_current_ch_map(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Merge 2 channel maps and return the number of available channel
 *
 * @param[in] output_map    Merged channel map
 * @param[in] host_map      Host channel map
 * @param[in] local_map     Local channel map (assessment and reassessment)
 *
 * @param[out] number of available channel
 *
 ****************************************************************************************
 */
uint8_t llc_ch_assess_merge_ch(struct le_chnl_map *output_map, struct le_chnl_map *host_map,
        struct le_chnl_map *local_map);

/**
 ****************************************************************************************
 * @brief Re-enable all the channel an clear the statistics for a dedicated link
 *
 * @param[in] output_map    Rassessment channel map
 * @param[in] conhdl        Connection handle of the current link
 *
 ****************************************************************************************
 */
void llc_ch_assess_reass_ch(struct le_chnl_map *output_map, uint16_t conhdl);
#endif//#if (BLE_CHNL_ASSESS)

/// @} LLCCHASSES

#endif // LLC_CH_ASSES_H_
