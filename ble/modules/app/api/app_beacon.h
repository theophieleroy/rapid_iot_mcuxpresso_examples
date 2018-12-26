/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#ifndef _BLE_BEACON_H_
#define _BLE_BEACON_H_
#include <stdint.h>
#include "timer_manager.h"
#define PERIODICAL_MODE
#define BEACON 1
#define FSLBEACON 1
#define EDDYBEACON 1

//#define DEVICE_NAME                 "\x4E\x58\x50\x42\x65\x61\x63\x6f\x6e" //NXPBeacon
/*! @brief Advertising minimum interval */
#define CFG_BEACON_INT_MIN (0xA0) // 625us as unit
/*! @brief Advertising maximum interval */
#define CFG_BEACON_INT_MAX (0xA0) // 625us as unit

#if BEACON
#define BEACON_LEN 0x1A
#define COMPANY_ID 0x004c
#define BEACON_TYPE 0x1502
#define PROXIMITY_UUID "\x4E\x58\x50\x2D\x51\x4E\x39\x30\x38\x58\x2D\x41\x44\x56\x2D\x33"
#define MAJOR 0x0101
#define MINOR 0x0202
#define MEASURED_PWR 0x43
#endif

#if FSLBEACON
#define FSLBEACON_LEN 0x1B
#define FSLCOMPANY_ID 0x01FF
#define BEACON_ID 0xBC
#define FSL_UUID "\x4E\x58\x50\x2D\x51\x4E\x39\x30\x38\x58\x2D\x41\x44\x56\x2D\x33"
#define APPDATA_A 0x0101
#define APPDATA_B 0x0202
#define APPDATA_C 0x0303
#define FSLMEASURED_PWR 0x43
#endif

#if EDDYBEACON
#define SERVICE_UUID 0xFEAA

#define FRAMETYPE_UID 0x00
#define FRAMETYPE_URL 0x10
#define FRAMETYPE_TLM 0x20
#define FRAMETYPE_EID 0x30
#define FRAMETYPE_RESERVED 0x40

#define FRAMETYPE FRAMETYPE_URL // FRAMETYPE_UID//FRAMETYPE_TLM//FRAMETYPE_EID

#define EDDYMEASURED_PWR 0xdd // at 0m

#define UID_NAMESPACE "\x4E\x58\x50\x2D\x51\x4E\x39\x30\x38\x58"
#define UID_INSTANCE "\x4E\x58\x50\x2D\x51\x4E"
#define UID_RFU 0x0000

#define URL_PREFIX 0x00                // |0|0x00|http://www.
#define URL_ENCODED "\x6E\x78\x70\x00" // nxp.com

#define TLM_VER 0x00
#define TLM_VBATT 0xB80B // 3V big endian
#define TLM_TEMP 0x0019  // 25 degree
#define TLM_ADV_CNT "\x4E\x58\x50\x2D"
#define TLM_SEC_CNT "\x4E\x58\x50\x2D"

#define EID "\x4E\x58\x50\x2D\x51\x4E\x39\x30"

#endif

typedef struct _packet_beacon
{
    uint8_t len;
    uint8_t type;
    uint16_t company_id;
    uint16_t beacon_type;
    uint8_t uuid[16];
    uint16_t major_ver;
    uint16_t minor_ver;
    uint8_t power_measured;
} packet_beacon_t;

typedef struct _packet_fslbeacon
{
    uint8_t len;
    uint8_t type;
    uint8_t company_id[2];
    uint8_t beacon_id;
    uint8_t uuid[16];
    uint8_t appData_A[2];
    uint8_t appData_B[2];
    uint8_t appData_C[2];
    uint8_t power_measured;
} packet_fslbeacon_t;

typedef struct _packet_eddy_beacon
{
    uint8_t len;
    uint8_t type;
    uint8_t eddystone_uuid1[2];
    uint8_t servicedata_len;
    uint8_t servicedata_type;
    uint8_t eddystone_uuid2[2];
    uint8_t *pServicedata_frame;
} packet_eddy_beacon_t;

typedef struct _servicedata_frame_UID
{
    uint8_t frametype;
    uint8_t tx_power_0m;
    uint8_t nid[10];
    uint8_t bid[6];
    uint16_t rfu;
} servicedata_frame_UID_t;

typedef struct _servicedata_frame_URL
{
    uint8_t frametype;
    uint8_t tx_power_0m;
    uint8_t prefix_scheme;
    uint8_t encoded_url[17];
} servicedata_frame_URL_t;

typedef struct _servicedata_frame_TLMPLAIN
{
    uint8_t frametype;
    uint8_t tlm_version;
    uint16_t vbatt;
    uint16_t beacon_temp;
    uint8_t adv_pdu_cnt[4];
    uint8_t sec_cnt[4];
} servicedata_frame_TLMPLAIN_t;

typedef struct _servicedata_frame_EID
{
    uint8_t frametype;
    uint8_t tx_power_0m;
    uint8_t eid[8];
} servicedata_frame_EID_t;

typedef struct _advertise_packet
{
    uint8_t packet_len;
    uint8_t adv_type;
    uint8_t *pdata;
} advertise_packet_t;

struct app_beacon_env_tag
{
    uint16_t meas_intv;
    uint8_t beacon_type;
    bool timer_flag;
};

enum app_beacon_type
{

    APP_IBEACON_TYPE = 0x00,
    APP_FSLBEACON_TYPE,
    APP_EDDYBEACON_TYPE,
    APP_TYPE_MAX,
};

// extern advertise_packet_t adv_device_name;

#if BEACON
extern packet_beacon_t g_PacketBeacon;
#endif

#if FSLBEACON
extern packet_fslbeacon_t g_PacketFslbeacon;
#endif

#if EDDYBEACON
extern packet_eddy_beacon_t g_PacketEddyBeacon;
#endif

extern struct app_beacon_env_tag g_AppBeaconEnv;

extern void APP_BeaconInit(void);
#endif
