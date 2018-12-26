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
#include <stdint.h>
#include <string.h>
#include "app_beacon.h"
#include "gap.h"

// advertise_packet_t adv_device_name;

#if BEACON
packet_beacon_t g_PacketBeacon;
#endif

#if FSLBEACON
packet_fslbeacon_t g_PacketFslbeacon;
#endif

#if EDDYBEACON
packet_eddy_beacon_t g_PacketEddyBeacon;
#endif

struct app_beacon_env_tag g_AppBeaconEnv;

void APP_BeaconInit(void)
{
//		adv_device_name.packet_len = sizeof(DEVICE_NAME) + 1;
//	  adv_device_name.adv_type = GAP_AD_TYPE_COMPLETE_NAME;
//		adv_device_name.pdata = (uint8_t *)&DEVICE_NAME;

#if BEACON
    g_PacketBeacon.len = BEACON_LEN;
    g_PacketBeacon.type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    g_PacketBeacon.company_id = COMPANY_ID;
    g_PacketBeacon.beacon_type = BEACON_TYPE;
    memcpy(g_PacketBeacon.uuid, PROXIMITY_UUID, sizeof(g_PacketBeacon.uuid));
    g_PacketBeacon.major_ver = MAJOR;
    g_PacketBeacon.minor_ver = MINOR;
    g_PacketBeacon.power_measured = MEASURED_PWR;

#endif

#if FSLBEACON
    g_PacketFslbeacon.len = FSLBEACON_LEN;
    g_PacketFslbeacon.type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    g_PacketFslbeacon.company_id[0] = FSLCOMPANY_ID & 0xff;
    g_PacketFslbeacon.company_id[1] = FSLCOMPANY_ID >> 8;
    g_PacketFslbeacon.beacon_id = BEACON_ID;
    memcpy(g_PacketFslbeacon.uuid, FSL_UUID, sizeof(g_PacketFslbeacon.uuid));
    g_PacketFslbeacon.appData_A[0] = APPDATA_A & 0xff;
    g_PacketFslbeacon.appData_A[1] = APPDATA_A >> 8;
    g_PacketFslbeacon.appData_B[0] = APPDATA_B & 0xff;
    g_PacketFslbeacon.appData_B[1] = APPDATA_B >> 8;
    g_PacketFslbeacon.appData_C[0] = APPDATA_C & 0xff;
    g_PacketFslbeacon.appData_C[1] = APPDATA_C >> 8;
    g_PacketFslbeacon.power_measured = FSLMEASURED_PWR;

#endif

#if EDDYBEACON
    g_PacketEddyBeacon.len = 0x03;
    g_PacketEddyBeacon.type = GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID;
    g_PacketEddyBeacon.eddystone_uuid1[0] = SERVICE_UUID & 0xff;
    g_PacketEddyBeacon.eddystone_uuid1[1] = SERVICE_UUID >> 8;
//	packet_eddy_beacon.eddystone_uuid1[0] = SERVICE_UUID >> 8;
//	packet_eddy_beacon.eddystone_uuid1[1] = SERVICE_UUID & 0xff;

#if FRAMETYPE == FRAMETYPE_UID
    static servicedata_frame_UID_t frame_uid;
    g_PacketEddyBeacon.servicedata_len = 3 + sizeof(servicedata_frame_UID_t);
    g_PacketEddyBeacon.servicedata_type = GAP_AD_TYPE_SERVICE_16_BIT_DATA;
    g_PacketEddyBeacon.eddystone_uuid2[0] = SERVICE_UUID & 0xff;
    g_PacketEddyBeacon.eddystone_uuid2[1] = SERVICE_UUID >> 8;
    //	packet_eddy_beacon.eddystone_uuid2[0] = SERVICE_UUID >> 8;
    //	packet_eddy_beacon.eddystone_uuid2[1] = SERVICE_UUID & 0xff;
    g_PacketEddyBeacon.pServicedata_frame = (uint8_t *)&frame_uid;

    frame_uid.frametype = FRAMETYPE_UID;
    frame_uid.tx_power_0m = EDDYMEASURED_PWR;
    memcpy(frame_uid.nid, UID_NAMESPACE, sizeof(frame_uid.nid));
    memcpy(frame_uid.bid, UID_INSTANCE, sizeof(frame_uid.bid));
    frame_uid.rfu = UID_RFU;
#endif

#if FRAMETYPE == FRAMETYPE_URL
    static servicedata_frame_URL_t frame_url;
    g_PacketEddyBeacon.servicedata_len = 3 + (sizeof(URL_ENCODED) - 1) + 3;
    g_PacketEddyBeacon.servicedata_type = GAP_AD_TYPE_SERVICE_16_BIT_DATA;
    g_PacketEddyBeacon.eddystone_uuid2[0] = SERVICE_UUID & 0xff;
    g_PacketEddyBeacon.eddystone_uuid2[1] = SERVICE_UUID >> 8;
    g_PacketEddyBeacon.pServicedata_frame = (uint8_t *)&frame_url;

    frame_url.frametype = FRAMETYPE_URL;
    frame_url.tx_power_0m = EDDYMEASURED_PWR;
    frame_url.prefix_scheme = URL_PREFIX;
    memcpy(frame_url.encoded_url, URL_ENCODED, sizeof(URL_ENCODED));
#endif

#if FRAMETYPE == FRAMETYPE_TLM
    static servicedata_frame_TLMPLAIN_t frame_tlm;
    g_PacketEddyBeacon.servicedata_len = 3 + sizeof(servicedata_frame_TLMPLAIN_t);
    g_PacketEddyBeacon.servicedata_type = GAP_AD_TYPE_SERVICE_16_BIT_DATA;
    g_PacketEddyBeacon.eddystone_uuid2[0] = SERVICE_UUID & 0xff;
    g_PacketEddyBeacon.eddystone_uuid2[1] = SERVICE_UUID >> 8;
    g_PacketEddyBeacon.pServicedata_frame = (uint8_t *)&frame_tlm;

    frame_tlm.frametype = FRAMETYPE_TLM;
    frame_tlm.tlm_version = TLM_VER;
    frame_tlm.vbatt = TLM_VBATT;
    frame_tlm.beacon_temp = TLM_TEMP;
    memcpy(frame_tlm.adv_pdu_cnt, TLM_ADV_CNT, sizeof(frame_tlm.adv_pdu_cnt));
    memcpy(frame_tlm.sec_cnt, TLM_SEC_CNT, sizeof(frame_tlm.sec_cnt));
#endif

#if FRAMETYPE == FRAMETYPE_EID
    static servicedata_frame_EID_t frame_eid;
    g_PacketEddyBeacon.servicedata_len = 3 + sizeof(servicedata_frame_EID_t);
    g_PacketEddyBeacon.servicedata_type = GAP_AD_TYPE_SERVICE_16_BIT_DATA;
    g_PacketEddyBeacon.eddystone_uuid2 = SERVICE_UUID;
    g_PacketEddyBeacon.pServicedata_frame = (uint8_t *)&frame_eid;

    frame_eid.frametype = FRAMETYPE_EID;
    frame_eid.tx_power_0m = EDDYMEASURED_PWR;
    memcpy(frame_eid.eid, EID, sizeof(frame_eid.eid));
#endif
#endif
    g_AppBeaconEnv.meas_intv = 5;
    g_AppBeaconEnv.beacon_type = APP_IBEACON_TYPE;
    g_AppBeaconEnv.timer_flag = false;
    return;
}
