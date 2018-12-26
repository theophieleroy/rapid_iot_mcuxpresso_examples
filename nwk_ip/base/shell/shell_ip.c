/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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

/*!=================================================================================================
\file       shell_ip.c
\brief      This is a public source file for the shell application. It contains the implementation
            of the shell commands used in the application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include <string.h>
#include <stdio.h>

#include "shell_ip.h"
#include "app_stack_config.h"
#include "stack_config.h"
#include "app_init.h"

#include "shell.h"
#include "ModuleInfo.h"
#include "ip_sockets.h"
#include "ip_if_management.h"
#include "ip6.h"
#include "icmp.h"
#include "sixlowpan.h"
#include "FunctionLib.h"
#include "session.h"
#include "nd.h"
#include "nd_cfg.h"
#include "mac_filtering.h"
#include "thread_utils.h"
#include "thread_meshcop.h"
#include "thread_meshcop_mgmt.h"
#include "thread_mgmt.h"
#include "coap.h"
/* LED Application */
#include "app_led.h"
#include "sensors.h"
#include "peripherals.h"

/* Authentication */
#include "rpk_base64.h"
#include "a100x_interface.h"

#if STACK_THREAD
    #include "thread_network.h"
    #include "thread_utils.h"
    #include "mle_basic.h"
    #include "app_thread_config.h"
#endif

#if SOCK_DEMO
    #include "app_socket_utils.h"
#endif

#if UDP_ECHO_PROTOCOL
    #include "app_echo_udp.h"
#endif

#if COAP_OBSERVE_CLIENT || COAP_OBSERVE_SERVER
    #include "app_observe_demo.h"
#endif

#if DNS_ENABLED
    #include "dns_client.h"
    #include "dns_utils.h"
    #include "thread_dns.h"
#endif

#if THREAD_USE_SHELL || FRDM_K64F_KW41Z

#include "cmd_threadip.h"
#include "cmd_ble.h"

#if gHybridApp_d
    #include "ble_shell.h"
#endif

#define THR_FSCI_IF     0
#define BLE_FSCI_IF     1

// extern bool_t gbRetryInterrupt;
bool_t gbRetryInterrupt;

#ifdef USE_MSD_BOOTLOADER
    extern void NvEraseSector(uint32_t sectorAddr);
#endif

//extern const uint32_t gThrNwkDataMinStableLifetime;

#if ECHO_PROTOCOL
    extern uint16_t mEchoUdpCounter;
#endif

/*==================================================================================================
Private macros
==================================================================================================*/
#define PING_ID                     (1U)
#define PING_SEQ_NB                 (1U)
#define PING_PAYLOAD_DEFAULT_SIZE   (32U)
#define PING_PAYLOAD_START          '@'
#define PING_PAYLOAD_END            'W'
#if SHELL_DUT_COMMISSIONER
    #define SHELL_CMD_MAX_ARGS          (25U)
#else
    #define SHELL_CMD_MAX_ARGS          (20U)
#endif
#define PING_HEADER_SIZE            (4U)
#define DEFAULT_TIMEOUT             (2000U)
#define SHELL_PING_MIN_TIMEOUT      (2000U)
#define PING_DELAY                  (500U)
#if COAP_OBSERVE_CLIENT
    #define SHELL_MAX_COAP_URI      (30U)
#endif
#define STRING_TRUE                 "TRUE"
#define STRING_FALSE                "FALSE"

/*! NXP Thread stack Version - Human readable form, for SHELL */
#define gNXPThreadStackVersion_c "NXP " \
                                 QUH(gIpVerMajor_c) "." \
                                 QUH(gIpVerMinor_c) "." \
                                 QUH(gIpVerPatch_c)

/* GET REQUESTS FROM GATEWAY */
#define APP_CO2_URI_PATH                        "/co2"
#define APP_LIGHT_URI_PATH                      "/light"
#define APP_HUMID_URI_PATH                      "/humid"
#define APP_TEMP_URI_PATH                       "/temp"
#define APP_PRES_URI_PATH                       "/pres"
#define APP_ROTSPEED_URI_PATH                   "/rot_speed"
#define APP_ACCEL_URI_PATH                      "/accel"
#define APP_MAGFIELD_URI_PATH                   "/mag_field"
#define APP_BATTERY_URI_PATH                    "/bat"

/* GET / POST REQUESTS FROM GATEWAY */
#define APP_BUZZER_URI_PATH                     "/buzzer"
#define APP_BACKLIGHT_URI_PATH                  "/backlight"
#define APP_RGBLED_URI_PATH                     "/rgb_led"
#define APP_DATE_URI_PATH                       "/date"
#define APP_TIME_URI_PATH                       "/time"

/* RPK PUSHING COAP MESSAGES AT SPECIFIC EVENTS */
#define APP_BUTTON_URI_PATH                     "/button"
#define APP_TOUCH_URI_PATH                      "/touch"

/* Authentication URIs */
#define APP_AUTH_REQ_URI_PATH                   "/auth_req"
#define APP_AUTH_CHL_URI_PATH                   "/auth_chl"
#define APP_AUTH_RES_URI_PATH                   "/auth_res"
#define APP_AUTH_UID_RAW_LEN					16
#define APP_AUTH_CERT_RAW_LEN					128
#define APP_AUTH_CHL_RSP_RAW_LEN				44
#define APP_AUTH_UID_ENC_LEN					24
#define APP_AUTH_CERT_ENC_LEN					172
#define APP_AUTH_CHL_RSP_ENC_LEN				60

/* JSON reply formats below */
#define APP_CO2_JSON_FORMAT                     "{\"uri\":\"/co2_get\",\"data\":[{\"co2\":\"%d\"}]}"
#define APP_LIGHT_JSON_FORMAT                   "{\"uri\":\"/light_get\",\"data\":[{\"light\":\"%.2f\"}]}"
#define APP_HUMID_JSON_FORMAT                   "{\"uri\":\"/humid_get\",\"data\":[{\"humid\":\"%.2f\"}]}"
#define APP_TEMP_JSON_FORMAT                    "{\"uri\":\"/temp_get\",\"data\":[{\"temp\":\"%.2f\"}]}"
#define APP_PRES_JSON_FORMAT                    "{\"uri\":\"/pres_get\",\"data\":[{\"pres\":\"%d\"}]}"
#define APP_ROTSPEED_JSON_FORMAT                "{\"uri\":\"/rot_speed_get\",\"data\":[{\"X\":\"%d\",\"Y\":\"%d\",\"Z\":\"%d\"}]}"
#define APP_ACCEL_JSON_FORMAT                   "{\"uri\":\"/accel_get\",\"data\":[{\"X\":\"%.6f\",\"Y\":\"%.6f\",\"Z\":\"%.6f\"}]}"
#define APP_MAGFIELD_JSON_FORMAT                "{\"uri\":\"/mag_field_get\",\"data\":[{\"X\":\"%.1f\",\"Y\":\"%.1f\",\"Z\":\"%.1f\"}]}"
#define APP_BATTERY_JSON_FORMAT                 "{\"uri\":\"/bat_get\",\"data\":[{\"bat_per\":\"%d\",\"bat_charge\":\"%d\"}]}"

#define APP_BUZZER_GET_JSON_FORMAT              "{\"uri\":\"/buzzer_get\",\"data\":[{\"buzzer\":\"%d\"}]}"
#define APP_BUZZER_POST_JSON_FORMAT             "{\"uri\":\"/buzzer_post\",\"data\":[{\"buzzer\":\"%d\"}]}"
#define APP_BACKLIGHT_GET_JSON_FORMAT           "{\"uri\":\"/backlight_get\",\"data\":[{\"backlight\":\"%d\"}]}"
#define APP_BACKLIGHT_POST_JSON_FORMAT          "{\"uri\":\"/backlight_post\",\"data\":[{\"backlight\":\"%d\"}]}"
#define APP_RGBLED_GET_JSON_FORMAT              "{\"uri\":\"/rgb_led_get\",\"data\":[{\"brightness\":\"%d\",\"color\":\"%d\"}]}"
#define APP_RGBLED_POST_JSON_FORMAT             "{\"uri\":\"/rgb_led_post\",\"data\":[{\"brightness\":\"%d\",\"color\":\"%d\"}]}"

#define APP_AUTH_REQ_JSON_FORMAT                "{\"uri\":\"/auth_req\",\"data\":[{\"uid\":\"%s\",\"cert\":\"%s\"}]}"
#define APP_AUTH_CHL_JSON_FORMAT                "{\"uri\":\"/auth_chl\",\"data\":[{\"chal\":\"%s\"}]}"
#define APP_AUTH_RES_JSON_FORMAT                "{\"uri\":\"/auth_res\",\"data\":[{\"res\":\"%s\"}]}"

static char buzzer_state = 0;

#define JSON_BUFF_SIZE  256

/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef void (*pfHandleAttr_t)(void *param);

typedef enum shellValueType_tag
{
    gString_c,
    gDecimal_c,
    gHex_c,
    gHexReversed_c,
    gArray_c,
    gTable_c,
} shellValueType_t;

typedef struct aShellThrSetAttr_tag
{
    char                                *pAttrName;
    THR_GetAttrConfirm_AttributeId_t    attrId;
    shellValueType_t                    valueType;
    uint32_t                            maxSize;
    pfHandleAttr_t                      pfHandleAttr;
    bool_t                              writable;
} aShellThrSetAttr_t;

typedef struct attrRef_tag
{
    void *pValue;
    bool_t requireFree;
} attrRef_t;

typedef struct aShellThrInterface_tag
{
    uint8_t        *pAttrName;
    ipIfUniqueId_t ifId;
} aShellThrInterface_t;

#if SHELL_DUT_COMMISSIONER
typedef struct aShellThrMgmtSetTlv_tag
{
    char                *pTlvName;
    meshCopTlv_t        tlvId;
    uint32_t            tlvLen;
} aShellThrMgmtSetTlv_t;

typedef struct shellMgmtCb_tag
{
    uint32_t tlvsLen;
    uint8_t  pTlvs[];
} shellMgmtCb_t;
#endif

typedef enum shellAutoStartStates_tag
{
    gAutostartFirstState_c,
    gFactoryResetState_c,
    gNetworkCreated_c,
    gCommissionerStarted_c,
    gAddedExpectedJoiners_c
} shellAutostartStates_t;

/*==================================================================================================
Private prototypes
==================================================================================================*/
#if 0
static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen);
#endif

static void SHELL_EvMonitorInit();
static void SHELL_Resume(void);
static int8_t SHELL_Reboot(uint8_t argc, char *argv[]);
static int8_t SHELL_Identify(uint8_t argc, char *argv[]);
static void SHELL_Process(void *param);
static void SHELL_ProcessCb(char *pCmd, uint16_t length);
static void SHELL_InterruptCb(void);
static int8_t SHELL_Ifconfig(uint8_t argc, char *argv[]);
static int8_t SHELL_Autostart(uint8_t argc, char *argv[]);
static int8_t SHELL_FactoryResetReq(uint8_t argc, char *argv[]);
static int8_t SHELL_ThrCommands(uint8_t argc, char *argv[]);

static void SHELL_ThrNwkCreate(void *param);
static void SHELL_ThrNwkJoin(void *param);
static void SHELL_ThrStartCommissioner(void *param);
static void SHELL_ThrStopCommissioner(void *param);
static void SHELL_PrintRoutingTbl(void *param);

static uint8_t *SHELL_SearchInterfaceById(ipIfUniqueId_t ifID);
static void SHELL_PrintStatus(thrStatus_t statusCode);
static void SHELL_PrintParentInfo(void *param);
static void SHELL_PrintNeighborTbl(void *param);
static void SHELL_PrintNeighborInfo(void *param);

static void SHELL_PrintDevRole(void *param);
static void SHELL_PrintDevType(void *param);
static void SHELL_PrintNwkCapabilities(void *param);
static void SHELL_PrintBuff(uint8_t *buff, uint32_t length);
static void SHELL_ReadBuff(char *pValue, uint8_t *pDest, uint32_t length);
static void SHELL_PrintActiveTimestamp(void *param);

/* CoAP functions */
static int8_t SHELL_CoapSend(uint8_t argc, char *argv []);
static void   SHELL_CoapSendMsg(void *param);
static int8_t SHELL_CoapReply(NWKU_CoapMsgReceivedIndication_t *indication);

/* Ping functions */
static int8_t SHELL_Ping(uint8_t argc, char *argv[]);
static void PING_EchoReplyReceive(void *pParam);
static void PING_HandleTimeout(void *param);
static void PING_RetransmitHandle(void *param);

#if ICMP_STATISTICS_ENABLED
    static void SHELL_PrintPingStatistics();
    static void float_division(char *intPart, char *fractPart, uint32_t sent, uint32_t lost);
#endif
static int8_t SHELL_MulticastGroups(uint8_t argc, char *argv[]);

#if SOCK_DEMO
    static int8_t SHELL_Socket(uint8_t argc, char *argv[]);
#endif

#if UDP_ECHO_PROTOCOL
    static int8_t SHELL_EchoUdp(uint8_t argc, char *argv[]);
#endif

#if DNS_ENABLED
static int8_t SHELL_SendDns(uint8_t argc, char *argv[]);
static void SHELL_DnsService(void *pMsg);
static void SHELL_DnsNdDataService(coapSessionStatus_t sessionStatus, void *pData,
                                   coapSession_t *pSession, uint32_t dataLen);
#endif

#if MAC_FILTERING_ENABLED
    static int8_t SHELL_Filter(uint8_t argc, char *argv[]);
    static void SHELL_Filtering_Print(uint32_t instanceID);
#endif /* MAC_FILTERING_ENABLED */

#ifdef USE_MSD_BOOTLOADER
    static int8_t SHELL_FlashEraseWithMSDFunc(uint8_t argc, char *argv[]);
#endif

#if gHybridApp_d
    static int8_t SHELL_BreakReq(uint8_t argc, char *argv[]);
#endif

#if GETIPv6ADDRESSES_APP
    /*===================================GETML64ADDRESSES=============================================*/
    static int8_t SHELL_GetNeighborsIpAddr(uint8_t argc, char *argv[]);
    static void SHELL_MgmtDiagnosticAppCb(mgmtDiagnostic_RspData_t mgmtDiagRspData);
    /*================================================================================================*/
#endif

#if SHELL_DUT_COMMISSIONER
static void SHELL_ReadMgmtTlv(uint8_t argc, char *argv[], uint8_t *pTlvBuff,
                              uint32_t *pTlvBuffLen, bool_t bTlvSet);
static void SHELL_MeshcopMgmtSetCb(meshcopHandlers_t *pIdHandlerEntry,
                                   uint8_t *pTlvs, uint32_t tlvsLen);
static void SHELL_MeshcopMgmtGetCb(meshcopHandlers_t *pIdHandlerEntry,
                                   uint8_t *pTlvs, uint32_t tlvsLen);
static void SHELL_MeshcopMgmtPanIdConflictCb(meshcopHandlers_t *pIdHandlerEntry,
        uint8_t *pTlvs, uint32_t tlvsLen);
static void SHELL_MgmtPanIdConflictCbHandler(void *param);
static void SHELL_MgmtSetCbHandler(void *param);
static void SHELL_MgmtGetCbHandler(void *param);
#endif
/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static NWKU_PingRequest_t mPingReq;
static bool_t           mContinuousPing = FALSE;
static uint32_t         mPingTimeoutMs;
static uint16_t         mPingSize = 0;
static uint16_t         mPingCounter = 0;
static ipAddr_t         mDstIpAddr;

static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;
static char             mAddrStr[INET6_ADDRSTRLEN];
static tmrTimerID_t     mTimerIDSelect = gTmrInvalidTimerID_c;
static tmrTimerID_t     mCoapDelayTimerID = gTmrInvalidTimerID_c;

static bool_t           mShellCommandsEnabled = FALSE;    /*!< Avoid initializing the module multiple times */
static uint8_t          mCoapInstId = THR_ALL_FFs8;       /*!< CoAP instance */
#if COAP_OBSERVE_CLIENT
static uint8_t mResourceString[SHELL_MAX_COAP_URI] = {0};
static coapUriPath_t mResource;
#endif

static bool_t mAutostartInProgress = FALSE;
static shellAutostartStates_t mAutostartState = gAutostartFirstState_c;
static uint8_t mAutostartChannel = THR_ALL_FFs8;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

const cmd_tbl_t aShellCommands[] =
{
    {
        "                   ", SHELL_CMD_MAX_ARGS, 0, NULL
#if SHELL_USE_HELP
        , "",
        ""
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
    {
        "thr", SHELL_CMD_MAX_ARGS, 0, SHELL_ThrCommands
#if SHELL_USE_HELP
        , "Thread Stack commands",
        "Commands for Thread Network \r\n"
        "   thr create\r\n"
        "   thr join\r\n"
        "   thr scan [active|energy|both]\r\n"
        "   thr detach\r\n"
        "   thr commissioner [start|stop]\r\n"
        "   thr joiner add <psk> <eui>\r\n"
        "   thr joiner remove <eui>\r\n"
        "   thr joiner removeall\r\n"
        // "   thr joiner view\r\n"
        "   thr sync steering {all}\r\n"
        "   thr sync nwkdata\r\n"
        "   thr get attributes - displays a list of all Thread attributes available for getting/setting\r\n"
        "   thr get <ATTRNAME/TABLENAME> - displays the value for the specified attribute\r\n"
        "   thr set <ATTRNAME> <value> - changes the value of the attribute with the specified value\r\n"
        "   thr nwkdata add slaac -p <Prefix> - len <prefixLength> -t <lifetime in seconds>\r\n"
        "   thr nwkdata add dhcpserver -p <Prefix> - len <prefixLength> -t <lifetime in seconds>\r\n"
        "   thr nwkdata add extroute -p <Prefix> - len <prefixLength> -t <lifetime in seconds>\r\n"
#if DNS_ENABLED
        "   thr nwkdata add dnsserver <ipAddress>\r\n"
        "   thr nwkdata remove dnsserver <ipAddress>\r\n"
#endif
        "   thr nwkdata remove -p <Prefix> - len <prefixLength>\r\n"
        "   thr nwkdata removeall\r\n"
#if SHELL_DUT_COMMISSIONER
        "   thr mgmt gettlvs - displays a list of all TLV names available for get/set\r\n"
        "   thr mgmt activeset <addr> <tlvname> <value>\r\n"
        "   thr mgmt activeget <addr> <tlvname>\r\n"
        "   thr mgmt pendset <addr> <tlvname> <value>\r\n"
        "   thr mgmt pendget <addr> <tlvname>\r\n"
        "   thr mgmt commset <addr> <tlvname> <value>\r\n"
        "   thr mgmt commget <addr> <tlvname>\r\n"
        "   thr mgmt query <addr> <tlvname> <value>\r\n"
#endif
        //"   thr remove  router <router's short address>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
    {
        "ifconfig", SHELL_CMD_MAX_ARGS, 0, SHELL_Ifconfig
#if SHELL_USE_HELP
        , "IP Stack interfaces configuration",
        "IP Stack interfaces configuration\r\n"
        "   ifconfig - displays all interfaces and addresses configured on the device\r\n"
#if ND_ENABLED
        "   ifconfig ncache - displays the ND neighbors\r\n"
#endif
#endif /* SHELL_USE_HELP */
        "   ifconfig <interface ID> ip <IP address>", NULL
    },
    {
        "mcastgroup", SHELL_CMD_MAX_ARGS, 0, SHELL_MulticastGroups
#if SHELL_USE_HELP
        , "Multicast groups management",
        "Multicast groups management\r\n"
        "   mcastgroup show - displays all joined multicast groups\r\n"
        "   mcastgroup add <IP group address> <interface id>- joins to a new multicast group\r\n"
        "   mcastgroup leave <IP group address> <interface id>- leaves a multicast group\r\n"
        "Warning! Do not remove addresses that may affect stack's behaviour!"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
    {
        "ping", SHELL_CMD_MAX_ARGS, 0, SHELL_Ping
#if SHELL_USE_HELP
        , "IP Stack ping tool",
        "IP Stack ping IPv4/IPv6 addresses\r\n"
        "   ping <ip address> -i <timeout> -c <count> -s <size> -t <continuous ping> -S <source IP address>\r\n"
        // "   Valid interfaces: 6LoWPAN, eth, serialtun, usbEnet\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#if DNS_ENABLED
    {
        "dnsrequest", SHELL_CMD_MAX_ARGS, 0, SHELL_SendDns
#if SHELL_USE_HELP
        , "Send DNS request",
        "dnsrequest <name>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif
    {
        "coap", SHELL_CMD_MAX_ARGS, 0, SHELL_CoapSend
#if SHELL_USE_HELP
        , "Send CoAP message",
        "Send CoAP message\r\n"
        "   coap <reqtype: CON/NON> <reqcode (GET/POST/PUT/DELETE)> <IP addr dest> <URI path> <payload ASCII>\r\n"
#if COAP_OBSERVE_CLIENT
        "   coap start observe client <server's IP address> <resource>\r\n"
        "   coap stop observe client <server's IP address> <resource>\r\n"
#endif
#if COAP_OBSERVE_SERVER
        "   coap start observe server\r\n"
#endif
        "Example: coap CON POST 2001::1 /led on \r\n"
        "         coap CON POST 2001::1 /led off \r\n"
        "         coap CON POST 2001::1 /led toggle \r\n"
        "         coap CON POST 2001::1 /led flash\r\n"
        "         coap CON POST 2001::1 /led rgb r255 g255 b255\r\n"
        "         coap CON GET 2001::1 /temp\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#if SOCK_DEMO
    {
        "socket", SHELL_CMD_MAX_ARGS, 0, SHELL_Socket
#if SHELL_USE_HELP
        , "IP Stack BSD Sockets commands",
        "IP Stack BSD Sockets commands\r\n"
        "   socket open <protocol> <remote ip addr> <remote port>\r\n"
        //"   socket open tcp <ip version>- open a socket and wait for tcp connections\r\n"
        "   socket send <socket id> <payload>\r\n"
        "   socket close <socket id>\r\n"
        //"   socket receive <socket id>\r\n"
        //"   socket select <socket id>\r\n"
        //"   socket poll <socket id> <timeout>\r\n"
        //"   socket post <socket id> <timeout>\r\n"
        //"   socket poll selected <timeout>\r\n"
        //"   socket connect <socket id>\r\n"
        //"   socket accept <socket id>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif
    {
        "reboot", SHELL_CMD_MAX_ARGS, 0, SHELL_Reboot
#if SHELL_USE_HELP
        , "MCU Reset",
        "MCU Reset"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#if gHybridApp_d
    {
        "break", SHELL_CMD_MAX_ARGS, 0, SHELL_BreakReq
#if SHELL_USE_HELP
        , "Stop process Command",
        "Stop process Command\r\n"
        "   break\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif
    {
        "factoryreset", SHELL_CMD_MAX_ARGS, 0, SHELL_FactoryResetReq
#if SHELL_USE_HELP
        , "FactoryReset Command",
        "FactoryReset Command\r\n"
        "   factoryreset\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },

    {
        "autostart", SHELL_CMD_MAX_ARGS, 0, SHELL_Autostart
#if SHELL_USE_HELP
        , "Start the device as a Leader and allow all joiners with PSKd `THREAD`",
        "Start the device as a Leader and allow all joiners with PSKd `THREAD`\r\n"
        "   autostart -c <802.15.4. channel>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },


#if UDP_ECHO_PROTOCOL
    {
        "echoudp", SHELL_CMD_MAX_ARGS, 0, SHELL_EchoUdp
#if SHELL_USE_HELP
        , "Echo udp client",
        "Echo udp client\r\n"
        "   echoudp -s<size> -S<source address> -t<continuous request> -i<timeout> <target ip address>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif /* UDP_ECHO_PROTOCOL */


#if MAC_FILTERING_ENABLED
    {
        "macfilter", SHELL_CMD_MAX_ARGS, 0, SHELL_Filter
#if SHELL_USE_HELP
        , "MAC filtering commands",
        "MAC filtering commands\r\n"
        "   macfilter enable <reject|accept>\r\n"
        "   macfilter add <neighbor extended address> reject <0/1> lqi <neighbor link indicator>\r\n"
        "   macfilter remove <neighbor extended address>\r\n"
        "   macfilter disable \r\n"
        "   macfilter show \r\n"
        "   \r\n"
        "   Example: macfilter enable accept\r\n"
        "   macfilter add 0x1122334455667788 reject 1\r\n"
        "   Neighbor link indicator values:\r\n"
        "\tGood Link: 20 - 255 \r\n"
        "\tMedium Link: 11 - 20 \r\n"
        "\tBad Link: 3 - 10  \r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif /* MAC_FILTERING_ENABLED */
#ifdef USE_MSD_BOOTLOADER
    {
        "flasherase", SHELL_CMD_MAX_ARGS, 0, SHELL_FlashEraseWithMSDFunc
#if SHELL_USE_HELP
        , "FlashErase Command",
        "FlashErase Command\r\n"
        "   flasherase\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */

    },
#endif /* USE_MSD_BOOTLOADER */
    {
        "identify", SHELL_CMD_MAX_ARGS, 0, SHELL_Identify
#if SHELL_USE_HELP
        , "Identify Device",
        "Led flash to identify device\r\n"
        "   identify\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#if GETIPv6ADDRESSES_APP
    {
        "getnodesip", 5, 0, SHELL_GetNeighborsIpAddr
#if SHELL_USE_HELP
        , "Gets IPv6 addresses from all Thread nodes",
        "Gets IPv6 addresses from all Thread nodes\r\n"
        "   getnodesip \r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        , NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    },
#endif
};

const aShellThrSetAttr_t aShellThrAttr[] =
{
    {"activetimestamp",         THR_GetAttrConfirm_AttributeId_ActiveTimestamp,                 gDecimal_c,      GetSizeOfMember(thrActiveAttr_t, timestamp), SHELL_PrintActiveTimestamp, TRUE},
    {"autostart",               THR_GetAttrConfirm_AttributeId_NVM_RestoreAutoStart,            gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"bestchannelthreshold",    THR_GetAttrConfirm_AttributeId_SelectBestChannelEDThreshold,    gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"channel",                 THR_GetAttrConfirm_AttributeId_Channel,                         gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"childaddrmask",           THR_GetAttrConfirm_AttributeId_ChildAddrMask,                   gHex_c,          GetSizeOfMember(thrAttr_t, childAddrMask), NULL, FALSE}, // ????
    {"childreqfullnd",          THR_GetAttrConfirm_AttributeId_EndDevice_ChildEDReqFullNwkData, gDecimal_c,      sizeof(bool_t),  NULL, TRUE},
    {"childtimeout",            THR_GetAttrConfirm_AttributeId_ChildEDTimeout,                  gDecimal_c,      sizeof(uint32_t), NULL, TRUE},
    {"devicerole",              THR_GetAttrConfirm_AttributeId_DeviceRole,                      gString_c,       sizeof(uint8_t), SHELL_PrintDevRole, TRUE},
    {"devicetype",              THR_GetAttrConfirm_AttributeId_DeviceType,                      gDecimal_c,      sizeof(uint8_t), SHELL_PrintDevType, FALSE},
    {"eui",                     THR_GetAttrConfirm_AttributeId_IeeeExtendedAddr,                gHexReversed_c,  sizeof(uint64_t), NULL, FALSE},
    {"hashaddr",                THR_GetAttrConfirm_AttributeId_HashIeeeAddr,                    gHexReversed_c,  sizeof(uint64_t), NULL, FALSE},
    {"iscommissioned",          THR_GetAttrConfirm_AttributeId_IsDevCommissioned,               gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"isconnected",             THR_GetAttrConfirm_AttributeId_IsDevConnected,                  gDecimal_c,      sizeof(uint8_t), NULL, FALSE},
    {"isfastpollenabled",       THR_GetAttrConfirm_AttributeId_EndDevice_IsFastPollEnabled,     gDecimal_c,      sizeof(bool_t), NULL, FALSE},
    {"joinlqithreshold",        THR_GetAttrConfirm_AttributeId_JoinLqiThreshold,                gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"keyrotationinterval",     THR_GetAttrConfirm_AttributeId_Security_KeyRotationInterval,    gDecimal_c,      sizeof(uint32_t), NULL, TRUE},
    {"keyswitchguard",          THR_GetAttrConfirm_AttributeId_Security_KeySwitchGuardTime,     gDecimal_c,      sizeof(uint32_t), NULL, TRUE},
    {"leaderweight",            THR_GetAttrConfirm_AttributeId_LeaderWeight,                    gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"masterkey",               THR_GetAttrConfirm_AttributeId_Security_NwkMasterKey,           gArray_c,        GetSizeOfMember(thrActiveAttr_t, nwkMasterKey), NULL, TRUE},
    {"mlprefix",                THR_GetAttrConfirm_AttributeId_MLPrefix,                        gArray_c, (THR_ML_PREFIX_LEN_BITS >> 3), NULL, TRUE},
    {"modelname",               THR_GetAttrConfirm_AttributeId_ModelName,                       gString_c,       sizeof(thrOctet16_t), NULL, FALSE},
    {"neighbor", (THR_GetAttrConfirm_AttributeId_t)0,                            gTable_c,        0, SHELL_PrintNeighborInfo, FALSE},
    {"neighbors", (THR_GetAttrConfirm_AttributeId_t)0,                            gTable_c,        0, SHELL_PrintNeighborTbl, FALSE},
    {"nvm",                     THR_GetAttrConfirm_AttributeId_NVM_Restore,                     gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"nwkcapabilities",         THR_GetAttrConfirm_AttributeId_NwkCapabilities,                 gHex_c,          sizeof(uint8_t), SHELL_PrintNwkCapabilities, TRUE},
    {"nwkname",                 THR_GetAttrConfirm_AttributeId_NwkName,                         gString_c,       sizeof(thrOctet16_t), NULL, TRUE},
    {"panid",                   THR_GetAttrConfirm_AttributeId_ShortPanId,                      gHex_c,          sizeof(uint16_t), NULL, TRUE},
    {"parent", (THR_GetAttrConfirm_AttributeId_t)0,                        gTable_c,        0, SHELL_PrintParentInfo, FALSE},
    {"parentholdtime",          THR_GetAttrConfirm_AttributeId_ParentHoldTime,                  gDecimal_c,      sizeof(uint16_t), NULL, TRUE},
    {"partitionid",             THR_GetAttrConfirm_AttributeId_PartitionId,                     gHex_c,          sizeof(uint32_t), NULL, TRUE},
    {"permitjoin",              THR_GetAttrConfirm_AttributeId_PermitJoin,                      gDecimal_c,      sizeof(uint8_t), NULL, FALSE},
    {"provisioningurl",         THR_GetAttrConfirm_AttributeId_ProvisioningURL,                 gString_c,       sizeof(thrOctet64_t), NULL, TRUE},
    {"pskcpassword",            THR_GetAttrConfirm_AttributeId_Security_PSKc,                   gString_c,       sizeof(thrOctet32_t), NULL, FALSE},
    {"pskd",                    THR_GetAttrConfirm_AttributeId_Security_PSKd,                   gString_c,       sizeof(thrOctet32_t), NULL, TRUE},
    {"randomaddr",              THR_GetAttrConfirm_AttributeId_RandomExtendedAddr,              gHexReversed_c,  sizeof(uint64_t), NULL, TRUE},
    {"routes", (THR_GetAttrConfirm_AttributeId_t)0,                            gTable_c,        0, SHELL_PrintRoutingTbl, FALSE},
    {"rxonidle",                THR_GetAttrConfirm_AttributeId_RxOnIdle,                        gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"scanduration",            THR_GetAttrConfirm_AttributeId_ScanDuration,                    gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"scanmask",                THR_GetAttrConfirm_AttributeId_ScanChannelMask,                 gHex_c,          sizeof(uint32_t), NULL, TRUE},
    {"sedfastpollinterval",     THR_GetAttrConfirm_AttributeId_SleepyEndDevice_FastPollInterval, gDecimal_c,      sizeof(uint32_t), NULL, TRUE},
    {"sedpollinterval",         THR_GetAttrConfirm_AttributeId_SedPollInterval,                 gDecimal_c,      sizeof(uint32_t), NULL, TRUE},
    {"shortaddr",               THR_GetAttrConfirm_AttributeId_ShortAddress,                    gHex_c,          sizeof(uint16_t), NULL, FALSE},
    {"slaacpolicy",             THR_GetAttrConfirm_AttributeId_SlaacPolicy,                     gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"stackversion",            THR_GetAttrConfirm_AttributeId_StackVersion,                    gString_c,       0, NULL, FALSE},
    {"swversion",               THR_GetAttrConfirm_AttributeId_SwVersion,                       gString_c,       sizeof(thrOctet16_t), NULL, FALSE},
    {"uniqueaddr",              THR_GetAttrConfirm_AttributeId_UniqueExtendedAddress,           gDecimal_c,      sizeof(uint8_t), NULL, TRUE},
    {"vendordata",              THR_GetAttrConfirm_AttributeId_VendorData,                      gString_c,       sizeof(thrOctet64_t), NULL, FALSE},
    {"vendorname",              THR_GetAttrConfirm_AttributeId_VendorName,                      gString_c,       sizeof(thrOctet32_t), NULL, FALSE},
    {"xpan",                    THR_GetAttrConfirm_AttributeId_ExtendedPanId,                   gArray_c,        GetSizeOfMember(thrActiveAttr_t, xPanId), NULL, TRUE}
};

const char *mMacFilteringStatus[] =
{
    "Disabled\r\n",
    "Enabled - Default policy:Reject\r\n",
    "Enabled - Default policy:Accept\r\n",
};

const char *mThrDevRole[] =
{
    "Disconnected\r\n",
    "Sleepy End Device\r\n",
    "Minimal End Device\r\n",
    "Full End Device\r\n",
    "Router Eligible End Device\r\n",
    "Router\r\n",
    "Leader\r\n",
};

/* Interface table. User is advised to add the name of the interface according to the existing interfaces. */
const aShellThrInterface_t mThrInterfaces[] =
{
    {(uint8_t *)"6LoWPAN",      gIpIfSlp0_c},
    {(uint8_t *)"eth",          gIpIfEth0_c},
    {(uint8_t *)"serialtun",    gIpIfSerialTun_c},
    {(uint8_t *)"usbEnet",      gIpIfUsbRndis_c},
    {(uint8_t *)"6LoBLE",       gIpIfBle0_c},
};

/* Ping variables */
uint64_t pingTimeStamp = 0;
tmrTimerID_t pingTimerID = gTmrInvalidTimerID_c;

taskMsgQueue_t *pmMainThreadMsgQueue;   /*!< Pointer to main thread message queue */

ipAddr_t *pSrcIpAddr = NULL; /* Used for keeping ping source address */

uint32_t threadInstanceID = 0;

#if SHELL_DUT_COMMISSIONER
const aShellThrMgmtSetTlv_t aShellThrMgmtTlvs[] =
{
    {"ch", gMeshCopTlvChannel_c, sizeof(meshCopChannelTlv_t) - MLE_TLV_HEADER_SIZE},
    {"pan", gMeshCopTlvPanID_c, sizeof(meshCopNwkPanIdTlv_t) - MLE_TLV_HEADER_SIZE},
    {"xpan", gMeshCopTlvXpanID_c, sizeof(meshCopNwkXPanIdTlv_t) - MLE_TLV_HEADER_SIZE},
    {"nwkname", gMeshCopTlvNwkName_c, 16},
    {"pskc", gMeshCopTlvPskc_c, 16},
    {"masterkey", gMeshCopTlvNwkMasterKey_c, sizeof(meshCopNwkMasterKeyTlv_t) - MLE_TLV_HEADER_SIZE},
    {"mlprefix", gMeshCopTlvNwkMlUla_c, sizeof(meshCopNwkMlUlaTlv_t) - MLE_TLV_HEADER_SIZE},
    {"steering", gMeshCopTlvSteeringData_c, 16},
    {"brloc", gMeshCopTlvBorderRouterLoc_c, sizeof(meshCopBrLocTlv_t) - MLE_TLV_HEADER_SIZE},
    {"sid", gMeshCopTlvCommSessId_c, sizeof(meshCopCommSessIdTlv_t) - MLE_TLV_HEADER_SIZE},
    {"secpol", gMeshCopTlvSecPolicy_c, sizeof(meshCopSecurityPolicyTlv_t) - MLE_TLV_HEADER_SIZE},
    {"activets", gMeshCopTlvActiveTimestamp_c, sizeof(meshCopActiveTimestampTlv_t) - MLE_TLV_HEADER_SIZE},
    {"pendts", gMeshCopTlvPendingTimestamp_c, sizeof(meshCopActiveTimestampTlv_t) - MLE_TLV_HEADER_SIZE},
    {"delaytmr", gMeshCopTlvDelayTimer_c, sizeof(meshCopDelayTimerTlv_t) - MLE_TLV_HEADER_SIZE},
    {"chmsk", gMeshCopTlvChannelMask_c, sizeof(meshCopChannelMaskTlv_t) - MLE_TLV_HEADER_SIZE},
    {"scan", gMeshCopTlvScanDuration_c, sizeof(meshCopScanDurationTlv_t) - MLE_TLV_HEADER_SIZE},
    {"energylist", gMeshCopTlvEnergyList_c, 10},
    {"bogus", gMeshCopTlvBogus_c, 0},
};
#endif
/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void SHELLComm_Init(taskMsgQueue_t *pMsgQueue)
\brief  This function is used to initialize the SHELL commands module.

\param  [in]    pMsgQueue    Pointer to the message queue
***************************************************************************************************/
void SHELLComm_Init
(
    taskMsgQueue_t *pMsgQueue
)
{
    if (!mShellCommandsEnabled)
    {
        SHELL_EvMonitorInit(threadInstanceID);

        // TBD: check the pointer to media interface.
        pmMainThreadMsgQueue = pMsgQueue;

        /* Initialize Shell module */
        shell_init("$ ");

        pfShellProcessCommand =  SHELL_ProcessCb;

        /* Register functions */
        shell_register_function_array((cmd_tbl_t *)&aShellCommands[0], NumberOfElements(aShellCommands));
        mShellCommandsEnabled = TRUE;
    }
}

/*!*************************************************************************************************
\fn     void SHELL_PrintIpAddr(threadAddrTypes_t addrType)
\brief  This function is used for printing in shell terminal the IP addresses of a certain type.

\param  [in]    threadAddrTypes_t    the type of the IP addresses to be printed
***************************************************************************************************/
void SHELL_PrintIpAddr
(
    threadAddrTypes_t addrType
)
{
    uint32_t iIfIdx, iIpIdx;
    char addrStr[INET6_ADDRSTRLEN];
    ifHandle_t ifHandle = NULL;
    ip6IfAddrData_t *pIp6AddrData = NULL;
#if IP_IP4_ENABLE
    ip4IfAddrData_t *pIp4AddrData;
#endif

    /* IP Interfaces */
    for (iIfIdx = 0; iIfIdx < IP_IF_NB; iIfIdx++)
    {
        /* Get interface by index */
        // ifHandle = IP_IF_GetIfByIndex(iIfIdx);
        // TODO: Stack API

        if ((ifHandle != NULL) && (ifHandle->ifUniqueId != gIpIfUndef_c))
        {
            uint8_t *ifName = SHELL_SearchInterfaceById(ifHandle->ifUniqueId);

            shell_printf("\n\rInterface %d: %s", iIfIdx, ifName);
#if IP_IP4_ENABLE
            /* Get IPv4 addresses for an interface */
            pIp4AddrData = IP_IF_GetAddrByIf4(ifHandle);

            if (pIp4AddrData)
            {
                ipAddr_t inIpAddr;
                NWKU_ConvertIp4Addr(pIp4AddrData->ip4Addr, &inIpAddr);
                ntop(AF_INET, &inIpAddr, addrStr, INET_ADDRSTRLEN);
                shell_printf("\n\r\tIPv4 Address: %s", addrStr);
            }

#endif

            /* Get IPv6 addresses for an interface */
            for (iIpIdx = 0; iIpIdx < IP_IF_IP6_ADDR_NB; iIpIdx++)
            {
                /* Get IP address by index */
                /* Do not print anycast addresses */
                // pIp6AddrData = IP_IF_GetAddrByIf6(ifHandle->ifUniqueId, iIpIdx, FALSE);
                // TODO: Stack API

                if (pIp6AddrData)
                {
                    ntop(AF_INET6, &pIp6AddrData->ip6Addr, addrStr, INET6_ADDRSTRLEN);

                    if (((addrType == gMeshLocalAddr_c) || (addrType == gAllIpAddr_c)) &&
                            (IP6_IsUniqueLocalAddr(&pIp6AddrData->ip6Addr)))
                    {
                        thrPrefixAttr_t thrMLPrefix;

                        // (void)THR_GetAttr(threadInstanceID, THR_GetAttrConfirm_AttributeId_MLPrefix, 0, NULL, &(thrMLPrefix));
                        // TODO: Stack API

                        if (FLib_MemCmp(&pIp6AddrData->ip6Addr, &thrMLPrefix.prefix, 8))
                        {
                            shell_printf("\n\r\tMesh local address");

                            if (IP6_IsAddrEui64(&pIp6AddrData->ip6Addr))
                            {
                                shell_printf(" (ML64): %s", addrStr);
                            }
                            else
                            {
                                shell_printf(" (ML16): %s", addrStr);
                            }
                        }
                        else
                        {
                            shell_printf("\n\r\tUnique local address: %s", addrStr);
                        }
                    }

                    if (((addrType == gGlobalAddr_c) || (addrType == gAllIpAddr_c)) &&
                            (IP6_IsGlobalAddr(&pIp6AddrData->ip6Addr)))
                    {
                        shell_printf("\n\r\tGlobal address: %s", addrStr);
                    }

                    if (((addrType == gLinkLocalAddr_c) || (addrType == gAllIpAddr_c)) &&
                            (IP6_IsLinkLocalAddr(&pIp6AddrData->ip6Addr)))
                    {
                        if (IP6_IsAddrEui64(&pIp6AddrData->ip6Addr))
                        {
                            shell_printf("\n\r\tLink local address (LL64): %s", addrStr);
                        }
                        else
                        {
                            shell_printf("\n\r\tLink local address (LL16): %s", addrStr);
                        }
                    }
                }
            }

            /* Link-local all-Thread-nodes, Realm-Local All Thread Nodes */
            if (addrType == gAllIpAddr_c)
            {
                /* Link-local all-Thread-nodes */
                // if(IP_IF_IsMyMulticastGroupAddr6(ifHandle->ifUniqueId, (ipAddr_t *)&in6addr_linklocal_allthreadnodes))
                // TODO: Stack API
                {
                    ntop(AF_INET6, &in6addr_linklocal_allthreadnodes, addrStr, INET6_ADDRSTRLEN);
                    shell_printf("\n\r\tLink local all Thread Nodes(MCast):  %s", addrStr);
                }
                /* Realm-Local All Thread Nodes */
                // if(IP_IF_IsMyMulticastGroupAddr6(ifHandle->ifUniqueId, (ipAddr_t *)&in6addr_realmlocal_allthreadnodes))
                // TODO: Stack API
                {
                    ntop(AF_INET6, &in6addr_realmlocal_allthreadnodes, addrStr, INET6_ADDRSTRLEN);
                    shell_printf("\n\r\tRealm local all Thread Nodes(MCast): %s", addrStr);
                }
            }
        }
    }
}

/*!*************************************************************************************************
\fn     void SHELL_EvMonitorInit(instanceId_t instanceId)
\brief  This function is used as a greeting shell message.
***************************************************************************************************/
static void SHELL_EvMonitorInit()
{
    /* Init demo application  */
    shell_write("\n\rSHELL starting...\n");
    shell_write("\rEnter \"thr join\" to join a network, or \"thr create\" to start new network\n");
    shell_write("\rEnter \"help\" for other commands\n");
    shell_refresh();
}

#if THR_ENABLE_EVENT_MONITORING
/*!*************************************************************************************************
\fn     bool_t SHELL_EvMonitor(void *param)
\brief  This function is used to handle the events received from the Event monitor.

\param  [in]   param    Pointer to monitor event
***************************************************************************************************/
void SHELL_EvMonitor
(
    void *param
)
{
    eventMonitorData_t *pEventMonitorData = (eventMonitorData_t *)param;

    if (pEventMonitorData)
    {
        uint16_t attrPanId = THR_GetAttr_PanId(pEventMonitorData->instanceId);
        uint8_t attrCapabilities = THR_GetAttr_NwkCapabilities(pEventMonitorData->instanceId);
        bool_t printMeshLocal = FALSE;
#if !THREAD_ED_CONFIG
        uint8_t attrChannel = THR_GetAttr_Channel(pEventMonitorData->instanceId);
#endif
        uint32_t eventCode = ((pEventMonitorData->eventSet[0] << 16) + (pEventMonitorData->eventSet[1] << 24) +
                              (pEventMonitorData->eventStatus[0]) + (pEventMonitorData->eventStatus[1] << 8));
#if THR_ROUTING_ENABLE
        extern uint32_t gThrRouterUpgradeThreshold;
#endif

        switch (eventCode)
        {
            case gThrEv_NwkJoinInd_Attaching_c:
                if (APP_GetState(threadInstanceID) != gDeviceState_JoiningOrAttaching_c)
                {
                    shell_write("\rJoining a Thread network...\n\r");
                }

                break;

            case gThrEv_NwkJoinCnf_Success_c:
                shell_printf("\rAttached to network with PAN ID: 0x%x \n\r", attrPanId);

                /* Check if the node is polling ED or ED */
                if (attrCapabilities & 0x04)
                {
                    shell_write("\rNode started as End Device\n\r");
                    printMeshLocal = TRUE;
                }

                /* Check if REED or ED */
                if ((attrCapabilities == 0) || ((attrCapabilities & THR_NWKCAP_CAN_BECOME_ACTIVE_ROUTER)
#if THR_ROUTING_ENABLE
                                                && (gThrRouterUpgradeThreshold == 1)
#endif
                                               ))
                {
                    printMeshLocal = TRUE;
                }

                break;

            case gThrEv_NwkJoinCnf_Failed_c:
            case gThrEv_MeshCop_JoinerDtlsError_c:
            case gThrEv_MeshCop_JoinerError_c:
                shell_write("\rJoin failed\n\r");
                shell_refresh();
                break;

            case gThrEv_GeneralInd_Disconnected_c:
                break;

            case gThrEv_GeneralInd_RequestGlobalAddr_c:
                shell_write("\rRequesting Global Address...\n\r");
                break;

            case gThrEv_GeneralInd_GlobalAddrAssigned_c:
                shell_write("\rGlobal Address has been assigned");
                shell_refresh();
                break;

            case gThrEv_GeneralInd_ResetMcuTimeout_c:
                shell_printf("\rReset MCU in: %d milliseconds\n", APP_GetResetMcuTimeout());
                break;
#if !THREAD_ED_CONFIG

            case gThrEv_GeneralInd_DeviceIsLeader_c:
                shell_write("\rNode has taken the Leader role");
                shell_refresh();
                break;

            case gThrEv_NwkCreateCnf_Success_c:
                shell_printf("\rCreated a new Thread network on channel %d and PAN ID:0x%x\n\r", attrChannel, attrPanId);
                printMeshLocal = TRUE;
                break;

            case gThrEv_GeneralInd_RequestRouterId_c:
                shell_write("\rRequesting to become Active Router...\n\r");
                break;

            case gThrEv_GeneralInd_RouterIdAssigned_c:
                shell_write("\rSuccess\n\r");
                printMeshLocal = TRUE;
                break;

            case gThrEv_MeshCop_CommissionerPetitionAccepted_c:
                shell_write("\r(Local) Commissioner Started\n\r");
                shell_refresh();
                break;

            case gThrEv_MeshCop_CommissionerJoinerAccepted_c:
                shell_write("\rJoiner accepted\n\r");
                shell_refresh();
                break;

            case gThrEv_MeshCop_JoinerAccepted_c:
                shell_write("\rCommissioning successful\n\r");
                shell_refresh();
                break;
#endif

            default:
                break;
        }

        if (printMeshLocal)
        {
            SHELL_PrintIpAddr(gMeshLocalAddr_c);
            shell_refresh();
        }

        MEM_BufferFree(pEventMonitorData);
    }
}
#endif /* THR_ENABLE_EVENT_MONITORING */

/*!*************************************************************************************************
\private
\fn     void SHELL_NwkScanPrint(THR_EventNwkScanConfirm_t *pScanResults)
\brief  This function is used to network scan information.

\param  [in]    pScanResults   The scan results
***************************************************************************************************/
static void SHELL_NwkScanPrint
(
    THR_EventNwkScanConfirm_t *pScanResults
)
{
    SHELL_NEWLINE();

    /* Handle the network scan result here */
    if (pScanResults)
    {
        shell_printf("Scan duration: %d, Scan mask: %04X\n\r", pScanResults->ScanDuration, pScanResults->ScanChannelMask);

        if (pScanResults->ScanType == THR_EventNwkScanConfirm_ScanType_EnergyDetect)
        {
            shell_write("\n\rEnergy on channel 11 to 26: 0x");
            SHELL_PrintBuff(pScanResults->Data.EnergyDetect.EnergyDetectList,
                            pScanResults->Data.EnergyDetect.EnergyDetectEntries);
            shell_printf("\n\r");

            MEM_BufferFree(pScanResults->Data.EnergyDetect.EnergyDetectList);
        }

        if (pScanResults->ScanType == THR_EventNwkScanConfirm_ScanType_ActiveScan)
        {
            for (uint8_t i = 0; i < pScanResults->Data.ActiveScan.NwkDiscoveryEntries; i++)
            {
                shell_printf("\rThread Network: %d\n\r", i);
                shell_printf("\r\tPAN ID: 0x%x \n\r",           pScanResults->Data.ActiveScan.NwkDiscoveryList[i].PanId);
                shell_printf("\r\tChannel: %d \n\r",            pScanResults->Data.ActiveScan.NwkDiscoveryList[i].Channel);
                shell_printf("\r\tLQI: %d\n\r",                 pScanResults->Data.ActiveScan.NwkDiscoveryList[i].Reserved);
                shell_printf("\r\tReceived beacons: %d\n\r",    pScanResults->Data.ActiveScan.NwkDiscoveryList[i].NumOfRcvdBeacons);
            }

            MEM_BufferFree(pScanResults->Data.ActiveScan.NwkDiscoveryList);
        }

        if (pScanResults->ScanType == THR_EventNwkScanConfirm_ScanType_EnergyDetectAndActiveScan)
        {
            shell_write("\n\rEnergy on channel 11 to 26: 0x");
            SHELL_PrintBuff(pScanResults->Data.EnergyDetectAndActiveScan.EnergyDetectList,
                            pScanResults->Data.EnergyDetectAndActiveScan.EnergyDetectEntries);
            shell_printf("\n\r");
            MEM_BufferFree(pScanResults->Data.EnergyDetectAndActiveScan.EnergyDetectList);

            for (uint8_t i = 0; i < pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryEntries; i++)
            {
                shell_printf("\rThread Network: %d\n\r", i);
                shell_printf("\r\tPAN ID: 0x%x \n\r",           pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryList[i].PanId);
                shell_printf("\r\tChannel: %d \n\r",            pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryList[i].Channel);
                shell_printf("\r\tLQI: %d\n\r",                 pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryList[i].Reserved);
                shell_printf("\r\tReceived beacons: %d\n\r",    pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryList[i].NumOfRcvdBeacons);
            }

            MEM_BufferFree(pScanResults->Data.EnergyDetectAndActiveScan.NwkDiscoveryList);
        }

        shell_refresh();
    }
}

/*==================================================================================================
Private functions
==================================================================================================*/

/*!*************************************************************************************************
\private
\fn     static void SHELL_Resume(void)
\brief  This function is used to resume SHELL execution after a pause was issued.
***************************************************************************************************/
static void SHELL_Resume
(
    void
)
{
#if SOCK_DEMO
    App_ShellSocketResume();
#endif

    /* Stop Ping timer */
    if (pingTimerID != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(pingTimerID);
        pingTimerID = gTmrInvalidTimerID_c;
    }

    mContinuousPing = FALSE;

    if (mTimerIDSelect != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(mTimerIDSelect);
        mTimerIDSelect = gTmrInvalidTimerID_c;
    }

    if (mDelayTimerID != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(mDelayTimerID);
        mDelayTimerID = gTmrInvalidTimerID_c;
    }

    if (pSrcIpAddr)
    {
        MEM_BufferFree(pSrcIpAddr);
        pSrcIpAddr = NULL;
    }

    shell_refresh();
}

/*!*************************************************************************************************
\fn     static void SHELL_Identify(void* param)
\brief  This function is used to start the board identification by flashing the TX LED.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Identify
(
    uint8_t argc,
    char *argv[]
)
{
    THR_IdentifyRequest(THR_FSCI_IF);

    return CMD_RET_SUCCESS;
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_Process(void *param)
\brief  This function is used to process the input command.

\param  [in]   param    Pointer to input
***************************************************************************************************/
static void SHELL_Process
(
    void *param
)
{
    extern int8_t (*mpfShellBreak)(uint8_t argc, char *argv[]);
    uint8_t argc;
    char *argv[SHELL_MAX_ARGS + 1];    /* NULL terminated  */
    cmd_tbl_t *cmdtp;
    int16_t ret = CMD_RET_FAILURE;

    // Split command into arguments
    argc = make_argv((char *)param, SHELL_MAX_ARGS + 1, argv);

    if (argc >= SHELL_MAX_ARGS)
    {
        shell_write("** Too many args (max. ");
        shell_writeDec(SHELL_MAX_ARGS);
        shell_write(") **\r\n");
    }

    // Search for the appropriate command
    cmdtp = shell_find_command(argv[0]);

    if ((cmdtp != NULL) && (cmdtp->cmd != NULL))
    {
        if (argc > cmdtp->maxargs)
        {
            ret = CMD_RET_USAGE;
        }
        else
        {
            ret = (cmdtp->cmd)(argc, argv);
        }
    }
    else
    {
        shell_write("Unknown command '");
        shell_write(argv[0]);
#if SHELL_USE_HELP
        shell_write("' - try 'help'\r\n");
#else
        shell_write("' ");
#endif
        ret = CMD_RET_FAILURE;
    }

#if SHELL_USE_HELP

    if (ret == CMD_RET_USAGE)
    {
        if (cmdtp->usage != NULL)
        {
            shell_write(cmdtp->name);
            shell_writeN(" - ", 3);
            shell_write(cmdtp->usage);
            SHELL_NEWLINE();
        }

        if (cmdtp->help != NULL)
        {
            shell_write("Usage:\r\n");
            shell_write(cmdtp->name);
            shell_writeN(" ", 1);
            shell_write(cmdtp->help);
            SHELL_NEWLINE();
        }
        else
        {
            shell_write("- No additional help available.\r\n");
        }
    }

#endif

    if (ret == CMD_RET_ASYNC)
    {
        mpfShellBreak = cmdtp->cmd;
    }
    else
    {
        mpfShellBreak = NULL;
        shell_refresh();
    }

    MEM_BufferFree(param);
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_ProcessCb(char *pCmd, uint16_t length)
\brief  This function is used as a callback function for inputting commands.

\param  [in]    pCmd      Pointer to the command
\param  [in]    length    Length of the command
***************************************************************************************************/
static void SHELL_ProcessCb
(
    char *pCmd,
    uint16_t length
)
{
    if ((NULL == pCmd) && (0 == length))
    {
        SHELL_InterruptCb();
    }
    else
    {
        uint8_t *pData = MEM_BufferAlloc(length + 1);

        if (NULL != pData)
        {
            FLib_MemSet(pData, 0, length + 1);
            FLib_MemCpy(pData, pCmd, length);

            SHELL_Process(pData);
            // if (FALSE == NWKU_SendMsg(SHELL_Process, pData, pmMainThreadMsgQueue))
            // {
            //     MEM_BufferFree(pData);
            // }
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_InterruptCb(void)
\brief  This function is used to intrerupt the ping or the board identification on user request.
***************************************************************************************************/
static void SHELL_InterruptCb
(
    void
)
{
    gbRetryInterrupt = FALSE;
    mContinuousPing = FALSE;
#if ECHO_PROTOCOL
    mEchoUdpCounter = 0;
#endif
}

/*!************************************************************************************************
\private
\fn     static void SHELL_ThrCommands(uint8_t argc, char *argv[])
\brief  This function is used to send commands for Thread network.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_ThrCommands
(
    uint8_t argc,
    char *argv[]
)
{
    thrStatus_t status = gThrStatus_EntryNotFound_c;

    if (!strcmp(argv[1], "create"))
    {
        shell_write("\rCreating network...");
        SHELL_ThrNwkCreate(NULL);
    }

    else if (!strcmp(argv[1], "join"))
    {
        shell_write("\rJoining network...");
        SHELL_ThrNwkJoin(NULL);
    }

    else if (!strcmp(argv[1], "detach"))
    {
        shell_write("\rDetaching from network...\r\n");

        THR_DisconnectRequest_t req;
        req.InstanceID = threadInstanceID;

        if (THR_DisconnectRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
        {
            status = gThrStatus_NoMem_c;
        }
        else
        {
            status = gThrStatus_Success_c;
        }
    }

    else if (!strcmp(argv[1], "commissioner"))
    {
        if (!strcmp(argv[2], "start"))
        {
            SHELL_ThrStartCommissioner(NULL);
        }
        else if (!strcmp(argv[2], "stop"))
        {
            SHELL_ThrStopCommissioner(NULL);
        }
    }

    else if (!strcmp(argv[1], "joiner"))
    {
        if (!strcmp(argv[2], "add"))
        {
            uint8_t eui[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            int32_t pskLen = strlen(argv[3]);
            uint8_t psk[32];

            FLib_MemSet(&psk, 0, 32);
            FLib_MemCpy(&psk, argv[3], pskLen);

            if (NULL != argv[4])
            {
                FLib_MemSet(&eui, 0, 8);
                SHELL_ReadBuff(argv[4] + 2, eui, 8);
            }

            MESHCOP_AddExpectedJoinerRequest_t req;
            req.InstanceId = threadInstanceID;
            req.Selected = TRUE;
            req.EuiType = MESHCOP_AddExpectedJoinerRequest_EuiType_LongEUI;
            req.EUI.LongEUI = NWKU_TransformArrayToUint64(eui);
            req.PSKdSize = pskLen;
            req.PSKd = (char *)psk;

            if (MESHCOP_AddExpectedJoinerRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
            {
                status = gThrStatus_NoMem_c;
            }
            else
            {
                status = gThrStatus_Success_c;
            }
        }

        else if (!strcmp(argv[2], "remove"))
        {
            uint8_t eui[8];

            FLib_MemSet(&eui, 0, 8);

            if (NULL != argv[3])
            {
                SHELL_ReadBuff(argv[3] + 2, eui, 8);
            }

            MESHCOP_RemoveExpectedJoinerRequest_t req;
            req.InstanceId = threadInstanceID;
            req.EuiType = MESHCOP_RemoveExpectedJoinerRequest_EuiType_LongEUI;
            req.EUI.LongEUI = NWKU_TransformArrayToUint64(eui);

            if (MESHCOP_RemoveExpectedJoinerRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
            {
                status = gThrStatus_NoMem_c;
            }
            else
            {
                status = gThrStatus_Success_c;
            }
        }

        else if (!strcmp(argv[2], "removeall"))
        {
            MESHCOP_RemoveAllExpectedJoinersRequest_t req;
            req.InstanceId = threadInstanceID;

            if (MESHCOP_RemoveAllExpectedJoinersRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
            {
                status = gThrStatus_NoMem_c;
            }
            else
            {
                status = gThrStatus_Success_c;
            }
        }
    }

    else if (!strcmp(argv[1], "sync"))
    {
        if (!strcmp(argv[2], "steering"))
        {
            MESHCOP_SyncSteeringDataRequest_t req;
            req.InstanceId = threadInstanceID;

            if (argv[3] && !strcmp(argv[3], "all"))
            {
                req.EuiMask = MESHCOP_SyncSteeringDataRequest_EuiMask_AllFFs;
            }
            else
            {
                req.EuiMask = MESHCOP_SyncSteeringDataRequest_EuiMask_ExpectedJoiners;
            }

            if (MESHCOP_SyncSteeringDataRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
            {
                status = gThrStatus_NoMem_c;
            }
            else
            {
                status = gThrStatus_Success_c;
            }
        }

        else if (!strcmp(argv[2], "nwkdata"))
        {
            THR_BrPrefixSyncRequest_t req;
            req.InstanceId = threadInstanceID;

            if (THR_BrPrefixSyncRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
            {
                status = gThrStatus_NoMem_c;
            }
            else
            {
                status = gThrStatus_Success_c;
            }
        }
    }

    else if (!strcmp(argv[1], "scan"))
    {
        THR_NwkScanRequest_t req;
        req.InstanceID = threadInstanceID;
        req.ScanChannelMask = 0x07FFF800;   // all channels
        req.ScanDuration = 2;               // (Range:1 - 14)
        req.maxThrNwkToDiscover = 10;       // maximum thread network to be discovered

        if (!strcmp(argv[2], "energy"))
        {
            req.ScanType = THR_NwkScanRequest_ScanType_EnergyDetect;
        }
        else if (!strcmp(argv[2], "active"))
        {
            req.ScanType = THR_NwkScanRequest_ScanType_ActiveScan;
        }
        else if (!strcmp(argv[2], "both"))
        {
            req.ScanType = THR_NwkScanRequest_ScanType_EnergyDetectAndActiveScan;
        }
        else
        {
            req.ScanType = THR_NwkScanRequest_ScanType_ActiveScan;
        }

        THR_NwkScanRequest(&req, THR_FSCI_IF);
    }

    else if (!strcmp(argv[1], "nwkdata"))
    {
        char *pValue;
        ipAddr_t prefix;
        uint8_t prefixLength = 64;
        uint32_t lifetime = 0;  //gThrNwkDataMinStableLifetime;

        /* Get prefix */
        pValue = shell_get_opt(argc, argv, "-p");

        if (pValue)
        {
            pton(AF_INET6, pValue, &prefix);
        }

        /* Get prefix length */
        pValue = shell_get_opt(argc, argv, "-len");

        if (pValue)
        {
            prefixLength = (uint8_t)NWKU_atoi(pValue);
        }

        /* Get prefix lifetime */
        pValue = shell_get_opt(argc, argv, "-t");

        if (pValue)
        {
            lifetime = NWKU_atoi(pValue);
        }

        if (!strcmp(argv[2], "add"))
        {
            THR_BrPrefixAddEntryRequest_t req = { 0 };
            req.InstanceId = threadInstanceID;
            req.prefixLength = prefixLength;
            FLib_MemCpy(req.PrefixValue, prefix.addr8, sizeof(ipAddr_t));
            req.prefixLifetime = lifetime;

            pValue = shell_get_opt(argc, argv, "dhcpserver");

            if (pValue)
            {
                THR_BR_FLAGS_SET(req.PrefixFlags, 1, THR_BR_PREFIX_FLAGS_P_DHCP_MASK, THR_BR_PREFIX_FLAGS_P_DHCP_OFFSET);
                THR_BR_FLAGS_SET(req.PrefixFlags, 1, THR_BR_PREFIX_FLAGS_P_ON_MESH_MASK, THR_BR_PREFIX_FLAGS_P_ON_MESH_OFFSET);
                req.prefixLifetime = lifetime;
                req.prefixAdvertised = TRUE;
            }

            pValue = shell_get_opt(argc, argv, "slaac");

            if (pValue)
            {
                THR_BR_FLAGS_SET(req.PrefixFlags, 1, THR_BR_PREFIX_FLAGS_P_SLAAC_MASK, THR_BR_PREFIX_FLAGS_P_SLAAC_OFFSET);
                THR_BR_FLAGS_SET(req.PrefixFlags, 1, THR_BR_PREFIX_FLAGS_P_ON_MESH_MASK, THR_BR_PREFIX_FLAGS_P_ON_MESH_OFFSET);
                req.prefixLifetime = lifetime;
                req.prefixAdvertised = TRUE;
            }

            pValue = shell_get_opt(argc, argv, "extroute");

            if (pValue)
            {
                THR_BR_FLAGS_SET(req.ExternalRouteFlags, 1, THR_BR_EXT_ROUTE_FLAGS_R_PREF_MASK, THR_BR_EXT_ROUTE_FLAGS_R_PREF_OFFSET);
                req.ExternalRouteAdvertised = TRUE;
                req.ExternalRouteLifetime = lifetime;
            }

            THR_BrPrefixAddEntryRequest(&req, THR_FSCI_IF);

#if DNS_ENABLED
            pValue = shell_get_opt(argc, argv, "dnsserver");

            if (pValue)
            {
                thrLocalServiceSet_t thrServerSet = {.thrSenterpriseNumber = THREAD_ENTERPRISE_NUMBER_ARRAY,
                                                     .thrSserviceDataLen   = 9,
                                                     .thrSserviceData      = "dnsserver",
                                                     .thrSserver16Addr     = {0xFF, 0xFF},
                                                     .thrSserverDataLen    = 0,
                                                     .thrSserverData       = {0},
                                                     .thrSserviceId        = 0,
                                                     .thrSstable           = TRUE
                                                    };

                thrServerSet.thrSserverDataLen = 16;
                pton(AF_INET6, argv[4], (ipAddr_t *)&thrServerSet.thrSserverData);
                // status = THR_ServiceAttrAddEntry(threadInstanceID, &thrServerSet);
            }

#endif

        }
        else if (!strcmp(argv[2], "remove"))
        {

            if (!strcmp(argv[3], "dnsserver"))
            {
                thrLocalServiceSet_t thrServerSet = {.thrSserviceDataLen   = 9,
                                                     .thrSserviceData      = "dnsserver",
                                                     .thrSserverDataLen    = 0,
                                                     .thrSserverData       = {0}
                                                    };
                pton(AF_INET6, argv[4], (ipAddr_t *)&thrServerSet.thrSserverData);

                // status = THR_BrServiceAttrRemoveEntry(threadInstanceID,
                // (uint8_t *)&thrServerSet.thrSserviceData, thrServerSet.thrSserviceDataLen,
                // (uint8_t *)&thrServerSet.thrSserverData, thrServerSet.thrSserverDataLen);
            }
            else
            {
                THR_BrPrefixRemoveEntryRequest_t req;
                req.InstanceId = threadInstanceID;
                req.prefixLength = prefixLength;
                FLib_MemCpy(req.PrefixValue, prefix.addr8, sizeof(ipAddr_t));

                THR_BrPrefixRemoveEntryRequest(&req, THR_FSCI_IF);
            }
        }
        else if (!strcmp(argv[2], "removeall"))
        {
            THR_BrPrefixRemoveAllRequest_t req;
            req.InstanceId = threadInstanceID;

            THR_BrPrefixRemoveAllRequest(&req, THR_FSCI_IF);
        }
    }

    else if (!strcmp(argv[1], "get"))
    {
        uint32_t i;

        if (!strcmp(argv[2], "attributes"))
        {
            shell_printf("Thread network attributes:\n\r");

            for (i = 0; i < NumberOfElements(aShellThrAttr); i++)
            {
                shell_printf("\t%s - %s\n\r", aShellThrAttr[i].pAttrName, aShellThrAttr[i].writable ? "RW" : "RO");
            }
        }

        else if (!strcmp(argv[2], "neighbor"))
        {
            if (argv[3])
            {
                THR_GetNeighborInfoRequest_t req;
                req.ShortAddress = NWKU_AsciiToHex((uint8_t *)(argv[3] + 2), FLib_StrLen(argv[3] + 2));
                req.InstanceId = threadInstanceID;
                THR_GetNeighborInfoRequest(&req, THR_FSCI_IF);
            }
            else
            {
                shell_write("\n\rPlease provide the short address of the neighbor!");
            }

        }

        else if (!strcmp(argv[2], "neighbors"))
        {
            THR_GetNeighborTableRequest_t req;
            req.InstanceId = threadInstanceID;
            req.StartIndex = 0;
            req.NoOfElements = THR_MAX_NEIGHBORS;
            (void)THR_GetNeighborTableRequest(&req, THR_FSCI_IF);
        }

        else if (!strcmp(argv[2], "routes"))
        {
            THR_GetRoutingTableRequest_t req;
            req.InstanceId = threadInstanceID;
            req.StartIndex = 0;
            req.NoOfElements = 10;
            (void)THR_GetRoutingTableRequest(&req, THR_FSCI_IF);
        }

        else if (!strcmp(argv[2], "parent"))
        {
            THR_GetParentRequest_t req;
            req.InstanceId = threadInstanceID;
            (void)THR_GetParentRequest(&req, THR_FSCI_IF);
        }

        else
        {
            for (i = 0; i < NumberOfElements(aShellThrAttr); i++)
            {
                if (!strcmp(argv[2], aShellThrAttr[i].pAttrName))
                {
                    THR_GetAttrRequest_t req;
                    req.InstanceId = threadInstanceID;
                    req.AttributeId = (THR_GetAttrRequest_AttributeId_t)(aShellThrAttr[i].attrId);
                    req.Index = 0;

                    (void)THR_GetAttrRequest(&req, THR_FSCI_IF);

                    break;
                }
            }

            if (i == NumberOfElements(aShellThrAttr))
            {
                shell_write("Unknown parameter!");
            }
        }
    }

    else if (!strcmp(argv[1], "set"))
    {
        uint32_t i;
        THR_SetAttrRequest_t req;
        req.InstanceId = threadInstanceID;
        req.Index = 0;

        for (i = 0; i < NumberOfElements(aShellThrAttr); i++)
        {
            if (!strcmp(argv[2], aShellThrAttr[i].pAttrName))
            {
                if (!aShellThrAttr[i].writable)
                {
                    shell_write("Entry is read only!");
                    break;
                }

                req.AttributeId = (THR_SetAttrRequest_AttributeId_t)aShellThrAttr[i].attrId;

                switch (aShellThrAttr[i].valueType)
                {
                    case gDecimal_c:
                    {
                        int64_t pValue = NWKU_atol(argv[3]);

                        if (THR_GetAttrConfirm_AttributeId_ActiveTimestamp == aShellThrAttr[i].attrId)
                        {
                            /* Ticks are not included. Add 0 for ticks */
                            pValue = pValue << 16;
                        }

                        FLib_MemCpy((uint8_t *)&req.AttributeValue, (void *)&aShellThrAttr[i].maxSize, 1);
                        FLib_MemCpy((uint8_t *)&req.AttributeValue + 1, (void *)&pValue, aShellThrAttr[i].maxSize);
                    }

                    break;

                    case gHex_c:
                    {
                        uint32_t value = NWKU_AsciiToHex((uint8_t *)(argv[3] + 2), FLib_StrLen(argv[3] + 2));

                        FLib_MemCpy((uint8_t *)&req.AttributeValue, (void *)&aShellThrAttr[i].maxSize, 1);
                        FLib_MemCpy((uint8_t *)&req.AttributeValue + 1, (void *)&value, aShellThrAttr[i].maxSize);
                    }
                    break;

                    case gString_c:
                    {
                        uint8_t pValue[32];
                        int32_t len = strlen(argv[3]);
                        FLib_MemCpy(&pValue, argv[3], len);

                        if (THR_GetAttrConfirm_AttributeId_Security_PSKc == aShellThrAttr[i].attrId)
                        {
                            shell_write("The PSKc is derived from the PSKd on the black-box.");
                        }

                        else if (THR_GetAttrConfirm_AttributeId_DeviceRole == aShellThrAttr[i].attrId)
                        {
                            THR_SetAttrRequest_AttributeValue_DeviceRole_Value_t devRole = THR_SetAttrRequest_AttributeValue_DeviceRole_Value_RouterEligibleEndDevice;

                            if (FLib_MemCmp(pValue, "SED", 3))
                            {
                                devRole = THR_SetAttrRequest_AttributeValue_DeviceRole_Value_SleepyEndDevice;
                            }
                            else if (FLib_MemCmp(pValue, "FED", 3))
                            {
                                devRole = THR_SetAttrRequest_AttributeValue_DeviceRole_Value_FullEndDevice;
                            }
                            else if (FLib_MemCmp(pValue, "MED", 3))
                            {
                                devRole = THR_SetAttrRequest_AttributeValue_DeviceRole_Value_MinimalEndDevice;
                            }

                            req.AttributeValue.DeviceRole.AttrSize = 1;
                            req.AttributeValue.DeviceRole.Value = devRole;
                        }

                        else
                        {
                            /* writable string attributes */
                            if (THR_GetAttrConfirm_AttributeId_NwkName == aShellThrAttr[i].attrId)
                            {
                                req.AttributeValue.NwkName.AttrSize = len;
                                req.AttributeValue.NwkName.Value = MEM_BufferAlloc(len);
                                FLib_MemCpy(req.AttributeValue.NwkName.Value, pValue, len);
                            }
                            else if (THR_GetAttrConfirm_AttributeId_ProvisioningURL == aShellThrAttr[i].attrId)
                            {
                                req.AttributeValue.ProvisioningURL.AttrSize = len;
                                req.AttributeValue.ProvisioningURL.Value = MEM_BufferAlloc(len);
                                FLib_MemCpy(req.AttributeValue.ProvisioningURL.Value, pValue, len);
                            }
                            else if (THR_GetAttrConfirm_AttributeId_Security_PSKd == aShellThrAttr[i].attrId)
                            {
                                req.AttributeValue.Security_PSKd.AttrSize = len;
                                req.AttributeValue.Security_PSKd.Value = MEM_BufferAlloc(len);
                                FLib_MemCpy(req.AttributeValue.Security_PSKd.Value, pValue, len);
                            }
                        }
                    }
                    break;

                    case gArray_c:
                    {
                        uint8_t pValue[16];
                        SHELL_ReadBuff(argv[3] + 2, pValue, aShellThrAttr[i].maxSize);

                        /* writable array attributes */
                        if (THR_GetAttrConfirm_AttributeId_Security_NwkMasterKey == aShellThrAttr[i].attrId)
                        {
                            req.AttributeValue.Security_NwkMasterKey.AttrSize = aShellThrAttr[i].maxSize;
                            req.AttributeValue.Security_NwkMasterKey.Value = MEM_BufferAlloc(aShellThrAttr[i].maxSize);
                            FLib_MemCpy(req.AttributeValue.Security_NwkMasterKey.Value, pValue, aShellThrAttr[i].maxSize);
                        }
                        else if (THR_GetAttrConfirm_AttributeId_MLPrefix == aShellThrAttr[i].attrId)
                        {
                            req.AttributeValue.MLPrefix.AttrSize = aShellThrAttr[i].maxSize;
                            req.AttributeValue.MLPrefix.Value.PrefixLength = aShellThrAttr[i].maxSize;
                            FLib_MemCpy(req.AttributeValue.MLPrefix.Value.PrefixData, pValue, aShellThrAttr[i].maxSize);
                        }
                        else if (THR_GetAttrConfirm_AttributeId_ExtendedPanId == aShellThrAttr[i].attrId)
                        {
                            req.AttributeValue.ExtendedPanId.AttrSize = aShellThrAttr[i].maxSize;
                            req.AttributeValue.ExtendedPanId.Value = MEM_BufferAlloc(aShellThrAttr[i].maxSize);
                            FLib_MemCpy(req.AttributeValue.ExtendedPanId.Value, pValue, aShellThrAttr[i].maxSize);
                        }
                    }
                    break;

                    case gHexReversed_c:
                    {
                        uint8_t pValue[8];
                        SHELL_ReadBuff(argv[3] + 2, pValue, 8);
                        NWKU_SwapArrayBytes(pValue, 8);

                        FLib_MemCpy((uint8_t *)&req.AttributeValue, (void *)&aShellThrAttr[i].maxSize, 1);
                        FLib_MemCpy((uint8_t *)&req.AttributeValue + 1, pValue, aShellThrAttr[i].maxSize);
                    }
                    break;

                    default:
                        break;
                }

                THR_SetAttrRequest(&req, THR_FSCI_IF);
                break;
            }

        }

        if (i == NumberOfElements(aShellThrAttr))
        {
            shell_write("Unknown parameter!");
        }
    }

    else if (!strcmp(argv[1], "remove"))
    {
        if (!strcmp(argv[2], "router"))
        {
            uint16_t shortAddress = NWKU_AsciiToHex((uint8_t *)(argv[3] + 2), FLib_StrLen(argv[3] + 2));

            THR_LeaderRemoveRouterIdRequest_t req;
            req.InstanceId = threadInstanceID;
            req.RouterShortAddr = shortAddress;

            (void)THR_LeaderRemoveRouterIdRequest(&req, THR_FSCI_IF);
        }
    }

#if SHELL_DUT_COMMISSIONER
    else if (!strcmp(argv[1], "mgmt"))
    {

        if (!strcmp(argv[2], "gettlvs"))
        {
            for (uint32_t iCount = 0; iCount < NumberOfElements(aShellThrMgmtTlvs); iCount++)
            {
                shell_printf("%s\r\n", aShellThrMgmtTlvs[iCount].pTlvName);
            }
        }
        else
        {
            ipAddr_t destIpAddr = {0};

            bool_t set = FALSE;
            bool_t get = FALSE;
            uint8_t *pTlvBuff = NULL;
            uint32_t maxBuffLen = 0;
            uint32_t tlvBuffLen = 0;
            meshcopMgmtParams_t mgmtParams = {0};
            nwkStatus_t nwkStatus = gNwkStatusFail_c;

            if (argv[3] != NULL)
            {
                pton(AF_INET6, argv[3], &destIpAddr);
            }

            if ((!strcmp(argv[2], "activeset")) || (!strcmp(argv[2], "pendset")) || (!strcmp(argv[2], "commset")))
            {
                set = TRUE;

                for (uint32_t iCount = 0; iCount < NumberOfElements(aShellThrMgmtTlvs); iCount++)
                {
                    maxBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                }
            }
            else if ((!strcmp(argv[2], "activeget")) || (!strcmp(argv[2], "pendget")) || (!strcmp(argv[2], "commget")))
            {
                get = TRUE;
                maxBuffLen = NumberOfElements(aShellThrMgmtTlvs);
            }
            else if (!strcmp(argv[2], "query"))
            {
                char *pValue = NULL;
                uint8_t channelMask[4] = {0};
                uint16_t pan = 0;

                pValue = shell_get_opt(argc, argv, "chmsk");

                if (pValue)
                {
                    SHELL_ReadBuff(pValue + 2, channelMask, sizeof(uint32_t));
                }

                pValue = shell_get_opt(argc, argv, "pan");

                if (pValue)
                {
                    pan = (uint16_t)NWKU_AsciiToHex((uint8_t *)(pValue + 2), FLib_StrLen(pValue + 2));
                }

                nwkStatus = MESHCOP_MgmtSendPanIdQuery(threadInstanceID, ntohal(channelMask), pan,
                                                       SHELL_MeshcopMgmtPanIdConflictCb, &destIpAddr);

            }

            if (get || set)
            {
                pTlvBuff = MEM_BufferAlloc(maxBuffLen);

                if (pTlvBuff == NULL)
                {
                    shell_write("No memory!");
                    return CMD_RET_SUCCESS;
                }

                FLib_MemSet(pTlvBuff, 0, maxBuffLen);
                SHELL_ReadMgmtTlv(argc, argv, pTlvBuff, &tlvBuffLen, set);

                mgmtParams.pDstIpAddr = &destIpAddr;
                mgmtParams.pTlvs = pTlvBuff;
                mgmtParams.tlvsLength = tlvBuffLen;
                mgmtParams.thrInstId = threadInstanceID;

                if (mgmtParams.tlvsLength == 0)
                {
                    mgmtParams.pTlvs = NULL;
                }

                if (set)
                {
                    mgmtParams.pfCb = SHELL_MeshcopMgmtSetCb;

                    if (!strcmp(argv[2], "activeset"))
                    {
                        nwkStatus = MESHCOP_MgmtActiveSet(&mgmtParams);
                    }

                    if (!strcmp(argv[2], "commset"))
                    {
                        nwkStatus = MESHCOP_MgmtCommSet(&mgmtParams);
                    }

                    if (!strcmp(argv[2], "pendset"))
                    {
                        nwkStatus = MESHCOP_MgmtPendingSet(&mgmtParams);
                    }
                }

                if (get)
                {
                    mgmtParams.pfCb = SHELL_MeshcopMgmtGetCb;

                    if (!strcmp(argv[2], "activeget"))
                    {
                        nwkStatus = MESHCOP_MgmtActiveGet(&mgmtParams);
                    }

                    if (!strcmp(argv[2], "commget"))
                    {
                        nwkStatus = MESHCOP_MgmtCommGet(&mgmtParams);
                    }

                    if (!strcmp(argv[2], "pendget"))
                    {
                        nwkStatus = MESHCOP_MgmtPendingGet(&mgmtParams);
                    }
                }
            }

            if (nwkStatus == gNwkStatusSuccess_c)
            {
                status = gThrStatus_Success_c;
            }
            else
            {
                status = gThrStatus_Failed_c;
            }

            MEM_BufferFree(pTlvBuff);
        }



    }

#endif
    else
    {
        shell_write("Unknown command!");
    }

    if (status != gThrStatus_EntryNotFound_c)
    {
        SHELL_PrintStatus(status);
    }

    return CMD_RET_SUCCESS;
}
#if SHELL_DUT_COMMISSIONER
/*!************************************************************************************************
\private
\fn     static void SHELL_ReadMgmtTlv(uint8_t argc, char *argv[], uint8_t *pTlvBuff,
                                    uint32_t* pTlvBuffLen,    bool_t bTlvSet)
\brief  This function is to read the TLV's values from terminal, process and copy them in a memory
buffer.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments
\param  [in]    pTlvBuff  Pointer to the start of the memory buffer.
\param  [in]    bTlvSet   TRUE - if it is a set command (i.e all TLV names are followed by a value)
                          FALSE - if it is a get command (all arguments are TLV names)
\param  [out]    pTlvBuffLen    Length of the output buffer

\return         none
***************************************************************************************************/
static void SHELL_ReadMgmtTlv
(
    uint8_t argc,
    char *argv[],
    uint8_t *pTlvBuff,
    uint32_t *pTlvBuffLen,
    bool_t bTlvSet
)
{
    uint8_t argNbTlv = 4;
    uint8_t *pCrtPos = pTlvBuff;
    uint32_t tlvBuffLen = 0;

    while ((argv[argNbTlv] != NULL) && (argNbTlv < argc))
    {
        for (uint32_t iCount = 0; iCount < NumberOfElements(aShellThrMgmtTlvs); iCount++)
        {
            if (!strcmp(argv[argNbTlv], aShellThrMgmtTlvs[iCount].pTlvName))
            {
                *pCrtPos++ = aShellThrMgmtTlvs[iCount].tlvId;

                if (bTlvSet)
                {
                    *pCrtPos++ = aShellThrMgmtTlvs[iCount].tlvLen;
                    argNbTlv++;

                    switch (aShellThrMgmtTlvs[iCount].tlvId)
                    {
                        case gMeshCopTlvChannel_c:
                        {
                            meshCopChannelTlv_t channelTlv = {0};
                            uint16_t value = NWKU_atoi(argv[argNbTlv]);
                            channelTlv.channelPage = 0;
                            htonas(channelTlv.channel, value);
                            FLib_MemCpy(pCrtPos, (uint8_t *)&channelTlv + MLE_TLV_HEADER_SIZE, aShellThrMgmtTlvs[iCount].tlvLen);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;

                        }
                        break;

                        case gMeshCopTlvPanID_c:
                        case gMeshCopTlvBorderRouterLoc_c:
                        case gMeshCopTlvCommSessId_c:
                        {
                            uint16_t value = 0;
                            value = (uint16_t)NWKU_AsciiToHex((uint8_t *)(argv[argNbTlv] + 2), FLib_StrLen(argv[argNbTlv] + 2));
                            htonas(pCrtPos, value);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvXpanID_c:
                        case gMeshCopTlvNwkMlUla_c:
                        {
                            SHELL_ReadBuff(argv[argNbTlv] + 2, pCrtPos, aShellThrMgmtTlvs[iCount].tlvLen);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvNwkName_c:
                        {
                            int32_t len = strlen(argv[argNbTlv]);
                            pCrtPos--;
                            *pCrtPos++ = len;
                            FLib_MemCpy(pCrtPos, argv[argNbTlv], len);
                            pCrtPos += len;
                            tlvBuffLen += len + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvNwkMasterKey_c:
                        case gMeshCopTlvPskc_c:
                        {
                            SHELL_ReadBuff(argv[argNbTlv] + 2, pCrtPos, 16);
                            pCrtPos += 16;
                            tlvBuffLen += 16 + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvSteeringData_c:
                        {
                            uint8_t len = (strlen(argv[argNbTlv]) - 2) >> 1;;
                            pCrtPos--;
                            *pCrtPos++ = len;
                            SHELL_ReadBuff(argv[argNbTlv] + 2, pCrtPos, len);
                            pCrtPos += len;
                            tlvBuffLen += len + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvSecPolicy_c:
                        {
                            meshCopSecurityPolicyTlv_t secPolicyTlv = {0};
                            uint16_t pValue = NWKU_atoi(argv[argNbTlv]);
                            htonas(secPolicyTlv.rotationInterval, pValue);
                            argNbTlv++;
                            secPolicyTlv.policy = NWKU_AsciiToHex((uint8_t *)(argv[argNbTlv] + 2), FLib_StrLen(argv[argNbTlv] + 2));
                            FLib_MemCpy(pCrtPos, (uint8_t *)&secPolicyTlv + MLE_TLV_HEADER_SIZE,
                                        aShellThrMgmtTlvs[iCount].tlvLen);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvActiveTimestamp_c:
                        case gMeshCopTlvPendingTimestamp_c:
                        {
                            uint64_t pValue = NWKU_atol(argv[argNbTlv]);
                            pValue = pValue << 16;
                            htonall(pCrtPos, pValue);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvDelayTimer_c:
                        {
                            uint32_t pValue = NWKU_atoi(argv[argNbTlv]);
                            htonal(pCrtPos, pValue);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;
                        }
                        break;

                        case gMeshCopTlvChannelMask_c:
                        {
                            meshCopChannelMaskTlv_t channelMaskTlv = {0};
                            SHELL_ReadBuff(argv[argNbTlv] + 2, channelMaskTlv.channelMask, sizeof(channelMaskTlv.channelMask));
                            channelMaskTlv.channelPage = 0;
                            channelMaskTlv.maskLength = sizeof(channelMaskTlv.channelMask);
                            FLib_MemCpy(pCrtPos, (uint8_t *)&channelMaskTlv + MLE_TLV_HEADER_SIZE,
                                        aShellThrMgmtTlvs[iCount].tlvLen);
                            pCrtPos += aShellThrMgmtTlvs[iCount].tlvLen;
                            tlvBuffLen += aShellThrMgmtTlvs[iCount].tlvLen + MLE_TLV_HEADER_SIZE;

                        }
                        break;

                        case gMeshCopTlvBogus_c:
                            tlvBuffLen += MLE_TLV_HEADER_SIZE;

                        default:
                            break;
                    }
                }
                else
                {
                    tlvBuffLen++;
                }

                break;
            }

        }

        argNbTlv++;
    }

    *pTlvBuffLen = tlvBuffLen;
}

/*!*************************************************************************************************
\private
\fn     SHELL_MeshcopMgmtPanIdConflictCb
\brief  This function is used to handle the response of Management Pan Id Query command.

\param  [in]    pIdHandlerEntry pointer to meshcop handlers
\param  [in]    pTlvs           pointer to the received tlvs
\param  [in]    tlvsLen         size of the received tlvs

\return         void
***************************************************************************************************/
static void SHELL_MeshcopMgmtPanIdConflictCb
(
    meshcopHandlers_t *pIdHandlerEntry,
    uint8_t *pTlvs,
    uint32_t tlvsLen
)
{
    shellMgmtCb_t *pMgmtCb = MEM_BufferAlloc(sizeof(shellMgmtCb_t) + tlvsLen);

    if (pMgmtCb != NULL)
    {
        pMgmtCb->tlvsLen = tlvsLen;
        FLib_MemCpy(&pMgmtCb->pTlvs, pTlvs, tlvsLen);
        // if (!NWKU_SendMsg(SHELL_MgmtPanIdConflictCbHandler, pMgmtCb, pmMainThreadMsgQueue))
        {
            MEM_BufferFree(pMgmtCb);
        }

    }
}
/*!*************************************************************************************************
\private
\fn     SHELL_MgmtPanIdConflictCbHandler
\brief  This function is used to handle the response of Management Pan Id Query command.

\param  [in]    param

\return         void
***************************************************************************************************/
static void SHELL_MgmtPanIdConflictCbHandler
(
    void *param
)
{
    shellMgmtCb_t *pMgmtCb = (shellMgmtCb_t *)param;

    if (pMgmtCb->tlvsLen != 0)
    {
        uint32_t channelMask = ntohal(pMgmtCb->pTlvs);
        uint16_t panId = ntohas(pMgmtCb->pTlvs + sizeof(channelMask));
        shell_printf("Pan ID conflict on channel mask: %i, pan ID: %i\n\r", channelMask, panId);
    }

    MEM_BufferFree(param);
    SHELL_Resume();
}
/*!*************************************************************************************************
\private
\fn     SHELL_MeshcopMgmtSetCb
\brief  This function is used to handle the response of Management Set command.

\param  [in]    pIdHandlerEntry pointer to meshcop handlers
\param  [in]    pTlvs           pointer to the received tlvs
\param  [in]    tlvsLen         size of the received tlvs

\return         void
***************************************************************************************************/
static void SHELL_MeshcopMgmtSetCb
(
    meshcopHandlers_t *pIdHandlerEntry,
    uint8_t *pTlvs,
    uint32_t tlvsLen
)
{
    shellMgmtCb_t *pMgmtCb = MEM_BufferAlloc(sizeof(shellMgmtCb_t) + tlvsLen);

    if (pMgmtCb != NULL)
    {
        pMgmtCb->tlvsLen = tlvsLen;
        FLib_MemCpy(&pMgmtCb->pTlvs, pTlvs, tlvsLen);
        // if (!NWKU_SendMsg(SHELL_MgmtSetCbHandler, pMgmtCb, pmMainThreadMsgQueue))
        {
            MEM_BufferFree(pMgmtCb);
        }

    }
};

/*!*************************************************************************************************
\private
\fn     SHELL_MgmtSetCbHandler
\brief  This function is used to handle the response of Management Set command.

\param  [in]    param

\return         void
***************************************************************************************************/
static void SHELL_MgmtSetCbHandler
(
    void *param
)
{
    shellMgmtCb_t *pMgmtCb = (shellMgmtCb_t *)param;

    meshCopStateTlv_t *pStateTlv = (meshCopStateTlv_t *)pMgmtCb->pTlvs;

    if ((pStateTlv) && (pStateTlv->type == gMeshCopTlvState_c))
    {
        shell_write("Received state: ");

        if (pStateTlv->state == MESHCOP_STATE_ACCEPT)
        {
            shell_write("Accept");
        }
        else if (pStateTlv->state == MESHCOP_STATE_PENDING)
        {
            shell_write("Pending");
        }

        if (pStateTlv->state == MESHCOP_STATE_REJECT)
        {
            shell_write("Reject");
        }
    }

    MEM_BufferFree(param);
    SHELL_Resume();
}

/*!*************************************************************************************************
\private
\fn     SHELL_MeshcopMgmtGetCb
\brief  This function is used to handle the response of Management Get command.

\param  [in]    pIdHandlerEntry pointer to meshcop handlers
\param  [in]    pTlvs           pointer to the received tlvs
\param  [in]    tlvsLen         size of the received tlvs

\return         void
***************************************************************************************************/
static void SHELL_MeshcopMgmtGetCb
(
    meshcopHandlers_t *pIdHandlerEntry,
    uint8_t *pTlvs,
    uint32_t tlvsLen
)
{
    shellMgmtCb_t *pMgmtCb = MEM_BufferAlloc(sizeof(shellMgmtCb_t) + tlvsLen);

    if (pMgmtCb != NULL)
    {
        pMgmtCb->tlvsLen = tlvsLen;
        FLib_MemCpy(&pMgmtCb->pTlvs, pTlvs, tlvsLen);
        // if (!NWKU_SendMsg(SHELL_MgmtGetCbHandler, pMgmtCb, pmMainThreadMsgQueue))
        {
            MEM_BufferFree(pMgmtCb);
        }

    }
};

/*!*************************************************************************************************
\private
\fn     SHELL_MgmtGetCbHandler
\brief  This function is used to handle the response of Management Get command.

\param  [in]    param

\return         void
***************************************************************************************************/
static void SHELL_MgmtGetCbHandler
(
    void *param
)
{
    shellMgmtCb_t *pMgmtCb = (shellMgmtCb_t *)param;
    uint8_t *pTlvs = pMgmtCb->pTlvs;
    int32_t remainingLen = pMgmtCb->tlvsLen;
    uint8_t tlvId;
    uint8_t tlvLen;

    while (remainingLen > 0)
    {
        tlvId = *pTlvs++;
        tlvLen = *pTlvs++;
        remainingLen -= MLE_TLV_HEADER_SIZE;

        for (uint32_t iCount = 0; iCount < NumberOfElements(aShellThrMgmtTlvs); iCount++)
        {
            if (aShellThrMgmtTlvs[iCount].tlvId == tlvId)
            {
                shell_printf("%s: ", aShellThrMgmtTlvs[iCount].pTlvName);
                pTlvs -= MLE_TLV_HEADER_SIZE;

                switch (aShellThrMgmtTlvs[iCount].tlvId)
                {
                    case gMeshCopTlvChannel_c:
                    {
                        meshCopChannelTlv_t *pChTlv = (meshCopChannelTlv_t *)pTlvs;
                        shell_printf("%d\r\n", ntohas(pChTlv->channel));
                    }
                    break;

                    case gMeshCopTlvPanID_c:
                    {
                        meshCopNwkPanIdTlv_t *pPanTlv = (meshCopNwkPanIdTlv_t *)pTlvs;
                        shell_printf("0x%04x\n\r", ntohas(pPanTlv->panId));
                    }
                    break;

                    case gMeshCopTlvXpanID_c:
                    {
                        meshCopNwkXPanIdTlv_t *pXpanTlv = (meshCopNwkXPanIdTlv_t *)pTlvs;
                        shell_write("0x");
                        SHELL_PrintBuff(pXpanTlv->xPanId, sizeof(pXpanTlv->xPanId));
                        shell_printf("\n\r");
                    }
                    break;

                    case gMeshCopTlvNwkName_c:
                    {
                        uint8_t aStr[16] = {0};
                        meshCopNwkNameTlv_t *pNwkNameTlv = (meshCopNwkNameTlv_t *)pTlvs;
                        FLib_MemCpy(aStr, pNwkNameTlv->nwkName, pNwkNameTlv->len);
                        shell_printf("%s\n\r", aStr);
                    }
                    break;

                    case gMeshCopTlvPskc_c:
                    {
                        meshCopPskcTlv_t *pPskcTlv = (meshCopPskcTlv_t *)pTlvs;
                        shell_write("0x");
                        SHELL_PrintBuff(pPskcTlv->pskc, pPskcTlv->len);
                        shell_printf("\n\r");
                    }
                    break;

                    case gMeshCopTlvNwkMasterKey_c:
                    {
                        meshCopNwkMasterKeyTlv_t *pMasterKeyTlv = (meshCopNwkMasterKeyTlv_t *)pTlvs;
                        shell_write("0x");
                        SHELL_PrintBuff(pMasterKeyTlv->masterKey, pMasterKeyTlv->len);
                        shell_printf("\n\r");
                    }
                    break;

                    case gMeshCopTlvNwkMlUla_c:
                    {
                        meshCopNwkMlUlaTlv_t *pMlTlv = (meshCopNwkMlUlaTlv_t *)pTlvs;
                        shell_write("0x");
                        SHELL_PrintBuff(pMlTlv->mlUla, pMlTlv->len);
                        shell_printf("\n\r");
                    }
                    break;

                    case gMeshCopTlvSteeringData_c:
                    {
                        meshCopSteeringTlv_t *pSteeringTlv = (meshCopSteeringTlv_t *)pTlvs;
                        shell_write("0x");
                        SHELL_PrintBuff(pSteeringTlv->filter, pSteeringTlv->len);
                        shell_printf("\n\r");
                    }
                    break;

                    case gMeshCopTlvBorderRouterLoc_c:
                    {
                        meshCopBrLocTlv_t *pBrTlv = (meshCopBrLocTlv_t *)pTlvs;
                        shell_printf("0x%04x\n\r", ntohas(pBrTlv->addr));
                    }
                    break;

                    case gMeshCopTlvCommSessId_c:
                    {
                        meshCopCommSessIdTlv_t *pCommSid = (meshCopCommSessIdTlv_t *)pTlvs;
                        shell_printf("0x%04x\n\r", ntohas(pCommSid->id));
                    }
                    break;

                    case gMeshCopTlvSecPolicy_c:
                    {
                        meshCopSecurityPolicyTlv_t *pSecPol = (meshCopSecurityPolicyTlv_t *)pTlvs;
                        shell_printf("%d, ", ntohas(pSecPol->rotationInterval));
                        shell_printf("0x%x\n\r", pSecPol->policy);
                    }
                    break;

                    case gMeshCopTlvActiveTimestamp_c:
                    case gMeshCopTlvPendingTimestamp_c:
                    {
                        meshCopActiveTimestampTlv_t *pTimestamp = (meshCopActiveTimestampTlv_t *)pTlvs;
                        uint64_t seconds = ntohall(pTimestamp->seconds);
                        uint16_t ticks = ntohas(pTimestamp->ticks);
                        uint8_t activeTimestampString[16] = {0};
                        seconds = seconds >> 16;

                        NWKU_PrintDec(seconds, activeTimestampString, THR_ALL_FFs8, FALSE);
                        shell_printf("%s sec, ", activeTimestampString);
                        shell_printf(" %d ticks\r\n", ticks);

                    }
                    break;

                    case gMeshCopTlvDelayTimer_c:
                    {
                        meshCopDelayTimerTlv_t *pDelayTmr = (meshCopDelayTimerTlv_t *)pTlvs;
                        shell_printf("%d ms\r\n", ntohal(pDelayTmr->timeRemaining));
                    }
                    break;

                    case gMeshCopTlvChannelMask_c:
                    {
                        meshCopChannelMaskTlv_t *pChMask = (meshCopChannelMaskTlv_t *)pTlvs;
                        shell_printf("0x%08x\n\r", ntohal(pChMask->channelMask));
                    }
                    break;

                    default:
                        break;
                }

                pTlvs += tlvLen + MLE_TLV_HEADER_SIZE;
                remainingLen -= tlvLen;
                break;
            }
        }

    }

    MEM_BufferFree(param);
    SHELL_Resume();
}

#endif

/*!*************************************************************************************************
\private
\fn     void SHELL_PrintRoutingTbl(void *param)
\brief  This function is used to print routing table.

\param  [in]    param    THR_GetRoutingTableConfirm
***************************************************************************************************/
static void SHELL_PrintRoutingTbl
(
    void *param
)
{
    THR_GetRoutingTableConfirm_t *tbl = (THR_GetRoutingTableConfirm_t *)param;

    if (tbl->NoOfElements == 0)
    {
        shell_printf("Router ID set is empty!\n\r");
        return;
    }

    shell_printf("ID Sequence: %d\n\r", tbl->IdSequenceNb);
    shell_printf("Router ID Mask: ");
    SHELL_PrintBuff((uint8_t *)&tbl->RouterIDMask, 8);
    shell_printf("\n\r");

    shell_printf("RouterID    Short Address    Next Hop    Cost    NOut    NIn \n\r");

    /* count the number of allocated router IDs */
    for (uint32_t i = 0; i < tbl->NoOfElements; i++)
    {
        shell_printf("%d", tbl->RoutingEntries[i].RouterID);

        shell_printf("           ");
        shell_printf("0x%04X", tbl->RoutingEntries[i].ShortAddress);

        shell_printf("           ");
        shell_printf("0x%04X", tbl->RoutingEntries[i].NextHop);

        shell_printf("      ");
        shell_printf("%d", tbl->RoutingEntries[i].Cost);

        shell_printf("       ");
        shell_printf("%d", tbl->RoutingEntries[i].nOut);

        shell_printf("       ");
        shell_printf("%d", tbl->RoutingEntries[i].nIn);

        shell_printf("\n\r");
    }

    MEM_BufferFree(tbl->RoutingEntries);
}

/*!*************************************************************************************************
\fn     static void SHELL_PrintStatus(thrStatus_t statusCode)
\brief  This function is used for printing in shell terminal the status of the input operation.

\param  [in]    statusCode    The code of the status to be printed
***************************************************************************************************/
static void SHELL_PrintStatus
(
    thrStatus_t statusCode
)
{
    if (gThrStatus_Success_c != statusCode)
    {
        shell_write(" An error has occurred!");
    }
    else
    {
        //shell_write(" Success!");
    }
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintParentInfo(void *param)
\brief  This function is used to print information about the parent of the node.

\param  [in]    param    THR_GetParentConfirm_t
***************************************************************************************************/
static void SHELL_PrintParentInfo
(
    void *param
)
{
    THR_GetParentConfirm_t *parent = (THR_GetParentConfirm_t *)param;
    uint64_t reversedExtAddr = parent->ExtendedAddress;

    NWKU_SwapArrayBytes((uint8_t *)&reversedExtAddr, 8);
    shell_printf("Parent short address: 0x%04X\n\r", parent->ShortAddress);
    shell_printf("Parent extended address: 0x");
    SHELL_PrintBuff((uint8_t *)&reversedExtAddr, 8);
    shell_printf("\n\r");
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintNeighborTbl(void *param)
\brief  This function is used to print neighbor table.

\param  [in]    param    THR_GetNeighborTableConfirm
***************************************************************************************************/
static void SHELL_PrintNeighborTbl
(
    void *param
)
{
    THR_GetNeighborTableConfirm_t *tbl = (THR_GetNeighborTableConfirm_t *)param;

    if (tbl->NoOfElements == 0)
    {
        shell_printf("No neighbors connected!\n\r");
    }

    shell_printf("Index Extended Address     ShortAddr  LastTime LastRSSI\n\r");

    for (uint8_t i = 0; i < tbl->NoOfElements; i++)
    {
        uint64_t reversedExtAddr = tbl->NeighborEntries[i].ExtendedAddress;

        NWKU_SwapArrayBytes((uint8_t *)&reversedExtAddr, 8);
        shell_printf("\r%d     ", i);
        shell_printf("0x");
        SHELL_PrintBuff((uint8_t *)&reversedExtAddr, 8);

        shell_printf("   ");
        shell_printf("0x%04X", tbl->NeighborEntries[i].ShortAddress);

        shell_printf("     ");
        shell_printf("%d", ((uint32_t)TmrMicrosecondsToSeconds(TMR_GetTimestamp()) - tbl->NeighborEntries[i].LastCommTime));

        shell_printf("\t%d  ", tbl->NeighborEntries[i].LastRSSI);
        shell_printf("\n\r");
    }

    MEM_BufferFree(tbl->NeighborEntries);
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintNeighborInfo(void *param)
\brief  This function is used to print information about a specific neighbor.

\param  [in]    param    THR_GetNeighborInfoConfirm_t
***************************************************************************************************/
static void SHELL_PrintNeighborInfo
(
    void *param
)
{
    THR_GetNeighborInfoConfirm_t *neighbor = (THR_GetNeighborInfoConfirm_t *)param;

    uint64_t reversedExtAddr = neighbor->NeighborInfo.Success.ExtendedAddress;

    NWKU_SwapArrayBytes((uint8_t *)&reversedExtAddr, 8);
    shell_printf("Extended Address: 0x");
    SHELL_PrintBuff((uint8_t *)&reversedExtAddr, 8);
    shell_printf("\n\r");

    shell_printf("Short Address:    0x%04X\n\r", neighbor->NeighborInfo.Success.ShortAddress);
    shell_printf("Last communication: %d seconds ago\n\r",
                 ((uint32_t)TmrMicrosecondsToSeconds(TMR_GetTimestamp()) - neighbor->NeighborInfo.Success.LastCommTime));
    shell_printf("InLinkMargin: %d\n\r", neighbor->NeighborInfo.Success.InRSSI);
    shell_printf("Device Timeout value: %d seconds\n\r", neighbor->NeighborInfo.Success.Timeoutsec);
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintDevRole(void *param)
\brief  This function is used to print the device's role.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_PrintDevRole
(
    void *param
)
{
    THR_GetAttrConfirm_AttributeValue_DeviceRole_t devRole = *((THR_GetAttrConfirm_AttributeValue_DeviceRole_t *)param);

    shell_printf("Device Role: ");

    switch (devRole)
    {
        case THR_GetAttrConfirm_AttributeValue_DeviceRole_Disconnected:
            shell_printf("Disconnected");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_SleepyEndDevice:
            shell_printf("Sleepy End Device");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_MinimalEndDevice:
            shell_printf("Minimal End Device");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_FullEndDevice:
            shell_printf("Full End Device");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_RouterEligibleEndDevice:
            shell_printf("Router Eligible End Device");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_Router:
            shell_printf("Router");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceRole_Leader:
            shell_printf("Leader");
            break;

        default:
            shell_printf("Unrecognized device role");
            break;
    }
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintDevType(void *param)
\brief  This function is used to print the device's type.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_PrintDevType
(
    void *param
)
{
    THR_GetAttrConfirm_AttributeValue_DeviceType_t devType = *((THR_GetAttrConfirm_AttributeValue_DeviceType_t *)param);

    shell_printf("Device Type: ");

    switch (devType)
    {
        case THR_GetAttrConfirm_AttributeValue_DeviceType_EndNode:
            shell_printf("End Node");
            break;

        case THR_GetAttrConfirm_AttributeValue_DeviceType_Combo:
            shell_printf("Combo Node");
            break;

        default:
            shell_printf("Unrecognized device type");
            break;
    }
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintActiveTimestamp(void *param)
\brief  This function is used to print the active timestamp.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_PrintActiveTimestamp
(
    void *param
)
{
    shell_printf("0x");
    SHELL_PrintBuff((uint8_t *)(param), 6);
    shell_printf(" seconds\n\r");
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintNwkCapabilities(void *param)
\brief  This function is used to print the device's network capabilities.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_PrintNwkCapabilities
(
    void *param
)
{
    uint8_t nwkCap = THR_ALL_FFs8;
    char *canCreateNewNwk, *isActiveRouter, *isPolling, *isFfd;

    canCreateNewNwk = STRING_FALSE;
    isActiveRouter = STRING_FALSE;
    isPolling = STRING_FALSE;
    isFfd = STRING_FALSE;

    // THR_GetAttr(threadInstanceID, THR_GetAttrConfirm_AttributeId_NwkCapabilities, 0, NULL, &nwkCap);
    // TODO: Stack API

    if (nwkCap & THR_NWKCAP_CAN_CREATE_NEW_NETWORK)
    {
        canCreateNewNwk = STRING_TRUE;
    }

    shell_printf("Can create new network: %s\r\n", canCreateNewNwk);

    if (nwkCap & THR_NWKCAP_CAN_BECOME_ACTIVE_ROUTER)
    {
        isActiveRouter = STRING_TRUE;
    }

    shell_printf("Can become active router: %s\r\n", isActiveRouter);

    if (nwkCap & THR_NWKCAP_IS_FULL_THREAD_DEVICE)
    {
        isFfd = STRING_TRUE;
    }

    shell_printf("Is full Thread device: %s\r\n", isFfd);

    if (nwkCap & THR_NWKCAP_IS_POLLING_END_DEVICE)
    {
        isPolling = STRING_TRUE;
    }

    shell_printf("Is polling end device: %s\r\n", isPolling);
}

/*!*************************************************************************************************
\fn     static void SHELL_PrintBuff(uint8_t *buff, uint32_t length)
\brief  Send to serial interface a data buffer

\param  [in]    buff      Pointer to the buffer to be send to the serial interface
\param  [in]    length    The length of data
***************************************************************************************************/
static void SHELL_PrintBuff
(
    uint8_t *buff,
    uint32_t length
)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        shell_printf("%02X", buff[i]);
    }
}

/*!*************************************************************************************************
\fn     static void SHELL_ReadBuff(char *pValue, uint8_t *pDest, uint32_t length)
\brief  Read hex stream bytes from ASCII.

\param  [in]     pValue       Pointer to start of the ASCII stream
\param  [out]    pDest        Pointer to the start of the output buffer
\param  [in]     length       The length of the ASCII stream
***************************************************************************************************/
static void SHELL_ReadBuff
(
    char *pValue,
    uint8_t *pDest,
    uint32_t length
)
{
    uint32_t iCount;
    uint16_t hexDig;

    for (iCount = 0; iCount < length; iCount++)
    {
        hexDig = NWKU_AsciiToHex((uint8_t *)pValue, sizeof(hexDig));
        pValue += sizeof(hexDig);
        pDest[iCount] = hexDig;
    }
}

/*!*************************************************************************************************
\private
\fn     uint8_t *SHELL_SearchInterfaceById(ipIfUniqueId_t ifID)
\brief  This function is used to search and validate the requested interface by the given interface ID.

\param  [in]    ifID         Interface ID

\return         uint8_t *    Pointer to the name of the interface ID
***************************************************************************************************/
static uint8_t *SHELL_SearchInterfaceById
(
    ipIfUniqueId_t ifID
)
{
    uint8_t *result = NULL;

    /* Search and verify that the interface exists on the Thread device */
    for (int i = 0; i < NumberOfElements(mThrInterfaces); i++)
    {
        if (ifID == mThrInterfaces[i].ifId)
        {
            result = mThrInterfaces[i].pAttrName;
            break;
        }
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     void SHELL_CoapSendMsg(void *param)
\brief  This function is used to send a CoAP command to the black-box. May be called on a timer.

\param  [in]    param      NWKU_CoapSendRequest_t populated with the user request
***************************************************************************************************/
static void SHELL_CoapSendMsg
(
    void *param
)
{
    uint8_t dummyCoapSession[4] = {0};
    NWKU_CoapSendRequest_t *coapSendReq = (NWKU_CoapSendRequest_t *)param;

    if (!FLib_MemCmp(dummyCoapSession, coapSendReq->coapSession, 4) ||
        mCoapInstId != THR_ALL_FFs8)
    {
        NWKU_CoapSendRequest(coapSendReq, THR_FSCI_IF);
    }
    else
    {
        shell_write("CoAP instance could not be created! Please try again..\r\n");
    }

    MEM_BufferFree(coapSendReq->Payload);
    MEM_BufferFree(coapSendReq);
}

static int8_t SHELL_CoapReply
(
    NWKU_CoapMsgReceivedIndication_t *indication
)
{
    command_ret_t ret = CMD_RET_SUCCESS;
    uint8_t *pCoapPayload = NULL;
    uint8_t coapPayloadSize = 0;
    uint8_t *currentPayloadData;
    char jsonString[JSON_BUFF_SIZE] = {0};

    NWKU_CoapSendRequest_t *pCoapSendReq = MEM_BufferAlloc(sizeof(NWKU_CoapSendRequest_t));

    if (!pCoapSendReq)
    {
        shell_write("Insufficient memory to create the FSCI request!\n\r");
        return CMD_RET_SUCCESS;
    }

    FLib_MemSet(pCoapSendReq, 0, sizeof(NWKU_CoapSendRequest_t));
    pCoapSendReq->InstanceID = threadInstanceID;
    pCoapSendReq->UDPPort = COAP_DEFAULT_PORT;

    pCoapSendReq->RequestType = gCoapAcknowledgement_c;
    pCoapSendReq->MessageType = gContent_c;

    /* Get destination address */
    FLib_MemCpy(pCoapSendReq->DestinationIpAddress, indication->RemoteIpAddress, sizeof(indication->RemoteIpAddress));

    /* Get URI path */
    FLib_MemSet(pCoapSendReq->URIpath, 0, sizeof(pCoapSendReq->URIpath));
    FLib_MemCpy(pCoapSendReq->URIpath, indication->URIpath, FLib_StrLen(indication->URIpath));

    /* Get the session */
    FLib_MemCpy(pCoapSendReq->coapSession, indication->coapSession, 4);

    /* Depending on the URI path, fill the requested payload here */
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_CO2_URI_PATH, sizeof(APP_CO2_URI_PATH) - 1)) {
        coapPayloadSize = sizeof(uint16_t);
        uint16_t airq = 0;

        if (get_air_quality((uint8_t *)&airq, &coapPayloadSize))
        {
            shell_write("Error while reading from air quality sensor!\n\r");
        }

        sprintf(jsonString, APP_CO2_JSON_FORMAT, airq);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_LIGHT_URI_PATH, sizeof(APP_LIGHT_URI_PATH) - 1)) {
        coapPayloadSize = sizeof(float);
        float light = 0;

        if (get_ambient_light((uint8_t *)&light, &coapPayloadSize))
        {
            shell_write("Error while reading from ambient light sensor!\n\r");
        }

        sprintf(jsonString, APP_LIGHT_JSON_FORMAT, light);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_HUMID_URI_PATH, sizeof(APP_HUMID_URI_PATH) - 1)) {
        coapPayloadSize = sizeof(float);
        float humid = 0;

        if (get_humidity((uint8_t *)&humid, &coapPayloadSize))
        {
            shell_write("Error while reading from relative humidity sensor!\n\r");
        }

        sprintf(jsonString, APP_HUMID_JSON_FORMAT, humid);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_TEMP_URI_PATH, sizeof(APP_TEMP_URI_PATH) - 1)) {
        coapPayloadSize = sizeof(float);
        float temp = 0;

        if (get_temperature((uint8_t *)&temp, &coapPayloadSize))
        {
            shell_write("Error while reading from temperature sensor!\n\r");
        }

        sprintf(jsonString, APP_TEMP_JSON_FORMAT, temp);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_PRES_URI_PATH, sizeof(APP_PRES_URI_PATH) - 1)) {
        coapPayloadSize = sizeof(int32_t);
        int32_t pres = 0;

        if (get_pressure((uint8_t *)&pres, &coapPayloadSize))
        {
            shell_write("Error while reading from pressure sensor!\n\r");
        }

        sprintf(jsonString, APP_PRES_JSON_FORMAT, pres);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_ROTSPEED_URI_PATH, sizeof(APP_ROTSPEED_URI_PATH) - 1)) {
        coapPayloadSize = 3 * sizeof(int16_t);
        int16_t rot_speed[3] = {0};

        if (get_rotation_speed((uint8_t *)rot_speed, &coapPayloadSize))
        {
            shell_write("Error while reading from gyroscope!\n\r");
        }

        sprintf(jsonString, APP_ROTSPEED_JSON_FORMAT, rot_speed[0], rot_speed[1], rot_speed[2]);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_ACCEL_URI_PATH, sizeof(APP_ACCEL_URI_PATH) - 1)) {
        coapPayloadSize = 3 * sizeof(float);
        float accel[3] = {0};

        if (get_acceleration((uint8_t *)accel, &coapPayloadSize))
        {
            shell_write("Error while reading from accelerometer!\n\r");
        }

        sprintf(jsonString, APP_ACCEL_JSON_FORMAT, accel[0], accel[1], accel[2]);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_MAGFIELD_URI_PATH, sizeof(APP_MAGFIELD_URI_PATH) - 1)) {
        coapPayloadSize = 3 * sizeof(float);
        float mag_field[3] = {0};

        if (get_magnetic_field((uint8_t *)mag_field, &coapPayloadSize))
        {
            shell_write("Error while reading from magnetometer!\n\r");
        }

        sprintf(jsonString, APP_MAGFIELD_JSON_FORMAT, mag_field[0], mag_field[1], mag_field[2]);
        coapPayloadSize = strlen(jsonString) + 1;
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_BATTERY_URI_PATH, sizeof(APP_BATTERY_URI_PATH) - 1)) {
        uint8_t batPercentLevel;
        uint8_t batChargingState;

        if (BatterySensor_GetState(&batPercentLevel, &batChargingState) == 0)
        {
            sprintf(jsonString, APP_BATTERY_JSON_FORMAT, batPercentLevel, batChargingState);
            coapPayloadSize = strlen(jsonString) + 1;
        }
        else
        {
        	shell_write("Error while reading from battery sensor!\n\r");
        }
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_BUZZER_URI_PATH, sizeof(APP_BUZZER_URI_PATH) - 1)) {
        if (NWKU_CoapMsgReceivedIndication_MessageType_GET ==
                                indication->MessageType)
        {
            sprintf(jsonString, APP_BUZZER_GET_JSON_FORMAT, buzzer_state);
            coapPayloadSize = strlen(jsonString) + 1;
        }
        else
        if (NWKU_CoapMsgReceivedIndication_MessageType_POST ==
                                indication->MessageType)
        {
            if (FLib_MemCmp(indication->Payload, "data:0", sizeof("data:0") - 1))
            {
                BUZZER_OFF;
                buzzer_state = 0;
            }
            else
            if (FLib_MemCmp(indication->Payload, "data:1", sizeof("data:1") - 1))
            {
                BUZZER_ON;
                buzzer_state = 1;
            }
            sprintf(jsonString, APP_BUZZER_POST_JSON_FORMAT, buzzer_state);
            coapPayloadSize = strlen(jsonString) + 1;
        }
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_BACKLIGHT_URI_PATH, sizeof(APP_BACKLIGHT_URI_PATH) - 1)) {
        if (NWKU_CoapMsgReceivedIndication_MessageType_GET ==
                                indication->MessageType)
        {
        	backlight_level_t backlight_level;
        	if (Backlight_GetLevel(&backlight_level))
        	{
        		shell_write("Error while reading from backlight!\n\r");
        	}
        	else
        	{
				sprintf(jsonString, APP_BACKLIGHT_GET_JSON_FORMAT, backlight_level);
				coapPayloadSize = strlen(jsonString) + 1;
        	}
        }
        else
        if (NWKU_CoapMsgReceivedIndication_MessageType_POST ==
                                indication->MessageType)
        {
        	_Bool noError = true;
        	backlight_level_t backlight_level;
            if (FLib_MemCmp(indication->Payload, "data:0", sizeof("data:0") - 1))
            {
            	backlight_level = BLIGHT_LEVEL_OFF;
            }
            else if (FLib_MemCmp(indication->Payload, "data:1", sizeof("data:1") - 1))
            {
            	backlight_level = BLIGHT_LEVEL_LOW;
            }
            else if (FLib_MemCmp(indication->Payload, "data:2", sizeof("data:2") - 1))
            {
            	backlight_level = BLIGHT_LEVEL_MEDIUM;
            }
            else if (FLib_MemCmp(indication->Payload, "data:3", sizeof("data:3") - 1))
            {
            	backlight_level = BLIGHT_LEVEL_HIGH;
            }
            else
            {
            	noError = false;
            }

            if (noError && (Backlight_SetLevel(backlight_level) == 0))

            {
                sprintf(jsonString, APP_BACKLIGHT_POST_JSON_FORMAT, backlight_level);
                coapPayloadSize = strlen(jsonString) + 1;
            }
            else
            {
                shell_write("Error while writing to backlight!\n\r");
            }
        }
    }
    else
    if (FLib_MemCmp(pCoapSendReq->URIpath, APP_RGBLED_URI_PATH, sizeof(APP_RGBLED_URI_PATH) - 1)) {
        if (NWKU_CoapMsgReceivedIndication_MessageType_GET ==
                                indication->MessageType)
        {
            //shell_write("RGBLED_GET\n\r");
            uint8_t brightness_state;
            uint8_t color_state;
            if (RGB_Led_Get_State(&brightness_state, &color_state))
			{
				shell_write("Error while reading from RGB LED!\n\r");
			}
            sprintf(jsonString, APP_RGBLED_GET_JSON_FORMAT, brightness_state, color_state);
            coapPayloadSize = strlen(jsonString) + 1;
        }
        else
        if (NWKU_CoapMsgReceivedIndication_MessageType_POST ==
                                indication->MessageType)
        {
            _Bool noError = true;
            uint8_t brightness_state;
            uint8_t color_state;
            //shell_write("RGBLED_POST\n\r");
            currentPayloadData = indication->Payload;
            if (FLib_MemCmp(currentPayloadData, "brightness:", sizeof("brightness:") - 1))
            {
                /* brightness */
                currentPayloadData += sizeof("brightness:") - 1;
                if (FLib_MemCmp(currentPayloadData, "0", 1))
                {
                    /**/
                    currentPayloadData += 1;
                    brightness_state = 0;
                }
                else
                if (FLib_MemCmp(currentPayloadData, "1", 1))
                {
                    /**/
                    currentPayloadData += 1;
                    brightness_state = 1;
                }
                else
                if (FLib_MemCmp(currentPayloadData, "2", 1))
                {
                    /**/
                    currentPayloadData += 1;
                    brightness_state = 2;
                }
                else
                if (FLib_MemCmp(currentPayloadData, "3", 1))
                {
                    /**/
                    currentPayloadData += 1;
                    brightness_state = 3;
                }
                else
                {
                    noError = false;
                }
            }
            else
            {
                noError = false;
            }

            if (noError && (FLib_MemCmp(currentPayloadData, ",color:", sizeof(",color:") - 1)))
            {
                /* color */
                currentPayloadData += sizeof(",color:") - 1;
                if (FLib_MemCmp(currentPayloadData, "0", 1))
                {
                    /**/
                    color_state = 0; /* green */
                }
                else
                if (FLib_MemCmp(currentPayloadData, "1", 1))
                {
                    /**/
                    color_state = 1; /* red */
                }
                else
                if (FLib_MemCmp(currentPayloadData, "2", 1))
                {
                    /**/
                    color_state = 2; /* blue */
                }
                else
                if (FLib_MemCmp(currentPayloadData, "3", 1))
                {
                    /**/
                    color_state = 3; /* white */
                }
                else
                {
                    noError = false;
                }
            }
            else
            {
                noError = false;
            }

            if (noError)
            {
                /* set brightness & color */
                if (RGB_Led_Set_State(brightness_state, color_state) == 0)
                {
                    sprintf(jsonString, APP_RGBLED_POST_JSON_FORMAT, brightness_state, color_state);
                    coapPayloadSize = strlen(jsonString) + 1;
                }
                else
                {
                	noError = false;
                }
            }

            if (!noError)
            {
                shell_write("Error while writing to RGB LED!\n\r");
            }
        }
    }
    else if (FLib_MemCmp(pCoapSendReq->URIpath, APP_AUTH_REQ_URI_PATH, sizeof(APP_AUTH_REQ_URI_PATH) - 1))
    {
        if (NWKU_CoapMsgReceivedIndication_MessageType_GET == indication->MessageType)
        {
        	uint8_t uid[APP_AUTH_UID_RAW_LEN] = {0};

        	if (get_auth_uid(uid))
        	{
        		shell_write("Error while writing retrieving UID!\n\r");
        	}

        	uint8_t cert[APP_AUTH_CERT_RAW_LEN] = {0};

        	if (get_auth_cert(cert))
        	{
        		shell_write("Error while writing retrieving Certificate!\n\r");
        	}

        	// Encode UID
        	uint8_t b64_UID[APP_AUTH_UID_ENC_LEN + 1] = {0};
        	RPK_Base64_Encode(uid, APP_AUTH_UID_RAW_LEN, b64_UID, APP_AUTH_UID_ENC_LEN);

        	// Encode Certificate
        	uint8_t b64_CERT[APP_AUTH_CERT_ENC_LEN + 1] = {0};
        	RPK_Base64_Encode(cert, APP_AUTH_CERT_RAW_LEN, b64_CERT, APP_AUTH_CERT_ENC_LEN);

        	shell_write("Sending /auth_req payload!\n\r");

            sprintf(jsonString, APP_AUTH_REQ_JSON_FORMAT, (const char *)b64_UID, (const char*)b64_CERT);
            coapPayloadSize = strlen(jsonString) + 1;
        }
    }
    else if (FLib_MemCmp(pCoapSendReq->URIpath, APP_AUTH_CHL_URI_PATH, sizeof(APP_AUTH_CHL_URI_PATH) - 1))
    {
        if (NWKU_CoapMsgReceivedIndication_MessageType_POST == indication->MessageType)
        {
        	uint8_t chlState = 0;
            currentPayloadData = (indication->Payload + sizeof("{\"uri\":\"/auth_chl\",\"data\":[{\"") - 1);
            if (FLib_MemCmp(currentPayloadData, "chal\":\"", sizeof("chal\":\"") - 1))
            {
            	currentPayloadData += sizeof("chal\":\"") - 1;

            	// Copy payload data to buffer
            	uint8_t b64_CHALLENGE[APP_AUTH_CHL_RSP_ENC_LEN + 1] = {0};
            	FLib_MemCpy(b64_CHALLENGE, currentPayloadData, APP_AUTH_CHL_RSP_ENC_LEN);

                char temp[APP_AUTH_CHL_RSP_ENC_LEN + 1] = {0};
                sprintf(temp, "Encoded Challenge: %s\n", b64_CHALLENGE);
                shell_write(temp);

            	// Decode challenge data
            	uint8_t challenge[APP_AUTH_CHL_RSP_RAW_LEN] = {0};
            	RPK_Base64_Decode(b64_CHALLENGE, APP_AUTH_CHL_RSP_ENC_LEN, challenge, APP_AUTH_CHL_RSP_RAW_LEN);

                memset(temp, 0, APP_AUTH_CHL_RSP_ENC_LEN + 1);
                shell_write("Decoded Challenge:");
                for (int i = 0; i < APP_AUTH_CHL_RSP_RAW_LEN; i++)
                {
                	memset(temp, 0, 4);
                	sprintf(temp, "%02X ", challenge[i]);
                	shell_write(temp);

                }
                shell_write("\n\n");

            	// Send Payload data to A1006
            	if (set_auth_challenge(challenge))
            	{
            		chlState = 2;
            		shell_write("Error while writing retrieving Certificate!\n\r");
            	}
            }
            else
            {
            	chlState = 1;
            }

            if (1 == chlState)
            {
            	sprintf(jsonString, APP_AUTH_CHL_JSON_FORMAT, "INVALID");
            }
            else if (2 == chlState)
            {
            	sprintf(jsonString, APP_AUTH_CHL_JSON_FORMAT, "ERROR");
            }
            else
            {
            	sprintf(jsonString, APP_AUTH_CHL_JSON_FORMAT, "OK");
            }

            coapPayloadSize = strlen(jsonString) + 1;
        }
    }
    else if (FLib_MemCmp(pCoapSendReq->URIpath, APP_AUTH_RES_URI_PATH, sizeof(APP_AUTH_RES_URI_PATH) - 1))
    {
        if (NWKU_CoapMsgReceivedIndication_MessageType_GET == indication->MessageType)
        {
        	// TODO: Get A1006 Response

        	uint8_t response[APP_AUTH_CHL_RSP_RAW_LEN] = {0};

        	if (get_auth_response(response))
        	{
        		shell_write("Error while writing retrieving challenge response!\n\r");
        	}

        	// Encode response
        	uint8_t b64_RESPONSE[APP_AUTH_CHL_RSP_ENC_LEN + 1] = {0};
        	RPK_Base64_Encode(response, APP_AUTH_CHL_RSP_RAW_LEN, b64_RESPONSE, APP_AUTH_CHL_RSP_ENC_LEN);

        	char temp[APP_AUTH_CHL_RSP_ENC_LEN + 1] = {0};
            sprintf(temp, "Encoded Response: %s\n", b64_RESPONSE);
            shell_write(temp);

            shell_write("Response:");
            for (int i = 0; i < APP_AUTH_CHL_RSP_RAW_LEN; i++)
            {
            	memset(temp, 0, 4);
            	sprintf(temp, "%02X ", response[i]);
            	shell_write(temp);

            }
            shell_write("\n\n");

            // We have the response, now trigger soft reset
            if (set_auth_softreset())
            {
                shell_write("Error while triggering A1006 soft reset!\n\r");
            }

            sprintf(jsonString, APP_AUTH_RES_JSON_FORMAT, (const char*)b64_RESPONSE);
            coapPayloadSize = strlen(jsonString) + 1;
        }
    }
    if (coapPayloadSize)
    {
        pCoapPayload = MEM_BufferAlloc(coapPayloadSize);
        FLib_MemCpy(pCoapPayload, jsonString, coapPayloadSize);
    }

    pCoapSendReq->PayloadLength = coapPayloadSize;
    pCoapSendReq->Payload = (char *)pCoapPayload;

    /* We don't need to request for a COAP instance creation */
    SHELL_CoapSendMsg((void *)pCoapSendReq);

    return ret;
}

/*!*************************************************************************************************
\private
\fn     void SHELL_CoapSend(uint8_t argc, char *argv[])
\brief  This function is used for "coap" command. Wrapper over SHELL_CoapSendMsg that also creates
        the CoAP instance if not already created.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_CoapSend
(
    uint8_t argc,
    char *argv[]
)
{
    /*coap <reqtype: CON/NON> <reqcode (GET/POST/PUT/DELETE)> <IP addr dest> <URI path> <payload ASCII>*/
    char *pValue;
    command_ret_t ret = CMD_RET_SUCCESS;
    NWKU_CoapSendRequest_RequestType_t requestType;
    NWKU_CoapSendRequest_MessageType_t requestCode;
    uint8_t *pCoapPayload = NULL;
    uint32_t coapPayloadSize = 0;
    ipAddr_t destAddr;

    NWKU_CoapSendRequest_t *pCoapSendReq = MEM_BufferAlloc(sizeof(NWKU_CoapSendRequest_t));

    if (!pCoapSendReq)
    {
        shell_write("Insufficient memory to create the FSCI request!\n\r");
        return CMD_RET_SUCCESS;
    }

    FLib_MemSet(pCoapSendReq, 0, sizeof(NWKU_CoapSendRequest_t));
    pCoapSendReq->InstanceID = threadInstanceID;
    pCoapSendReq->UDPPort = COAP_DEFAULT_PORT;

    if (argc >= 4)
    {
        if (!strcmp(argv[1], "CON"))
        {
            requestType = NWKU_CoapSendRequest_RequestType_CON;
        }
        else if (!strcmp(argv[1], "NON"))
        {
            requestType = NWKU_CoapSendRequest_RequestType_NON;
            ret = CMD_RET_SUCCESS;
        }
        else
        {
            SHELL_PrintStatus(gThrStatus_Failed_c);
            return CMD_RET_SUCCESS;
        }

        if (!strcmp(argv[2], "GET"))
        {
            requestCode = NWKU_CoapSendRequest_MessageType_GET;
        }
        else if (!strcmp(argv[2], "POST"))
        {
            requestCode = NWKU_CoapSendRequest_MessageType_POST;
        }
        else if (!strcmp(argv[2], "PUT"))
        {
            requestCode = NWKU_CoapSendRequest_MessageType_PUT;
        }
        else if (!strcmp(argv[2], "DELETE"))
        {
            requestCode = NWKU_CoapSendRequest_MessageType_DELETE;
        }
        else
        {
            SHELL_PrintStatus(gThrStatus_Failed_c);
            return CMD_RET_SUCCESS;
        }

        pCoapSendReq->RequestType = requestType;
        pCoapSendReq->MessageType = requestCode;

        /* Get destination address */
        pton(AF_INET6, argv[3], &destAddr);
        FLib_MemCpyReverseOrder(pCoapSendReq->DestinationIpAddress, destAddr.addr8, sizeof(ipAddr_t));

        /* Get URI path */
        if (argc >= 5)
        {
            pValue = argv[4];

            if (pValue)
            {
                FLib_MemCpy(pCoapSendReq->URIpath, pValue, FLib_StrLen(pValue));
            }
            else
            {
                SHELL_PrintStatus(gThrStatus_Failed_c);
                return CMD_RET_SUCCESS;
            }
        }

        /* Get payload */
        if (argc >= 6)
        {
            pValue = argv[5];

            if (pValue)
            {
                coapPayloadSize = strlen(pValue);

                if (!strcmp(argv[5], "rgb"))
                {
                    coapPayloadSize += strlen(argv[6]) + strlen(argv[7]) + strlen(argv[8]) + 4;
                }

                pCoapPayload = MEM_BufferAlloc(coapPayloadSize);

                if (pCoapPayload)
                {
                    FLib_MemSet(pCoapPayload, 0, coapPayloadSize);
                    FLib_MemCpy(pCoapPayload, pValue, coapPayloadSize);
                }
            }
        }

        pCoapSendReq->PayloadLength = coapPayloadSize;
        pCoapSendReq->Payload = (char *)pCoapPayload;

        /* Send CoAP message asynchronously; first create the CoAP instance */
        if (THR_ALL_FFs8 == mCoapInstId)
        {
            NWKU_CoapCreateInstanceRequest_t req;
            req.UDPPort = COAP_DEFAULT_PORT;
            req.SocketDomain = NWKU_CoapCreateInstanceRequest_SocketDomain_AF_INET6;

            NWKU_CoapCreateInstanceRequest(&req, THR_FSCI_IF);

            /* Start timer */
            if (mCoapDelayTimerID == gTmrInvalidTimerID_c)
            {
                mCoapDelayTimerID = TMR_AllocateTimer();
            }

            TMR_StartSingleShotTimer(mCoapDelayTimerID, 100, SHELL_CoapSendMsg, (void *)pCoapSendReq);
        }
        /* Send CoAP message synchronously */
        else
        {
            SHELL_CoapSendMsg((void *)pCoapSendReq);
        }
    }
    else
    {
        shell_write("Invalid number of parameters!\n\r");
        ret = CMD_RET_SUCCESS;
    }

    return ret;
}

/*!*************************************************************************************************
\private
\fn     static int8_t  SHELL_Ping(uint8_t argc, char *argv[])
\brief  This function is used for "ping" shell command.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Ping
(
    uint8_t argc,
    char *argv[]
)
{
    command_ret_t ret = CMD_RET_SUCCESS;
    uint8_t i, ap = AF_UNSPEC;
    uint16_t count;
    char *pValue;
    bool_t validDstIpAddr = FALSE;

    /* Stop infinite ping */
    if (argc == 0)
    {
        if (mContinuousPing)
        {
            SHELL_Resume();
        }
    }
    /* Check number of arguments according to the shellComm table */
    else
    {
        /* Reset the size of the ping */
        mPingSize = PING_PAYLOAD_DEFAULT_SIZE;
        mPingTimeoutMs = DEFAULT_TIMEOUT;
        count = 0; /* infinite ping */

        /* Get option value for -I */
        // pValue = shell_get_opt(argc, argv, "-I");

        // if (pValue && !mContinuousPing)
        // {
        //     mPingInterfaceId = gIpIfUndef_c; //SHELL_SearchInterfaceByName((uint8_t *)pValue, strlen(pValue));

        //     if (mPingInterfaceId == gIpIfUndef_c)
        //     {
        //         shell_write("Interface doesn't exist or hasn't been started!");
        //         return CMD_RET_SUCCESS;
        //     }
        // }

        /* Get option value for -s */
        pValue = shell_get_opt(argc, argv, "-s");

        if (pValue)
        {
            mPingSize = (uint16_t)NWKU_atoi(pValue);
        }

        /* Get option value for -i */
        pValue = shell_get_opt(argc, argv, "-i");

        if (pValue)
        {
            mPingTimeoutMs = (uint16_t)NWKU_atoi(pValue);

            if (mPingTimeoutMs < SHELL_PING_MIN_TIMEOUT)
            {
                mPingTimeoutMs = SHELL_PING_MIN_TIMEOUT;
            }
        }

        /* Get option value for -c */
        pValue = shell_get_opt(argc, argv, "-c");

        if (pValue)
        {
            count = (uint16_t)NWKU_atoi(pValue);
        }

        /* Get option value for -src */
        pValue = shell_get_opt(argc, argv, "-S");

        if (pValue)
        {
            /* Use MEM_BufferAllocForever() in case the ping takes more than 2 minutes */
            pSrcIpAddr = MEM_BufferAllocForever(sizeof(ipAddr_t), AppPoolId_d);
            pton(AF_INET6, pValue, pSrcIpAddr);
        }

        /* Find "-t" option */
        for (i = 1; i < argc; i++)
        {
            if (FLib_MemCmp(argv[i], "-t", 2))
            {
                count = THR_ALL_FFs16;
                break;
            }
        }

        /* Check if the destination IPv4/IPv6 address is valid */
        for (i = 1; i < argc; i++)
        {
            /* Verify IP address (v4 or v6) */
            uint8_t *pText = (uint8_t *)argv[i];

            while (*pText != '\0')
            {
                if (*pText == '.')
                {
                    ap = AF_INET;
                    break;
                }

                if (*pText == ':')
                {
                    ap = AF_INET6;
                    break;
                }

                pText++;
            }

            if ((ap != AF_UNSPEC) && (1 == pton(ap, argv[i], &mDstIpAddr)))
            {
                validDstIpAddr = TRUE;
                break;
            }
        }

        if (!validDstIpAddr)
        {
            shell_write("Invalid destination IP address");
            return CMD_RET_FAILURE;
        }

        /* Pause SHELL command parsing */
        ret = CMD_RET_ASYNC;
        /* Check number of counts */
        mContinuousPing = TRUE;
        mPingCounter = THR_ALL_FFs16;

        if (count == 0)
        {
            mPingCounter = 3;
        }
        else if (count == 1)
        {
            mContinuousPing = FALSE;

        }
        else if (count != THR_ALL_FFs16)
        {
            mPingCounter = count - 1;
        }

        /* Create Ping packet */
        FLib_MemCpyReverseOrder(mPingReq.DestinationIpAddress, mDstIpAddr.addr8, sizeof(ipAddr_t));

        if (pSrcIpAddr)
        {
            FLib_MemCpyReverseOrder(mPingReq.SourceIpAddress, pSrcIpAddr, sizeof(ipAddr_t));
        }

        mPingReq.Payloadlength = mPingSize;
        mPingReq.Timeout = mPingTimeoutMs;
        mPingReq.Secured = TRUE;

        shell_printf("Pinging %s with %u bytes of data:\r\n", argv[i], mPingSize);
        pingTimeStamp = TMR_GetTimestamp();
        (void)NWKU_PingRequest(&mPingReq, THR_FSCI_IF);

    } /* Correct number of arguments */

    return ret;
}

/*!*************************************************************************************************
\private
\fn     void PING_EchoReplyReceive(void *pParam)
\brief  Interface function for the user application. It handles a received Ping Echo Reply message.

\param  [in]    pParam    not used
***************************************************************************************************/
static void PING_EchoReplyReceive
(
    void *pParam
)
{
    uint64_t tempTimestamp;

    if (IP_IsAddrIPv6(&mDstIpAddr))
    {
        ntop(AF_INET6, &mDstIpAddr, mAddrStr, INET6_ADDRSTRLEN);
    }
    else
    {
        ntop(AF_INET, &mDstIpAddr, mAddrStr, INET_ADDRSTRLEN);
    }

    shell_write("Reply from ");
    shell_write(mAddrStr);
    shell_write(": bytes=");
    shell_writeDec(mPingSize);
    shell_write(" time=");
    tempTimestamp = TMR_GetTimestamp();
    tempTimestamp -= pingTimeStamp;
    tempTimestamp /= 1000;
    shell_writeDec(tempTimestamp);
    shell_write("ms");
    SHELL_NEWLINE();

    /* Stop timer */
    if (pingTimerID != gTmrInvalidTimerID_c)
    {
        TMR_StopTimer(pingTimerID);
    }

    /* Continuous ping: restart */
    if (mContinuousPing)
    {
        /* Start timer */
        if (mDelayTimerID == gTmrInvalidTimerID_c)
        {
            mDelayTimerID = TMR_AllocateTimer();
        }

        /* send next ping after PING_DELAY */
        TMR_StartSingleShotTimer(mDelayTimerID, PING_DELAY, PING_RetransmitHandle, NULL);
    }
    else
    {
#if ICMP_STATISTICS_ENABLED
        SHELL_PrintPingStatistics();
#endif
        SHELL_Resume();
    }
}

/*!*************************************************************************************************
\private
\fn     static void PING_HandleTimeout(void *param)
\brief  This function is used to handle the ping timeout.

\param  [in]    param    Not used
***************************************************************************************************/
static void PING_HandleTimeout
(
    void *param
)
{
    /* Ping reply was not received */
    shell_write("Request timed out.");
    SHELL_NEWLINE();

    if (mContinuousPing)
    {
        if (mPingCounter > 0 && mPingCounter != THR_ALL_FFs16)
        {
            mPingCounter--;
        }

        /* Counter have reached 0: stop pinging */
        if (mPingCounter == 0)
        {
            mContinuousPing = FALSE;
            FLib_MemSet(&mPingReq, 0, sizeof(NWKU_PingRequest_t));
        }

        PING_RetransmitHandle(NULL);
    }
    else
    {
#if ICMP_STATISTICS_ENABLED
        SHELL_PrintPingStatistics();
#endif
        SHELL_Resume();
    }
}

/*!*************************************************************************************************
\private
\fn     void PING_RetransmitHandle(void *param)
\brief  This function handles the 500ms timeout. This timer is used to send another Ping.Request
        after a Ping.Reply packet was received.

\param  [in]    param    Not used
***************************************************************************************************/
static void PING_RetransmitHandle
(
    void *param
)
{
    pingTimeStamp = TMR_GetTimestamp();
    NWKU_PingRequest(&mPingReq, THR_FSCI_IF);

    if (mPingCounter > 0 && mPingCounter != THR_ALL_FFs16)
    {
        mPingCounter--;
    }

    /* Counter have reached 0: stop pinging */
    if (mPingCounter == 0)
    {
        mContinuousPing = FALSE;
        MEM_BufferFree(pSrcIpAddr);
        pSrcIpAddr = NULL;
        FLib_MemSet(&mPingReq, 0, sizeof(NWKU_PingRequest_t));
    }
}

#if ICMP_STATISTICS_ENABLED
/*!*************************************************************************************************
\private
\fn     static void SHELL_PrintPingStatistics(void)
\brief  This function prints ping statistics.
***************************************************************************************************/
static void SHELL_PrintPingStatistics
(
    void
)
{
    uint32_t sent = 0, received = 0, lost;
    char aLoss[5], intPart[3], fractPart[2];

    if (IP_IsAddrIPv6(&mDstIpAddr))
    {
#if IP_IP6_ENABLE
        SHELL_NEWLINE();

        sent = ICMP_GetStatistics(gIcmpStatsProtIcmp6_c, gIcmpStatsDirTransmit_c, gIcmpStatsTypeEchoRequest6_c);
        received = ICMP_GetStatistics(gIcmpStatsProtIcmp6_c, gIcmpStatsDirReceive_c, gIcmpStatsTypeEchoReply6_c);

        if (sent != 0)
        {
            shell_printf("Ping statistics for %s:", ntop(AF_INET6, &mDstIpAddr, mAddrStr, INET6_ADDRSTRLEN));
        }

#endif /* IP_IP6_ENABLE */
    }
    else
    {
#if IP_IP4_ENABLE
        SHELL_NEWLINE();

        sent = ICMP_GetStatistics(gIcmpStatsProtIcmp4_c, gIcmpStatsDirTransmit_c, gIcmpStatsTypeEchoRequest4_c);
        received = ICMP_GetStatistics(gIcmpStatsProtIcmp4_c, gIcmpStatsDirReceive_c, gIcmpStatsTypeEchoReply4_c);

        if (sent != 0)
        {
            shell_printf("Ping statistics for %s:", ntop(AF_INET, &mDstIpAddr, mAddrStr, INET_ADDRSTRLEN));
        }

#endif /* IP_IP4_ENABLE */
    }

    if ((IP_IP4_ENABLE && !IP_IsAddrIPv6(&mDstIpAddr)) || (IP_IP6_ENABLE && IP_IsAddrIPv6(&mDstIpAddr)))
    {
        SHELL_NEWLINE();

        lost = sent - received;

        if (sent != 0)
        {
            if (sent == lost)
            {
                sprintf(aLoss, "%s", "100");
            }
            else if (lost == 0)
            {
                sprintf(aLoss, "%s", "0");
            }
            else
            {
                float_division(intPart, fractPart, sent, lost);

                if (strcmp(fractPart, "00") == 0)
                {
                    sprintf(aLoss, "%s", "0");
                }
                else
                {
                    sprintf(aLoss, "%s.%s", intPart, fractPart);
                }
            }

            shell_printf("Packets: Sent = %d, Received = %d, Lost = %d (%s%% loss)", sent, received, lost, aLoss);
        }

        ICMP_ResetStatistics();
    }
}
#endif /* ICMP_STATISTICS_ENABLED */

/*!*************************************************************************************************
\fn     static int8_t SHELL_MulticastGroups(thrStatus_t statusCode)
\brief  This function is used for checking the joined multicast groups or to join a new one.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_MulticastGroups
(
    uint8_t argc,
    char *argv[]
)
{
    ipAddr_t groupAddr = {0};
    command_ret_t ret = CMD_RET_SUCCESS;

    if (!strcmp(argv[1], "show"))
    {
        NWKU_McastGroupShowRequest_t req;
        req.InterfaceId = 0;  // only 6LoWPAN for now

        NWKU_McastGroupShowRequest(&req, THR_FSCI_IF);
    }

    else if ((!strcmp(argv[1], "add")) || (!strcmp(argv[1], "leave")))
    {
        if (argv[2] != NULL)
        {
            if (1 != pton(AF_INET6, argv[2], &groupAddr))
            {
                shell_write("Invalid multicast group address");
            }
            else
            {
                NWKU_McastGroupManageRequest_t req;
                req.InterfaceId = 0;  // only 6LoWPAN for now
                FLib_MemCpyReverseOrder(req.McastAddress, groupAddr.addr8, sizeof(ipAddr_t));

                if (!strcmp(argv[1], "add"))
                {
                    req.Action = NWKU_McastGroupManageRequest_Action_JoinGroup;
                }
                else if (!strcmp(argv[1], "leave"))
                {
                    req.Action = NWKU_McastGroupManageRequest_Action_LeaveGroup;
                }
                else
                {
                    shell_write("Invalid multicast group action. Please choose between 'add' and 'leave'.");
                    return ret;
                }

                NWKU_McastGroupManageRequest(&req, THR_FSCI_IF);
            }
        }
        else
        {
            shell_write("Type a multicast group address");
        }
    }

    return ret;
}

#if SOCK_DEMO
/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_Socket(uint8_t argc, char *argv[])
\brief  This function is used for sockets shell command.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Socket
(
    uint8_t argc,
    char *argv[]
)
{
    command_ret_t ret = CMD_RET_ASYNC;
    appSockCmdParams_t *pAppSockCmdParams = MEM_BufferAlloc(sizeof(appSockCmdParams_t));

    /* Stop any socket pending commands */
    if ((argc == 0) || (pAppSockCmdParams == NULL))
    {
        SHELL_Resume();
        ret = CMD_RET_SUCCESS;
        return ret;
    }

    FLib_MemSet(pAppSockCmdParams, 0, sizeof(appSockCmdParams_t));

    /* socket open */
    if (!strcmp(argv[1], "open"))
    {
        /* Check number of arguments according to the shellComm table */
        pAppSockCmdParams->appSocketsCmd = gSockOpenUdp;

        /* socket open udp */
        if (!strcmp(argv[2], "udp"))
        {
            uint32_t result = -1;

            pAppSockCmdParams->appSocketsTrans = gSockUdp;
            /* Set local information */
            pAppSockCmdParams->ipVersion = AF_INET6;

            /* Set remote information for easier send */
            if (pAppSockCmdParams->ipVersion == AF_INET6)
            {
                pAppSockCmdParams->sin6_port = atoi((char const *)argv[4]);
                result = pton(AF_INET6, argv[3], &pAppSockCmdParams->sin6_addr);
            }
            else
            {
                pAppSockCmdParams->sin_port = atoi((char const *)argv[4]);
                result = pton(AF_INET, argv[3], &pAppSockCmdParams->sin_addr);
            }

            if (result == -1)
            {
                shell_write("IP address has a wrong format");
                SHELL_NEWLINE();
                ret = CMD_RET_FAILURE;
            }
        }
        else
        {
            ret = CMD_RET_FAILURE;
            shell_write("Wrong number of parameters");
        }

    }
    /* socket close */
    else if (!strcmp(argv[1], "close"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockClose;
        pAppSockCmdParams->sock32 = atoi((const char *)argv[2]);
    }
    /* socket send */
    else if (!strcmp(argv[1], "send"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockSend;

        /* Check number of arguments according to the shellComm table */
        if (argc == 4)
        {
            /* Get socket id */
            pAppSockCmdParams->sock32 = atoi((const char *)argv[2]);
            pAppSockCmdParams->pData = MEM_BufferAlloc(sizeof(pollInfo_t));

            if (NULL != pAppSockCmdParams->pData)
            {
                pAppSockCmdParams->dataLen = strlen((char const *)argv[3]);
                FLib_MemCpy(pAppSockCmdParams->pData, argv[3], pAppSockCmdParams->dataLen);
            }
            else
            {
                ret = CMD_RET_FAILURE;
            }
        }
        else
        {
            ret = CMD_RET_FAILURE;
            shell_write("Wrong number of parameters");
        }
    }
    /* socket select */
    else if (!strcmp(argv[1], "select"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockSelect;

        /* Check number of arguments according to the shellComm table */
        if (argc == 3)
        {
            /* Get socket ID from command */
            pAppSockCmdParams->sock32 = atoi((char const *)argv[2]);
        }
        else
        {
            ret = CMD_RET_FAILURE;
            shell_write("Wrong number of parameters");
        }
    }
    /* socket post */
    else if (!strcmp(argv[1], "post"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockPost;

        /* Check number of arguments according to the shellComm table */
        if (argc == 4 || argc == 3 || argc == 5)
        {
            pAppSockCmdParams->bSelectedFlag = FALSE;

            /* socket poll <socket id> <timeout> */
            if (NWKU_IsNUmber(argv[2]))
            {
                pAppSockCmdParams->sock32 = atoi((char const *)argv[2]);

                if (argc == 4)
                {
                    /* Get time from command */
                    pAppSockCmdParams->timeMs = atoi((char const *)argv[3]);
                }
                else
                {
                    pAppSockCmdParams->timeMs = DEFAULT_TIMEOUT;
                }
            }
            /* socket poll selected */
            else if (!strcmp((const char *)argv[2], "selected"))
            {
                pAppSockCmdParams->bSelectedFlag = TRUE;
                /* Get time from command */
                pAppSockCmdParams->timeMs = atoi((char const *)argv[3]);
            }
        }
        else
        {
            shell_write("Wrong number of parameters");
            ret = CMD_RET_FAILURE;
        }
    }
    else
    {
        shell_write("Wrong command syntax");
        ret = CMD_RET_FAILURE;
    }

    if (ret == CMD_RET_ASYNC)
    {
        if (FALSE == App_SocketSendAsync(pAppSockCmdParams))
        {
            ret = CMD_RET_FAILURE;
            shell_write("No memory for creating the command");
            MEM_BufferFree(pAppSockCmdParams);
        }
    }
    else
    {
        MEM_BufferFree(pAppSockCmdParams);
    }

    return ret;
}
#endif /* SOCK_DEMO */

#if UDP_ECHO_PROTOCOL
/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_EchoUdp(uint8_t argc, char *argv[])
\brief  This function is used to send packets with echo protocol.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_EchoUdp
(
    uint8_t argc,
    char *argv[]
)
{
    return ECHO_ShellUdp(argc, argv);
}
#endif

#if DNS_ENABLED
/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_SendDns(uint8_t argc, char *argv[])
\brief  This function is used to send a DNS request.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_SendDns
(
    uint8_t argc,
    char *argv[]
)
{
    if (strcmp(argv[1], "nddata") == 0)
    {
        THR_DNS_ND_DataReq(threadInstanceID, SHELL_DnsNdDataService);
    }
    else
    {
        uint8_t nodename[30] = {0};
        appReturnHandler_t pfAppReturn = SHELL_DnsService;
        int32_t len = strlen(argv[1]);

        FLib_MemCpy(&nodename, argv[1], len);
        DNS_GetHostByName(nodename, mDnsTypeAAAA_c, mDnsClassIN_c, pfAppReturn, pmMainThreadMsgQueue);
    }

    return CMD_RET_ASYNC;
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_DnsService(void *pMsg)
\brief  This function is the callback function for the DNS response message.

\param  [in]    pMsg    Pointer to message
***************************************************************************************************/
static void SHELL_DnsService
(
    void *pMsg
)
{
    hostent_t *pDnsResponse = (hostent_t *)pMsg;
    ipAddr_t *pIpAddr = NULL;
    char respAddrStr[INET6_ADDRSTRLEN];

    if (pMsg != NULL)
    {
        shell_printf("%s is at ", pDnsResponse->pHostName);

        do
        {
            pIpAddr = ListGetHeadMsg(pDnsResponse->pHostAddrList);

            if (pIpAddr)
            {
                ntop(AF_INET6, pIpAddr, respAddrStr, INET6_ADDRSTRLEN);
                shell_printf(respAddrStr);
                shell_write("\n");

                ListRemoveMsg(pIpAddr);
                MEM_BufferFree(pIpAddr);
            }
        }
        while (pIpAddr);

        MEM_BufferFree(pDnsResponse->pHostName);
        MEM_BufferFree(pDnsResponse->pHostAliasesList);
        MEM_BufferFree(pDnsResponse->pHostAddrList);
        MEM_BufferFree(pMsg);
    }
    else
    {
        shell_write("No response received!\n");
    }

    SHELL_Resume();
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_DnsNdDataService(coapSessionStatus_t sessionStatus, void *pData,
                                           coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for the ND_DATA.req message.

\param  [in]    sessionStatus    Status of the session
\param  [in]    pData            Pointer to data
\param  [in]    pSession         Pointer to CoAP session
\param  [in]    dataLen          Length of data
***************************************************************************************************/
static void SHELL_DnsNdDataService
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    uint8_t *pCursor = pData;
    uint8_t iCount, noServers, noDomainSrvBytes, domainSrvLen;
    char serverAddrStr[INET6_ADDRSTRLEN];

    if (dataLen > 0 && pCursor != NULL)
    {
        /* check type */
        if (*pCursor != 9) // THR_NWK_ND_DATA_TLV_TYPE
        {
            shell_write("Did not receive a valid ND_DATA.rsp!\r\n");
            return;
        }

        pCursor += MLE_TLV_HEADER_SIZE;
        dataLen -= MLE_TLV_HEADER_SIZE;

        while (dataLen > 0)
        {
            switch (*pCursor)
            {
                case gNdOptionRDNSS_c:
                    shell_write("Recursive DNS Servers:\r\n");
                    noServers = (*(++pCursor) - 1) >> 1;
                    dataLen -= 1;

                    pCursor += 7;  // skip Length(1), Reserved(2) and Lifetime(4)
                    dataLen -= 7;

                    for (iCount = 0; iCount < noServers; iCount++)
                    {
                        shell_write("\t");
                        shell_write(ntop(AF_INET6, (ipAddr_t *)pCursor, serverAddrStr, INET_ADDRSTRLEN));
                        shell_write("\r\n");
                        pCursor += sizeof(ipAddr_t);
                        dataLen -= sizeof(ipAddr_t);
                    }

                    break;

                case gNdOptionDNSSL_c:
                    shell_write("DNS Search Lists:\r\n");
                    noDomainSrvBytes = (*(++pCursor) - 1) << 3;
                    dataLen -= 1;

                    pCursor += 7;  // skip Length(1), Reserved(2) and Lifetime(4)
                    dataLen -= 7;

                    while (*pCursor != '\0')
                    {
                        shell_write("\t");
                        shell_write((char *)pCursor);
                        shell_write("\r\n");
                        domainSrvLen = FLib_StrLen((char *)pCursor) + 1;  // +1 to include '\0'
                        pCursor += domainSrvLen;
                        dataLen -= domainSrvLen;
                        noDomainSrvBytes -= domainSrvLen;
                    }

                    pCursor += noDomainSrvBytes;
                    dataLen -= noDomainSrvBytes;  // jump over extra padding bytes

                    break;
            }
        }
    }
    else
    {
        shell_write("There is no BR able to supply DNS information!\r\n");
    }

    if (pSession)
    {
        COAP_CloseSession(pSession);
    }

    SHELL_Resume();
}
#endif /* DNS_ENABLED */

#if MAC_FILTERING_ENABLED
/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_Filter(uint8_t argc, char *argv[])
\brief  This function is used for "filter" shell command.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Filter
(
    uint8_t argc,
    char *argv[]
)
{
    command_ret_t ret = CMD_RET_SUCCESS;
    char *pValue;

    /* Stop any socket pending commands */
    if (argc > 0)
    {
        if (!strcmp(argv[1], "add"))
        {
            uint64_t extendedAddress = 0;
            uint8_t linkIndicator = THR_ALL_FFs8;
            bool_t block = FALSE;

            /* Get the extended address of the neighbor */
            if (argv[2])
            {
                SHELL_ReadBuff(argv[2] + 2, (uint8_t *)&extendedAddress, 8);
                NWKU_SwapArrayBytes((uint8_t *)&extendedAddress, 8);
            }

            /* Get the value for LQI */
            pValue = shell_get_opt(argc, argv, "lqi");

            if (pValue)
            {
                linkIndicator = (uint8_t)NWKU_atoi(pValue);
            }

            pValue = shell_get_opt(argc, argv, "reject");

            if (pValue)
            {
                block = (bool_t)NWKU_atoi(pValue);
            }

            if (0 != extendedAddress)
            {
                if (MacFiltering_AddNeighbor(threadInstanceID, extendedAddress, THR_ALL_FFs16,
                                             linkIndicator, block) == gThrStatus_Success_c)
                {
                    shell_write("Entry has been added!\r\n");
                }
                else
                {
                    shell_write("An error has occurred!\r\n");
                }
            }
        }
        else if (!strcmp(argv[1], "remove"))
        {
            uint64_t extendedAddress = 0;

            if (argv[2])
            {
                SHELL_ReadBuff(argv[2] + 2, (uint8_t *)&extendedAddress, 8);
                NWKU_SwapArrayBytes((uint8_t *)&extendedAddress, 8);
            }
            else
            {
                shell_write("Enter the extended address of the neighbor!\r\n");
                ret = CMD_RET_ASYNC;
            }

            if (0 != extendedAddress)
            {
                if (MacFiltering_RemoveNeighbor(threadInstanceID, extendedAddress) == gThrStatus_Success_c)
                {
                    shell_write("Entry has been removed!\r\n");
                }
                else
                {
                    shell_write("An error has occurred!\r\n");
                }
            }

        }
        else if (!strcmp(argv[1], "enable"))
        {
            if (NULL != argv[2])
            {
                if (!strcmp(argv[2], "reject"))
                {
                    MacFiltering_Active(threadInstanceID, mMacFilteringDefaultPolicyReject_c);
                }
                else if (!strcmp(argv[2], "accept"))
                {
                    MacFiltering_Active(threadInstanceID, mMacFilteringDefaultPolicyAccept_c);
                }
            }
            else
            {
                MacFiltering_Active(threadInstanceID, mMacFilteringDefaultPolicyAccept_c);
            }

            shell_printf("MAC Filtering Status: %s",
                         mMacFilteringStatus[MacFiltering_IsActive(threadInstanceID)]);
        }
        else if (!strcmp(argv[1], "disable"))
        {
            MacFiltering_Active(threadInstanceID, mMacFilteringDisabled_c);
            shell_write("MAC Filtering Disabled!\r\n");
        }
        else if (!strcmp(argv[1], "show"))
        {
            shell_printf("MAC Filtering Status: %s",
                         mMacFilteringStatus[MacFiltering_IsActive(threadInstanceID)]);

            SHELL_Filtering_Print(threadInstanceID);
        }
        else
        {
            ret = CMD_RET_ASYNC;
        }
    }
    else
    {
        SHELL_Resume();
    }

    return ret;
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_Filtering_Print(uint32_t instanceID)
\brief  This function is used to print the filtered addresses.

\param  [in]    instanceID    Thread instance ID
***************************************************************************************************/
static void SHELL_Filtering_Print
(
    uint32_t instanceID
)
{
    uint32_t iCount = MAC_FILTERING_TABLE_SIZE;
    macFilteringNeighborData_t macFilterEntry;

    shell_printf("Idx   Extended Address     Short Address   Link Quality  Reject\n\r");

    do
    {
        iCount--;

        // if(THR_GetAttr(instanceID, gNwkAttrId_WhiteListEntry_c, iCount, NULL, &(macFilterEntry)) == gThrStatus_Success_c)
        // TODO: Stack API
        {
            uint64_t reversedExtAddr = macFilterEntry.extendedAddress;

            NWKU_SwapArrayBytes((uint8_t *)&reversedExtAddr, 8);
            shell_printf("%d", iCount);
            shell_printf("     ");
            shell_printf("0x");
            SHELL_PrintBuff((uint8_t *)&reversedExtAddr, 8);
            shell_printf("   ");

            if (macFilterEntry.shortAddress != THR_ALL_FFs16)
            {
                shell_printf("0x%04X", macFilterEntry.shortAddress);
            }
            else
            {
                shell_write("------");
            }

            shell_printf("          ");
            shell_printf("%d", macFilterEntry.linkIndicator);
            shell_printf("            ");

            if (macFilterEntry.blockNeighbor == TRUE)
            {
                shell_printf("TRUE\r\n");
            }
            else
            {
                shell_printf("FALSE\r\n");
            }
        }
    }
    while (iCount);

    shell_printf("\r\nEnd of MAC Filtering Table.\r\n");
}
#endif /* MAC_FILTERING_ENABLED */

#ifdef USE_MSD_BOOTLOADER
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_FlashEraseWithMSDFunc(uint8_t argc, char *argv[])
\brief  This function is used for firmware erase and re-enabling the MSD bootloader.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_FlashEraseWithMSDFunc
(
    uint8_t argc,
    char *argv[]
)
{
    shell_write("\r Start Firmware Erase and enable the MSD Bootloader! \n\r");
    NvEraseSector(0xBFF8);
    ResetMCU();
    return FALSE;
}
#endif /* USE_MSD_BOOTLOADER */

#if gHybridApp_d
/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_BreakReq(uint8_t argc, char *argv[])
\brief  This function is used to resume a continuous process(ex: ping).

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_BreakReq
(
    uint8_t argc,
    char *argv[]
)
{
    gbRetryInterrupt = FALSE;
    mContinuousPing = FALSE;
#if ECHO_PROTOCOL
    mEchoUdpCounter = 0;
#endif

    SHELL_Resume();
    return FALSE;
}
#endif

#if GETIPv6ADDRESSES_APP
/*===================================GETIPv6ADDRESSES=============================================*/
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_GetNeighborsIpAddr()
\brief  This function is the callback function for "getneighborsip" shell command.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_GetNeighborsIpAddr
(
    uint8_t argc,
    char *argv[]
)
{
    ipAddr_t coapDestAddress =  in6addr_realmlocal_allthreadnodes;
    static bool_t bIsCbRegistered = FALSE;

    /* Request the IPv6 Address List from each node */
    /* This could be an array of multiple TLVs */
    uint8_t requestedTLVs[] = {gDiagnosticTlv_IPv6AddrList_c};

    mgmtDiagnostic_CoapMsg_t *pDiagGetTlv = MEM_BufferAlloc(sizeof(mgmtDiagnostic_CoapMsg_t) + NumberOfElements(requestedTLVs));

    if (pDiagGetTlv != NULL)
    {
        if (FALSE == bIsCbRegistered)
        {
            /* Register the application callback for receiving the diagnostic reply messages */
            (void)MgmtDiagnostic_RegisterAppCb(SHELL_MgmtDiagnosticAppCb);
            bIsCbRegistered = TRUE;
        }

        /* Set the destination address */
        /* By default, the destination is All Thread Nodes */
        FLib_MemCpy(pDiagGetTlv->dstIpAddr, &coapDestAddress, sizeof(ipAddr_t));

        pDiagGetTlv->payloadSize = NumberOfElements(requestedTLVs);
        FLib_MemCpy(pDiagGetTlv->payload, requestedTLVs, NumberOfElements(requestedTLVs));

        /* Send Diagnostic Message */
        MgmtDiagnostic_SendGetReq(threadInstanceID, pDiagGetTlv, NULL);

        MEM_BufferFree(pDiagGetTlv);
    }

    /* Refresh Shell */
    return CMD_RET_SUCCESS;
}

/*!*************************************************************************************************
\fn     static void SHELL_MgmtDiagnosticAppCb(mgmtDiagnostic_RspData_t mgmtDiagRspData)
\brief  This function is the callback function for receiving reply to diagnostic message.

\param  [in]    mgmtDiagRspData     Diagnostic reply data
***************************************************************************************************/
static void SHELL_MgmtDiagnosticAppCb
(
    mgmtDiagnostic_RspData_t mgmtDiagRspData
)
{
    mgmtDiagnostic_Tlv_t *pTlvHeader = (mgmtDiagnostic_Tlv_t *)mgmtDiagRspData.pPayload;
    char addrStr[INET6_ADDRSTRLEN] = {0};

    if (mgmtDiagRspData.status == gThrStatus_Success_c)
    {
        /* Check if the reply has IPv6AddrList TLV */
        if (pTlvHeader->tlvType == gDiagnosticTlv_IPv6AddrList_c)
        {
            uint8_t noOfAddr = pTlvHeader->length / sizeof(ipAddr_t);

            shell_write("\r\nReceived IP Addresses from node:\r\n");

            /* Print addresses */
            for (uint32_t iCount = 0; iCount < noOfAddr; iCount++)
            {
                ipAddr_t ipAddr = {0};
                ipAddr_t *pIpAddr = (ipAddr_t *)(mgmtDiagRspData.pPayload + sizeof(mgmtDiagnostic_Tlv_t) + sizeof(ipAddr_t) * iCount);

                /* Need aligned data structure */
                FLib_MemCpy(&ipAddr, pIpAddr, sizeof(ipAddr_t));

                if (IP_IsAddrIPv6(&ipAddr))
                {
                    ntop(AF_INET6, &ipAddr, addrStr, INET6_ADDRSTRLEN);
                    shell_write(addrStr);
                    SHELL_NEWLINE();
                }
            }

            shell_refresh();
        }
    }
}
#endif

#if ICMP_STATISTICS_ENABLED
/*!*************************************************************************************************
\private
\fn     static void float_division(char *intPart, char *fractPart, int sent, int lost)
\brief  This function determines integer and fractional part of lost / sent division.

\param  [in]    intPart     Pointer to retained integer part of division
\param  [in]    fractPart   Pointer to retained fractional part of division
\param  [in]    sent        Number of packets sent
\param  [in]    lost        Number of packets lost
***************************************************************************************************/
static void float_division(char *intPart, char *fractPart, uint32_t sent, uint32_t lost)
{
    uint32_t multiplier = 1, mulFactor, lostCopy = lost, sentCopy = sent;
    uint16_t loss;
    char aLoss[5];
    int8_t cIntPartDigits;
    uint8_t cLostDigits = 0, cSentDigits = 0;

    while (lostCopy != 0)
    {
        cLostDigits++;
        lostCopy = lostCopy / 10;
    }

    while (sentCopy != 0)
    {
        cSentDigits++;
        sentCopy = sentCopy / 10;
    }

    mulFactor = 3 + cLostDigits;
    cIntPartDigits = 2 + (cLostDigits - cSentDigits);

    while (mulFactor > 0)
    {
        multiplier *= 10;
        mulFactor--;
    }

    loss = lost * multiplier / sent;
    sprintf(aLoss, "%d", loss);
    aLoss[4] = '\0';

    switch (cIntPartDigits)
    {
        case 2:
            intPart[0] = aLoss[0];
            intPart[1] = aLoss[1];
            intPart[2] = '\0';

            fractPart[0] = aLoss[2];
            fractPart[1] = aLoss[3];
            fractPart[2] = '\0';
            break;

        case 1:
            intPart[0] = aLoss[0];
            intPart[1] = '\0';

            fractPart[0] = aLoss[1];
            fractPart[1] = aLoss[2];
            fractPart[2] = '\0';
            break;

        case 0:
            intPart[0] = '0';
            intPart[1] = '\0';

            fractPart[0] = aLoss[0];
            fractPart[1] = aLoss[1];
            fractPart[2] = '\0';
            break;

        case -1:
            intPart[0] = '0';
            intPart[1] = '\0';

            fractPart[0] = '0';
            fractPart[1] = aLoss[0];
            fractPart[2] = '\0';
            break;

        default:
            sprintf(intPart, "%s", "0");
            sprintf(fractPart, "%s", "00");
            break;
    }
}
#endif/* ICMP_STATISTICS_ENABLED */

/*!*************************************************************************************************
\private
\fn     static void SHELL_ThrNwkCreate(void *param)
\brief  This function is used to create a Thread network.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_ThrNwkCreate
(
    void *param
)
{
    THR_CreateNwkRequest_t req;
    req.InstanceID = threadInstanceID;

    if (THR_CreateNwkRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("No memory!\n\r");
    }
}

/*!*************************************************************************************************
\private
\fn     static void SHELL_ThrNwkJoin(void *param)
\brief  This function is used to join to a Thread network.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_ThrNwkJoin
(
    void *param
)
{
    THR_JoinRequest_t req;
    req.InstanceID = threadInstanceID;
    req.discoveryMethod = THR_JoinRequest_discoveryMethod_gUseThreadDiscovery_c;

    if (THR_JoinRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("No memory!\n\r");
    }
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_FactoryResetReq(uint8_t argc, char *argv[])
\brief  This function is used to reset the device and restore the factory defaults.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_FactoryResetReq
(
    uint8_t argc,
    char *argv[]
)
{
    shell_write("\rReset Thread stack to factory defaults!\n\r");
    (void)THR_FactoryResetRequest(THR_FSCI_IF);
    return CMD_RET_SUCCESS;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_FactoryResetReq(uint8_t argc, char *argv[])
\brief  This function is used to reset the device and restore the factory defaults.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Autostart
(
    uint8_t argc,
    char *argv[]
)
{
    char *pValue = shell_get_opt(argc, argv, "-c");

    if (!pValue)
    {
        shell_write("\n\rPlease provide the 802.15.4 to start on!");
        return CMD_RET_SUCCESS;
    }
    else
    {
        mAutostartChannel = (uint8_t)NWKU_atoi(pValue);

        if (mAutostartChannel < 11 || mAutostartChannel > 26)
        {
            shell_write("\n\rInvalid 802.15.4 channel, please select one in interval [11-26]!");
            return CMD_RET_SUCCESS;
        }
    }

    mAutostartInProgress = TRUE;
    mAutostartState = gAutostartFirstState_c;

    SHELL_FactoryResetReq(argc, argv);
    return CMD_RET_SUCCESS;
}

/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_Ifconfig(uint8_t argc, char *argv[])
\brief  This function is used for ifconfig.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Ifconfig
(
    uint8_t argc,
    char *argv[]
)
{
    (void)NWKU_IfconfigAllRequest(THR_FSCI_IF);
    return CMD_RET_SUCCESS;
}

static void SHELL_PrintIfconfigAllRsp
(
    NWKU_IfconfigAllResponse_t *evt
)
{
    char addrStr[INET6_ADDRSTRLEN];
    uint8_t temp[INET_ADDRSTRLEN];  // container for the reversed bytes

    for (uint8_t i = 0; i < evt->CountInterfaces; i++)
    {
        // protection against badly formatted response
        if (evt->InterfaceIDs[i].InterfaceID != i)
        {
            SHELL_NEWLINE();
            // shell_write("No address available!");
            break;
        }

        shell_printf("\n\rInterface #%d", i);

        for (uint8_t j = 0; j < evt->InterfaceIDs[i].CountIpAddresses; j++)
        {
            FLib_MemCpyReverseOrder(temp, &evt->InterfaceIDs[i].Addresses[j * INET_ADDRSTRLEN], INET_ADDRSTRLEN);
            ntop(AF_INET6, (ipAddr_t *)temp, addrStr, INET6_ADDRSTRLEN);
            shell_printf("\n\r\tAddress #%d: %s", j + 1, addrStr);
        }

        MEM_BufferFree(evt->InterfaceIDs[i].Addresses);
    }

    MEM_BufferFree(evt->InterfaceIDs);
}


static attrRef_t SHELL_GetAttrRefFromId
(
    THR_GetAttrConfirm_t *evt
)
{
    attrRef_t res;
    res.pValue = NULL;
    res.requireFree = FALSE;

    switch (evt->AttributeId)
    {
        case THR_GetAttrConfirm_AttributeId_RandomExtendedAddr:
            res.pValue = &(evt->AttributeValue.RandomExtendedAddr);
            break;

        case THR_GetAttrConfirm_AttributeId_ShortAddress:
            res.pValue = &(evt->AttributeValue.ShortAddress);
            break;

        case THR_GetAttrConfirm_AttributeId_ScanChannelMask:
            res.pValue = &(evt->AttributeValue.ScanChannelMask);
            break;

        case THR_GetAttrConfirm_AttributeId_ScanDuration:
            res.pValue = &(evt->AttributeValue.ScanDuration);
            break;

        case THR_GetAttrConfirm_AttributeId_Channel:
            res.pValue = &(evt->AttributeValue.Channel);
            break;

        case THR_GetAttrConfirm_AttributeId_ShortPanId:
            res.pValue = &(evt->AttributeValue.ShortPanId);
            break;

        case THR_GetAttrConfirm_AttributeId_ExtendedPanId:
            res.pValue = evt->AttributeValue.ExtendedPanId;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_PermitJoin:
            res.pValue = &(evt->AttributeValue.PermitJoin);
            break;

        case THR_GetAttrConfirm_AttributeId_RxOnIdle:
            res.pValue = &(evt->AttributeValue.RxOnIdle);
            break;

        case THR_GetAttrConfirm_AttributeId_SedPollInterval:
            res.pValue = &(evt->AttributeValue.SedPollInterval);
            break;

        case THR_GetAttrConfirm_AttributeId_UniqueExtendedAddress:
            res.pValue = &(evt->AttributeValue.UniqueExtendedAddress);
            break;

        case THR_GetAttrConfirm_AttributeId_VendorName:
            res.pValue = evt->AttributeValue.VendorName;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_ModelName:
            res.pValue = evt->AttributeValue.ModelName;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_SwVersion:
            res.pValue = evt->AttributeValue.SwVersion;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_StackVersion:
            res.pValue = &(evt->AttributeValue.StackVersion);
            break;

        case THR_GetAttrConfirm_AttributeId_NwkCapabilities:
            res.pValue = &(evt->AttributeValue.NwkCapabilities);
            break;

        case THR_GetAttrConfirm_AttributeId_NwkName:
            res.pValue = evt->AttributeValue.NwkName;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_DeviceType:
            res.pValue = &(evt->AttributeValue.DeviceType);
            break;

        case THR_GetAttrConfirm_AttributeId_IsDevConnected:
            res.pValue = &(evt->AttributeValue.IsDevConnected);
            break;

        case THR_GetAttrConfirm_AttributeId_IsDevCommissioned:
            res.pValue = &(evt->AttributeValue.IsDevCommissioned);
            break;

        case THR_GetAttrConfirm_AttributeId_PartitionId:
            res.pValue = &(evt->AttributeValue.PartitionId);
            break;

        case THR_GetAttrConfirm_AttributeId_DeviceRole:
            res.pValue = &(evt->AttributeValue.DeviceRole);
            break;

        case THR_GetAttrConfirm_AttributeId_Security_NwkMasterKey:
            res.pValue = evt->AttributeValue.Security_NwkMasterKey;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_Security_NwkKeySeq:
            res.pValue = &(evt->AttributeValue.Security_NwkKeySeq);
            break;

        case THR_GetAttrConfirm_AttributeId_Security_PSKc:
            res.pValue = evt->AttributeValue.Security_PSKc;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_Security_PSKd:
            res.pValue = evt->AttributeValue.Security_PSKd;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_VendorData:
            res.pValue = evt->AttributeValue.VendorData;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_MLPrefix:
            res.pValue = &(evt->AttributeValue.MLPrefix);
            break;

        case THR_GetAttrConfirm_AttributeId_MacFilteringEntry:
            res.pValue = &(evt->AttributeValue.MacFilteringEntry);
            break;

        case THR_GetAttrConfirm_AttributeId_Security_KeyRotationInterval:
            res.pValue = &(evt->AttributeValue.Security_KeyRotationInterval);
            break;

        case THR_GetAttrConfirm_AttributeId_ChildAddrMask:
            res.pValue = evt->AttributeValue.ChildAddrMask;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_ChildSEDTimeout:
            res.pValue = &(evt->AttributeValue.ChildSEDTimeout);
            break;

        case THR_GetAttrConfirm_AttributeId_ChildEDTimeout:
            res.pValue = &(evt->AttributeValue.ChildEDTimeout);
            break;

        case THR_GetAttrConfirm_AttributeId_EndDevice_ChildEDReqFullNwkData:
            res.pValue = &(evt->AttributeValue.EndDevice_ChildEDReqFullNwkData);
            break;

        case THR_GetAttrConfirm_AttributeId_EndDevice_IsFastPollEnabled:
            res.pValue = &(evt->AttributeValue.EndDevice_IsFastPollEnabled);
            break;

        case THR_GetAttrConfirm_AttributeId_SleepyEndDevice_FastPollInterval:
            res.pValue = &(evt->AttributeValue.SleepyEndDevice_FastPollInterval);
            break;

        case THR_GetAttrConfirm_AttributeId_JoinLqiThreshold:
            res.pValue = &(evt->AttributeValue.JoinLqiThreshold);
            break;

        case THR_GetAttrConfirm_AttributeId_ProvisioningURL:
            res.pValue = evt->AttributeValue.ProvisioningURL;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_SelectBestChannelEDThreshold:
            res.pValue = &(evt->AttributeValue.SelectBestChannelEDThreshold);
            break;

        case THR_GetAttrConfirm_AttributeId_CommissionerMode:
            res.pValue = &(evt->AttributeValue.CommissionerMode);
            break;

        case THR_GetAttrConfirm_AttributeId_BorderRouter_BrPrefixEntry:
            res.pValue = &(evt->AttributeValue.BorderRouter_BrPrefixEntry);
            break;

        case THR_GetAttrConfirm_AttributeId_SteeringData:
            res.pValue = evt->AttributeValue.SteeringData;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_Security_KeySwitchGuardTime:
            res.pValue = &(evt->AttributeValue.Security_KeySwitchGuardTime);
            break;

        case THR_GetAttrConfirm_AttributeId_ParentHoldTime:
            res.pValue = &(evt->AttributeValue.ParentHoldTime);
            break;

        case THR_GetAttrConfirm_AttributeId_Security_Policy:
            res.pValue = &(evt->AttributeValue.Security_Policy);
            break;

        case THR_GetAttrConfirm_AttributeId_NVM_RestoreAutoStart:
            res.pValue = &(evt->AttributeValue.NVM_RestoreAutoStart);
            break;

        case THR_GetAttrConfirm_AttributeId_NVM_Restore:
            res.pValue = &(evt->AttributeValue.NVM_Restore);
            break;

        case THR_GetAttrConfirm_AttributeId_SlaacPolicy:
            res.pValue = &(evt->AttributeValue.SlaacPolicy);
            break;

        case THR_GetAttrConfirm_AttributeId_IeeeExtendedAddr:
            res.pValue = &(evt->AttributeValue.IeeeExtendedAddr);
            break;

        case THR_GetAttrConfirm_AttributeId_LeaderWeight:
            res.pValue = &(evt->AttributeValue.LeaderWeight);
            break;

        case THR_GetAttrConfirm_AttributeId_HashIeeeAddr:
            res.pValue = &(evt->AttributeValue.HashIeeeAddr);
            break;

        case THR_GetAttrConfirm_AttributeId_BorderRouter_BrGlobalOnMeshPrefix:
            res.pValue = &(evt->AttributeValue.BorderRouter_BrGlobalOnMeshPrefix);
            break;

        case THR_GetAttrConfirm_AttributeId_BorderRouter_BrDefaultRouteOnMeshPrefix:
            res.pValue = &(evt->AttributeValue.BorderRouter_BrDefaultRouteOnMeshPrefix);
            break;

        case THR_GetAttrConfirm_AttributeId_BorderRouter_BrExternalIfPrefix:
            res.pValue = &(evt->AttributeValue.BorderRouter_BrExternalIfPrefix);
            break;

        case THR_GetAttrConfirm_AttributeId_ActiveTimestamp:
            res.pValue = &(evt->AttributeValue.ActiveTimestamp);
            break;

        case THR_GetAttrConfirm_AttributeId_CommissionerId:
            res.pValue = evt->AttributeValue.CommissionerID;
            res.requireFree = TRUE;
            break;

        case THR_GetAttrConfirm_AttributeId_JoinerPort:
            res.pValue = &(evt->AttributeValue.JoinerPort);
            break;

        case THR_GetAttrConfirm_AttributeId_CommissionerUdpPort:
            res.pValue = &(evt->AttributeValue.CommissionerUdpPort);
            break;

        default:
            break;
    }

    return res;
}


static void SHELL_PrintGetAttrRsp
(
    THR_GetAttrConfirm_t *evt
)
{
    uint64_t value = 0;
    char strValue[64] = { 0 };

    attrRef_t attrRef = SHELL_GetAttrRefFromId(evt);

    if (attrRef.pValue == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < NumberOfElements(aShellThrAttr); i++)
    {
        if (aShellThrAttr[i].attrId == evt->AttributeId)
        {
            /* check if a function handles this */
            if (NULL != aShellThrAttr[i].pfHandleAttr)
            {
                aShellThrAttr[i].pfHandleAttr(attrRef.pValue);
            }

            else
            {
                switch (aShellThrAttr[i].valueType)
                {
                    case gDecimal_c:
                        FLib_MemCpy(&value, attrRef.pValue, evt->AttrSize);
                        shell_printf("%s: %d\n\r", aShellThrAttr[i].pAttrName, value);
                        break;

                    case gHex_c:
                        FLib_MemCpy(&value, attrRef.pValue, evt->AttrSize);

                        if (evt->AttrSize <= 4)
                        {
                            shell_printf("%s: 0x%04x\n\r", aShellThrAttr[i].pAttrName, value);
                        }
                        else
                        {
                            shell_printf("%s: 0x%08x\n\r", aShellThrAttr[i].pAttrName, value);
                        }

                        break;

                    case gArray_c:
                        shell_printf("%s: 0x", aShellThrAttr[i].pAttrName);
                        SHELL_PrintBuff((uint8_t *)attrRef.pValue, evt->AttrSize);
                        shell_printf("\n\r");
                        break;

                    case gString_c:

                        if (aShellThrAttr[i].attrId == THR_GetAttrConfirm_AttributeId_Security_PSKc)
                        {
                            shell_printf("%s: 0x", "Binary PSKc");
                            SHELL_PrintBuff((uint8_t *)attrRef.pValue, evt->AttrSize);
                            shell_printf("\n\r");
                        }
                        else
                        {
                            FLib_MemCpy(strValue, attrRef.pValue, evt->AttrSize);
                            shell_printf("%s: %s\n\r", aShellThrAttr[i].pAttrName, strValue);
                        }

                        break;

                    case gHexReversed_c:
                    {
                        FLib_MemCpy(&value, attrRef.pValue, evt->AttrSize);
                        NWKU_SwapArrayBytes((uint8_t *)&value, 8);
                        shell_printf("%s: 0x", aShellThrAttr[i].pAttrName);
                        SHELL_PrintBuff((uint8_t *)&value, 8);
                        shell_printf("\n\r");
                    }
                    break;

                    default:
                        break;
                }
            }

            break;
        }
    }

    if (attrRef.requireFree)
    {
        MEM_BufferFree(attrRef.pValue);
    }
}


/*!*************************************************************************************************
\private
\fn     static void SHELL_ThrStartCommissioner(void *param)
\brief  This function is used to start a commissioner.

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_ThrStartCommissioner
(
    void *param
)
{
    MESHCOP_StartCommissionerRequest_t req;
    req.InstanceId = threadInstanceID;

    if (MESHCOP_StartCommissionerRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("No memory!\n\r");
    }
}

/*!************************************************************************************************
*
\private
\fn     static void SHELL_ThrStopCommissioner(void *param)
\brief  This function is used to stop a commissioner..

\param  [in]    param    Not used
***************************************************************************************************/
static void SHELL_ThrStopCommissioner
(
    void *param
)
{
    MESHCOP_StopCommissionerRequest_t req;
    req.InstanceId = threadInstanceID;

    if (MESHCOP_StopCommissionerRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("No memory!\n\r");
    }
}

/*!************************************************************************************************
*
\private
\fn     static void SHELL_PrintCoapMsg(NWKU_CoapMsgReceivedIndication_t *evt)
\brief  This function is used to print info about a received CoAP message.

\param  [in]    param    NWKU_CoapMsgReceivedIndication_t
***************************************************************************************************/
static void SHELL_PrintCoapMsg
(
    NWKU_CoapMsgReceivedIndication_t *evt
)
{
    SHELL_NEWLINE();
    shell_write("CoAP Message Received -> ");
    shell_write(evt->URIpath);

    if (evt->PayloadLength > 0)
    {
        shell_write("payload -> ");
        shell_writeN((char *)evt->Payload, evt->PayloadLength);
        shell_write(", hex -> ");
        SHELL_PrintBuff(evt->Payload, evt->PayloadLength);
    }
}

/*!************************************************************************************************
*
\private
\fn     static void SHELL_PrintMcastGroups(NWKU_McastGroupShowResponse_t *evt)
\brief  This function is used to print info about the registered multicast groups.

\param  [in]    param    NWKU_McastGroupShowResponse_t
***************************************************************************************************/
static void SHELL_PrintMcastGroups
(
    NWKU_McastGroupShowResponse_t *evt
)
{
    char groupStr[INET6_ADDRSTRLEN] = { 0 };
    ipAddr_t temp;

    if (evt->CountIpAddresses == 0)
    {
        SHELL_NEWLINE();
        shell_write("No multicast groups registered!\n\r");
    }

    for (uint8_t i = 0; i < evt->CountIpAddresses; i++)
    {
        FLib_MemCpyReverseOrder(temp.addr8, &evt->Addresses[sizeof(ipAddr_t) * i], sizeof(ipAddr_t));
        ntop(AF_INET6, &temp, groupStr, INET6_ADDRSTRLEN);
        shell_printf("\n\r%s", groupStr);
        FLib_MemSet(groupStr, 0, INET6_ADDRSTRLEN);
    }

    MEM_BufferFree(evt->Addresses);
}

/*!*************************************************************************************************
\private
\fn     static int8_t SHELL_Reboot(uint8_t argc, char *argv[])
\brief  This function is used to reset the device.

\param  [in]    argc      Number of arguments the command was called with
\param  [in]    argv      Pointer to a list of pointers to the arguments

\return         int8_t    Status of the command
***************************************************************************************************/
static int8_t SHELL_Reboot
(
    uint8_t argc,
    char *argv[]
)
{
    THR_CpuResetRequest_t req;
    req.TimeoutMs = 0;

    if (THR_CpuResetRequest(&req, THR_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("No memory!\n\r");
    }

    return CMD_RET_SUCCESS;
}


static void SHELL_ThrSetChannel(void *param)
{
    THR_SetAttrRequest_t req;
    req.InstanceId = threadInstanceID;
    req.AttributeId = THR_SetAttrRequest_AttributeId_Channel;
    req.Index = 0;
    req.AttributeValue.Channel.AttrSize = 1;
    req.AttributeValue.Channel.Value = mAutostartChannel;
    shell_printf("\r\nSet channel to %d", mAutostartChannel);
    THR_SetAttrRequest(&req, THR_FSCI_IF);
}

/*==================================================================================================
                                        START OF GENERATED CODE
==================================================================================================*/
static char *const OK = "OK";
static char *const ERROR = "ERROR";
static char *const Success = "Success";
static char *const Notpermitted = "Notpermitted";
static char *const Nomemory = "Nomemory";
static char *const Invalidinstance = "Invalidinstance";
static char *const Invalidparameter = "Invalidparameter";
static char *const UnsupportedAttribute = "UnsupportedAttribute";
static char *const EmptyEntry = "EmptyEntry";
static char *const InvalidValue = "InvalidValue";
static char *const InvalidParam = "InvalidParam";
static char *const Theselectedconfigurationisnotvalid = "Theselectedconfigurationisnotvalid";
static char *const AlreadyConnected = "AlreadyConnected";
static char *const AlreadyCreated = "AlreadyCreated";
static char *const ResetCpuSuccess = "ResetCpuSuccess";
static char *const ResetCpuPending = "ResetCpuPending";
static char *const ScanResult = "ScanResult";
static char *const EnergyDetect = "EnergyDetect";
static char *const ActiveScan = "ActiveScan";
static char *const EnergyDetectAndActiveScan = "EnergyDetectAndActiveScan";
static char *const Failed = "Failed";
static char *const SelectBestChannel = "SelectBestChannel";
static char *const GeneratePSKc = "GeneratePSKc";
static char *const Attaching = "Attaching";
static char *const JoinSuccess = "JoinSuccess";
static char *const JoinFailed = "JoinFailed";
static char *const ScanStarted = "ScanStarted";
static char *const ReceivedBeacon = "ReceivedBeacon";
static char *const ScanEnded = "ScanEnded";
static char *const Disconnected = "Disconnected";
static char *const Connected = "Connected";
static char *const Resettofactorydefault = "Resettofactorydefault";
static char *const Instancerestorestarted = "Instancerestorestarted";
static char *const RouterSynced = "RouterSynced";
static char *const EndDeviceSynced = "EndDeviceSynced";
static char *const Connectingstarted = "Connectingstarted";
static char *const Connectingfailed = "Connectingfailed";
static char *const Connectingdeffered = "Connectingdeffered";
static char *const DeviceisLeader = "DeviceisLeader";
static char *const DeviceisRouter = "DeviceisRouter";
static char *const DeviceisREED = "DeviceisREED";
static char *const DeviceisEndDevice = "DeviceisEndDevice";
static char *const DeviceisSleepyEndDevice = "DeviceisSleepyEndDevice";
static char *const RequestingGlobalAddress = "RequestingGlobalAddress";
static char *const GlobalAddressassigned = "GlobalAddressassigned";
static char *const RequestingRouterId = "RequestingRouterId";
static char *const RouterIdassigned = "RouterIdassigned";
static char *const RouterIdassignedfailed = "RouterIdassignedfailed";
static char *const Allowdevicetosleep = "Allowdevicetosleep";
static char *const Disallowdevicetosleep = "Disallowdevicetosleep";
static char *const ChildIdassigned = "ChildIdassigned";
static char *const JoinerDiscoveryStarted = "JoinerDiscoveryStarted";
static char *const JoinerDiscoveryFailedNoBeacon = "JoinerDiscoveryFailedNoBeacon";
static char *const JoinerDiscoveryFailedFiltered = "JoinerDiscoveryFailedFiltered";
static char *const JoinerDiscoverySuccess = "JoinerDiscoverySuccess";
static char *const JoinerDtlsSessionStarted = "JoinerDtlsSessionStarted";
static char *const JoinerDtlsError = "JoinerDtlsError";
static char *const JoinerError = "JoinerError";
static char *const JoinerAccepted = "JoinerAccepted";
static char *const CommissionerPetitionStarted = "CommissionerPetitionStarted";
static char *const CommissionerPetitionAccepted = "CommissionerPetitionAccepted";
static char *const CommissionerPetitionRejected = "CommissionerPetitionRejected";
static char *const CommissionerPetitionError = "CommissionerPetitionError";
static char *const CommissionerKeepAliveSent = "CommissionerKeepAliveSent";
static char *const CommissionerError = "CommissionerError";
static char *const CommissionerJoinerDtlsSessionStarted = "CommissionerJoinerDtlsSessionStarted";
static char *const CommissionerJoinerDtlsError = "CommissionerJoinerDtlsError";
static char *const CommissionerJoinerAccepted = "CommissionerJoinerAccepted";
static char *const CommissionerNwkDataSynced = "CommissionerNwkDataSynced";
static char *const CommissionerBrDtlsSessionStarted = "CommissionerBrDtlsSessionStarted";
static char *const CommissionerBrDtlsError = "CommissionerBrDtlsError";
static char *const CommissionerBrError = "CommissionerBrError";
static char *const CommissionerBrAccepted = "CommissionerBrAccepted";
static char *const BrCommissionerDtlsSessionStarted = "BrCommissionerDtlsSessionStarted";
static char *const BrCommissionerDtlsError = "BrCommissionerDtlsError";
static char *const BrCommissionerAccepted = "BrCommissionerAccepted";
static char *const BrCommissionerDataRelayedInbound = "BrCommissionerDataRelayedInbound";
static char *const BrCommissionerDataRelayedOutbound = "BrCommissionerDataRelayedOutbound";
static char *const JoinerrouterJoinerDataRelayedInbound = "JoinerrouterJoinerDataRelayedInbound";
static char *const JoinerrouterJoinerDataRelayedOutbound = "JoinerrouterJoinerDataRelayedOutbound";
static char *const JoinerrouterJoinerAccepted = "JoinerrouterJoinerAccepted";
static char *const StartVendorProvisioning = "StartVendorProvisioning";
static char *const OUT = "OUT";
static char *const IN = "IN";
static char *const JOIN_FIN_REQ = "JOIN_FIN_REQ";
static char *const JOIN_FIN_RSP = "JOIN_FIN_RSP";
static char *const JOIN_ENT_REQ = "JOIN_ENT_REQ";
static char *const JOIN_ENT_RSP = "JOIN_ENT_RSP";
static char *const DTLS_CLOSE_NOTIFY = "DTLS_CLOSE_NOTIFY";
static char *const InvalidParameter = "InvalidParameter";
static char *const NotPermitted = "NotPermitted";
static char *const NoMemory = "NoMemory";
static char *const Error = "Error";
static char *const InvalidInstance = "InvalidInstance";
static char *const NotStarted = "NotStarted";
static char *const FailedNotsupported = "FailedNotsupported";
static char *const FailedNotallowed = "FailedNotallowed";
static char *const NeighborNotFound = "NeighborNotFound";
static char *const Addressesmaximumlimitreached = "Addressesmaximumlimitreached";
static char *const RequestTimeout = "RequestTimeout";
static char *const Wrongdestinationaddress = "Wrongdestinationaddress";
static char *const Duplicate = "Duplicate";
static char *const FALSE_str = "FALSE";
static char *const TRUE_str = "TRUE";

/*!*************************************************************************************************
\fn     void SHELL_ThrEventNotify(void *param)
\brief  Print to host's shell information about Thread events coming from black-box

\param  [in]    param       the received event
***************************************************************************************************/
void SHELL_ThrEventNotify(void *param)
{
    thrEvtContainer_t *container = (thrEvtContainer_t *)param;

    SHELL_NEWLINE();

    switch (container->id)
    {
        case 0xCF00:
            shell_write("SocketCreateConfirm");
            break;

        case 0xCF01:
            shell_write("SocketShutdownConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketShutdownConfirm.Status)
            {
                case SocketShutdownConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketShutdownConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketShutdownConfirm.Status);
                    break;
            }

            break;

        case 0xCF02:
            shell_write("SocketBindConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketBindConfirm.Status)
            {
                case SocketBindConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketBindConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketBindConfirm.Status);
                    break;
            }

            break;

        case 0xCF03:
            shell_write("SocketSendConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketSendConfirm.Status)
            {
                case SocketSendConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketSendConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketSendConfirm.Status);
                    break;
            }

            break;

        case 0xCF04:
            shell_write("SocketSendToConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketSendToConfirm.Status)
            {
                case SocketSendToConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketSendToConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketSendToConfirm.Status);
                    break;
            }

            break;

        case 0xCF05:
            shell_write("SocketReceiveConfirm");
            break;

        case 0xCF06:
            shell_write("SocketReceiveFromConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketReceiveFromConfirm.Status)
            {
                case SocketReceiveFromConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketReceiveFromConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketReceiveFromConfirm.Status);
                    break;
            }

            break;

        case 0xCF07:
            shell_write("SocketConnectConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketConnectConfirm.Status)
            {
                case SocketConnectConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketConnectConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketConnectConfirm.Status);
                    break;
            }

            break;

        case 0xCF08:
            shell_write("SocketListenConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketListenConfirm.Status)
            {
                case SocketListenConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketListenConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketListenConfirm.Status);
                    break;
            }

            break;

        case 0xCF09:
            shell_write("SocketAcceptConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketAcceptConfirm.Status)
            {
                case SocketAcceptConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketAcceptConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketAcceptConfirm.Status);
                    break;
            }

            break;

        case 0xCF0A:
            shell_write("SocketSetOptionConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketSetOptionConfirm.Status)
            {
                case SocketSetOptionConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketSetOptionConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketSetOptionConfirm.Status);
                    break;
            }

            break;

        case 0xCF0B:
            shell_write("SocketGetOptionConfirm");
            shell_write(" -> ");

            switch (container->Data.SocketGetOptionConfirm.Status)
            {
                case SocketGetOptionConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case SocketGetOptionConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.SocketGetOptionConfirm.Status);
                    break;
            }

            break;

        case 0xCF12:
            shell_write("MAC_MacFilteringAddEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.MAC_MacFilteringAddEntryConfirm.Status)
            {
                case MAC_MacFilteringAddEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MAC_MacFilteringAddEntryConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case MAC_MacFilteringAddEntryConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MAC_MacFilteringAddEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF13:
            shell_write("MAC_MacFilteringRemoveEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.MAC_MacFilteringRemoveEntryConfirm.Status)
            {
                case MAC_MacFilteringRemoveEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MAC_MacFilteringRemoveEntryConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MAC_MacFilteringRemoveEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF14:
            shell_write("MAC_MacFilteringEnableConfirm");
            shell_write(" -> ");

            switch (container->Data.MAC_MacFilteringEnableConfirm.Status)
            {
                case MAC_MacFilteringEnableConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MAC_MacFilteringEnableConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MAC_MacFilteringEnableConfirm.Status);
                    break;
            }

            break;

        case 0xCF15:
            shell_write("MAC_MacFilteringGetTableConfirm");
            break;

        case 0xCF16:
            shell_write("THR_SetDeviceConfigConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SetDeviceConfigConfirm.Status)
            {
                case THR_SetDeviceConfigConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_SetDeviceConfigConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_SetDeviceConfigConfirm_Status_Invalidparameter:
                    shell_write(Invalidparameter);
                    break;

                case THR_SetDeviceConfigConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_SetDeviceConfigConfirm_Status_UnsupportedAttribute:
                    shell_write(UnsupportedAttribute);
                    break;

                case THR_SetDeviceConfigConfirm_Status_EmptyEntry:
                    shell_write(EmptyEntry);
                    break;

                case THR_SetDeviceConfigConfirm_Status_InvalidValue:
                    shell_write(InvalidValue);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SetDeviceConfigConfirm.Status);
                    break;
            }

            break;

        case 0xCF1A:
            shell_write("THR_NwkScanConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_NwkScanConfirm.Status)
            {
                case THR_NwkScanConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_NwkScanConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_NwkScanConfirm_Status_InvalidParam:
                    shell_write(InvalidParam);
                    break;

                case THR_NwkScanConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_NwkScanConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_NwkScanConfirm.Status);
                    break;
            }

            break;

        case 0xCF1B:
            shell_write("THR_CreateNwkConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_CreateNwkConfirm.Status)
            {
                case THR_CreateNwkConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_CreateNwkConfirm_Status_AlreadyConnected:
                    shell_write(AlreadyConnected);
                    break;

                case THR_CreateNwkConfirm_Status_AlreadyCreated:
                    shell_write(AlreadyCreated);
                    break;

                case THR_CreateNwkConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_CreateNwkConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_CreateNwkConfirm.Status);
                    break;
            }

            break;

        case 0xCF1C:
            shell_write("THR_JoinConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_JoinConfirm.Status)
            {
                case THR_JoinConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_JoinConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_JoinConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_JoinConfirm.Status);
                    break;
            }

            break;

        case 0xCF1F:
            shell_write("THR_FactoryResetConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_FactoryResetConfirm.Status)
            {
                case THR_FactoryResetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_FactoryResetConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_FactoryResetConfirm.Status);
                    break;
            }

            break;

        case 0xCF21:
            shell_write("THR_CpuResetConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_CpuResetConfirm.Status)
            {
                case THR_CpuResetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_CpuResetConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_CpuResetConfirm.Status);
                    break;
            }

            break;

        case 0xCF22:
            shell_write("THR_CpuResetIndication");
            shell_write(" -> ");

            switch (container->Data.THR_CpuResetIndication.Status)
            {
                case THR_CpuResetIndication_Status_ResetCpuSuccess:
                    shell_write(ResetCpuSuccess);
                    MEM_BufferFree(container->Data.THR_CpuResetIndication.ResetCpuPayload.ResetCpuSuccess.BoardName);
                    MEM_BufferFree(container->Data.THR_CpuResetIndication.ResetCpuPayload.ResetCpuSuccess.SwVersion);

                    /* invalidate the CoAP instance ID */
                    mCoapInstId = THR_ALL_FFs8;
#if gHybridApp_d
                    BleApp_Terminate();
#endif

                    break;

                case THR_CpuResetIndication_Status_ResetCpuPending:
                    shell_write(ResetCpuPending);
                    shell_printf(" in %u milliseconds\n", container->Data.THR_CpuResetIndication.ResetCpuPayload.ResetCpuPending_TimeoutMs);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_CpuResetIndication.Status);
                    break;
            }

            break;

        case 0xCF1D:
            shell_write("THR_DisconnectConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_DisconnectConfirm.Status)
            {
                case THR_DisconnectConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_DisconnectConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_DisconnectConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_DisconnectConfirm.Status);
                    break;
            }

            break;

        case 0xCF80:
            shell_write("THR_AttachConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_AttachConfirm.Status)
            {
                case THR_AttachConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_AttachConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_AttachConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_AttachConfirm.Status);
                    break;
            }

            break;

        case 0xCF94:
            shell_write("THR_PromoteAsRouterConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_PromoteAsRouterConfirm.Status)
            {
                case THR_PromoteAsRouterConfirm_Status_OK:
                    shell_write(OK);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_PromoteAsRouterConfirm.Status);
                    break;
            }

            break;

        case 0xCF50:
            shell_write("THR_EventNwkScanConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_EventNwkScanConfirm.EventStatus)
            {
                case THR_EventNwkScanConfirm_EventStatus_ScanResult:
                    shell_write(ScanResult);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventNwkScanConfirm.EventStatus);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.THR_EventNwkScanConfirm.ScanType)
            {
                case THR_EventNwkScanConfirm_ScanType_EnergyDetect:
                    shell_write(EnergyDetect);
                    break;

                case THR_EventNwkScanConfirm_ScanType_ActiveScan:
                    shell_write(ActiveScan);
                    break;

                case THR_EventNwkScanConfirm_ScanType_EnergyDetectAndActiveScan:
                    shell_write(EnergyDetectAndActiveScan);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_EventNwkScanConfirm.ScanType);
                    break;
            }

            SHELL_NwkScanPrint(&container->Data.THR_EventNwkScanConfirm);
            break;

        case 0xCF51:
            shell_write("THR_EventNwkCreateConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_EventNwkCreateConfirm.EventStatus)
            {
                case THR_EventNwkCreateConfirm_EventStatus_Success:
                    shell_write(Success);
                    break;

                case THR_EventNwkCreateConfirm_EventStatus_Failed:
                    shell_write(Failed);
                    break;

                case THR_EventNwkCreateConfirm_EventStatus_SelectBestChannel:
                    shell_write(SelectBestChannel);
                    break;

                case THR_EventNwkCreateConfirm_EventStatus_GeneratePSKc:
                    shell_write(GeneratePSKc);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventNwkCreateConfirm.EventStatus);
                    break;
            }

            MEM_BufferFree(container->Data.THR_EventNwkCreateConfirm.Data);
            break;

        case 0xCF52:
            shell_write("THR_EventNwkJoinConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_EventNwkJoinConfirm.EventStatus)
            {
                case THR_EventNwkJoinConfirm_EventStatus_Attaching:
                    shell_write(Attaching);
                    break;

                case THR_EventNwkJoinConfirm_EventStatus_JoinSuccess:
                    shell_write(JoinSuccess);
                    break;

                case THR_EventNwkJoinConfirm_EventStatus_JoinFailed:
                    shell_write(JoinFailed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventNwkJoinConfirm.EventStatus);
                    break;
            }

            break;

        case 0xCF53:
            shell_write("THR_EventNwkJoinSelectParentsConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_EventNwkJoinSelectParentsConfirm.EventStatus)
            {
                case THR_EventNwkJoinSelectParentsConfirm_EventStatus_ScanStarted:
                    shell_write(ScanStarted);
                    break;

                case THR_EventNwkJoinSelectParentsConfirm_EventStatus_ReceivedBeacon:
                    shell_write(ReceivedBeacon);
                    break;

                case THR_EventNwkJoinSelectParentsConfirm_EventStatus_ScanEnded:
                    shell_write(ScanEnded);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventNwkJoinSelectParentsConfirm.EventStatus);
                    break;
            }

            break;

        case 0xCF54:
            shell_write("THR_EventGeneralConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_EventGeneralConfirm.EventStatus)
            {
                case THR_EventGeneralConfirm_EventStatus_Disconnected:
                    shell_write(Disconnected);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Connected:
                    shell_write(Connected);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Resettofactorydefault:
                    shell_write(Resettofactorydefault);

                    // on autostart, after factory reset set the channel
                    if (mAutostartInProgress && (mAutostartState == gAutostartFirstState_c))
                    {
                        mAutostartState = gFactoryResetState_c;

                        /* Start timer */
                        if (mDelayTimerID == gTmrInvalidTimerID_c)
                        {
                            mDelayTimerID = TMR_AllocateTimer();
                        }

                        /* send next command after a small delay - leave time for DHCPv6-PD */
                        TMR_StartSingleShotTimer(mDelayTimerID, 3000, SHELL_ThrSetChannel, NULL);
                    }

#if gHybridApp_d
                    BleApp_Terminate();
#endif

                    break;

                case THR_EventGeneralConfirm_EventStatus_Instancerestorestarted:
                    shell_write(Instancerestorestarted);
                    break;

                case THR_EventGeneralConfirm_EventStatus_RouterSynced:
                    shell_write(RouterSynced);
                    break;

                case THR_EventGeneralConfirm_EventStatus_EndDeviceSynced:
                    shell_write(EndDeviceSynced);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Connectingstarted:
                    shell_write(Connectingstarted);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Connectingfailed:
                    shell_write(Connectingfailed);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Connectingdeffered:
                    shell_write(Connectingdeffered);
                    break;

                case THR_EventGeneralConfirm_EventStatus_DeviceisLeader:
                    shell_write(DeviceisLeader);
                    break;

                case THR_EventGeneralConfirm_EventStatus_DeviceisRouter:
                    shell_write(DeviceisRouter);
                    break;

                case THR_EventGeneralConfirm_EventStatus_DeviceisREED:
                    shell_write(DeviceisREED);
                    break;

                case THR_EventGeneralConfirm_EventStatus_DeviceisEndDevice:
                    shell_write(DeviceisEndDevice);
                    break;

                case THR_EventGeneralConfirm_EventStatus_DeviceisSleepyEndDevice:
                    shell_write(DeviceisSleepyEndDevice);
                    break;

                case THR_EventGeneralConfirm_EventStatus_RequestingGlobalAddress:
                    shell_write(RequestingGlobalAddress);
                    break;

                case THR_EventGeneralConfirm_EventStatus_GlobalAddressassigned:
                    shell_write(GlobalAddressassigned);
                    break;

                case THR_EventGeneralConfirm_EventStatus_RequestingRouterId:
                    shell_write(RequestingRouterId);
                    break;

                case THR_EventGeneralConfirm_EventStatus_RouterIdassigned:
                    shell_write(RouterIdassigned);
                    break;

                case THR_EventGeneralConfirm_EventStatus_RouterIdassignedfailed:
                    shell_write(RouterIdassignedfailed);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Allowdevicetosleep:
                    shell_write(Allowdevicetosleep);
                    break;

                case THR_EventGeneralConfirm_EventStatus_Disallowdevicetosleep:
                    shell_write(Disallowdevicetosleep);
                    break;

                case THR_EventGeneralConfirm_EventStatus_ChildIdassigned:
                    shell_write(ChildIdassigned);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventGeneralConfirm.EventStatus);
                    break;
            }

            break;

        case 0xCF55:
            shell_write("THR_EventNwkCommissioningIndication");
            shell_write(" -> ");

            switch (container->Data.THR_EventNwkCommissioningIndication.EventStatus)
            {
                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryStarted:
                    shell_write(JoinerDiscoveryStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryFailedNoBeacon:
                    shell_write(JoinerDiscoveryFailedNoBeacon);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryFailedFiltered:
                    shell_write(JoinerDiscoveryFailedFiltered);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoverySuccess:
                    shell_write(JoinerDiscoverySuccess);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDtlsSessionStarted:
                    shell_write(JoinerDtlsSessionStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerDtlsError:
                    shell_write(JoinerDtlsError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerError:
                    shell_write(JoinerError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerAccepted:
                    shell_write(JoinerAccepted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionStarted:
                    shell_write(CommissionerPetitionStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionAccepted:
                    shell_write(CommissionerPetitionAccepted);

                    if (mAutostartInProgress && (mAutostartState == gFactoryResetState_c))
                    {
                        mAutostartState = gNetworkCreated_c;
                        shell_write("\r\nStarting Commissioner..");

                        /* Start timer */
                        if (mDelayTimerID == gTmrInvalidTimerID_c)
                        {
                            mDelayTimerID = TMR_AllocateTimer();
                        }

                        /* send next command after a small delay */
                        TMR_StartSingleShotTimer(mDelayTimerID, PING_DELAY, SHELL_ThrStartCommissioner, NULL);
                    }
                    else if (mAutostartInProgress && (mAutostartState == gNetworkCreated_c))
                    {
                        mAutostartState = gCommissionerStarted_c;
                        shell_write("\r\nAllowing all joiners with PSKd `THREAD`..");

                        MESHCOP_AddExpectedJoinerRequest_t req;
                        req.InstanceId = threadInstanceID;
                        req.Selected = TRUE;
                        req.EuiType = MESHCOP_AddExpectedJoinerRequest_EuiType_LongEUI;
                        req.EUI.LongEUI = THR_ALL_FFs64;
                        req.PSKdSize = FLib_StrLen("THREAD");
                        req.PSKd = "THREAD";

                        MESHCOP_AddExpectedJoinerRequest(&req, THR_FSCI_IF);
                    }

                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionRejected:
                    shell_write(CommissionerPetitionRejected);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionError:
                    shell_write(CommissionerPetitionError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerKeepAliveSent:
                    shell_write(CommissionerKeepAliveSent);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerError:
                    shell_write(CommissionerError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerDtlsSessionStarted:
                    shell_write(CommissionerJoinerDtlsSessionStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerDtlsError:
                    shell_write(CommissionerJoinerDtlsError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerAccepted:
                    shell_write(CommissionerJoinerAccepted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerNwkDataSynced:
                    shell_write(CommissionerNwkDataSynced);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrDtlsSessionStarted:
                    shell_write(CommissionerBrDtlsSessionStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrDtlsError:
                    shell_write(CommissionerBrDtlsError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrError:
                    shell_write(CommissionerBrError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrAccepted:
                    shell_write(CommissionerBrAccepted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDtlsSessionStarted:
                    shell_write(BrCommissionerDtlsSessionStarted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDtlsError:
                    shell_write(BrCommissionerDtlsError);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerAccepted:
                    shell_write(BrCommissionerAccepted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDataRelayedInbound:
                    shell_write(BrCommissionerDataRelayedInbound);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDataRelayedOutbound:
                    shell_write(BrCommissionerDataRelayedOutbound);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerDataRelayedInbound:
                    shell_write(JoinerrouterJoinerDataRelayedInbound);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerDataRelayedOutbound:
                    shell_write(JoinerrouterJoinerDataRelayedOutbound);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerAccepted:
                    shell_write(JoinerrouterJoinerAccepted);
                    break;

                case THR_EventNwkCommissioningIndication_EventStatus_StartVendorProvisioning:
                    shell_write(StartVendorProvisioning);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.THR_EventNwkCommissioningIndication.EventStatus);
                    break;
            }

            break;

        case 0xCF4E:
            shell_write("THR_CommissioningDiagnosticIndication");
            shell_write(" -> ");

            switch (container->Data.THR_CommissioningDiagnosticIndication.Direction)
            {
                case THR_CommissioningDiagnosticIndication_Direction_OUT:
                    shell_write(OUT);
                    break;

                case THR_CommissioningDiagnosticIndication_Direction_IN:
                    shell_write(IN);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_CommissioningDiagnosticIndication.Direction);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.THR_CommissioningDiagnosticIndication.Type)
            {
                case THR_CommissioningDiagnosticIndication_Type_JOIN_FIN_REQ:
                    shell_write(JOIN_FIN_REQ);
                    break;

                case THR_CommissioningDiagnosticIndication_Type_JOIN_FIN_RSP:
                    shell_write(JOIN_FIN_RSP);
                    break;

                case THR_CommissioningDiagnosticIndication_Type_JOIN_ENT_REQ:
                    shell_write(JOIN_ENT_REQ);
                    break;

                case THR_CommissioningDiagnosticIndication_Type_JOIN_ENT_RSP:
                    shell_write(JOIN_ENT_RSP);
                    break;

                case THR_CommissioningDiagnosticIndication_Type_DTLS_CLOSE_NOTIFY:
                    shell_write(DTLS_CLOSE_NOTIFY);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_CommissioningDiagnosticIndication.Type);
                    break;
            }

            MEM_BufferFree(container->Data.THR_CommissioningDiagnosticIndication.TlvsBytes);

            break;

        case 0xCF61:
            shell_write("THR_MgmtDiagnosticGetConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_MgmtDiagnosticGetConfirm.Status)
            {
                case THR_MgmtDiagnosticGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_MgmtDiagnosticGetConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_MgmtDiagnosticGetConfirm_Status_NotPermitted:
                    shell_write(NotPermitted);
                    break;

                case THR_MgmtDiagnosticGetConfirm_Status_NoMemory:
                    shell_write(NoMemory);
                    break;

                case THR_MgmtDiagnosticGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_MgmtDiagnosticGetConfirm.Status);
                    break;
            }

            break;

        case 0xCF67:
            shell_write("THR_DiagTestGetConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_DiagTestGetConfirm.Status)
            {
                case THR_DiagTestGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_DiagTestGetConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_DiagTestGetConfirm_Status_NotPermitted:
                    shell_write(NotPermitted);
                    break;

                case THR_DiagTestGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_DiagTestGetConfirm.Status);
                    break;
            }

            break;

        case 0xCF62:
            shell_write("THR_MgmtDiagnosticResetConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_MgmtDiagnosticResetConfirm.Status)
            {
                case THR_MgmtDiagnosticResetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_MgmtDiagnosticResetConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_MgmtDiagnosticResetConfirm_Status_NotPermitted:
                    shell_write(NotPermitted);
                    break;

                case THR_MgmtDiagnosticResetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_MgmtDiagnosticResetConfirm.Status);
                    break;
            }

            break;

        case 0xCF65:
            shell_write("THR_MgmtReadMemoryConfirm");
            break;

        case 0xCF66:
            shell_write("THR_MgmtWriteMemoryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_MgmtWriteMemoryConfirm.Status)
            {
                case THR_MgmtWriteMemoryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_MgmtWriteMemoryConfirm.Status);
                    break;
            }

            break;

        case 0xCF75:
            shell_write("THR_SetManualSlaacIIDConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SetManualSlaacIIDConfirm.Status)
            {
                case THR_SetManualSlaacIIDConfirm_Status_OK:
                    shell_write(OK);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SetManualSlaacIIDConfirm.Status);
                    break;
            }

            break;

        case 0xCF76:
            shell_write("THR_SendProactiveAddrNotifConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SendProactiveAddrNotifConfirm.Status)
            {
                case THR_SendProactiveAddrNotifConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_SendProactiveAddrNotifConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case THR_SendProactiveAddrNotifConfirm_Status_InvalidInstance:
                    shell_write(InvalidInstance);
                    break;

                case THR_SendProactiveAddrNotifConfirm_Status_NotStarted:
                    shell_write(NotStarted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SendProactiveAddrNotifConfirm.Status);
                    break;
            }

            break;

        case 0xCFC0:
            shell_write("THR_NwkDiscoveryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_NwkDiscoveryConfirm.Status)
            {
                case THR_NwkDiscoveryConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_NwkDiscoveryConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_NwkDiscoveryConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_NwkDiscoveryConfirm.Status);
                    break;
            }

            break;

        case 0xCFC1:
            shell_write("THR_NwkDiscoveryStopConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_NwkDiscoveryStopConfirm.Status)
            {
                case THR_NwkDiscoveryStopConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_NwkDiscoveryStopConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_NwkDiscoveryStopConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_NwkDiscoveryStopConfirm.Status);
                    break;
            }

            break;

        case 0xCFC2:
            shell_write("THR_SearchNwkWithAnounceConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SearchNwkWithAnounceConfirm.Status)
            {
                case THR_SearchNwkWithAnounceConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_SearchNwkWithAnounceConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_SearchNwkWithAnounceConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SearchNwkWithAnounceConfirm.Status);
                    break;
            }

            break;

        case 0xCF63:
            shell_write("THR_MgmtDiagnosticGetRspIndication");
            shell_write(" -> ");

            switch (container->Data.THR_MgmtDiagnosticGetRspIndication.Status)
            {
                case THR_MgmtDiagnosticGetRspIndication_Status_Success:
                    shell_write(Success);
                    break;

                case THR_MgmtDiagnosticGetRspIndication_Status_FailedNotsupported:
                    shell_write(FailedNotsupported);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_MgmtDiagnosticGetRspIndication.Status);
                    break;
            }

            break;

        case 0xCF68:
            shell_write("THR_DiagTestGetRspIndication");
            shell_write(" -> ");

            switch (container->Data.THR_DiagTestGetRspIndication.Status)
            {
                case THR_DiagTestGetRspIndication_Status_Success:
                    shell_write(Success);
                    break;

                case THR_DiagTestGetRspIndication_Status_FailedNotsupported:
                    shell_write(FailedNotsupported);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_DiagTestGetRspIndication.Status);
                    break;
            }

            break;

        case 0xCF64:
            shell_write("THR_MgmtDiagnosticResetRspIndication");
            shell_write(" -> ");

            switch (container->Data.THR_MgmtDiagnosticResetRspIndication.Status)
            {
                case THR_MgmtDiagnosticResetRspIndication_Status_Success:
                    shell_write(Success);
                    break;

                case THR_MgmtDiagnosticResetRspIndication_Status_FailedNotallowed:
                    shell_write(FailedNotallowed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_MgmtDiagnosticResetRspIndication.Status);
                    break;
            }

            break;

        case 0xCF2E:
            shell_write("THR_SetNwkIdTimeoutConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SetNwkIdTimeoutConfirm.Status)
            {
                case THR_SetNwkIdTimeoutConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_SetNwkIdTimeoutConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SetNwkIdTimeoutConfirm.Status);
                    break;
            }

            break;

        case 0xCF20:
            shell_write("THR_SetThresholdConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SetThresholdConfirm.Status)
            {
                case THR_SetThresholdConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_SetThresholdConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SetThresholdConfirm.Status);
                    break;
            }

            break;

        case 0xCF10:
            shell_write("THR_GetNeighborInfoConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_GetNeighborInfoConfirm.Status)
            {
                case THR_GetNeighborInfoConfirm_Status_Success:
                    SHELL_PrintNeighborInfo(&(container->Data.THR_GetNeighborInfoConfirm));
                    break;

                case THR_GetNeighborInfoConfirm_Status_NeighborNotFound:
                    shell_write(NeighborNotFound);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_GetNeighborInfoConfirm.Status);
                    break;
            }

            break;

        case 0xCF23:
            shell_write("THR_GetChildrenTableConfirm");
            break;

        case 0xCF24:
            SHELL_PrintNeighborTbl(&(container->Data.THR_GetNeighborTableConfirm));
            break;

        case 0xCF25:
            SHELL_PrintRoutingTbl(&(container->Data.THR_GetRoutingTableConfirm));
            break;

        case 0xCF17:
            shell_write("THR_GetAttrConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_GetAttrConfirm.Status)
            {
                case THR_GetAttrConfirm_Status_Success:
                    SHELL_PrintGetAttrRsp(&container->Data.THR_GetAttrConfirm);
                    break;

                case THR_GetAttrConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_GetAttrConfirm_Status_Invalidparameter:
                    shell_write(Invalidparameter);
                    break;

                case THR_GetAttrConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_GetAttrConfirm_Status_UnsupportedAttribute:
                    shell_write(UnsupportedAttribute);
                    break;

                case THR_GetAttrConfirm_Status_EmptyEntry:
                    shell_write(EmptyEntry);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_GetAttrConfirm.Status);
                    break;
            }

            break;

        case 0xCF18:
            shell_write("THR_SetAttrConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SetAttrConfirm.Status)
            {
                case THR_SetAttrConfirm_Status_Success:
                    shell_write(Success);

                    if (mAutostartInProgress)
                    {
                        shell_write("\r\nCreating the network..");
                        SHELL_ThrNwkCreate(NULL);
                    }

                    break;

                case THR_SetAttrConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_SetAttrConfirm_Status_Invalidparameter:
                    shell_write(Invalidparameter);
                    break;

                case THR_SetAttrConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_SetAttrConfirm_Status_UnsupportedAttribute:
                    shell_write(UnsupportedAttribute);
                    break;

                case THR_SetAttrConfirm_Status_EmptyEntry:
                    shell_write(EmptyEntry);
                    break;

                case THR_SetAttrConfirm_Status_InvalidValue:
                    shell_write(InvalidValue);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SetAttrConfirm.Status);
                    break;
            }

            break;

        case 0xCF19:
            shell_write("THR_GetThreadIpAddrConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_GetThreadIpAddrConfirm.Status)
            {
                case THR_GetThreadIpAddrConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_GetThreadIpAddrConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case THR_GetThreadIpAddrConfirm_Status_InvalidInstance:
                    shell_write(InvalidInstance);
                    break;

                case THR_GetThreadIpAddrConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_GetThreadIpAddrConfirm.Status);
                    break;
            }

            break;

        case 0xCF1E:
            shell_write("THR_GetParentConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_GetParentConfirm.Status)
            {
                case THR_GetParentConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_GetParentConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case THR_GetParentConfirm_Status_InvalidInstance:
                    shell_write(InvalidInstance);
                    break;

                case THR_GetParentConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_GetParentConfirm.Status);
                    break;
            }

            break;

        case 0xCF2F:
            shell_write("THR_ChildUpdateToParentConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_ChildUpdateToParentConfirm.Status)
            {
                case THR_ChildUpdateToParentConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_ChildUpdateToParentConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_ChildUpdateToParentConfirm_Status_NotPermitted:
                    shell_write(NotPermitted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_ChildUpdateToParentConfirm.Status);
                    break;
            }

            break;

        case 0xCF26:
            shell_write("THR_LeaderRemoveRouterIdConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_LeaderRemoveRouterIdConfirm.Status)
            {
                case THR_LeaderRemoveRouterIdConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_LeaderRemoveRouterIdConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_LeaderRemoveRouterIdConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_LeaderRemoveRouterIdConfirm.Status);
                    break;
            }

            break;

        case 0xCF28:
            shell_write("THR_GenerateAllKeysConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_GenerateAllKeysConfirm.Status)
            {
                case THR_GenerateAllKeysConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_GenerateAllKeysConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_GenerateAllKeysConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_GenerateAllKeysConfirm.Status);
                    break;
            }

            break;

        case 0xCF27:
            shell_write("THR_SwitchKeyKeyConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_SwitchKeyKeyConfirm.Status)
            {
                case THR_SwitchKeyKeyConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case THR_SwitchKeyKeyConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_SwitchKeyKeyConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_SwitchKeyKeyConfirm.Status);
                    break;
            }

            break;

        case 0xCF29:
            shell_write("THR_BrPrefixAddEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrPrefixAddEntryConfirm.Status)
            {
                case THR_BrPrefixAddEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrPrefixAddEntryConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrPrefixAddEntryConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_BrPrefixAddEntryConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrPrefixAddEntryConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrPrefixAddEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF2A:
            shell_write("THR_BrPrefixGetTableConfirm");
            break;

        case 0xCF2B:
            shell_write("THR_BrPrefixRemoveEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrPrefixRemoveEntryConfirm.Status)
            {
                case THR_BrPrefixRemoveEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrPrefixRemoveEntryConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrPrefixRemoveEntryConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_BrPrefixRemoveEntryConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrPrefixRemoveEntryConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrPrefixRemoveEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF86:
            shell_write("THR_BrServiceRemoveEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrServiceRemoveEntryConfirm.Status)
            {
                case THR_BrServiceRemoveEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrServiceRemoveEntryConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrServiceRemoveEntryConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_BrServiceRemoveEntryConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrServiceRemoveEntryConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrServiceRemoveEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF85:
            shell_write("THR_BrServiceAddEntryConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrServiceAddEntryConfirm.Status)
            {
                case THR_BrServiceAddEntryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrServiceAddEntryConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrServiceAddEntryConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_BrServiceAddEntryConfirm_Status_NotPermitted:
                    shell_write(NotPermitted);
                    break;

                case THR_BrServiceAddEntryConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrServiceAddEntryConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrServiceAddEntryConfirm.Status);
                    break;
            }

            break;

        case 0xCF2C:
            shell_write("THR_BrPrefixSyncConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrPrefixSyncConfirm.Status)
            {
                case THR_BrPrefixSyncConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrPrefixSyncConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrPrefixSyncConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_BrPrefixSyncConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_BrPrefixSyncConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrPrefixSyncConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrPrefixSyncConfirm.Status);
                    break;
            }

            break;

        case 0xCF2D:
            shell_write("THR_BrPrefixRemoveAllConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_BrPrefixRemoveAllConfirm.Status)
            {
                case THR_BrPrefixRemoveAllConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case THR_BrPrefixRemoveAllConfirm_Status_Invalidinstance:
                    shell_write(Invalidinstance);
                    break;

                case THR_BrPrefixRemoveAllConfirm_Status_InvalidParameter:
                    shell_write(InvalidParameter);
                    break;

                case THR_BrPrefixRemoveAllConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                case THR_BrPrefixRemoveAllConfirm_Status_Nomemory:
                    shell_write(Nomemory);
                    break;

                case THR_BrPrefixRemoveAllConfirm_Status_Theselectedconfigurationisnotvalid:
                    shell_write(Theselectedconfigurationisnotvalid);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_BrPrefixRemoveAllConfirm.Status);
                    break;
            }

            break;

        case 0xCF69:
            shell_write("THR_IdentifyConfirm");
            shell_write(" -> ");

            switch (container->Data.THR_IdentifyConfirm.Status)
            {
                case THR_IdentifyConfirm_Status_Success:
                    shell_write(Success);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.THR_IdentifyConfirm.Status);
                    break;
            }

            break;

        case 0xCF0C:
            shell_write("NWKU_IfconfigBindConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_IfconfigBindConfirm.Status)
            {
                case NWKU_IfconfigBindConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case NWKU_IfconfigBindConfirm_Status_Addressesmaximumlimitreached:
                    shell_write(Addressesmaximumlimitreached);
                    break;

                case NWKU_IfconfigBindConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_IfconfigBindConfirm.Status);
                    break;
            }

            break;

        case 0xCF0D:
            SHELL_PrintIfconfigAllRsp(&container->Data.NWKU_IfconfigAllResponse);
            break;

        case 0xCF0E:
            switch (container->Data.NWKU_PingConfirm.Status)
            {
                case NWKU_PingConfirm_Status_OK:
                    PING_EchoReplyReceive(NULL);
                    break;

                case NWKU_PingConfirm_Status_RequestTimeout:
                    PING_HandleTimeout(NULL);
                    break;

                case NWKU_PingConfirm_Status_Wrongdestinationaddress:
                    shell_write(Wrongdestinationaddress);
                    break;

                case NWKU_PingConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_PingConfirm.Status);
                    break;
            }

            break;

        case 0xCF70:
            shell_write("NWKU_EchoUDPConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_EchoUDPConfirm.Status)
            {
                case NWKU_EchoUDPConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case NWKU_EchoUDPConfirm_Status_RequestTimeout:
                    shell_write(RequestTimeout);
                    break;

                case NWKU_EchoUDPConfirm_Status_Wrongdestinationaddress:
                    shell_write(Wrongdestinationaddress);
                    break;

                case NWKU_EchoUDPConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_EchoUDPConfirm.Status);
                    break;
            }

            break;

        case 0xCF92:
            switch (container->Data.NWKU_CoapMsgReceivedIndication.Status)
            {
                case NWKU_CoapMsgReceivedIndication_Status_Success:
                    if (NWKU_CoapMsgReceivedIndication_MessageType_GET ==
                        container->Data.NWKU_CoapMsgReceivedIndication.MessageType ||
                        NWKU_CoapMsgReceivedIndication_MessageType_POST ==
                        container->Data.NWKU_CoapMsgReceivedIndication.MessageType)
                    {
                         SHELL_CoapReply(&container->Data.NWKU_CoapMsgReceivedIndication);
                    }
                    SHELL_PrintCoapMsg(&(container->Data.NWKU_CoapMsgReceivedIndication));
                    break;

                case NWKU_CoapMsgReceivedIndication_Status_Failed:
                    shell_write(Failed);
                    break;

                case NWKU_CoapMsgReceivedIndication_Status_Duplicate:
                    shell_write(Duplicate);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_CoapMsgReceivedIndication.Status);
                    break;
            }

            MEM_BufferFree(container->Data.NWKU_CoapMsgReceivedIndication.Payload);

            break;

        case 0xCF96:
            shell_write("NWKU_DnsResponseReceivedIndication");
            break;

        case 0xCF91:
            shell_write("NWKU_CoapRegisterConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_CoapRegisterConfirm.Status)
            {
                case NWKU_CoapRegisterConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case NWKU_CoapRegisterConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_CoapRegisterConfirm.Status);
                    break;
            }

            break;

        case 0xCF93:
            shell_write("NWKU_CoapCreateInstanceConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_CoapCreateInstanceConfirm.Status)
            {
                case NWKU_CoapCreateInstanceConfirm_Status_Success:
                    shell_write(Success);
                    mCoapInstId = 0;
                    break;

                case NWKU_CoapCreateInstanceConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_CoapCreateInstanceConfirm.Status);
                    break;
            }

            break;

        case 0xCF90:
            shell_write("NWKU_CoapSendConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_CoapSendConfirm.Status)
            {
                case NWKU_CoapSendConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case NWKU_CoapSendConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_CoapSendConfirm.Status);
                    break;
            }

            break;

        case 0xCF95:
            shell_write("NWKU_DnsSendRequestConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_DnsSendRequestConfirm.Status)
            {
                case NWKU_DnsSendRequestConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case NWKU_DnsSendRequestConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_DnsSendRequestConfirm.Status);
                    break;
            }

            break;

        case 0xCF71:
            shell_write("NWKU_McastGroupManageConfirm");
            shell_write(" -> ");

            switch (container->Data.NWKU_McastGroupManageConfirm.Status)
            {
                case NWKU_McastGroupManageConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case NWKU_McastGroupManageConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.NWKU_McastGroupManageConfirm.Status);
                    break;
            }

            break;

        case 0xCF72:
            SHELL_PrintMcastGroups(&(container->Data.NWKU_McastGroupShowResponse));
            break;

        case 0xCF40:
            shell_write("MESHCOP_StartCommissionerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_StartCommissionerConfirm.Status)
            {
                case MESHCOP_StartCommissionerConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_StartCommissionerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_StartCommissionerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_StartCommissionerConfirm.Status);
                    break;
            }

            break;

        case 0xCF4F:
            shell_write("MESHCOP_StartNativeCommissionerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_StartNativeCommissionerConfirm.Status)
            {
                case MESHCOP_StartNativeCommissionerConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_StartNativeCommissionerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_StartNativeCommissionerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_StartNativeCommissionerConfirm.Status);
                    break;
            }

            break;

        case 0xCF41:
            shell_write("MESHCOP_StopCommissionerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_StopCommissionerConfirm.Status)
            {
                case MESHCOP_StopCommissionerConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_StopCommissionerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_StopCommissionerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_StopCommissionerConfirm.Status);
                    break;
            }

            break;

        case 0xCF42:
            shell_write("MESHCOP_AddExpectedJoinerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_AddExpectedJoinerConfirm.Status)
            {
                case MESHCOP_AddExpectedJoinerConfirm_Status_Success:
                    shell_write(Success);

                    if (mAutostartInProgress)
                    {
                        shell_write("\r\nSynchronizing the steering data..");
                        MESHCOP_SyncSteeringDataRequest_t req;
                        req.InstanceId = threadInstanceID;
                        req.EuiMask = MESHCOP_SyncSteeringDataRequest_EuiMask_AllFFs;

                        MESHCOP_SyncSteeringDataRequest(&req, THR_FSCI_IF);
                    }

                    break;

                case MESHCOP_AddExpectedJoinerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_AddExpectedJoinerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_AddExpectedJoinerConfirm.Status);
                    break;
            }

            break;

        case 0xCF43:
            shell_write("MESHCOP_GetExpectedJoinerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_GetExpectedJoinerConfirm.Status)
            {
                case MESHCOP_GetExpectedJoinerConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_GetExpectedJoinerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_GetExpectedJoinerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_GetExpectedJoinerConfirm.Status);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.MESHCOP_GetExpectedJoinerConfirm.Selected)
            {
                case MESHCOP_GetExpectedJoinerConfirm_Selected_FALSE:
                    shell_write(FALSE_str);
                    break;

                case MESHCOP_GetExpectedJoinerConfirm_Selected_TRUE:
                    shell_write(TRUE_str);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_GetExpectedJoinerConfirm.Selected);
                    break;
            }

            break;

        case 0xCF44:
            shell_write("MESHCOP_RemoveExpectedJoinerConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_RemoveExpectedJoinerConfirm.Status)
            {
                case MESHCOP_RemoveExpectedJoinerConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_RemoveExpectedJoinerConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_RemoveExpectedJoinerConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_RemoveExpectedJoinerConfirm.Status);
                    break;
            }

            break;

        case 0xCF45:
            shell_write("MESHCOP_RemoveAllExpectedJoinersConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_RemoveAllExpectedJoinersConfirm.Status)
            {
                case MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_RemoveAllExpectedJoinersConfirm.Status);
                    break;
            }

            break;

        case 0xCF46:
            shell_write("MESHCOP_SyncSteeringDataConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_SyncSteeringDataConfirm.Status)
            {
                case MESHCOP_SyncSteeringDataConfirm_Status_Success:
                    shell_write(Success);

                    // this is the last command in the autostart demo
                    if (mAutostartInProgress)
                    {
                        mAutostartInProgress = FALSE;
                        mAutostartState = gAutostartFirstState_c;
                        shell_write("\r\nAutostart completed successfully!");
                        shell_printf("\r\n\tJoiners may be added on channel %d, any EUI, PSKd `THREAD`", mAutostartChannel);
                        SHELL_Resume();
                    }

                    break;

                case MESHCOP_SyncSteeringDataConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_SyncSteeringDataConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_SyncSteeringDataConfirm.Status);
                    break;
            }

            break;

        case 0xCF4B:
            shell_write("MESHCOP_MgmtSetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtSetConfirm.Status)
            {
                case MESHCOP_MgmtSetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtSetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtSetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtSetConfirm.Status);
                    break;
            }

            break;

        case 0xCF4C:
            shell_write("MESHCOP_MgmtGetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtGetConfirm.Status)
            {
                case MESHCOP_MgmtGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtGetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtGetConfirm.Status);
                    break;
            }

            break;

        case 0xCF4D:
            shell_write("MESHCOP_SetCommissionerCredentialConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_SetCommissionerCredentialConfirm.Status)
            {
                case MESHCOP_SetCommissionerCredentialConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_SetCommissionerCredentialConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_SetCommissionerCredentialConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_SetCommissionerCredentialConfirm.Status);
                    break;
            }

            break;

        case 0xCF47:
            shell_write("MESHCOP_MgmNwkFormConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmNwkFormConfirm.Status)
            {
                case MESHCOP_MgmNwkFormConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmNwkFormConfirm_Status_Notpermitted:
                    shell_write(Notpermitted);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmNwkFormConfirm.Status);
                    break;
            }

            break;

        case 0xCFA0:
            shell_write("MESHCOP_MgmtCommissionerGetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtCommissionerGetConfirm.Status)
            {
                case MESHCOP_MgmtCommissionerGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtCommissionerGetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtCommissionerGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtCommissionerGetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA2:
            shell_write("MESHCOP_MgmtActiveGetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtActiveGetConfirm.Status)
            {
                case MESHCOP_MgmtActiveGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtActiveGetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtActiveGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtActiveGetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA4:
            shell_write("MESHCOP_MgmtPendingGetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtPendingGetConfirm.Status)
            {
                case MESHCOP_MgmtPendingGetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtPendingGetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtPendingGetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtPendingGetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA1:
            shell_write("MESHCOP_MgmtCommissionerSetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtCommissionerSetConfirm.Status)
            {
                case MESHCOP_MgmtCommissionerSetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtCommissionerSetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtCommissionerSetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtCommissionerSetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA3:
            shell_write("MESHCOP_MgmtActiveSetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtActiveSetConfirm.Status)
            {
                case MESHCOP_MgmtActiveSetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtActiveSetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtActiveSetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtActiveSetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA5:
            shell_write("MESHCOP_MgmtPendingSetConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtPendingSetConfirm.Status)
            {
                case MESHCOP_MgmtPendingSetConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtPendingSetConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtPendingSetConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtPendingSetConfirm.Status);
                    break;
            }

            break;

        case 0xCFA8:
            shell_write("MESHCOP_MgmtSendPanIdQueryConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtSendPanIdQueryConfirm.Status)
            {
                case MESHCOP_MgmtSendPanIdQueryConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtSendPanIdQueryConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtSendPanIdQueryConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtSendPanIdQueryConfirm.Status);
                    break;
            }

            break;

        case 0xCFA9:
            shell_write("MESHCOP_MgmtPanIdConflictConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtPanIdConflictConfirm.Status)
            {
                case MESHCOP_MgmtPanIdConflictConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtPanIdConflictConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtPanIdConflictConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtPanIdConflictConfirm.Status);
                    break;
            }

            break;

        case 0xCFAA:
            shell_write("MESHCOP_MgmtSendEdScanConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtSendEdScanConfirm.Status)
            {
                case MESHCOP_MgmtSendEdScanConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtSendEdScanConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtSendEdScanConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtSendEdScanConfirm.Status);
                    break;
            }

            break;

        case 0xCFAB:
            shell_write("MESHCOP_MgmtEdReportConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtEdReportConfirm.Status)
            {
                case MESHCOP_MgmtEdReportConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtEdReportConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtEdReportConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtEdReportConfirm.Status);
                    break;
            }

            break;

        case 0xCFA7:
            shell_write("MESHCOP_MgmtSendAnnounceBeginConfirm");
            shell_write(" -> ");

            switch (container->Data.MESHCOP_MgmtSendAnnounceBeginConfirm.Status)
            {
                case MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Success:
                    shell_write(Success);
                    break;

                case MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Failed:
                    shell_write(Failed);
                    break;

                case MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Error:
                    shell_write(Error);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.MESHCOP_MgmtSendAnnounceBeginConfirm.Status);
                    break;
            }

            break;

        case 0xCF30:
            shell_write("DTLSOpenConfirm");
            break;

        case 0xCF31:
            shell_write("DTLSCloseConfirm");
            shell_write(" -> ");

            switch (container->Data.DTLSCloseConfirm.Status)
            {
                case DTLSCloseConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case DTLSCloseConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.DTLSCloseConfirm.Status);
                    break;
            }

            break;

        case 0xCF32:
            shell_write("DTLSClosePeerConfirm");
            shell_write(" -> ");

            switch (container->Data.DTLSClosePeerConfirm.Status)
            {
                case DTLSClosePeerConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case DTLSClosePeerConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.DTLSClosePeerConfirm.Status);
                    break;
            }

            break;

        case 0xCF33:
            shell_write("DTLSConnectConfirm");
            shell_write(" -> ");

            switch (container->Data.DTLSConnectConfirm.Status)
            {
                case DTLSConnectConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case DTLSConnectConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.DTLSConnectConfirm.Status);
                    break;
            }

            break;

        case 0xCF34:
            shell_write("DTLSClientConnectedConfirm");
            shell_write(" -> ");

            switch (container->Data.DTLSClientConnectedConfirm.Status)
            {
                case DTLSClientConnectedConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case DTLSClientConnectedConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.DTLSClientConnectedConfirm.Status);
                    break;
            }

            break;

        case 0xCF35:
            shell_write("DTLSSendConfirm");
            shell_write(" -> ");

            switch (container->Data.DTLSSendConfirm.Status)
            {
                case DTLSSendConfirm_Status_OK:
                    shell_write(OK);
                    break;

                case DTLSSendConfirm_Status_ERROR:
                    shell_write(ERROR);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.DTLSSendConfirm.Status);
                    break;
            }

            break;

        case 0xCF36:
            shell_write("DTLSReceiveConfirm");
            break;

        case 0xCFF3:
            shell_write("SerialTun_IPPacketReceivedConfirm");
            break;

        case 0xCF89:
            shell_write("DBGConfirm");
            break;

        case 0xCFD0:
            shell_write("Debug: ");
            shell_write(container->Data.DebugEvent.text);
            break;

        case 0xCFD1:
            shell_write("K64 OTA update finished. Rebooting ... ");
            ResetMCU();
            break;

    }

    shell_cmd_finished();
}

/*==================================================================================================
                                        END OF GENERATED CODE
==================================================================================================*/

#endif /* THREAD_USE_SHELL */

/*!*************************************************************************************************
\private
\fn     static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to process a LED command (on, off, flash, toggle, rgb, color wheel).

\param  [in]    pCommand    Pointer to command data
\param  [in]    dataLen     Data length
***************************************************************************************************/
#if 0
static void APP_ProcessLedCmd
(
    uint8_t *pCommand,
    uint8_t dataLen
)
{

    /* Process command */
    if (FLib_MemCmp(pCommand, "on", 2))
    {
        Led_SetState(gDeviceMode_Application_c, gDeviceState_AppLedOn_c);
    }
    else if (FLib_MemCmp(pCommand, "off", 3))
    {
        Led_SetState(gDeviceMode_Application_c, gDeviceState_AppLedOff_c);
    }
    else if (FLib_MemCmp(pCommand, "toggle", 6))
    {
        Led_SetState(gDeviceMode_Application_c, gDeviceState_AppLedToggle_c);
    }
    else if (FLib_MemCmp(pCommand, "flash", 5))
    {
        Led_SetState(gDeviceMode_Application_c, gDeviceState_AppLedFlash_c);
    }
    else if (FLib_MemCmp(pCommand, "rgb", 3))
    {
        char *p = (char *)pCommand + strlen("rgb");
        uint8_t redValue = 0, greenValue = 0, blueValue = 0;

        dataLen -= strlen("rgb");

        while (dataLen != 0)
        {
            if (*p == 'r')
            {
                p++;
                dataLen--;
                redValue = NWKU_atoi(p);
            }

            if (*p == 'g')
            {
                p++;
                dataLen--;
                greenValue = NWKU_atoi(p);
            }

            if (*p == 'b')
            {
                p++;
                dataLen--;
                blueValue = NWKU_atoi(p);
            }

            dataLen--;
            p++;
        }

        /* Update RGB values */
#if gLedRgbEnabled_d
        Led_UpdateRgbState(redValue, greenValue, blueValue);
        Led_SetState(gDeviceMode_Application_c, gDeviceState_AppLedRgb_c);
#endif
    }
}
#endif

/*==================================================================================================
Private debug functions
==================================================================================================*/
