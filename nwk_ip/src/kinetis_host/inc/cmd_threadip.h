/* Header file generated from ThreadIP.xml */

#ifndef _THREADIP_CMD_H
#define _THREADIP_CMD_H
/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "MemManager.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define MAC_ENABLE TRUE
#define THREAD_MANAGEMENT_ENABLE TRUE
#define THREAD_UTILS_ENABLE TRUE
#define SOCKET_COMMANDS_ENABLE TRUE
#define MESHCOP_ENABLE TRUE
#define IP_UTILS_ENABLE TRUE
#define DTLS_ENABLE TRUE
#define OTHER_UTILS_ENABLE TRUE

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef PACKED_STRUCT MAC_MacFilteringAddEntryRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint64_t ExtendedAddress;  // Extended Address
	uint16_t ShortAddress;  // Short Address
	uint8_t LinkIndicator;  // The neighbor Quality Link Indicator: Good Link: 20 - 255 Medium Link: 11 - 20 Bad Link: 3 - 10.
	bool_t BlockNeighbor;  // Add this neighbor to blacklist
} MAC_MacFilteringAddEntryRequest_t;

typedef PACKED_STRUCT MAC_MacFilteringRemoveEntryRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint64_t ExtendedAddress;  // Extended Address
} MAC_MacFilteringRemoveEntryRequest_t;

/* Enable or disable mac filtering */
typedef enum MAC_MacFilteringEnableRequest_MacFiltering_tag {
	MAC_MacFilteringEnableRequest_MacFiltering_Disable = 0x00,
	MAC_MacFilteringEnableRequest_MacFiltering_EnableReject = 0x01,
	MAC_MacFilteringEnableRequest_MacFiltering_EnableAccept = 0x02
} MAC_MacFilteringEnableRequest_MacFiltering_t;

typedef PACKED_STRUCT MAC_MacFilteringEnableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	MAC_MacFilteringEnableRequest_MacFiltering_t MacFiltering;  // Enable or disable mac filtering
} MAC_MacFilteringEnableRequest_t;

typedef PACKED_STRUCT MAC_MacFilteringGetTableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t StartIndex;  // Start Index
	uint8_t NoOfElements;  // No of elements to print
} MAC_MacFilteringGetTableRequest_t;

typedef struct THR_SetDeviceConfigRequest_tag {
	uint8_t ThrInstanceID;  // Thread instance ID
	bool_t OutOfBandPreconfigured;  // OutOfBandPreconfigured
	uint8_t outOfBandChannel;  // outOfBandChannel
	uint16_t PanId;  // PanId
	uint32_t ScanChannels;  // ScanChannels
	uint8_t ExtendedPanId[8];  // Extended PAN ID
	uint8_t NwkNameSize;  // Network Name Size
	char *NwkName;  // Network Name
	uint8_t MLPrefix[16];  // MLPrefix
	uint8_t MLprefixSizeInBits;  // ML Prefix Size
	uint8_t MasterKey[16];  // Master Key
} THR_SetDeviceConfigRequest_t;

/* Scan type */
typedef enum THR_NwkScanRequest_ScanType_tag {
	THR_NwkScanRequest_ScanType_EnergyDetect = 0x01,
	THR_NwkScanRequest_ScanType_ActiveScan = 0x02,
	THR_NwkScanRequest_ScanType_EnergyDetectAndActiveScan = 0x03
} THR_NwkScanRequest_ScanType_t;

typedef PACKED_STRUCT THR_NwkScanRequest_tag {
	uint8_t InstanceID;  // InstanceID
	uint32_t ScanChannelMask;  // Channel mask
	THR_NwkScanRequest_ScanType_t ScanType;  // Scan type
	uint8_t ScanDuration;  // Scan Duration. Exponential scale, as seen in the 802.15.4 specification (Range:1 - 14)
	uint16_t maxThrNwkToDiscover;  // maximum thread network to be discovered
} THR_NwkScanRequest_t;

typedef PACKED_STRUCT THR_CreateNwkRequest_tag {
	uint8_t InstanceID;  // InstanceID
} THR_CreateNwkRequest_t;

/* discovery method */
typedef enum THR_JoinRequest_discoveryMethod_tag {
	THR_JoinRequest_discoveryMethod_gUseMACBeacon_c = 0x00,
	THR_JoinRequest_discoveryMethod_gUseThreadDiscovery_c = 0x01
} THR_JoinRequest_discoveryMethod_t;

typedef PACKED_STRUCT THR_JoinRequest_tag {
	uint8_t InstanceID;  // thread instance id
	THR_JoinRequest_discoveryMethod_t discoveryMethod;  // discovery method
} THR_JoinRequest_t;

typedef PACKED_STRUCT THR_CpuResetRequest_tag {
	uint32_t TimeoutMs;  // the time in milliseconds after that the device will enter in the reset state.
} THR_CpuResetRequest_t;

typedef PACKED_STRUCT THR_DisconnectRequest_tag {
	uint8_t InstanceID;  // Thread instance id
} THR_DisconnectRequest_t;

typedef PACKED_STRUCT THR_AttachRequest_tag {
	uint8_t InstanceID;  // InstanceID
} THR_AttachRequest_t;

/* Promoting reason */
typedef enum THR_PromoteAsRouterRequest_Reason_tag {
	THR_PromoteAsRouterRequest_Reason_TooFewRouters = 0x02,
	THR_PromoteAsRouterRequest_Reason_HaveChildIdRequest = 0x03,
	THR_PromoteAsRouterRequest_Reason_ParentPartitionChange = 0x04
} THR_PromoteAsRouterRequest_Reason_t;

typedef PACKED_STRUCT THR_PromoteAsRouterRequest_tag {
	uint8_t InstanceID;  // InstanceID
	THR_PromoteAsRouterRequest_Reason_t Reason;  // Promoting reason
} THR_PromoteAsRouterRequest_t;

/* TlvId */
typedef enum THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_tag {
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_SourceAddr = 0x00,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_ShortAddr = 0x01,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Mode = 0x02,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Timeout = 0x03,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_LinkQuality = 0x04,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_RoutingTable = 0x05,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_LeaderData = 0x06,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_NwkData = 0x07,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Ip6AddrList = 0x08,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_MacCounters = 0x09,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_BatteryLevel = 0x0E,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_SupplyVoltage = 0x0F,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_ChildTable = 0x10,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_ChannelPages = 0x11,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_Mac6lowPanNvmDataCount = 0xA0,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_NetworkNvmDataCount_c = 0xA1,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_SecurityNvmDataCount_c = 0xA2,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_FunctionalNvmDataCount_c = 0xA3,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_BoardName_c = 0xA4,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_UniqueMcuId_c = 0xA5,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_StackVersion_c = 0xA6,
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_Fsl_SoftwareVersion_c = 0xA7
} THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_t;

typedef struct THR_MgmtDiagnosticGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t DestIpAddr[16];  // Destination Ip Address
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	THR_MgmtDiagnosticGetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} THR_MgmtDiagnosticGetRequest_t;

/* TlvId */
typedef enum THR_DiagTestGetRequest_TlvId_tag {
	THR_DiagTestGetRequest_TlvId_ColdFactoryReset = 0xB0,
	THR_DiagTestGetRequest_TlvId_WarmCPUReset = 0xB1,
	THR_DiagTestGetRequest_TlvId_Data = 0xB2,
	THR_DiagTestGetRequest_TlvId_Results = 0xB3
} THR_DiagTestGetRequest_TlvId_t;

typedef struct THR_DiagTestGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t DestIpAddr[16];  // Destination Ip Address
	uint8_t PayloadSize;  // Size of payload - enter manually
	THR_DiagTestGetRequest_TlvId_t TlvId;  // TlvId
	union {
		struct {
			uint8_t Value;  // IncludeLatencyReport|IncludeTxTimestamp  // Value
			uint8_t *Payload;  // Payload to be transmited OTA
		} Data;  // Large Network Data
	} AttributeValue;  // Attribute Value
} THR_DiagTestGetRequest_t;

/* TlvId */
typedef enum THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_tag {
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_SourceAddr = 0x00,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_ShortAddr = 0x01,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Mode = 0x02,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Timeout = 0x03,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_LinkQuality = 0x04,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_RoutingTable = 0x05,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_LeaderData = 0x06,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_NwkData = 0x07,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Ip6AddrList = 0x08,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_MacCounters = 0x09,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_BatteryLevel = 0x0E,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_SupplyVoltage = 0x0F,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_ChildTable = 0x10,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_ChannelPages = 0x11,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_Mac6lowPanNvmDataCount = 0xA0,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_NetworkNvmDataCount_c = 0xA1,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_SecurityNvmDataCount_c = 0xA2,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_FunctionalNvmDataCount_c = 0xA3,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_BoardName_c = 0xA4,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_UniqueMcuId_c = 0xA5,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_StackVersion_c = 0xA6,
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_Fsl_SoftwareVersion_c = 0xA7
} THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_t;

typedef struct THR_MgmtDiagnosticResetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t DestIpAddr[16];  // Destination Ip Address
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	THR_MgmtDiagnosticResetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} THR_MgmtDiagnosticResetRequest_t;

typedef PACKED_STRUCT THR_MgmtReadMemoryRequest_tag {
	uint32_t Address;  // Memory address
	uint8_t Length;  // Length
} THR_MgmtReadMemoryRequest_t;

typedef struct THR_MgmtWriteMemoryRequest_tag {
	uint32_t Address;  // Memory address
	uint8_t Length;  // Length
	uint8_t *Value;  // Value
} THR_MgmtWriteMemoryRequest_t;

typedef PACKED_STRUCT THR_NwkDiscoveryRequest_tag {
	uint8_t InstanceID;  // Thread instance id
} THR_NwkDiscoveryRequest_t;

typedef PACKED_STRUCT THR_NwkDiscoveryStopRequest_tag {
	uint8_t InstanceID;  // Thread instance id
} THR_NwkDiscoveryStopRequest_t;

typedef PACKED_STRUCT THR_SearchNwkWithAnounceRequest_tag {
	uint8_t InstanceID;  // Thread instance id
} THR_SearchNwkWithAnounceRequest_t;

typedef PACKED_STRUCT THR_ChildUpdateToParentRequest_tag {
	uint8_t InstanceID;  // InstanceID
} THR_ChildUpdateToParentRequest_t;

typedef PACKED_STRUCT THR_SetManualSlaacIIDRequest_tag {
	uint64_t IID;  // IID
} THR_SetManualSlaacIIDRequest_t;

typedef PACKED_STRUCT THR_SendProactiveAddrNotifRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t DestinationIPAddress[16];  // DestinationIPAddress
} THR_SendProactiveAddrNotifRequest_t;

/* Threshold type  */
typedef enum THR_SetThresholdRequest_ThresholdType_tag {
	THR_SetThresholdRequest_ThresholdType_RouterUpgradeThreshold = 0x00,
	THR_SetThresholdRequest_ThresholdType_RouterDowngradeThreshold = 0x01,
	THR_SetThresholdRequest_ThresholdType_MinDowngradeNeighbors = 0x02,
	THR_SetThresholdRequest_ThresholdType_MaxAllowedRouters = 0x03,
	THR_SetThresholdRequest_ThresholdType_ContextReuseDelay = 0x04
} THR_SetThresholdRequest_ThresholdType_t;

typedef PACKED_STRUCT THR_SetThresholdRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	THR_SetThresholdRequest_ThresholdType_t ThresholdType;  // Threshold type
	uint8_t Value;  // Value
} THR_SetThresholdRequest_t;

typedef PACKED_STRUCT THR_SetNwkIdTimeoutRequest_tag {
	uint32_t TimeoutInSeconds;  // TimeoutInSeconds
} THR_SetNwkIdTimeoutRequest_t;

typedef PACKED_STRUCT THR_GetRoutingTableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t StartIndex;  // Start Index
	uint8_t NoOfElements;  // No of elements to print
} THR_GetRoutingTableRequest_t;

typedef PACKED_STRUCT THR_GetNeighborTableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t StartIndex;  // Start Index
	uint8_t NoOfElements;  // No of elements to print
} THR_GetNeighborTableRequest_t;

typedef PACKED_STRUCT THR_GetNeighborInfoRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint16_t ShortAddress;  // Short Address of the neighbor
} THR_GetNeighborInfoRequest_t;

typedef PACKED_STRUCT THR_GetChildrenTableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t StartIndex;  // Start Index
	uint8_t NoOfElements;  // No of elements to print
} THR_GetChildrenTableRequest_t;

typedef enum THR_GetAttrRequest_AttributeId_tag {
	THR_GetAttrRequest_AttributeId_RandomExtendedAddr = 0x00,
	THR_GetAttrRequest_AttributeId_ShortAddress = 0x01,
	THR_GetAttrRequest_AttributeId_ScanChannelMask = 0x02,
	THR_GetAttrRequest_AttributeId_ScanDuration = 0x03,
	THR_GetAttrRequest_AttributeId_Channel = 0x04,
	THR_GetAttrRequest_AttributeId_ShortPanId = 0x05,
	THR_GetAttrRequest_AttributeId_ExtendedPanId = 0x06,
	THR_GetAttrRequest_AttributeId_PermitJoin = 0x07,
	THR_GetAttrRequest_AttributeId_RxOnIdle = 0x08,
	THR_GetAttrRequest_AttributeId_SedPollInterval = 0x09,
	THR_GetAttrRequest_AttributeId_UniqueExtendedAddress = 0x0A,
	THR_GetAttrRequest_AttributeId_VendorName = 0x0B,
	THR_GetAttrRequest_AttributeId_ModelName = 0x0C,
	THR_GetAttrRequest_AttributeId_SwVersion = 0x0D,
	THR_GetAttrRequest_AttributeId_StackVersion = 0x0E,
	THR_GetAttrRequest_AttributeId_NwkCapabilities = 0x0F,
	THR_GetAttrRequest_AttributeId_NwkName = 0x10,
	THR_GetAttrRequest_AttributeId_DeviceType = 0x11,
	THR_GetAttrRequest_AttributeId_IsDevConnected = 0x12,
	THR_GetAttrRequest_AttributeId_IsDevCommissioned = 0x13,
	THR_GetAttrRequest_AttributeId_PartitionId = 0x14,
	THR_GetAttrRequest_AttributeId_DeviceRole = 0x15,
	THR_GetAttrRequest_AttributeId_Security_NwkMasterKey = 0x16,
	THR_GetAttrRequest_AttributeId_Security_NwkKeySeq = 0x17,
	THR_GetAttrRequest_AttributeId_Security_PSKc = 0x18,
	THR_GetAttrRequest_AttributeId_Security_PSKd = 0x19,
	THR_GetAttrRequest_AttributeId_VendorData = 0x1A,
	THR_GetAttrRequest_AttributeId_MLPrefix = 0x1C,
	THR_GetAttrRequest_AttributeId_MacFilteringEntry = 0x1D,
	THR_GetAttrRequest_AttributeId_Security_KeyRotationInterval = 0x20,
	THR_GetAttrRequest_AttributeId_ChildAddrMask = 0x21,
	THR_GetAttrRequest_AttributeId_ChildSEDTimeout = 0x22,
	THR_GetAttrRequest_AttributeId_ChildEDTimeout = 0x1B,
	THR_GetAttrRequest_AttributeId_EndDevice_ChildEDReqFullNwkData = 0x23,
	THR_GetAttrRequest_AttributeId_EndDevice_IsFastPollEnabled = 0x24,
	THR_GetAttrRequest_AttributeId_SleepyEndDevice_FastPollInterval = 0x25,
	THR_GetAttrRequest_AttributeId_JoinLqiThreshold = 0x26,
	THR_GetAttrRequest_AttributeId_ProvisioningURL = 0x27,
	THR_GetAttrRequest_AttributeId_SelectBestChannelEDThreshold = 0x28,
	THR_GetAttrRequest_AttributeId_CommissionerMode = 0x29,
	THR_GetAttrRequest_AttributeId_BorderRouter_BrPrefixEntry = 0x30,
	THR_GetAttrRequest_AttributeId_SteeringData = 0x31,
	THR_GetAttrRequest_AttributeId_Security_KeySwitchGuardTime = 0x33,
	THR_GetAttrRequest_AttributeId_ParentHoldTime = 0x34,
	THR_GetAttrRequest_AttributeId_Security_Policy = 0x35,
	THR_GetAttrRequest_AttributeId_NVM_RestoreAutoStart = 0x36,
	THR_GetAttrRequest_AttributeId_NVM_Restore = 0x37,
	THR_GetAttrRequest_AttributeId_SlaacPolicy = 0x38,
	THR_GetAttrRequest_AttributeId_IeeeExtendedAddr = 0x39,
	THR_GetAttrRequest_AttributeId_LeaderWeight = 0x3A,
	THR_GetAttrRequest_AttributeId_HashIeeeAddr = 0x40,
	THR_GetAttrRequest_AttributeId_BorderRouter_BrGlobalOnMeshPrefix = 0x50,
	THR_GetAttrRequest_AttributeId_BorderRouter_BrDefaultRouteOnMeshPrefix = 0x51,
	THR_GetAttrRequest_AttributeId_BorderRouter_BrExternalIfPrefix = 0x52,
	THR_GetAttrRequest_AttributeId_ActiveTimestamp = 0x60,
	THR_GetAttrRequest_AttributeId_PendingChannel = 0x61,
	THR_GetAttrRequest_AttributeId_PendingChannelMask = 0x62,
	THR_GetAttrRequest_AttributeId_PendingXpanId = 0x63,
	THR_GetAttrRequest_AttributeId_PendingMLprefix = 0x64,
	THR_GetAttrRequest_AttributeId_PendingNwkMasterKey = 0x65,
	THR_GetAttrRequest_AttributeId_PendingNwkName = 0x66,
	THR_GetAttrRequest_AttributeId_PendingPanId = 0x67,
	THR_GetAttrRequest_AttributeId_PendingPSK = 0x68,
	THR_GetAttrRequest_AttributeId_PendingSecurityPolicy = 0x69,
	THR_GetAttrRequest_AttributeId_PendingNwkKeyRotationInterval = 0x6A,
	THR_GetAttrRequest_AttributeId_PendingDelayTimer = 0x6B,
	THR_GetAttrRequest_AttributeId_PendingActiveTimestamp = 0x6C,
	THR_GetAttrRequest_AttributeId_PendingTimestamp = 0x6D,
	THR_GetAttrRequest_AttributeId_CommissionerId = 0x6E,
	THR_GetAttrRequest_AttributeId_JoinerPort = 0x6F,
	THR_GetAttrRequest_AttributeId_CommissionerUdpPort = 0x70,
	THR_GetAttrRequest_AttributeId_DiscoveryReqMacTxOptions = 0x71
} THR_GetAttrRequest_AttributeId_t;

typedef PACKED_STRUCT THR_GetAttrRequest_tag {
	uint8_t InstanceId;  // Instance Id
	THR_GetAttrRequest_AttributeId_t AttributeId;
	uint8_t Index;  // Index into a table
} THR_GetAttrRequest_t;

typedef enum THR_SetAttrRequest_AttributeId_tag {
	THR_SetAttrRequest_AttributeId_RandomExtendedAddr = 0x00,
	THR_SetAttrRequest_AttributeId_ScanChannelMask = 0x02,
	THR_SetAttrRequest_AttributeId_ScanDuration = 0x03,
	THR_SetAttrRequest_AttributeId_Channel = 0x04,
	THR_SetAttrRequest_AttributeId_ShortPanId = 0x05,
	THR_SetAttrRequest_AttributeId_ExtendedPanId = 0x06,
	THR_SetAttrRequest_AttributeId_RxOnIdle = 0x08,
	THR_SetAttrRequest_AttributeId_SedPollInterval = 0x09,
	THR_SetAttrRequest_AttributeId_UniqueExtendedAddress = 0x0A,
	THR_SetAttrRequest_AttributeId_VendorName = 0x0B,
	THR_SetAttrRequest_AttributeId_ModelName = 0x0C,
	THR_SetAttrRequest_AttributeId_SwVersion = 0x0D,
	THR_SetAttrRequest_AttributeId_NwkCapabilities = 0x0F,
	THR_SetAttrRequest_AttributeId_NwkName = 0x10,
	THR_SetAttrRequest_AttributeId_IsDevConnected = 0x12,
	THR_SetAttrRequest_AttributeId_IsDevCommissioned = 0x13,
	THR_SetAttrRequest_AttributeId_PartitionId = 0x14,
	THR_SetAttrRequest_AttributeId_DeviceRole = 0x15,
	THR_SetAttrRequest_AttributeId_Security_NwkMasterKey = 0x16,
	THR_SetAttrRequest_AttributeId_Security_NwkKeySeq = 0x17,
	THR_SetAttrRequest_AttributeId_Security_PSKc = 0x18,
	THR_SetAttrRequest_AttributeId_Security_PSKd = 0x19,
	THR_SetAttrRequest_AttributeId_VendorData = 0x1A,
	THR_SetAttrRequest_AttributeId_MLPrefix = 0x1C,
	THR_SetAttrRequest_AttributeId_Security_KeyRotationInterval = 0x20,
	THR_SetAttrRequest_AttributeId_ChildSEDTimeout = 0x22,
	THR_SetAttrRequest_AttributeId_EndDevice_ChildEDReqFullNwkData = 0x23,
	THR_SetAttrRequest_AttributeId_EndDevice_IsFastPollEnabled = 0x24,
	THR_SetAttrRequest_AttributeId_SleepyEndDevice_FastPollInterval = 0x25,
	THR_SetAttrRequest_AttributeId_JoinLqiThreshold = 0x26,
	THR_SetAttrRequest_AttributeId_ProvisioningURL = 0x27,
	THR_SetAttrRequest_AttributeId_SelectBestChannelEDThreshold = 0x28,
	THR_SetAttrRequest_AttributeId_BorderRouter_BrPrefixEntry = 0x30,
	THR_SetAttrRequest_AttributeId_Security_KeySwitchGuardTime = 0x33,
	THR_SetAttrRequest_AttributeId_ParentHoldTime = 0x34,
	THR_SetAttrRequest_AttributeId_Security_Policy = 0x35,
	THR_SetAttrRequest_AttributeId_NVM_RestoreAutoStart = 0x36,
	THR_SetAttrRequest_AttributeId_NVM_Restore = 0x37,
	THR_SetAttrRequest_AttributeId_SlaacPolicy = 0x38,
	THR_SetAttrRequest_AttributeId_IeeeExtendedAddr = 0x39,
	THR_SetAttrRequest_AttributeId_LeaderWeight = 0x3A,
	THR_SetAttrRequest_AttributeId_DoNotGeneratePartitionId = 0x41,
	THR_SetAttrRequest_AttributeId_BorderRouter_BrGlobalOnMeshPrefix = 0x50,
	THR_SetAttrRequest_AttributeId_BorderRouter_BrDefaultRouteOnMeshPrefix = 0x51,
	THR_SetAttrRequest_AttributeId_BorderRouter_BrExternalIfPrefix = 0x52,
	THR_SetAttrRequest_AttributeId_ActiveTimestamp = 0x60,
	THR_SetAttrRequest_AttributeId_JoinerPort = 0x6F
} THR_SetAttrRequest_AttributeId_t;

/* Value */
typedef enum THR_SetAttrRequest_AttributeValue_DeviceType_Value_tag {
	THR_SetAttrRequest_AttributeValue_DeviceType_Value_EndNode = 0x00,
	THR_SetAttrRequest_AttributeValue_DeviceType_Value_Combo = 0x01
} THR_SetAttrRequest_AttributeValue_DeviceType_Value_t;

/* Value */
typedef enum THR_SetAttrRequest_AttributeValue_DeviceRole_Value_tag {
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_Disconnected = 0x00,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_SleepyEndDevice = 0x01,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_MinimalEndDevice = 0x02,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_FullEndDevice = 0x03,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_RouterEligibleEndDevice = 0x04,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_Router = 0x05,
	THR_SetAttrRequest_AttributeValue_DeviceRole_Value_Leader = 0x06
} THR_SetAttrRequest_AttributeValue_DeviceRole_Value_t;

/* Value */
typedef enum THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_tag {
	THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_Disabled = 0x00,
	THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_Collapsed = 0x01,
	THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_Native = 0x02,
	THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_Ethernet = 0x04,
	THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_OnMesh = 0x08
} THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_t;

/* P_preference */
typedef enum THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_tag {
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_MediumDefault = 0x00,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_High = 0x01,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_Reserved = 0x02,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_Low = 0x03
} THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_PrefixFlags_P_preference_t;

/* R_preference */
typedef enum THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_tag {
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_MediumDefault = 0x00,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_High = 0x01,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_Reserved = 0x02,
	THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_Low = 0x03
} THR_SetAttrRequest_AttributeValue_BorderRouter_BrPrefixEntry_Value_ExternalRouteFlags_R_preference_t;

/* Value */
typedef enum THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_tag {
	THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_SlaacRandom = 0x00,
	THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_SlaacManual = 0x01,
	THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_SlaacMlIid = 0x02
} THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_t;

typedef struct THR_SetAttrRequest_tag {
	uint8_t InstanceId;  // Instance Id
	THR_SetAttrRequest_AttributeId_t AttributeId;
	uint8_t Index;  // Index in table if required
	union {
		struct {
			uint8_t AttrSize;  // Attribute size
			uint64_t Value;  // Value
		} RandomExtendedAddr;  // Random Extended Address
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} ScanChannelMask;  // Scan Channel mask
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} ScanDuration;  // Scan duration in seconds (per channel)
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} Channel;  // Channel
		struct {
			uint8_t AttrSize;  // Attribute size
			uint16_t Value;  // Value
		} ShortPanId;  // Short PanId
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t *Value;  // Value
		} ExtendedPanId;  // Extended PanId
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} PermitJoin;  // Permit Join
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} RxOnIdle;  // RxOnIdle
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} SedPollInterval;  // The polling interval for the sleepy end device (SED) [miliseconds]
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} UniqueExtendedAddress;  // Unique Extended Address Enabled or disabled
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} VendorName;  // Vendor Name
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} ModelName;  // Model Name
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} SwVersion;  // Software version
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // IsFullThreadDevice|IsPollingEndDevice|CanBecomeActiveRouter|CanCreateNewNetwork  // Value
		} NwkCapabilities;  // Network Capabilities
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} NwkName;  // Network Name
		struct {
			uint8_t AttrSize;  // Attribute size
			THR_SetAttrRequest_AttributeValue_DeviceType_Value_t Value;  // Value
		} DeviceType;  // Device Type
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} IsDevConnected;  // Is Device Connected?
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // VAlue
		} IsDevCommissioned;  // Is Device Commissioned?
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} PartitionId;  // Partition Identifier
		struct {
			uint8_t AttrSize;  // Attribute size
			THR_SetAttrRequest_AttributeValue_DeviceRole_Value_t Value;  // Value
		} DeviceRole;  // Device Role
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t *Value;  // Value
		} Security_NwkMasterKey;  // Network Master Key
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} Security_NwkKeySeq;  // Network key sequence
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} Security_PSKc;  // PSKc - network password
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} Security_PSKd;  // PSKd - device password
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} VendorData;  // Vendor Data
		struct {
			uint8_t AttrSize;  // Attribute size
			struct {
				uint8_t PrefixData[16];  // Prefix Data
				uint8_t PrefixLength;  // Prefix length in bits
			} Value;  // Value
		} MLPrefix;  // Mesh Local Prefix
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} Security_KeyRotationInterval;  // Key Rotation Interval
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} ChildSEDTimeout;  // The timeout period included in the Child ID Request sent to the parent
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} ChildEDTimeout;  // The timeout period included in the Child ID Request sent to the parent
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} EndDevice_ChildEDReqFullNwkData;  // If it is set TRUE The child End device should request the Full network data
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} EndDevice_IsFastPollEnabled;  // Is Fast Poll Interval enabled?
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} SleepyEndDevice_FastPollInterval;  // Fast Poll Interval
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} JoinLqiThreshold;  // JoinLqiThreshold
		struct {
			uint8_t AttrSize;  // Attribute size
			char *Value;  // Value
		} ProvisioningURL;  // ProvisioningURL
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} SelectBestChannelEDThreshold;  // The energy channel threshold to select the best channel when more channels are scan to form the network
		struct {
			uint8_t AttrSize;  // Attribute size
			THR_SetAttrRequest_AttributeValue_CommissionerMode_Value_t Value;  // Value
		} CommissionerMode;  // The device is a thread native commissioner
		struct {
			uint8_t AttrSize;  // Attribute size
			struct {
				uint8_t prefixLength;  // Prefix Length in bits
				uint8_t PrefixValue[16];  // Prefix value
				uint8_t PrefixFlagsReserved;  // PrefixFlagsReserved
				uint8_t PrefixFlags;  // P_preference|P_Slaac_preferred|P_Slaac_valid|P_Dhcp|P_Configure|P_Default|Reserved_Bit0  // PrefixFlags
				uint32_t prefixLifetime;  // prefixLifetime
				bool_t prefixAdvertised;  // prefixAdvertised
				uint8_t ExternalRouteFlags;  // R_preference  // ExternalRouteFlags
				uint32_t ExternalRouteLifetime;  // ExternalRouteLifetime
				bool_t ExternalRouteAdvertised;  // ExternalRouteAdvertised
			} Value;  // Value
		} BorderRouter_BrPrefixEntry;  // Border router prefix attribute entry
		struct {
			uint8_t AttrSize;  // Attribute size
			uint32_t Value;  // Value
		} Security_KeySwitchGuardTime;  // The thread Key switch guard time to prevent inadvertent key switching
		struct {
			uint8_t AttrSize;  // Attribute size
			uint16_t Value;  // Value
		} ParentHoldTime;  // Hold time on parent in seconds
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} Security_Policy;  // Security Policy, O and N bits without the rotation time
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} NVM_RestoreAutoStart;  // Stack starts automatically with NVM restore after reset
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} NVM_Restore;  // Restore from NVM
		struct {
			uint8_t AttrSize;  // Attribute size
			THR_SetAttrRequest_AttributeValue_SlaacPolicy_Value_t Value;  // Value
		} SlaacPolicy;
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t Value;  // Value
		} LeaderWeight;  // Leader Weight
		struct {
			uint8_t AttrSize;  // Attribute size
			uint64_t Value;  // Value
		} IeeeExtendedAddr;  // Ieee Extended Address
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} DoNotGeneratePartitionId;  // Avoid generation of a random partition ID
		struct {
			uint8_t AttrSize;  // Attribute size
			struct {
				uint8_t PrefixData[16];  // Prefix Data
				uint8_t PrefixLength;  // Prefix length in bits
			} Value;  // Value
		} BorderRouter_BrGlobalOnMeshPrefix;  // Global /64 on-Mesh Prefix on Border Router
		struct {
			uint8_t AttrSize;  // Attribute size
			bool_t Value;  // Value
		} BorderRouter_BrDefaultRouteOnMeshPrefix;  // Default Route of the /64 on-mesh prefix
		struct {
			uint8_t AttrSize;  // Attribute size
			struct {
				uint8_t PrefixData[16];  // Prefix Data
				uint8_t PrefixLength;  // Prefix length in bits
			} Value;  // Value
		} BorderRouter_BrExternalIfPrefix;  // Global /64 external interface prefix
		struct {
			uint8_t AttrSize;  // Attribute size
			uint8_t ActiveSeconds[6];  // Attribute size
			uint16_t ActiveTicks;  // Attribute size
		} ActiveTimestamp;  // Active timestamp
		struct {
			uint8_t AttrSize;  // Attribute size
			uint16_t Value;  // Value
		} JoinerPort;  // Joiner Port
	} AttributeValue;  // Attribute Value
} THR_SetAttrRequest_t;

typedef PACKED_STRUCT THR_LeaderRemoveRouterIdRequest_tag {
	uint8_t InstanceId;  // InstanceId
	uint16_t RouterShortAddr;  // Router short address.
} THR_LeaderRemoveRouterIdRequest_t;

typedef PACKED_STRUCT THR_GenerateAllKeysRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint32_t KeySequenceCounter;  // Key sequence counter
} THR_GenerateAllKeysRequest_t;

typedef PACKED_STRUCT THR_SwitchKeyRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint32_t KeySequenceCounter;  // Key sequence counter
} THR_SwitchKeyRequest_t;

/* P_Preference */
typedef enum THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_tag {
	THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_MediumDefault = 0x00,
	THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_High = 0x01,
	THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_Reserved = 0x02,
	THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_Low = 0x03
} THR_BrPrefixAddEntryRequest_PrefixFlags_P_Preference_t;

/* R_preference */
typedef enum THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_tag {
	THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_MediumDefault = 0x00,
	THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_High = 0x01,
	THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_Reserved = 0x02,
	THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_Low = 0x03
} THR_BrPrefixAddEntryRequest_ExternalRouteFlags_R_preference_t;

typedef PACKED_STRUCT THR_BrPrefixAddEntryRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t prefixLength;  // Prefix Length in bits
	uint8_t PrefixValue[16];  // Prefix value
	uint8_t PrefixFlagsReserved;  // PrefixFlagsReserved
	uint8_t PrefixFlags;  // P_Preference|P_Preferred|P_Slaac|P_Dhcp|P_Configure|P_Default|P_On_Mesh  // PrefixFlags
	uint32_t prefixLifetime;  // prefixLifetime
	bool_t prefixAdvertised;  // prefixAdvertised
	uint8_t ExternalRouteFlags;  // R_preference  // ExternalRouteFlags
	uint32_t ExternalRouteLifetime;  // ExternalRouteLifetime
	bool_t ExternalRouteAdvertised;  // ExternalRouteAdvertised
} THR_BrPrefixAddEntryRequest_t;

typedef PACKED_STRUCT THR_BrPrefixGetTableRequest_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t StartIndex;  // Start Index
	uint8_t NoOfElements;  // No of elements to print
} THR_BrPrefixGetTableRequest_t;

typedef PACKED_STRUCT THR_BrPrefixRemoveEntryRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t prefixLength;  // Prefix Length in bits
	uint8_t PrefixValue[16];  // Prefix value
} THR_BrPrefixRemoveEntryRequest_t;

typedef PACKED_STRUCT THR_BrServiceRemoveEntryRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	char ServiceName[9];
	uint8_t ServerAddress[16];  // ServerIPAddress
} THR_BrServiceRemoveEntryRequest_t;

typedef PACKED_STRUCT THR_BrServiceAddEntryRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	char ServiceName[9];
	uint8_t ServerAddress[16];  // ServerIPAddress
} THR_BrServiceAddEntryRequest_t;

typedef PACKED_STRUCT THR_BrPrefixSyncRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} THR_BrPrefixSyncRequest_t;

typedef PACKED_STRUCT THR_BrPrefixRemoveAllRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} THR_BrPrefixRemoveAllRequest_t;

/* Address type */
typedef enum THR_GetThreadIpAddrRequest_AddressType_tag {
	THR_GetThreadIpAddrRequest_AddressType_Link_Local_64 = 0x00,
	THR_GetThreadIpAddrRequest_AddressType_MLEID = 0x01,
	THR_GetThreadIpAddrRequest_AddressType_RLOC = 0x02,
	THR_GetThreadIpAddrRequest_AddressType_Global = 0x03
} THR_GetThreadIpAddrRequest_AddressType_t;

typedef struct THR_GetThreadIpAddrRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	THR_GetThreadIpAddrRequest_AddressType_t AddressType;  // Address type
	union {
		struct {
			uint8_t StartIndex;  // Start Index
			uint8_t NoOfElements;  // Number of elements
		} Global;  // Global Addresses
	} Data;  // Data
} THR_GetThreadIpAddrRequest_t;

typedef PACKED_STRUCT THR_GetParentRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} THR_GetParentRequest_t;

/* Socket Domain */
typedef enum SocketCreateRequest_SocketDomain_tag {
	SocketCreateRequest_SocketDomain_AF_INET6 = 0x0A,
	SocketCreateRequest_SocketDomain_AF_INET = 0x02
} SocketCreateRequest_SocketDomain_t;

/* Socket Type */
typedef enum SocketCreateRequest_SocketType_tag {
	SocketCreateRequest_SocketType_Datagram = 0x00,
	SocketCreateRequest_SocketType_Stream = 0x01
} SocketCreateRequest_SocketType_t;

/* Socket Protocol */
typedef enum SocketCreateRequest_SocketProtocol_tag {
	SocketCreateRequest_SocketProtocol_UDP = 0x11,
	SocketCreateRequest_SocketProtocol_TCP = 0x06
} SocketCreateRequest_SocketProtocol_t;

typedef PACKED_STRUCT SocketCreateRequest_tag {
	SocketCreateRequest_SocketDomain_t SocketDomain;  // Socket Domain
	SocketCreateRequest_SocketType_t SocketType;  // Socket Type
	SocketCreateRequest_SocketProtocol_t SocketProtocol;  // Socket Protocol
} SocketCreateRequest_t;

typedef PACKED_STRUCT SocketShutdownRequest_tag {
	uint8_t SocketIndex;  // The socket index.
} SocketShutdownRequest_t;

/* Socket Domain */
typedef enum SocketBindRequest_SocketDomain_tag {
	SocketBindRequest_SocketDomain_AF_INET6 = 0x0A,
	SocketBindRequest_SocketDomain_AF_INET = 0x02
} SocketBindRequest_SocketDomain_t;

typedef PACKED_STRUCT SocketBindRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	uint8_t LocalIpAddress[16];  // The local IP address.
	uint16_t LocalPort;  // The local port.
	SocketBindRequest_SocketDomain_t SocketDomain;  // Socket Domain
} SocketBindRequest_t;

/* Flags */
typedef enum SocketSendRequest_Flags_tag {
	SocketSendRequest_Flags_MSG_DONTWAIT = 0x40
} SocketSendRequest_Flags_t;

typedef struct SocketSendRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	SocketSendRequest_Flags_t Flags;  // Flags
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} SocketSendRequest_t;

/* Flags */
typedef enum SocketSendToRequest_Flags_tag {
	SocketSendToRequest_Flags_MSG_DONTWAIT = 0x40
} SocketSendToRequest_Flags_t;

typedef struct SocketSendToRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	SocketSendToRequest_Flags_t Flags;  // Flags
	uint16_t Size;  // The number of payload bytes
	uint16_t RemotePort;  // The remote port.
	uint8_t RemoteIpAddress[16];  // The remote IP address.
	uint8_t *Data;  // Data
} SocketSendToRequest_t;

/* Flags */
typedef enum SocketReceiveRequest_Flags_tag {
	SocketReceiveRequest_Flags_MSG_DONTWAIT = 0x40
} SocketReceiveRequest_Flags_t;

typedef PACKED_STRUCT SocketReceiveRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	uint16_t DataSize;  // Data Size
	SocketReceiveRequest_Flags_t Flags;  // Flags
} SocketReceiveRequest_t;

/* Flags */
typedef enum SocketReceiveFromRequest_Flags_tag {
	SocketReceiveFromRequest_Flags_MSG_DONTWAIT = 0x40
} SocketReceiveFromRequest_Flags_t;

typedef PACKED_STRUCT SocketReceiveFromRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	uint16_t DataSize;  // Data Size
	SocketReceiveFromRequest_Flags_t Flags;  // Flags
} SocketReceiveFromRequest_t;

/* Socket Domain */
typedef enum SocketConnectRequest_SocketDomain_tag {
	SocketConnectRequest_SocketDomain_AF_INET6 = 0x0A,
	SocketConnectRequest_SocketDomain_AF_INET = 0x02
} SocketConnectRequest_SocketDomain_t;

typedef PACKED_STRUCT SocketConnectRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	uint8_t RemoteIpAddress[16];  // The remote IP address.
	uint16_t RemotePort;  // The remote port.
	SocketConnectRequest_SocketDomain_t SocketDomain;  // Socket Domain
} SocketConnectRequest_t;

typedef PACKED_STRUCT SocketListenRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	uint8_t Backlog;  // Backlog
} SocketListenRequest_t;

typedef PACKED_STRUCT SocketAcceptRequest_tag {
	uint8_t SocketIndex;  // The socket index.
} SocketAcceptRequest_t;

/* Socket Level */
typedef enum SocketSetOptionRequest_SocketLevel_tag {
	SocketSetOptionRequest_SocketLevel_SOL_SOCKET = 0x00,
	SocketSetOptionRequest_SocketLevel_SOL_IP = 0x01,
	SocketSetOptionRequest_SocketLevel_SOL_UDP = 0x02,
	SocketSetOptionRequest_SocketLevel_SOL_TCP = 0x03
} SocketSetOptionRequest_SocketLevel_t;

/* Socket Option */
typedef enum SocketSetOptionRequest_SocketOption_tag {
	SocketSetOptionRequest_SocketOption_SO_TYPE = 0x0000,
	SocketSetOptionRequest_SocketOption_SO_BINDTODEVICE = 0x0019,
	SocketSetOptionRequest_SocketOption_SO_REUSEADDR = 0x0004,
	SocketSetOptionRequest_SocketOption_IPV6_MULTICAST_HOPS = 0x0012,
	SocketSetOptionRequest_SocketOption_IPV6_UNICAST_HOPS = 0x0010,
	SocketSetOptionRequest_SocketOption_IPV6_JOIN_GROUP = 0x0014,
	SocketSetOptionRequest_SocketOption_IP_MULTICAST_TTL = 0x0021,
	SocketSetOptionRequest_SocketOption_IP_ADD_MEMBERSHIP = 0x0023,
	SocketSetOptionRequest_SocketOption_IP_DROP_MEMBERSHIP = 0x0024
} SocketSetOptionRequest_SocketOption_t;

typedef PACKED_STRUCT SocketSetOptionRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	SocketSetOptionRequest_SocketLevel_t SocketLevel;  // Socket Level
	SocketSetOptionRequest_SocketOption_t SocketOption;  // Socket Option
	uint32_t SocketOptionValue;  // Socket Option Value
} SocketSetOptionRequest_t;

/* Socket Level */
typedef enum SocketGetOptionRequest_SocketLevel_tag {
	SocketGetOptionRequest_SocketLevel_SOL_SOCKET = 0x00,
	SocketGetOptionRequest_SocketLevel_SOL_IP = 0x01,
	SocketGetOptionRequest_SocketLevel_SOL_UDP = 0x02,
	SocketGetOptionRequest_SocketLevel_SOL_TCP = 0x03
} SocketGetOptionRequest_SocketLevel_t;

/* Socket Option */
typedef enum SocketGetOptionRequest_SocketOption_tag {
	SocketGetOptionRequest_SocketOption_SO_TYPE = 0x0000,
	SocketGetOptionRequest_SocketOption_SO_RCVBUF = 0x1002
} SocketGetOptionRequest_SocketOption_t;

typedef PACKED_STRUCT SocketGetOptionRequest_tag {
	uint8_t SocketIndex;  // The socket index.
	SocketGetOptionRequest_SocketLevel_t SocketLevel;  // Socket Level
	SocketGetOptionRequest_SocketOption_t SocketOption;  // Socket Option
} SocketGetOptionRequest_t;

typedef PACKED_STRUCT MESHCOP_StartCommissionerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} MESHCOP_StartCommissionerRequest_t;

typedef PACKED_STRUCT MESHCOP_StartNativeCommissionerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} MESHCOP_StartNativeCommissionerRequest_t;

typedef PACKED_STRUCT MESHCOP_StopCommissionerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} MESHCOP_StopCommissionerRequest_t;

/* The type of the EUI */
typedef enum MESHCOP_AddExpectedJoinerRequest_EuiType_tag {
	MESHCOP_AddExpectedJoinerRequest_EuiType_ShortEUI = 0x00,
	MESHCOP_AddExpectedJoinerRequest_EuiType_LongEUI = 0x01
} MESHCOP_AddExpectedJoinerRequest_EuiType_t;

typedef struct MESHCOP_AddExpectedJoinerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	bool_t Selected;  // Allow this device?
	MESHCOP_AddExpectedJoinerRequest_EuiType_t EuiType;  // The type of the EUI
	union {
		uint32_t ShortEUI;  // ShortEUI
		uint64_t LongEUI;  // LongEUI
	} EUI;  // EUI
	uint8_t PSKdSize;  // The size of the PSKd
	char *PSKd;  // PSKd
} MESHCOP_AddExpectedJoinerRequest_t;

/* The type of the EUI */
typedef enum MESHCOP_GetExpectedJoinerRequest_EuiType_tag {
	MESHCOP_GetExpectedJoinerRequest_EuiType_ShortEUI = 0x00,
	MESHCOP_GetExpectedJoinerRequest_EuiType_LongEUI = 0x01
} MESHCOP_GetExpectedJoinerRequest_EuiType_t;

typedef struct MESHCOP_GetExpectedJoinerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	MESHCOP_GetExpectedJoinerRequest_EuiType_t EuiType;  // The type of the EUI
	union {
		uint32_t ShortEUI;  // ShortEUI
		uint64_t LongEUI;  // LongEUI
	} EUI;  // EUI
} MESHCOP_GetExpectedJoinerRequest_t;

/* The type of the EUI */
typedef enum MESHCOP_RemoveExpectedJoinerRequest_EuiType_tag {
	MESHCOP_RemoveExpectedJoinerRequest_EuiType_ShortEUI = 0x00,
	MESHCOP_RemoveExpectedJoinerRequest_EuiType_LongEUI = 0x01
} MESHCOP_RemoveExpectedJoinerRequest_EuiType_t;

typedef struct MESHCOP_RemoveExpectedJoinerRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	MESHCOP_RemoveExpectedJoinerRequest_EuiType_t EuiType;  // The type of the EUI
	union {
		uint32_t ShortEUI;  // ShortEUI
		uint64_t LongEUI;  // LongEUI
	} EUI;  // EUI
} MESHCOP_RemoveExpectedJoinerRequest_t;

typedef PACKED_STRUCT MESHCOP_RemoveAllExpectedJoinersRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
} MESHCOP_RemoveAllExpectedJoinersRequest_t;

/* The EUI mask */
typedef enum MESHCOP_SyncSteeringDataRequest_EuiMask_tag {
	MESHCOP_SyncSteeringDataRequest_EuiMask_AllZeroes = 0x00,
	MESHCOP_SyncSteeringDataRequest_EuiMask_AllFFs = 0x01,
	MESHCOP_SyncSteeringDataRequest_EuiMask_ExpectedJoiners = 0x02
} MESHCOP_SyncSteeringDataRequest_EuiMask_t;

typedef PACKED_STRUCT MESHCOP_SyncSteeringDataRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	MESHCOP_SyncSteeringDataRequest_EuiMask_t EuiMask;  // The EUI mask
} MESHCOP_SyncSteeringDataRequest_t;

typedef struct MESHCOP_MgmtSetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	bool_t PSKcEnable;  // PSKcEnable
	uint8_t PSKcSize;  // PSKc size
	char *PSKc;  // PSKc
	bool_t NetworkNameEnable;  // NetworkNameEnable
	uint8_t NetworkNameSize;  // Network Name Size
	char *NetworkName;  // Network Name
	bool_t PolicyEnable;  // PolicyEnable
	uint8_t Policy;  // OutOfBand|Native  // Policy
	uint16_t KeyRotationInterval;  // KeyRotationInterval
	bool_t TimestampEnable;  // TimestampEnable
	uint8_t Seconds[6];  // Seconds
	uint16_t Ticks;  // Ticks
} MESHCOP_MgmtSetRequest_t;

typedef enum MESHCOP_MgmtGetRequest_TlvIds_TlvId_tag {
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_PSKc = 0x04,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_Channel = 0x00,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_PanId = 0x01,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_XpanId = 0x02,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_NetworkName = 0x03,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_MasterKey = 0x05,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_KeySequence = 0x06,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_MeshLocalUla = 0x07,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_SteeringData = 0x08,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_BorderRouterLocator = 0x09,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_CommissionerID = 0x0A,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_CommissionerSessionID = 0x0B,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_SecurityPolicy = 0x0C,
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_ActiveTimestamp = 0x0E
} MESHCOP_MgmtGetRequest_TlvIds_TlvId_t;

typedef struct MESHCOP_MgmtGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	MESHCOP_MgmtGetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} MESHCOP_MgmtGetRequest_t;

typedef struct MESHCOP_SetCommissionerCredentialRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t PSKcSize;  // PSKc size
	char *PSKc;  // PSKc
	uint8_t XpanId[8];  // Extended Pan Id
	uint8_t NetworkNameSize;  // Network Name Size
	char *NetworkName;  // Network Name
} MESHCOP_SetCommissionerCredentialRequest_t;

typedef struct MESHCOP_MgmNwkFormRequest_tag {
	uint8_t IPAddress[16];  // The IPv6 address of the device added to network
	uint8_t NwkNameSize;  // Network Name Size
	char *NwkName;  // Network Name
	uint8_t NetworkMasterKey[16];  // Network Master Key
	uint8_t PSKcSize;  // Length of PSKc
	char *PSKc;  // PSKc
	uint8_t Channel;  // Channel
} MESHCOP_MgmNwkFormRequest_t;

typedef enum MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_tag {
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_BorderRouterLocator = 0x09,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_CommissionerSessionId = 0x0B,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_SteeringData = 0x08,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_Channel = 0x00,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_ChannelMask = 0x35,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_XpanId = 0x02,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_MeshLocalUla = 0x07,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_MasterKey = 0x05,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_NetworkName = 0x03,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_PanId = 0x01,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_PSKc = 0x04,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_SecurityPolicy = 0x0C,
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_ActiveTimestamp = 0x0E
} MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_t;

typedef struct MESHCOP_MgmtCommissionerGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	MESHCOP_MgmtCommissionerGetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} MESHCOP_MgmtCommissionerGetRequest_t;

typedef enum MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_tag {
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_Channel = 0x00,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_ChannelMask = 0x35,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_XpanId = 0x02,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_MeshLocalUla = 0x07,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_MasterKey = 0x05,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_NetworkName = 0x03,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_PanId = 0x01,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_PSKc = 0x04,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_SecurityPolicy = 0x0C,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_ActiveTimestamp = 0x0E,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_ScanDuration = 0x38,
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_EnergyList = 0x39
} MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_t;

typedef struct MESHCOP_MgmtActiveGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	MESHCOP_MgmtActiveGetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} MESHCOP_MgmtActiveGetRequest_t;

typedef enum MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_tag {
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_Channel = 0x00,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_ChannelMask = 0x35,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_XpanId = 0x02,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_MeshLocalUla = 0x07,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_MasterKey = 0x05,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_NetworkName = 0x03,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_PanId = 0x01,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_PSKc = 0x04,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_SecurityPolicy = 0x0C,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_DelayTimer = 0x34,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_ActiveTimestamp = 0x0E,
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_PendingTimestamp = 0x33
} MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_t;

typedef struct MESHCOP_MgmtPendingGetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	uint8_t NumberOfTlvIds;  // Number of TLV Ids
	MESHCOP_MgmtPendingGetRequest_TlvIds_TlvId_t *TlvIds;  // TlvIds

} MESHCOP_MgmtPendingGetRequest_t;

typedef struct MESHCOP_MgmtCommissionerSetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	bool_t SessionIdEnable;  // SessionIdEnable
	uint16_t SessionId;  // SessionId
	bool_t BorderRouterLocatorEnable;  // BorderRouterLocatorEnable
	uint16_t BorderRouterLocator;  // Border Router Locator
	bool_t NewSessionIdEnable;  // NewSessionIdEnable
	uint16_t NewSessionId;  // Send new session ID
	bool_t SteeringDataEnable;  // SteeringDataEnable
	uint8_t SteeringDataSize;  // The size of the Steering Data
	uint8_t *SteeringData;  // SteeringData
	bool_t ChannelEnable;  // ChannelEnable
	uint8_t Channel;  // Channel
	bool_t ChannelMaskEnable;  // ChannelMaskEnable
	uint8_t ChannelPage;  // ChannelPage
	uint8_t ChannelMaskLength;  // ChannelMaskLength
	uint8_t *ChannelMask;  // ChannelMask
	bool_t XpanIdEnable;  // XpanIdEnable
	uint8_t XpanId[8];  // XpanId
	bool_t MLPrefixEnable;  // MLPrefixEnable
	struct {
		uint8_t PrefixData[16];  // Prefix Data
		uint8_t PrefixLength;  // Prefix length in bits
	} MLPrefix;  // ML Prefix
	bool_t MasterKeyEnable;  // MasterKeyEnable
	uint8_t MasterKey[16];  // MasterKey
	bool_t NwkNameEnable;  // NwkNameEnable
	uint8_t NwkNameSize;  // Network Name Size
	char *NwkName;  // Network Name
	bool_t PanIdEnable;  // PanIdEnable
	uint16_t PanId;  // PanId
	bool_t PSKcEnable;  // PSKcEnable
	uint8_t PskcSize;  // PskcSize
	char *PSKc;  // PSKc
	bool_t PolicyEnable;  // PolicyEnable
	uint16_t RotationInterval;  // RotationInterval
	uint8_t Policy;  // OutOfBand|Native  // Policy
	bool_t ActiveTimestampEnable;  // Will be applied after Delay Timer expires
	uint8_t ActiveSeconds[6];  // Active timestamp seconds
	uint16_t ActiveTicks;  // Active timestamp ticks
	bool_t PendingTimestampEnable;  // Used to compare multiple Pending Set requests
	uint8_t PendingSeconds[6];  // Pending timestamp Seconds
	uint16_t PendingTicks;  // Pending timestamp ticks
	bool_t DelayTimerEnable;  // DelayTimerEnable
	uint32_t Timeout;  // Timeout(ms)
	bool_t FutureTlvEnable;  // Future Tlv Enable - for certification purposes
	struct {
		uint8_t FutureTlvSize;  // Future Tlv Size - for certification purposes
		uint8_t *FutureTlvValue;  // Future Tlv Value - for certification purposes
	} FutureTlv;  // Future TLV Structure - for certification purposes
} MESHCOP_MgmtCommissionerSetRequest_t;

typedef struct MESHCOP_MgmtActiveSetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	bool_t SessionIdEnable;  // SessionIdEnable
	uint16_t SessionId;  // SessionId
	bool_t BorderRouterLocatorEnable;  // BorderRouterLocatorEnable
	uint16_t BorderRouterLocator;  // Border Router Locator
	bool_t NewSesswionIdEnable;  // NewSesswionIdEnable
	uint16_t NewSesswionId;  // Send new session ID
	bool_t SteeringDataEnable;  // SteeringDataEnable
	uint8_t SteeringDataSize;  // The size of the Steering Data
	uint8_t *SteeringData;  // SteeringData
	bool_t ChannelEnable;  // ChannelEnable
	uint8_t Channel;  // Channel
	bool_t ChannelMaskEnable;  // ChannelMaskEnable
	uint8_t ChannelPage;  // ChannelPage
	uint8_t ChannelMaskLength;  // ChannelMaskLength
	uint8_t *ChannelMask;  // ChannelMask
	bool_t XpanIdEnable;  // XpanIdEnable
	uint8_t XpanId[8];  // XpanId
	bool_t MLPrefixEnable;  // MLPrefixEnable
	struct {
		uint8_t PrefixData[16];  // Prefix Data
		uint8_t PrefixLength;  // Prefix length in bits
	} MLPrefix;  // ML Prefix
	bool_t MasterKeyEnable;  // MasterKeyEnable
	uint8_t MasterKey[16];  // MasterKey
	bool_t NwkNameEnable;  // NwkNameEnable
	uint8_t NwkNameSize;  // Network Name Size
	char *NwkName;  // Network Name
	bool_t PanIdEnable;  // PanIdEnable
	uint16_t PanId;  // PanId
	bool_t PSKcEnable;  // PSKcEnable
	uint8_t PskcSize;  // PskcSize
	char *PSKc;  // PSKc
	bool_t PolicyEnable;  // PolicyEnable
	uint16_t RotationInterval;  // RotationInterval
	uint8_t Policy;  // O|N|R|C|B  // Policy
	bool_t ActiveTimestampEnable;  // Will be applied after Delay Timer expires
	uint8_t ActiveSeconds[6];  // Active timestamp seconds
	uint16_t ActiveTicks;  // Active timestamp ticks
	bool_t PendingTimestampEnable;  // Used to compare multiple Pending Set requests
	uint8_t PendingSeconds[6];  // Pending timestamp Seconds
	uint16_t PendingTicks;  // Pending timestamp ticks
	bool_t DelayTimerEnable;  // DelayTimerEnable
	uint32_t Timeout;  // Timeout(ms)
	bool_t FutureTlvEnable;  // Future Tlv Enable - for certification purposes
	struct {
		uint8_t FutureTlvSize;  // Future Tlv Size - for certification purposes
		uint8_t *FutureTlvValue;  // Future Tlv Value - for certification purposes
	} FutureTlv;  // Future TLV Structure - for certification purposes
} MESHCOP_MgmtActiveSetRequest_t;

typedef struct MESHCOP_MgmtPendingSetRequest_tag {
	uint8_t InstanceId;  // Thread instance Id
	uint8_t IPAddress[16];  // The IPv6 address
	bool_t SessionIdEnable;  // SessionIdEnable
	uint16_t SessionId;  // SessionId
	bool_t BorderRouterLocatorEnable;  // BorderRouterLocatorEnable
	uint16_t BorderRouterLocator;  // Border Router Locator
	bool_t NewSesswionIdEnable;  // NewSesswionIdEnable
	uint16_t NewSesswionId;  // Send new session ID
	bool_t SteeringDataEnable;  // SteeringDataEnable
	uint8_t SteeringDataSize;  // The size of the Steering Data
	char *SteeringData;  // SteeringData
	bool_t ChannelEnable;  // ChannelEnable
	uint8_t Channel;  // Channel
	bool_t ChannelMaskEnable;  // ChannelMaskEnable
	uint8_t ChannelPage;  // ChannelPage
	uint8_t ChannelMaskLength;  // ChannelMaskLength
	uint8_t *ChannelMask;  // ChannelMask
	bool_t XpanIdEnable;  // XpanIdEnable
	uint8_t XpanId[8];  // XpanId
	bool_t MLPrefixEnable;  // MLPrefixEnable
	struct {
		uint8_t PrefixData[16];  // Prefix Data
		uint8_t PrefixLength;  // Prefix length in bits
	} MLPrefix;  // ML Prefix
	bool_t MasterKeyEnable;  // MasterKeyEnable
	uint8_t MasterKey[16];  // MasterKey
	bool_t NwkNameEnable;  // NwkNameEnable
	uint8_t NwkNameSize;  // Network Name Size
	char *NwkName;  // Network Name
	bool_t PanIdEnable;  // PanIdEnable
	uint16_t PanId;  // PanId
	bool_t PSKcEnable;  // PSKcEnable
	uint8_t PskcSize;  // PskcSize
	char *PSKc;  // PSKc
	bool_t PolicyEnable;  // PolicyEnable
	uint16_t RotationInterval;  // RotationInterval
	uint8_t Policy;  // OutOfBand|Native  // Policy
	bool_t ActiveTimestampEnable;  // Will be applied after Delay Timer expires
	uint8_t ActiveSeconds[6];  // Active timestamp seconds
	uint16_t ActiveTicks;  // Active timestamp ticks
	bool_t PendingTimestampEnable;  // Used to compare multiple Pending Set requests
	uint8_t PendingSeconds[6];  // Pending timestamp Seconds
	uint16_t PendingTicks;  // Pending timestamp ticks
	bool_t DelayTimerEnable;  // DelayTimerEnable
	uint32_t Timeout;  // Timeout(ms)
	bool_t FutureTlvEnable;  // Future Tlv Enable - for certification purposes
	struct {
		uint8_t FutureTlvSize;  // Future Tlv Size - for certification purposes
		uint8_t *FutureTlvValue;  // Future Tlv Value - for certification purposes
	} FutureTlv;  // Future TLV Structure - for certification purposes
} MESHCOP_MgmtPendingSetRequest_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendPanIdQueryRequest_tag {
	uint8_t ThrInstanceID;  // Thread instance ID
	uint32_t ScanChannelMask;  // Channel mask
	uint16_t PanId;  // PanId
	uint8_t IPAddress[16];  // The IPv6 address of the device added to network
} MESHCOP_MgmtSendPanIdQueryRequest_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendEdScanRequest_tag {
	uint8_t ThrInstanceID;  // Thread instance ID
	uint32_t ScanChannelMask;  // Channel mask
	uint8_t Count;  // The number of IEEE 802.15.4 ED Scans per channel
	uint16_t Period;  // The period between successive IEEE 802.15.4 ED Scans
	uint16_t ScanDuration;  // The IEEE 802.15.4 ScanDuration to use when performing an IEEE 802.15.4 ED Scan
	uint8_t IPAddress[16];  // The IPv6 address of the device added to network
} MESHCOP_MgmtSendEdScanRequest_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendAnnounceBeginRequest_tag {
	uint8_t ThrInstanceID;  // Thread instance ID
	uint16_t CommissionerSessionID;  // Commissioner Session ID. Set to 0 to use the current value on the device.
	uint32_t ChannelMask;  // Channel mask
	uint8_t Count;  // The number of MGMT_ANNOUNCE.ntf that the destination device should send.
	uint16_t Period;  // The period between successive MGMT_ANNOUNCE.ntf frames
	uint8_t IPAddress[16];  // The IPv6 address of the destination. Unicast or multicast.
} MESHCOP_MgmtSendAnnounceBeginRequest_t;

typedef PACKED_STRUCT NWKU_IfconfigBindRequest_tag {
	uint8_t IpAddress[16];  // Ip Address
	uint8_t InterfaceId;  // Interface Id
} NWKU_IfconfigBindRequest_t;

typedef PACKED_STRUCT NWKU_PingRequest_tag {
	uint8_t DestinationIpAddress[16];  // Destination Ip Address
	uint8_t SourceIpAddress[16];  // Source Ip Address
	uint16_t Payloadlength;  // Payload Length
	uint16_t Timeout;  // Timeout
	bool_t Secured;  // Set security for ping packets
} NWKU_PingRequest_t;

typedef PACKED_STRUCT NWKU_EchoUDPRequest_tag {
	uint8_t DestinationIpAddress[16];  // Destination Ip Address
	uint8_t SourceIpAddress[16];  // Source Ip Address
	uint16_t Payloadlength;  // Payload Length
	uint16_t Timeout;  // Timeout
	uint8_t Iterations;  // Number of iterations
} NWKU_EchoUDPRequest_t;

/* Socket Domain */
typedef enum NWKU_CoapCreateInstanceRequest_SocketDomain_tag {
	NWKU_CoapCreateInstanceRequest_SocketDomain_AF_INET6 = 0x0A,
	NWKU_CoapCreateInstanceRequest_SocketDomain_AF_INET = 0x02
} NWKU_CoapCreateInstanceRequest_SocketDomain_t;

typedef PACKED_STRUCT NWKU_CoapCreateInstanceRequest_tag {
	uint16_t UDPPort;  // UDP Port
	NWKU_CoapCreateInstanceRequest_SocketDomain_t SocketDomain;  // Socket Domain
} NWKU_CoapCreateInstanceRequest_t;

/* CoAP request type */
typedef enum NWKU_CoapSendRequest_RequestType_tag {
	NWKU_CoapSendRequest_RequestType_CON = 0x00,
	NWKU_CoapSendRequest_RequestType_NON = 0x01
} NWKU_CoapSendRequest_RequestType_t;

/* CoAP message type */
typedef enum NWKU_CoapSendRequest_MessageType_tag {
	NWKU_CoapSendRequest_MessageType_GET = 0x01,
	NWKU_CoapSendRequest_MessageType_POST = 0x02,
	NWKU_CoapSendRequest_MessageType_PUT = 0x03,
	NWKU_CoapSendRequest_MessageType_DELETE = 0x04
} NWKU_CoapSendRequest_MessageType_t;

typedef struct NWKU_CoapSendRequest_tag {
	uint8_t coapSession[4];
	uint8_t InstanceID;  // InstanceID
	uint8_t DestinationIpAddress[16];  // Destination Ip Address
	uint16_t UDPPort;  // UDPPort
	NWKU_CoapSendRequest_RequestType_t RequestType;  // CoAP request type
	NWKU_CoapSendRequest_MessageType_t MessageType;  // CoAP message type
	char URIpath[30];  // URI-path options separated by /
	uint8_t PayloadLength;  // CoAP message payload length
	char *Payload;  // CoAP message payload
} NWKU_CoapSendRequest_t;

/* Socket Domain */
typedef enum NWKU_CoapRegisterRequest_SocketDomain_tag {
	NWKU_CoapRegisterRequest_SocketDomain_AF_INET6 = 0x0A,
	NWKU_CoapRegisterRequest_SocketDomain_AF_INET = 0x02
} NWKU_CoapRegisterRequest_SocketDomain_t;

typedef PACKED_STRUCT NWKU_CoapRegisterRequest_tag {
	uint8_t InstanceID;  // InstanceID
	NWKU_CoapRegisterRequest_SocketDomain_t SocketDomain;  // Socket Domain
	char URIpath[30];  // URI-path options separated by /
	uint16_t Port;  // UDP Port
} NWKU_CoapRegisterRequest_t;

typedef PACKED_STRUCT NWKU_DnsSendRequestRequest_tag {
	char DomainName[30];  // Domain Name
} NWKU_DnsSendRequestRequest_t;

typedef PACKED_STRUCT NWKU_McastGroupShowRequest_tag {
	uint8_t InterfaceId;  // Interface Id
} NWKU_McastGroupShowRequest_t;

/* Join or leave a multicast group */
typedef enum NWKU_McastGroupManageRequest_Action_tag {
	NWKU_McastGroupManageRequest_Action_JoinGroup = 0x01,
	NWKU_McastGroupManageRequest_Action_LeaveGroup = 0x02
} NWKU_McastGroupManageRequest_Action_t;

typedef PACKED_STRUCT NWKU_McastGroupManageRequest_tag {
	uint8_t InterfaceId;  // Interface Id
	NWKU_McastGroupManageRequest_Action_t Action;  // Join or leave a multicast group
	uint8_t McastAddress[16];  // Multicast group address
} NWKU_McastGroupManageRequest_t;

typedef PACKED_STRUCT DTLSOpenRequest_tag {
	uint8_t Maximumretransmissionscount;  // Maximum Retransmissions Count
	uint16_t Timeout;  // Timeout
	uint16_t LocalPort;  // The local port.
} DTLSOpenRequest_t;

typedef PACKED_STRUCT DTLSCloseRequest_tag {
	uint8_t ContextNumber;  // Context Number
} DTLSCloseRequest_t;

typedef PACKED_STRUCT DTLSClosePeerRequest_tag {
	uint8_t PeerNumber;  // Peer Number
} DTLSClosePeerRequest_t;

typedef PACKED_STRUCT DTLSConnectRequest_tag {
	uint8_t ContextNumber;  // Context Number
	uint8_t ServerIPAddress[16];  // Server IP Address
	uint16_t ServerPort;  // Server Port
} DTLSConnectRequest_t;

typedef struct DTLSSendRequest_tag {
	uint8_t PeerNumber;  // Peer Number
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} DTLSSendRequest_t;

typedef struct SerialTun_IPPacketSendRequest_tag {
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} SerialTun_IPPacketSendRequest_t;

/* The MAC PIB attribute identifier */
typedef enum Sniffer_MacSetPIBAttributeRequest_PIBAttribute_tag {
	Sniffer_MacSetPIBAttributeRequest_PIBAttribute_macLogicalChannel = 0x21,
	Sniffer_MacSetPIBAttributeRequest_PIBAttribute_macMAC_PromiscuousRxIndicationMode = 0x51,
	Sniffer_MacSetPIBAttributeRequest_PIBAttribute_macRxOnWhenIdle = 0x52
} Sniffer_MacSetPIBAttributeRequest_PIBAttribute_t;

typedef PACKED_STRUCT Sniffer_MacSetPIBAttributeRequest_tag {
	Sniffer_MacSetPIBAttributeRequest_PIBAttribute_t PIBAttribute;  // The MAC PIB attribute identifier
	uint8_t Value[7];  // The value to set the attribute to
} Sniffer_MacSetPIBAttributeRequest_t;

/* Status */
typedef enum FSCIACK_Status_tag {
	FSCIACK_Status_OK = 0x00,
	FSCIACK_Status_ERROR = 0xFF
} FSCIACK_Status_t;

typedef PACKED_STRUCT FSCIACK_tag {
	FSCIACK_Status_t Status;  // Status
} FSCIACK_t;

typedef PACKED_STRUCT AspSetPowerLevelRequest_tag {
	uint8_t powerLevel;  // Power Level
} AspSetPowerLevelRequest_t;

typedef PACKED_STRUCT SocketCreateConfirm_tag {
	uint8_t SocketIndex;  // The socket index.
} SocketCreateConfirm_t;

/* Status */
typedef enum SocketShutdownConfirm_Status_tag {
	SocketShutdownConfirm_Status_OK = 0x00,
	SocketShutdownConfirm_Status_ERROR = 0xFF
} SocketShutdownConfirm_Status_t;

typedef PACKED_STRUCT SocketShutdownConfirm_tag {
	SocketShutdownConfirm_Status_t Status;  // Status
} SocketShutdownConfirm_t;

/* Status */
typedef enum SocketBindConfirm_Status_tag {
	SocketBindConfirm_Status_OK = 0x00,
	SocketBindConfirm_Status_ERROR = 0xFF
} SocketBindConfirm_Status_t;

typedef PACKED_STRUCT SocketBindConfirm_tag {
	SocketBindConfirm_Status_t Status;  // Status
} SocketBindConfirm_t;

/* Status */
typedef enum SocketSendConfirm_Status_tag {
	SocketSendConfirm_Status_OK = 0x00,
	SocketSendConfirm_Status_ERROR = 0xFF
} SocketSendConfirm_Status_t;

typedef PACKED_STRUCT SocketSendConfirm_tag {
	SocketSendConfirm_Status_t Status;  // Status
} SocketSendConfirm_t;

/* Status */
typedef enum SocketSendToConfirm_Status_tag {
	SocketSendToConfirm_Status_OK = 0x00,
	SocketSendToConfirm_Status_ERROR = 0xFF
} SocketSendToConfirm_Status_t;

typedef PACKED_STRUCT SocketSendToConfirm_tag {
	SocketSendToConfirm_Status_t Status;  // Status
} SocketSendToConfirm_t;

typedef struct SocketReceiveConfirm_tag {
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} SocketReceiveConfirm_t;

/* Status */
typedef enum SocketReceiveFromConfirm_Status_tag {
	SocketReceiveFromConfirm_Status_OK = 0x00,
	SocketReceiveFromConfirm_Status_ERROR = 0xFF
} SocketReceiveFromConfirm_Status_t;

typedef struct SocketReceiveFromConfirm_tag {
	SocketReceiveFromConfirm_Status_t Status;  // Status
	uint8_t RemoteIpAddress[16];  // The remote IP address.
	uint16_t RemotePort;  // The remote port.
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} SocketReceiveFromConfirm_t;

/* Status */
typedef enum SocketConnectConfirm_Status_tag {
	SocketConnectConfirm_Status_OK = 0x00,
	SocketConnectConfirm_Status_ERROR = 0xFF
} SocketConnectConfirm_Status_t;

typedef PACKED_STRUCT SocketConnectConfirm_tag {
	SocketConnectConfirm_Status_t Status;  // Status
} SocketConnectConfirm_t;

/* Status */
typedef enum SocketListenConfirm_Status_tag {
	SocketListenConfirm_Status_OK = 0x00,
	SocketListenConfirm_Status_ERROR = 0xFF
} SocketListenConfirm_Status_t;

typedef PACKED_STRUCT SocketListenConfirm_tag {
	SocketListenConfirm_Status_t Status;  // Status
} SocketListenConfirm_t;

/* Status */
typedef enum SocketAcceptConfirm_Status_tag {
	SocketAcceptConfirm_Status_OK = 0x00,
	SocketAcceptConfirm_Status_ERROR = 0xFF
} SocketAcceptConfirm_Status_t;

typedef PACKED_STRUCT SocketAcceptConfirm_tag {
	SocketAcceptConfirm_Status_t Status;  // Status
	uint8_t ConnectedSocketIndex;  // The connected socket index.
} SocketAcceptConfirm_t;

/* Status */
typedef enum SocketSetOptionConfirm_Status_tag {
	SocketSetOptionConfirm_Status_OK = 0x00,
	SocketSetOptionConfirm_Status_ERROR = 0xFF
} SocketSetOptionConfirm_Status_t;

typedef PACKED_STRUCT SocketSetOptionConfirm_tag {
	SocketSetOptionConfirm_Status_t Status;  // Status
} SocketSetOptionConfirm_t;

/* Status */
typedef enum SocketGetOptionConfirm_Status_tag {
	SocketGetOptionConfirm_Status_OK = 0x00,
	SocketGetOptionConfirm_Status_ERROR = 0xFF
} SocketGetOptionConfirm_Status_t;

typedef PACKED_STRUCT SocketGetOptionConfirm_tag {
	SocketGetOptionConfirm_Status_t Status;  // Status
	uint32_t OptionValue;  // Option Value
} SocketGetOptionConfirm_t;

/* Status */
typedef enum MAC_MacFilteringAddEntryConfirm_Status_tag {
	MAC_MacFilteringAddEntryConfirm_Status_Success = 0x00,
	MAC_MacFilteringAddEntryConfirm_Status_Notpermitted = 0x04,
	MAC_MacFilteringAddEntryConfirm_Status_Nomemory = 0x06
} MAC_MacFilteringAddEntryConfirm_Status_t;

typedef PACKED_STRUCT MAC_MacFilteringAddEntryConfirm_tag {
	MAC_MacFilteringAddEntryConfirm_Status_t Status;  // Status
} MAC_MacFilteringAddEntryConfirm_t;

/* Status */
typedef enum MAC_MacFilteringRemoveEntryConfirm_Status_tag {
	MAC_MacFilteringRemoveEntryConfirm_Status_Success = 0x00,
	MAC_MacFilteringRemoveEntryConfirm_Status_Notpermitted = 0x04
} MAC_MacFilteringRemoveEntryConfirm_Status_t;

typedef PACKED_STRUCT MAC_MacFilteringRemoveEntryConfirm_tag {
	MAC_MacFilteringRemoveEntryConfirm_Status_t Status;  // Status
} MAC_MacFilteringRemoveEntryConfirm_t;

/* Status */
typedef enum MAC_MacFilteringEnableConfirm_Status_tag {
	MAC_MacFilteringEnableConfirm_Status_Success = 0x00,
	MAC_MacFilteringEnableConfirm_Status_Notpermitted = 0x04
} MAC_MacFilteringEnableConfirm_Status_t;

typedef PACKED_STRUCT MAC_MacFilteringEnableConfirm_tag {
	MAC_MacFilteringEnableConfirm_Status_t Status;  // Status
} MAC_MacFilteringEnableConfirm_t;

typedef struct MAC_MacFilteringGetTableConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t NoOfElements;  // No Of Elements
	struct {
		uint64_t ExtendedAddress;  // Extended Address
		uint16_t ShortAddress;  // Short Address
		uint8_t LinkIndicator;  // Link Indicator
		bool_t BlockedNeighbor;  // This neighbor is blacklisted
	} *MacFilteringEntries;  // Mac Filtering Entry
} MAC_MacFilteringGetTableConfirm_t;

/* Attribute Status */
typedef enum THR_SetDeviceConfigConfirm_Status_tag {
	THR_SetDeviceConfigConfirm_Status_Success = 0x00,
	THR_SetDeviceConfigConfirm_Status_Invalidinstance = 0x02,
	THR_SetDeviceConfigConfirm_Status_Invalidparameter = 0x03,
	THR_SetDeviceConfigConfirm_Status_Notpermitted = 0x04,
	THR_SetDeviceConfigConfirm_Status_UnsupportedAttribute = 0x07,
	THR_SetDeviceConfigConfirm_Status_EmptyEntry = 0x08,
	THR_SetDeviceConfigConfirm_Status_InvalidValue = 0x09
} THR_SetDeviceConfigConfirm_Status_t;

typedef PACKED_STRUCT THR_SetDeviceConfigConfirm_tag {
	THR_SetDeviceConfigConfirm_Status_t Status;  // Attribute Status
} THR_SetDeviceConfigConfirm_t;

/* Status */
typedef enum THR_NwkScanConfirm_Status_tag {
	THR_NwkScanConfirm_Status_Success = 0x00,
	THR_NwkScanConfirm_Status_Invalidinstance = 0x02,
	THR_NwkScanConfirm_Status_InvalidParam = 0x03,
	THR_NwkScanConfirm_Status_Nomemory = 0x06,
	THR_NwkScanConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_NwkScanConfirm_Status_t;

typedef PACKED_STRUCT THR_NwkScanConfirm_tag {
	THR_NwkScanConfirm_Status_t Status;  // Status
} THR_NwkScanConfirm_t;

/* Status */
typedef enum THR_CreateNwkConfirm_Status_tag {
	THR_CreateNwkConfirm_Status_OK = 0x00,
	THR_CreateNwkConfirm_Status_AlreadyConnected = 0x0A,
	THR_CreateNwkConfirm_Status_AlreadyCreated = 0x0B,
	THR_CreateNwkConfirm_Status_Invalidinstance = 0x02,
	THR_CreateNwkConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_CreateNwkConfirm_Status_t;

typedef PACKED_STRUCT THR_CreateNwkConfirm_tag {
	THR_CreateNwkConfirm_Status_t Status;  // Status
} THR_CreateNwkConfirm_t;

/* Status */
typedef enum THR_JoinConfirm_Status_tag {
	THR_JoinConfirm_Status_OK = 0x00,
	THR_JoinConfirm_Status_Invalidinstance = 0x02,
	THR_JoinConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_JoinConfirm_Status_t;

typedef PACKED_STRUCT THR_JoinConfirm_tag {
	THR_JoinConfirm_Status_t Status;  // Status
} THR_JoinConfirm_t;

/* Status */
typedef enum THR_FactoryResetConfirm_Status_tag {
	THR_FactoryResetConfirm_Status_Success = 0x00,
	THR_FactoryResetConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_FactoryResetConfirm_Status_t;

typedef PACKED_STRUCT THR_FactoryResetConfirm_tag {
	THR_FactoryResetConfirm_Status_t Status;  // Status
} THR_FactoryResetConfirm_t;

/* Status */
typedef enum THR_CpuResetConfirm_Status_tag {
	THR_CpuResetConfirm_Status_Success = 0x00,
	THR_CpuResetConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_CpuResetConfirm_Status_t;

typedef PACKED_STRUCT THR_CpuResetConfirm_tag {
	THR_CpuResetConfirm_Status_t Status;  // Status
} THR_CpuResetConfirm_t;

/* Status */
typedef enum THR_CpuResetIndication_Status_tag {
	THR_CpuResetIndication_Status_ResetCpuSuccess = 0x00,
	THR_CpuResetIndication_Status_ResetCpuPending = 0x01
} THR_CpuResetIndication_Status_t;

/* Stack Vendor OUI */
typedef enum THR_CpuResetIndication_ResetCpuPayload_ResetCpuSuccess_StackVersionStruct_StackVendorOUI_tag {
	THR_CpuResetIndication_ResetCpuPayload_ResetCpuSuccess_StackVersionStruct_StackVendorOUI_NXP = 0x006037
} THR_CpuResetIndication_ResetCpuPayload_ResetCpuSuccess_StackVersionStruct_StackVendorOUI_t;

typedef struct THR_CpuResetIndication_tag {
	THR_CpuResetIndication_Status_t Status;  // Status
	union {
		struct {
			uint8_t BoardNameLen;  // Board Name Length
			char *BoardName;  // BoardName
			uint8_t UniqueMcuId[16];  // Unique Mcu Identifier
			struct {
				THR_CpuResetIndication_ResetCpuPayload_ResetCpuSuccess_StackVersionStruct_StackVendorOUI_t StackVendorOUI;  // Stack Vendor OUI
				uint8_t StackVersion[3];  // BuildNo|RevNo|MinorNo|MajorNo  // Stack Version
			} StackVersionStruct;  // Stack Version Structure
			uint8_t SwVersionLen;  // Software Version Length
			char *SwVersion;  // SwVersion
		} ResetCpuSuccess;  // Reset Cpu success payload
		uint32_t ResetCpuPending_TimeoutMs;  // Reset Cpu pending payload

	} ResetCpuPayload;  // Reset CPU Payload
} THR_CpuResetIndication_t;

/* Status */
typedef enum THR_DisconnectConfirm_Status_tag {
	THR_DisconnectConfirm_Status_OK = 0x00,
	THR_DisconnectConfirm_Status_Invalidinstance = 0x02,
	THR_DisconnectConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_DisconnectConfirm_Status_t;

typedef PACKED_STRUCT THR_DisconnectConfirm_tag {
	THR_DisconnectConfirm_Status_t Status;  // Status
} THR_DisconnectConfirm_t;

/* Status */
typedef enum THR_AttachConfirm_Status_tag {
	THR_AttachConfirm_Status_OK = 0x00,
	THR_AttachConfirm_Status_Invalidinstance = 0x02,
	THR_AttachConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_AttachConfirm_Status_t;

typedef PACKED_STRUCT THR_AttachConfirm_tag {
	THR_AttachConfirm_Status_t Status;  // Status
} THR_AttachConfirm_t;

/* Status */
typedef enum THR_PromoteAsRouterConfirm_Status_tag {
	THR_PromoteAsRouterConfirm_Status_OK = 0x00
} THR_PromoteAsRouterConfirm_Status_t;

typedef PACKED_STRUCT THR_PromoteAsRouterConfirm_tag {
	THR_PromoteAsRouterConfirm_Status_t Status;  // Status
} THR_PromoteAsRouterConfirm_t;

/* Event status */
typedef enum THR_EventNwkScanConfirm_EventStatus_tag {
	THR_EventNwkScanConfirm_EventStatus_ScanResult = 0x0001
} THR_EventNwkScanConfirm_EventStatus_t;

/* ScanType */
typedef enum THR_EventNwkScanConfirm_ScanType_tag {
	THR_EventNwkScanConfirm_ScanType_EnergyDetect = 0x01,
	THR_EventNwkScanConfirm_ScanType_ActiveScan = 0x02,
	THR_EventNwkScanConfirm_ScanType_EnergyDetectAndActiveScan = 0x03
} THR_EventNwkScanConfirm_ScanType_t;

typedef struct THR_EventNwkScanConfirm_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventNwkScanConfirm_EventStatus_t EventStatus;  // Event status
	uint16_t DataSize;  // The number of payload bytes
	uint32_t ScanChannelMask;  // ScanChannelMask
	THR_EventNwkScanConfirm_ScanType_t ScanType;  // ScanType
	uint8_t ScanDuration;  // ScanDuration
	uint16_t maxThrNwkToDiscover;  // maxThrNwkToDiscover
	union {
		struct {
			uint8_t EnergyDetectEntries;  // Number of Energy Detect Entries
			uint8_t *EnergyDetectList;  // Energy Detect List
		} EnergyDetect;  // Active Scan
		struct {
			uint8_t NwkDiscoveryEntries;  // Number of network discovery Entries
			struct {
				uint16_t NumOfRcvdBeacons;  // numOfRcvdBeacons
				uint16_t PanId;  // PanId
				uint8_t Channel;  // Channel
				uint8_t Reserved;  // Reserved
			} *NwkDiscoveryList;  // Nwk Discovery List
		} ActiveScan;  // Active Scan
		struct {
			uint8_t EnergyDetectEntries;  // Number of Energy Detect Entries
			uint8_t *EnergyDetectList;  // Energy Detect List
			uint8_t NwkDiscoveryEntries;  // Number of network discovery Entries
			struct {
				uint16_t NumOfRcvdBeacons;  // numOfRcvdBeacons
				uint16_t PanId;  // PanId
				uint8_t Channel;  // Channel
				uint8_t Reserved;  // Reserved
			} *NwkDiscoveryList;  // Nwk Discovery List
		} EnergyDetectAndActiveScan;  // EnergyDetectAndActiveScan
	} Data;  // Data
} THR_EventNwkScanConfirm_t;

/* Event status */
typedef enum THR_EventNwkCreateConfirm_EventStatus_tag {
	THR_EventNwkCreateConfirm_EventStatus_Success = 0x0001,
	THR_EventNwkCreateConfirm_EventStatus_Failed = 0x0002,
	THR_EventNwkCreateConfirm_EventStatus_SelectBestChannel = 0x0003,
	THR_EventNwkCreateConfirm_EventStatus_GeneratePSKc = 0x0004
} THR_EventNwkCreateConfirm_EventStatus_t;

typedef struct THR_EventNwkCreateConfirm_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventNwkCreateConfirm_EventStatus_t EventStatus;  // Event status
	uint16_t DataSize;  // The number of payload bytes
	uint8_t *Data;  // Data
} THR_EventNwkCreateConfirm_t;

/* Event status */
typedef enum THR_EventNwkJoinConfirm_EventStatus_tag {
	THR_EventNwkJoinConfirm_EventStatus_Attaching = 0x0001,
	THR_EventNwkJoinConfirm_EventStatus_JoinSuccess = 0x0002,
	THR_EventNwkJoinConfirm_EventStatus_JoinFailed = 0x0003
} THR_EventNwkJoinConfirm_EventStatus_t;

typedef struct THR_EventNwkJoinConfirm_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventNwkJoinConfirm_EventStatus_t EventStatus;  // Event status
	uint16_t DataSize;  // The number of payload bytes
	uint8_t *Data;  // Data
} THR_EventNwkJoinConfirm_t;

/* Event status */
typedef enum THR_EventNwkJoinSelectParentsConfirm_EventStatus_tag {
	THR_EventNwkJoinSelectParentsConfirm_EventStatus_ScanStarted = 0x0001,
	THR_EventNwkJoinSelectParentsConfirm_EventStatus_ReceivedBeacon = 0x0002,
	THR_EventNwkJoinSelectParentsConfirm_EventStatus_ScanEnded = 0x0003
} THR_EventNwkJoinSelectParentsConfirm_EventStatus_t;

typedef struct THR_EventNwkJoinSelectParentsConfirm_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventNwkJoinSelectParentsConfirm_EventStatus_t EventStatus;  // Event status
	uint16_t DataSize;  // The number of payload bytes
	uint8_t *Data;  // Data
} THR_EventNwkJoinSelectParentsConfirm_t;

/* Event status */
typedef enum THR_EventGeneralConfirm_EventStatus_tag {
	THR_EventGeneralConfirm_EventStatus_Disconnected = 0x0001,
	THR_EventGeneralConfirm_EventStatus_Connected = 0x0002,
	THR_EventGeneralConfirm_EventStatus_Resettofactorydefault = 0x0003,
	THR_EventGeneralConfirm_EventStatus_Instancerestorestarted = 0x0004,
	THR_EventGeneralConfirm_EventStatus_RouterSynced = 0x0005,
	THR_EventGeneralConfirm_EventStatus_EndDeviceSynced = 0x0006,
	THR_EventGeneralConfirm_EventStatus_Connectingstarted = 0x0007,
	THR_EventGeneralConfirm_EventStatus_Connectingfailed = 0x0008,
	THR_EventGeneralConfirm_EventStatus_Connectingdeffered = 0x0009,
	THR_EventGeneralConfirm_EventStatus_DeviceisLeader = 0x000A,
	THR_EventGeneralConfirm_EventStatus_DeviceisRouter = 0x000B,
	THR_EventGeneralConfirm_EventStatus_DeviceisREED = 0x000C,
	THR_EventGeneralConfirm_EventStatus_DeviceisEndDevice = 0x000D,
	THR_EventGeneralConfirm_EventStatus_DeviceisSleepyEndDevice = 0x000E,
	THR_EventGeneralConfirm_EventStatus_RequestingGlobalAddress = 0x000F,
	THR_EventGeneralConfirm_EventStatus_GlobalAddressassigned = 0x0010,
	THR_EventGeneralConfirm_EventStatus_RequestingRouterId = 0x0011,
	THR_EventGeneralConfirm_EventStatus_RouterIdassigned = 0x0012,
	THR_EventGeneralConfirm_EventStatus_RouterIdassignedfailed = 0x0013,
	THR_EventGeneralConfirm_EventStatus_Allowdevicetosleep = 0x0014,
	THR_EventGeneralConfirm_EventStatus_Disallowdevicetosleep = 0x0015,
	THR_EventGeneralConfirm_EventStatus_ChildIdassigned = 0x0016
} THR_EventGeneralConfirm_EventStatus_t;

typedef struct THR_EventGeneralConfirm_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventGeneralConfirm_EventStatus_t EventStatus;  // Event status
	uint16_t DataSize;  // The number of payload bytes
	uint8_t *Data;  // Data
} THR_EventGeneralConfirm_t;

/* Event status */
typedef enum THR_EventNwkCommissioningIndication_EventStatus_tag {
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryStarted = 0x0001,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryFailedNoBeacon = 0x0002,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoveryFailedFiltered = 0x0003,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDiscoverySuccess = 0x0004,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDtlsSessionStarted = 0x0005,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerDtlsError = 0x0006,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerError = 0x0007,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerAccepted = 0x0008,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionStarted = 0x0009,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionAccepted = 0x000A,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionRejected = 0x000B,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerPetitionError = 0x000C,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerKeepAliveSent = 0x000D,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerError = 0x000E,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerDtlsSessionStarted = 0x000F,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerDtlsError = 0x0010,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerJoinerAccepted = 0x0011,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerNwkDataSynced = 0x0012,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrDtlsSessionStarted = 0x0013,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrDtlsError = 0x0014,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrError = 0x0015,
	THR_EventNwkCommissioningIndication_EventStatus_CommissionerBrAccepted = 0x0016,
	THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDtlsSessionStarted = 0x0017,
	THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDtlsError = 0x0018,
	THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerAccepted = 0x0019,
	THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDataRelayedInbound = 0x001A,
	THR_EventNwkCommissioningIndication_EventStatus_BrCommissionerDataRelayedOutbound = 0x001B,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerDataRelayedInbound = 0x001C,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerDataRelayedOutbound = 0x001D,
	THR_EventNwkCommissioningIndication_EventStatus_JoinerrouterJoinerAccepted = 0x001E,
	THR_EventNwkCommissioningIndication_EventStatus_StartVendorProvisioning = 0x001F
} THR_EventNwkCommissioningIndication_EventStatus_t;

typedef PACKED_STRUCT THR_EventNwkCommissioningIndication_tag {
	uint8_t InstanceId;  // Thread Instance Id
	THR_EventNwkCommissioningIndication_EventStatus_t EventStatus;  // Event status
} THR_EventNwkCommissioningIndication_t;

/* Direction */
typedef enum THR_CommissioningDiagnosticIndication_Direction_tag {
	THR_CommissioningDiagnosticIndication_Direction_OUT = 0x00,
	THR_CommissioningDiagnosticIndication_Direction_IN = 0x01
} THR_CommissioningDiagnosticIndication_Direction_t;

/* Type */
typedef enum THR_CommissioningDiagnosticIndication_Type_tag {
	THR_CommissioningDiagnosticIndication_Type_JOIN_FIN_REQ = 0x00,
	THR_CommissioningDiagnosticIndication_Type_JOIN_FIN_RSP = 0x01,
	THR_CommissioningDiagnosticIndication_Type_JOIN_ENT_REQ = 0x02,
	THR_CommissioningDiagnosticIndication_Type_JOIN_ENT_RSP = 0x03,
	THR_CommissioningDiagnosticIndication_Type_DTLS_CLOSE_NOTIFY = 0x04
} THR_CommissioningDiagnosticIndication_Type_t;

typedef struct THR_CommissioningDiagnosticIndication_tag {
	THR_CommissioningDiagnosticIndication_Direction_t Direction;  // Direction
	THR_CommissioningDiagnosticIndication_Type_t Type;  // Type
	uint8_t EUI[8];  // EUI
	uint8_t TlvsLength;  // Total size of the following TLVs
	uint8_t *TlvsBytes;  // The actual TLVs
} THR_CommissioningDiagnosticIndication_t;

/* Status */
typedef enum THR_MgmtDiagnosticGetConfirm_Status_tag {
	THR_MgmtDiagnosticGetConfirm_Status_Success = 0x00,
	THR_MgmtDiagnosticGetConfirm_Status_InvalidParameter = 0x03,
	THR_MgmtDiagnosticGetConfirm_Status_NotPermitted = 0x04,
	THR_MgmtDiagnosticGetConfirm_Status_NoMemory = 0x06,
	THR_MgmtDiagnosticGetConfirm_Status_Error = 0xFF
} THR_MgmtDiagnosticGetConfirm_Status_t;

typedef PACKED_STRUCT THR_MgmtDiagnosticGetConfirm_tag {
	THR_MgmtDiagnosticGetConfirm_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
} THR_MgmtDiagnosticGetConfirm_t;

/* Status */
typedef enum THR_DiagTestGetConfirm_Status_tag {
	THR_DiagTestGetConfirm_Status_Success = 0x00,
	THR_DiagTestGetConfirm_Status_InvalidParameter = 0x03,
	THR_DiagTestGetConfirm_Status_NotPermitted = 0x04,
	THR_DiagTestGetConfirm_Status_Error = 0xFF
} THR_DiagTestGetConfirm_Status_t;

typedef PACKED_STRUCT THR_DiagTestGetConfirm_tag {
	THR_DiagTestGetConfirm_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
} THR_DiagTestGetConfirm_t;

/* Status */
typedef enum THR_MgmtDiagnosticResetConfirm_Status_tag {
	THR_MgmtDiagnosticResetConfirm_Status_Success = 0x00,
	THR_MgmtDiagnosticResetConfirm_Status_InvalidParameter = 0x03,
	THR_MgmtDiagnosticResetConfirm_Status_NotPermitted = 0x04,
	THR_MgmtDiagnosticResetConfirm_Status_Error = 0xFF
} THR_MgmtDiagnosticResetConfirm_Status_t;

typedef PACKED_STRUCT THR_MgmtDiagnosticResetConfirm_tag {
	THR_MgmtDiagnosticResetConfirm_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
} THR_MgmtDiagnosticResetConfirm_t;

typedef PACKED_STRUCT THR_MgmtReadMemoryConfirm_tag {
	uint8_t Memory[1];  // Memory content
} THR_MgmtReadMemoryConfirm_t;

/* Status */
typedef enum THR_MgmtWriteMemoryConfirm_Status_tag {
	THR_MgmtWriteMemoryConfirm_Status_Success = 0x00
} THR_MgmtWriteMemoryConfirm_Status_t;

typedef PACKED_STRUCT THR_MgmtWriteMemoryConfirm_tag {
	THR_MgmtWriteMemoryConfirm_Status_t Status;  // Status
} THR_MgmtWriteMemoryConfirm_t;

/* Status */
typedef enum THR_SetManualSlaacIIDConfirm_Status_tag {
	THR_SetManualSlaacIIDConfirm_Status_OK = 0x00
} THR_SetManualSlaacIIDConfirm_Status_t;

typedef PACKED_STRUCT THR_SetManualSlaacIIDConfirm_tag {
	THR_SetManualSlaacIIDConfirm_Status_t Status;  // Status
} THR_SetManualSlaacIIDConfirm_t;

/* Status */
typedef enum THR_SendProactiveAddrNotifConfirm_Status_tag {
	THR_SendProactiveAddrNotifConfirm_Status_Success = 0x00,
	THR_SendProactiveAddrNotifConfirm_Status_Failed = 0x01,
	THR_SendProactiveAddrNotifConfirm_Status_InvalidInstance = 0x02,
	THR_SendProactiveAddrNotifConfirm_Status_NotStarted = 0x05
} THR_SendProactiveAddrNotifConfirm_Status_t;

typedef PACKED_STRUCT THR_SendProactiveAddrNotifConfirm_tag {
	THR_SendProactiveAddrNotifConfirm_Status_t Status;  // Status
} THR_SendProactiveAddrNotifConfirm_t;

/* Status */
typedef enum THR_NwkDiscoveryConfirm_Status_tag {
	THR_NwkDiscoveryConfirm_Status_OK = 0x00,
	THR_NwkDiscoveryConfirm_Status_Invalidinstance = 0x02,
	THR_NwkDiscoveryConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_NwkDiscoveryConfirm_Status_t;

typedef PACKED_STRUCT THR_NwkDiscoveryConfirm_tag {
	THR_NwkDiscoveryConfirm_Status_t Status;  // Status
} THR_NwkDiscoveryConfirm_t;

/* Status */
typedef enum THR_NwkDiscoveryStopConfirm_Status_tag {
	THR_NwkDiscoveryStopConfirm_Status_OK = 0x00,
	THR_NwkDiscoveryStopConfirm_Status_Invalidinstance = 0x02,
	THR_NwkDiscoveryStopConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_NwkDiscoveryStopConfirm_Status_t;

typedef PACKED_STRUCT THR_NwkDiscoveryStopConfirm_tag {
	THR_NwkDiscoveryStopConfirm_Status_t Status;  // Status
} THR_NwkDiscoveryStopConfirm_t;

/* Status */
typedef enum THR_SearchNwkWithAnounceConfirm_Status_tag {
	THR_SearchNwkWithAnounceConfirm_Status_OK = 0x00,
	THR_SearchNwkWithAnounceConfirm_Status_Invalidinstance = 0x02,
	THR_SearchNwkWithAnounceConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_SearchNwkWithAnounceConfirm_Status_t;

typedef PACKED_STRUCT THR_SearchNwkWithAnounceConfirm_tag {
	THR_SearchNwkWithAnounceConfirm_Status_t Status;  // Status
} THR_SearchNwkWithAnounceConfirm_t;

/* Status */
typedef enum THR_MgmtDiagnosticGetRspIndication_Status_tag {
	THR_MgmtDiagnosticGetRspIndication_Status_Success = 0x00,
	THR_MgmtDiagnosticGetRspIndication_Status_FailedNotsupported = 0x01
} THR_MgmtDiagnosticGetRspIndication_Status_t;

/* TLV identifier */
typedef enum THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_tag {
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_SourceAddr = 0x00,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_ShortAddr = 0x01,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Mode = 0x02,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Timeout = 0x03,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_LinkQuality = 0x04,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_RoutingTable = 0x05,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_LeaderData = 0x06,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_NwkData = 0x07,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Ip6AddrList = 0x08,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_MacCounters = 0x09,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_BatteryLevel = 0x0E,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_SupplyVoltage = 0x0F,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_ChildTable = 0x10,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_ChannelPages = 0x11,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_Mac6lowPanNvmDataCount = 0xA0,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_NetworkNvmDataCount_c = 0xA1,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_SecurityNvmDataCount_c = 0xA2,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_FunctionalNvmDataCount_c = 0xA3,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_BoardName_c = 0xA4,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_UniqueMcuId_c = 0xA5,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_StackVersion_c = 0xA6,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_Fsl_SoftwareVersion_c = 0xA7
} THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_t;

/* NvmDataSetId */
typedef enum THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_tag {
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_NvmId_SlwpStruct = 0x0000,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_NvmId_ContextTable = 0x0001,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_NvmId_macFilteringTable = 0x000B
} THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_t;

/* NvmDataSetId */
typedef enum THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_tag {
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_NvmId_MleSecInfo = 0x0012,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_NvmId_MleActiveKeyIndex = 0x0013,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_NvmId_MacOutSecFrameCounter = 0x0014
} THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_t;

/* NvmDataSetId */
typedef enum THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_tag {
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_NvmId_ThrAttr = 0x0010,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_NvmId_EventsTbl = 0x0011,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_NvmId_ThrStringAttr = 0x0018,
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_NvmId_BrPrefixSetAttr = 0x0019
} THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_t;

/* Stack Vendor OUI */
typedef enum THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_StackVersion_c_StackVendorOUI_tag {
	THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_StackVersion_c_StackVendorOUI_NXP = 0x006037
} THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_StackVersion_c_StackVendorOUI_t;

typedef struct THR_MgmtDiagnosticGetRspIndication_tag {
	THR_MgmtDiagnosticGetRspIndication_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
	uint16_t DataLen;  // Data length
	struct {
		THR_MgmtDiagnosticGetRspIndication_Payload_TlvId_t TlvId;  // TLV identifier
		uint8_t TlvLength;  // TLV Length
		union {
			uint64_t SourceAddr;  // Source Addr- EUI 64
			uint16_t ShortAddr;  // Short Address
			uint8_t Mode;  // Mode TLV
			uint16_t Timeout;  // Timeout - Sleepy polling rate
			struct {
				uint8_t MaxChildCount;  // MaxChildCount
				uint8_t ChildCount;  // ChildCount
				uint8_t cLinkQuality3;  // cLinkQuality3
				uint8_t cLinkQuality2;  // cLinkQuality2
				uint8_t cLinkQuality1;  // cLinkQuality1
				uint8_t LeaderCost;  // LeaderCost
				uint8_t IdSequence;  // IdSequence
			} LinkQuality;  // Link Quality
			uint8_t *RoutingTable;  // RoutingTable
			struct {
				uint32_t PartitionId;  // PartitionId
				uint8_t Weighting;  // Weighting
				uint8_t DataVersion;  // DataVersion
				uint8_t StableVersion;  // StableVersion
				uint8_t LeaderId;  // LeaderId
			} LeaderData;  // LeaderData
			uint8_t *NwkData;  // NwkData
			uint8_t *Ip6AddrList;  // Ip6AddrList
			struct {
				uint32_t ifInUnknownProtos;  // ifInUnknownProtos Counter
				uint32_t ifInErrors;  // ifInErrors Counter
				uint32_t ifOutErrors;  // ifOutErrors Counter
				uint32_t ifInUcastPkts;  // ifInUcastPkts Counter
				uint32_t ifInBroadcastPkts;  // ifInBroadcastPkts Counter
				uint32_t ifInDiscards;  // ifInDiscards Counter
				uint32_t ifOutUcastPkts;  // ifOutUcastPkts Counter
				uint32_t ifOutBroadcastPkts;  // ifOutBroadcastPkts Counter
				uint32_t ifOutDiscards;  // ifOutDiscards Counter
			} MacCounters;  // Mac Counters
			uint8_t BatteryLevel;  // Battery Level
			uint16_t SupplyVoltage;  // Supply Voltage
			uint8_t *ChildTable;  // ChildTable
			uint8_t ChannelPages;  // ChannelPages
			struct {
				THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_Mac6lowPanNvmDataCount_NvmDataSetId_t NvmDataSetId;  // NvmDataSetId
				uint16_t DataSetCount;  // DataSetCount
			} Fsl_Mac6lowPanNvmDataCount;  // MAC, 6lowpan (Mac Filtering, 6LowPan) NVM data save count
			uint8_t *Fsl_NetworkNvmDataCount_c;  // Network (IP, DHCP, Leader Id Assignment, Trickle, MPL, ND) NVM data save count
			struct {
				THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_SecurityNvmDataCount_c_NvmDataSetId_t NvmDataSetId;  // NvmDataSetId
				uint16_t DataSetCount;  // DataSetCount
			} Fsl_SecurityNvmDataCount_c;  // Security NVM data save count
			struct {
				THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_FunctionalNvmDataCount_c_NvmDataSetId_t NvmDataSetId;  // NvmDataSetId
				uint16_t DataSetCount;  // DataSetCount
			} Fsl_FunctionalNvmDataCount_c;  // Attributes, events NVM data save count
			char *Fsl_BoardName_c;  // Board name
			uint8_t Fsl_UniqueMcuId_c[16];  // Unique MCU identifier
			struct {
				THR_MgmtDiagnosticGetRspIndication_Payload_TlvData_Fsl_StackVersion_c_StackVendorOUI_t StackVendorOUI;  // Stack Vendor OUI
				uint8_t StackVersion[3];  // BuildNo|RevNo|MinorNo|MajorNo  // Stack Version
			} Fsl_StackVersion_c;  // Attribute, stack version
			char *Fsl_SoftwareVersion_c;  // Attribute, software version
		} TlvData;  // TLV data
	} *Payload;  // Payload
} THR_MgmtDiagnosticGetRspIndication_t;

/* Status */
typedef enum THR_DiagTestGetRspIndication_Status_tag {
	THR_DiagTestGetRspIndication_Status_Success = 0x00,
	THR_DiagTestGetRspIndication_Status_FailedNotsupported = 0x01
} THR_DiagTestGetRspIndication_Status_t;

/* TLV identifier */
typedef enum THR_DiagTestGetRspIndication_Payload_TlvId_tag {
	THR_DiagTestGetRspIndication_Payload_TlvId_ColdFactoryReset = 0xB0,
	THR_DiagTestGetRspIndication_Payload_TlvId_WarmCPUReset = 0xB1,
	THR_DiagTestGetRspIndication_Payload_TlvId_Data = 0xB2,
	THR_DiagTestGetRspIndication_Payload_TlvId_Results = 0xB3
} THR_DiagTestGetRspIndication_Payload_TlvId_t;

typedef struct THR_DiagTestGetRspIndication_tag {
	THR_DiagTestGetRspIndication_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
	uint16_t DataLen;  // Data length
	struct {
		THR_DiagTestGetRspIndication_Payload_TlvId_t TlvId;  // TLV identifier
		union {
			uint16_t ColdFactoryReset;  // Factory reset the node
			uint16_t WarmCPUReset;  // Warm/CPU Reset
			struct {
				uint8_t Flags;  // Flags
				uint32_t RspLatency;  // Response Latency
				uint32_t Offset;  // Calculated offset
				uint8_t SequenceNumber;  // Sequence Number
				uint8_t PayloadSize;  // Payload Size
				uint8_t *Payload;  // OTA transmited payload
			} Data;  // Large Network Data
			struct {
				uint32_t ReqLatency;  // Request Latency
				uint32_t RspLatency;  // Response Latency
				uint32_t Offset;  // Calculated offset
				uint8_t SequenceNumber;  // Sequence Number
			} Results;  // Large Network Results
		} TLVPayload;  // TLV Payload
	} Payload;  // Packet Payload
} THR_DiagTestGetRspIndication_t;

/* Status */
typedef enum THR_MgmtDiagnosticResetRspIndication_Status_tag {
	THR_MgmtDiagnosticResetRspIndication_Status_Success = 0x00,
	THR_MgmtDiagnosticResetRspIndication_Status_FailedNotallowed = 0x01
} THR_MgmtDiagnosticResetRspIndication_Status_t;

typedef PACKED_STRUCT THR_MgmtDiagnosticResetRspIndication_tag {
	THR_MgmtDiagnosticResetRspIndication_Status_t Status;  // Status
	uint16_t CoapMsgId;  // Coap over the air message id -  used for syncronization btw Req-Rsp
} THR_MgmtDiagnosticResetRspIndication_t;

/* Status */
typedef enum THR_SetNwkIdTimeoutConfirm_Status_tag {
	THR_SetNwkIdTimeoutConfirm_Status_Success = 0x00,
	THR_SetNwkIdTimeoutConfirm_Status_Notpermitted = 0x04
} THR_SetNwkIdTimeoutConfirm_Status_t;

typedef PACKED_STRUCT THR_SetNwkIdTimeoutConfirm_tag {
	THR_SetNwkIdTimeoutConfirm_Status_t Status;  // Status
} THR_SetNwkIdTimeoutConfirm_t;

/* Status */
typedef enum THR_SetThresholdConfirm_Status_tag {
	THR_SetThresholdConfirm_Status_Success = 0x00,
	THR_SetThresholdConfirm_Status_InvalidParameter = 0x03
} THR_SetThresholdConfirm_Status_t;

typedef PACKED_STRUCT THR_SetThresholdConfirm_tag {
	THR_SetThresholdConfirm_Status_t Status;  // Status
} THR_SetThresholdConfirm_t;

/* Status */
typedef enum THR_GetNeighborInfoConfirm_Status_tag {
	THR_GetNeighborInfoConfirm_Status_Success = 0x00,
	THR_GetNeighborInfoConfirm_Status_NeighborNotFound = 0x01
} THR_GetNeighborInfoConfirm_Status_t;

typedef struct THR_GetNeighborInfoConfirm_tag {
	THR_GetNeighborInfoConfirm_Status_t Status;  // Status
	union {
		struct {
			uint64_t ExtendedAddress;  // ExtendedAddress
			uint16_t ShortAddress;  // ShortAddress
			uint32_t LastCommTime;  // LastCommTime
			uint8_t InRSSI;  // InRSSI
			uint32_t Timeoutsec;  // Timeout in seconds
		} Success;  // Success
	} NeighborInfo;  // Neighbor Info
} THR_GetNeighborInfoConfirm_t;

typedef struct THR_GetChildrenTableConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t NoOfElements;  // No Of Elements
	struct {
		uint64_t ExtendedAddress;  // Extended Address
		uint16_t ShortAddress;  // Short Address
		uint32_t LastCommTime;  // Last Communication Time
		uint8_t LastRSSI;  // LastRSSI
		uint32_t Timeout;  // Timeout
	} *NeighborEntries;  // Neighbor Table Entry
} THR_GetChildrenTableConfirm_t;

typedef struct THR_GetNeighborTableConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t NoOfElements;  // No Of Elements
	struct {
		uint64_t ExtendedAddress;  // Extended Address
		uint16_t ShortAddress;  // Short Address
		uint32_t LastCommTime;  // Last Communication Time
		uint8_t LastRSSI;  // LastRSSI
	} *NeighborEntries;  // Neighbor Table Entry
} THR_GetNeighborTableConfirm_t;

typedef struct THR_GetRoutingTableConfirm_tag {
	uint8_t NoOfElements;  // No Of Elements
	uint8_t IdSequenceNb;  // Id Sequence Number
	uint64_t RouterIDMask;  // Router ID Mask
	struct {
		uint8_t RouterID;  // Router ID
		uint16_t ShortAddress;  // Short Address
		uint16_t NextHop;  // Next Hop
		uint8_t Cost;  // Route Cost
		uint8_t nIn;  // Incoming LQI
		uint8_t nOut;  // Outgoing LQI
	} *RoutingEntries;  // Routing Entry
} THR_GetRoutingTableConfirm_t;

typedef enum THR_GetAttrConfirm_AttributeId_tag {
	THR_GetAttrConfirm_AttributeId_RandomExtendedAddr = 0x00,
	THR_GetAttrConfirm_AttributeId_ShortAddress = 0x01,
	THR_GetAttrConfirm_AttributeId_ScanChannelMask = 0x02,
	THR_GetAttrConfirm_AttributeId_ScanDuration = 0x03,
	THR_GetAttrConfirm_AttributeId_Channel = 0x04,
	THR_GetAttrConfirm_AttributeId_ShortPanId = 0x05,
	THR_GetAttrConfirm_AttributeId_ExtendedPanId = 0x06,
	THR_GetAttrConfirm_AttributeId_PermitJoin = 0x07,
	THR_GetAttrConfirm_AttributeId_RxOnIdle = 0x08,
	THR_GetAttrConfirm_AttributeId_SedPollInterval = 0x09,
	THR_GetAttrConfirm_AttributeId_UniqueExtendedAddress = 0x0A,
	THR_GetAttrConfirm_AttributeId_VendorName = 0x0B,
	THR_GetAttrConfirm_AttributeId_ModelName = 0x0C,
	THR_GetAttrConfirm_AttributeId_SwVersion = 0x0D,
	THR_GetAttrConfirm_AttributeId_StackVersion = 0x0E,
	THR_GetAttrConfirm_AttributeId_NwkCapabilities = 0x0F,
	THR_GetAttrConfirm_AttributeId_NwkName = 0x10,
	THR_GetAttrConfirm_AttributeId_DeviceType = 0x11,
	THR_GetAttrConfirm_AttributeId_IsDevConnected = 0x12,
	THR_GetAttrConfirm_AttributeId_IsDevCommissioned = 0x13,
	THR_GetAttrConfirm_AttributeId_PartitionId = 0x14,
	THR_GetAttrConfirm_AttributeId_DeviceRole = 0x15,
	THR_GetAttrConfirm_AttributeId_Security_NwkMasterKey = 0x16,
	THR_GetAttrConfirm_AttributeId_Security_NwkKeySeq = 0x17,
	THR_GetAttrConfirm_AttributeId_Security_PSKc = 0x18,
	THR_GetAttrConfirm_AttributeId_Security_PSKd = 0x19,
	THR_GetAttrConfirm_AttributeId_VendorData = 0x1A,
	THR_GetAttrConfirm_AttributeId_MLPrefix = 0x1C,
	THR_GetAttrConfirm_AttributeId_MacFilteringEntry = 0x1D,
	THR_GetAttrConfirm_AttributeId_Security_KeyRotationInterval = 0x20,
	THR_GetAttrConfirm_AttributeId_ChildAddrMask = 0x21,
	THR_GetAttrConfirm_AttributeId_ChildSEDTimeout = 0x22,
	THR_GetAttrConfirm_AttributeId_ChildEDTimeout = 0x1B,
	THR_GetAttrConfirm_AttributeId_EndDevice_ChildEDReqFullNwkData = 0x23,
	THR_GetAttrConfirm_AttributeId_EndDevice_IsFastPollEnabled = 0x24,
	THR_GetAttrConfirm_AttributeId_SleepyEndDevice_FastPollInterval = 0x25,
	THR_GetAttrConfirm_AttributeId_JoinLqiThreshold = 0x26,
	THR_GetAttrConfirm_AttributeId_ProvisioningURL = 0x27,
	THR_GetAttrConfirm_AttributeId_SelectBestChannelEDThreshold = 0x28,
	THR_GetAttrConfirm_AttributeId_CommissionerMode = 0x29,
	THR_GetAttrConfirm_AttributeId_BorderRouter_BrPrefixEntry = 0x30,
	THR_GetAttrConfirm_AttributeId_SteeringData = 0x31,
	THR_GetAttrConfirm_AttributeId_Security_KeySwitchGuardTime = 0x33,
	THR_GetAttrConfirm_AttributeId_ParentHoldTime = 0x34,
	THR_GetAttrConfirm_AttributeId_Security_Policy = 0x35,
	THR_GetAttrConfirm_AttributeId_NVM_RestoreAutoStart = 0x36,
	THR_GetAttrConfirm_AttributeId_NVM_Restore = 0x37,
	THR_GetAttrConfirm_AttributeId_SlaacPolicy = 0x38,
	THR_GetAttrConfirm_AttributeId_IeeeExtendedAddr = 0x39,
	THR_GetAttrConfirm_AttributeId_LeaderWeight = 0x3A,
	THR_GetAttrConfirm_AttributeId_HashIeeeAddr = 0x40,
	THR_GetAttrConfirm_AttributeId_BorderRouter_BrGlobalOnMeshPrefix = 0x50,
	THR_GetAttrConfirm_AttributeId_BorderRouter_BrDefaultRouteOnMeshPrefix = 0x51,
	THR_GetAttrConfirm_AttributeId_BorderRouter_BrExternalIfPrefix = 0x52,
	THR_GetAttrConfirm_AttributeId_ActiveTimestamp = 0x60,
	THR_GetAttrConfirm_AttributeId_PendingChannel = 0x61,
	THR_GetAttrConfirm_AttributeId_PendingChannelMask = 0x62,
	THR_GetAttrConfirm_AttributeId_PendingXpanId = 0x63,
	THR_GetAttrConfirm_AttributeId_PendingMLprefix = 0x64,
	THR_GetAttrConfirm_AttributeId_PendingNwkMasterKey = 0x65,
	THR_GetAttrConfirm_AttributeId_PendingNwkName = 0x66,
	THR_GetAttrConfirm_AttributeId_PendingPanId = 0x67,
	THR_GetAttrConfirm_AttributeId_PendingPSK = 0x68,
	THR_GetAttrConfirm_AttributeId_PendingSecurityPolicy = 0x69,
	THR_GetAttrConfirm_AttributeId_PendingNwkKeyRotationInterval = 0x6A,
	THR_GetAttrConfirm_AttributeId_PendingDelayTimer = 0x6B,
	THR_GetAttrConfirm_AttributeId_PendingActiveTimestamp = 0x6C,
	THR_GetAttrConfirm_AttributeId_PendingTimestamp = 0x6D,
	THR_GetAttrConfirm_AttributeId_CommissionerId = 0x6E,
	THR_GetAttrConfirm_AttributeId_JoinerPort = 0x6F,
	THR_GetAttrConfirm_AttributeId_CommissionerUdpPort = 0x70,
	THR_GetAttrConfirm_AttributeId_DiscoveryReqMacTxOptions = 0x71
} THR_GetAttrConfirm_AttributeId_t;

/* Attribute Status */
typedef enum THR_GetAttrConfirm_Status_tag {
	THR_GetAttrConfirm_Status_Success = 0x00,
	THR_GetAttrConfirm_Status_Invalidinstance = 0x02,
	THR_GetAttrConfirm_Status_Invalidparameter = 0x03,
	THR_GetAttrConfirm_Status_Notpermitted = 0x04,
	THR_GetAttrConfirm_Status_UnsupportedAttribute = 0x07,
	THR_GetAttrConfirm_Status_EmptyEntry = 0x08
} THR_GetAttrConfirm_Status_t;

/* Stack Vendor OUI */
typedef enum THR_GetAttrConfirm_AttributeValue_StackVersion_StackVendorOUI_tag {
	THR_GetAttrConfirm_AttributeValue_StackVersion_StackVendorOUI_NXP = 0x006037
} THR_GetAttrConfirm_AttributeValue_StackVersion_StackVendorOUI_t;

/* Device Type */
typedef enum THR_GetAttrConfirm_AttributeValue_DeviceType_tag {
	THR_GetAttrConfirm_AttributeValue_DeviceType_EndNode = 0x00,
	THR_GetAttrConfirm_AttributeValue_DeviceType_Combo = 0x01
} THR_GetAttrConfirm_AttributeValue_DeviceType_t;

/* Device Role */
typedef enum THR_GetAttrConfirm_AttributeValue_DeviceRole_tag {
	THR_GetAttrConfirm_AttributeValue_DeviceRole_Disconnected = 0x00,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_SleepyEndDevice = 0x01,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_MinimalEndDevice = 0x02,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_FullEndDevice = 0x03,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_RouterEligibleEndDevice = 0x04,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_Router = 0x05,
	THR_GetAttrConfirm_AttributeValue_DeviceRole_Leader = 0x06
} THR_GetAttrConfirm_AttributeValue_DeviceRole_t;

/* The device is a thread native commissioner */
typedef enum THR_GetAttrConfirm_AttributeValue_CommissionerMode_tag {
	THR_GetAttrConfirm_AttributeValue_CommissionerMode_Disabled = 0x00,
	THR_GetAttrConfirm_AttributeValue_CommissionerMode_Collapsed = 0x01,
	THR_GetAttrConfirm_AttributeValue_CommissionerMode_Native = 0x02,
	THR_GetAttrConfirm_AttributeValue_CommissionerMode_Ethernet = 0x04,
	THR_GetAttrConfirm_AttributeValue_CommissionerMode_OnMesh = 0x08
} THR_GetAttrConfirm_AttributeValue_CommissionerMode_t;

/* P_preference */
typedef enum THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_tag {
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_MediumDefault = 0x00,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_High = 0x01,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_Reserved = 0x02,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_Low = 0x03
} THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_PrefixFlags_P_preference_t;

/* R_preference */
typedef enum THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_tag {
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_MediumDefault = 0x00,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_High = 0x01,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_Reserved = 0x02,
	THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_Low = 0x03
} THR_GetAttrConfirm_AttributeValue_BorderRouter_BrPrefixEntry_ExternalRouteFlags_R_preference_t;

/* Slaac Policy */
typedef enum THR_GetAttrConfirm_AttributeValue_SlaacPolicy_tag {
	THR_GetAttrConfirm_AttributeValue_SlaacPolicy_SlaacRandom = 0x00,
	THR_GetAttrConfirm_AttributeValue_SlaacPolicy_SlaacManual = 0x01,
	THR_GetAttrConfirm_AttributeValue_SlaacPolicy_SlaacMlIid = 0x02
} THR_GetAttrConfirm_AttributeValue_SlaacPolicy_t;

typedef struct THR_GetAttrConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	THR_GetAttrConfirm_AttributeId_t AttributeId;
	uint8_t Index;  // Index in table if required
	THR_GetAttrConfirm_Status_t Status;  // Attribute Status
	uint8_t AttrSize;  // Attribute size
	union {
		uint64_t RandomExtendedAddr;  // Random MAC address used for communication inside the Thread nwk
		uint16_t ShortAddress;  // Short Address
		uint32_t ScanChannelMask;  // Scan Channel mask
		uint32_t ScanDuration;  // Scan Duration
		uint8_t Channel;  // Channel
		uint16_t ShortPanId;  // Short PanId
		uint8_t *ExtendedPanId;  // Extended PanId
		bool_t PermitJoin;  // Permit Join
		bool_t RxOnIdle;  // RxOnidle
		uint32_t SedPollInterval;  // The polling interval for the sleepy end device (SED)[milliseconds]]
		bool_t UniqueExtendedAddress;  // Use unique extended address
		char *VendorName;  // Vendor Name
		char *ModelName;  // Model Name
		char *SwVersion;  // Software version
		struct {
			THR_GetAttrConfirm_AttributeValue_StackVersion_StackVendorOUI_t StackVendorOUI;  // Stack Vendor OUI
			uint8_t StackVersion[3];  // BuildNo|RevNo|MinorNo|MajorNo  // Stack Version
		} StackVersion;  // Stack Version Structure
		uint8_t NwkCapabilities;  // IsFullThreadDevice|IsPollingEndDevice|CanBecomeActiveRouter|CanCreateNewNetwork  // Network Capabilities
		char *NwkName;  // Network Name
		THR_GetAttrConfirm_AttributeValue_DeviceType_t DeviceType;  // Device Type
		bool_t IsDevConnected;  // Is Device Connected?
		bool_t IsDevCommissioned;  // Is Device Commissioned?
		uint32_t PartitionId;  // Partition Identifier
		THR_GetAttrConfirm_AttributeValue_DeviceRole_t DeviceRole;  // Device Role
		uint8_t *Security_NwkMasterKey;  // Network Master Key
		uint32_t Security_NwkKeySeq;  // Network key sequence
		uint8_t *Security_PSKc;  // PSKc
		char *Security_PSKd;  // PSKd
		char *VendorData;  // Vendor Data
		struct {
			uint8_t PrefixData[16];  // Prefix Data
			uint8_t PrefixLength;  // Prefix length in bits
		} MLPrefix;  // ML Prefix
		struct {
			uint64_t ExtendedAddress;  // Extended Address
			uint16_t ShortAddress;  // Short Address
			uint8_t LinkIndicator;  // The neighbor Quality Link Indicator: Good Link: 20 - 255 Medium Link: 11 - 20 Bad Link: 3 - 10
			bool_t BlockNeighbor;  // Add this neighbor to blacklist
		} MacFilteringEntry;  // Mac Filtering Entry
		uint32_t Security_KeyRotationInterval;  // key rotation interval
		uint8_t *ChildAddrMask;  // ChildAddrMask
		uint32_t ChildSEDTimeout;  // The timeout period included in the Child ID Request sent to the parent
		uint32_t ChildEDTimeout;  // The timeout period included in the Child ID Request sent to the parent
		bool_t EndDevice_ChildEDReqFullNwkData;  // If it is set TRUE The child End device should request the Full network data
		bool_t EndDevice_IsFastPollEnabled;  // IsFastPollEnabled
		uint32_t SleepyEndDevice_FastPollInterval;  // FastPollInterval
		uint8_t JoinLqiThreshold;  // JoinLqiThreshold
		char *ProvisioningURL;  // ProvisioningURL
		uint8_t SelectBestChannelEDThreshold;  // SelectBestChannelEDThreshold
		THR_GetAttrConfirm_AttributeValue_CommissionerMode_t CommissionerMode;  // The device is a thread native commissioner
		struct {
			uint8_t prefixLength;  // Prefix Length in bits
			uint8_t PrefixValue[16];  // Prefix value
			uint8_t PrefixFlagsReserved;  // PrefixFlagsReserved
			uint8_t PrefixFlags;  // P_preference|P_Slaac_preferred|P_Slaac_valid|P_Dhcp|P_Configure|P_Default|Reserved_bit0  // PrefixFlags
			uint32_t prefixLifetime;  // prefixLifetime
			bool_t prefixAdvertised;  // prefixAdvertised
			uint8_t ExternalRouteFlags;  // R_preference  // ExternalRouteFlags
			uint32_t ExternalRouteLifetime;  // ExternalRouteLifetime
			bool_t ExternalRouteAdvertised;  // ExternalRouteAdvertised
		} BorderRouter_BrPrefixEntry;  // Border router prefix attribute entry
		uint8_t *SteeringData;  // Steering Data
		uint32_t Security_KeySwitchGuardTime;  // The thread Key switch guard time to prevent inadvertent key switching
		uint16_t ParentHoldTime;  // Hold Time on parent in seconds
		uint8_t Security_Policy;  // SecurityPolicy, O and N bits without the rotation times
		bool_t NVM_RestoreAutoStart;  // Stack starts automatically with NVM restore after reset
		bool_t NVM_Restore;  // Restore from NVM
		THR_GetAttrConfirm_AttributeValue_SlaacPolicy_t SlaacPolicy;  // Slaac Policy
		uint64_t IeeeExtendedAddr;  // IEEE extended mac address
		uint8_t LeaderWeight;  // Leader Weight
		uint64_t HashIeeeAddr;  // IEEE extended mac address
		struct {
			uint8_t PrefixData[16];  // Prefix Data
			uint8_t PrefixLength;  // Prefix length in bits
		} BorderRouter_BrGlobalOnMeshPrefix;  // Global /64 on-Mesh Prefix on Border Router
		bool_t BorderRouter_BrDefaultRouteOnMeshPrefix;  // Default Route of the /64 on-mesh prefix
		struct {
			uint8_t PrefixData[16];  // Prefix Data
			uint8_t PrefixLength;  // Prefix length in bits
		} BorderRouter_BrExternalIfPrefix;  // Global /64 external interface prefix
		struct {
			uint8_t ActiveSeconds[6];  // Active seconds
			uint8_t ActiveTicks[2];  // Active ticks
		} ActiveTimestamp;  // Active timestamp
		char *CommissionerID;  // CommissionerID
		uint16_t JoinerPort;  // Joiner Port
		uint16_t CommissionerUdpPort;  // Commissioner UDP Port
	} AttributeValue;  // Attribute Value
} THR_GetAttrConfirm_t;

/* Attribute Status */
typedef enum THR_SetAttrConfirm_Status_tag {
	THR_SetAttrConfirm_Status_Success = 0x00,
	THR_SetAttrConfirm_Status_Invalidinstance = 0x02,
	THR_SetAttrConfirm_Status_Invalidparameter = 0x03,
	THR_SetAttrConfirm_Status_Notpermitted = 0x04,
	THR_SetAttrConfirm_Status_UnsupportedAttribute = 0x07,
	THR_SetAttrConfirm_Status_EmptyEntry = 0x08,
	THR_SetAttrConfirm_Status_InvalidValue = 0x09
} THR_SetAttrConfirm_Status_t;

typedef PACKED_STRUCT THR_SetAttrConfirm_tag {
	THR_SetAttrConfirm_Status_t Status;  // Attribute Status
} THR_SetAttrConfirm_t;

/* Status */
typedef enum THR_GetThreadIpAddrConfirm_Status_tag {
	THR_GetThreadIpAddrConfirm_Status_Success = 0x00,
	THR_GetThreadIpAddrConfirm_Status_Failed = 0x01,
	THR_GetThreadIpAddrConfirm_Status_InvalidInstance = 0x02,
	THR_GetThreadIpAddrConfirm_Status_Error = 0xFF
} THR_GetThreadIpAddrConfirm_Status_t;

typedef struct THR_GetThreadIpAddrConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	THR_GetThreadIpAddrConfirm_Status_t Status;  // Status
	uint8_t AddressType;  // AddressType
	uint8_t NoOfIpAddr;  // Number of Ip Addresses
	uint8_t *AddressList;  // AddressList
} THR_GetThreadIpAddrConfirm_t;

/* Status */
typedef enum THR_GetParentConfirm_Status_tag {
	THR_GetParentConfirm_Status_Success = 0x00,
	THR_GetParentConfirm_Status_Failed = 0x01,
	THR_GetParentConfirm_Status_InvalidInstance = 0x02,
	THR_GetParentConfirm_Status_Error = 0xFF
} THR_GetParentConfirm_Status_t;

typedef PACKED_STRUCT THR_GetParentConfirm_tag {
	THR_GetParentConfirm_Status_t Status;  // Status
	uint8_t InstanceId;  // Instance Id
	uint16_t ShortAddress;  // Short Address
	uint64_t ExtendedAddress;  // Extended address
} THR_GetParentConfirm_t;

/* Status */
typedef enum THR_ChildUpdateToParentConfirm_Status_tag {
	THR_ChildUpdateToParentConfirm_Status_OK = 0x00,
	THR_ChildUpdateToParentConfirm_Status_Invalidinstance = 0x02,
	THR_ChildUpdateToParentConfirm_Status_NotPermitted = 0x04
} THR_ChildUpdateToParentConfirm_Status_t;

typedef PACKED_STRUCT THR_ChildUpdateToParentConfirm_tag {
	THR_ChildUpdateToParentConfirm_Status_t Status;  // Status
} THR_ChildUpdateToParentConfirm_t;

/* Status */
typedef enum THR_LeaderRemoveRouterIdConfirm_Status_tag {
	THR_LeaderRemoveRouterIdConfirm_Status_OK = 0x00,
	THR_LeaderRemoveRouterIdConfirm_Status_Invalidinstance = 0x02,
	THR_LeaderRemoveRouterIdConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_LeaderRemoveRouterIdConfirm_Status_t;

typedef PACKED_STRUCT THR_LeaderRemoveRouterIdConfirm_tag {
	THR_LeaderRemoveRouterIdConfirm_Status_t Status;  // Status
} THR_LeaderRemoveRouterIdConfirm_t;

/* Status */
typedef enum THR_GenerateAllKeysConfirm_Status_tag {
	THR_GenerateAllKeysConfirm_Status_OK = 0x00,
	THR_GenerateAllKeysConfirm_Status_Invalidinstance = 0x02,
	THR_GenerateAllKeysConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_GenerateAllKeysConfirm_Status_t;

typedef PACKED_STRUCT THR_GenerateAllKeysConfirm_tag {
	THR_GenerateAllKeysConfirm_Status_t Status;  // Status
} THR_GenerateAllKeysConfirm_t;

/* Status */
typedef enum THR_SwitchKeyKeyConfirm_Status_tag {
	THR_SwitchKeyKeyConfirm_Status_OK = 0x00,
	THR_SwitchKeyKeyConfirm_Status_Invalidinstance = 0x02,
	THR_SwitchKeyKeyConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_SwitchKeyKeyConfirm_Status_t;

typedef PACKED_STRUCT THR_SwitchKeyKeyConfirm_tag {
	THR_SwitchKeyKeyConfirm_Status_t Status;  // Status
} THR_SwitchKeyKeyConfirm_t;

/* Status */
typedef enum THR_BrPrefixAddEntryConfirm_Status_tag {
	THR_BrPrefixAddEntryConfirm_Status_Success = 0x00,
	THR_BrPrefixAddEntryConfirm_Status_Invalidinstance = 0x02,
	THR_BrPrefixAddEntryConfirm_Status_Notpermitted = 0x04,
	THR_BrPrefixAddEntryConfirm_Status_Nomemory = 0x06,
	THR_BrPrefixAddEntryConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrPrefixAddEntryConfirm_Status_t;

typedef PACKED_STRUCT THR_BrPrefixAddEntryConfirm_tag {
	THR_BrPrefixAddEntryConfirm_Status_t Status;  // Status
} THR_BrPrefixAddEntryConfirm_t;

/* P_preference */
typedef enum THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_tag {
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_MediumDefault = 0x00,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_High = 0x01,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_Reserved = 0x02,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_Low = 0x03
} THR_BrPrefixGetTableConfirm_BrPrefixEntries_PrefixFlags_P_preference_t;

/* R_preference */
typedef enum THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_tag {
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_MediumDefault = 0x00,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_High = 0x01,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_Reserved = 0x02,
	THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_Low = 0x03
} THR_BrPrefixGetTableConfirm_BrPrefixEntries_ExternalRouteFlags_R_preference_t;

typedef struct THR_BrPrefixGetTableConfirm_tag {
	uint8_t InstanceId;  // Instance Id
	uint8_t NoOfElements;  // No Of Elements
	struct {
		uint8_t prefixLength;  // Prefix Length in bits
		uint8_t PrefixValue[16];  // Prefix value
		uint8_t PrefixFlagsReserved;  // Prefix Flags reserved
		uint8_t PrefixFlags;  // P_preference|P_Slaac_preferred|P_Slaac_valid|P_Dhcp|P_Configure|P_Default|Reserved_Bit0  // PrefixFlags
		uint32_t prefixLifetime;  // prefixLifetime
		bool_t prefixAdvertised;  // prefixAdvertised
		uint8_t ExternalRouteFlags;  // R_preference  // ExternalRouteFlags
		uint32_t ExternalRouteLifetime;  // ExternalRouteLifetime
		bool_t ExternalRouteAdvertised;  // ExternalRouteAdvertised
	} *BrPrefixEntries;  // Border router prefix entry
} THR_BrPrefixGetTableConfirm_t;

/* Status */
typedef enum THR_BrPrefixRemoveEntryConfirm_Status_tag {
	THR_BrPrefixRemoveEntryConfirm_Status_Success = 0x00,
	THR_BrPrefixRemoveEntryConfirm_Status_Invalidinstance = 0x02,
	THR_BrPrefixRemoveEntryConfirm_Status_InvalidParameter = 0x03,
	THR_BrPrefixRemoveEntryConfirm_Status_Nomemory = 0x06,
	THR_BrPrefixRemoveEntryConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrPrefixRemoveEntryConfirm_Status_t;

typedef PACKED_STRUCT THR_BrPrefixRemoveEntryConfirm_tag {
	THR_BrPrefixRemoveEntryConfirm_Status_t Status;  // Status
} THR_BrPrefixRemoveEntryConfirm_t;

/* Status */
typedef enum THR_BrServiceRemoveEntryConfirm_Status_tag {
	THR_BrServiceRemoveEntryConfirm_Status_Success = 0x00,
	THR_BrServiceRemoveEntryConfirm_Status_Invalidinstance = 0x02,
	THR_BrServiceRemoveEntryConfirm_Status_InvalidParameter = 0x03,
	THR_BrServiceRemoveEntryConfirm_Status_Nomemory = 0x06,
	THR_BrServiceRemoveEntryConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrServiceRemoveEntryConfirm_Status_t;

typedef PACKED_STRUCT THR_BrServiceRemoveEntryConfirm_tag {
	THR_BrServiceRemoveEntryConfirm_Status_t Status;  // Status
} THR_BrServiceRemoveEntryConfirm_t;

/* Status */
typedef enum THR_BrServiceAddEntryConfirm_Status_tag {
	THR_BrServiceAddEntryConfirm_Status_Success = 0x00,
	THR_BrServiceAddEntryConfirm_Status_Invalidinstance = 0x02,
	THR_BrServiceAddEntryConfirm_Status_InvalidParameter = 0x03,
	THR_BrServiceAddEntryConfirm_Status_NotPermitted = 0x04,
	THR_BrServiceAddEntryConfirm_Status_Nomemory = 0x06,
	THR_BrServiceAddEntryConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrServiceAddEntryConfirm_Status_t;

typedef PACKED_STRUCT THR_BrServiceAddEntryConfirm_tag {
	THR_BrServiceAddEntryConfirm_Status_t Status;  // Status
} THR_BrServiceAddEntryConfirm_t;

/* Status */
typedef enum THR_BrPrefixSyncConfirm_Status_tag {
	THR_BrPrefixSyncConfirm_Status_Success = 0x00,
	THR_BrPrefixSyncConfirm_Status_Invalidinstance = 0x02,
	THR_BrPrefixSyncConfirm_Status_InvalidParameter = 0x03,
	THR_BrPrefixSyncConfirm_Status_Notpermitted = 0x04,
	THR_BrPrefixSyncConfirm_Status_Nomemory = 0x06,
	THR_BrPrefixSyncConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrPrefixSyncConfirm_Status_t;

typedef PACKED_STRUCT THR_BrPrefixSyncConfirm_tag {
	THR_BrPrefixSyncConfirm_Status_t Status;  // Status
} THR_BrPrefixSyncConfirm_t;

/* Status */
typedef enum THR_BrPrefixRemoveAllConfirm_Status_tag {
	THR_BrPrefixRemoveAllConfirm_Status_Success = 0x00,
	THR_BrPrefixRemoveAllConfirm_Status_Invalidinstance = 0x02,
	THR_BrPrefixRemoveAllConfirm_Status_InvalidParameter = 0x03,
	THR_BrPrefixRemoveAllConfirm_Status_Notpermitted = 0x04,
	THR_BrPrefixRemoveAllConfirm_Status_Nomemory = 0x06,
	THR_BrPrefixRemoveAllConfirm_Status_Theselectedconfigurationisnotvalid = 0xFF
} THR_BrPrefixRemoveAllConfirm_Status_t;

typedef PACKED_STRUCT THR_BrPrefixRemoveAllConfirm_tag {
	THR_BrPrefixRemoveAllConfirm_Status_t Status;  // Status
} THR_BrPrefixRemoveAllConfirm_t;

/* Status */
typedef enum THR_IdentifyConfirm_Status_tag {
	THR_IdentifyConfirm_Status_Success = 0x00
} THR_IdentifyConfirm_Status_t;

typedef PACKED_STRUCT THR_IdentifyConfirm_tag {
	THR_IdentifyConfirm_Status_t Status;  // Status
} THR_IdentifyConfirm_t;

/* Status */
typedef enum NWKU_IfconfigBindConfirm_Status_tag {
	NWKU_IfconfigBindConfirm_Status_OK = 0x00,
	NWKU_IfconfigBindConfirm_Status_Addressesmaximumlimitreached = 0x01,
	NWKU_IfconfigBindConfirm_Status_ERROR = 0xFF
} NWKU_IfconfigBindConfirm_Status_t;

typedef PACKED_STRUCT NWKU_IfconfigBindConfirm_tag {
	NWKU_IfconfigBindConfirm_Status_t Status;  // Status
} NWKU_IfconfigBindConfirm_t;

typedef struct NWKU_IfconfigAllResponse_tag {
	uint8_t CountInterfaces;  // Number of interfaces
	struct {
		uint8_t InterfaceID;  // The interface ID.
		uint8_t CountIpAddresses;  // Number of IP addresses
		uint8_t *Addresses;  // Addresses
	} *InterfaceIDs;  // The interface ID.
} NWKU_IfconfigAllResponse_t;

/* Status */
typedef enum NWKU_PingConfirm_Status_tag {
	NWKU_PingConfirm_Status_OK = 0x00,
	NWKU_PingConfirm_Status_RequestTimeout = 0x02,
	NWKU_PingConfirm_Status_Wrongdestinationaddress = 0x03,
	NWKU_PingConfirm_Status_ERROR = 0xFF
} NWKU_PingConfirm_Status_t;

typedef PACKED_STRUCT NWKU_PingConfirm_tag {
	NWKU_PingConfirm_Status_t Status;  // Status
	uint16_t Interval;  // Interval
} NWKU_PingConfirm_t;

/* Status */
typedef enum NWKU_EchoUDPConfirm_Status_tag {
	NWKU_EchoUDPConfirm_Status_OK = 0x00,
	NWKU_EchoUDPConfirm_Status_RequestTimeout = 0x01,
	NWKU_EchoUDPConfirm_Status_Wrongdestinationaddress = 0x03,
	NWKU_EchoUDPConfirm_Status_ERROR = 0xFF
} NWKU_EchoUDPConfirm_Status_t;

typedef PACKED_STRUCT NWKU_EchoUDPConfirm_tag {
	NWKU_EchoUDPConfirm_Status_t Status;  // Status
} NWKU_EchoUDPConfirm_t;

/* Status */
typedef enum NWKU_CoapMsgReceivedIndication_Status_tag {
	NWKU_CoapMsgReceivedIndication_Status_Success = 0x00,
	NWKU_CoapMsgReceivedIndication_Status_Failed = 0x01,
	NWKU_CoapMsgReceivedIndication_Status_Duplicate = 0x02
} NWKU_CoapMsgReceivedIndication_Status_t;

/* CoAP request type */
typedef enum NWKU_CoapMsgReceivedIndication_RequestType_tag {
	NWKU_CoapMsgReceivedIndication_RequestType_CON = 0x00,
	NWKU_CoapMsgReceivedIndication_RequestType_NON = 0x01,
	NWKU_CoapMsgReceivedIndication_RequestType_ACK = 0x02
} NWKU_CoapMsgReceivedIndication_RequestType_t;

/* CoAP message type */
typedef enum NWKU_CoapMsgReceivedIndication_MessageType_tag {
	NWKU_CoapMsgReceivedIndication_MessageType_GET = 0x01,
	NWKU_CoapMsgReceivedIndication_MessageType_POST = 0x02,
	NWKU_CoapMsgReceivedIndication_MessageType_PUT = 0x03,
	NWKU_CoapMsgReceivedIndication_MessageType_DELETE = 0x04,
	NWKU_CoapMsgReceivedIndication_MessageType_Success = 0x05
} NWKU_CoapMsgReceivedIndication_MessageType_t;

typedef struct NWKU_CoapMsgReceivedIndication_tag {
	uint8_t coapSession[4];
	NWKU_CoapMsgReceivedIndication_Status_t Status;  // Status
	uint8_t RemoteIpAddress[16];  // The source address of the device that sent the message
	uint16_t UDPPort;  // UDPPort
	NWKU_CoapMsgReceivedIndication_RequestType_t RequestType;  // CoAP request type
	NWKU_CoapMsgReceivedIndication_MessageType_t MessageType;  // CoAP message type
	char URIpath[30];  // URI-path options separated by /
	uint8_t PayloadLength;  // Data length
	uint8_t *Payload;  // Data
} NWKU_CoapMsgReceivedIndication_t;

typedef PACKED_STRUCT NWKU_DnsResponseReceivedIndication_tag {
	uint8_t IpAddress[16];  // The IPv6 address of the domain name
	char DomainName[30];  // DomainName
} NWKU_DnsResponseReceivedIndication_t;

/* Status */
typedef enum NWKU_CoapRegisterConfirm_Status_tag {
	NWKU_CoapRegisterConfirm_Status_Success = 0x00,
	NWKU_CoapRegisterConfirm_Status_Failed = 0x01
} NWKU_CoapRegisterConfirm_Status_t;

typedef PACKED_STRUCT NWKU_CoapRegisterConfirm_tag {
	NWKU_CoapRegisterConfirm_Status_t Status;  // Status
} NWKU_CoapRegisterConfirm_t;

/* Status */
typedef enum NWKU_CoapCreateInstanceConfirm_Status_tag {
	NWKU_CoapCreateInstanceConfirm_Status_Success = 0x00,
	NWKU_CoapCreateInstanceConfirm_Status_Failed = 0x01
} NWKU_CoapCreateInstanceConfirm_Status_t;

typedef PACKED_STRUCT NWKU_CoapCreateInstanceConfirm_tag {
	NWKU_CoapCreateInstanceConfirm_Status_t Status;  // Status
} NWKU_CoapCreateInstanceConfirm_t;

/* Status */
typedef enum NWKU_CoapSendConfirm_Status_tag {
	NWKU_CoapSendConfirm_Status_Success = 0x00,
	NWKU_CoapSendConfirm_Status_Failed = 0x01
} NWKU_CoapSendConfirm_Status_t;

typedef PACKED_STRUCT NWKU_CoapSendConfirm_tag {
	NWKU_CoapSendConfirm_Status_t Status;  // Status
} NWKU_CoapSendConfirm_t;

/* Status */
typedef enum NWKU_DnsSendRequestConfirm_Status_tag {
	NWKU_DnsSendRequestConfirm_Status_Success = 0x00,
	NWKU_DnsSendRequestConfirm_Status_Failed = 0x01
} NWKU_DnsSendRequestConfirm_Status_t;

typedef PACKED_STRUCT NWKU_DnsSendRequestConfirm_tag {
	NWKU_DnsSendRequestConfirm_Status_t Status;  // Status
} NWKU_DnsSendRequestConfirm_t;

/* Status */
typedef enum NWKU_McastGroupManageConfirm_Status_tag {
	NWKU_McastGroupManageConfirm_Status_Success = 0x00,
	NWKU_McastGroupManageConfirm_Status_Failed = 0x01
} NWKU_McastGroupManageConfirm_Status_t;

typedef PACKED_STRUCT NWKU_McastGroupManageConfirm_tag {
	NWKU_McastGroupManageConfirm_Status_t Status;  // Status
} NWKU_McastGroupManageConfirm_t;

typedef struct NWKU_McastGroupShowResponse_tag {
	uint8_t CountIpAddresses;  // Number of IP addresses
	uint8_t *Addresses;  // Addresses
} NWKU_McastGroupShowResponse_t;

/* Status */
typedef enum MESHCOP_StartCommissionerConfirm_Status_tag {
	MESHCOP_StartCommissionerConfirm_Status_Success = 0x00,
	MESHCOP_StartCommissionerConfirm_Status_Failed = 0x01,
	MESHCOP_StartCommissionerConfirm_Status_Error = 0xFF
} MESHCOP_StartCommissionerConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_StartCommissionerConfirm_tag {
	MESHCOP_StartCommissionerConfirm_Status_t Status;  // Status
} MESHCOP_StartCommissionerConfirm_t;

/* Status */
typedef enum MESHCOP_StartNativeCommissionerConfirm_Status_tag {
	MESHCOP_StartNativeCommissionerConfirm_Status_Success = 0x00,
	MESHCOP_StartNativeCommissionerConfirm_Status_Failed = 0x01,
	MESHCOP_StartNativeCommissionerConfirm_Status_Error = 0xFF
} MESHCOP_StartNativeCommissionerConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_StartNativeCommissionerConfirm_tag {
	MESHCOP_StartNativeCommissionerConfirm_Status_t Status;  // Status
} MESHCOP_StartNativeCommissionerConfirm_t;

/* Status */
typedef enum MESHCOP_StopCommissionerConfirm_Status_tag {
	MESHCOP_StopCommissionerConfirm_Status_Success = 0x00,
	MESHCOP_StopCommissionerConfirm_Status_Failed = 0x01,
	MESHCOP_StopCommissionerConfirm_Status_Error = 0xFF
} MESHCOP_StopCommissionerConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_StopCommissionerConfirm_tag {
	MESHCOP_StopCommissionerConfirm_Status_t Status;  // Status
} MESHCOP_StopCommissionerConfirm_t;

/* Status */
typedef enum MESHCOP_AddExpectedJoinerConfirm_Status_tag {
	MESHCOP_AddExpectedJoinerConfirm_Status_Success = 0x00,
	MESHCOP_AddExpectedJoinerConfirm_Status_Failed = 0x01,
	MESHCOP_AddExpectedJoinerConfirm_Status_Error = 0xFF
} MESHCOP_AddExpectedJoinerConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_AddExpectedJoinerConfirm_tag {
	MESHCOP_AddExpectedJoinerConfirm_Status_t Status;  // Status
} MESHCOP_AddExpectedJoinerConfirm_t;

/* Status */
typedef enum MESHCOP_GetExpectedJoinerConfirm_Status_tag {
	MESHCOP_GetExpectedJoinerConfirm_Status_Success = 0x00,
	MESHCOP_GetExpectedJoinerConfirm_Status_Failed = 0x01,
	MESHCOP_GetExpectedJoinerConfirm_Status_Error = 0xFF
} MESHCOP_GetExpectedJoinerConfirm_Status_t;

/* Selected */
typedef enum MESHCOP_GetExpectedJoinerConfirm_Selected_tag {
	MESHCOP_GetExpectedJoinerConfirm_Selected_FALSE = 0x00,
	MESHCOP_GetExpectedJoinerConfirm_Selected_TRUE = 0x01
} MESHCOP_GetExpectedJoinerConfirm_Selected_t;

typedef struct MESHCOP_GetExpectedJoinerConfirm_tag {
	MESHCOP_GetExpectedJoinerConfirm_Status_t Status;  // Status
	MESHCOP_GetExpectedJoinerConfirm_Selected_t Selected;  // Selected
	uint8_t PSKdSize;  // Length of PSKd
	char *PSKd;  // PSKd
} MESHCOP_GetExpectedJoinerConfirm_t;

/* Status */
typedef enum MESHCOP_RemoveExpectedJoinerConfirm_Status_tag {
	MESHCOP_RemoveExpectedJoinerConfirm_Status_Success = 0x00,
	MESHCOP_RemoveExpectedJoinerConfirm_Status_Failed = 0x01,
	MESHCOP_RemoveExpectedJoinerConfirm_Status_Error = 0xFF
} MESHCOP_RemoveExpectedJoinerConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_RemoveExpectedJoinerConfirm_tag {
	MESHCOP_RemoveExpectedJoinerConfirm_Status_t Status;  // Status
} MESHCOP_RemoveExpectedJoinerConfirm_t;

/* Status */
typedef enum MESHCOP_RemoveAllExpectedJoinersConfirm_Status_tag {
	MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Success = 0x00,
	MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Failed = 0x01,
	MESHCOP_RemoveAllExpectedJoinersConfirm_Status_Error = 0xFF
} MESHCOP_RemoveAllExpectedJoinersConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_RemoveAllExpectedJoinersConfirm_tag {
	MESHCOP_RemoveAllExpectedJoinersConfirm_Status_t Status;  // Status
} MESHCOP_RemoveAllExpectedJoinersConfirm_t;

/* Status */
typedef enum MESHCOP_SyncSteeringDataConfirm_Status_tag {
	MESHCOP_SyncSteeringDataConfirm_Status_Success = 0x00,
	MESHCOP_SyncSteeringDataConfirm_Status_Failed = 0x01,
	MESHCOP_SyncSteeringDataConfirm_Status_Error = 0xFF
} MESHCOP_SyncSteeringDataConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_SyncSteeringDataConfirm_tag {
	MESHCOP_SyncSteeringDataConfirm_Status_t Status;  // Status
} MESHCOP_SyncSteeringDataConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtSetConfirm_Status_tag {
	MESHCOP_MgmtSetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtSetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtSetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtSetConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtSetConfirm_tag {
	MESHCOP_MgmtSetConfirm_Status_t Status;  // Status
} MESHCOP_MgmtSetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtGetConfirm_Status_tag {
	MESHCOP_MgmtGetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtGetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtGetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtGetConfirm_Status_t;

typedef enum MESHCOP_MgmtGetConfirm_Type_tag {
	MESHCOP_MgmtGetConfirm_Type_Channel = 0x00,
	MESHCOP_MgmtGetConfirm_Type_ChannelMask = 0x35,
	MESHCOP_MgmtGetConfirm_Type_PanId = 0x01,
	MESHCOP_MgmtGetConfirm_Type_XpanId = 0x02,
	MESHCOP_MgmtGetConfirm_Type_NetworkName = 0x03,
	MESHCOP_MgmtGetConfirm_Type_PSKc = 0x04,
	MESHCOP_MgmtGetConfirm_Type_MasterKey = 0x05,
	MESHCOP_MgmtGetConfirm_Type_KeySequence = 0x06,
	MESHCOP_MgmtGetConfirm_Type_MeshLocalUla = 0x07,
	MESHCOP_MgmtGetConfirm_Type_SteeringData = 0x08,
	MESHCOP_MgmtGetConfirm_Type_BorderRouterLocator = 0x09,
	MESHCOP_MgmtGetConfirm_Type_CommissionerID = 0x0A,
	MESHCOP_MgmtGetConfirm_Type_CommissionerSessionID = 0x0B,
	MESHCOP_MgmtGetConfirm_Type_SecurityPolicy = 0x0C,
	MESHCOP_MgmtGetConfirm_Type_DelayTimer = 0x34,
	MESHCOP_MgmtGetConfirm_Type_ActiveTimestamp = 0x0E,
	MESHCOP_MgmtGetConfirm_Type_PendingTimestamp = 0x33
} MESHCOP_MgmtGetConfirm_Type_t;

typedef struct MESHCOP_MgmtGetConfirm_tag {
	MESHCOP_MgmtGetConfirm_Status_t Status;  // Status
	MESHCOP_MgmtGetConfirm_Type_t Type;
	uint8_t Length;  // Length of the next field
	union {
		uint8_t Channel;  // Channel
		uint8_t *ChannelMask;  // ChannelMask
		uint8_t *PanId;  // PanId
		uint8_t *XpanId;  // XpanId
		char *NetworkName;  // NetworkName
		uint8_t *PSKc;  // PSKc
		uint8_t *MasterKey;  // MasterKey
		uint8_t *KeySequence;  // KeySequence
		uint8_t *MeshLocalUla;  // MeshLocalUla
		uint8_t *SteeringData;  // SteeringData
		uint8_t *BorderRouterLocator;  // BorderRouterLocator
		char *CommissionerID;  // CommissionerID
		uint8_t *CommissionerSessionID;  // CommissionerSessionID
		uint8_t *SecurityPolicy;  // SecurityPolicy
		uint8_t *DatasetTimestamp;  // DatasetTimestamp
	} Value;  // Value
} MESHCOP_MgmtGetConfirm_t;

/* Status */
typedef enum MESHCOP_SetCommissionerCredentialConfirm_Status_tag {
	MESHCOP_SetCommissionerCredentialConfirm_Status_Success = 0x00,
	MESHCOP_SetCommissionerCredentialConfirm_Status_Failed = 0x01,
	MESHCOP_SetCommissionerCredentialConfirm_Status_Error = 0xFF
} MESHCOP_SetCommissionerCredentialConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_SetCommissionerCredentialConfirm_tag {
	MESHCOP_SetCommissionerCredentialConfirm_Status_t Status;  // Status
} MESHCOP_SetCommissionerCredentialConfirm_t;

/* Status */
typedef enum MESHCOP_MgmNwkFormConfirm_Status_tag {
	MESHCOP_MgmNwkFormConfirm_Status_Success = 0x00,
	MESHCOP_MgmNwkFormConfirm_Status_Notpermitted = 0x04
} MESHCOP_MgmNwkFormConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmNwkFormConfirm_tag {
	MESHCOP_MgmNwkFormConfirm_Status_t Status;  // Status
} MESHCOP_MgmNwkFormConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtCommissionerGetConfirm_Status_tag {
	MESHCOP_MgmtCommissionerGetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtCommissionerGetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtCommissionerGetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtCommissionerGetConfirm_Status_t;

typedef struct MESHCOP_MgmtCommissionerGetConfirm_tag {
	MESHCOP_MgmtCommissionerGetConfirm_Status_t Status;  // Status
	uint32_t Length;  // Length of the next field
	uint8_t *TLVs;  // TLVs
} MESHCOP_MgmtCommissionerGetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtActiveGetConfirm_Status_tag {
	MESHCOP_MgmtActiveGetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtActiveGetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtActiveGetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtActiveGetConfirm_Status_t;

typedef struct MESHCOP_MgmtActiveGetConfirm_tag {
	MESHCOP_MgmtActiveGetConfirm_Status_t Status;  // Status
	uint32_t Length;  // Length of the next field
	uint8_t *TLVs;  // TLVs
} MESHCOP_MgmtActiveGetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtPendingGetConfirm_Status_tag {
	MESHCOP_MgmtPendingGetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtPendingGetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtPendingGetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtPendingGetConfirm_Status_t;

typedef struct MESHCOP_MgmtPendingGetConfirm_tag {
	MESHCOP_MgmtPendingGetConfirm_Status_t Status;  // Status
	uint32_t Length;  // Length of the next field
	uint8_t *TLVs;  // TLVs
} MESHCOP_MgmtPendingGetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtCommissionerSetConfirm_Status_tag {
	MESHCOP_MgmtCommissionerSetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtCommissionerSetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtCommissionerSetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtCommissionerSetConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtCommissionerSetConfirm_tag {
	MESHCOP_MgmtCommissionerSetConfirm_Status_t Status;  // Status
} MESHCOP_MgmtCommissionerSetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtActiveSetConfirm_Status_tag {
	MESHCOP_MgmtActiveSetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtActiveSetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtActiveSetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtActiveSetConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtActiveSetConfirm_tag {
	MESHCOP_MgmtActiveSetConfirm_Status_t Status;  // Status
} MESHCOP_MgmtActiveSetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtPendingSetConfirm_Status_tag {
	MESHCOP_MgmtPendingSetConfirm_Status_Success = 0x00,
	MESHCOP_MgmtPendingSetConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtPendingSetConfirm_Status_Error = 0xFF
} MESHCOP_MgmtPendingSetConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtPendingSetConfirm_tag {
	MESHCOP_MgmtPendingSetConfirm_Status_t Status;  // Status
} MESHCOP_MgmtPendingSetConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtSendPanIdQueryConfirm_Status_tag {
	MESHCOP_MgmtSendPanIdQueryConfirm_Status_Success = 0x00,
	MESHCOP_MgmtSendPanIdQueryConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtSendPanIdQueryConfirm_Status_Error = 0xFF
} MESHCOP_MgmtSendPanIdQueryConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendPanIdQueryConfirm_tag {
	MESHCOP_MgmtSendPanIdQueryConfirm_Status_t Status;  // Status
} MESHCOP_MgmtSendPanIdQueryConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtPanIdConflictConfirm_Status_tag {
	MESHCOP_MgmtPanIdConflictConfirm_Status_Success = 0x00,
	MESHCOP_MgmtPanIdConflictConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtPanIdConflictConfirm_Status_Error = 0xFF
} MESHCOP_MgmtPanIdConflictConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtPanIdConflictConfirm_tag {
	MESHCOP_MgmtPanIdConflictConfirm_Status_t Status;  // Status
	uint32_t ScanChannelMask;  // Channel mask
	uint16_t PanId;  // PanId
} MESHCOP_MgmtPanIdConflictConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtSendEdScanConfirm_Status_tag {
	MESHCOP_MgmtSendEdScanConfirm_Status_Success = 0x00,
	MESHCOP_MgmtSendEdScanConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtSendEdScanConfirm_Status_Error = 0xFF
} MESHCOP_MgmtSendEdScanConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendEdScanConfirm_tag {
	MESHCOP_MgmtSendEdScanConfirm_Status_t Status;  // Status
} MESHCOP_MgmtSendEdScanConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtEdReportConfirm_Status_tag {
	MESHCOP_MgmtEdReportConfirm_Status_Success = 0x00,
	MESHCOP_MgmtEdReportConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtEdReportConfirm_Status_Error = 0xFF
} MESHCOP_MgmtEdReportConfirm_Status_t;

typedef struct MESHCOP_MgmtEdReportConfirm_tag {
	MESHCOP_MgmtEdReportConfirm_Status_t Status;  // Status
	uint32_t ScanChannelMask;  // Channel mask
	uint8_t Length;  // Length
	uint8_t *EnergyList;  // EnergyList
} MESHCOP_MgmtEdReportConfirm_t;

/* Status */
typedef enum MESHCOP_MgmtSendAnnounceBeginConfirm_Status_tag {
	MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Success = 0x00,
	MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Failed = 0x01,
	MESHCOP_MgmtSendAnnounceBeginConfirm_Status_Error = 0xFF
} MESHCOP_MgmtSendAnnounceBeginConfirm_Status_t;

typedef PACKED_STRUCT MESHCOP_MgmtSendAnnounceBeginConfirm_tag {
	MESHCOP_MgmtSendAnnounceBeginConfirm_Status_t Status;  // Status
} MESHCOP_MgmtSendAnnounceBeginConfirm_t;

typedef PACKED_STRUCT DTLSOpenConfirm_tag {
	uint8_t ContextIndex;  // Context Index
} DTLSOpenConfirm_t;

/* Status */
typedef enum DTLSCloseConfirm_Status_tag {
	DTLSCloseConfirm_Status_OK = 0x00,
	DTLSCloseConfirm_Status_ERROR = 0xFF
} DTLSCloseConfirm_Status_t;

typedef PACKED_STRUCT DTLSCloseConfirm_tag {
	DTLSCloseConfirm_Status_t Status;  // Status
} DTLSCloseConfirm_t;

/* Status */
typedef enum DTLSClosePeerConfirm_Status_tag {
	DTLSClosePeerConfirm_Status_OK = 0x00,
	DTLSClosePeerConfirm_Status_ERROR = 0xFF
} DTLSClosePeerConfirm_Status_t;

typedef PACKED_STRUCT DTLSClosePeerConfirm_tag {
	DTLSClosePeerConfirm_Status_t Status;  // Status
} DTLSClosePeerConfirm_t;

/* Status */
typedef enum DTLSConnectConfirm_Status_tag {
	DTLSConnectConfirm_Status_OK = 0x00,
	DTLSConnectConfirm_Status_ERROR = 0xFF
} DTLSConnectConfirm_Status_t;

typedef PACKED_STRUCT DTLSConnectConfirm_tag {
	DTLSConnectConfirm_Status_t Status;  // Status
	uint8_t PeerIndex;  // Peer Index
} DTLSConnectConfirm_t;

/* Status */
typedef enum DTLSClientConnectedConfirm_Status_tag {
	DTLSClientConnectedConfirm_Status_OK = 0x00,
	DTLSClientConnectedConfirm_Status_ERROR = 0xFF
} DTLSClientConnectedConfirm_Status_t;

typedef PACKED_STRUCT DTLSClientConnectedConfirm_tag {
	DTLSClientConnectedConfirm_Status_t Status;  // Status
	uint8_t PeerIndex;  // Peer Index
} DTLSClientConnectedConfirm_t;

/* Status */
typedef enum DTLSSendConfirm_Status_tag {
	DTLSSendConfirm_Status_OK = 0x00,
	DTLSSendConfirm_Status_ERROR = 0xFF
} DTLSSendConfirm_Status_t;

typedef PACKED_STRUCT DTLSSendConfirm_tag {
	DTLSSendConfirm_Status_t Status;  // Status
} DTLSSendConfirm_t;

typedef struct DTLSReceiveConfirm_tag {
	uint8_t PeerIndex;  // Peer Index
	uint16_t Size;  // The number of payload bytes
	uint8_t *Data;  // Data
} DTLSReceiveConfirm_t;

typedef PACKED_STRUCT FSCIGetUniqueIdConfirm_tag {
	uint8_t UniqueId[16];  // Unique Id
} FSCIGetUniqueIdConfirm_t;

typedef PACKED_STRUCT FSCIGetMcuIdConfirm_tag {
	uint32_t McuId;  // Mcu Id
} FSCIGetMcuIdConfirm_t;

typedef struct FSCIGetSwVersionsConfirm_tag {
	uint8_t listSize;  // Length of the Payload
	uint8_t *SwVersions;  // List of SW Module versions
} FSCIGetSwVersionsConfirm_t;

/* The result of the Sniffer_MacSetPIBAttribute.Request */
typedef enum Sniffer_MacSetPIBAttributeConfirm_Status_tag {
	Sniffer_MacSetPIBAttributeConfirm_Status_gSuccess_c = 0x00,
	Sniffer_MacSetPIBAttributeConfirm_Status_UNSUPORTED_ATTRIBUTE = 0xF4,
	Sniffer_MacSetPIBAttributeConfirm_Status_gInvalidParameter_c = 0xE8,
	Sniffer_MacSetPIBAttributeConfirm_Status_gReadOnly_c = 0xFB
} Sniffer_MacSetPIBAttributeConfirm_Status_t;

/* The MAC PIB attribute identifier */
typedef enum Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_tag {
	Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_macLogicalChannel = 0x21,
	Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_macMAC_PromiscuousRxIndicationMode = 0x51,
	Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_macRxOnWhenIdle = 0x52
} Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_t;

typedef struct Sniffer_MacSetPIBAttributeConfirm_tag {
	Sniffer_MacSetPIBAttributeConfirm_Status_t Status;  // The result of the Sniffer_MacSetPIBAttribute.Request
	Sniffer_MacSetPIBAttributeConfirm_PIBAttribute_t PIBAttribute;  // The MAC PIB attribute identifier
	uint16_t DataLength;  // Length of the attribute data
	uint8_t *PIBAttributeValue;  // The MAC PIB attribute value
} Sniffer_MacSetPIBAttributeConfirm_t;

typedef struct MAC_PromiscuousRxIndication_tag {
	uint8_t LinkQuality;  // Link Quality
	uint32_t TimeStamp;  // Time Stamp
	uint8_t msduLength;  // Length of the frame
	uint8_t *msdu;  // The actual frame received
} MAC_PromiscuousRxIndication_t;

typedef PACKED_STRUCT SerialTun_IPPacketReceivedConfirm_tag {
	uint16_t Size;
	uint8_t *IPpayload;  // IP payload
} SerialTun_IPPacketReceivedConfirm_t;

/* The status of the Set Power Level request */
typedef enum AspSetPowerLevelConfirm_Status_tag {
	AspSetPowerLevelConfirm_Status_SUCCESS = 0x00,
	AspSetPowerLevelConfirm_Status_INVALID_PARAMETER = 0xE8
} AspSetPowerLevelConfirm_Status_t;

typedef PACKED_STRUCT AspSetPowerLevelConfirm_tag {
	AspSetPowerLevelConfirm_Status_t Status;  // The status of the Set Power Level request
} AspSetPowerLevelConfirm_t;

typedef PACKED_STRUCT AspGetPowerLevelConfirm_tag {
	uint8_t Value;  // The Value of Power Level
} AspGetPowerLevelConfirm_t;

typedef PACKED_STRUCT DBGConfirm_tag {
	char *text;  // text
} DBGConfirm_t;

typedef PACKED_STRUCT GenericEvent_tag {
    char text[128];  // text
} GenericEvent_t;

typedef struct thrEvtContainer_tag
{
	uint16_t id;
	union {
		SocketCreateConfirm_t SocketCreateConfirm;
		SocketShutdownConfirm_t SocketShutdownConfirm;
		SocketBindConfirm_t SocketBindConfirm;
		SocketSendConfirm_t SocketSendConfirm;
		SocketSendToConfirm_t SocketSendToConfirm;
		SocketReceiveConfirm_t SocketReceiveConfirm;
		SocketReceiveFromConfirm_t SocketReceiveFromConfirm;
		SocketConnectConfirm_t SocketConnectConfirm;
		SocketListenConfirm_t SocketListenConfirm;
		SocketAcceptConfirm_t SocketAcceptConfirm;
		SocketSetOptionConfirm_t SocketSetOptionConfirm;
		SocketGetOptionConfirm_t SocketGetOptionConfirm;
		MAC_MacFilteringAddEntryConfirm_t MAC_MacFilteringAddEntryConfirm;
		MAC_MacFilteringRemoveEntryConfirm_t MAC_MacFilteringRemoveEntryConfirm;
		MAC_MacFilteringEnableConfirm_t MAC_MacFilteringEnableConfirm;
		MAC_MacFilteringGetTableConfirm_t MAC_MacFilteringGetTableConfirm;
		THR_SetDeviceConfigConfirm_t THR_SetDeviceConfigConfirm;
		THR_NwkScanConfirm_t THR_NwkScanConfirm;
		THR_CreateNwkConfirm_t THR_CreateNwkConfirm;
		THR_JoinConfirm_t THR_JoinConfirm;
		THR_FactoryResetConfirm_t THR_FactoryResetConfirm;
		THR_CpuResetConfirm_t THR_CpuResetConfirm;
		THR_CpuResetIndication_t THR_CpuResetIndication;
		THR_DisconnectConfirm_t THR_DisconnectConfirm;
		THR_AttachConfirm_t THR_AttachConfirm;
		THR_PromoteAsRouterConfirm_t THR_PromoteAsRouterConfirm;
		THR_EventNwkScanConfirm_t THR_EventNwkScanConfirm;
		THR_EventNwkCreateConfirm_t THR_EventNwkCreateConfirm;
		THR_EventNwkJoinConfirm_t THR_EventNwkJoinConfirm;
		THR_EventNwkJoinSelectParentsConfirm_t THR_EventNwkJoinSelectParentsConfirm;
		THR_EventGeneralConfirm_t THR_EventGeneralConfirm;
		THR_EventNwkCommissioningIndication_t THR_EventNwkCommissioningIndication;
		THR_CommissioningDiagnosticIndication_t THR_CommissioningDiagnosticIndication;
		THR_MgmtDiagnosticGetConfirm_t THR_MgmtDiagnosticGetConfirm;
		THR_DiagTestGetConfirm_t THR_DiagTestGetConfirm;
		THR_MgmtDiagnosticResetConfirm_t THR_MgmtDiagnosticResetConfirm;
		THR_MgmtReadMemoryConfirm_t THR_MgmtReadMemoryConfirm;
		THR_MgmtWriteMemoryConfirm_t THR_MgmtWriteMemoryConfirm;
		THR_SetManualSlaacIIDConfirm_t THR_SetManualSlaacIIDConfirm;
		THR_SendProactiveAddrNotifConfirm_t THR_SendProactiveAddrNotifConfirm;
		THR_NwkDiscoveryConfirm_t THR_NwkDiscoveryConfirm;
		THR_NwkDiscoveryStopConfirm_t THR_NwkDiscoveryStopConfirm;
		THR_SearchNwkWithAnounceConfirm_t THR_SearchNwkWithAnounceConfirm;
		THR_MgmtDiagnosticGetRspIndication_t THR_MgmtDiagnosticGetRspIndication;
		THR_DiagTestGetRspIndication_t THR_DiagTestGetRspIndication;
		THR_MgmtDiagnosticResetRspIndication_t THR_MgmtDiagnosticResetRspIndication;
		THR_SetNwkIdTimeoutConfirm_t THR_SetNwkIdTimeoutConfirm;
		THR_SetThresholdConfirm_t THR_SetThresholdConfirm;
		THR_GetNeighborInfoConfirm_t THR_GetNeighborInfoConfirm;
		THR_GetChildrenTableConfirm_t THR_GetChildrenTableConfirm;
		THR_GetNeighborTableConfirm_t THR_GetNeighborTableConfirm;
		THR_GetRoutingTableConfirm_t THR_GetRoutingTableConfirm;
		THR_GetAttrConfirm_t THR_GetAttrConfirm;
		THR_SetAttrConfirm_t THR_SetAttrConfirm;
		THR_GetThreadIpAddrConfirm_t THR_GetThreadIpAddrConfirm;
		THR_GetParentConfirm_t THR_GetParentConfirm;
		THR_ChildUpdateToParentConfirm_t THR_ChildUpdateToParentConfirm;
		THR_LeaderRemoveRouterIdConfirm_t THR_LeaderRemoveRouterIdConfirm;
		THR_GenerateAllKeysConfirm_t THR_GenerateAllKeysConfirm;
		THR_SwitchKeyKeyConfirm_t THR_SwitchKeyKeyConfirm;
		THR_BrPrefixAddEntryConfirm_t THR_BrPrefixAddEntryConfirm;
		THR_BrPrefixGetTableConfirm_t THR_BrPrefixGetTableConfirm;
		THR_BrPrefixRemoveEntryConfirm_t THR_BrPrefixRemoveEntryConfirm;
		THR_BrServiceRemoveEntryConfirm_t THR_BrServiceRemoveEntryConfirm;
		THR_BrServiceAddEntryConfirm_t THR_BrServiceAddEntryConfirm;
		THR_BrPrefixSyncConfirm_t THR_BrPrefixSyncConfirm;
		THR_BrPrefixRemoveAllConfirm_t THR_BrPrefixRemoveAllConfirm;
		THR_IdentifyConfirm_t THR_IdentifyConfirm;
		NWKU_IfconfigBindConfirm_t NWKU_IfconfigBindConfirm;
		NWKU_IfconfigAllResponse_t NWKU_IfconfigAllResponse;
		NWKU_PingConfirm_t NWKU_PingConfirm;
		NWKU_EchoUDPConfirm_t NWKU_EchoUDPConfirm;
		NWKU_CoapMsgReceivedIndication_t NWKU_CoapMsgReceivedIndication;
		NWKU_DnsResponseReceivedIndication_t NWKU_DnsResponseReceivedIndication;
		NWKU_CoapRegisterConfirm_t NWKU_CoapRegisterConfirm;
		NWKU_CoapCreateInstanceConfirm_t NWKU_CoapCreateInstanceConfirm;
		NWKU_CoapSendConfirm_t NWKU_CoapSendConfirm;
		NWKU_DnsSendRequestConfirm_t NWKU_DnsSendRequestConfirm;
		NWKU_McastGroupManageConfirm_t NWKU_McastGroupManageConfirm;
		NWKU_McastGroupShowResponse_t NWKU_McastGroupShowResponse;
		MESHCOP_StartCommissionerConfirm_t MESHCOP_StartCommissionerConfirm;
		MESHCOP_StartNativeCommissionerConfirm_t MESHCOP_StartNativeCommissionerConfirm;
		MESHCOP_StopCommissionerConfirm_t MESHCOP_StopCommissionerConfirm;
		MESHCOP_AddExpectedJoinerConfirm_t MESHCOP_AddExpectedJoinerConfirm;
		MESHCOP_GetExpectedJoinerConfirm_t MESHCOP_GetExpectedJoinerConfirm;
		MESHCOP_RemoveExpectedJoinerConfirm_t MESHCOP_RemoveExpectedJoinerConfirm;
		MESHCOP_RemoveAllExpectedJoinersConfirm_t MESHCOP_RemoveAllExpectedJoinersConfirm;
		MESHCOP_SyncSteeringDataConfirm_t MESHCOP_SyncSteeringDataConfirm;
		MESHCOP_MgmtSetConfirm_t MESHCOP_MgmtSetConfirm;
		MESHCOP_MgmtGetConfirm_t MESHCOP_MgmtGetConfirm;
		MESHCOP_SetCommissionerCredentialConfirm_t MESHCOP_SetCommissionerCredentialConfirm;
		MESHCOP_MgmNwkFormConfirm_t MESHCOP_MgmNwkFormConfirm;
		MESHCOP_MgmtCommissionerGetConfirm_t MESHCOP_MgmtCommissionerGetConfirm;
		MESHCOP_MgmtActiveGetConfirm_t MESHCOP_MgmtActiveGetConfirm;
		MESHCOP_MgmtPendingGetConfirm_t MESHCOP_MgmtPendingGetConfirm;
		MESHCOP_MgmtCommissionerSetConfirm_t MESHCOP_MgmtCommissionerSetConfirm;
		MESHCOP_MgmtActiveSetConfirm_t MESHCOP_MgmtActiveSetConfirm;
		MESHCOP_MgmtPendingSetConfirm_t MESHCOP_MgmtPendingSetConfirm;
		MESHCOP_MgmtSendPanIdQueryConfirm_t MESHCOP_MgmtSendPanIdQueryConfirm;
		MESHCOP_MgmtPanIdConflictConfirm_t MESHCOP_MgmtPanIdConflictConfirm;
		MESHCOP_MgmtSendEdScanConfirm_t MESHCOP_MgmtSendEdScanConfirm;
		MESHCOP_MgmtEdReportConfirm_t MESHCOP_MgmtEdReportConfirm;
		MESHCOP_MgmtSendAnnounceBeginConfirm_t MESHCOP_MgmtSendAnnounceBeginConfirm;
		DTLSOpenConfirm_t DTLSOpenConfirm;
		DTLSCloseConfirm_t DTLSCloseConfirm;
		DTLSClosePeerConfirm_t DTLSClosePeerConfirm;
		DTLSConnectConfirm_t DTLSConnectConfirm;
		DTLSClientConnectedConfirm_t DTLSClientConnectedConfirm;
		DTLSSendConfirm_t DTLSSendConfirm;
		DTLSReceiveConfirm_t DTLSReceiveConfirm;
		FSCIGetUniqueIdConfirm_t FSCIGetUniqueIdConfirm;
		FSCIGetMcuIdConfirm_t FSCIGetMcuIdConfirm;
		FSCIGetSwVersionsConfirm_t FSCIGetSwVersionsConfirm;
		Sniffer_MacSetPIBAttributeConfirm_t Sniffer_MacSetPIBAttributeConfirm;
		MAC_PromiscuousRxIndication_t MAC_PromiscuousRxIndication;
		SerialTun_IPPacketReceivedConfirm_t SerialTun_IPPacketReceivedConfirm;
		AspSetPowerLevelConfirm_t AspSetPowerLevelConfirm;
		AspGetPowerLevelConfirm_t AspGetPowerLevelConfirm;
		DBGConfirm_t DBGConfirm;
		GenericEvent_t DebugEvent;
		GenericEvent_t OtaResetEvent;
	} Data;
} thrEvtContainer_t;

typedef memStatus_t (*pfnThrEvtHandler)(thrEvtContainer_t *container, uint8_t *pPayload);

typedef struct thrEvtHandler_tag
{
	uint16_t id;
	pfnThrEvtHandler handlerFunc;
} thrEvtHandler_t;


/*==================================================================================================
Public function prototypes
==================================================================================================*/
memStatus_t MAC_MacFilteringAddEntryRequest(MAC_MacFilteringAddEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t MAC_MacFilteringRemoveEntryRequest(MAC_MacFilteringRemoveEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t MAC_MacFilteringEnableRequest(MAC_MacFilteringEnableRequest_t *req, uint32_t fsciInterface);
memStatus_t MAC_MacFilteringGetTableRequest(MAC_MacFilteringGetTableRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SetDeviceConfigRequest(THR_SetDeviceConfigRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_NwkScanRequest(THR_NwkScanRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_CreateNwkRequest(THR_CreateNwkRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_JoinRequest(THR_JoinRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_FactoryResetRequest(uint32_t fsciInterface);
memStatus_t THR_CpuResetRequest(THR_CpuResetRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_DisconnectRequest(THR_DisconnectRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_AttachRequest(THR_AttachRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_PromoteAsRouterRequest(THR_PromoteAsRouterRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_MgmtDiagnosticGetRequest(THR_MgmtDiagnosticGetRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_DiagTestGetRequest(THR_DiagTestGetRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_MgmtDiagnosticResetRequest(THR_MgmtDiagnosticResetRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_MgmtReadMemoryRequest(THR_MgmtReadMemoryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_MgmtWriteMemoryRequest(THR_MgmtWriteMemoryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_NwkDiscoveryRequest(THR_NwkDiscoveryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_NwkDiscoveryStopRequest(THR_NwkDiscoveryStopRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SearchNwkWithAnounceRequest(THR_SearchNwkWithAnounceRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_ChildUpdateToParentRequest(THR_ChildUpdateToParentRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SetManualSlaacIIDRequest(THR_SetManualSlaacIIDRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SendProactiveAddrNotifRequest(THR_SendProactiveAddrNotifRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SetThresholdRequest(THR_SetThresholdRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SetNwkIdTimeoutRequest(THR_SetNwkIdTimeoutRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetRoutingTableRequest(THR_GetRoutingTableRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetNeighborTableRequest(THR_GetNeighborTableRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetNeighborInfoRequest(THR_GetNeighborInfoRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetChildrenTableRequest(THR_GetChildrenTableRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetAttrRequest(THR_GetAttrRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SetAttrRequest(THR_SetAttrRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_LeaderRemoveRouterIdRequest(THR_LeaderRemoveRouterIdRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GenerateAllKeysRequest(THR_GenerateAllKeysRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_SwitchKeyRequest(THR_SwitchKeyRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrPrefixAddEntryRequest(THR_BrPrefixAddEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrPrefixGetTableRequest(THR_BrPrefixGetTableRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrPrefixRemoveEntryRequest(THR_BrPrefixRemoveEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrServiceRemoveEntryRequest(THR_BrServiceRemoveEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrServiceAddEntryRequest(THR_BrServiceAddEntryRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrPrefixSyncRequest(THR_BrPrefixSyncRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_BrPrefixRemoveAllRequest(THR_BrPrefixRemoveAllRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetThreadIpAddrRequest(THR_GetThreadIpAddrRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_GetParentRequest(THR_GetParentRequest_t *req, uint32_t fsciInterface);
memStatus_t THR_IdentifyRequest(uint32_t fsciInterface);
memStatus_t SocketCreateRequest(SocketCreateRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketShutdownRequest(SocketShutdownRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketBindRequest(SocketBindRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketSendRequest(SocketSendRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketSendToRequest(SocketSendToRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketReceiveRequest(SocketReceiveRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketReceiveFromRequest(SocketReceiveFromRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketConnectRequest(SocketConnectRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketListenRequest(SocketListenRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketAcceptRequest(SocketAcceptRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketSetOptionRequest(SocketSetOptionRequest_t *req, uint32_t fsciInterface);
memStatus_t SocketGetOptionRequest(SocketGetOptionRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_StartCommissionerRequest(MESHCOP_StartCommissionerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_StartNativeCommissionerRequest(MESHCOP_StartNativeCommissionerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_StopCommissionerRequest(MESHCOP_StopCommissionerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_AddExpectedJoinerRequest(MESHCOP_AddExpectedJoinerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_GetExpectedJoinerRequest(MESHCOP_GetExpectedJoinerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_RemoveExpectedJoinerRequest(MESHCOP_RemoveExpectedJoinerRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_RemoveAllExpectedJoinersRequest(MESHCOP_RemoveAllExpectedJoinersRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_SyncSteeringDataRequest(MESHCOP_SyncSteeringDataRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtSetRequest(MESHCOP_MgmtSetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtGetRequest(MESHCOP_MgmtGetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_SetCommissionerCredentialRequest(MESHCOP_SetCommissionerCredentialRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmNwkFormRequest(MESHCOP_MgmNwkFormRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtCommissionerGetRequest(MESHCOP_MgmtCommissionerGetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtActiveGetRequest(MESHCOP_MgmtActiveGetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtPendingGetRequest(MESHCOP_MgmtPendingGetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtCommissionerSetRequest(MESHCOP_MgmtCommissionerSetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtActiveSetRequest(MESHCOP_MgmtActiveSetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtPendingSetRequest(MESHCOP_MgmtPendingSetRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtSendPanIdQueryRequest(MESHCOP_MgmtSendPanIdQueryRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtSendEdScanRequest(MESHCOP_MgmtSendEdScanRequest_t *req, uint32_t fsciInterface);
memStatus_t MESHCOP_MgmtSendAnnounceBeginRequest(MESHCOP_MgmtSendAnnounceBeginRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_IfconfigBindRequest(NWKU_IfconfigBindRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_IfconfigAllRequest(uint32_t fsciInterface);
memStatus_t NWKU_PingRequest(NWKU_PingRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_EchoUDPRequest(NWKU_EchoUDPRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_CPUResetRequest(uint32_t fsciInterface);
memStatus_t NWKU_CoapCreateInstanceRequest(NWKU_CoapCreateInstanceRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_CoapSendRequest(NWKU_CoapSendRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_CoapRegisterRequest(NWKU_CoapRegisterRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_DnsSendRequestRequest(NWKU_DnsSendRequestRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_McastGroupShowRequest(NWKU_McastGroupShowRequest_t *req, uint32_t fsciInterface);
memStatus_t NWKU_McastGroupManageRequest(NWKU_McastGroupManageRequest_t *req, uint32_t fsciInterface);
memStatus_t DTLSOpenRequest(DTLSOpenRequest_t *req, uint32_t fsciInterface);
memStatus_t DTLSCloseRequest(DTLSCloseRequest_t *req, uint32_t fsciInterface);
memStatus_t DTLSClosePeerRequest(DTLSClosePeerRequest_t *req, uint32_t fsciInterface);
memStatus_t DTLSConnectRequest(DTLSConnectRequest_t *req, uint32_t fsciInterface);
memStatus_t DTLSSendRequest(DTLSSendRequest_t *req, uint32_t fsciInterface);
memStatus_t FSCICPUResetRequest(uint32_t fsciInterface);
memStatus_t FSCIGetUniqueIdRequest(uint32_t fsciInterface);
memStatus_t FSCIGetMcuIdRequest(uint32_t fsciInterface);
memStatus_t FSCIGetSwVersionsRequest(uint32_t fsciInterface);
memStatus_t SerialTun_IPPacketSendRequest(SerialTun_IPPacketSendRequest_t *req, uint32_t fsciInterface);
memStatus_t Sniffer_MacSetPIBAttributeRequest(Sniffer_MacSetPIBAttributeRequest_t *req, uint32_t fsciInterface);
memStatus_t FSCIACK(FSCIACK_t *req, uint32_t fsciInterface);
memStatus_t FSCIEnableMSDBootloaderRequest(uint32_t fsciInterface);
memStatus_t AspSetPowerLevelRequest(AspSetPowerLevelRequest_t *req, uint32_t fsciInterface);
memStatus_t AspGetPowerLevelRequest(uint32_t fsciInterface);

void KHC_ThreadIP_RX_MsgHandler(void *pData, void *param, uint32_t fsciInterface);

#endif  /* _THREADIP_CMD_H */
