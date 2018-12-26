/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#include "Instance.h"
//#ifdef YOTTA_CFG_MBED_OS
//    #include "mbed-drivers/mbed.h"
//#else
//    #include "mbed.h"
//#endif
#include "BLE.h"
#include "mbedGap.h"
#include "Qn_Gap.h"
#include "Qn_Instance.h"

extern "C" {
#include "ble_config.h"
#include "gapc_task.h"
    void connection_start(struct gapc_connection_req_ind const *param) {QN_Gap::connectionCallback(param);}
    void connection_end(uint16_t conhdl, uint8_t reason) {QN_Gap::disconnectionCallback(conhdl, reason);}
#include "gap_func.h"
#include "app_gap.h"		
}

/*
void radioNotificationStaticCallback(bool param) {
    QN_BLE &gap = (nRF5xGap &) nRF5xn::Instance(BLE::DEFAULT_INSTANCE).getGap();
    gap.processRadioNotificationEvent(param);
}
*/
/**************************************************************************/
/*!
    @brief  Sets the advertising parameters and payload for the device

    @param[in]  params
                Basic advertising details, including the advertising
                delay, timeout and how the device should be advertised
    @params[in] advData
                The primary advertising data payload
    @params[in] scanResponse
                The optional Scan Response payload if the advertising
                type is set to \ref GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED
                in \ref GapAdveritinngParams

    @returns    \ref ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @retval     BLE_ERROR_BUFFER_OVERFLOW
                The proposed action would cause a buffer overflow.  All
                advertising payloads must be <= 31 bytes, for example.

    @retval     BLE_ERROR_NOT_IMPLEMENTED
                A feature was requested that is not yet supported in the
                QN firmware or hardware.

    @retval     BLE_ERROR_PARAM_OUT_OF_RANGE
                One of the proposed values is outside the valid range.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::setAdvertisingData(const GapAdvertisingData &advData, const GapAdvertisingData &scanResponse)
{
    /* Make sure we don't exceed the advertising payload length */
    if (advData.getPayloadLen() > GAP_ADVERTISING_DATA_MAX_PAYLOAD) {
        return BLE_ERROR_BUFFER_OVERFLOW;
    }

    /* Make sure we have a payload! */
    if (advData.getPayloadLen() == 0) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
	
    /* Check the scan response payload limits */
    //if ((params.getAdvertisingType() == GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED))
    //{
    //    /* Check if we're within the upper limit */
    //    if (advData.getPayloadLen() > GAP_ADVERTISING_DATA_MAX_PAYLOAD)
    //    {
    //        return BLE_ERROR_BUFFER_OVERFLOW;
    //    }
    //    /* Make sure we have a payload! */
    //    if (advData.getPayloadLen() == 0)
    //    {
    //        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    //    }
    //}
	//uint16_t appearance = advData.getAppearance();
    /* Send advertising data! */
		 APP_ConstructAdvData(advData.getPayload(),
                                   advData.getPayloadLen(),
                                   scanResponse.getPayload(),
                                   scanResponse.getPayloadLen());
  
    /* Make sure the GAP Service appearance value is aligned with the
     *appearance from GapAdvertisingData */

	 //attm_att_set_value(gapm_get_att_handle(GAP_IDX_ICON),2,0,(uint8_t *) &appearance);
    /* ToDo: Perform some checks on the payload, for example the Scan Response can't */
    /* contains a flags AD type, etc. */

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Starts the BLE HW, initialising any services that were
            added before this function was called.

    @note   All services must be added before calling this function!

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::startAdvertising(const GapAdvertisingParams &params)
{
    uint8_t code;
	/* Make sure we support the advertising type */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) {
        /* ToDo: This requires a propery security implementation, etc. */
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /* Check interval range */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED) {
        /* Min delay is slightly longer for unconnectable devices */
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN_NONCON) ||
            (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    } else {
		
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN) ||
            (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }

    /* Check timeout is zero for Connectable Directed */
    if ((params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) && (params.getTimeout() != 0)) {
        /* Timeout must be 0 with this type, although we'll never get here */
        /* since this isn't implemented yet anyway */
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Check timeout for other advertising types */
    if ((params.getAdvertisingType() != GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) &&
        (params.getTimeout() > GapAdvertisingParams::GAP_ADV_PARAMS_TIMEOUT_MAX)) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Allocate the stack's whitelist statically */
   /* ble_gap_whitelist_t  whitelist;
    ble_gap_addr_t      *whitelistAddressPtrs[YOTTA_CFG_WHITELIST_MAX_SIZE];
    ble_gap_irk_t       *whitelistIrkPtrs[YOTTA_CFG_IRK_TABLE_MAX_SIZE];*/
    /* Initialize the whitelist */
   /* whitelist.pp_addrs   = whitelistAddressPtrs;
    whitelist.pp_irks    = whitelistIrkPtrs;
    whitelist.addr_count = 0;
    whitelist.irk_count  = 0;*/

    /* Add missing IRKs to whitelist from the bond table held by the SoftDevice */
 /*   if (advertisingPolicyMode != Gap::ADV_POLICY_IGNORE_WHITELIST) {
        ble_error_t error = generateStackWhitelist(whitelist);
        if (error != BLE_ERROR_NONE) {
            return error;
        }
    }
*/
    /* Start Advertising */
  /*  ble_gap_adv_params_t adv_para = {0};

    adv_para.type        = params.getAdvertisingType();
    adv_para.p_peer_addr = NULL;                           // Undirected advertisement
    adv_para.fp          = advertisingPolicyMode;
    adv_para.p_whitelist = &whitelist;
    adv_para.interval    = params.getIntervalInADVUnits(); // advertising interval (in units of 0.625 ms)
    adv_para.timeout     = params.getTimeout(); */
    
	if(params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED)
    {
		code = GAPM_ADV_UNDIRECT;
    }
	else if(params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED)
    {
		code = GAPM_ADV_DIRECT;
    }
	else if(params.getAdvertisingType() == GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED)
    {
		code = GAPM_ADV_UNDIRECT; //tbd mahesh
    }
	else
    {
		code = GAPM_ADV_NON_CONN;
    }
	
	APP_GapmStartAdvertiseCmd(code, GAPM_STATIC_ADDR, GAP_GEN_DISCOVERABLE, params.getIntervalInADVUnits(), 
								 params.getIntervalInADVUnits(), _advPayload.getPayloadLen(), (uint8_t *)_advPayload.getPayload(), _scanResponse.getPayloadLen(), (uint8_t *)_scanResponse.getPayload(), NULL);
										 
/*	struct gapm_start_advertise_cmd *adv_cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD, TASK_GAPM, TASK_APP, gapm_start_advertise_cmd);
	adv_cmd->op.code = params.getAdvertisingType();
    avd_cmd->op.addr_src = GAPM_STATIC_ADDR;

    adv_cmd->intv_min = GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN;
    adv_cmd->intv_max = GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX;
    adv_cmd->channel_map = CFG_ADV_CHMAP;  //tbd
	
	if ((params.getAdvertisingType() == GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED) || 
	(params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED))
	{
		adv_cmd->info.host.mode = mode;
        adv_cmd->info.host.adv_filt_policy = advertisingPolicyMode;
        adv_cmd->info.host.adv_data_len = _advPayload.getPayloadLen();
        memcpy(&adv_cmd->info.host.adv_data[0], _advPayload.getPayload(), adv_data_len);
        adv_cmd->info.host.scan_rsp_data_len = _scanResponse.getPayloadLen();
        memcpy(&adv_cmd->info.host.scan_rsp_data[0], _scanResponse.getPayload(), _scanResponse.getPayloadLen());
        // To find out the local IRK from resolving list by a specified peer address
       // adv_cmd->info.host.peer_addr = *peer_addr;  //tbd Mahesh
	   adv_cmd->info.host.peer_addr = NULL;
	}
	else
	{
		// Direct advertising
       // ASSERT_ERR(peer_addr != NULL);  //tbd Mahesh
       // cmd->info.direct = *peer_addr; 
	}
	
	app_env.state = APP_OP_ADVERTISING;
	ke_msg_send(adv_cmd);*/
	
    /*uint32_t err = sd_ble_gap_adv_start(&adv_para);
    switch(err) {
        case ERROR_NONE:
            return BLE_ERROR_NONE;
        case NRF_ERROR_NO_MEM:
            return BLE_ERROR_NO_MEM;
        default:
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }*/
    return BLE_ERROR_NONE;
}

ble_error_t QN_Gap::startRadioScan(const GapScanningParams &scanningParams)
{
#if 0	
	uint8_t code;
	uint8_t filt_policy;
   /* Allocate the stack's whitelist statically */
/*    ble_gap_whitelist_t  whitelist;
    ble_gap_addr_t      *whitelistAddressPtrs[YOTTA_CFG_WHITELIST_MAX_SIZE];
    ble_gap_irk_t       *whitelistIrkPtrs[YOTTA_CFG_IRK_TABLE_MAX_SIZE];*/
    /* Initialize the whitelist */
/*    whitelist.pp_addrs   = whitelistAddressPtrs;
    whitelist.pp_irks    = whitelistIrkPtrs;
    whitelist.addr_count = 0;
    whitelist.irk_count  = 0;*/

    /* Add missing IRKs to whitelist from the bond table held by the SoftDevice */
   /* if (scanningPolicyMode != Gap::SCAN_POLICY_IGNORE_WHITELIST) {
        ble_error_t error = generateStackWhitelist(whitelist);
        if (error != BLE_ERROR_NONE) {
            return error;
        }
    }*/
    
	if(scanningParams.getActiveScanning() == true )
		code = GAPM_SCAN_ACTIVE;
	if (scanningPolicyMode != Gap::SCAN_POLICY_IGNORE_WHITELIST)
		filt_policy = SCAN_ALLOW_ADV_ALL;
	else
		filt_policy = SCAN_ALLOW_ADV_WLST;
#if (BLE_CENTRAL)	
	APP_GapmStartScanCmd(code,GAPM_STATIC_ADDR,GAP_GEN_DISCOVERY,scanningParams.getInterval(),scanningParams.getWindow(),filt_policy,SCAN_FILT_DUPLIC_EN);
#endif   
#endif	
	return BLE_ERROR_NONE;
}

  // Just doing for peripheral
/************************************************
 *  Stop scan function
 ***********************************************/
 
ble_error_t QN_Gap::stopScan(void) {
   APP_GapmStopScanning();
    return BLE_ERROR_NONE;
}


/**************************************************************************/
/*!
    @brief  Stops the BLE HW and disconnects from any devices

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::stopAdvertising(void)
{
    /* Stop Advertising */
	APP_GapmStopAdvertising();

    return BLE_ERROR_NONE;
}

/*********************************************************************
 *  
 *********************************************************************/

 ble_error_t QN_Gap::connect(const BLEProtocol::AddressBytes_t    peerAddr,
                              BLEProtocol::AddressType_t  peerAddrType,
                              const ConnectionParams_t   *connectionParams,
                              const GapScanningParams    *scanParamsIn)
{
  struct gap_bdaddr addr;
	addr.addr_type = peerAddrType;
	memcpy(addr.addr.addr, peerAddr, Gap::ADDR_LEN);
#if (BLE_CENTRAL)	
	APP_GapmStartConnectionCmd(GAPM_CONNECTION_DIRECT, GAPM_STATIC_ADDR, 
                                          connectionParams->minConnectionInterval, connectionParams->maxConnectionInterval,
                                          connectionParams->slaveLatency, connectionParams->connectionSupervisionTimeout, 
                                          1, &addr);
#endif										  
	 return BLE_ERROR_NONE;
        
}

/*********************************************************************
 *  
 *********************************************************************/

ble_error_t QN_Gap::disconnect(Handle_t connectionHandle, DisconnectionReason_t reason)
{
    uint8_t code = CO_ERROR_REMOTE_USER_TERM_CON;
    switch (reason) {
        case REMOTE_USER_TERMINATED_CONNECTION:
            code = CO_ERROR_REMOTE_USER_TERM_CON;
            break;
        case CONN_INTERVAL_UNACCEPTABLE:
            code = CO_ERROR_UNACCEPTABLE_CONN_INT;
            break;
				case REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES:
						code = CO_ERROR_REMOTE_DEV_TERM_LOW_RESOURCES;
            break;
				case REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
						code = CO_ERROR_REMOTE_DEV_POWER_OFF;
            break;
        default:
            break;
			}	
			/* Disconnect if we are connected to a central device */
    APP_GapcDisconnectCmd(connectionHandle,code); 

    return BLE_ERROR_NONE;
}

/*!
    @brief  Disconnects if we are connected to a central device

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
ble_error_t QN_Gap::disconnect(DisconnectionReason_t reason)
{
    return disconnect(m_connectionHandle, reason);
}

/*********************************************************************
 *  
 *********************************************************************/

ble_error_t QN_Gap::getPreferredConnectionParams(ConnectionParams_t *params) //FixmeMah
{
   // ASSERT_INT(NRF_SUCCESS, 
    //    sd_ble_gap_ppcp_get(reinterpret_cast<ble_gap_conn_params_t *>(params)),
    //    BLE_ERROR_PARAM_OUT_OF_RANGE);
    //app_gapc_get_peer_slv_pref_params(m_connectionHandle)
    return BLE_ERROR_NONE;
}

/*********************************************************************
 *  
 *********************************************************************/

ble_error_t QN_Gap::setPreferredConnectionParams(const ConnectionParams_t *params) //FixmeMah
{
    //ASSERT_INT(NRF_SUCCESS,
    //    sd_ble_gap_ppcp_set(reinterpret_cast<const ble_gap_conn_params_t *>(params)),
   //     BLE_ERROR_PARAM_OUT_OF_RANGE);

    return BLE_ERROR_NONE;
}

/*********************************************************************
 *  
 *********************************************************************/

ble_error_t QN_Gap::updateConnectionParams(Handle_t handle, const ConnectionParams_t *newParams)
{
    uint32_t rc;

    rc = updateBleConnectionParams(handle, reinterpret_cast<gapc_conn_param *>(const_cast<ConnectionParams_t*>(newParams)));
    if (rc == GAP_ERR_NO_ERROR) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
}

/**************************************************************************/
/*!
    @brief  Clear QN_Gap's state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t QN_Gap::reset(void)
{
    /* Clear all state that is from the parent, including private members */
    if (Gap::reset() != BLE_ERROR_NONE) {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear derived class members */
   // m_connectionHandle = BLE_CONN_HANDLE_INVALID;  //FixMeMah

    /* Set the whitelist policy filter modes to IGNORE_WHITELIST */
    advertisingPolicyMode = Gap::ADV_POLICY_IGNORE_WHITELIST;
    scanningPolicyMode    = Gap::SCAN_POLICY_IGNORE_WHITELIST;

    /* Clear the internal whitelist */
    whitelistAddressesSize = 0;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the 16-bit connection handle
*/
/**************************************************************************/
void QN_Gap::setConnectionHandle(uint16_t con_handle)
{
    m_connectionHandle = con_handle;
}

/**************************************************************************/
/*!
    @brief  Gets the 16-bit connection handle
*/
/**************************************************************************/
uint16_t QN_Gap::getConnectionHandle(void)
{
    return m_connectionHandle;
}

/**************************************************************************/
/*!
    @brief      Sets the BLE device address

    @returns    ble_error_t

    @section EXAMPLE

    @code

    uint8_t device_address[6] = { 0xca, 0xfe, 0xf0, 0xf0, 0xf0, 0xf0 };
    QN_BLE.getGap().setAddress(Gap::BLEProtocol::AddressType::RANDOM_STATIC, device_address);

    @endcode
*/

/**************************************************************************/
ble_error_t QN_Gap::setAddress(AddressType_t type, const Address_t address)
{
    uint8_t gap_address_type;
 
	struct gap_bdaddr dev_addr;
	 
   
    if (type == BLEProtocol::AddressType::PUBLIC)
    {
        
        memcpy(dev_addr.addr.addr, address, ADDR_LEN); //FixmeMah
    }
    else if ((type == BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE) || (type == BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE)
			||(type == BLEProtocol::AddressType::RANDOM_STATIC))
    {
        switch(type){
					case BLEProtocol::AddressType::RANDOM_STATIC:
						gap_address_type = GAP_STATIC_ADDR;
					break;
					case BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE:
						gap_address_type = GAP_RSLV_ADDR;
					break;
					case BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE:
						gap_address_type = GAP_NON_RSLV_ADDR;
					break;		
				}
				APP_GapmGenRandAddrCmd(gap_address_type);  //FixmeMah the address will show in indication handler
    }
    else
    {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
  
    return BLE_ERROR_NONE;
}

/*
*  Device Address in from the stack come in the handler so can not get it immediately
*/
ble_error_t QN_Gap::getAddress(AddressType_t *typeP, Address_t address)
{
    struct gap_bdaddr dev_addr;
	if (typeP != NULL) {
        *typeP = static_cast<AddressType_t>(dev_addr.addr_type);
    }
	if (typeP != NULL){
		APP_GapmGenRandAddrCmd(*typeP);
	}
	else if(getBleGapAddress(&dev_addr) != GAP_ERR_NO_ERROR){
				return BLE_ERROR_PARAM_OUT_OF_RANGE;
			}

    if (address != NULL) {
        memcpy(address, dev_addr.addr.addr, ADDR_LEN);
    }
    return BLE_ERROR_NONE;
}


//ble_error_t QN_Gap::setDeviceName(uint8_t *deviceName)   //FixMeMah
//{
//   
//    if (setBleDeviceName(deviceName, strlen((char *)deviceName)) == GAP_ERR_NO_ERROR){
//        return BLE_ERROR_NONE;
//    } else {
//        return BLE_ERROR_PARAM_OUT_OF_RANGE;
//    }
//}

ble_error_t QN_Gap::getDeviceName(uint8_t *deviceName, unsigned *lengthP) //FixMeMah
{
    if (getBleDeviceName(deviceName, (uint16_t *)lengthP) == GAP_ERR_NO_ERROR) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
}

#if 0
ble_error_t QN_Gap::setAppearance(GapAdvertisingData::Appearance appearance)
{
    if (setBleDeviceAppearance(appearance) == GAP_ERR_NO_ERROR) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
}

ble_error_t QN_Gap::getAppearance(GapAdvertisingData::Appearance *appearanceP)
{
    if ((sd_ble_gap_appearance_get(reinterpret_cast<uint16_t *>(appearanceP)) == NRF_SUCCESS)) {
        return BLE_ERROR_NONE;
    } else {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
}

/* (Valid values are -40, -20, -16, -12, -8, -4, 0, 4) */
ble_error_t nRF5xGap::setTxPower(int8_t txPower)
{
    unsigned rc;
    if ((rc = sd_ble_gap_tx_power_set(txPower)) != NRF_SUCCESS) {
        switch (rc) {
            case NRF_ERROR_BUSY:
                return BLE_STACK_BUSY;
            case NRF_ERROR_INVALID_PARAM:
            default:
                return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }

    return BLE_ERROR_NONE;
}

void nRF5xGap::getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP)
{
#if defined(NRF51)
    static const int8_t permittedTxValues[] = {
        -30, -20, -16, -12, -8, -4, 0, 4
    };
#elif defined(NRF52)
    static const int8_t permittedTxValues[] = {
        -40, -20, -16, -12, -8, -4, 0, 4
    };
#else
#error permitted TX power values unknown for this SOC
#endif

    *valueArrayPP = permittedTxValues;
    *countP = sizeof(permittedTxValues) / sizeof(int8_t);
}
#endif

/**************************************************************************/
/*!
    @brief  Get the capacity of the internal whitelist maintained by this
            implementation.

    @returns    The capacity of the internal whitelist.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/

uint8_t QN_Gap::getMaxWhitelistSize(void) const
{
    //return YOTTA_CFG_WHITELIST_MAX_SIZE;
	return 0;
}

/**************************************************************************/
/*!
    @brief  Get a copy of the implementation's internal whitelist.

    @param[out] whitelistOut
                A \ref Gap::Whitelist_t structure containing a copy of the
                addresses in the implemenetation's internal whitelist.

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
#if 0
ble_error_t nRF5xGap::getWhitelist(Gap::Whitelist_t &whitelistOut) const
{
    uint8_t i;
    for (i = 0; i < whitelistAddressesSize && i < whitelistOut.capacity; ++i) {
        memcpy(&whitelistOut.addresses[i], &whitelistAddresses[i], sizeof(BLEProtocol::Address_t));
    }
    whitelistOut.size = i;

    return BLE_ERROR_NONE;
}
#endif
/**************************************************************************/
/*!
    @brief  Set the whitelist that will be used in the next call to
            startAdvertising().

    @param[in]  whitelistIn
                A reference to a \ref Gap::Whitelist_t structure
                representing a whitelist containing all the white listed
                BLE addresses.

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_INVALID_PARAM
                The supplied whitelist contains a private non-resolvable
                address

                BLE_ERROR_PARAM_OUT_OF_RANGE
                The size of the supplied whitelist exceeds the maximum
                capacity of the implementation's internal whitelist.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
#if 0
ble_error_t nRF5xGap::setWhitelist(const Gap::Whitelist_t &whitelistIn)
{
    if (whitelistIn.size > getMaxWhitelistSize()) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Test for invalid parameters before we change the internal state */
    for (uint8_t i = 0; i < whitelistIn.size; ++i) {
        if (whitelistIn.addresses[i].type == BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE) {
            /* This is not allowed because it is completely meaningless */
            return BLE_ERROR_INVALID_PARAM;
        }
    }

    whitelistAddressesSize = 0;
    for (uint8_t i = 0; i < whitelistIn.size; ++i) {
        memcpy(&whitelistAddresses[whitelistAddressesSize], &whitelistIn.addresses[i], sizeof(BLEProtocol::Address_t));
        whitelistAddressesSize++;
    }

    return BLE_ERROR_NONE;
}
#endif
/**************************************************************************/
/*!
    @brief  Set the advertising policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::setAdvertisingPolicyMode(Gap::AdvertisingPolicyMode_t mode)
{
    advertisingPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the scanning policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::setScanningPolicyMode(Gap::ScanningPolicyMode_t mode)
{
    scanningPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the initiator policy filter mode that will be used in
            the next call to startAdvertising()

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_Gap::setInitiatorPolicyMode(Gap::InitiatorPolicyMode_t mode)
{
    return BLE_ERROR_NOT_IMPLEMENTED;
}

/**************************************************************************/
/*!
    @brief  Get the current advertising policy filter mode.

    @returns    The advertising policy filter mode.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::AdvertisingPolicyMode_t QN_Gap::getAdvertisingPolicyMode(void) const
{
    return advertisingPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Get the current scanning policy filter mode.

    @returns    The scanning policy filter mode.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::ScanningPolicyMode_t QN_Gap::getScanningPolicyMode(void) const
{
    return scanningPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Get the current initiator policy filter mode.

    @returns    The initiator policy filter mode.

    @note   Currently initiator filtering using the whitelist is not
            implemented in this module.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::InitiatorPolicyMode_t QN_Gap::getInitiatorPolicyMode(void) const
{
    return Gap::INIT_POLICY_IGNORE_WHITELIST;
}


void QN_Gap::connectionCallback(struct gapc_connection_req_ind const *param)
{
    Gap::ConnectionParams_t connection_param;
    
    connection_param.minConnectionInterval = param->con_interval;
    connection_param.maxConnectionInterval = param->con_interval;
    connection_param.slaveLatency = param->con_latency;
    connection_param.connectionSupervisionTimeout = param->sup_to;
    ((QN_GattServer &)(QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE).getGattServer())).connectionEvent(param->conhdl);
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().processConnectionEvent(param->conhdl, Gap::PERIPHERAL, (BLEProtocol::AddressType_t)param->peer_addr_type, 
    param->peer_addr.addr, (BLEProtocol::AddressType_t)0, NULL, &connection_param);
}

void QN_Gap::disconnectionCallback(uint16_t conhdl, uint8_t reason)
{
    ((QN_GattServer &)(QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE).getGattServer())).disconnectionEvent(conhdl);
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().processDisconnectionEvent(conhdl, (Gap::DisconnectionReason_t)reason);
}


/**************************************************************************/
/*!
    @brief  Helper function used to populate the ble_gap_whitelist_t that
            will be used by the SoftDevice for filtering requests.

    @returns    \ref ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @retval     BLE_ERROR_INVALID_STATE
                The internal stack was not initialized correctly.

    @note  Both the SecurityManager and Gap must initialize correctly for
           this function to succeed.

    @note  This function is needed because for the BLE API the whitelist
           is just a collection of keys, but for the stack it also includes
           the IRK table.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
#if 0
ble_error_t nRF5xGap::generateStackWhitelist(ble_gap_whitelist_t &whitelist)
{
    ble_gap_whitelist_t  whitelistFromBondTable;
    ble_gap_addr_t      *addressPtr[1];
    ble_gap_irk_t       *irkPtr[YOTTA_CFG_IRK_TABLE_MAX_SIZE];

    nRF5xSecurityManager& securityManager = (nRF5xSecurityManager&) nRF5xn::Instance(0).getSecurityManager();

    if (securityManager.hasInitialized()) {
        /* We do not care about the addresses, set the count to 0 */
        whitelistFromBondTable.addr_count = 0;
        /* The Nordic SDK will return a failure if we set pp_addr to NULL */
        whitelistFromBondTable.pp_addrs   = addressPtr;
        /* We want all the IRKs we can get because we do not know which ones match the addresses */
        whitelistFromBondTable.irk_count  = YOTTA_CFG_IRK_TABLE_MAX_SIZE;
        whitelistFromBondTable.pp_irks    = irkPtr;

        /* Use the security manager to get the IRKs from the bond table */
        ble_error_t error = securityManager.createWhitelistFromBondTable(whitelistFromBondTable);
        if (error != BLE_ERROR_NONE) {
            return error;
        }
    } else  {
        /**
         * If there is no security manager then we cannot access the bond table,
         * so disable IRK matching
         */
        whitelistFromBondTable.addr_count = 0;
        whitelistFromBondTable.irk_count  = 0;
    }

    /**
     * For every private resolvable address in the local whitelist check if
     * there is an IRK for said address in the bond table and add it to the
     * local IRK list.
     */
    whitelist.irk_count  = 0;
    whitelist.addr_count = 0;
    for (uint8_t i = 0; i < whitelistAddressesSize; ++i) {
        if (whitelistAddresses[i].addr_type == BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE) {
            /* Test if there is a matching IRK for this private resolvable address */
            for (uint8_t j = 0; j < whitelistFromBondTable.irk_count; ++j) {
                if (securityManager.matchAddressAndIrk(&whitelistAddresses[i], whitelistFromBondTable.pp_irks[j])) {
                    /* Found the corresponding IRK, add it to our local whitelist */
                    whitelist.pp_irks[whitelist.irk_count] = whitelistFromBondTable.pp_irks[j];
                    whitelist.irk_count++;
                    /* Make sure we do not look at this IRK again */
                    if (j != whitelistFromBondTable.irk_count - 1) {
                        /**
                         * This is not the last IRK, so replace the pointer
                         * with the last pointer in the array
                         */
                        whitelistFromBondTable.pp_irks[j] =
                            whitelistFromBondTable.pp_irks[whitelistFromBondTable.irk_count - 1];
                    }
                    /**
                     * If the IRK is the last pointer in the array simply
                     * decrement the total IRK count
                     */
                    whitelistFromBondTable.irk_count--;
                    break;
                }
            }
        } else {
            /* Include the address into the whitelist */
            whitelist.pp_addrs[whitelist.addr_count] = &whitelistAddresses[i];
            whitelist.addr_count++;
        }
    }

    return BLE_ERROR_NONE;
}
#endif
