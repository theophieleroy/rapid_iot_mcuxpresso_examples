/**
 ****************************************************************************************
 *
 * @file gap_func.c
 *
 * @brief all gap functions.
 *
 * Copyright(C) 2016 NXP Semiconductors N.V.
 *
 * All rights reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDES
 ****************************************************************************************
 */
#include "app_config.h"
#include "app_ble.h"
#include "app.h"
#include "ke_mem.h"
#include "gap_func.h"

void *data_ptr;
volatile unsigned char func_flag=0,type;
volatile unsigned char func_flag1=0;
volatile unsigned char func_status=0;
struct gap_bdaddr ble_address;
/*
 * EXTERN FUNCTION DECLARATIONS 
  ****************************************************************************************
 */
 
 /**
 ****************************************************************************************
 * @brief get Local bluetooth address.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * This will initialize the BLE Host stack - rearrange to default settings the ATT, GAP,
 * GATT, L2CAP and SMP blocks. Furthermore, this will cause the host to send a reset command
 * down to the link layer part.
 *
 ****************************************************************************************
 */
 
 uint32_t getBleGapAddress( struct gap_bdaddr *ptr_addr)
{ 
	struct gapm_dev_bdaddr_ind *ble_dev_addr; 
	APP_GapmGetDevBdAddr();
	do{ 
	KE_Schedule();
	}while(!(func_flag & BLE_ADDRESS));
    type = ptr_addr->addr_type;
	func_flag &= BLE_ADDRESS_MASK;
	ble_dev_addr = (struct gapm_dev_bdaddr_ind *)data_ptr;
	
	//memcpy(ble_dev_addr,(struct gapm_dev_bdaddr_ind *)data_ptr,sizeof(struct gapm_dev_bdaddr_ind));
	//QPRINTF("Local BD address type: 0x%x.\r\n", *(struct gapm_dev_bdaddr_ind *)data_ptr);
	QPRINTF("Local BD address type: 0x%02x.\r\n", (struct gapm_dev_bdaddr_ind *)ble_dev_addr->addr.addr_type);
    QPRINTF("Local BD address: %02x%02x%02x%02x%02x%02x.\r\n", ble_dev_addr->addr.addr.addr[5], ble_dev_addr->addr.addr.addr[4],
            ble_dev_addr->addr.addr.addr[3], ble_dev_addr->addr.addr.addr[2], ble_dev_addr->addr.addr.addr[1], ble_dev_addr->addr.addr.addr[0]);
	ke_free(data_ptr);
   return GAP_ERR_NO_ERROR;
}

uint32_t updateBleConnectionParams(uint16_t conhdl, struct gapc_conn_param *params){
	
	APP_GapcParamUpdateCmd(conhdl,params);
	do{ 
	KE_Schedule();
	}while(!(func_flag & FB_DEV_UPD_PARAM));
	func_flag &= FB_DEV_UPD_PARAM_MASK;
	return GAP_ERR_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief get Device name.
 *
 * @response error status
 *
 * @description
 * This function gets device address locally from NVM.
 *
 ****************************************************************************************
 */

uint32_t getBleDeviceName(uint8_t *dev_name,uint16_t *dev_name_len)
{
	if (nvds_get(NVDS_TAG_DEVICE_NAME, dev_name_len, dev_name) != NVDS_OK)
  {
    dev_name_len = (uint16_t *)sizeof(GAP_DEV_NAME);
	memcpy(dev_name, GAP_DEV_NAME, *dev_name_len);
  }
	
	else
	{
		QPRINTF("get device name seccessful.\r\n");
	
	}
		return GAP_ERR_NO_ERROR;	
}
 /**
 ****************************************************************************************
 * @brief Set Device name.
 *
 * @response error status
 *
 * @description
 * This function sets device address locally in NVM.
 *
 ****************************************************************************************
 */

uint32_t setBleDeviceName(uint8_t *ptr_dev_name,uint16_t len)
{
	if (nvds_put(NVDS_TAG_DEVICE_NAME,len, ptr_dev_name) != NVDS_OK)
	{
			QPRINTF("Set device name failed.\r\n");
			/* Reject to change parameters */		
			return GAP_ERR_CANCELED;
	}
	else
	{
		QPRINTF("Set device name seccessful.\r\n");
		return GAP_ERR_NO_ERROR;
	}
						
}

#if 0
/**
 ****************************************************************************************
 * @brief set Device Apperance.
 *
 * @response 
 *
 * @description
 * This will initialize the BLE Host stack - rearrange to default settings the ATT, GAP,
 * GATT, L2CAP and SMP blocks. Furthermore, this will cause the host to send a reset command
 * down to the link layer part.
 *
 ****************************************************************************************
 */

uint32_t setBleDeviceAppearance(uint16_t appearance)
{
	
	struct gapc_set_dev_info_req_ind *dev_info_ptr;
	dev_info_ptr->req = GAPC_DEV_APPEARANCE;
	dev_info_ptr->info.apperance = appearance;
	APP_GapcSetDevInfoReqIndHandler(GAPC_SET_DEV_INFO_CFM,dev_info_ptr,TASK_GAP,TASK_APP);
	//do{ 
	KE_Schedule();
	//}while(!(func_flag & FB_DEV_NAME_SET));    
	
}

/**
 ****************************************************************************************
 * @brief set Device Apperance.
 *
 * @response 
 *
 * @description
 * This will initialize the BLE Host stack - rearrange to default settings the ATT, GAP,
 * GATT, L2CAP and SMP blocks. Furthermore, this will cause the host to send a reset command
 * down to the link layer part.
 *
 ****************************************************************************************
 */

uint32_t getBleDeviceAppearance(uint16_t appearance)
{
	
	struct gapc_set_dev_info_req_ind *dev_info_ptr;
	dev_info_ptr->req = GAPC_DEV_APPEARANCE;
	dev_info_ptr->info.apperance = appearance;
	APP_GapcSetDevInfoReqIndHandler(GAPC_SET_DEV_INFO_CFM,dev_info_ptr,TASK_GAP,TASK_APP);
	//do{ 
	KE_Schedule();
	//}while(!(func_flag & FB_DEV_NAME_SET));    
	
}
#endif
