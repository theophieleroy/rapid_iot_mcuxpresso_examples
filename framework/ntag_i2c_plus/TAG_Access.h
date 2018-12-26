//////////////////////////////////////////////////////////
//
// TAG_Access.h
//
//  Created on: 06-Oct-2016
//      Author: Volansys
//
//////////////////////////////////////////////////////////

#ifndef SOURCE_APPLICATION_INC_TAG_ACCESS_H_
#define SOURCE_APPLICATION_INC_TAG_ACCESS_H_

/*!=================================================================================================
\file       TAG_Access.h
\brief      This is a header file to access NTAG I2C
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/*==================================================================================================
Public Macro
==================================================================================================*/
#define NTAG_DEBUG     0

#define VCODE_SIZE			(15)
#define VDEVICENAME_SIZE	        (15)
#define MODEL_SIZE			(15)
#define DESCRIPTION_SIZE	        (20)
#define SHAREDKEY_SIZE		        (16)
#define EUI_SIZE			(17)
#define TIME_SIZE			(25)

#define LOCATION_SIZE		        (10)
#define TIME_SIZE			(25)

/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum NTAG_STATUS_E
{
	NTAG_FAIL = 0,
	NTAG_SUCCESS
}ntag_status_e;

typedef struct MFG_T
{
	uint8_t vcode[VCODE_SIZE];			// Vendor Code
	uint8_t vdevicename[VDEVICENAME_SIZE];	        // Vendor Device Name
	uint8_t model[MODEL_SIZE];			// Model number
	uint8_t description[DESCRIPTION_SIZE];	        // Device description
	uint8_t sharedkey[SHAREDKEY_SIZE];		// 15 byte shared key
	uint8_t eui[EUI_SIZE];			        // 16 byte EUI
	uint8_t time[TIME_SIZE];			// GMT format
}mfg_t;

typedef struct INSTALL_T
{
	uint8_t location[LOCATION_SIZE];
	uint8_t time[TIME_SIZE];
	uint8_t state;
}install_t;



/*==================================================================================================
Public function prototypes
==================================================================================================*/

///
///	Api to init I2C perpheral
///
void InitNTAG(void);

///
///	Get manufacturing information like eui-64, model, part, time from NTAG I2C
/// \param[out] mfg	pointer to structure
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t GetMfgTag(mfg_t *mfg);

///
///	Get installation related information like location, time, state from NTAG I2C
///	\param[out]	install	pointer to structure
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t GetInstallTag(install_t *install);

///
///	Get EUI of the device
///	\param[out]	eui	buffer to store EUI
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t GetEuiTag(uint8_t *eui);

///
///	Get Status of the device
///	\param[out]	state	buffer to store device status
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t GetStateTag(uint8_t *state);

///
///	Get location of device
///	\param[out]	x device position
///	\param[out]	y device position
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t GetLocationTag(uint8_t *x, uint8_t *y);

///
///	Set status of the device
///	\param[in]	state	status of the device
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t SetStateTag(uint8_t *state);

///
///	Set EUI of the device
///	\param[in]	eui	eui of the device
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t SetEuiTag(char *eui);

///
///	Write EUI of the device to the NTAG
///
///	\return NTAG_SUCCESS/NTAG_FAIL
///
uint8_t WriteTag(uint8_t* MAC_Add);

#endif /* SOURCE_APPLICATION_INC_TAG_ACCESS_H_ */
