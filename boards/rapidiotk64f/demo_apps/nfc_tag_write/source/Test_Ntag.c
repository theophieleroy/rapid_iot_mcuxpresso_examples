/*
 * Test_Ntag.c
 *
 *  Created on: Dec 13, 2017
 *      Author: frq05115
 */
#include <stdio.h>
#include "Test_Ntag.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "board.h"

#include "HAL_I2C_driver.h"
#include "HAL_ISR_driver.h"
#include "nfc_device.h"
#include "ntag_bridge.h"
#include "jsmn.h"
#include "ntag_driver_intern.h"

#include "ndef_message.h"

/*==================================================================================================
Private Macro
==================================================================================================*/
#define NTAG_I2C_BLOCK		(23)
#define NTAG_I2C_BLOCK_SIZE	0x10
#define MAX_TOKEN	        (30)
#define MIN_VAL(a,b)		((a) < (b) ? (a) : (b))


/* Prototypes */
static uint8_t writeNtagUserMemory(uint8_t *JString, uint16_t JStringLen, uint8_t usrMemAddrOffset);
void factory_reset_Tag();

/* Variables */
static uint8_t JString[NTAG_I2C_BLOCK * NFC_I2C_BLOCK_SIZE] = {0};      // buffer to store payload(json string) of NDEF msg
uint8_t ntagUsrMemBuff[NTAG_I2C_BLOCK][NTAG_I2C_BLOCK_SIZE] = {{0}};    // buffer to store User Memory Blocks of NTAG
static NTAG_HANDLE_T      ntag_handle;


bool Init_Ntag_NDEF(void)
{
	HAL_I2C_InitDevice(HAL_I2C_INIT_DEFAULT, I2C2_CLK_SRC, NTAG_I2C_SLAVE_BASEADDR);

	ntag_handle = NTAG_InitDevice((NTAG_ID_T)0, NTAG_I2C_SLAVE_BASEADDR);

	/* Switch Power supply and I2C */
	Connect_NTAG_A1006();

	/* New code for NDEF stucture Write => NFC commisioning */
	factory_reset_Tag();

	return Write_Ntag_NDEF("---");
}

bool Write_Ntag_NDEF(char *buf)
{
	uint8_t strlength;
	uint8_t applength;

	char ntagheader[] = {0x03,
						 0x00,	// will be replaced later by calculated length field
						 0x91,	// 0xD1 replaced by 0x91: MB=1, ME=0 <=> first record
						 0x01,	// Type length = 1 <=> 1 byte
						 0x00,	// Will be replaced later by calculated payload length = 0x37
						 0x54,
						 0x02,0x65,0x6E};


	char ntag_rpk_app[] = {0x54,0x0F,														// record header
						   0x15,															// Payload lenght = 21 bytes
						   0x61,0x6E,0x64,0x72,0x6F,0x69,0x64,0x2E,0x63,0x6F,0x6D,0x3A,		//android.com:
						   0x70,0x6B,0x67,													//pkg
						   0x63,0x6F,0x6D,0x2E,0x6E,0x78,0x70,0x2E,0x69,0x6F,				//com:nxp.io
						   0x74,0x2e,0x72,0x61,0x70,0x69,0x64,0x2d,0x69,0x6f,0x74,			//t.rapid-iot
						   0xFE}; 															// end of message

	applength = sizeof(ntag_rpk_app);

	/* DO NOT REMOVE !!! */
	App_WaitUsec(100);

	/* Write NDEF header to I2C tag */
	if(!writeNtagUserMemory((uint8_t *)ntagheader, 9, 0))
	{
		return NTAG_FAIL;
	}

	memcpy(JString, buf,strlen((char const *)buf));
	strlength = strlen((char *)JString);

	/* Write actual data */
	if(!writeNtagUserMemory((uint8_t *)JString, strlength, 9))
	{
		return NTAG_FAIL;
	}

	uint8_t length_field = strlength+6+applength;
	uint8_t payload_length = strlength+3;

	if(!writeNtagUserMemory(&length_field, 1, 1))
	{
		return NTAG_FAIL;
	}
	if(!writeNtagUserMemory(&payload_length, 1, 4))
	{
		return NTAG_FAIL;
	}

	/* Add Android app link */
	if(!writeNtagUserMemory((uint8_t *)ntag_rpk_app, applength, strlength + 9))
	{
		return NTAG_FAIL;
	}

	return NTAG_SUCCESS;
}

/*
 * Write "JStringLen" bytes of data from "JString"
 * to the NFC memory address plus offset "usrMemAddrOffset"
 */
static uint8_t writeNtagUserMemory(uint8_t *JString, uint16_t JStringLen, uint8_t usrMemAddrOffset)
{
	char msg[100];
	if(!JString)
	{
		return NTAG_FAIL;
	}

	if (NTAG_OK != NFC_WriteBytes(ntag_handle, NFC_MEM_ADDR_START_USER_MEMORY + usrMemAddrOffset, JString, JStringLen))
	{
		sprintf(msg,"Failed to write status\r\n");
		return NTAG_FAIL;
	}

	return NTAG_SUCCESS;
}

/*
 * Reset NFC module
 */
void factory_reset_Tag()
{
	uint8_t page = 8;

	/* reset default eeprom memory values (smart poster) */
	NFC_WriteBytes(ntag_handle, NTAG_MEM_ADRR_I2C_ADDRESS, Default_BeginingOfMemory, Default_BeginingOfMemory_length);

	/* reset pages from 8 to 56 */
	while(page < 56)
	{
		NTAG_WriteBlock(ntag_handle, page, Null_Block, NTAG_I2C_BLOCK_SIZE);
		page ++;
	}

	/* reset pages 56,57,58 */
	NTAG_WriteBlock(ntag_handle, 56, Default_Page_56, NTAG_I2C_BLOCK_SIZE);
	NTAG_WriteBlock(ntag_handle, 57, Default_Page_57, NTAG_I2C_BLOCK_SIZE);
	NTAG_WriteBlock(ntag_handle, 58, Default_Page_58, NTAG_I2C_BLOCK_SIZE);
}

