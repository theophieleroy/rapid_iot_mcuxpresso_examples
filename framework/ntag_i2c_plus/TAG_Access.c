/*
 * TAG_Access.c
 *
 *  Created on: 06-Oct-2016
 *      Author: Bhavya
 */

/*==================================================================================================
Include Files
==================================================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "TAG_Access.h"
#include "Test_Functions.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "board.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "HAL_I2C_driver.h"
#include "HAL_ISR_driver.h"
#include "nfc_device.h"
#include "ntag_bridge.h"
#include "jsmn.h"
//#include "thread_attributes.h"

#include "ndef_message.h"
/*==================================================================================================
Private Macro
==================================================================================================*/
#define NTAG_I2C_BLOCK		(23)
#define NTAG_I2C_BLOCK_SIZE	0x10
#define MAX_TOKEN	        (30)
#define MIN_VAL(a,b)		((a) < (b) ? (a) : (b))

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static uint8_t JString[NTAG_I2C_BLOCK * NFC_I2C_BLOCK_SIZE] = {0};      // buffer to store payload(json string) of NDEF msg
uint8_t ntagUsrMemBuff[NTAG_I2C_BLOCK][NTAG_I2C_BLOCK_SIZE] = {{0}};    // buffer to store User Memory Blocks of NTAG
static jsmntok_t	t[MAX_TOKEN];
//static HAL_I2C_HANDLE_T *i2cHandleMaster;
static NTAG_HANDLE_T      ntag_handle;
static uint8_t          payLoadPosInNdef;

/*==================================================================================================
Private prototypes
==================================================================================================*/
static int jsoneq(const char *json, jsmntok_t *tok, const char *s);
static int8_t GetParsedData(jsmntok_t t[]);
static uint8_t GetValueOfKey(char *val, jsmntok_t *t, uint8_t totalKey);
static void parse_ndef( uint8_t rxBuffer[], uint8_t msgBuffer[], uint16_t *msgBufferSize, uint8_t *payloadStartPos);
static uint8_t writeNtagUserMemory(uint8_t *JString, uint16_t JStringLen, uint8_t usrMemAddrOffset);
static ntag_status_e readNtagUserMemory(uint8_t *readMessagebuffer);
static void WriteBuffer(uint8_t *dstBuff, uint8_t *srcBuff, uint8_t actualSize, uint8_t idleSize);

/*==================================================================================================
Private functions
==================================================================================================*/
static void WriteBuffer(uint8_t *dstBuff, uint8_t *srcBuff, uint8_t actualSize, uint8_t idleSize)
{
	uint8_t i = 0;

	actualSize = MIN_VAL(actualSize, idleSize-1);	// idleSize-1 to put NULL at last byte

	// Copy data into buffer
    for (i = 0; i < actualSize; i++)
	{
    	dstBuff[i] = srcBuff[i];
	}

    // zero(NULL) rest of the buffer
    for (i = actualSize; i < idleSize; i++)
    {
    	dstBuff[i] = 0;
    }
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) 
{
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) 
        {
		return 0;
	}
	return -1;
}

static int8_t GetParsedData(jsmntok_t t[])
{
	char msg[100];
	//Read User space of I2C for data
	if (NTAG_FAIL == readNtagUserMemory(JString))
	{
		return NTAG_FAIL;
	}

	//Parse JSON string
	int r;
	jsmn_parser p;

	jsmn_init(&p);	//init
	r = jsmn_parse(&p, (const char *)JString, strlen((const char *)JString), t, MAX_TOKEN);	//parse
  
	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT)
	{
		sprintf(msg,"Object expected\r\n");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
		return NTAG_FAIL;
	}

#if NTAG_DEBUG
	shell_printf("Total Key: %d\r\n", r);
#endif  //NTAG_DEBUG

	return r;	//total key
}

static uint8_t GetValueOfKey(char *val, jsmntok_t *t, uint8_t totalKey)
{
	/* Loop over all keys of the root object */
	uint8_t i;
	for (i = 1; i < totalKey; i++)
	{
		if (jsoneq((const char *)JString, &t[i], val) == 0)
		{
			return i;
		}
	}

	return NTAG_FAIL;	//if key is not available in data
}

/* NDEF Parser to parse JSON string*/
static void parse_ndef( uint8_t rxBuffer[], uint8_t msgBuffer[], uint16_t *msgBufferSize, uint8_t *payloadStartPos)
{
	uint8_t tnfPos;
	uint8_t recordTypePos;
	uint8_t textDataStartPos;
	uint16_t payLoadLenght;

	if ( 0xFF == rxBuffer[1] )	//TLV length is more than 1 byte
	{
		tnfPos = 4;
		recordTypePos = 10;
		payLoadLenght = (rxBuffer[8] * 256) + rxBuffer[9];
		textDataStartPos = 14;
	}
	else
	{
		tnfPos = 2;
		recordTypePos = 5;
		payLoadLenght = rxBuffer[4];
		textDataStartPos = 9;
	}

	uint8_t tnf = rxBuffer[tnfPos] & 0x07 ;

	if (tnf == 0x01) //well know type
	{
		if(rxBuffer[recordTypePos] == 'T') //text Record
		{
			*msgBufferSize = payLoadLenght-3;
			memcpy(msgBuffer, rxBuffer+textDataStartPos, *msgBufferSize);
                        *payloadStartPos = textDataStartPos;
		}
	}
}

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
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
		return NTAG_FAIL;
	}

	return NTAG_SUCCESS;
}

static ntag_status_e readNtagUserMemory(uint8_t *readMessagebuffer)
{
	char msg[100];
    uint16_t NDEFmessagelen=0;

    /* Read User Memory */
    uint8_t current_block;
    for (current_block = 1; current_block <= NTAG_I2C_BLOCK; current_block++)
    {
        if (NTAG_OK != NTAG_ReadBlock (ntag_handle, current_block, &ntagUsrMemBuff[(current_block - 1)][0], NTAG_I2C_BLOCK_SIZE))
        {
            sprintf(msg,"Failed to read block %u\r\n", current_block);
    		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
            return NTAG_FAIL;
        }
    }

    parse_ndef(&ntagUsrMemBuff[0][0], readMessagebuffer, &NDEFmessagelen, &payLoadPosInNdef);       //Parse NDEF message
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
    sprintf(msg,"payLoadPosInNdef: %d\r\n", payLoadPosInNdef);

    return NTAG_SUCCESS;
}

/*==================================================================================================
Public functions
==================================================================================================*/

//void InitNTAG(void)
//{
//    /* Declare and initialise for pull up configuration */
//    port_pin_config_t pinConfig;
//    memset(&pinConfig,0,sizeof(pinConfig));
//    pinConfig.pullSelect = kPORT_PullUp;
//
//    /* Ungate the port clock */
//    CLOCK_EnableClock(kCLOCK_PortB);
//    /* I2C0 pull up resistor setting */
//    PORT_SetPinConfig(PORTB, 0U, &pinConfig);
//    PORT_SetPinConfig(PORTB, 1U, &pinConfig);
//    /* I2C0 PIN_MUX Configuration */
//    PORT_SetPinMux(PORTB, 0U, kPORT_MuxAlt3);
//    PORT_SetPinMux(PORTB, 1U, kPORT_MuxAlt3);
//
//
//    // Initialize I2C
//    i2cHandleMaster = HAL_I2C_InitDevice(HAL_I2C_BAUDRATE_DEFAULT);
//    SystemCoreClockUpdate();
//
//    // Set interrupt for time measurement
////    SysTick_Config(SystemCoreClock / 1000); // produce a timer interrupt every 1ms
//
//    // Initialize the NTAG I2C components
//    ntag_handle = NFC_InitDevice(NTAG0, i2cHandleMaster);
////    HAL_ISR_RegisterCallback(0, ISR_LEVEL_LO, NULL, NULL);
//
//}

uint8_t  GetIdTag(uint8_t *id)
{
	if(!id)
	{
           return NTAG_FAIL;
	}

	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;

	if(!(retKeyPos = GetValueOfKey("id", t, totalKey)))
		return NTAG_FAIL;	//No Key
        if(t[retKeyPos+1].end - t[retKeyPos+1].start < 16)
            return NTAG_FAIL;           //No value                      
        
        WriteBuffer(id, JString + t[retKeyPos+1].start, t[retKeyPos+1].end - t[retKeyPos+1].start, EUI_SIZE);

	return NTAG_SUCCESS;
}

uint8_t GetTypeTag(uint8_t *type)
{
	if(!type)
	{
		return NTAG_FAIL;
	}

	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;

	if(!(retKeyPos = GetValueOfKey("type", t, totalKey)))
		return NTAG_FAIL;	//No Key
        
        if(t[retKeyPos+1].end - t[retKeyPos+1].start == 0)
            return NTAG_FAIL;           //No Value                                 
        
	*type =	*(JString + t[retKeyPos+1].start) - 48;

	return NTAG_SUCCESS;
}

uint8_t GetActiveTagValue(uint8_t *activetag)
{
	if(!activetag)
	{
		return NTAG_FAIL;
	}

	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;

	if(!(retKeyPos = GetValueOfKey("activetag", t, totalKey)))
		return NTAG_FAIL;	//No Key
        if(t[retKeyPos+1].end - t[retKeyPos+1].start == 0)
            return NTAG_FAIL;           //No Value
        
	*activetag =	*(JString + t[retKeyPos+1].start) - 48;

	return NTAG_SUCCESS;
}

uint8_t SetIdTag(char *id)
{
	char msg[100];
        if(!id)
	{
        sprintf(msg,"String is empty\n\r");
        PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
		return NTAG_FAIL;
	}
       
	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;
               
	if(!(retKeyPos = GetValueOfKey("id", t, totalKey)))
		return NTAG_FAIL;	//No Key
        
        if(!memcmp((JString + t[retKeyPos+1].start),id, 16))
          return NTAG_SUCCESS;
         
        //Modified new ID(EUI) 
         memcpy((JString + t[retKeyPos+1].start),id, 16);

        
        //Write NDEF Header to I2C Tag
	if(!writeNtagUserMemory(&ntagUsrMemBuff[0][0], payLoadPosInNdef, 0))
	{
		return NTAG_FAIL;
	}
   
	//Write modified data to I2C Tag
	if(!writeNtagUserMemory(JString, strlen((char const *)JString), payLoadPosInNdef))
	{
		return NTAG_FAIL;
	}
	return NTAG_SUCCESS;
}

uint8_t SetTypeTag(uint8_t *type)
{
	if(!type)
	{
		return NTAG_FAIL;
	}

	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;

	if(!(retKeyPos = GetValueOfKey("type", t, totalKey)))
		return NTAG_FAIL;	//No Key

        // Validate the tag has specified data or not
        // If it has same data as user want then return as no need a write operation
        if((JString + t[retKeyPos+1].start)[0] ==  (*type + 48)) {
            return NTAG_SUCCESS;
        }
	(JString + t[retKeyPos+1].start)[0] = *type + 48;

        //Write NDEF Header to I2C Tag
	if(!writeNtagUserMemory(&ntagUsrMemBuff[0][0], payLoadPosInNdef, 0))
	{
		return NTAG_FAIL;
	}
        
	//Write modified data to I2C Tag
	if(!writeNtagUserMemory(JString, strlen((char const *)JString), payLoadPosInNdef))
	{
		return NTAG_FAIL;
	}

	return NTAG_SUCCESS;
}

uint8_t SetActivetagTag(uint8_t *activetag)
{
	if(!activetag)        
	{
		return NTAG_FAIL;
	}

	uint8_t retKeyPos = 0;
	uint8_t totalKey = 0;

	if(!(totalKey = GetParsedData(t)))
		return NTAG_FAIL;

	if(!(retKeyPos = GetValueOfKey("activetag", t, totalKey)))
		return NTAG_FAIL;	//No Key

        // Validate the tag has specified data or not
        // If it has same data as user want then return as no need a write operation
        if((JString + t[retKeyPos+1].start)[0] ==  (*activetag + 48)) 
        {
            return NTAG_SUCCESS;
        }
	(JString + t[retKeyPos+1].start)[0] = *activetag + 48;

        //Write NDEF Header to I2C Tag
	if(!writeNtagUserMemory(&ntagUsrMemBuff[0][0], payLoadPosInNdef, 0))
	{
		return NTAG_FAIL;
	}
        
	//Write modified data to I2C Tag
	if(!writeNtagUserMemory(JString, strlen((char const *)JString), payLoadPosInNdef))
	{
		return NTAG_FAIL;
	}

	return NTAG_SUCCESS;
}


void factory_reset_Tag()
    {
    uint8_t page = 8;

    //reset default eeprom memory values (smart poster)
    NFC_WriteBytes(ntag_handle, NTAG_MEM_ADRR_I2C_ADDRESS, Default_BeginingOfMemory, Default_BeginingOfMemory_length);

    //reset pages from 8 to 56
    while(page < 56)
        {
        NFC_WriteBlock(ntag_handle, page, Null_Block, NTAG_I2C_BLOCK_SIZE);
        page ++;
        }

    //reset pages 56,57,58
    NFC_WriteBlock(ntag_handle, 56, Default_Page_56, NTAG_I2C_BLOCK_SIZE);
    NFC_WriteBlock(ntag_handle, 57, Default_Page_57, NTAG_I2C_BLOCK_SIZE);
    NFC_WriteBlock(ntag_handle, 58, Default_Page_58, NTAG_I2C_BLOCK_SIZE);
    
    }

uint8_t WriteTag()
{
    uint8_t eui[8],state = 0, eui_value[8];
    char id_string[18]={0};
    char buf[128] = {0};
    char msg[128];
    uint32_t attrLength;
    uint8_t status,i,j;
    mfg_t mfg;
    uint8_t strlength;
    uint8_t totalKey = 0;
    uint8_t empty=0;
    uint8_t retKeyPos = 0;
    uint8_t JString1[NTAG_I2C_BLOCK * NFC_I2C_BLOCK_SIZE] = {0}; 
    char data1[] = "{\"id\":\"0000000000000000\",\"type\":\"0\",\"activetag\":\"0\"}" ;
    uint8_t eui1[18];
    char str[] = ",\"activetag\":\"1\"";
    uint8_t len = 0;
 
    memset(eui_value,0,sizeof(eui_value));
  
//    THR_GetAttr(0,gNwkAttrId_IeeeAddr_c, 0, &(attrLength), eui_value); 	// [VF] To be replaced by other function
//    NWKU_SwapArrayBytes((uint8_t*)&eui_value, 8);						// [VF] To be replaced by other function


    // VF: Temp values only for test
    attrLength = 8; // [VF] 64 bit eui = 8 bytest ???
    eui_value[0] = 0x00;
    eui_value[1] = 0x60;
    eui_value[2] = 0x37;
    eui_value[3] = 0x00;
    eui_value[4] = 0x00;
    eui_value[5] = 0x0E;
    eui_value[6] = 0x22;
    eui_value[7] = 0xC4;
    
    for(i = 0,j=0; i < attrLength ; i++,j++) 
    {
        sprintf(&id_string[j++],"%02x",eui_value[i]);       
    }
    id_string[j] = '\0';        //Contains ID read from Thread Stack
    
    sprintf(buf,"{\"id\":\"%s\",\"type\":\"2\",\"activetag\":\"1\"}",id_string); 
    
    len = strlen(buf)+3;                //todo check why i need to add 3 bytes.
    sprintf(msg,"[Generated Tag]: %s, length: %d\n\r",buf,strlen((char *)buf));
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
    
    totalKey = GetParsedData(t);
	PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
    sprintf(msg,"[Read Tag]: %s, length: %d\n\r",(char *)(JString),strlen((char*)buf));
    //Check if no key found then rewrite complete string contained in buf.
    if(!totalKey) 
    {
         empty = 1;
    }
    else
    {
      char     nfcrx_id[17];
      uint8_t  type = 0xff;
      uint8_t  activetag = 0xff;
      
      if(NTAG_SUCCESS == GetIdTag((uint8_t *)nfcrx_id))
      {
          if(memcmp(nfcrx_id,id_string,16))
          {
              SetIdTag(id_string);           //Write Tag with actual value;
          } 
      }
      else
      {
          empty = 1;       
      }

      if(!empty)
      {
          if(NTAG_SUCCESS == GetTypeTag(&type))
          {
              if(type != 2)
              {                    
                  type = 2;
                  SetTypeTag(&type);          //Write value Tag;
              }
          }
          else
          {
            empty = 1;                    //Set the flag to write it from scratch      
          }
      }
      if(!empty)
      {
          if(NTAG_SUCCESS == GetActiveTagValue(&activetag))
          {
              if(activetag != 1)
              {                    
                  activetag = 1;
                  SetActivetagTag(&activetag);
              }         
          }
          else
          {
             
             //Add this Tag
             strlength = strlen((char *)JString);
             strcpy((char *)(JString+(strlength-1)),(const char *)str);
             strcpy((char *)(JString+(strlen((char *)JString))),(const char *)"}");
             sprintf(msg,"After Add ActiveTAG Jstring = %s\n",JString);
             PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
             
             factory_reset_Tag();
             
             char ntagheader[] = {0x03,0x00,0xD1,0x01,
                                0x00,0x54,0x02,0x65,
				0x6E};

              strlength = strlen((char *)JString);

              /* Write Zero to the previous data of string */
              if(!writeNtagUserMemory((uint8_t *)ntagheader, 9, 0)) 
              {
                  sprintf(msg,"failed to write header\n\r");
                  PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
                  return NTAG_FAIL;
              }
              if(!writeNtagUserMemory((uint8_t *)JString, strlen((char const *)JString), payLoadPosInNdef)) 
              {
                 sprintf(msg,"faild to write on a tag");
                 PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
                 return NTAG_FAIL;
              }
              
              strlength = strlen((char *)JString);
	
              uint8_t end_byte = 0xFE;
              // Write 0xFE which is indicate last byte of message
              if(!writeNtagUserMemory(&end_byte, 1 , strlen((char *)JString)+9))
              {
                 sprintf(msg,"faild to write on a tag");
                 PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
                 return NTAG_FAIL;
              }

              uint8_t length_field = strlength+7;
              uint8_t payload_length = strlength+3;
              if(!writeNtagUserMemory(&length_field, 1, 1)) 
              {
                 sprintf(msg,"Failed to write length field\n\r");
                 PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
                 return NTAG_FAIL;
              }
              if(!writeNtagUserMemory(&payload_length, 1, 4)) 
              {
                 sprintf(msg,"Failed to write payloadpos\n\r");
                 PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
                 return NTAG_FAIL;
              }

              if(NTAG_FAIL != (GetParsedData(t))) 
              {
                 return NTAG_SUCCESS;
              }
              else 
              {
                return NTAG_FAIL;
              }
          }
       }
    }
    
    
    if(empty)
    {
      sprintf(msg,"Writing Default Jstring\n\r");
      PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
      
      factory_reset_Tag();
      
      char ntagheader[] = {0x03,0x00,0xD1,0x01,
                            0x00,0x54,0x02,0x65,
                            0x6E};
      
      //Write NDEF header to I2C tag
      if(!writeNtagUserMemory((uint8_t *)ntagheader, 9, 0)) 
      {
        sprintf(msg,"failed to write\n\r");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
      
      memcpy(JString, buf,strlen((char const *)buf));
      strlength = strlen((char *)JString);
      
      payLoadPosInNdef = (0==payLoadPosInNdef)?9:payLoadPosInNdef;
      
      //Write actual data
      if(!writeNtagUserMemory((uint8_t *)JString, strlength, payLoadPosInNdef)) 
      {
        sprintf(msg,"faild to write Default Tag");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
      
      // Write 0xFE which is indicate last byte of message
      uint8_t end_byte = 0xFE;
      if(!writeNtagUserMemory(&end_byte, 1 , strlength + 9))
      {
        sprintf(msg,"faild to write End Byte");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
      
      uint8_t length_field = strlength+7;
      uint8_t payload_length = strlength+3;
      
      sprintf(msg,"length_field %d, payload_length:%d\n",length_field,payload_length);
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
      if(!writeNtagUserMemory(&length_field, 1, 1))
      {
        sprintf(msg,"Failed to write length field\n\r");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
      if(!writeNtagUserMemory(&payload_length, 1, 4))
      {
        sprintf(msg,"Failed to write payloadpos\n\r");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
      
      if(NTAG_FAIL != (GetParsedData(t)))
      {
        sprintf(msg,"[Jstring]  :%s  len:%d\n\r",totalKey,(char *)(JString),strlen((char *)JString));
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_SUCCESS;
      }
      else
      {
        sprintf(msg,"Failed to Parse NTAG Data After Default write\n\r");
		PrintUart(TERMINAL_UART, (uint8_t *)msg, strlen(msg));
        return NTAG_FAIL;
      }
    } 
    return NTAG_SUCCESS;  
}
