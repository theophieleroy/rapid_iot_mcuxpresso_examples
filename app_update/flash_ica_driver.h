/*
 * Copyright 2017 NXP
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
#ifndef _FICA_DRIVER_H_
#define _FICA_DRIVER_H_

// Enable to perform reads before and after writes to flash
//#define DEBUG_IMG

//! @addtogroup flash_ica
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Externals
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#include "stdbool.h"

#define USE_CRC_16 1

/*******************************************************************************
 * ICA (Image Configuration Area) Definitions
 ******************************************************************************/
#define FICA_VER 1

// ICA Descriptor, used to determine if the ICA area has been initialized or not
#define FICA_ICA_DESC 0xA5A5A5A5

// Start address of ICA area in flash
#define FICA_START_ADDR 0x00000000

// Current size of K64F internal flash
#define FICA_MAX_APP_SIZE 0x100000

// ICA field size
#define FICA_FIELD_SIZE 4

// ICA Offsets from FICA_START_ADDR
#define FICA_OFFSET_ICA_DESC   0
#define FICA_OFFSET_ICA_VER    4
#define FICA_OFFSET_ICA_COMM   8
#define FICA_OFFSET_ICA_RSV2  12
#define FICA_OFFSET_ICA_RSV3  16

#define ICA_COMM_K64_FACT_BOOT  0x00000001
#define ICA_COMM_K41_FACT_BOOT  0x00000002
#define ICA_COMM_K41_PROGRAM    0x00000004

// Size of the ICA definitions (summary of ICA Offsets right above)
#define FICA_ICA_DEFINITION_SIZE 20

// Maximum number of FICA images for this flash (MT25QL128)
#define FICA_MAX_IMAGES 14


/*******************************************************************************
 * Image Field Definitions
 * Defaults and choices for each of the ICA Record fields
 * Each offset is a word (4 bytes)
 ******************************************************************************/

// Image Record Start Descriptor
#define FICA_REC_START_ID 0x5A5A5A5A

// Image Record offsets from start of record
#define FICA_OFFSET_IMG_DESC          0
#define FICA_OFFSET_IMG_TYPE          4
#define FICA_OFFSET_IMG_ICA_RSV1      8
#define FICA_OFFSET_IMG_ICA_RSV2     12
#define FICA_OFFSET_IMG_ICA_RSV3     16
#define FICA_OFFSET_IMG_ICA_RSV4     20
#define FICA_OFFSET_IMG_ICA_RSV5     24
#define FICA_OFFSET_IMG_ICA_RSV6     28
#define FICA_OFFSET_IMG_START        32
#define FICA_OFFSET_IMG_SIZE         36
#define FICA_OFFSET_IMG_FMT          40
#define FICA_OFFSET_IMG_HASH_TYPE    44
#define FICA_OFFSET_IMG_HASH_LOC     48
#define FICA_OFFSET_IMG_ENC_TYPE     52
#define FICA_OFFSET_IMG_PKI_TYPE     56
#define FICA_OFFSET_IMG_PKI_LOC      60
#define FICA_OFFSET_IMG_FAC_BOOT     64
#define FICA_OFFSET_IMG_RSV2         68
#define FICA_OFFSET_IMG_RSV3         72
#define FICA_OFFSET_IMG_RSV4         76
#define FICA_OFFSET_IMG_RSV5         80
#define FICA_OFFSET_IMG_RSV6         84
#define FICA_OFFSET_IMG_RSV7         88
#define FICA_OFFSET_IMG_RSV8         92
#define FICA_OFFSET_IMG_RSV9         96

// Image Record Field Default values and choices
// Offset 0 - Descriptor ID (start id)
#define FICA_IMG_DESC_ID     0x5A5A5A5A

// Offset 1 - Image Types
#define FICA_IMG_TYPE_NONE   0     // default
#define FICA_IMG_TYPE_K64F_FACT      1 // K64F Factory Image
#define FICA_IMG_TYPE_K41Z_FACT      2 // K41Z Factory Image
#define FICA_IMG_TYPE_K64F_USER      3 // K64F User Image
#define FICA_IMG_TYPE_K41Z_USER      4 // K41Z User Image
#define FICA_IMG_TYPE_K64F_BL_FACT   5 // K64F Bootloader Factory Image
#define FICA_IMG_TYPE_K41Z_BL_FACT   6 // K41Z Bootloader Factory Image
#define FICA_IMG_TYPE_K64F_CUR       7 // K64F Current Image
#define FICA_IMG_TYPE_K41Z_CUR       8 // K41Z Current Image

// Total Number of Images Defined in this Version
#define FICA_NUM_IMG_TYPES 8

// Image Flash Start Addresses for this flash (MT25QL128)
#define FICA_IMG_ICA_DEF_ADDR          0x00000000
#define FICA_IMG_FAC_K64F_APP_ADDR     0x00100000
#define FICA_IMG_FAC_K41Z_APP_ADDR     0x00200000
#define FICA_IMG_NEW_K64F_APP_ADDR     0x00300000
#define FICA_IMG_NEW_K41Z_APP_ADDR     0x00400000
#define FICA_IMG_BL_FACT_K64F_APP_ADDR 0x00500000
#define FICA_IMG_BL_FACT_K41Z_APP_ADDR 0x00600000
#define FICA_IMG_CUR_K64F_APP_ADDR     0x00700000 // just defaulting to this, not used at the moment
#define FICA_IMG_CUR_K41Z_APP_ADDR     0x00800000 // just defaulting to this, not used at the moment
#define FICA_IMG_DEFAULT_APP_ADDR      0x00F00000

// Image Size
#define FICA_IMG_SIZE_ZERO   0 // default

// Image Format
#define FICA_IMG_FMT_NONE    0 // default
#define FICA_IMG_FMT_BIN     1
#define FICA_IMG_FMT_SREC    2
#define FICA_IMG_FMT_AES_128 3

// Image CRC
#define FICA_IMG_CRC_NONE    0 // default

// Image Key
#define FICA_IMG_HASH_NONE   0 // default

// Offset 7 - Reserved
#define FICA_IMG_RSV         0 // default

// ICA Record Structure
typedef struct ICA_Record
{
	uint32_t descriptor;
	uint32_t imgtype;
	uint32_t ica_rsv1;
	uint32_t ica_rsv2;
	uint32_t ica_rsv3;
	uint32_t ica_rsv4;
	uint32_t ica_rsv5;
	uint32_t ica_rsv6;
	uint32_t imgaddr;
	uint32_t imglen;
	uint32_t imgfmt;
	uint32_t imghashtype;
	uint32_t imghashloc;
	uint32_t imgenctype;
	uint32_t imgpkitype;
	uint32_t imgpkiloc;
	uint32_t rsv1;
	uint32_t rsv2;
	uint32_t rsv3;
	uint32_t rsv4;
	uint32_t rsv5;
	uint32_t rsv6;
	uint32_t rsv7;
	uint32_t rsv8;
	uint32_t rsv9;
} FICA_Record;


// Uncomment to Blink progress on LEDs
#define BLINK_PROGRESS 1
#define BLINK_RATE 500000 // 500ms

#define TYPE_APP 0
#define TYPE_IMG 1

#define IMG_EXT_NO_ERROR 0
#define IMG_EXT_ERROR 1

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


int32_t FICA_app_program_ext_init(uint32_t newimgtype);
int32_t FICA_app_program_ext_cont(void *pbuf, uint32_t len);
int32_t FICA_app_program_ext_abs(uint32_t imgaddr, uint8_t *pbuf, uint32_t len, bool apporimg);
int32_t FICA_app_program_ext_flush();
int32_t FICA_app_program_ext_calculate_crc(uint32_t imgtype, uint32_t *pcrc);
int32_t FICA_app_program_ext_program_crc(uint32_t imgtype, uint32_t crc);
int32_t FICA_clear_buf(uint8_t *pbuf, uint8_t initval);
int32_t FICA_read_db();
int32_t FICA_write_db();
int32_t FICA_write_buf(uint32_t offset, uint32_t len, void *buf);
int32_t FICA_read_buf(uint32_t offset, uint32_t len, void *buf);
int32_t FICA_get_field_addr(uint32_t imgtype, uint32_t fieldoffset, uint32_t *pfieldaddr);
int32_t FICA_write_field(uint32_t imgtype, uint32_t fieldoffset, uint32_t val);
int32_t FICA_write_field_no_save(uint32_t imgtype, uint32_t fieldoffset, uint32_t val);
int32_t FICA_read_field(uint32_t imgtype, uint32_t field_offset, uint32_t *pfieldval);
int32_t FICA_Calculate_CRC(uint32_t imgtype, uint32_t *crcval);
int32_t FICA_Calculate_CRC_Internal(uint32_t imgtype, uint32_t len, uint32_t *crcval);
int32_t FICA_Program_CRC(uint32_t imgtype, uint32_t crcval);
uint32_t FICA_get_app_img_start_addr(uint32_t imgtype);
uint32_t FICA_get_app_img_len(uint32_t imgtype);
int32_t FICA_write_image_info(uint32_t imgtype, uint32_t imgfmt, uint32_t len);
int32_t FICA_read_record(uint32_t imgtype, FICA_Record *pimgrec);
int32_t FICA_write_record(uint32_t imgtype, FICA_Record *pimgrec);
bool is_FICA_initialized(void);
int32_t FICA_initialize(void);
int32_t FICA_verify_ext_flash(void);
uint16_t FICA_compute_chunk_CRC(uint8_t *pData, uint16_t lenData, uint16_t crcValueOld);
int32_t FICA_get_comm_flag(uint32_t comflag, bool *flagstate);
int32_t FICA_set_comm_flag(uint32_t comflag, bool flagstate);
void reset_mcu(void);


#if defined(__cplusplus)
}
#endif

//! @}

#endif /* _FICA_DRIVER_H_ */
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

