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

#ifndef __BL_APU_H__
#define __BL_APU_H__

//! @addtogroup app_update
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#if defined(CPU_MK64FN1M0VMD12)
#define INT_FLASH_ERASE_PAGE 4096
#define APP_UPDATE_VECTOR_TABLE_ADDRESS 0x00014000
#elif defined(CPU_MKW41Z512VHT4)
#define INT_FLASH_ERASE_PAGE 2048
#define APP_UPDATE_VECTOR_TABLE_ADDRESS 0x00004000
#endif

#define TEST_VERSION				"1.1"

/* TERMINAL UART definitions*/
#define TERMINAL_UART 				UART3
#define TERMINAL_UART_CLKSRC 		UART3_CLK_SRC
#define TERMINAL_UART_CLK_FREQ 		CLOCK_GetFreq(TERMINAL_UART_CLKSRC)
#define TERMINAL_UART_RX_TX_IRQn 	UART3_RX_TX_IRQn
#define TERMINAL_UART_BAUDRATE		115200
#define TERMINAL_UART_IRQHandler 	UART3_RX_TX_IRQHandler
#define TERMINAL_RING_BUFFER_SIZE	1024

/* KW41Z Reset control definitions */
#define BOARD_KW41RST_GPIO 			GPIOB
#define BOARD_KW41RST_GPIO_PIN		23U
#define RESET_WAIT_TIME_MS			10U

/* RTC CLOCK ENABLE */
#define BOARD_RTC_CLKOE_GPIO 		GPIOD
#define BOARD_RTC_CLKOE_GPIO_PIN	14U


/*! @brief Generic application update status return codes. */
enum _app_generic_status
{
    kAppStatus_Success = MAKE_STATUS(kStatusGroup_Generic, 0),
    kAppStatus_Fail = MAKE_STATUS(kStatusGroup_Generic, 1),
    kAppStatus_ReadOnly = MAKE_STATUS(kStatusGroup_Generic, 2),
    kAppStatus_OutOfRange = MAKE_STATUS(kStatusGroup_Generic, 3),
    kAppStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),
    kAppStatus_Timeout = MAKE_STATUS(kStatusGroup_Generic, 5),
    kAppStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),
    kAppStatus_TransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 7),
    kAppStatus_TransferPassed = MAKE_STATUS(kStatusGroup_Generic, 8),
    kAppStatus_TransferFailed = MAKE_STATUS(kStatusGroup_Generic, 9)
};

/*! @brief Type used for all application update status and error return values. */
typedef int32_t appstatus_t;

/*! @brief Capture application update status and return it as well */
//#define return_status(code) AppUpdStatus=code; return(code);


////////////////////////////////////////////////////////////////////////////////
// Externals
////////////////////////////////////////////////////////////////////////////////

extern char *pmsg;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus


appstatus_t apu_program(uint32_t curimgtype, uint32_t newimgtype);
appstatus_t apu_move_ext_page_to_int_page(uint32_t intimgaddr, uint32_t extimgaddr, uint32_t intpagenum, uint32_t extpagenum, uint8_t *pbuf, uint32_t intpagelen, uint32_t extpagelen, uint32_t writelen);
appstatus_t apu_start(uint32_t newimgtype);
appstatus_t apu_finish(uint32_t curimgtype);
appstatus_t apu_init();
appstatus_t apu_calculate_crc(uint32_t imgtype, uint32_t len, uint32_t *pcrc);
appstatus_t apu_program_crc(uint32_t imgtype, uint32_t crc);
appstatus_t apu_blocking(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype);
#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
appstatus_t apu_blocking_k64(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype);
#else
appstatus_t apu_blocking_k41(uint32_t curimgtype, uint32_t newimgtype, uint32_t facimgtype);
#endif
appstatus_t apu_info_get_status();
bool apu_is_new_app_needed(uint32_t curimgtype, uint32_t newimgtype);
bool apu_is_fac_app_needed();
void apu_clear_buf(uint8_t *buf, uint32_t len);
appstatus_t apu_validate_image_crc(uint32_t imgtype);
#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
appstatus_t apu_wait_for_k41();
bool apu_force_factory_load(uint32_t cur_img_type);
void apu_clear_msd();
#endif

#ifdef UART_DEBUG
void PrintMsg(char *msg);
void InitTerminalUART();
void PrintUart(UART_Type *base, const uint8_t *data, size_t length);
#endif

#if defined(__cplusplus)
}
#endif // __cplusplus

//! @}

#endif // __BL_APU_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
