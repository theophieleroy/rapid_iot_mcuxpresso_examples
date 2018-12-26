/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
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
 * o Neither the name of the copyright holder nor the names of its
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

#ifndef _USB_VCOM_ADAPTER_H_
#define _USB_VCOM_ADAPTER_H_
#include "fsl_common.h"
#include "usb.h"
#include "usb_device_descriptor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#define CONTROLLER_ID kUSB_ControllerEhci0
#define DATA_BUFF_SIZE HS_CDC_VCOM_BULK_OUT_PACKET_SIZE

#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0)
#define CONTROLLER_ID kUSB_ControllerKhci0
#define DATA_BUFF_SIZE FS_CDC_VCOM_BULK_OUT_PACKET_SIZE

#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#define DATA_BUFF_SIZE FS_CDC_VCOM_BULK_OUT_PACKET_SIZE

#endif

#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#define DATA_BUFF_SIZE HS_CDC_VCOM_BULK_OUT_PACKET_SIZE
#endif

#define USB_DEVICE_INTERRUPT_PRIORITY (3U)

/* Currently configured line coding */
#define LINE_CODING_SIZE (0x07)
#define LINE_CODING_DTERATE (115200)
#define LINE_CODING_CHARFORMAT (0x00)
#define LINE_CODING_PARITYTYPE (0x00)
#define LINE_CODING_DATABITS (0x08)

/* Communications feature */
#define COMM_FEATURE_DATA_SIZE (0x02)
#define STATUS_ABSTRACT_STATE (0x0000)
#define COUNTRY_SETTING (0x0000)

/* Notification of serial state */
#define NOTIF_PACKET_SIZE (0x08)
#define UART_BITMAP_SIZE (0x02)
#define NOTIF_REQUEST_TYPE (0xA1)

/* Define the types for application */
typedef struct _usb_cdc_vcom_struct
{
    usb_device_handle deviceHandle; /* USB device handle. */
    volatile uint8_t attach; /* A flag to indicate whether a usb device is attached. 1: attached, 0: not attached */
    uint8_t speed;           /* Speed of USB device. USB_SPEED_FULL/USB_SPEED_LOW/USB_SPEED_HIGH.                 */
    volatile uint8_t
        startTransactions; /* A flag to indicate whether a CDC device is ready to transmit and receive data.    */
    uint8_t currentConfiguration; /* Current configuration value. */
    uint8_t currentInterfaceAlternateSetting
        [USB_CDC_VCOM_INTERFACE_COUNT]; /* Current alternate setting value for each interface. */
    uint8_t hasSentState; /*!< 1: The device has primed the state in interrupt pipe, 0: Not primed the state. */
} usb_cdc_vcom_struct_t;

/* Define the infomation relates to abstract control model */
typedef struct _usb_cdc_acm_info
{
    uint8_t serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE]; /* Serial state buffer of the CDC device to notify the
                                                                     serial state to host. */
    bool dtePresent;          /* A flag to indicate whether DTE is present.         */
    uint16_t breakDuration;   /* Length of time in milliseconds of the break signal */
    uint8_t dteStatus;        /* Status of data terminal equipment                  */
    uint8_t currentInterface; /* Current interface index.                           */
    uint16_t uartState;       /* UART state of the CDC device.                      */
} usb_cdc_acm_info_t;

/* Forward declaration of the handle typedef. */
typedef struct _usb_handle usb_handle_t;

/*! @brief USB handle structure. */
struct _usb_handle
{
    uint8_t *volatile txData;   /*!< Address of remaining data to send. */
    volatile size_t txDataSize; /*!< Size of the remaining data to send. */
    uint8_t *volatile rxData;   /*!< Address of remaining data to receive. */
    volatile size_t rxDataSize; /*!< Size of the remaining data to receive. */

    void (*usb_tx_callback)(void); /*!< tx complete callback */
    void (*usb_rx_callback)(void); /*!< rx complete callback */
};

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief USB adapter initialization function.
 *
 * This function initializes the USB adapter.
 *
 * @return None.
 */
void ADAPTER_UsbInit(void);

/*!
 * @brief Start a data reception.
 *
 * This function receives data from the USB adapter.
 *
 * @param bufptr Pointer to the receive buffer.
 * @param size Size of the expected reception.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when reception is finished.
 */
void ADAPTER_UsbReceiveInt(uint8_t *buf, uint32_t size, void (*rx_callback)(void));

/*!
 * @brief Starts a data transmission.
 *
 * This function transmit data to the USB adapter
 *
 * @param bufptr Pointer to the transmit buffer.
 * @param size Size of the transmission.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when transmission is finished.
 */
void ADAPTER_UsbTransmitInt(uint8_t *buf, uint32_t size, void (*tx_callback)(void));

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _USB_VCOM_ADAPTER_H_ */
