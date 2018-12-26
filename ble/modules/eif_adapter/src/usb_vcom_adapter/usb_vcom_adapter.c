/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
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
#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "fsl_debug_console.h"

#include "usb_device_descriptor.h"
#include "usb_vcom_adapter.h"
#include "fsl_power.h"
#include "fsl_calibration.h"

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/
/* Data structure of virtual com device */
static usb_cdc_vcom_struct_t s_cdcVcom;

/* Line codinig of cdc device */
static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    /* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
    (LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS};

/* Abstract state of cdc device */
static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
                                                          (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU};

/* Country code of cdc device */
static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {(COUNTRY_SETTING >> 0U) & 0x00FFU,
                                                        (COUNTRY_SETTING >> 8U) & 0x00FFU};

/* CDC ACM information */
USB_DATA_ALIGNMENT static usb_cdc_acm_info_t s_usbCdcAcmInfo = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0};

/* Data buffer for receiving and sending from USB */
USB_DATA_ALIGNMENT static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
USB_DATA_ALIGNMENT static uint8_t s_currSendBuf[DATA_BUFF_SIZE];
volatile static uint32_t s_recvSize = 0;
volatile static uint32_t s_sendSize = 0;
static uint32_t s_usbBulkMaxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
/* Pointer to the start of the valid data in s_currRecvBuf */
uint8_t s_validDataStart = 0;

/* USB adapter handle */
usb_handle_t s_usbHandle;

/*******************************************************************************
* Prototypes
******************************************************************************/
/*!
 * @brief USB adapter receive data.
 *
 * This function copys the data receive from USB to the receive buffer of
 * the BLE stack. And schedule buffer for next USB receive event.
 *
 * @param handle USB device handle.
 *
 * @retval kStatus_USB_Success              Get a device handle successfully.
 * @retval kStatus_USB_InvalidHandle        The device handle is invalided.
 * @retval kStatus_USB_ControllerNotFound   The controller interface is not found.
 * @retval kStatus_USB_Error                The device is doing reset.
 */
static usb_status_t ADAPTER_UsbReceiveData(usb_device_handle handle);

/*******************************************************************************
* Code
******************************************************************************/

static usb_status_t ADAPTER_UsbReceiveData(usb_device_handle handle)
{
    usb_status_t error = kStatus_USB_Success;
    size_t bytesAvailable = s_recvSize;
    s_validDataStart = 0;
    if (bytesAvailable >= s_usbHandle.rxDataSize)
    {
        for (int i = 0; i < s_usbHandle.rxDataSize; i++)
        {
            s_usbHandle.rxData[i] = s_currRecvBuf[s_validDataStart++];
        }
        s_usbHandle.rxDataSize = 0;
        s_usbHandle.usb_rx_callback();
    }
    else
    {
        for (int i = 0; i < bytesAvailable; i++)
        {
            s_usbHandle.rxData[i] = s_currRecvBuf[s_validDataStart++];
        }
        s_usbHandle.rxDataSize -= s_recvSize;
        s_usbHandle.rxData += s_recvSize;
    }

    /* If s_currRecvBuf is empty, schedule buffer for next USB receive event. */
    if (s_validDataStart == s_recvSize)
    {
        error = USB_DeviceRecvRequest(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf, s_usbBulkMaxPacketSize);
    }

    return error;
}

/*!
 * @brief Interrupt in pipe callback function.
 *
 * This function serves as the callback function for interrupt in pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmInterruptIn(usb_device_handle handle,
                                         usb_device_endpoint_callback_message_struct_t *message,
                                         void *callbackParam)
{
    usb_status_t error = kStatus_USB_Error;
    s_cdcVcom.hasSentState = 0;
    return error;
}

/*!
 * @brief Bulk in pipe callback function.
 *
 * This function serves as the callback function for bulk in pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmBulkIn(usb_device_handle handle,
                                    usb_device_endpoint_callback_message_struct_t *message,
                                    void *callbackParam)
{
    usb_status_t error = kStatus_USB_Success;
    size_t bytesToSend = 0;

    if ((message->length != 0) && (!(message->length % s_usbBulkMaxPacketSize)))
    {
        /* If the last packet is the size of endpoint, then send also zero-ended packet,
         ** meaning that we want to inform the host that we do not have any additional
         ** data, so it can flush the output.
         */
        USB_DeviceSendRequest(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
    }
    else if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
    {
        if ((message->buffer != NULL) || ((message->buffer == NULL) && (message->length == 0)))
        {
            /* User: add your own code for send complete event */
            if (s_usbHandle.txDataSize > 64)
            {
                bytesToSend = 64;
            }
            else if (s_usbHandle.txDataSize == 0)
            {
                s_usbHandle.usb_tx_callback();
                bytesToSend = 0;
            }
            else
            {
                bytesToSend = s_usbHandle.txDataSize;
            }
            /* There is more data to send */
            if (bytesToSend)
            {
                memcpy(s_currSendBuf, s_usbHandle.txData, bytesToSend);
                error = USB_DeviceSendRequest(s_cdcVcom.deviceHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf,
                                              bytesToSend);
                if (error != kStatus_USB_Success)
                {
                    /* Failure to send Data Handling code here */
                    s_usbHandle.usb_tx_callback();
                }
                s_usbHandle.txDataSize -= bytesToSend;
                s_usbHandle.txData += bytesToSend;
            }
        }
    }
    else
    {
    }
    return error;
}

/*!
 * @brief Bulk out pipe callback function.
 *
 * This function serves as the callback function for bulk out pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmBulkOut(usb_device_handle handle,
                                     usb_device_endpoint_callback_message_struct_t *message,
                                     void *callbackParam)
{
    usb_status_t error = kStatus_USB_Error;

    if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
    {
        s_recvSize = message->length;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
        s_waitForDataReceive = 0;
        USB0->INTEN |= USB_INTEN_SOFTOKEN_MASK;
#endif
        if (!s_recvSize)
        {
            /* Schedule buffer for next receive event */
            USB_DeviceRecvRequest(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf, s_usbBulkMaxPacketSize);
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
            s_waitForDataReceive = 1;
            USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
        }
        else
        {
            ADAPTER_UsbReceiveData(handle);
            s_recvSize = 0;
        }
    }
    return error;
}

/*!
 * @brief Get the setup packet buffer.
 *
 * This function provides the buffer for setup packet.
 *
 * @param handle The USB device handle.
 * @param setupBuffer The pointer to the address of setup packet buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    static uint32_t cdcVcomSetup[2];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *)&cdcVcomSetup;
    return kStatus_USB_Success;
}

/*!
 * @brief Get the setup packet data buffer.
 *
 * This function gets the data buffer for setup packet.
 *
 * @param handle The USB device handle.
 * @param setup The pointer to the setup packet.
 * @param length The pointer to the length of the data buffer.
 * @param buffer The pointer to the address of setup packet data buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                             usb_setup_struct_t *setup,
                                             uint32_t *length,
                                             uint8_t **buffer)
{
    static uint8_t setupOut[8];
    if ((NULL == buffer) || ((*length) > sizeof(setupOut)))
    {
        return kStatus_USB_InvalidRequest;
    }
    *buffer = setupOut;
    return kStatus_USB_Success;
}

/*!
 * @brief Configure remote wakeup feature.
 *
 * This function configures the remote wakeup feature.
 *
 * @param handle The USB device handle.
 * @param enable 1: enable, 0: disable.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    return kStatus_USB_InvalidRequest;
}

/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle The USB device handle.
 * @param setup The pointer to the setup packet.
 * @param length The pointer to the length of the data buffer.
 * @param buffer The pointer to the address of setup packet data buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
    uint32_t len;
    uint16_t *uartBitmap;
    if (setup->wIndex != USB_CDC_VCOM_COMM_INTERFACE_INDEX)
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
            break;
        case USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
            break;
        case USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == setup->wValue)
            {
                *buffer = s_abstractState;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == setup->wValue)
            {
                *buffer = s_countryCode;
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == setup->wValue)
            {
                *buffer = s_abstractState;
                *length = COMM_FEATURE_DATA_SIZE;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == setup->wValue)
            {
                *buffer = s_countryCode;
                *length = COMM_FEATURE_DATA_SIZE;
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE:
            break;
        case USB_DEVICE_CDC_REQUEST_GET_LINE_CODING:
            *buffer = s_lineCoding;
            *length = LINE_CODING_SIZE;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_CDC_REQUEST_SET_LINE_CODING:
            *buffer = s_lineCoding;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE:
        {
            acmInfo->dteStatus = setup->wValue;
            /* activate/deactivate Tx carrier */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }

            /* activate carrier and DTE */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }

            /* Indicates to DCE if DTE is present or not */
            acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

            /* Initialize the serial state buffer */
            acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                        /* bmRequestType */
            acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_REQUEST_SERIAL_STATE_NOTIF; /* bNotification */
            acmInfo->serialStateBuf[2] = 0x00;                                      /* wValue */
            acmInfo->serialStateBuf[3] = 0x00;
            acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
            acmInfo->serialStateBuf[5] = 0x00;
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
            acmInfo->serialStateBuf[7] = 0x00;
            /* Notifiy to host the line state */
            acmInfo->serialStateBuf[4] = setup->wIndex;
            /* Lower byte of UART BITMAP */
            uartBitmap = (uint16_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            *uartBitmap = acmInfo->uartState;
            len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (0 == s_cdcVcom.hasSentState)
            {
                error = USB_DeviceSendRequest(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
                if (kStatus_USB_Success != error)
                {
                    usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
                }
                s_cdcVcom.hasSentState = 1;
            }
            /* Update status */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                /*    To do: CARRIER_ACTIVATED */
            }
            else
            {
                /* To do: CARRIER_DEACTIVATED */
            }
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                /* DTE_ACTIVATED */
                if (1 == s_cdcVcom.attach)
                {
                    s_cdcVcom.startTransactions = 1;
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
                    s_waitForDataReceive = 1;
                    USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
                    s_comOpen = 1;
                    usb_echo("USB_APP_CDC_DTE_ACTIVATED\r\n");
#endif
                }
            }
            else
            {
                /* DTE_DEACTIVATED */
                if (1 == s_cdcVcom.attach)
                {
                    s_cdcVcom.startTransactions = 0;
                }
            }
        }
        break;
        case USB_DEVICE_CDC_REQUEST_SEND_BREAK:
            break;
        default:
            break;
    }

    return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint8_t *temp8 = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            USB_DeviceControlPipeInit(s_cdcVcom.deviceHandle);
            s_cdcVcom.attach = 0;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success ==
                USB_DeviceGetStatus(s_cdcVcom.deviceHandle, kUSB_DeviceStatusSpeed, &s_cdcVcom.speed))
            {
                USB_DeviceSetSpeed(handle, s_cdcVcom.speed);
            }
#endif
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (param)
            {
                s_cdcVcom.attach = 1;
                s_cdcVcom.currentConfiguration = *temp8;
                if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
                {
                    usb_device_endpoint_init_struct_t epInitStruct;
                    usb_device_endpoint_callback_struct_t endpointCallback;

                    /* Initiailize endpoint for interrupt pipe */
                    endpointCallback.callbackFn = USB_DeviceCdcAcmInterruptIn;
                    endpointCallback.callbackParam = handle;

                    epInitStruct.zlt = 0;
                    epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
                    epInitStruct.endpointAddress = USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT |
                                                   (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                    if (USB_SPEED_HIGH == s_cdcVcom.speed)
                    {
                        epInitStruct.maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
                    }
                    else
                    {
                        epInitStruct.maxPacketSize = FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
                    }

                    USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &endpointCallback);

                    /* Initiailize endpoints for bulk pipe */
                    endpointCallback.callbackFn = USB_DeviceCdcAcmBulkIn;
                    endpointCallback.callbackParam = handle;

                    epInitStruct.zlt = 0;
                    epInitStruct.transferType = USB_ENDPOINT_BULK;
                    epInitStruct.endpointAddress =
                        USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                    if (USB_SPEED_HIGH == s_cdcVcom.speed)
                    {
                        epInitStruct.maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
                    }
                    else
                    {
                        epInitStruct.maxPacketSize = FS_CDC_VCOM_BULK_IN_PACKET_SIZE;
                    }

                    USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &endpointCallback);

                    endpointCallback.callbackFn = USB_DeviceCdcAcmBulkOut;
                    endpointCallback.callbackParam = handle;

                    epInitStruct.zlt = 0;
                    epInitStruct.transferType = USB_ENDPOINT_BULK;
                    epInitStruct.endpointAddress =
                        USB_CDC_VCOM_BULK_OUT_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                    if (USB_SPEED_HIGH == s_cdcVcom.speed)
                    {
                        epInitStruct.maxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }
                    else
                    {
                        epInitStruct.maxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }

                    USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &endpointCallback);

                    if (USB_SPEED_HIGH == s_cdcVcom.speed)
                    {
                        s_usbBulkMaxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }
                    else
                    {
                        s_usbBulkMaxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }
                    /* Schedule buffer for receive */
                    USB_DeviceRecvRequest(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                          s_usbBulkMaxPacketSize);
                }
            }
            break;
        default:
            break;
    }

    return error;
}

/*!
 * @brief USB configure endpoint function.
 *
 * This function configure endpoint status.
 *
 * @param handle The USB device handle.
 * @param ep Endpoint address.
 * @param status A flag to indicate whether to stall the endpoint. 1: stall, 0: unstall.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    if (status)
    {
        return USB_DeviceStallEndpoint(handle, ep);
    }
    else
    {
        return USB_DeviceUnstallEndpoint(handle, ep);
    }
}

/*!
 * @brief USB Interrupt service routine.
 *
 * This function serves as the USB interrupt service routine.
 *
 * @return None.
 */
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
void USBHS_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(s_cdcVcom.deviceHandle);
}
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(s_cdcVcom.deviceHandle);
}
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
void USB0_IRQHandler(void)
{
    USB_DeviceLpcIp3511IsrFunction(s_cdcVcom.deviceHandle);
}
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
void USB1_IRQHandler(void)
{
    USB_DeviceLpcIp3511IsrFunction(s_cdcVcom.deviceHandle);
}
#endif

void ADAPTER_UsbInit(void)
{
    uint8_t irqNo = USB0_IRQn;

#if defined(CFG_QN908XA)
    POWER_DisablePD(kPDRUNCFG_PD_VREG_A);
#endif

    /* Turn on USB PLL */
    POWER_DisablePD(kPDRUNCFG_PD_USBPLL);

    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 0);
    CLOCK_EnableClock(kCLOCK_Cal);
    CLOCK_EnableUsbfs0DeviceClock(kCLOCK_UsbSrcFro, 0);

    /* Disable USB PHY standby */
    SYSCON->USB_CFG = ((1 << SYSCON_USB_CFG_DPPUEN_B_PHY_POL_SHIFT) | (1 << SYSCON_USB_CFG_DPPUEN_B_PHY_SEL_SHIFT) |
                       (1 << SYSCON_USB_CFG_USB_VBUS_SHIFT) | (0 << SYSCON_USB_CFG_USB_PHYSTDBY_SHIFT) |
                       (0 << SYSCON_USB_CFG_USB_PHYSTDBY_WEN_SHIFT));
    SYSCON->PMU_CTRL1 &= ~(SYSCON_PMU_CTRL1_XTAL_DIS(1));
    SYSCON->PMU_CTRL0 &= ~(SYSCON_PMU_CTRL0_FSP_DIS(1));
    SYSCON->MISC &= ~(1 << SYSCON_PIO_CFG_MISC_TRX_EN_INV_SHIFT);

    /*Calibration USB 48M PLL*/
    CALIB_CalibPLL48M();

    s_cdcVcom.speed = USB_SPEED_FULL;
    s_cdcVcom.attach = 0;
    s_cdcVcom.deviceHandle = NULL;

    if (kStatus_USB_Success != USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &s_cdcVcom.deviceHandle))
    {
        usb_echo("USB device vcom failed\r\n");
        return;
    }
    else
    {
        usb_echo("USB device CDC virtual com demo\r\n");
    }
    NVIC_SetPriority((IRQn_Type)irqNo, USB_DEVICE_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)irqNo);

    USB_DeviceRun(s_cdcVcom.deviceHandle);
}

void ADAPTER_UsbTransmitInt(uint8_t *buf, uint32_t size, void (*tx_callback)(void))
{
    size_t bytesToSend = 0;
    if ((1 != s_cdcVcom.attach) || (1 != s_cdcVcom.startTransactions))
    {
        tx_callback();
        return;
    }
    s_usbHandle.usb_tx_callback = tx_callback;
    if (size <= 64)
    {
        s_usbHandle.txData = buf;
        s_usbHandle.txDataSize = 0;
        bytesToSend = size;
    }
    else
    {
        s_usbHandle.txData = buf + 64;
        s_usbHandle.txDataSize = size - 64;
        bytesToSend = 64;
    }
    /* Copy the send data to USB send buffer */
    memcpy(s_currSendBuf, buf, bytesToSend);
    /* Send the data */
    USB_DeviceSendRequest(s_cdcVcom.deviceHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, bytesToSend);
}

void ADAPTER_UsbReceiveInt(uint8_t *buf, uint32_t size, void (*rx_callback)(void))
{
    size_t bytesAvailable = s_recvSize - s_validDataStart;
    size_t bytesToCopy = 0;
    s_usbHandle.usb_rx_callback = rx_callback;

    s_usbHandle.rxData = buf;
    if (bytesAvailable >= size)
    {
        s_usbHandle.rxDataSize = 0;
        bytesToCopy = size;
    }
    else if (bytesAvailable > 0)
    {
        s_usbHandle.rxDataSize = size - bytesAvailable;
        bytesToCopy = bytesAvailable;
    }
    else
    {
        s_usbHandle.rxDataSize = size;
    }

    if (bytesToCopy)
    {
        /* Copy Data from USB receive buffer to the receive buffer of BLE stack*/
        for (int i = 0; i < bytesToCopy; i++)
        {
            s_usbHandle.rxData[i] = s_currRecvBuf[s_validDataStart++];
        }
        s_usbHandle.rxData = buf + bytesToCopy;
        if (bytesToCopy == size)
        {
            rx_callback();
        }
    }
}
