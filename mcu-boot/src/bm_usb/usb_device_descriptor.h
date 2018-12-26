/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
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
 * o Neither the name the copyright holder nor the names of its
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

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_DFU_HID_COMPOSITE_APP (1)
#define USB_BCD_VERSION (0x0200U)
#define USB_BCD_VERSION_DFU (0x0100U)
#define USB_APP_BCD_VERSION (0x0101U)

#define USB_DEVICE_CLASS (0x00U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)
#define USB_DEVICE_CONFIGURATION_COUNT (1U)
#define USB_DEVICE_STRING_COUNT (5U)
#define USB_DEVICE_LANGUAGE_COUNT (1U)
#define USB_COMPOSITE_CONFIGURE_INDEX (1U)

#define USB_DEVICE_MAX_POWER (0x32U)
#define USB_CONFIGURE_DRAWN (0x32)

// Defines the length of g_usb_str_x[]
////////////////////////////////////////////////////////////////////////////////
#define USB_DESCRIPTOR_LENGTH_FUNCTINAL (9U)
#define USB_DESCRIPTOR_LENGTH_STRING0 (sizeof(g_usb_str_0))
#define USB_DESCRIPTOR_LENGTH_STRING1 (sizeof(g_usb_str_1))
#define USB_DESCRIPTOR_LENGTH_STRING2 (sizeof(g_usb_str_2))
#define USB_DESCRIPTOR_LENGTH_STRING3 (sizeof(g_usb_str_3))
#define USB_DESCRIPTOR_LENGTH_STRING4 (sizeof(g_usb_str_4))

// DFU
////////////////////////////////////////////////////////////////////////////////
#define USB_DESCRIPTOR_TYPE_DFU_FUNCTIONAL (0x21)
#define USB_DFU_DETACH_TIMEOUT (0x6400)
#define USB_DFU_BIT_WILL_DETACH (1U)
#define USB_DFU_BIT_MANIFESTATION_TOLERANT (0U)
#define USB_DFU_BIT_CAN_UPLOAD (0U)
#define USB_DFU_BIT_CAN_DNLOAD (1U)
#define MAX_TRANSFER_SIZE (0x200)
#define USB_DFU_INTERFACE_COUNT (1U)
#define USB_DFU_CONFIGURE_INDEX (1U)
#define USB_DFU_CLASS (0xFEU)
#define USB_DFU_SUBCLASS (0x01U)
#define USB_DFU_PROTOCOL (0x01U)
#define USB_DFU_MODE_PROTOCOL (0x02U)

// HID
////////////////////////////////////////////////////////////////////////////////
#define USB_HID_REPORT_DESC_SIZE (sizeof(g_hid_generic_report_descriptor))
#define USB_HID_GENERIC_DESCRIPTOR_LENGTH (32)
#define USB_HID_DESCRIPTOR_LENGTH (9)
#define USB_IAD_DESC_SIZE (8)
#define USB_HID_GENERIC_INTERFACE_COUNT (1)
#define USB_HID_GENERIC_CONFIGURE_INDEX (1)
#define USB_HID_GENERIC_IN_BUFFER_LENGTH (8)
#define USB_HID_GENERIC_OUT_BUFFER_LENGTH (8)
#define USB_HID_GENERIC_ENDPOINT_COUNT (2)
#define USB_HID_GENERIC_ENDPOINT_IN (1)
#define USB_HID_GENERIC_ENDPOINT_OUT (2)
#define USB_HID_GENERIC_CLASS (0x03)
#define USB_HID_GENERIC_SUBCLASS (0x00)
#define USB_HID_GENERIC_PROTOCOL (0x00)
#define HS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE (64)
#define FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE (64)
#define HS_HID_GENERIC_INTERRUPT_OUT_INTERVAL (0x04) /* 2^(4-1) = 1ms */
#define FS_HID_GENERIC_INTERRUPT_OUT_INTERVAL (0x01)
#define HS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE (64)
#define FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE (64)
#define HS_HID_GENERIC_INTERRUPT_IN_INTERVAL (0x04) /* 2^(4-1) = 1ms */
#define FS_HID_GENERIC_INTERRUPT_IN_INTERVAL (0x01)
#define BL_CONFIG_REPORT_SIZE_MULTIPLER (5)

// MSC
////////////////////////////////////////////////////////////////////////////////
#define USB_MSC_CLASS (0x08)
#define USB_MSC_SUBCLASS (0x06)
#define USB_MSC_PROTOCOL (0x50)
#define HS_BULK_IN_PACKET_SIZE (512)
#define HS_BULK_OUT_PACKET_SIZE (512)
#define FS_BULK_IN_PACKET_SIZE (64)
#define FS_BULK_OUT_PACKET_SIZE (64)
#define USB_MSC_CONFIGURE_INDEX (1)
#define USB_MSC_ENDPOINT_COUNT (2)
#define USB_MSC_BULK_IN_ENDPOINT (3)
#define USB_MSC_BULK_OUT_ENDPOINT (4)
#define USB_MSC_INTERFACE_COUNT (1)

// Defines the length of g_config_descriptor[] and g_config_descriptor_dfu[]
////////////////////////////////////////////////////////////////////////////////
#if (USB_DEVICE_CONFIG_HID == 1)
#define USB_DESCRIPTOR_LENGTH_HID_PART    (USB_DESCRIPTOR_LENGTH_FUNCTINAL + \
                                           USB_HID_DESCRIPTOR_LENGTH + \
                                           2 * USB_DESCRIPTOR_LENGTH_ENDPOINT)
#else
#define USB_DESCRIPTOR_LENGTH_HID_PART    (0)
#endif
#if (USB_DEVICE_CONFIG_DFU == 1)
#define USB_DESCRIPTOR_LENGTH_DFU_PART    (USB_DESCRIPTOR_LENGTH_INTERFACE + \
                                           USB_DESCRIPTOR_LENGTH_FUNCTINAL)
#else
#define USB_DESCRIPTOR_LENGTH_DFU_PART    (0)
#endif
#if (USB_DEVICE_CONFIG_MSC == 1)
#define USB_DESCRIPTOR_LENGTH_MSC_PART    (USB_DESCRIPTOR_LENGTH_FUNCTINAL + \
                                           2* USB_DESCRIPTOR_LENGTH_ENDPOINT)
#else
#define USB_DESCRIPTOR_LENGTH_MSC_PART    (0)
#endif

#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL     (USB_DESCRIPTOR_LENGTH_CONFIGURE + \
                                                     USB_DESCRIPTOR_LENGTH_DFU_PART + \
                                                     USB_DESCRIPTOR_LENGTH_HID_PART + \
                                                     USB_DESCRIPTOR_LENGTH_MSC_PART)
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU (USB_DESCRIPTOR_LENGTH_CONFIGURE + \
                                                     USB_DESCRIPTOR_LENGTH_DFU_PART)


// Defines the configuration index of each class in g_config_descriptor[]
////////////////////////////////////////////////////////////////////////////////
#define USB_DFU_CONFIG_INDEX     (USB_DESCRIPTOR_LENGTH_CONFIGURE)
#define USB_HID_CONFIG_INDEX     (USB_DFU_CONFIG_INDEX + \
                                  USB_DESCRIPTOR_LENGTH_DFU_PART)
#define USB_MSC_CONFIG_INDEX     (USB_HID_CONFIG_INDEX + \
                                  USB_DESCRIPTOR_LENGTH_HID_PART)

// Defines the interface index and count of each class in g_composite_device[]
////////////////////////////////////////////////////////////////////////////////
#if (USB_DEVICE_CONFIG_HID == 1) && (USB_DEVICE_CONFIG_DFU == 1) && (USB_DEVICE_CONFIG_MSC == 0)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT + USB_DFU_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX         (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (1)
#define USB_MSC_INTERFACE_INDEX         (0)
#elif (USB_DEVICE_CONFIG_HID == 1) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 1)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT + USB_MSC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX         (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX         (1)
#elif (USB_DEVICE_CONFIG_HID == 1) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 0)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX         (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX         (0)
#else
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX         (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX         (0)
#endif

enum _usb_descriptor_index
{
    kUsbDescriptorIndex_VidLow = 8,
    kUsbDescriptorIndex_VidHigh = 9,
    kUsbDescriptorIndex_PidLow = 10,
    kUsbDescriptorIndex_PidHigh = 11
};

typedef struct _usb_hid_config_descriptor
{
    usb_descriptor_interface_t interface;   /* Interface descriptor */
    usb_descriptor_interface_t hid_report;  /* Interface descriptor */
    usb_descriptor_endpoint_t endpoint_in;  /* Endpoint descriptor */
    usb_descriptor_endpoint_t endpoint_out; /* Endpoint descriptor */
} usb_hid_config_descriptor_t;

typedef struct _usb_msc_config_descriptor
{
    usb_descriptor_interface_t interface;   /* Interface descriptor */
    usb_descriptor_endpoint_t endpoint_in;  /* Endpoint descriptor */
    usb_descriptor_endpoint_t endpoint_out; /* Endpoint descriptor */
} usb_msc_config_descriptor_t;

extern usb_device_class_struct_t g_hid_generic_class;
extern usb_device_class_struct_t g_msc_class;
extern usb_device_class_struct_t g_dfu_class;

extern uint8_t g_device_descriptor[];
extern usb_language_list_t g_language_list;
extern usb_language_list_t *g_language_ptr;

/*******************************************************************************
 * API
 ******************************************************************************/
/* Get device descriptor request */
usb_status_t usb_device_get_device_descriptor(usb_device_handle handle,
                                              usb_device_get_device_descriptor_struct_t *device_descriptor);

/* Get device configuration descriptor request */
usb_status_t usb_device_get_configuration_descriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configuration_descriptor);

/* Get device string descriptor request */
usb_status_t usb_device_get_string_descriptor(usb_device_handle handle,
                                              usb_device_get_string_descriptor_struct_t *string_descriptor);

/* Configure the device according to the USB speed. */
extern usb_status_t usb_device_set_speed(usb_device_handle handle, uint8_t speed);

/* Get hid descriptor request */
usb_status_t usb_device_get_hid_descriptor(usb_device_handle handle,
                                           usb_device_get_hid_descriptor_struct_t *hid_descriptor);

/* Get hid report descriptor request */
usb_status_t usb_device_get_hid_report_descriptor(usb_device_handle handle,
                                                  usb_device_get_hid_report_descriptor_struct_t *hid_report_descriptor);

/* Get hid physical descriptor request */
usb_status_t usb_device_get_hid_physical_descriptor(
    usb_device_handle handle, usb_device_get_hid_physical_descriptor_struct_t *hid_physical_descriptor);

#endif /* __USB_DEVICE_DESCRIPTOR_H__ */
