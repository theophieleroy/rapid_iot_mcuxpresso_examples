/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2009 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file usb_descriptor.h
 *
 * @author
 *
 * @version
 *
 * @date May-28-2009
 *
 * @brief The file is a header file for USB Descriptors required for Mouse
 *        Application
 *****************************************************************************/

#ifndef _USB_DESCRIPTOR_H
#define _USB_DESCRIPTOR_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "types.h"
#include "usb_types.h"
#include "usb_class.h"
#include "user_config.h"
#include "usb_class_hid.h"

/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
#define BCD_USB_VERSION                     (0x0200)
   
#define REMOTE_WAKEUP_SHIFT                 (5)
#define REMOTE_WAKEUP_SUPPORT               (TRUE)
/* Various descriptor sizes */
#define DEVICE_DESCRIPTOR_SIZE              (18)
#define CONFIG_DESC_SIZE                    (41)
#define DEVICE_QUALIFIER_DESCRIPTOR_SIZE    (10)
#define REPORT_DESC_SIZE                    (76)
#define CONFIG_ONLY_DESC_SIZE               (9)
#define IFACE_ONLY_DESC_SIZE                (9)
#define HID_ONLY_DESC_SIZE                  (9)
#define ENDP_ONLY_DESC_SIZE                 (7)
#define OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE  (CONFIG_DESC_SIZE)  

/* HID buffer size */
#define HID_BUFFER_SIZE                     (8)
/* Max descriptors provided by the Application */
#define USB_MAX_STD_DESCRIPTORS             (8)
#define USB_MAX_CLASS_SPECIFIC_DESCRIPTORS  (2)
/* Max configuration supported by the Application */
#define USB_MAX_CONFIG_SUPPORTED            (1)

/* Max string descriptors supported by the Application */
#define USB_MAX_STRING_DESCRIPTORS          (3)

/* Max language codes supported by the USB */
#define USB_MAX_LANGUAGES_SUPPORTED         (1)
#define HID_DESC_ENDPOINT_COUNT             (2)
#define HID_IN_ENDPOINT                     (1)
#define HID_OUT_ENDPOINT                    (2)
#define HID_ENDPOINT                        HID_IN_ENDPOINT
#define HID_ENDPOINT_PACKET_SIZE            (64)
/* string descriptors sizes */
#define USB_STR_DESC_SIZE                   (2)
#define USB_STR_0_SIZE                      (2)
#define USB_STR_1_SIZE                      (56)
#define USB_STR_2_SIZE                      (36)
#define USB_STR_n_SIZE                      (32)
/* descriptors codes */
#define USB_DEVICE_DESCRIPTOR               (1)
#define USB_CONFIG_DESCRIPTOR               (2)
#define USB_STRING_DESCRIPTOR               (3)
#define USB_IFACE_DESCRIPTOR                (4)
#define USB_ENDPOINT_DESCRIPTOR             (5)
#define USB_DEVQUAL_DESCRIPTOR              (6)
#define USB_OTHER_SPEED_DESCRIPTOR          (7)
#define USB_HID_DESCRIPTOR                  (0x21)
#define USB_REPORT_DESCRIPTOR               (0x22)

#define USB_MAX_SUPPORTED_ENDPOINTS         (8)
#define USB_MAX_SUPPORTED_INTERFACES        (1)

// New added for K65
#define DEVICE_DESC_DEVICE_CLASS                (0x00)
#define DEVICE_DESC_DEVICE_SUBCLASS             (0x00)
#define DEVICE_DESC_DEVICE_PROTOCOL             (0x00)
#define DEVICE_DESC_NUM_CONFIG_SUPPOTED         (0x01)
/* Keep the following macro Zero if you dont Support Other Speed Configuration
   If you suppoort Other Speeds make it 0x01 */
#define DEVICE_OTHER_DESC_NUM_CONFIG_SUPPOTED   (0x00) 
#define CONFIG_DESC_NUM_INTERFACES_SUPPOTED     (0x01)
#define CONFIG_DESC_CURRENT_DRAWN               (0x32)
#define USB_MAX_SUPPORTED_LANGUAGES             (1)
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum _usb_descriptor_index {
    kUsbDescriptorIndex_VidLow = 8,
    kUsbDescriptorIndex_VidHigh = 9,
    kUsbDescriptorIndex_PidLow = 10,
    kUsbDescriptorIndex_PidHigh = 11
};

/******************************************************************************
 * Types
 *****************************************************************************/
typedef const struct _USB_LANGUAGE
{
    uint16_t const language_id;      /* Language ID */
    uint8_t const ** lang_desc;      /* Language Descriptor String */
    uint8_t const * lang_desc_size;  /* Language Descriptor Size */
} USB_LANGUAGE;


typedef const struct _USB_ALL_LANGUAGES
{
    /* Pointer to Supported Language String */
    uint8_t const *languages_supported_string;
    /* Size of Supported Language String */
    uint8_t const languages_supported_size;
    /* Array of Supported Languages */
    USB_LANGUAGE usb_language[USB_MAX_SUPPORTED_INTERFACES];
}USB_ALL_LANGUAGES;

/******************************************************************************
 * Global Functions
 *****************************************************************************/
extern uint8_t USB_Desc_Get_Descriptor(
     hid_handle_t handle,
     uint8_t type,
     uint8_t str_num,
     uint16_t index,
     uint8_t * *descriptor,
     uint32_t *size);

extern uint8_t USB_Desc_Get_Interface(
    hid_handle_t handle,
    uint8_t interface,
    uint8_t * alt_interface);


extern uint8_t USB_Desc_Set_Interface(
    hid_handle_t handle,
    uint8_t interface,
    uint8_t alt_interface);

extern bool USB_Desc_Remote_Wakeup(hid_handle_t handle);

extern usb_endpoints_t *USB_Desc_Get_Endpoints(hid_handle_t handle); 

extern uint8_t g_device_descriptor[];

extern USB_ALL_LANGUAGES g_languages;
extern USB_ALL_LANGUAGES * g_lang_ptr;

#endif
