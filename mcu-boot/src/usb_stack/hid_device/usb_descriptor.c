/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2013 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file usb_descriptor.c
 *
 * @author
 *
 * @version
 *
 * @date May-28-2009
 *
 * @brief The file contains USB descriptors
 *
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"
#include "usb_class_hid.h"
#include "usb_descriptor.h"
#include "soc.h"
#include "types.h"

#include "bootloader_hid_report_ids.h"
#include "bootloader/bl_peripheral.h"
#include "property/property.h"
#include "bootloader/bl_context.h"

#if (defined __MCF52xxx_H__) || (defined __MK_xxx_H__)
/* Put CFV2 descriptors in RAM */
#define USB_DESC_CONST
#else
#define USB_DESC_CONST	const
#endif

/*****************************************************************************
 * Constant and Macro's
 *****************************************************************************/


#define BL_MIN_PACKET_SIZE (32)
#define BL_PACKET_SIZE_HEADER_SIZE (3)  // alignment byte + length lsb + length msb (does not include report id)
#define BL_REPORT_SIZE (BL_MIN_PACKET_SIZE + BL_PACKET_SIZE_HEADER_SIZE)


	/* hidtc data buffer out report descriptor */
#define HID_USAGE_HIDTC_DATA_OUT(__id, __count, __size)       \
	  0x85, ((uint8_t)(__id)),		/*	 REPORT_ID (__id) */	  \
	  0x19, 0x01,					 /*   USAGE_MINIMUM (1)*/	   \
	  0x29, 0x01,					 /*   USAGE_MAXIMUM (1)*/	   \
	  0x15, 0x00,					 /*   LOGICAL_MINIMUM (0)*/    \
	  0x26, 0xff, 0x00, 			 /*   LOGICAL_MAXIMUM (255)*/  \
	  0x75, ((uint8_t)(__size)), 	/*	 REPORT_SIZE (n)*/		  \
	  0x95, ((uint8_t)(__count)),	/*	 REPORT_COUNT (n)*/ 	  \
	  0x91, 0x02					 /*   OUTPUT (Data,Var,Abs) */

	/* hidtc data buffer in report descriptor */
#define HID_USAGE_HIDTC_DATA_IN(__id, __count, __size)        \
	  0x85, ((uint8_t)(__id)),		/*	 REPORT_ID (__id) */	  \
	  0x19, 0x01,					 /*   USAGE_MINIMUM (1)*/	   \
	  0x29, 0x01,					 /*   USAGE_MAXIMUM (1)*/	   \
	  0x15, 0x00,					 /*   LOGICAL_MINIMUM (0)*/    \
	  0x26, 0xff, 0x00, 			 /*   LOGICAL_MAXIMUM (255)*/  \
	  0x75, ((uint8_t)(__size)), 	/*	 REPORT_SIZE (n)*/		  \
	  0x95, ((uint8_t)(__count)),	/*	 REPORT_COUNT (n)*/ 	  \
	  0x81, 0x02					 /*   INPUT (Data,Var,Abs) */


usb_ep_struct_t g_ep[HID_DESC_ENDPOINT_COUNT] =
{
    {HID_IN_ENDPOINT,
     USB_INTERRUPT_PIPE,
     USB_SEND,
     HID_ENDPOINT_PACKET_SIZE,
     },
    {HID_OUT_ENDPOINT,
     USB_INTERRUPT_PIPE,
     USB_RECV,
     HID_ENDPOINT_PACKET_SIZE,
     }
};

/* structure containing details of all the endpoints used by this device */
USB_DESC_CONST usb_endpoints_t usb_desc_ep =
{
    HID_DESC_ENDPOINT_COUNT,
    g_ep
};

static usb_if_struct_t g_usb_if[1] ;

usb_class_struct_t g_usb_dec_class =
{
   USB_CLASS_HID,
   {
        1,
        g_usb_if
    }
};


uint8_t USB_DESC_CONST g_device_descriptor[DEVICE_DESCRIPTOR_SIZE] =
{
   DEVICE_DESCRIPTOR_SIZE,               /* "Device Descriptor Size        */
   USB_DEVICE_DESCRIPTOR,                /* "Device" Type of descriptor    */
   0x00, 0x02,                           /*  BCD USB version               */
   DEVICE_DESC_DEVICE_CLASS,             /*  Device Class is indicated in
                                             the interface descriptors     */
   DEVICE_DESC_DEVICE_SUBCLASS,          /*  Device Subclass is indicated
                                             in the interface descriptors  */
   DEVICE_DESC_DEVICE_PROTOCOL,          /*  Device Protocol               */
   CONTROL_MAX_PACKET_SIZE,              /*  Max Packet size               */
   0xA2,0x15,                            /*  Vendor ID for Freescale       */
   0x73,0x00,                            /* 0x73,0x00,ProductID for KL25Z48M */
   0x02,0x00,                            /*  BCD Device version            */
   0x01,                                 /*  Manufacturer string index     */
   0x02,                                 /*  Product string index          */
   0x00,                                 /*  Serial number string index    */
   DEVICE_DESC_NUM_CONFIG_SUPPOTED       /*  Number of configurations      */
};

uint8_t USB_DESC_CONST g_config_descriptor[CONFIG_DESC_SIZE] =
{
   CONFIG_ONLY_DESC_SIZE,  /*  Configuration Descriptor Size - always 9 bytes*/
   USB_CONFIG_DESCRIPTOR,  /* "Configuration" type of descriptor */
   CONFIG_DESC_SIZE, 0x00, /*  Total length of the Configuration descriptor */
   CONFIG_DESC_NUM_INTERFACES_SUPPOTED,     /*  NumInterfaces */
   1,                      /*  Configuration Value */
   0,                      /*  Configuration Description String Index*/
   (USB_DESC_CFG_ATTRIBUTES_D7_POS) | (USBCFG_DEV_SELF_POWER << USB_DESC_CFG_ATTRIBUTES_SELF_POWERED_SHIFT) | (USBCFG_DEV_REMOTE_WAKEUP << USB_DESC_CFG_ATTRIBUTES_REMOTE_WAKEUP_SHIFT),
   /* S08/CFv1 are both self powered (its compulsory to set bus powered)*/
   /*Attributes.support RemoteWakeup and self power*/
   CONFIG_DESC_CURRENT_DRAWN,              /*  Current draw from bus */

   /* Interface Descriptor */
   IFACE_ONLY_DESC_SIZE,
   USB_IFACE_DESCRIPTOR,
   0x00,
   0x00,
   HID_DESC_ENDPOINT_COUNT,
   0x03,
   0x00,  
   0x00,  /* protocol None*/
   0x00,

   /* HID descriptor */
   HID_ONLY_DESC_SIZE,
   USB_HID_DESCRIPTOR,
   0x00,0x01,
   0x00,
   0x01,
   0x22,/*  hid report        */

   (REPORT_DESC_SIZE & 0x00ff),		 /* report_desc_length_l */
   (REPORT_DESC_SIZE & 0xff00) >> 8, /* report_desc_length_h */

   /*IN Endpoint descriptor */
   ENDP_ONLY_DESC_SIZE,
   USB_ENDPOINT_DESCRIPTOR,
   HID_IN_ENDPOINT|(USB_SEND << 7),
   USB_INTERRUPT_PIPE,
   HID_ENDPOINT_PACKET_SIZE, 0x00,
   0x0A,

   /* OUT Endpoint descriptor */
   ENDP_ONLY_DESC_SIZE,
   USB_ENDPOINT_DESCRIPTOR,
   HID_OUT_ENDPOINT|(USB_RECV << 7),
   USB_INTERRUPT_PIPE,
   HID_ENDPOINT_PACKET_SIZE, 0x00,
   0x0A
};

uint8_t  g_device_qualifier_descriptor[DEVICE_QUALIFIER_DESCRIPTOR_SIZE] =
{
    /* Device Qualifier Descriptor Size */
    DEVICE_QUALIFIER_DESCRIPTOR_SIZE, 
    /* Type of Descriptor */
    USB_DEVQUAL_DESCRIPTOR,
    /*  BCD USB version  */
    USB_uint_16_low(BCD_USB_VERSION), USB_uint_16_high(BCD_USB_VERSION),
    /* bDeviceClass */
    DEVICE_DESC_DEVICE_CLASS,
    /* bDeviceSubClass */
    DEVICE_DESC_DEVICE_SUBCLASS,
    /* bDeviceProtocol */
    DEVICE_DESC_DEVICE_PROTOCOL,
    /* bMaxPacketSize0 */
    CONTROL_MAX_PACKET_SIZE,
    /* bNumConfigurations */
    DEVICE_OTHER_DESC_NUM_CONFIG_SUPPOTED,  
    /* Reserved : must be zero */ 
    0x00
};

uint8_t  g_other_speed_config_descriptor[OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE] =
{
    /* Length of this descriptor */
    CONFIG_ONLY_DESC_SIZE,     
    /* This is a Other speed config descr */
    USB_OTHER_SPEED_DESCRIPTOR,
    /*  Total length of the Configuration descriptor */
    USB_uint_16_low(CONFIG_DESC_SIZE), USB_uint_16_high(CONFIG_DESC_SIZE),
    CONFIG_DESC_NUM_INTERFACES_SUPPOTED,
    /*value used to selct this configuration : Configuration Value */
    1, 
    /*  Configuration Description String Index*/   
    0, 
    /*  Attributes.support RemoteWakeup and self power */
    (USB_DESC_CFG_ATTRIBUTES_D7_POS) | (USBCFG_DEV_SELF_POWER << USB_DESC_CFG_ATTRIBUTES_SELF_POWERED_SHIFT) | (USBCFG_DEV_REMOTE_WAKEUP << USB_DESC_CFG_ATTRIBUTES_REMOTE_WAKEUP_SHIFT),    
    /*  Current draw from bus */
    CONFIG_DESC_CURRENT_DRAWN, 

    /* Interface Descriptor */
    IFACE_ONLY_DESC_SIZE,
    USB_IFACE_DESCRIPTOR,
    0x00,
    0x00,
    HID_DESC_ENDPOINT_COUNT,
    0x03,
    0x01,
    0x02,
    0x00,

    /* HID descriptor */
    HID_ONLY_DESC_SIZE, 
    USB_HID_DESCRIPTOR,
    0x00,0x01,
    0x00,
    0x01,
    0x22,
    0x34,0x00,
     
    /*Endpoint descriptor */
/*    ENDP_ONLY_DESC_SIZE, 
    USB_ENDPOINT_DESCRIPTOR,
    HID_ENDPOINT|(USB_SEND << 7),
    USB_INTERRUPT_PIPE, 
    HID_ENDPOINT_PACKET_SIZE, 0x00, 
    0x0A
*/      
   /*IN Endpoint descriptor */
   ENDP_ONLY_DESC_SIZE,
   USB_ENDPOINT_DESCRIPTOR,
   HID_IN_ENDPOINT|(USB_SEND << 7),
   USB_INTERRUPT_PIPE,
   HID_ENDPOINT_PACKET_SIZE, 0x00,
   0x0A,

   /* OUT Endpoint descriptor */
   ENDP_ONLY_DESC_SIZE,
   USB_ENDPOINT_DESCRIPTOR,
   HID_OUT_ENDPOINT|(USB_RECV << 7),
   USB_INTERRUPT_PIPE,
   HID_ENDPOINT_PACKET_SIZE, 0x00,
   0x0A        
};

uint8_t USB_DESC_CONST g_report_descriptor[REPORT_DESC_SIZE] =
{
   0x06, 0x00, 0xFF ,  /* Usage Page (Vendor Defined Page 1)*/
   0x09, 0x01,   /* USAGE (Vendor 1) */
   0xA1, 0x01,   /* Collection (Application) */
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_CommandOut, BL_REPORT_SIZE, 8),
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_DataOut, BL_REPORT_SIZE , 8),
    HID_USAGE_HIDTC_DATA_IN (kBootloaderReportID_CommandIn, BL_REPORT_SIZE, 8),
    HID_USAGE_HIDTC_DATA_IN (kBootloaderReportID_DataIn, BL_REPORT_SIZE, 8),
   0xC0          /* end collection */
};

uint8_t USB_DESC_CONST USB_STR_0[USB_STR_0_SIZE+USB_STR_DESC_SIZE] =
                                    {sizeof(USB_STR_0),
                                     USB_STRING_DESCRIPTOR,
                                      0x09,
                                      0x04/*equiavlent to 0x0409*/
                                    };

uint8_t USB_DESC_CONST USB_STR_1[USB_STR_1_SIZE+USB_STR_DESC_SIZE]
                          = {  sizeof(USB_STR_1),
                               USB_STRING_DESCRIPTOR,
                               'F',0,
                               'r',0,
                               'e',0,
                               'e',0,
                               's',0,
                               'c',0,
                               'a',0,
                               'l',0,
                               'e',0,
                               ' ',0,
                               'S',0,
                               'e',0,
                               'm',0,
                               'i',0,
                               'c',0,
                               'o',0,
                               'n',0,
                               'd',0,
                               'u',0,
                               'c',0,
                               't',0,
                               'o',0,
                               'r',0,
                               ' ',0,
                               'I',0,
                               'n',0,
                               'c',0,
                               '.',0
                          };


uint8_t USB_DESC_CONST USB_STR_2[USB_STR_2_SIZE+USB_STR_DESC_SIZE]
                          = {  sizeof(USB_STR_2),
                               USB_STRING_DESCRIPTOR,
                               'K',0,
                               'i',0,
                               'n',0,
                               'e',0,
                               't',0,
                               'i',0,
                               's',0,
                               ' ',0,
                               'B',0,
                               'o',0,
                               'o',0,
                               't',0,
                               'l',0,
                               'o',0,
                               'a',0,
                               'd',0,
                               'e',0,
                               'r',0,
                          };

uint8_t USB_DESC_CONST USB_STR_n[USB_STR_n_SIZE+USB_STR_DESC_SIZE]
                          = {  sizeof(USB_STR_n),
                               USB_STRING_DESCRIPTOR,
                               'B',0,
                               'A',0,
                               'D',0,
                               ' ',0,
                               'S',0,
                               'T',0,
                               'R',0,
                               'I',0,
                               'N',0,
                               'G',0,
                               ' ',0,
                               'I',0,
                               'N',0,
                               'D',0,
                               'E',0,
                               'X',0
                          };


USB_PACKET_SIZE const g_std_desc_size[USB_MAX_STD_DESCRIPTORS+1] =
                                    {0,
                                     DEVICE_DESCRIPTOR_SIZE,
                                     CONFIG_DESC_SIZE,
                                     0, /* string */
                                     0, /* Interface */
                                     0, /* Endpoint */
                                     0, /* Device Qualifier */
                                     0, /* other speed config */
                                     REPORT_DESC_SIZE
                                    };

USB_PACKET_SIZE const g_std_desc_size_hs[USB_MAX_STD_DESCRIPTORS+1] =
                                    {0,
                                     DEVICE_DESCRIPTOR_SIZE,
                                     CONFIG_DESC_SIZE,
                                     0, /* string */
                                     0, /* Interface */
                                     0, /* Endpoint */
                                     DEVICE_QUALIFIER_DESCRIPTOR_SIZE,
                                     OTHER_SPEED_CONFIG_DESCRIPTOR_SIZE,
                                     REPORT_DESC_SIZE
                                    };

uint_8_ptr const g_std_descriptors[USB_MAX_STD_DESCRIPTORS+1] =
                                            {
                                                NULL,
                                                (uint_8_ptr)g_device_descriptor,
                                                (uint_8_ptr)g_config_descriptor,
                                                NULL, /* string */
                                                NULL, /* Interface */
                                                NULL, /* Endpoint */
                                                NULL, /* Device Qualifier */
                                                NULL, /* other speed config*/
                                                (uint_8_ptr)g_report_descriptor
                                            };

uint_8_ptr const g_std_descriptors_hs[USB_MAX_STD_DESCRIPTORS+1] =
                                            {
                                                NULL,
                                                (uint_8_ptr)g_device_descriptor,
                                                (uint_8_ptr)g_config_descriptor,
                                                NULL, /* string */
                                                NULL, /* Interface */
                                                NULL, /* Endpoint */
                                                g_device_qualifier_descriptor,
                                                g_other_speed_config_descriptor,
                                                (uint_8_ptr)g_report_descriptor
                                            };

uint8_t const g_string_desc_size[USB_MAX_STRING_DESCRIPTORS+1] =
                                   {
                                     sizeof(USB_STR_0),
                                     sizeof(USB_STR_1),
                                     sizeof(USB_STR_2),
                                     sizeof(USB_STR_n)
                                    };

uint_8_ptr const g_string_descriptors[USB_MAX_STRING_DESCRIPTORS+1] =
                                  {
                                      (uint_8_ptr) USB_STR_0,
                                      (uint_8_ptr) USB_STR_1,
                                      (uint_8_ptr) USB_STR_2,
                                      (uint_8_ptr) USB_STR_n
                                  };

USB_ALL_LANGUAGES g_languages = { USB_STR_0, sizeof(USB_STR_0),
                                  { (uint16_t)0x0409,
                                   (const uint8_t **)g_string_descriptors,
                                      g_string_desc_size}
                                };


USB_ALL_LANGUAGES * g_lang_ptr;
uint8_t const g_valid_config_values[USB_MAX_CONFIG_SUPPORTED+1]={0,1};

/****************************************************************************
 * Global Variables
 ****************************************************************************/
static uint8_t g_alternate_interface[USB_MAX_SUPPORTED_INTERFACES];

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Local Variables - None
 *****************************************************************************/

 /*****************************************************************************
 * Local Functions - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions
 *****************************************************************************/
extern uint32_t usb_get_instance_via_ipsr(void);

/**************************************************************************//*!
 *
 * @name  USB_Desc_Get_Descriptor
 *
 * @brief The function returns the correponding descriptor
 *
 * @param controller_ID : Controller ID
 * @param type          : Type of descriptor requested
 * @param sub_type      : String index for string descriptor
 * @param index         : String descriptor language Id
 * @param descriptor    : Output descriptor pointer
 * @param size          : Size of descriptor returned
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************
 * This function is used to pass the pointer of the requested descriptor
 *****************************************************************************/
int device_desc_req_count = 0;
uint8_t USB_Desc_Get_Descriptor(
     hid_handle_t handle,     /* [IN]  handle */
     uint8_t type,            /* [IN]  Type of descriptor requested */
     uint8_t str_num,         /* [IN]  String index for string descriptor */
     uint16_t index,          /* [IN]  String descriptor language Id */
     uint_8_ptr *descriptor, /* [OUT] Output descriptor pointer */
     uint32_t *size
)
{

    uint32_t hidInforIndex = usb_get_instance_via_ipsr(); 
    UNUSED (handle)
       
    switch(type)
    {
      case USB_REPORT_DESCRIPTOR:
        {
          type = USB_MAX_STD_DESCRIPTORS;
          if (hidInforIndex == USB_CONTROLLER_KHCI_0)
          {
              *descriptor = (uint_8_ptr)g_std_descriptors[type];
              *size = g_std_desc_size[type];
          }
#if USBCFG_DEV_EHCI          
          else if (hidInforIndex == USB_CONTROLLER_EHCI_0)
          {
              *descriptor = (uint_8_ptr)g_std_descriptors_hs[type];
              *size = g_std_desc_size_hs[type];
          }  
#endif          
        }
        break;
      case USB_HID_DESCRIPTOR:
        {
          type = USB_CONFIG_DESCRIPTOR ;
          if (hidInforIndex == USB_CONTROLLER_KHCI_0)
          {
              *descriptor = (uint_8_ptr)(g_std_descriptors [type]+
                               CONFIG_ONLY_DESC_SIZE+IFACE_ONLY_DESC_SIZE);
          }
#if USBCFG_DEV_EHCI          
          else if (hidInforIndex == USB_CONTROLLER_EHCI_0)
          {
              *descriptor = (uint_8_ptr)(g_std_descriptors_hs [type]+
                               CONFIG_ONLY_DESC_SIZE+IFACE_ONLY_DESC_SIZE);
          }
#endif          
          *size = HID_ONLY_DESC_SIZE;
        }
        break;
      case USB_STRING_DESCRIPTOR:
        {
            if(index == 0)
            {
                /* return the string and size of all languages */
                *descriptor = (uint8_t *)g_languages.languages_supported_string;
                *size = g_languages.languages_supported_size; 
            } else
            {
                uint8_t lang_id=0;
                uint8_t lang_index=USB_MAX_LANGUAGES_SUPPORTED;

                for(;lang_id< USB_MAX_LANGUAGES_SUPPORTED;lang_id++)
                {
                    /* check whether we have a string for this language */
                     if (index == g_languages.usb_language[lang_id].language_id) 
                    {   /* check for max descriptors */
                        if(str_num < USB_MAX_STRING_DESCRIPTORS)
                        {   /* setup index for the string to be returned */
                            lang_index=str_num;
                        }
                        break;
                    }

                }

                /* set return val for descriptor and size */
                *descriptor = (uint8_t *)
                        g_languages.usb_language[lang_id].lang_desc[lang_index];
                *size = g_languages.usb_language[lang_id].lang_desc_size[lang_index];
            }
        }
        break;
      default :
        if (type < USB_MAX_STD_DESCRIPTORS)
        {
            /* set return val for descriptor and size*/
          if (hidInforIndex == USB_CONTROLLER_KHCI_0)
          {
              *descriptor = (uint_8_ptr)g_std_descriptors[type];
          }
#if USBCFG_DEV_EHCI           
          else if (hidInforIndex == USB_CONTROLLER_EHCI_0)
          {
              *descriptor = (uint_8_ptr)g_std_descriptors_hs[type];
          }    
#endif          
//            *descriptor = (uint_8_ptr)g_std_descriptors [type];
		//	printf("type: %d \n", type);

            // device descriptor
            if(type == 1){
            	device_desc_req_count++;
            }
            // andrei: second request
            if(device_desc_req_count>=2){
            	device_desc_req_count = 0;
            }

            /* if there is no descriptor then return error */
            if(*descriptor == NULL)
            {
                return USBERR_INVALID_REQ_TYPE;
            }

            if (hidInforIndex == USB_CONTROLLER_KHCI_0)
            {
                *size = g_std_desc_size[type];
            }
#if USBCFG_DEV_EHCI          
            else if (hidInforIndex == USB_CONTROLLER_EHCI_0)
            {
                *size = g_std_desc_size_hs[type];
            }  
#endif          
          
//            *size = g_std_desc_size[type];
        }
        else /* invalid descriptor */
        {
            return USBERR_INVALID_REQ_TYPE;
        }
        break;
    }
    return USB_OK;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Get_Interface
 *
 * @brief The function returns the alternate interface
 *
 * @param controller_ID : Controller ID
 * @param interface     : Interface number
 * @param alt_interface : Output alternate interface
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************
 *This function is called by the framework module to get the current interface
 *****************************************************************************/
uint8_t USB_Desc_Get_Interface(
      hid_handle_t handle,
      uint8_t interface,         /* [IN] Interface number */
      uint_8_ptr alt_interface  /* [OUT] Output alternate interface */
)
{
    UNUSED (handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get alternate interface*/
        *alt_interface = g_alternate_interface[interface];
        return USB_OK;
    }
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Set_Interface
 *
 * @brief The function sets the alternate interface
 *
 * @param controller_ID : Controller ID
 * @param interface     : Interface number
 * @param alt_interface : Input alternate interface
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************
 *This function is called by the framework module to set the interface
 *****************************************************************************/
uint8_t USB_Desc_Set_Interface(
      hid_handle_t handle,
      uint8_t interface,     /* [IN] Interface number */
      uint8_t alt_interface  /* [IN] Input alternate interface */
)
{
    UNUSED (handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set alternate interface*/
        g_alternate_interface[interface] = alt_interface;
        return USB_OK;
    }

    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Configation
 *
 * @brief The function checks whether the configuration parameter
 *        input is valid or not
 *
 * @param handle          handle
 * @param config_val      configuration value
 *
 * @return TRUE           When Valid
 *         FALSE          When Error
 *****************************************************************************/
uint8_t USB_Set_Configation
(
    hid_handle_t handle,
    uint8_t config
)
{
    UNUSED (handle)

    return USB_OK;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Valid_Configation
 *
 * @brief The function checks whether the configuration parameter
 *        input is valid or not
 *
 * @param controller_ID : Controller ID
 * @param config_val    : Configuration value
 *
 * @return TRUE           When Valid
 *         FALSE          When Error
 *****************************************************************************
 * This function checks whether the configuration is valid or not
 *****************************************************************************/
bool USB_Desc_Valid_Configation(
      hid_handle_t handle,
      uint16_t config_val)   //[IN] Configuration value 
{
    uint8_t loop_index=0;
    UNUSED (handle)

    // check with only supported val right now 
    while(loop_index < (USB_MAX_CONFIG_SUPPORTED+1))
    {
        if(config_val == g_valid_config_values[loop_index])
        {
            return TRUE;
        }
        loop_index++;
    }
    return FALSE;
}

/**************************************************************************//*!
 *
 * @name  USB_Desc_Valid_Interface
 *
 * @brief The function checks whether the interface parameter
 *        input is valid or not
 *
 * @param controller_ID : Controller ID
 * @param interface     : Target interface
 *
 * @return TRUE           When Valid
 *         FALSE          When Error
 *****************************************************************************
 * This function checks whether the interface is valid or not
 *****************************************************************************/
bool USB_Desc_Valid_Interface(
      hid_handle_t handle,
      uint8_t interface      /*[IN] Target interface */
)
{
    uint8_t loop_index=0;
    UNUSED (handle)

    /* check with only supported val right now */
    while(loop_index < USB_MAX_SUPPORTED_INTERFACES)
    {
        if(interface == g_alternate_interface[loop_index])
        {
            return TRUE;
        }
        loop_index++;
    }
    return FALSE;
}
/**************************************************************************//*!
 *
 * @name  USB_Desc_Remote_Wakeup
 *
 * @brief The function checks whether the remote wakeup is supported or not
 *
 * @param controller_ID : Controller ID
 *
 * @return REMOTE_WAKEUP_SUPPORT (TRUE) - If remote wakeup supported
 *****************************************************************************
 * This function returns remote wakeup is supported or not
 *****************************************************************************/
bool USB_Desc_Remote_Wakeup(
      hid_handle_t handle
)
{
    UNUSED (handle)
    return REMOTE_WAKEUP_SUPPORT;
}


/**************************************************************************//*!
 *
 * @name  USB_Desc_Get_Endpoints
 *
 * @brief The function returns with the list of all non control endpoints used
 *
 * @param controller_ID : Controller ID
 *
 * @return pointer to USB_ENDPOINTS
 *****************************************************************************
 * This function returns the information about all the non control endpoints
 * implemented
 *****************************************************************************/
// JB void* USB_Desc_Get_Endpoints(
extern usb_endpoints_t *USB_Desc_Get_Endpoints(hid_handle_t handle)
{
    UNUSED (handle)
    return (void*)&usb_desc_ep;
}

uint8_t USB_Desc_Get_Entity(
      hid_handle_t handle,
      entity_type type,
      uint32_t * object)
{
    switch (type)
    {
        case USB_CLASS_INFO:
            g_usb_if[0].index = 1;
            g_usb_if[0].endpoints = usb_desc_ep;
            *object = (unsigned long)&g_usb_dec_class;
            break;
        default :
            break;
    }/* End Switch */
    return USB_OK;
}

usb_desc_request_notify_struct_t  g_desc_callback =
{
    USB_Desc_Get_Descriptor,
    USB_Desc_Get_Interface,
    USB_Desc_Set_Interface,
    USB_Set_Configation,
    USB_Desc_Get_Entity
};
