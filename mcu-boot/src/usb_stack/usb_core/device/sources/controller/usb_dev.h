/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: usb_dev.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*  This file contains the declarations specific to the USB Device API
*
*END*********************************************************************/
#ifndef __USB_DEV_H__
#define __USB_DEV_H__ 1

#include "usb_framework.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)

#undef USBCFG_DEV_USE_TASK
#define USBCFG_DEV_USE_TASK 0

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)

#undef USBCFG_DEV_USE_TASK
#define USBCFG_DEV_USE_TASK 1

#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#if USE_RTOS
#undef USBCFG_DEV_USE_TASK
#define USBCFG_DEV_USE_TASK 1
#else
#undef USBCFG_DEV_USE_TASK
#define USBCFG_DEV_USE_TASK 0
#endif

#endif
#define MAX_DEVICE_SERVICE_NUMBER     8

/* Callback function storage structure */
typedef struct service_struct 
{
   usb_event_service_t              service;
   void*                            arg;
   uint8_t                          type;
} service_struct_t;

typedef struct usb_dev_state_struct
{
    usb_device_handle               controller_handle;
    void*                           usb_dev_interface;
#if USBCFG_DEV_USE_TASK
    os_msgq_handle                  usb_dev_service_que;
    uint32_t                        task_id;
#endif
    usb_class_fw_object_struct_t    usb_framework;
    service_struct_t                services[MAX_DEVICE_SERVICE_NUMBER];
    uint8_t                         occupied;
    uint8_t                         controller_id;       /* Device controller ID */
    os_mutex_handle                 mutex;
} usb_dev_state_struct_t;


typedef struct usb_dev_interface_functions_struct
{
   /* The Host/Device init function */
   usb_status (_CODE_PTR_ dev_preint)(usb_device_handle, usb_device_handle *);

   /* The Host/Device init function */
   usb_status (_CODE_PTR_ dev_init)(uint8_t, usb_device_handle);
   
   /* The Host/Device init function */
   usb_status (_CODE_PTR_ dev_postinit)(uint8_t, usb_device_handle);

   /* The function to send data */
   usb_status (_CODE_PTR_ dev_send)(usb_device_handle, struct xd_struct *);

   /* The function to receive data */
   usb_status (_CODE_PTR_ dev_recv)(usb_device_handle, struct xd_struct *);
#if USBCFG_DEV_ADVANCED_CANCEL_ENABLE   
   /* The function to cancel the transfer */
   usb_status (_CODE_PTR_ dev_cancel_transfer)(usb_device_handle, uint8_t, uint8_t);
#endif
   
   usb_status (_CODE_PTR_ dev_init_endoint)(usb_device_handle, struct xd_struct *);
   
   usb_status (_CODE_PTR_ dev_deinit_endoint)(usb_device_handle, uint8_t, uint8_t);
   
   usb_status (_CODE_PTR_ dev_unstall_endpoint)(usb_device_handle, uint8_t, uint8_t);
   
   usb_status (_CODE_PTR_ dev_get_endpoint_status)(usb_device_handle, uint8_t, uint16_t*);
   
   usb_status (_CODE_PTR_ dev_set_endpoint_status)(usb_device_handle, uint8_t, uint16_t);

   usb_status (_CODE_PTR_ dev_get_transfer_status)(usb_device_handle, uint8_t, uint8_t);
   
   usb_status (_CODE_PTR_ dev_set_address)(usb_device_handle, uint8_t);
   
   usb_status (_CODE_PTR_ dev_shutdown)(usb_device_handle);
   
   usb_status (_CODE_PTR_ dev_get_setup_data)(usb_device_handle, uint8_t, uint8_t *);
#if USBCFG_DEV_ADVANCED_SUSPEND_RESUME  
   usb_status (_CODE_PTR_ dev_assert_resume)(usb_device_handle);
#endif
   
   usb_status (_CODE_PTR_ dev_stall_endpoint)(usb_device_handle, uint8_t, uint8_t);
   
   usb_status (_CODE_PTR_ dev_set_device_status)(usb_device_handle, uint8_t, uint16_t);
   
   usb_status (_CODE_PTR_ dev_get_device_status)(usb_device_handle, uint8_t, uint16_t*);

   usb_status (_CODE_PTR_ dev_get_xd)(usb_device_handle, xd_struct_t**);

   usb_status (_CODE_PTR_ dev_reset)(usb_device_handle);
   
#if USBCFG_DEV_EHCI_TEST_MODE  
   usb_status (_CODE_PTR_ dev_set_test_mode)(usb_device_handle, uint16_t);
#endif

} usb_dev_interface_functions_struct_t;

#ifdef __cplusplus
extern "C" {
#endif
#ifdef USBCFG_OTG
extern uint32_t usb_otg_device_on_class_init
(
    usb_otg_handle     otg_handle, 
    usb_device_handle  dev_handle, 
    uint8_t             bm_attributes 
);

extern uint32_t usb_otg_device_hnp_enable
(
    usb_otg_handle handle, 
    uint8_t enable
);
#endif
#ifdef __cplusplus
}
#endif

#endif
/* EOF */
