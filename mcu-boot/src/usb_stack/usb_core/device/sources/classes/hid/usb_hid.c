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
* $FileName: usb_hid.c$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains USB stack HID layer implimentation.
*
*****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"

#if USBCFG_DEV_HID
#include "usb_class_internal.h"
#include "usb_class_hid.h"
#include "usb_hid.h"

#if (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID)

/*****************************************************************************
 * Constant and Macro's
 *****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/         
hid_device_struct_t g_hid_class[MAX_HID_DEVICE];
/*****************************************************************************
 * Local Types - None
 *****************************************************************************/

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
#if 0
static uint8_t HID_USB_Map_Ep_To_Struct_Index(hid_device_struct_t* devicePtr, 
                          uint8_t ep_num
                          );                          
#endif                          
/*****************************************************************************
 * Local Variables - None
 *****************************************************************************/
 
 
/*****************************************************************************
 * Local Functions
 *****************************************************************************/
 
   /*************************************************************************//*!
  *
  * @name  USB_Hid_Allocate_Handle
  *
  * @brief The funtion reserves entry in device array and returns the index.
  *
  * @param none.
  * @return returns the reserved handle or if no entry found device busy.      
  *
  *****************************************************************************/
 static usb_status USB_Hid_Allocate_Handle(hid_device_struct_t** handle)
 {
     uint32_t cnt = 0;
     for (;cnt< MAX_HID_DEVICE;cnt++)
     {
         if (g_hid_class[cnt].handle == NULL)
         {
             *handle = (hid_device_struct_t*)&g_hid_class[cnt];
             return USB_OK;
         }
     }
     return USBERR_DEVICE_BUSY;
 }
  /*************************************************************************//*!
  *
  * @name  USB_Hid_Free_Handle
  *
  * @brief The funtion releases entry in device array .
  *
  * @param handle  index in device array to be released..
  * @return returns and error code or USB_OK.      
  *
  *****************************************************************************/
 
 static usb_status USB_Hid_Free_Handle(hid_device_struct_t* handle)
 {
     int32_t cnt = 0;
     for (;cnt< MAX_HID_DEVICE;cnt++)
     {
         if ((&g_hid_class[cnt]) == handle)
         {
             OS_Mem_zero((void*)handle, sizeof(hid_device_struct_t));
             return USB_OK;
         }
     }
     return USBERR_INVALID_PARAM;
 }

 /*************************************************************************//*!
 *
 * @name  USB_Hid_Get_Device_Ptr
 *
 * @brief The funtion gets the device pointer from device array .
 *
 * @param handle  index in device array.
 * @return returns returns pointer to HID device structure..      
 *
 *****************************************************************************/
static hid_device_struct_t* USB_Hid_Get_Device_Ptr(hid_handle_t handle)
{
    return (hid_device_struct_t *)handle; 
}
#if 0
 /*************************************************************************//*!
 *
 * @name  HID_USB_Map_Ep_To_Struct_Index
 *
 * @brief The funtion maps the endpoint num to the index of the ep data 
 *           structure
 *
 * @param handle          handle to identify the controller
 * @param ep_num          endpoint num
 *
 * @return index          mapped index       
 *
 *****************************************************************************/
static uint8_t HID_USB_Map_Ep_To_Struct_Index(hid_device_struct_t* devicePtr, uint8_t ep_num)
{
    uint8_t count = 0;
    
    usb_endpoints_t *ep_desc_data = devicePtr->ep_desc_data;
    /* map the endpoint num to the index of the endpoint structure */
    for (count = 0; count < ep_desc_data->count; count++) 
    {
        if (ep_desc_data->ep[count].ep_num == ep_num)
        {
            break;
        }
    }

    return count; 
}
#endif

/**************************************************************************//*!
 *
 * @name  USB_Service_Hid_IN
 *
 * @brief The funtion ic callback function of HID endpoint 
 *
 * @param event
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Service_Hid_IN
(
    usb_event_struct_t* event,
    void* arg
)
{
#if HID_IMPLEMENT_QUEUING
    uint8_t index;
    uint8_t producer;
    uint8_t consumer;
#endif
    hid_device_struct_t*  devicePtr;
    //usb_endpoints_t *ep_desc_data;
   
    devicePtr = (hid_device_struct_t*)arg;
    
    //ep_desc_data = devicePtr->ep_desc_data; 
#if HID_IMPLEMENT_QUEUING
     /* map the endpoint num to the index of the endpoint structure */
    index = HID_USB_Map_Ep_To_Struct_Index(devicePtr, event->ep_num); 
    producer = devicePtr->hid_endpoint_data.ep[index].bin_producer;
        
    /* if there are no errors de-queue the queue and decrement the no. of 
     transfers left, else send the same data again */
     /* de-queue if the send is complete with no error */
    devicePtr->hid_endpoint_data.ep[index].bin_consumer++;  

    consumer = devicePtr->hid_endpoint_data.ep[index].bin_consumer;

    if (consumer != producer) 
    {
        /*if bin is not empty */
        hid_queue_struct_t queue;    
        /* send the next packet in queue */
        queue = devicePtr->hid_endpoint_data.ep[index].queue[consumer % HID_MAX_QUEUE_ELEMS];
        (void)USB_Class_Send_Data(devicePtr->class_handle, queue.channel, queue.app_buff, queue.size);
    }
#endif
    /* notify the app of the send complete */
     if (devicePtr->class_specific_callback.callback != NULL) 
     {
         devicePtr->class_specific_callback.callback(USB_DEV_EVENT_SEND_COMPLETE, USB_REQ_VAL_INVALID,(uint8_t**)event,0,
                (event->len == 0xFFFFFFFF)? 0 : devicePtr->class_specific_callback.arg);
    }

}

/**************************************************************************//*!
 *
 * @name  USB_Service_Hid_OUT
 *
 * @brief The funtion ic callback function of HID endpoint 
 *
 * @param event
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Service_Hid_OUT
(
    usb_event_struct_t* event,
    void* arg
)
{
#if HID_IMPLEMENT_QUEUING
    uint8_t index;
    uint8_t producer;
    uint8_t consumer;
#endif
    hid_device_struct_t*  devicePtr;
    //usb_endpoints_t *ep_desc_data;
   
    devicePtr = (hid_device_struct_t*)arg;
    
    //ep_desc_data = devicePtr->ep_desc_data; 
#if HID_IMPLEMENT_QUEUING
     /* map the endpoint num to the index of the endpoint structure */
    index = HID_USB_Map_Ep_To_Struct_Index(devicePtr, event->ep_num); 
    producer = devicePtr->hid_endpoint_data.ep[index].bin_producer;
        
    /* if there are no errors de-queue the queue and decrement the no. of 
     transfers left, else send the same data again */
     /* de-queue if the send is complete with no error */
    devicePtr->hid_endpoint_data.ep[index].bin_consumer++;  

    consumer = devicePtr->hid_endpoint_data.ep[index].bin_consumer;

    if (consumer != producer) 
    {
        /*if bin is not empty */
        hid_queue_struct_t queue;    
        /* send the next packet in queue */
        queue = devicePtr->hid_endpoint_data.ep[index].queue[consumer % HID_MAX_QUEUE_ELEMS];
        (void)USB_Class_Send_Data(devicePtr->class_handle, queue.channel, queue.app_buff, queue.size);
    }
#endif
    /* notify the app of the send complete */
     if (devicePtr->class_specific_callback.callback != NULL) 
     {
         devicePtr->class_specific_callback.callback(USB_DEV_EVENT_DATA_RECEIVED, USB_REQ_VAL_INVALID,(uint8_t**)event,0,
                (event->len == 0xFFFFFFFF)? 0 : devicePtr->class_specific_callback.arg);
    }

}


/**************************************************************************//*!
 *
 * @name  USB_Class_Hid_Event
 *
 * @brief The funtion initializes HID endpoint 
 *
 * @param handle          handle to Identify the controller
 * @param event           pointer to event structure
 * @param val             gives the configuration value 
 *
 * @return None       
 *
 *****************************************************************************/
void USB_Class_Hid_Event
(
    uint8_t     event, 
    void*       val,
    void *      arg
) 
{
    uint8_t                      index;
#if USBCFG_DEV_COMPOSITE
    usb_composite_info_struct_t* usb_composite_info;
    uint32_t                     interface_index = 0xFF;
#else
    usb_class_struct_t*          usbclass;
#endif    
    usb_ep_struct_t*             ep_struct_ptr;
    hid_device_struct_t*         devicePtr;

    devicePtr = (hid_device_struct_t*)arg;
    if (event == USB_DEV_EVENT_CONFIG_CHANGED)
    {
#if USBCFG_DEV_COMPOSITE
        uint8_t count = 0;
        uint8_t type_sel;
        devicePtr->desc_callback.get_desc_entity((uint32_t)devicePtr->handle,
            USB_COMPOSITE_INFO, (uint32_t *)&usb_composite_info);
        devicePtr->desc_callback.get_desc_entity((uint32_t)devicePtr,
            USB_CLASS_INTERFACE_INDEX_INFO, (uint32_t *)&interface_index);
        
        if(interface_index == 0xFF)
        {
 //           USB_PRINTF("not find interface index\n");
            return;
        }
        for (type_sel = 0;type_sel < usb_composite_info->count;type_sel++)
        {
            if ((usb_composite_info->class[type_sel].type == USB_CLASS_HID) && (type_sel == interface_index))
            {
                break;
            }
        }
        if(type_sel >= usb_composite_info->count)
        {
//            USB_PRINTF("not find hid interface\n");
            return;
        }
        devicePtr->ep_desc_data = (usb_endpoints_t *) &usb_composite_info->class[type_sel].interfaces.interface->endpoints;
        
        if (usb_composite_info->class[type_sel].interfaces.interface->endpoints.count > MAX_HID_CLASS_EP_NUM)
        {
            USB_PRINTF("too many hid endpoint for the class driver\n");
            return;
        }

        for (index = 0; index < usb_composite_info->class[type_sel].interfaces.interface->endpoints.count; index++) 
        {
            devicePtr->hid_endpoint_data.ep[index].endpoint = 
                                 usb_composite_info->class[type_sel].interfaces.interface->endpoints.ep[index].ep_num;
            devicePtr->hid_endpoint_data.ep[index].type = 
                                  usb_composite_info->class[type_sel].interfaces.interface->endpoints.ep[index].type;
#if HID_IMPLEMENT_QUEUING
            devicePtr->hid_endpoint_data.ep[index].bin_consumer = 0x00;
            devicePtr->hid_endpoint_data.ep[index].bin_producer = 0x00;
#endif /* HID_IMPLEMENT_QUEUING */
        }
#else
        uint8_t count = 0;
        devicePtr->desc_callback.get_desc_entity((uint32_t)devicePtr->handle,
            USB_CLASS_INFO, (uint32_t *)&usbclass);
        devicePtr->ep_desc_data = (usb_endpoints_t *) &usbclass->interfaces.interface->endpoints;

        if (usbclass->interfaces.interface->endpoints.count > MAX_HID_CLASS_EP_NUM)
        {
        #ifdef _DEBUG
//            USB_PRINTF("too many hid endpoint for the class driver\n");
        #endif
            return;
        }

        for (index = 0; index < usbclass->interfaces.interface->endpoints.count; index++) 
        {
            devicePtr->hid_endpoint_data.ep[index].endpoint = 
                                        usbclass->interfaces.interface->endpoints.ep[index].ep_num;
            devicePtr->hid_endpoint_data.ep[index].type = 
                                         usbclass->interfaces.interface->endpoints.ep[index].type;
#if HID_IMPLEMENT_QUEUING
            devicePtr->hid_endpoint_data.ep[index].bin_consumer = 0x00;
            devicePtr->hid_endpoint_data.ep[index].bin_producer = 0x00;
#endif /* HID_IMPLEMENT_QUEUING */
        }
#endif
        usb_endpoints_t *ep_desc_data = devicePtr->ep_desc_data; 
        
        /* intialize all non control endpoints */
        while (count < ep_desc_data->count) 
        {
            ep_struct_ptr= (usb_ep_struct_t*) &ep_desc_data->ep[count];
            (void)usb_device_init_endpoint(devicePtr->handle, ep_struct_ptr, TRUE);

            if (ep_struct_ptr->direction == USB_SEND)
            {
                /* register callback service for endpoint */
                (void)usb_device_register_service(devicePtr->handle,
                    (uint8_t)((USB_SERVICE_EP0+ep_struct_ptr->ep_num) | ((uint8_t)(ep_struct_ptr->direction << 7))), 
                    USB_Service_Hid_IN, arg);
            }
            else if (ep_struct_ptr->direction == USB_RECV)
            {
                /* register callback service for endpoint */
                (void)usb_device_register_service(devicePtr->handle,
                    (uint8_t)((USB_SERVICE_EP0+ep_struct_ptr->ep_num) | ((uint8_t)(ep_struct_ptr->direction << 7))), 
                    USB_Service_Hid_OUT, arg);
            }
            else
            {
            }
            count++;
        }
    }
    else if (event == USB_DEV_EVENT_ENUM_COMPLETE) 
    {

    }
    else if (event == USB_DEV_EVENT_BUS_RESET) 
    {
#if HID_IMPLEMENT_QUEUING
        if (devicePtr->hid_endpoint_data.ep != NULL){
            for (index = 0; index < MAX_HID_CLASS_EP_NUM; index++) 
            {
                devicePtr->hid_endpoint_data.ep[index].bin_consumer = 0x00;
                devicePtr->hid_endpoint_data.ep[index].bin_producer = 0x00;
            }
        }
#endif
    }
	else if(event == USB_DEV_EVENT_EP_UNSTALLED)
    {
        uint8_t value;
        value = *((uint8_t *)val);
        if (devicePtr->hid_endpoint_data.ep != NULL)
		{
            for (index = 0; index < MAX_HID_CLASS_EP_NUM; index++) 
            {
				if((value & 0x0F) == devicePtr->hid_endpoint_data.ep[index].endpoint)
					usb_device_unstall_endpoint(devicePtr->handle,value & 0x0F, (value & 0x80) >> 7);
            }
        }
      
    }
	else if(event == USB_DEV_EVENT_TYPE_CLR_EP_HALT)
    {
        uint8_t value;
        value = *((uint8_t *)val);
        if (devicePtr->hid_endpoint_data.ep != NULL)
		{
            for (index = 0; index < MAX_HID_CLASS_EP_NUM; index++) 
            {
				if((value & 0x0F) == devicePtr->hid_endpoint_data.ep[index].endpoint)
					usb_device_unstall_endpoint(devicePtr->handle,value & 0x0F, (value & 0x80) >> 7);
            }
        }
      
    }
    if (devicePtr->hid_application_callback.callback != NULL)
    {
        devicePtr->hid_application_callback.callback(event,val,
        devicePtr->hid_application_callback.arg);
    }
}

/**************************************************************************//*!
 *
 * @name  USB_HID_Other_Requests
 *
 * @brief The funtion provides flexibilty to add class and vendor specific
 *        requests 
 *
 * @param handle:
 * @param setup_packet:     setup packet recieved      
 * @param data:             data to be send back
 * @param size:             size to be returned    
 *
 * @return status:       
 *                        USB_OK : When Successfull       
 *                        Others : When Error
 *
 *****************************************************************************/
usb_status USB_HID_Requests
(
    usb_setup_struct_t* setup_packet, 
    uint8_t *           *data, 
    uint32_t            *size,
    void*               arg
) 
{    
    //uint8_t index;
    usb_status error = USBERR_INVALID_REQ_TYPE;
    hid_device_struct_t*  devicePtr;
    /* buffer to send in case of get report req */
    //uint8_t rpt_buf[REPORT_SIZE];
    
    //*((uint32_t*)rpt_buf) = 0;
    
    /*Get the device pointer to which this callback corresponds.*/
    devicePtr = (hid_device_struct_t*)arg;
    
    if((setup_packet->request_type & USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_POS) == 
                                                      USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_CLASS) 
    {  
        if(devicePtr->class_specific_callback.callback != NULL) 
        {  
          /* handle callback if the application has supplied it */
          /* set the size of the transfer from the setup packet */  
            *size = setup_packet->length; 
                
          /* notify the application of the class request.*/
          /* give control to the application */
           error = devicePtr->class_specific_callback.callback(
                                      /* request type */ 
                                      setup_packet->request,
                                      setup_packet->value, 
                                      /* pointer to the data */
                                      data,
                                      /* size of the transfer */
                                      size,devicePtr->class_specific_callback.arg);
        }
    } 
    else if((setup_packet->request_type & USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_POS) == 
                                                     USB_DEV_REQ_STD_REQUEST_TYPE_TYPE_VENDOR) 
    {   /* vendor specific request  */
        if(devicePtr->vendor_req_callback.callback != NULL) 
        {
            error = devicePtr->vendor_req_callback.callback(setup_packet,
                data,size,devicePtr->vendor_req_callback.arg);
        }
    } 
    return error;
}

/*****************************************************************************
 * Global Functions
 *****************************************************************************/


/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Init
 *
 * @brief The funtion initializes the Device and Controller layer 
 *
 * @param *handle: handle pointer to Identify the controller
 * @param hid_class_callback:   event callback      
 * @param vendor_req_callback:  vendor specific class request callback      
 * @param param_callback:       application params callback      
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 ******************************************************************************
 *
 *This function initializes the HID Class layer and layers it is dependent on 
 *
 *****************************************************************************/
usb_status USB_Class_HID_Init
(
    uint8_t controller_id,
    hid_config_struct_t* hid_config_ptr,
    hid_handle_t *  hidHandle
) 
{
    //uint8_t index;
    usb_status error = USBERR_ERROR;
    hid_device_struct_t*  devicePtr;

    if (NULL == hid_config_ptr)
    {
        return USBERR_ERROR;
    }
    error = USB_Hid_Allocate_Handle(&devicePtr);/*(hid_device_struct_t*)OS_Mem_alloc_zero(sizeof(hid_device_struct_t));*/
    if (USB_OK != error)
    {
        return error;
    }
    /*if (NULL == devicePtr)
    {
        #if _DEBUG
            USB_PRINTF("USB_Class_HID_Init: Memalloc failed\n");
        #endif  
        return USBERR_ALLOC;
    }
    devicePtr->desc_callback_ptr = (usb_desc_request_notify_struct_t*)OS_Mem_alloc_zero(sizeof(usb_desc_request_notify_struct_t));
    if (NULL == devicePtr->desc_callback_ptr)
    {
        #ifdef _DEBUG
            USB_PRINTF("USB_Class_Audio_Init: desc_callback_ptr Memalloc failed\n");
        #endif
        OS_Mem_free(devicePtr);
        return USBERR_ALLOC;
    }*/
    //devicePtr->hid_endpoint_data.ep = NULL;
#if USBCFG_DEV_COMPOSITE
    devicePtr->class_handle = USB_Class_Get_Class_Handle();
    devicePtr->handle = (usb_device_handle)USB_Class_Get_Ctrler_Handle(devicePtr->class_handle);
    if (NULL == devicePtr->handle)
    {
        goto error1;
    }
#else
    /* Initialize the device layer*/
    error = usb_device_init(controller_id,(&devicePtr->handle));
    if (error != USB_OK)
    {
        goto error1;  
    }
    /* Initialize the generic class functions */
    devicePtr->class_handle = USB_Class_Init(devicePtr->handle,
    USB_Class_Hid_Event,USB_HID_Requests,(void *)devicePtr,hid_config_ptr->desc_callback_ptr);
    if (error != USB_OK)
    {
        goto error2;  
    }

#endif
    /* save the callback pointer */
    OS_Mem_copy(&hid_config_ptr->hid_application_callback,
    &devicePtr->hid_application_callback,sizeof(usb_application_callback_struct_t));           

    /* save the callback pointer */
    OS_Mem_copy(&hid_config_ptr->vendor_req_callback,
    &devicePtr->vendor_req_callback,sizeof(usb_vendor_req_callback_struct_t));        

    /* Save the callback to ask application for class specific params*/
    OS_Mem_copy(&hid_config_ptr->class_specific_callback,
    &devicePtr->class_specific_callback,sizeof(usb_class_specific_callback_struct_t));                    

    /* Save the dec  callback to ask application for class specific params*/
    OS_Mem_copy(hid_config_ptr->desc_callback_ptr,
    &devicePtr->desc_callback,sizeof(usb_desc_request_notify_struct_t)); 

    *hidHandle =(unsigned long)devicePtr;
    //devicePtr->user_handle = *hidHandle;
    usb_device_postinit(controller_id,devicePtr->handle);

    return USB_OK;
#if !USBCFG_DEV_COMPOSITE
error2:
     /* TBD:Implement usb_device_deinit and call here*/
#endif
error1: 
    //OS_Mem_free(devicePtr);
    devicePtr = NULL;
    return error;     
}

/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Deinit
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
usb_status USB_Class_HID_Deinit
(
    hid_handle_t handle
) 
{
    usb_status error = USB_OK;
    hid_device_struct_t*  devicePtr;

    if (handle == 0)
    {
        return USBERR_ERROR;
    }

    devicePtr = USB_Hid_Get_Device_Ptr(handle);

    if (NULL == devicePtr)
    {
        return USBERR_NO_DEVICE_CLASS;
    }
#if !USBCFG_DEV_COMPOSITE  
    /* Deinitialize the generic class functions */
    error = USB_Class_Deinit(devicePtr->handle,devicePtr->class_handle);
    if(error == USB_OK)
    {
        /* Deinitialize the device layer*/
        error = usb_device_deinit(devicePtr->handle);
    }
#endif 
    /*

    if(NULL != devicePtr->hid_endpoint_data.ep)
    {
        OS_Mem_free(devicePtr->hid_endpoint_data.ep);
    }
    
    if(NULL != devicePtr->desc_callback_ptr)
    {
        OS_Mem_free(devicePtr->desc_callback_ptr);
    }
    if(NULL != devicePtr)
    {
        OS_Mem_free(devicePtr);
    }
    */
    USB_Hid_Free_Handle(devicePtr);
    devicePtr = NULL;

    return error;
}

#if USBCFG_DEV_ADVANCED_CANCEL_ENABLE
/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Cancel
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init
 * @param ep_num          :   endpoint num 
 * @param direction        :   direction of the endpoint 
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/

usb_status USB_Class_HID_Cancel
(
    hid_handle_t handle,/*[IN]*/
    uint8_t ep_num,/*[IN]*/
    uint8_t direction
)
{
    hid_device_struct_t*  devicePtr;
    //usb_endpoints_t *ep_desc_data;    
    usb_status error = USB_OK;
    if (handle == 0)
    {
        return USBERR_ERROR;
    }

    devicePtr = USB_Hid_Get_Device_Ptr(handle);
    if (NULL == devicePtr)
    {
        return USBERR_NO_DEVICE_CLASS;
    }
    //ep_desc_data = devicePtr->ep_desc_data; 
    error = usb_device_cancel_transfer(devicePtr->handle, ep_num, direction);

    return error;

}
#endif

/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Send_Data
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
usb_status USB_Class_HID_Send_Data
(
    hid_handle_t handle,/*[IN]*/
    uint8_t ep_num,/*[IN]*/
    uint8_t * app_buff,/*[IN]*/
    uint32_t size /*[IN]*/
) 
{
#if HID_IMPLEMENT_QUEUING
    uint8_t index;
    uint8_t producer;
    uint8_t consumer;
#endif 
    usb_status error = USB_OK;
    hid_device_struct_t*  devicePtr;
    //usb_endpoints_t *ep_desc_data;    
       
    if (handle == 0)
    {
        return USBERR_ERROR;
    }

    devicePtr = USB_Hid_Get_Device_Ptr(handle);
    if (NULL == devicePtr)
    {
        return USBERR_NO_DEVICE_CLASS;
    }
    //ep_desc_data = devicePtr->ep_desc_data; 
#if HID_IMPLEMENT_QUEUING    
     /* map the endpoint num to the index of the endpoint structure */
    index = HID_USB_Map_Ep_To_Struct_Index(devicePtr, ep_num); 
   
    producer = devicePtr->hid_endpoint_data.ep[index].bin_producer;
    consumer = devicePtr->hid_endpoint_data.ep[index].bin_consumer;

    if((uint8_t)(producer - consumer) != (uint8_t)(HID_MAX_QUEUE_ELEMS))  
    {/* the bin is not full*/
    
        uint8_t queue_num = (uint8_t)(producer % HID_MAX_QUEUE_ELEMS);
        /* put all send request parameters in the endpoint data structure */
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].channel = ep_num;
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].app_buff = app_buff;
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].size = size; 
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].handle = devicePtr->handle;
   
        /* increment producer bin by 1*/       
        devicePtr->hid_endpoint_data.ep[index].bin_producer++;
        producer++;
             
        if((uint8_t)(producer - consumer) == (uint8_t)1)         
        {
#endif
            /*send the IO if there is only one element in the queue */          
            error = USB_Class_Send_Data(devicePtr->class_handle, ep_num, app_buff,size);
#if HID_IMPLEMENT_QUEUING
        }
    }
    else /* bin is full */
    {
        error = USBERR_DEVICE_BUSY; 
    }
#endif
    return error;
}

/**************************************************************************//*!
 *
 * @name  USB_Class_HID_Recv_Data
 *
 * @brief 
 *
 * @param handle          :   handle returned by USB_Class_HID_Init
 * @param ep_num          :   endpoint num 
 * @param app_buff        :   buffer to send
 * @param size            :   length of the transfer   
 *
 * @return status       
 *         USB_OK           : When Successfull 
 *         Others           : Errors
 *****************************************************************************/
usb_status USB_Class_HID_Recv_Data
(
    hid_handle_t handle,/*[IN]*/
    uint8_t ep_num,/*[IN]*/
    uint8_t * app_buff,/*[IN]*/
    uint32_t size /*[IN]*/
) 
{
#if HID_IMPLEMENT_QUEUING
    uint8_t index;
    uint8_t producer;
    uint8_t consumer;
#endif 
    usb_status error = USB_OK;
    hid_device_struct_t*  devicePtr;
    //usb_endpoints_t *ep_desc_data;    
       
    if (handle == 0)
    {
        return USBERR_ERROR;
    }

    devicePtr = USB_Hid_Get_Device_Ptr(handle);
    if (NULL == devicePtr)
    {
        return USBERR_NO_DEVICE_CLASS;
    }
    //ep_desc_data = devicePtr->ep_desc_data; 
#if HID_IMPLEMENT_QUEUING    
     /* map the endpoint num to the index of the endpoint structure */
    index = HID_USB_Map_Ep_To_Struct_Index(devicePtr, ep_num); 
   
    producer = devicePtr->hid_endpoint_data.ep[index].bin_producer;
    consumer = devicePtr->hid_endpoint_data.ep[index].bin_consumer;

    if((uint8_t)(producer - consumer) != (uint8_t)(HID_MAX_QUEUE_ELEMS))  
    {/* the bin is not full*/
    
        uint8_t queue_num = (uint8_t)(producer % HID_MAX_QUEUE_ELEMS);
        /* put all send request parameters in the endpoint data structure */
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].channel = ep_num;
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].app_buff = app_buff;
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].size = size; 
        devicePtr->hid_endpoint_data.ep[index].queue[queue_num].handle = devicePtr->handle;
   
        /* increment producer bin by 1*/       
        devicePtr->hid_endpoint_data.ep[index].bin_producer++;
        producer++;
             
        if((uint8_t)(producer - consumer) == (uint8_t)1)         
        {
#endif
            /*send the IO if there is only one element in the queue */          
            error = usb_device_recv_data(devicePtr->handle, ep_num, app_buff,size);
#if HID_IMPLEMENT_QUEUING
        }
    }
    else /* bin is full */
    {
        error = USBERR_DEVICE_BUSY; 
    }
#endif
    return error;
}

#endif /*HID_CONFIG*/

#endif //BL_CONFIG_USB_HID
/* EOF */
