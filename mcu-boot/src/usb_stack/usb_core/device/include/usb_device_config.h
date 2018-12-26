/**HEADER********************************************************************
* 
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
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
* $FileName: usb_device_config.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*   
*
*END************************************************************************/
#include "bootloader_common.h"

#ifndef __usb_dev_config_h__
#define __usb_dev_config_h__

#define CPU_MK64F12                      1

/* if KHCI device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_KHCI                   BL_CONFIG_USB_HID

/* if EHCI device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_EHCI                   0

/* if HID device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_HID                    BL_CONFIG_USB_HID

/* if PHDC device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_PHDC                   BL_CONFIG_USB_HID

/* if AUDIO device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_AUDIO                  BL_CONFIG_USB_HID

/* if Audio class 2.0  supported 
 * 1 supported
 * 0 not supported
 */
#if USBCFG_DEV_AUDIO
#define USBCFG_AUDIO_CLASS_2_0            0
#endif

/* if CDC device supported 
 * 1 supported
 * 0 not supported
 */
 
#define USBCFG_DEV_CDC                    BL_CONFIG_USB_HID

/* if RNDIS  supported 
 * 1 supported
 * 0 not supported
 */
#if USBCFG_DEV_CDC
#define USBCFG_DEV_RNDIS_SUPPORT          0
#endif

/* if MSC device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_MSC                    BL_CONFIG_USB_HID

/* if composite device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_COMPOSITE              0

/* if device is self powered 
 * 1 self power
 * 0 bus power
 */
#define USBCFG_DEV_SELF_POWER             1

/* if device remote wakeup supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_REMOTE_WAKEUP          0

/* how many device instance supported */
#define USBCFG_DEV_NUM                    1

/* how many endpoints are supported */
#define USBCFG_DEV_MAX_ENDPOINTS          (6) 

/* how many XDs are supported at most */
#define USBCFG_DEV_MAX_XDS                (12)

/* how many instance should be supported for one class type device */
#define USBCFG_DEV_MAX_CLASS_OBJECT       (2)

#if USBCFG_DEV_KHCI
    /* 
    ** Allow workaround for bug in the peripheral when unaligned buffer @4B address is used
    */
    #define USBCFG_KHCI_4BYTE_ALIGN_FIX       (1)

    #if USBCFG_KHCI_4BYTE_ALIGN_FIX
    /*
    ** The aligned buffer size for IN transactions, active when USBCFG_KHCI_4BYTE_ALIGN_FIX is defined
    */
        #define USBCFG_DEV_KHCI_SWAP_BUF_MAX          (64)
    #endif

    #define USBCFG_DEV_KHCI_ADVANCED_ERROR_HANDLING    (0)
#endif

/* If the buffer provided by APP is cacheable
* 1 cacheable, buffer cache maintenance is needed
* 0 uncacheable, buffer cache maintenance is not needed
*/
#define	USBCFG_BUFF_PROPERTY_CACHEABLE        0

#define _DEBUG                               0


#define USBCFG_DEV_ADVANCED_SUSPEND_RESUME    (0)

#define USBCFG_DEV_ADVANCED_CANCEL_ENABLE     (1)

#endif
