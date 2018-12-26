/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#include "bootloader/bl_context.h"
#include "bootloader/bootloader.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "usb_device_stack_interface.h"
#include "usb_msc.h"
#include "fat_directory_entry.h"
#include "usb_error.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <string.h>
#include <stdio.h>
#include "property/property.h"
#include "usb_stack/msd_device/usb_descriptor.h"
#include "usb_class_msc.h"
#include "usb_msc_scsi.h"
#include "usb_descriptor.h"
#include "usb_msd_disk.h"
#include "sbloader/sbloader.h"
#include "sbloader/sb_file_format.h"
#include "sbloader/sbloader.h"
#include "utilities/fsl_assert.h"

#if (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID)

//! @addtogroup usb_msd_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! Set to 1 to dump new root directory entries to the debug console.
#define PRINT_DIR_ENTRIES (0)

//! @brief Status values for SB tranfsers.
typedef enum _sb_transfer_status
{
    kReadyStatus = 0,
    kTransferringStatus,
    kSuccessStatus,
    kFailureStatus
} sb_transfer_status_t;

//! @brief Various constants for the MSD peripheral.
enum _usb_msd_constants
{
    //! Number of sector buffers we use.
    kNumberOfSectorBuffers = 2,

    //! Value used to indicate that the #usb_msd_state_t::lastBufferReadIndex is unset.
    kInvalidBufferReadIndex = ~0
};

//! @brief The various states that a sector buffer can be in.
typedef enum _sector_buffer_status
{
    kBufferFree = 0, //!< Buffer contains no data.
    kBufferPending,  //!< A write is pending for this buffer.
    kBufferFilled,   //!< Buffer has data that is waited to be processed.
    kBufferInUse     //!< The buffer's data is being processed by the sbloader.
} sector_buffer_status_t;

//! @brief State information for the USB MSD peripheral.
//!
//! The sector buffers come first so they will be word aligned.
typedef struct _usb_msd_state
{
    msd_handle_t app_handle;
    uint32_t start_app;
    uint16_t speed;
    uint8_t sectorBuffer[kNumberOfSectorBuffers][kDiskSectorSize];     //!< Buffer to hold SB file sectors.
    sector_buffer_status_t sectorBufferStatus[kNumberOfSectorBuffers]; //!< Current state of the sector buffers.
    struct
    {
        uint32_t isEnumerated : 1; //!< Whether USB has completed enumeration.
        uint32_t isActive : 1;     //!< Set to true when the first SB file is transferred.
        uint32_t isDiskLocked : 1; //!< Disk locked flag.
        uint8_t read_write_error;
        uint32_t isTransferring : 1;   //!< Whether an SB file transfer has been started.
        uint32_t isLoaderInited : 1;   //!< Flag to indicate that the SB loader has been initialized.
        uint32_t isProcessing : 1;     //!< Set to true while the SB file is being processed.
        uint32_t isReceivePending : 1; //!< Whether a sector write is being held off until a buffer is free.
        uint32_t _pad : 25;
    } flags;                             //!< Various flags.
    uint32_t infoFileLength;             //!< Size in bytes of the info.txt file.
    uint32_t nextSector;                 //!< The next expected sector write for the SB file transfer.
    uint32_t nextBufferWriteIndex;       //!< The next sector buffer index to write into. Toggles between 0 and 1.
    uint32_t lastBufferReadIndex;        //!< The last sector buffer index that was read from. Toggles between 0 and 1.
    uint32_t remainingFileLength;        //!< Total length in bytes of the SB file being transferred.
    uint32_t dataBytesAvailable;         //!< Number of bytes available in sectorBuffer.
    sb_transfer_status_t transferStatus; //!< Status to present in the status.txt file.
} usb_msd_state_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

// static void usb_msd_app_callback(uint8_t controller_ID, uint8_t event_type, void* val);
// static void usb_msd_event_callback(uint8_t controller_ID, uint8_t event_type, void* val);
static void usb_msd_app_callback(uint8_t event_type, void *val, void *arg);
extern uint8_t usb_msd_app_class_callback(
    uint8_t event_type, uint16_t value, uint8_t **data, uint32_t *size, void *arg);

uint32_t g_msc_bulk_out_buff[128];
uint32_t g_msc_bulk_in_buff[128];

static bool usb_msd_poll_for_activity(const peripheral_descriptor_t *self);
static status_t usb_msd_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void usb_msd_full_shutdown(const peripheral_descriptor_t *self);

static void usb_msd_read_sector(uint32_t sector, lba_app_struct_t *lbaData);
// static void usb_msd_prepare_for_write_sector(uint32_t sector, const lba_app_struct_t * lbaData);
static void usb_msd_write_sector(uint32_t sector, const lba_app_struct_t *lbaData);

#if PRINT_DIR_ENTRIES
static void print_dir_entry(uint32_t i, const fat_directory_entry_t *dir);
static void usb_msd_scan_directory(uint32_t sector, const uint8_t *data);
#endif // PRINT_DIR_ENTRIES

static bool is_valid_sb_file_header(const uint8_t *data, uint32_t *fileLength);
static void usb_msd_start_transfer(uint32_t length);
static void usb_msd_pump(const peripheral_descriptor_t *self);
static status_t feed_sbloader(void);

static void format_hex_string(char *s, uint32_t value);
static void usb_msd_update_info_file(uint8_t *buffer);

/****************************************************************************
 * Global Variables
 ****************************************************************************/
/* Add all the variables needed for disk.c to this structure */
extern usb_desc_request_notify_struct_t desc_callback;
usb_application_callback_struct_t msc_application_callback;
usb_vendor_req_callback_struct_t vend_req_callback;
usb_class_specific_callback_struct_t class_specific_callback;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Used by format_hex_string().
static const char kHexChars[] = "0123456789abcdef";

//! @brief Messages corresponding to the various transfer statuses.
static const char *const kStatusMessages[] = {
    "Ready\r\n",        // kReadyStatus
    "Transferring\r\n", // kTransferringStatus
    "Success\r\n",      // kSuccessStatus
    "Failure\r\n"       // kFailureStatus
};

//! @brief Format for contents of the info.txt file.
static const char kInfoFileFormat[] =
    "Kinetis Bootloader %c%d.%d.%d\r\n"
    "\r\n"
    "System device ID: %s\r\n"
    "Flash size: %u bytes\r\n"
    "Flash range: %s-%s\r\n"
    "Flash sector size: %u bytes\r\n"
    "Flash blocks: %u\r\n"
    "RAM size: %u bytes\r\n"
    "RAM range: %s-%s\r\n"
    "Reserved region 0: %s-%s\r\n"
    "Reserved region 1: %s-%s\r\n"
    "Verify writes: %s\r\n"
    "Check reserved regions: %s\r\n"
    "Boot config present: %s\r\n"
    "Peripheral detection timeout: %u ms\r\n"
    "CPU clock: %u Hz\r\n";

const peripheral_control_interface_t g_usbMsdControlInterface = {.pollForActivity = usb_msd_poll_for_activity,
                                                                 .init = usb_msd_full_init,
                                                                 .shutdown = usb_msd_full_shutdown,
                                                                 .pump = usb_msd_pump };

//! @brief Current state of the USB MSD disk.
static usb_msd_state_t s_diskState;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief Determine when MSD becomes the active peripheral.
 *
 * Normally, MSD will just run in the background. If no communication happens on other
 * peripherals, the bootloader peripheral detection loop will run indefinitely. But when a user
 * drags a file onto the disk and execution of the .sb file begins, we indicate that MSD is
 * the active peripheral so other peripherals will be shut down. This prevents the possibility
 * of having two command streams running simultaneously.
 */
bool usb_msd_poll_for_activity(const peripheral_descriptor_t *self)
{
    return s_diskState.flags.isEnumerated && s_diskState.flags.isActive;
}

/*!
 * @brief This function handles the callback
 *
 * This function is called from the class layer whenever reset occurs or enum
 * is complete. after the enum is complete this function sets a variable so
 * that the application can start
 *
 * @param controller_ID  Controller ID
 * @param event_type     value of the event
 * @param val            gives the configuration value
 */
// void usb_msd_app_callback(uint8_t controller_ID, uint8_t event_type, void* val)
void usb_msd_app_callback(uint8_t event_type, void *val, void *arg)
{
    switch (event_type)
    {
        case USB_DEV_EVENT_BUS_RESET:
            s_diskState.flags.isEnumerated = false;
            if (USB_OK == USB_Class_MSC_Get_Speed(s_diskState.app_handle, &s_diskState.speed))
            {
                USB_Desc_Set_Speed(s_diskState.app_handle, s_diskState.speed);
            }
            break;
        case USB_DEV_EVENT_ENUM_COMPLETE:
            s_diskState.flags.isEnumerated = true;
            s_diskState.start_app = TRUE;
            break;
        case USB_DEV_EVENT_ERROR:
            break;
        case USB_MSC_DEVICE_GET_SEND_BUFF_INFO:
            if (NULL != val)
            {
                *((uint32_t *)val) = (uint32_t)kDiskSectorSize;
            }
            break;
        case USB_MSC_DEVICE_GET_RECV_BUFF_INFO:
            if (NULL != val)
            {
                *((uint32_t *)val) = (uint32_t)kDiskSectorSize;
            }
            break;
        default:
            break;
    }

    return;
}

uint8_t usb_msd_app_class_callback(uint8_t event_type, uint16_t value, uint8_t **data, uint32_t *size, void *arg)
{
    lba_app_struct_t *lba_data_ptr;
    uint8_t *prevent_removal_ptr;
    device_lba_info_struct_t *device_lba_info_ptr;
    uint8_t error = USB_OK;
    uint32_t sector;

    if (s_diskState.flags.read_write_error)
    {
        return USBERR_ERROR;
    }

    switch (event_type)
    {
        case USB_DEV_EVENT_DATA_RECEIVED:
            /* Add User defined code -- if required*/
            lba_data_ptr = (lba_app_struct_t *)size;
            if (data != NULL)
            {
                sector = lba_data_ptr->offset / kDiskSectorSize;
                usb_msd_write_sector(sector, lba_data_ptr);
            }
            break;
        case USB_DEV_EVENT_SEND_COMPLETE:
            break;
        case USB_MSC_START_STOP_EJECT_MEDIA:
            /*  Code to be added by user for starting, stopping or
             ejecting the disk drive. e.g. starting/stopping the motor in
             case of CD/DVD*/
            break;
        case USB_MSC_DEVICE_READ_REQUEST:
            lba_data_ptr = (lba_app_struct_t *)size;

            // read data from mass storage device to driver buffer
            if (data != NULL)
            {
                *data = (uint8_t *)g_msc_bulk_in_buff;
            }
            lba_data_ptr->buff_ptr = (uint8_t *)g_msc_bulk_in_buff;
            sector = lba_data_ptr->offset / kDiskSectorSize;
            usb_msd_read_sector(sector, lba_data_ptr);

            break;
        case USB_MSC_DEVICE_WRITE_REQUEST:
            lba_data_ptr = (lba_app_struct_t *)size;
            // write data from mass storage device to driver buffer
            if (data != NULL)
            {
                *data = (uint8_t *)g_msc_bulk_out_buff;
            }
            sector = lba_data_ptr->offset / kDiskSectorSize;
            usb_msd_write_sector(sector, lba_data_ptr);

            break;
        case USB_MSC_DEVICE_FORMAT_COMPLETE:
            break;
        case USB_MSC_DEVICE_REMOVAL_REQUEST:
            prevent_removal_ptr = (uint8_t *)size;
            if (SUPPORT_DISK_LOCKING_MECHANISM)
            {
                s_diskState.flags.isDiskLocked = *prevent_removal_ptr;
            }
            else if ((!SUPPORT_DISK_LOCKING_MECHANISM) && (!(*prevent_removal_ptr)))
            {
                /*there is no support for disk locking and removal of medium is enabled*/
                /* code to be added here for this condition, if required */
            }
            break;
        case USB_MSC_DEVICE_GET_INFO:
            device_lba_info_ptr = (device_lba_info_struct_t *)size;
            device_lba_info_ptr->total_lba_device_supports = kDiskTotalLogicalBlocks;
            device_lba_info_ptr->length_of_each_lab_of_device = kDiskSectorSize;
            device_lba_info_ptr->num_lun_supported = kDiskLogicalUnits;
            break;
        default:
            break;
    }
    return error;
}

status_t usb_msd_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    // Not used for USB
    (void)function;

    // Get the length of the info.txt file by writing it to a buffer temporarily.
    usb_msd_update_info_file(s_diskState.sectorBuffer[0]);
    uint32_t length = strlen((char *)s_diskState.sectorBuffer[0]);

    // Init the state info. This also clears the sector buffers.
    memset(&s_diskState, 0, sizeof(s_diskState));

    s_diskState.infoFileLength = length;

    // Init the usb clock
    usb_clock_init();

    // Clear any pending interrupts on USB
    NVIC_DisableIRQ(USB0_IRQn);
    NVIC_ClearPendingIRQ(USB0_IRQn);

    // Update VID/PID and strings if specified in bootloader config data.
    //     property_store_t * propertyStore = g_bootloaderContext.propertyInterface->store;
    //
    //     if (propertyStore->configurationData.usbVid != 0xFFFF)
    //     {
    //         g_device_descriptor[kUsbDescriptorIndex_VidLow] = ((propertyStore->configurationData.usbVid) & 0xFF);
    //         g_device_descriptor[kUsbDescriptorIndex_VidHigh] = ((propertyStore->configurationData.usbVid >> 8) &
    //         0xFF);
    //     }
    //
    //     if (propertyStore->configurationData.usbPid != 0xFFFF)
    //     {
    //         g_device_descriptor[kUsbDescriptorIndex_PidLow] = ((propertyStore->configurationData.usbPid) & 0xFF);
    //         g_device_descriptor[kUsbDescriptorIndex_PidHigh] = ((propertyStore->configurationData.usbPid >> 8) &
    //         0xFF);
    //     }
    //
    //     if (propertyStore->configurationData.usbStringsPointer != 0xFFFFFFFF)
    //     {
    //         g_lang_ptr = (struct _USB_ALL_LANGUAGES *)propertyStore->configurationData.usbStringsPointer;
    //     }
    //     else
    //     {
    //         g_lang_ptr = &g_languages;
    //     }

    // Init HID class driver.
    msc_application_callback.callback = usb_msd_app_callback;
    msc_application_callback.arg = &s_diskState.app_handle;

    /* Register the callbacks to lower layers */
    msc_config_struct_t msdConfig;
    msdConfig.msc_application_callback = msc_application_callback;
    msdConfig.vendor_req_callback = vend_req_callback;
    msdConfig.class_specific_callback.callback = usb_msd_app_class_callback;
    msdConfig.class_specific_callback.arg = &s_diskState.app_handle;
    msdConfig.desc_callback_ptr = &desc_callback;

    // Init MSD class driver.
    uint8_t status = USB_Class_MSC_Init(self->instance, &msdConfig, &s_diskState.app_handle);

    // Enable interrupts from USB module
    NVIC_EnableIRQ(USB0_IRQn);

    return status;
}

//! @brief Shutdown handler for MSD.
//!
//! The USB peripheral is only actually shutdown if we are shutting down due to a
//! jump finalization.
//!
//! @todo This is broken, because we need to actually shut down for any peripheral
//!     that causes a shutdown/exit of the bootloader, not just MSD. But we want to
//!     prevent MSD from shutting down earlier, when active peripheral detection
//!     is exiting.
void usb_msd_full_shutdown(const peripheral_descriptor_t *self)
{
    // Make sure we are clocking to the peripheral to ensure there
    // are no bus errors
    if ((SIM_SCGC4 & SIM_SCGC4_USBOTG_MASK))
    {
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB0_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB0_IRQn);

        // Turn off clocking to USB
        SIM_SCGC4 &= ~SIM_SCGC4_USBOTG_MASK;
    }
}

void usb_msd_read_sector(uint32_t sector, lba_app_struct_t *lbaData)
{
    //     debug_printf("read:%u\r\n", sector);

    // Clear the sector contents (all zeroes).
    memset(lbaData->buff_ptr, 0, lbaData->size);

    // Search for a sector entry in our table.
    const sector_info_t *sectorInfo = g_msdDiskSectors;
    for (; sectorInfo->data; ++sectorInfo)
    {
        if (sectorInfo->sector == sector)
        {
            // Copy sector data into output buffer.
            memcpy(lbaData->buff_ptr, sectorInfo->data, sectorInfo->length);
            break;
        }
    }

    // Special handling for certain sectors.
    switch (sector)
    {
        case kPbsSector:
            // Fill in signature bytes for PBS, so we don't have to have the full 512 bytes as const data.
            lbaData->buff_ptr[510] = 0x55;
            lbaData->buff_ptr[511] = 0xaa;
            break;

        case kRootDir1Sector:
        {
            // Update info.txt and status.txt file sizes in root directory.
            fat_directory_entry_t *rootDir = (fat_directory_entry_t *)lbaData->buff_ptr;

            rootDir[kInfoFileDirEntry].entry.fileSize = s_diskState.infoFileLength;

            const char *statusMessage = kStatusMessages[s_diskState.transferStatus];
            rootDir[kStatusFileDirEntry].entry.fileSize = strlen(statusMessage);

            break;
        }

        case kInfoFileSector:
            usb_msd_update_info_file(lbaData->buff_ptr);
            break;

        case kStatusFileSector:
        {
            const char *statusMessage = kStatusMessages[s_diskState.transferStatus];
            uint32_t statusLength = strlen(statusMessage);
            memcpy(lbaData->buff_ptr, statusMessage, statusLength);
            break;
        }
    }
}

bool is_valid_sb_file_header(const uint8_t *data, uint32_t *fileLength)
{
    const sb_image_header_t *header = (const sb_image_header_t *)data;

    if ((header->m_signature == BOOT_SIGNATURE) && (header->m_signature2 == BOOT_SIGNATURE2) &&
        (header->m_majorVersion == SB_FILE_MAJOR_VERSION))
    {
        assert(fileLength);
        *fileLength = header->m_imageBlocks * BYTES_PER_CHUNK;

        return true;
    }
    else
    {
        return false;
    }
}
/*
void usb_msd_prepare_for_write_sector(uint32_t sector, const lba_app_struct_t * lbaData)
{
//     debug_printf("prepwrite:%u\r\n", sector);

    // By default we receive into the buffer provided by the MSC stack.
    uint8_t * receiveBuffer = NULL;

    // Is this a sector we're interested in?
    if (s_diskState.flags.isTransferring && sector == s_diskState.nextSector)
    {
        // If the next sector buffer is in use we have to hold off the receive.
        if (s_diskState.sectorBufferStatus[s_diskState.nextBufferWriteIndex] != kBufferFree)
        {
            debug_printf("prepare_for_write_sector: receive pending on sector %u, buffer %u, state=%d\r\n", sector,
s_diskState.nextBufferWriteIndex, (int)s_diskState.sectorBufferStatus[s_diskState.nextBufferWriteIndex]);

            s_diskState.flags.isReceivePending = true;

            return;
        }
        else
        {
            // Go ahead and mark the next buffer as in use.
            s_diskState.sectorBufferStatus[s_diskState.nextBufferWriteIndex] = kBufferPending;
            receiveBuffer = s_diskState.sectorBuffer[s_diskState.nextBufferWriteIndex];

            debug_printf("prepare_for_write_sector: buffer %u is pending for sector %u\r\n",
s_diskState.nextBufferWriteIndex, sector);
        }
    }

    // Go ahead and receive the data immediately.
    USB_MSC_Initiate_Pending_Receive(CONTROLLER_ID, receiveBuffer);
}
*/
void usb_msd_write_sector(uint32_t sector, const lba_app_struct_t *lbaData)
{
//     debug_printf("write:%u\r\n", sector);

#if PRINT_DIR_ENTRIES
    if (sector == kRootDir1Sector || sector == kRootDir2Sector)
    {
        debug_printf("Received new root directory sector (%u)\r\n", sector);

        // Examine updated directory contents.
        usb_msd_scan_directory(sector, lbaData->buff_ptr);
    }
    else
#endif // PRINT_DIR_ENTRIES
        if (sector >= kFirstUnusedSector)
    {
        bool saveSbData = false;

        if (!s_diskState.flags.isTransferring)
        {
            if (s_diskState.flags.isProcessing)
            {
                debug_printf("write_sector: ignoring sector %u write while SB processing continues...\r\n", sector);
                return;
            }

            // Check if this is an .sb file by looking for signatures in the header. This call
            // also returns the total size in bytes of the SB file.
            uint32_t sbLength;
            if (is_valid_sb_file_header(lbaData->buff_ptr, &sbLength))
            {
                debug_printf("write_sector: started receiving SB file on sector %u (length=%u bytes)\r\n", sector,
                             sbLength);

                // This is a valid .sb file header
                usb_msd_start_transfer(sbLength);

                // Save the sector data.
                saveSbData = true;

                // For the first sector of the transfer, we have to copy the data from the
                // MSC stack's buffer into our own. For following sectors, we can tell the MSC
                // stack to DMA directly into one of our buffers.
                memcpy(s_diskState.sectorBuffer[s_diskState.nextBufferWriteIndex], lbaData->buff_ptr, lbaData->size);
            }
        }
        else if (sector == s_diskState.nextSector)
        {
            //             debug_printf("Received next SB file sector (%u)\r\n", sector);

            saveSbData = true;
        }
        else
        {
            // Out of order sector write.
            //             debug_printf("Out of order sector write during transfer (%u)\r\n", sector);
        }

        // Save the SB file contents for processing by the main bootloader run loop (outside
        // of interrupt context).
        if (saveSbData)
        {
            // Update state information.
            uint32_t count = MIN(s_diskState.remainingFileLength, lbaData->size);
            s_diskState.dataBytesAvailable += count;
            s_diskState.remainingFileLength -= count;
            s_diskState.sectorBufferStatus[s_diskState.nextBufferWriteIndex] = kBufferFilled;
            s_diskState.nextBufferWriteIndex ^= 1;
            s_diskState.nextSector = sector + 1;

            if (s_diskState.remainingFileLength == 0)
            {
                s_diskState.flags.isTransferring = false;
            }

            debug_printf("write_sector: %u bytes (sector=%u, buffer=%u, avail=%u, remain=%u)\r\n", count, sector,
                         s_diskState.nextBufferWriteIndex ^ 1, s_diskState.dataBytesAvailable,
                         s_diskState.remainingFileLength);
        }
    }
}

#if PRINT_DIR_ENTRIES
void print_dir_entry(uint32_t i, const fat_directory_entry_t *dir)
{
    char s[12] = { 0 };
    memcpy(s, dir->entry.name, 11);
    if (dir->entry.attributes == kLongNameAttribute)
    {
        debug_printf("Root dir entry %u: long name 0x%c%c='%c%c%c%c%c%c%c%c%c%c%c%c%c'\r\n", i,
                     kHexChars[(dir->longName.order >> 4) & 0xf], kHexChars[dir->longName.order & 0xf],
                     dir->longName.name1[0], dir->longName.name1[1], dir->longName.name1[2], dir->longName.name1[3],
                     dir->longName.name1[4], dir->longName.name2[0], dir->longName.name2[1], dir->longName.name2[2],
                     dir->longName.name2[3], dir->longName.name2[4], dir->longName.name2[5], dir->longName.name3[0],
                     dir->longName.name3[1]);
    }
    else
    {
        debug_printf("Root dir entry %u: '%s' (attr=0x%c%c, cluster=%d, size=%u)\r\n", i, s,
                     kHexChars[(dir->entry.attributes >> 4) & 0xf], kHexChars[dir->entry.attributes & 0xf],
                     (int)dir->entry.firstClusterLow, dir->entry.fileSize);
    }
}

void usb_msd_scan_directory(uint32_t sector, const uint8_t *data)
{
    uint32_t i = 0;

    // If scanning the first root dir sector, we can skip our own canned entries.
    if (sector == kRootDir1Sector)
    {
        i = kFirstUnusedDirEntry;
    }

    // Examine directory entries.
    for (; i < (kDiskSectorSize / sizeof(fat_directory_entry_t)); ++i)
    {
        const fat_directory_entry_t *dir = &((const fat_directory_entry_t *)data)[i];

        // Exit loop if there are no more entries (first byte of name is 0).
        if (dir->entry.name[0] == 0)
        {
            break;
        }

        // Skip entries marked as free.
        if (dir->entry.name[0] == kFreeEntryMarkerByte)
        {
            continue;
        }

        print_dir_entry(i, dir);

        // Check file extension.
        //         char ext0 = dir->entry.name[8];
        //         char ext1 = dir->entry.name[9];
        //         char ext2 = dir->entry.name[10];
        //         if ((ext0 == 'S' || ext0 == 's') && (ext1 == 'B' || ext1 == 'b') && (ext2 == ' ') &&
        //         (dir->entry.name[0] != '_'))
        //         {
        //         }
    }
}
#endif // PRINT_DIR_ENTRIES

//! @brief Prepare for a new SB file transfer.
void usb_msd_start_transfer(uint32_t length)
{
    s_diskState.flags.isActive = true;
    s_diskState.flags.isTransferring = true;
    s_diskState.flags.isLoaderInited = false;
    s_diskState.flags.isProcessing = true;
    s_diskState.flags.isReceivePending = false;
    s_diskState.nextBufferWriteIndex = 0;
    s_diskState.lastBufferReadIndex = kInvalidBufferReadIndex;
    s_diskState.dataBytesAvailable = 0;
    s_diskState.remainingFileLength = length;
    s_diskState.transferStatus = kTransferringStatus;

    // Clear sector buffer statuses.
    memset(s_diskState.sectorBufferStatus, 0, sizeof(s_diskState.sectorBufferStatus));
}

//! @brief Run the sbloader state machine.
//!
//! This function is called repeatedly by the main application loop. We use it to run the
//! sbloader state machine from non-interrupt context.
void usb_msd_pump(const peripheral_descriptor_t *self)
{
    status_t status;

    // Init the sb loader state machine.
    if (!s_diskState.flags.isLoaderInited)
    {
        sbloader_init();
        s_diskState.flags.isLoaderInited = true;
    }

    if (s_diskState.flags.isProcessing)
    {
        // Process the SB file. This loop will exit under four conditions:
        //  - Error during SB file command execution.
        //  - Underrun: more data needed.
        //  - Jump command encountered.
        do
        {
            status = feed_sbloader();
        } while (status == kStatus_Success);

        // kStatusRomLdrDataUnderrun means need more data. This is OK to continue on.
        if (status == kStatusRomLdrDataUnderrun)
        {
            status = kStatus_Success;
        }

        // Stop saving SB data if we got an error.
        if (status != kStatus_Success)
        {
            s_diskState.flags.isProcessing = false;

            if (status == kStatusRomLdrSectionOverrun)
            {
                s_diskState.transferStatus = kSuccessStatus;
                debug_printf("SB processing done: section overrun\r\n");
            }
            else if (status == kStatus_AbortDataPhase)
            {
                s_diskState.transferStatus = kSuccessStatus;

                debug_printf("SB processing done: hit jump command\r\n");

                sbloader_finalize_jump();
            }
            else
            {
                s_diskState.transferStatus = kFailureStatus;
                debug_printf("SB processing done: error %d\r\n", status);
            }
        }
    }
}

//! @brief Push data that has been received from host during data phase.
status_t feed_sbloader(void)
{
    status_t status = kStatus_Success;

    // Disable interrupts while we access shared globals.
    __disable_irq();

    // Free the last buffer we returned.
    if ((s_diskState.lastBufferReadIndex != kInvalidBufferReadIndex) &&
        (s_diskState.sectorBufferStatus[s_diskState.lastBufferReadIndex] == kBufferInUse))
    {
        s_diskState.sectorBufferStatus[s_diskState.lastBufferReadIndex] = kBufferFree;

        debug_printf("feed_sbloader: buffer %u is free\r\n", s_diskState.lastBufferReadIndex);

        // Start the next sector transfer if we were waiting for a buffer to become free.
        /*        if (s_diskState.flags.isReceivePending)
                {
                    assert(s_diskState.nextBufferWriteIndex == s_diskState.lastBufferReadIndex);
                    debug_printf("feed_sbloader: initiating pending receive\r\n");

                    // Initiate the USB transaction to write the sector directly into our buffer.
                    USB_MSC_Initiate_Pending_Receive(CONTROLLER_ID,
           s_diskState.sectorBuffer[s_diskState.nextBufferWriteIndex]);
                    s_diskState.flags.isReceivePending = false;
                }
        */
    }

    // Get the number of bytes we can give to sbloader. If there is no available data,
    // then there is nothing else for us to do here and we can return an underrun meaning waiting for more data
    uint32_t available = MIN(s_diskState.dataBytesAvailable, kDiskSectorSize);
    if (!available)
    {
        __enable_irq();
        return kStatusRomLdrDataUnderrun;
    }

    // Return last filled sector buffer.
    uint32_t nextReadIndex;
    if (s_diskState.lastBufferReadIndex == ~0)
    {
        nextReadIndex = 0;
    }
    else
    {
        nextReadIndex = s_diskState.lastBufferReadIndex ^ 1;
    }

    assert(s_diskState.sectorBufferStatus[nextReadIndex] == kBufferFilled);

    // Update state.
    s_diskState.sectorBufferStatus[nextReadIndex] = kBufferInUse;
    s_diskState.lastBufferReadIndex = nextReadIndex;
    s_diskState.dataBytesAvailable -= available;

    // Give the data to sbloader.
    status = sbloader_pump(s_diskState.sectorBuffer[nextReadIndex], available);

    debug_printf("feed_sbloader: avail=%u, buffer=%d, new avail=%u\r\n", available, nextReadIndex,
                 s_diskState.dataBytesAvailable);

    // Restore interrutps.
    __enable_irq();

    return status;
}

void format_hex_string(char *s, uint32_t value)
{
    s[0] = '0';
    s[1] = 'x';
    s[10] = 0;

    uint32_t i;
    for (i = 0; i < 8; ++i)
    {
        s[i + 2] = kHexChars[(value >> (28 - (i * 4))) & 0xf];
    }
}

void usb_msd_update_info_file(uint8_t *buffer)
{
    property_store_t *store = g_bootloaderContext.propertyInterface->store;
    standard_version_t version = store->bootloaderVersion;

    char hexBuffers[9][11];
    format_hex_string(hexBuffers[1], store->flashStartAddress);
    format_hex_string(hexBuffers[2], store->flashStartAddress + store->flashSizeInBytes - 1);
    format_hex_string(hexBuffers[3], store->ramStartAddress);
    format_hex_string(hexBuffers[4], store->ramStartAddress + store->ramSizeInBytes - 1);
    format_hex_string(hexBuffers[5], store->reservedRegions[0].startAddress);
    format_hex_string(hexBuffers[6], store->reservedRegions[0].endAddress);
    format_hex_string(hexBuffers[7], store->reservedRegions[1].startAddress);
    format_hex_string(hexBuffers[8], store->reservedRegions[1].endAddress);

    sprintf((char *)buffer, kInfoFileFormat, version.name, version.major, version.minor, version.bugfix, // Version
            hexBuffers[0],                // System device ID
            store->flashSizeInBytes,      // Flash size
            hexBuffers[1], hexBuffers[2], // Flash range
            store->flashSectorSize,       // Flash sector size
            store->flashBlockCount,       // Flash block count
            store->ramSizeInBytes,        // RAM size
            hexBuffers[3], hexBuffers[4], // RAM range
            hexBuffers[5], hexBuffers[6], // Reserved range 0
            hexBuffers[7], hexBuffers[8], // Reserved range 1
            store->verifyWrites ? "yes" : "no", store->validateRegions ? "yes" : "no",
            (store->configurationData.tag == kPropertyStoreTag) ? "yes" : "no",
            (store->configurationData.peripheralDetectionTimeoutMs != 0xffff) ?
                store->configurationData.peripheralDetectionTimeoutMs :
                BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT,
            SystemCoreClock);
}

//! @}

#endif // BL_CONFIG_USB_HID

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
