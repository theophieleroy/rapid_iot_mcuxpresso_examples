/*
 * Copyright (c) 2013,2016, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "bootloader/bootloader.h"
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
#include "fsl_flash.h"
#include "memory/src/flash_memory.h"
#else
#include "flashiap_wrapper/fsl_flashiap_wrapper.h"
#include "memory/src/flashiap_memory.h"
#endif
#endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if BL_DEVICE_IS_LPC_SERIES
#include "power/power.h"
#endif
#if BL_FEATURE_ENCRYPTION
#include "aes_security.h"
#endif // #if BL_FEATURE_ENCRYPTION

//! @addtogroup context
//! @{

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Function table for flash driver.
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
const flash_driver_interface_t g_flashDriverInterface = {
    .version = {.name = kFLASH_DriverVersionName,
                .major = kFLASH_DriverVersionMajor,
                .minor = kFLASH_DriverVersionMinor,
                .bugfix = kFLASH_DriverVersionBugfix },
    .flash_init = FLASH_Init,
    .flash_erase_all = FLASH_EraseAll,
#if BL_FEATURE_ERASEALL_UNSECURE
    .flash_erase_all_unsecure = FLASH_EraseAllUnsecure,
#else
    .flash_erase_all_unsecure = NULL,
#endif
    .flash_erase = FLASH_Erase,
    .flash_program = FLASH_Program,
    .flash_get_security_state = FLASH_GetSecurityState,
    .flash_security_bypass = FLASH_SecurityBypass,
    .flash_verify_erase_all = FLASH_VerifyEraseAll,
    .flash_verify_erase = FLASH_VerifyErase,
    .flash_verify_program = FLASH_VerifyProgram,
    .flash_get_property = FLASH_GetProperty,
#if !BL_FEATURE_MIN_PROFILE
    .flash_program_once = FTFx_CMD_ProgramOnce,
    .flash_read_once = FTFx_CMD_ReadOnce,
#if defined(FSL_FEATURE_FLASH_HAS_READ_RESOURCE_CMD) && FSL_FEATURE_FLASH_HAS_READ_RESOURCE_CMD
    .flash_read_resource = FLASH_ReadResource,
#else
    .flash_read_resource = NULL,
#endif
#else
    .flash_program_once = NULL,
    .flash_read_once = NULL,
    .flash_read_resource = NULL,
#endif
    .flash_is_execute_only = FLASH_IsExecuteOnly,
#if BL_FEATURE_FAC_ERASE
    .flash_erase_all_execute_only_segments = FLASH_EraseAllExecuteOnlySegments,
    .flash_verify_erase_all_execute_only_segments = FLASH_VerifyEraseAllExecuteOnlySegments,
#else
    .flash_erase_all_execute_only_segments = NULL,
    .flash_verify_erase_all_execute_only_segments = NULL,
#endif
#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
#if FSL_FEATURE_FLASH_HAS_SET_FLEXRAM_FUNCTION_CMD
    .flash_set_flexram_function = FLASH_SetFlexramFunction,
#else
    .flash_set_flexram_function = NULL,
#endif
    .flash_program_section = FLASH_ProgramSection,
#else
    .flash_set_flexram_function = NULL,
    .flash_program_section = NULL,
#endif
};
#else // BL_DEVICE_IS_LPC_SERIES
const flashiap_driver_interface_t g_flashDriverInterface = {
    .version = {.name = kFLASHIAP_DriverVersionName,
                .major = kFLASHIAP_DriverVersionMajor,
                .minor = kFLASHIAP_DriverVersionMinor,
                .bugfix = kFLASHIAP_DriverVersionBugfix },
    .flash_init = FLASHIAP_Init,
    .flash_erase = FLASHIAP_Erase,
    .flash_program = FLASHIAP_Program,
    .flash_verify_erase = FLASHIAP_VerifyErase,
    .flash_verify_program = FLASHIAP_VerifyProgram,
    .flash_get_property = FLASHIAP_GetProperty,
#if BL_FEATURE_BYPASS_WATCHDOG
    .flash_register_callback = FLASHIAP_SetCallback,
#else
    .flash_register_callback = NULL,
#endif
};
#endif // !BL_DEVICE_IS_LPC_SERIES
#endif // !BL_FEATURE_HAS_NO_INTERNAL_FLASH

#if BL_DEVICE_IS_LPC_SERIES
const power_driver_interface_t  g_powerDriverInterface = {
    .power_set_pll = set_pll,
    .power_set_voltage = set_voltage,
    .power_mode_configure = power_mode_configure,
    .power_set_low_power_mode = set_low_power_mode,
    .power_set_vd_level = set_vd_level,
    .power_set_lpvd_level = set_lpvd_level,
    .power_energy_reclaim = energy_reclaim,
    .power_enter_RBB = enter_RBB,
    .power_enter_NBB = enter_NBB,
    .power_enter_FBB = enter_FBB,
    .power_disable_energy_reclaim = disable_energy_reclaim,
    .power_PVT_interrupt = PVT_interrupt,
    .power_set_pll_with_fracdiv = set_pll_with_fracdiv,
    .power_get_firmware_version = power_get_firmware_version
};
#endif

const aes_driver_interface_t g_aesInterface = {
#if AES_SECURITY_SUPPORTED
    .aes_init = aes_init, .aes_encrypt = aes_encrypt, .aes_decrypt = aes_decrypt
};
#else
    0
};
#endif

//! @brief Copyright string for the bootloader.
const char bootloaderCopyright[] = "Copyright (c) 2013-2016 Freescale Semiconductor, Inc. All rights reserved.";

//! @brief Static API tree.
const bootloader_tree_t g_bootloaderTree = {.runBootloader = bootloader_user_entry,
                                            .version = {.name = kBootloader_Version_Name,
                                                        .major = kBootloader_Version_Major,
                                                        .minor = kBootloader_Version_Minor,
                                                        .bugfix = kBootloader_Version_Bugfix },
                                            .copyright = bootloaderCopyright,
                                            .runtimeContext = &g_bootloaderContext,
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
                                            .flashDriver = &g_flashDriverInterface,
//#else
//                                            .flashDriver = NULL,
#endif // !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if BL_DEVICE_IS_LPC_SERIES
                                            .powerDriver = &g_powerDriverInterface,
#endif
                                            .aesDriver = &g_aesInterface };

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
