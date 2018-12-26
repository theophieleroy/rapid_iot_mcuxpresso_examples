/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#ifndef __USB_PD_ALT_MODE_H__
#define __USB_PD_ALT_MODE_H__

#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_ALT_MODE_MAX_MODULES (7)
#define PD_ALT_MODE_COMMAND_RETRY_COUNT (3)

#define DP_SVID (0xFF01)
#define PD_ALT_MODE_ERROR_RETRY_WAIT_TIME (10) /* ms */

#define PD_ALT_MODE_EVENT_MODULES (0x000000FFu)
#define PD_ALT_MODE_EVENT_MODULE1 (0x00000001u)
#define PD_ALT_MODE_EVENT_MODULE2 (0x00000002u)
#define PD_ALT_MODE_EVENT_MODULE3 (0x00000004u)
#define PD_ALT_MODE_EVENT_MODULE4 (0x00000008u)
#define PD_ALT_MODE_EVENT_COMMAND (0x00000200u)

typedef void *pd_alt_mode_handle;

typedef enum _pd_alt_mode_control_code
{
    kAltMode_Invalid = 0,
    kAltMode_TriggerEnterMode,
    kAltMode_TriggerExitMode,
    kAltMode_GetModeState,
} pd_alt_mode_control_code_t;

typedef enum _pd_alt_mode_callback_code
{
    kAltMode_EventInvalid = 0,
    kAltMode_Attach,
    kAltMode_Detach,
    kAltMode_HardReset,
    /* receive partner's vdm request msg */
    kAltMode_StructedVDMMsgReceivedProcess,
    /* slef send VMD message, and receive result (ACK) */
    kAltMode_StructedVDMMsgSuccess,
    /* slef send VMD message, (NAK, BUSY) or send success/fail */
    kAltMode_StructedVDMMsgFail,
    kAltMode_UnstructedVDMMsgReceived,
    kAltMode_UnstructedVDMMsgSentResult,
} pd_alt_mode_callback_code_t;

typedef struct _pd_alt_mode_module
{
    uint32_t SVID;
    const void *config; /* customer defined */
} pd_alt_mode_module_t;

typedef struct _pd_dpm_alt_mode_config
{
    /* UFP */
    //    uint32_t *identityData;
    //    uint16_t identityObjectCount;
    /*! #typec_data_function_config_t values
     * kDataConfig_UFP: support alt mode and support UFP.
     * kDataConfig_DFP: support alt mode and support DFP.
     * kDataConfig_DRD: support alt mode and support DFP and UFP.
     * kDataConfig_None: don't support alt mode.
     */
    uint8_t altModeFunction;
    uint8_t moduleCount;
    /* the first module has high priority */
    const pd_alt_mode_module_t *modules;
} pd_alt_mode_config_t;

typedef struct _pd_alt_mode_state
{
    uint32_t mode;
    uint16_t SVID;
} pd_alt_mode_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/* initialize whole alt mode module */
pd_status_t PD_AltModeInit(void);
/* de-initialize whole alt mode module */
pd_status_t PD_AltModeDeinit(void);
/* initialize one alt mode instance related to one pd instance */
pd_status_t PD_AltModeInstanceInit(pd_handle pdHandle,
                                   const pd_alt_mode_config_t *altModeConfig,
                                   pd_alt_mode_handle *altModeHandle);
/* de-initialize one alt mode instance */
pd_status_t PD_AltModeInstanceDeinit(pd_alt_mode_handle altModeHandle);

/* In this implementation, use one task to process all modules' task. so module can use this API to wake up self task to
 * excute. */
void PD_AltModeModuleTaskWakeUp(pd_alt_mode_handle altModeHandle, void *moduleHandle);

/* this function will be called in one 1ms timer ISR */
void PD_AltModeTimer1msISR(void);

/* PD stack callback events */
pd_status_t PD_AltModeCallback(pd_handle pdHandle, uint32_t event, void *param);

/* get the alt mode state, 0 - not in alt mode; 1 - in alt mode */
pd_status_t PD_AltModeState(pd_handle pdHandle, uint8_t *state);

#endif

#endif
