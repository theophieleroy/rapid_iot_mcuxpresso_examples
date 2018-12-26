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

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "string.h"
#include "pd_board_config.h"
#include "usb_pd_alt_mode.h"
#include "usb_pd_alt_mode_dp.h"
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
#include "fsl_debug_console.h"
#endif

#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
#if (defined PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _pd_alt_mode_displayport
{
    pd_handle pdHandle;
    void *altModeHandle;
    void *dpBoardChipHandle;
    pd_alt_mode_dp_config_t *dpConfigParam;

    uint32_t taskEvent;
    uint32_t hpdTime;
    uint32_t pdMsgBuffer[7];
    uint32_t pdMsgReceivedBuffer[7];
    volatile uint32_t delayTime;
    volatile uint32_t delayEvents;
    pd_dp_status_obj_t dpSelfStatus;
    pd_dp_status_obj_t dpPartnerStatus;
    pd_dp_configure_obj_t dpConfigure;
    pd_structured_vdm_header_t pdVDMMsgReceivedHeader;
    volatile uint32_t retryCount;
    volatile uint8_t retryCommand;
    uint8_t pdMsgReceivedVDOCount;
    uint8_t selectModeIndex;
    uint8_t occupied;
    uint8_t dpState; /* pd_dp_state_t */
    uint8_t triggerCommand;
    volatile uint8_t dfpCommand;

    uint8_t waitSendResult : 1;
    uint8_t dfpCommandDoing : 1;
} pd_alt_mode_dp_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_DpmAltModeCallback(pd_handle pdHandle, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/

pd_alt_mode_dp_t s_AltModeDisplayPortInstance[PD_CONFIG_MAX_PORT];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_DpDelayRetryCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command, uint32_t delay)
{
    dpInstance->delayTime = delay;
    dpInstance->retryCommand = command;
}

/* DP DFP */
static pd_status_t PD_DpDFPSendCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command)
{
    pd_svdm_command_param_t structuredVDMCommandParam;
    uint32_t vdmCommand = 0;

    if (dpInstance->dfpCommandDoing)
    {
        return kStatus_PD_Error;
    }
    structuredVDMCommandParam.vdmSop = kPD_MsgSOP;
    structuredVDMCommandParam.vdmHeader.bitFields.SVID = DP_SVID;
    structuredVDMCommandParam.vdmHeader.bitFields.vdmType = 1;
    structuredVDMCommandParam.vdmHeader.bitFields.objPos = 0;
    structuredVDMCommandParam.vdmHeader.bitFields.commandType = kVDM_Initiator;

    switch (command)
    {
        case kVDM_DiscoverModes:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            vdmCommand = PD_DPM_CONTROL_DISCOVERY_MODES;
            break;

        case kVDM_EnterMode:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_EnterMode;
            vdmCommand = PD_DPM_CONTROL_ENTER_MODE;
            break;

        case kDPVDM_StatusUpdate:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 1;
            structuredVDMCommandParam.vdoData = (uint32_t *)&dpInstance->dpSelfStatus;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_StatusUpdate;
            structuredVDMCommandParam.vendorVDMNeedResponse = 1;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kDPVDM_Configure:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 1;
            structuredVDMCommandParam.vdoData = (uint32_t *)&dpInstance->dpConfigure;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_Configure;
            structuredVDMCommandParam.vendorVDMNeedResponse = 1;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kVDM_ExitMode:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_ExitMode;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        default:
            break;
    }

    if (vdmCommand != 0)
    {
        dpInstance->dfpCommandDoing = 1;
        if (PD_Command(dpInstance->pdHandle, vdmCommand, &structuredVDMCommandParam) != kStatus_PD_Success)
        {
            dpInstance->dfpCommandDoing = 0;
            /* wait and retry again */
            PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
    }

    return kStatus_PD_Success;
}

/* DP DFP */
static void PD_DpDFPTrigerCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command)
{
    USB_OSA_SR_ALLOC();

    dpInstance->dfpCommand = command;
    USB_OSA_ENTER_CRITICAL();
    dpInstance->triggerCommand = 1;
    USB_OSA_EXIT_CRITICAL();
    PD_AltModeModuleTaskWakeUp(dpInstance->altModeHandle, dpInstance);
}

/* DP DFP function */
static pd_status_t PD_DpDFPGetModesCheckHaveSupportedMode(pd_alt_mode_dp_t *dpInstance)
{
    if (dpInstance->pdMsgReceivedVDOCount < 1)
    {
        return kStatus_PD_Success;
    }
    dpInstance->selectModeIndex = 0;

#if (defined PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE) && (PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE)
    {
        pd_altmode_dp_modes_sel_t dpModes;
        dpModes.modesCount = dpInstance->pdMsgReceivedVDOCount;
        dpModes.modes = (pd_dp_mode_obj_t *)&(dpInstance->pdMsgReceivedBuffer[0]);
        dpModes.selectIndex = 0;

        PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_SELECT_MODE_AND_PINASSIGN, &dpModes);
        if ((dpModes.selectIndex > 0) && (dpModes.selectIndex <= dpModes.modesCount))
        {
            dpInstance->selectModeIndex = dpModes.selectIndex;
            dpInstance->dpConfigure.bitFields.configureUFPUPin = dpModes.selectPinAssign;
        }
    }
#else
    {
        pd_dp_mode_obj_t modeObj;
        uint8_t configurePin = 0;
        uint8_t index;
        for (index = 0; index < dpInstance->pdMsgReceivedVDOCount; ++index)
        {
            modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[index];
            if ((modeObj.modeVal & 0xFF000000u) || ((modeObj.modeVal & 0x00FFFFFFu) == 0) ||
                ((modeObj.bitFields.portCap & kDPPortCap_UFPD) == 0))
            {
                /* invalid mode */
                continue;
            }

            if (modeObj.bitFields.receptacleIndication)
            {
                /* receptacle */
                configurePin = modeObj.bitFields.UFPDPinSupport;
            }
            else
            {
                configurePin = modeObj.bitFields.DFPDPinSupport;
            }

            if (configurePin & (dpInstance->dpConfigParam->supportPinAssigns))
            {
                dpInstance->selectModeIndex = index + 1;
                break;
            }
        }
    }
#endif

    if ((dpInstance->selectModeIndex > 0) && (dpInstance->selectModeIndex <= dpInstance->pdMsgReceivedVDOCount))
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        uint8_t configurePin = 0;
        pd_dp_mode_obj_t modeObj;

        modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[dpInstance->selectModeIndex - 1];
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        PRINTF("device supported pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
        PRINTF("\r\n");
#endif
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

/* DP DFP function */
static pd_status_t PD_DpDFPConstructConfigure(pd_alt_mode_dp_t *dpInstance)
{
    pd_dp_mode_obj_t modeObj;
    uint8_t setSignal = 0;
    uint8_t configurePin = 0;

    if (dpInstance->selectModeIndex == 0)
    {
        return kStatus_PD_Error;
    }
    modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[dpInstance->selectModeIndex - 1];

#if (defined PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE) && (PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE)
    {
        configurePin = dpInstance->dpConfigure.bitFields.configureUFPUPin;
    }
#else
    /* if prefer multi function, kPinAssign_B and kPinAssign_D has high priority */
    if ((dpInstance->dpPartnerStatus.bitFields.multiFunctionPreferred) ||
        (dpInstance->dpConfigParam->multiFunctionPrefered))
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }
        configurePin &= (kPinAssign_B | kPinAssign_D);
        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_D)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else
            {
            }
        }
    }

    /* multi function is not prefered or don't get kPinAssign_B and kPinAssign_D
     * prefer the 4 lane pin assignment*/
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= (~(kPinAssign_B | kPinAssign_D));
        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_C)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else if (configurePin & kPinAssign_A)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else
            {
            }
        }
    }

    /* get the first one */
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_A)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else if (configurePin & kPinAssign_C)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_D)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else
            {
            }
        }
    }
#endif

    if (configurePin != 0)
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("select pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        else if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        else if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        else if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        else if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
        else
        {
        }
        PRINTF("\r\n");
#endif
        dpInstance->dpConfigure.bitFields.setConfig = kDPConfig_UFPD;
        dpInstance->dpConfigure.bitFields.setSignal = setSignal;
        dpInstance->dpConfigure.bitFields.configureUFPUPin = configurePin;
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

static void PD_DpDFPSetConfigureAsUSB(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpConfigure.bitFields.setConfig = kDPConfig_USB;
    dpInstance->dpConfigure.bitFields.setSignal = kDPSignal_Unspecified;
    dpInstance->dpConfigure.bitFields.configureUFPUPin = kPinAssign_DeSelect;
}

static void PD_DpDFPProcessUFPstatus(pd_alt_mode_dp_t *dpInstance)
{
    uint8_t driverVal = kDPDriver_Low;
    if (dpInstance->dpPartnerStatus.bitFields.HPDInterrupt)
    {
        driverVal = kDPDriver_IRQ;
    }
    else if (dpInstance->dpPartnerStatus.bitFields.HPDState)
    {
        driverVal = kDPDriver_High;
    }
    else
    {
        driverVal = kDPDriver_Low;
    }
    dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle, kBoardChip_ControlHPDValue,
                                                            &driverVal);

    /* Figure 5-4 */
    if ((dpInstance->dpPartnerStatus.bitFields.exitDPModeReq) || (dpInstance->dpPartnerStatus.bitFields.USBConfigReq) ||
        (!(dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)))
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("start exit mode\r\n");
#endif
        driverVal = kDPDriver_Low;
        dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                kBoardChip_ControlHPDValue, &driverVal);
        /* The DFP_U shall issue an Exit Mode command only when the port is configured to be in USB configuration.
         */
        /* Receipt of an Exit Mode command while not configured in USB Configuration indicates an error in the
         * DFP_U. */
        dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                kBoardChip_ControlHPDSetLow, NULL);
        if (dpInstance->dpState == kDPMode_ConfigureDone)
        {
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlSetMuxSaftMode, NULL);
            PD_DpDFPSetConfigureAsUSB(dpInstance);
            PD_DpDFPSendCommand(dpInstance, kDPVDM_Configure);
        }
        else
        {
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlSetMuxUSB3Only, NULL);
            PD_DpDFPSendCommand(dpInstance, kVDM_ExitMode);
        }
    }
}

/* DP UFP function */
static void PD_DpUFPProcessStatusUpdate(pd_alt_mode_dp_t *dpInstance)
{
    /* check dfp's status and update self status */
}

/* DP UFP function */
static pd_status_t PD_DpUFPProcessConfigure(pd_alt_mode_dp_t *dpInstance)
{
    if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
        (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
    {
    }
    else
    {
    }

    return kStatus_PD_Success;
}

static void PD_DpInstanceReset(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpState = kDPMode_Exited;
    dpInstance->taskEvent = 0u;
    dpInstance->dfpCommandDoing = 0;
    uint8_t driverVal = kDPDriver_Low;
    dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle, kBoardChip_ControlHPDValue,
                                                            &driverVal);
    dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                            kBoardChip_ControlSetMuxUSB3Only, NULL);
}

void PD_DPModule1msISR(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

    if (dpInstance->delayTime > 0)
    {
        dpInstance->delayTime--;
        if (dpInstance->delayTime == 0)
        {
            /* PD_DpModuleSetEvent(dpInstance, dpInstance->delayEvents); */
            PD_DpDFPTrigerCommand(dpInstance, dpInstance->retryCommand);
        }
    }
}

/*
 * pdHandle - PD tack handle.
 * altModeHandle - alt mode driver handle.
 * moduleConfig - displayport module configuration parameter.
 * moduleInstance - return the displayport module instance handle
 *
*/
pd_status_t PD_DPInit(pd_handle pdHandle, void *altModeHandle, const void *moduleConfig, void **moduleInstance)
{
    uint32_t index = 0;
    pd_alt_mode_dp_t *dpInstance = NULL;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    for (index = 0; index < sizeof(s_AltModeDisplayPortInstance) / sizeof(pd_alt_mode_dp_t); ++index)
    {
        if (s_AltModeDisplayPortInstance[index].occupied == 0)
        {
            s_AltModeDisplayPortInstance[index].occupied = 1;
            dpInstance = &s_AltModeDisplayPortInstance[index];
            break;
        }
    }

    if (dpInstance == NULL)
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    USB_OSA_EXIT_CRITICAL();
    dpInstance->pdHandle = pdHandle;
    dpInstance->altModeHandle = altModeHandle;
    dpInstance->dpConfigParam = (pd_alt_mode_dp_config_t *)moduleConfig;
    if (dpInstance->dpConfigParam->chipInterface->dpChipInit(&(dpInstance->dpBoardChipHandle), pdHandle,
                                                             (void *)dpInstance->dpConfigParam->boardChipConfig) !=
        kStatus_PD_Success)
    {
        dpInstance->occupied = 0;
        return kStatus_PD_Error;
    }
    PD_DpInstanceReset(dpInstance);

    *moduleInstance = dpInstance;
    return kStatus_PD_Success;
}

pd_status_t PD_DPDeinit(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

    dpInstance->dpConfigParam->chipInterface->dpChipDeinit(dpInstance->dpBoardChipHandle);
    dpInstance->occupied = 0;
    return kStatus_PD_Success;
}

pd_status_t PD_DPControl(void *moduleInstance, uint32_t controlCode, void *controlParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    pd_status_t status = kStatus_PD_Success;

    /* dfp */
    switch (controlCode)
    {
        /* DFP start to enter mode sequence */
        case kAltMode_TriggerEnterMode:
            PD_DpDFPTrigerCommand(dpInstance, kVDM_DiscoverModes);
            break;

        case kAltMode_TriggerExitMode:
            PD_DpDFPTrigerCommand(dpInstance, kVDM_ExitMode);
            /* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_EXIT_MODE_TRIGGER); */
            break;

        case kAltMode_GetModeState:
        {
            if (controlParam == NULL)
            {
                status = kStatus_PD_Error;
            }
            else
            {
                pd_alt_mode_state_t *modeState = (pd_alt_mode_state_t *)controlParam;
                modeState->SVID = 0u;
                modeState->mode = 0;
                if (dpInstance->dpState >= kDPMode_EnterDPDone)
                {
                    modeState->SVID = DP_SVID;
                    modeState->mode = dpInstance->selectModeIndex;
                }
            }
            break;
        }

        default:
            break;
    }

    return status;
}

/* msgSVID: 0 - this msg related event doesn't know SVID. */
pd_status_t PD_DPCallbackEvent(void *moduleInstance, uint32_t processCode, uint16_t msgSVID, void *param)
{
    pd_status_t status = kStatus_PD_Error;
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    uint32_t index = 0;

    if ((msgSVID != 0) && (msgSVID != 0xFF01u))
    {
        return status;
    }

    /* process the msg related events, if not self msg or self shouldn't process this event return error. */
    switch (processCode)
    {
        case kAltMode_Attach:
        case kAltMode_HardReset:
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlSetMuxUSB3Only, NULL);
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlHPDSetLow, NULL);
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_Detach:
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlSetMuxShutDown, NULL);
            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                    kBoardChip_ControlHPDSetLow, NULL);
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_StructedVDMMsgReceivedProcess:
        {
            pd_svdm_command_request_t *svdmRequest = (pd_svdm_command_request_t *)param;
            if (msgSVID == 0xFF01)
            {
                status = kStatus_PD_Success;
                switch (svdmRequest->vdmHeader.bitFields.command)
                {
                    case kVDM_DiscoverModes: /* DP UFP */
                        for (index = 0; index < dpInstance->dpConfigParam->modesCount; ++index)
                        {
                            dpInstance->pdMsgBuffer[index] = dpInstance->dpConfigParam->modesList[index];
                        }
                        svdmRequest->vdoData = (uint32_t *)&dpInstance->pdMsgBuffer[0];
                        svdmRequest->vdoCount = dpInstance->dpConfigParam->modesCount;
                        svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        break;

                    case kVDM_EnterMode: /* DP UFP */
                        dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                        svdmRequest->vdoData = NULL;
                        svdmRequest->vdoCount = 0;
                        if (svdmRequest->vdmHeader.bitFields.objPos <= dpInstance->dpConfigParam->modesCount)
                        {
                            dpInstance->dpState = kDPMode_EnterDPDone;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        break;

                    case kVDM_ExitMode: /* DP UFP */
                        svdmRequest->vdoData = NULL;
                        svdmRequest->vdoCount = 0;
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpState = kDPMode_Exited;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_StatusUpdate: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            PD_DpUFPProcessStatusUpdate(dpInstance);
                            svdmRequest->vdoData = (uint32_t *)&dpInstance->dpSelfStatus;
                            svdmRequest->vdoCount = 1;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                            if (dpInstance->dpState == kDPMode_EnterDPDone)
                            {
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_Configure: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                            if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                                (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                            {
                                if (dpInstance->dpState != kDPMode_ConfigureDone)
                                {
                                    dpInstance->dpState = kDPMode_ConfigureDone;
                                }
                            }
                            else
                            {
                                /* back the state */
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }

                            svdmRequest->vdoData = NULL;
                            svdmRequest->vdoCount = 0;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                            /* after set the displayport signal then ACK */
                            PD_DpUFPProcessConfigure(dpInstance);
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kVDM_Attention: /* DP DFP */
                        if (svdmRequest->vdoCount == 1)
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            /* process DP status */
                            if (dpInstance->dpState == kDPMode_StatusUpdateDone)
                            {
                                if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                                {
                                    dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                        dpInstance->dpBoardChipHandle, kBoardChip_ControlSetMuxSaftMode, NULL);
                                    if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                                    {
                                        PD_DpDFPTrigerCommand(dpInstance, kDPVDM_Configure);
                                    }
                                }
                            }
                            else if (dpInstance->dpState == kDPMode_ConfigureDone)
                            {
/* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                                PRINTF("receive attention\r\n");
#endif
                                PD_DpDFPProcessUFPstatus(dpInstance);
                            }
                            else
                            {
                                /* don't process */
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case kAltMode_StructedVDMMsgSuccess:
        {
            /* ACK msg, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->dfpCommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                dpInstance->dfpCommandDoing = 0;
                status = kStatus_PD_Success;
                switch (svdmResult->vdmCommand)
                {
                    case kVDM_DiscoverModes:
                        dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected = kDFP_D_Connected;
                        dpInstance->pdMsgReceivedVDOCount = svdmResult->vdoCount;
                        for (index = 0; index < svdmResult->vdoCount; ++index)
                        {
                            dpInstance->pdMsgReceivedBuffer[index] = svdmResult->vdoData[index];
                        }
                        dpInstance->pdVDMMsgReceivedHeader.structuredVdmHeaderVal =
                            svdmResult->vdmHeader.structuredVdmHeaderVal;
                        if (PD_DpDFPGetModesCheckHaveSupportedMode(dpInstance) == kStatus_PD_Success)
                        {
                            PD_DpDFPTrigerCommand(dpInstance, kVDM_EnterMode);
                        }
                        break;

                    case kVDM_EnterMode:
                        dpInstance->dpState = kDPMode_EnterDPDone;
                        PD_DpDFPTrigerCommand(dpInstance, kDPVDM_StatusUpdate);
                        break;

                    case kDPVDM_StatusUpdate:
                        dpInstance->dpState = kDPMode_StatusUpdateDone;
                        dpInstance->dpPartnerStatus.statusVal = svdmResult->vdoData[0];
                        if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                        {
                            /* if false, wait attention message */
                            dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                dpInstance->dpBoardChipHandle, kBoardChip_ControlSetMuxSaftMode, NULL);
                            if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                            {
                                PD_DpDFPTrigerCommand(dpInstance, kDPVDM_Configure);
                            }
                        }
                        break;

                    case kDPVDM_Configure:
                        /* 1. configure as DP; 2. configure as USB (exit DP) */
                        if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                            (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                        {
                            dpInstance->dpState = kDPMode_ConfigureDone;
                            if ((dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_C) ||
                                (dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_E))
                            {
                                dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                    dpInstance->dpBoardChipHandle, kBoardChip_ControlSetMuxDP4LANE,
                                    &dpInstance->dpConfigure.configureVal);
                            }
                            else
                            {
                                dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                    dpInstance->dpBoardChipHandle, kBoardChip_ControlSetMuxDP2LANEUSB3,
                                    &dpInstance->dpConfigure.configureVal);
                            }
                            dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                dpInstance->dpBoardChipHandle, kBoardChip_ControlHPDReleaseLow, NULL);
                            /* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
                            PD_DpDFPProcessUFPstatus(dpInstance);
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                            PRINTF("dp configure success\r\n");
#endif
                            PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_MODE_CONFIGURED, NULL);
                        }
                        else if (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_USB)
                        {
                            /* back the state */
                            dpInstance->dpState = kDPMode_StatusUpdateDone;
                            dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                                    kBoardChip_ControlHPDSetLow, NULL);
                            dpInstance->dpConfigParam->chipInterface->dpChipControl(
                                dpInstance->dpBoardChipHandle, kBoardChip_ControlSetMuxUSB3Only, NULL);
                            PD_DpDFPTrigerCommand(dpInstance, kVDM_ExitMode);
                            PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_MODE_UNCONFIGURED, NULL);
                        }
                        break;

                    case kVDM_ExitMode:
                        dpInstance->dpConfigParam->chipInterface->dpChipControl(dpInstance->dpBoardChipHandle,
                                                                                kBoardChip_ControlHPDSetLow, NULL);
                        dpInstance->dpState = kDPMode_Exited;
                        break;

                    default:
                        break;
                }
            }
            break;
        }

        case kAltMode_StructedVDMMsgFail:
        {
            /* NAK/Not_supported/BUSY/time_out, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->dfpCommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                uint32_t command = 0;
                status = kStatus_PD_Success;
                dpInstance->dfpCommandDoing = 0;
                if (svdmResult->vdmCommandResult == kCommandResult_VDMNAK)
                {
                    /* don't support this command */
                    return status;
                }
                if ((svdmResult->vdmCommand == kVDM_DiscoverModes) || (svdmResult->vdmCommand == kVDM_EnterMode) ||
                    (svdmResult->vdmCommand == kVDM_ExitMode) || (svdmResult->vdmCommand == kDPVDM_StatusUpdate) ||
                    (svdmResult->vdmCommand == kDPVDM_Configure))
                {
                    command = svdmResult->vdmCommand;
                }

                if (command != 0)
                {
                    PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
                }
            }
        }

        case kAltMode_UnstructedVDMMsgReceived:
        case kAltMode_UnstructedVDMMsgSentResult:
            /* DP doesn't have this type message */
            break;

        default:
            break;
    }
    return status;
}

/* 1. send msg from self.
 * wait for send result callback (timer); wait for ACK reply msg callback.
 *
 * 2. ACK received VDM msg.
 * ACK is sent in the callback -> task wait the send result (timer);
 *
 * 3. HPD
 * Dock board detect HPD, Host board driver HPD.
*/
void PD_DPTask(void *taskParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)taskParam;

    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    if (dpInstance->triggerCommand)
    {
        dpInstance->triggerCommand = 0;
        USB_OSA_EXIT_CRITICAL();

        if (dpInstance->retryCommand != dpInstance->dfpCommand)
        {
            dpInstance->retryCount = PD_ALT_MODE_COMMAND_RETRY_COUNT;
            dpInstance->retryCommand = dpInstance->dfpCommand;
            PD_DpDFPSendCommand(dpInstance, dpInstance->dfpCommand);
        }
        else
        {
            if (dpInstance->retryCount > 0)
            {
                dpInstance->retryCount--;
                PD_DpDFPSendCommand(dpInstance, dpInstance->dfpCommand);
            }
            else
            {
                /* do hard reset */
                PD_Command(dpInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL);
            }
        }
    }
    else
    {
        USB_OSA_EXIT_CRITICAL();
    }
    //    /* exit critical */
    //    PD_DpHpdDrvierProcess(&dpInstance->hpdDriverInstance);
}

#endif
#endif
