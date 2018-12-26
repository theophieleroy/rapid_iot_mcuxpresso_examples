/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "eif_adapter.h"
#if ((defined(CFG_BLE_EIF)) && (CFG_BLE_EIF == kEIF_Uart))
#include "uart_adapter.h"
#elif((defined(CFG_BLE_EIF)) && (CFG_BLE_EIF == kEIF_Usb_Vcom))
#include "usb_vcom_adapter.h"
#endif
#ifdef NXP_BLE_STACK
#include "hci_transport.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct
{
    void (*rx_callback)(void *, uint8_t);
    void *rx_dummy;
    void (*tx_callback)(void *, uint8_t);
    void *tx_dummy;
} eif_env_t;

#ifdef NXP_BLE_STACK
typedef enum _eif_read_state
{
    /* Command Type Received*/
    CMDTYPE_RCVD,
    /* Opcode Received */
    OPCODE_RCVD
} eif_read_state_t;
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*Used to store callback information*/
static eif_env_t s_eifEnv;

#ifdef NXP_BLE_STACK
/* Global variables for HCI communication with QN908x Controller */
hciPacketType_t *hciPktType; /* Packet type of HCI command from Host */
uint32_t hciPktHdrLen;       /*  HCI packet header length */

extern uint8_t hciRxBuf[MAX_HCI_CMD_LEN]; /* HCI command receive buffer*/
extern volatile int hciCmdRcvdFlag;       /* Flag to indicate HCI command reception from host */
extern uint16_t hciPktSize;               /* HCI command packet size */
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/

void EIF_RxCallback(void)
{
    /* Retrieve callback pointer */
    void (*callback)(void *, uint8_t) = s_eifEnv.rx_callback;
    void *data = s_eifEnv.rx_dummy;

    if (callback)
    {
        /* Clear callback pointer */
        s_eifEnv.rx_callback = NULL;
        s_eifEnv.rx_dummy = NULL;

        /* Wakeup BLE module if it's in sleep mode */
        if (!APP_BleIsActive())
        {
            APP_BleSoftwareWakeup();
        }

        /* Call handler */
        callback(data, 0);
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/*! @brief BLE stack external interface transmit callback.*/
void EIF_TxCallback(void)
{
    /* Retrieve callback pointer */
    void (*callback)(void *, uint8_t) = s_eifEnv.tx_callback;
    void *data = s_eifEnv.tx_dummy;

    if (callback)
    {
        /* Clear callback pointer */
        s_eifEnv.tx_callback = NULL;
        s_eifEnv.tx_dummy = NULL;

        /* Call handler */
        callback(data, 0);
    }
    else
    {
        ASSERT_ERR(0);
    }
}

void EIF_Init(void)
{
#if ((defined(CFG_BLE_EIF)) && (CFG_BLE_EIF == kEIF_Uart))
    ADAPTER_UartInit();
#elif((defined(CFG_BLE_EIF)) && (CFG_BLE_EIF == kEIF_Usb_Vcom))
    ADAPTER_UsbInit();
#endif
}

/*!
* \brief EIF_Read() is invoked by the controller multiple times. EIF_RxCallback() calls controller stack and controller
* stack invokes EIF_Read().
* First  : During ble initialization where the ble controller provides pointer (bufptr) and size 0x01 to store HCI
* packet type.
* Second : Invoked from controller to receive the HCI packet header. bufptr and size is provided by the controller to
* store the data.
* Third  : Invoked from controller to receive the HCI packet payload. bufptr and size is provided by the controller to
* store the data.
*
*/
void EIF_Read(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy)
{
#ifdef NXP_BLE_STACK
    static eif_read_state_t state; /* To maintain state of the EIF read invoked from controller */
#endif
    ASSERT_ERR(bufptr != NULL);
    ASSERT_ERR(size != 0);
    ASSERT_ERR(callback != NULL);

    s_eifEnv.rx_callback = callback;
    s_eifEnv.rx_dummy = dummy;
#ifdef NXP_BLE_STACK
    if (hciCmdRcvdFlag == 0)
    {
        /* During initialization set state to Command Packet Type received.
           During initilzation Contoller gives bufptr to receive hci packet type */
        state = CMDTYPE_RCVD;
        hciPktType = bufptr; /* Pointer to copy the packet type from HCI command. See Hcit_SendPacket()*/
    }

    /* Enters only if Hcit_SendPacket() has been called by host.i.e. HCI command has been received from Host */
    if (hciCmdRcvdFlag == 1)
    {
        switch (state)
        {
            case CMDTYPE_RCVD: /* When the command type has been received and now its time to obtain the packet header
                                  */
            {
                state = OPCODE_RCVD;
                hciPktHdrLen = size; /* Copy size of packet header. Used for determining size of packet payload */
                memcpy(bufptr, hciRxBuf, size); /* Copy the packet header from hciRxBuf to bufptr from controller*/

                /* If packet type is HCI Command Packet and if param len ( 3rd byte of packet header ) is zero reset
                 * flag and state*/
                if ((*hciPktType == gHciCommandPacket_c) && (*(hciRxBuf + 2) == 0))
                {
                    hciCmdRcvdFlag = 0;
                    state = CMDTYPE_RCVD;
                }

                /*BLE stack external interface receive callback.*/
                EIF_RxCallback();
            }
            break;
            case OPCODE_RCVD: /* Packet header has been received and now its time to obtain the packet payload */
            {
                hciCmdRcvdFlag = 0;   /* Reset flag */
                state = CMDTYPE_RCVD; /* Reset state */
                                      /* Copy packet payload from hciRxBuf into bufptr from controller*/
                memcpy(bufptr, (hciRxBuf + hciPktHdrLen), hciPktSize - hciPktHdrLen);

                /*BLE stack external interface receive callback.*/
                EIF_RxCallback();
            }
            break;
        }
    }
#else
#if (defined(CFG_BLE_EIF) && (CFG_BLE_EIF == kEIF_Uart))
    ADAPTER_UartReceiveInt(bufptr, size, EIF_RxCallback);
#elif(defined(CFG_BLE_EIF) && (CFG_BLE_EIF == kEIF_Usb_Vcom))
    ADAPTER_UsbReceiveInt(bufptr, size, EIF_RxCallback);
#endif
#endif /* #ifdef NXP_BLE_STACK */
}

/*!
* \brief  Invoked by the controller to send HCI packet to the Host.
*/
void EIF_Write(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy)
{
    ASSERT_ERR(bufptr != NULL);
    ASSERT_ERR(size != 0);
    ASSERT_ERR(callback != NULL);

    s_eifEnv.tx_callback = callback;
    s_eifEnv.tx_dummy = dummy;
#ifdef NXP_BLE_STACK
    /* Send packet from Controller to the Host */
    Hcit_RecvPacket(bufptr, size);
    /* BLE stack external interface transmit callback*/
    EIF_TxCallback();
#else
#if (defined(CFG_BLE_EIF) && (CFG_BLE_EIF == kEIF_Uart))
    ADAPTER_UartTransmitInt(bufptr, size, EIF_TxCallback);
#elif(defined(CFG_BLE_EIF) && (CFG_BLE_EIF == kEIF_Usb_Vcom))
    ADAPTER_UsbTransmitInt(bufptr, size, EIF_TxCallback);
#endif
#endif /*#ifdef NXP_BLE_STACK*/
}

void EIF_FlowOn(void)
{
}

bool EIF_FlowOff(void)
{
    return true;
}
