/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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

#include "fsl_dpu.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dpu"
#endif

typedef struct
{
    __IO uint32_t IPIDENTIFIER;
    uint8_t RESERVED_0[60];
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t USERINTERRUPTMASK[2];
    __IO uint32_t INTERRUPTENABLE[2];
    __IO uint32_t INTERRUPTPRESET[2];
    __IO uint32_t INTERRUPTCLEAR[2];
    __IO uint32_t INTERRUPTSTATUS[2];
    uint8_t RESERVED_1[16];
    __IO uint32_t USERINTERRUPTENABLE[2];
    __IO uint32_t USERINTERRUPTPRESET[2];
    __IO uint32_t USERINTERRUPTCLEAR[2];
    __IO uint32_t USERINTERRUPTSTATUS[2];
    uint8_t RESERVED_2[96];
    __IO uint32_t GENERALPURPOSE;
} DPU_COMCTRL_Type;

typedef struct
{
    __IO uint32_t BASEADDRESS;
    __IO uint32_t SOURCEBUFFERATTRIBUTES;
    __IO uint32_t SOURCEBUFFERDIMENSION;
    __IO uint32_t COLORCOMPONENTBITS;
    __IO uint32_t COLORCOMPONENTSHIFT;
    __IO uint32_t LAYEROFFSET;
    __IO uint32_t CLIPWINDOWOFFSET;
    __IO uint32_t CLIPWINDOWDIMENSIONS;
    __IO uint32_t CONSTANTCOLOR;
    __IO uint32_t LAYERPROPERTY;
} DPU_SUBLAYER_CONTROL_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __I uint32_t LOCKSTATUS;
    __IO uint32_t STATIC;
    __IO uint32_t DYNAMIC;
    __IO uint32_t REQUEST;
    __O uint32_t TRIGGER;
    __I uint32_t STATUS;
} DPU_PIPELINE_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t BURSTBUFFERMANAGEMENT;
    DPU_SUBLAYER_CONTROL_Type LAYER[1];
    __IO uint32_t FRAMEDIMENSIONS;
    __IO uint32_t FRAMERESAMPLING;
    __IO uint32_t CONTROL;
    __IO uint32_t CONTROLTRIGGER;
    __IO uint32_t START;
    __IO uint32_t FETCHTYPE;
    __IO uint32_t BURSTBUFFERPROPERTIES;
    __IO uint32_t HIDDENSTATUS;
} DPU_FETCHECO_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t FRAMEDIMENSIONS;
    __IO uint32_t CONSTANTCOLOR;
    __IO uint32_t CONTROLTRIGGER;
    __IO uint32_t START;
    __IO uint32_t STATUS;
} DPU_CONSTFRAME_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t CONTROL;
    __IO uint32_t NEUTRALBORDER;
    __IO uint32_t CONSTANTCOLOR;
    __IO uint32_t COLORREDBLENDFUNCTION;
    __IO uint32_t COLORGREENBLENDFUNCTION;
    __IO uint32_t COLORBLUEBLENDFUNCTION;
    __IO uint32_t ALPHABLENDFUNCTION;
    __IO uint32_t BLENDMODE1;
    __IO uint32_t BLENDMODE2;
    __IO uint32_t DIRECTSETUP;
    __IO uint32_t PRIMCONTROLWORD;
    __IO uint32_t SECCONTROLWORD;
} DPU_BLITBLEND_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t CLOCKCTRL;
    __IO uint32_t POLARITYCTRL;
    __IO uint32_t SRCSELECT;
    uint8_t RESERVED[12];
} DPU_DISENGCONF_BLOCK_Type;

typedef struct
{
    DPU_DISENGCONF_BLOCK_Type DISPLAY[2];
} DPU_DISENGCONF_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t CONTROL;
    __IO uint32_t SOFTWAREKICK;
    __IO uint32_t STATUS;
    __IO uint32_t CONTROLWORD;
    __IO uint32_t CURPIXELCNT;
    __IO uint32_t LASTPIXELCNT;
    __IO uint32_t PERFCOUNTER;
} DPU_EXTDST_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t BURSTBUFFERMANAGEMENT;
    __IO uint32_t RINGBUFSTARTADDR0;
    __IO uint32_t RINGBUFWRAPADDR0;
    __IO uint32_t FRAMEPROPERTIES0;
    DPU_SUBLAYER_CONTROL_Type LAYER[1];
    __IO uint32_t FRAMEDIMENSIONS;
    __IO uint32_t FRAMERESAMPLING;
    __IO uint32_t DECODECONTROL;
    __IO uint32_t SOURCEBUFFERLENGTH;
    __IO uint32_t CONTROL;
    __IO uint32_t CONTROLTRIGGER;
    __IO uint32_t START;
    __IO uint32_t FETCHTYPE;
    __IO uint32_t DECODERSTATUS;
    __IO uint32_t READADDRESS0;
    __IO uint32_t BURSTBUFFERPROPERTIES;
    __IO uint32_t STATUS;
    __IO uint32_t HIDDENSTATUS;
    uint8_t RESERVED_0[904];
    __IO uint32_t COLORPALETTE;
} DPU_FETCHDECODER_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t BURSTBUFFERMANAGEMENT;
    DPU_SUBLAYER_CONTROL_Type LAYER[8];
    __IO uint32_t FRAMEDIMENSIONS;
    __IO uint32_t FRAMERESAMPLING;
    __IO uint32_t CONTROL;
    __IO uint32_t TRIGGERENABLE;
    __IO uint32_t CONTROLTRIGGER;
    __IO uint32_t START;
    __IO uint32_t FETCHTYPE;
    __IO uint32_t BURSTBUFFERPROPERTIES;
    __IO uint32_t STATUS;
    __IO uint32_t HIDDENSTATUS;
    uint8_t RESERVED_0[648];
    __IO uint32_t COLORPALETTE;
} DPU_FETCHLAYER_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t BURSTBUFFERMANAGEMENT;
    DPU_SUBLAYER_CONTROL_Type LAYER[8];
    __IO uint32_t FRAMEDIMENSIONS;
    __IO uint32_t FRAMERESAMPLING;
    __IO uint32_t WARPCONTROL;
    __IO uint32_t ARBSTARTX;
    __IO uint32_t ARBSTARTY;
    __IO uint32_t ARBDELTA;
    __IO uint32_t FIRPOSITIONS;
    __IO uint32_t FIRCOEFFICIENTS;
    __IO uint32_t CONTROL;
    __IO uint32_t TRIGGERENABLE;
    __IO uint32_t CONTROLTRIGGER;
    __IO uint32_t START;
    __IO uint32_t FETCHTYPE;
    __IO uint32_t BURSTBUFFERPROPERTIES;
    __IO uint32_t STATUS;
    __IO uint32_t HIDDENSTATUS;
} DPU_FETCHWARP_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t FGSTCTRL;
    __IO uint32_t HTCFG1;
    __IO uint32_t HTCFG2;
    __IO uint32_t VTCFG1;
    __IO uint32_t VTCFG2;
    __IO uint32_t INTCONFIG[4];
    __IO uint32_t PKICKCONFIG;
    __IO uint32_t SKICKCONFIG;
    __IO uint32_t SECSTATCONFIG;
    __IO uint32_t FGSRCR1;
    __IO uint32_t FGSRCR2;
    __IO uint32_t FGSRCR3;
    __IO uint32_t FGSRCR4;
    __IO uint32_t FGSRCR5;
    __IO uint32_t FGSRCR6;
    __IO uint32_t FGKSDR;
    __IO uint32_t PACFG;
    __IO uint32_t SACFG;
    __IO uint32_t FGINCTRL;
    __IO uint32_t FGINCTRLPANIC;
    __IO uint32_t FGCCR;
    __IO uint32_t FGENABLE;
    __IO uint32_t FGSLR;
    __IO uint32_t FGENSTS;
    __IO uint32_t FGTIMESTAMP;
    __IO uint32_t FGCHSTAT;
    __IO uint32_t FGCHSTATCLR;
    __IO uint32_t FGSKEWMON;
    __IO uint32_t FGSFIFOMIN;
    __IO uint32_t FGSFIFOMAX;
    __IO uint32_t FGSFIFOFILLCLR;
    __IO uint32_t FGSREPD;
    __IO uint32_t FGSRFTD;
} DPU_FRAMEGEN_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t SETUP1;
    __IO uint32_t SETUP2;
    __IO uint32_t CONTROL;
} DPU_HSCALER_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t CONTROL;
    __IO uint32_t BLENDCONTROL;
    __IO uint32_t POSITION;
    __IO uint32_t PRIMCONTROLWORD;
    __IO uint32_t SECCONTROLWORD;
} DPU_LAYERBLEND_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t SETUP1;
    __IO uint32_t SETUP2;
    __IO uint32_t SETUP3;
    __IO uint32_t SETUP4;
    __IO uint32_t SETUP5;
    __IO uint32_t CONTROL;
} DPU_VSCALER_Type;

typedef struct
{
    __IO uint32_t SSQCNTS;
    uint8_t RESERVED_0[1020];
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t SSQCYCLE;
    __IO uint32_t SWRESET;
    __IO uint32_t TCON_CTRL;
    __IO uint32_t RSDSINVCTRL;
    __IO uint32_t MAPBIT3_0;
    __IO uint32_t MAPBIT7_4;
    __IO uint32_t MAPBIT11_8;
    __IO uint32_t MAPBIT15_12;
    __IO uint32_t MAPBIT19_16;
    __IO uint32_t MAPBIT23_20;
    __IO uint32_t MAPBIT27_24;
    __IO uint32_t MAPBIT31_28;
    __IO uint32_t MAPBIT34_32;
    __IO uint32_t MAPBIT3_0_DUAL;
    __IO uint32_t MAPBIT7_4_DUAL;
    __IO uint32_t MAPBIT11_8_DUAL;
    __IO uint32_t MAPBIT15_12_DUAL;
    __IO uint32_t MAPBIT19_16_DUAL;
    __IO uint32_t MAPBIT23_20_DUAL;
    __IO uint32_t MAPBIT27_24_DUAL;
    __IO uint32_t MAPBIT31_28_DUAL;
    __IO uint32_t MAPBIT34_32_DUAL;

    struct
    {
        __IO uint32_t SPGPOSON;
        __IO uint32_t SPGMASKON;
        __IO uint32_t SPGPOSOFF;
        __IO uint32_t SPGMASKOFF;
    } SPG[12];

    struct
    {
        __IO uint32_t SMXSIGS;
        __IO uint32_t SMXFCTTABLE;
    } SMX[12];

    __IO uint32_t RESET_OVER_UNFERFLOW;
    __IO uint32_t DUAL_DEBUG;
} DPU_TCON_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t CONTROL;
    __IO uint32_t RASTEROPERATIONINDICES;
    __IO uint32_t PRIMCONTROLWORD;
    __IO uint32_t SECCONTROLWORD;
    __IO uint32_t TERTCONTROLWORD;
} DPU_ROP_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t BURSTBUFFERMANAGEMENT;
    __IO uint32_t RINGBUFSTARTADDR;
    __IO uint32_t RINGBUFWRAPADDR;
    __IO uint32_t BASEADDRESS;
    __IO uint32_t DESTINATIONBUFFERATTRIBUTES;
    __IO uint32_t DESTINATIONBUFFERDIMENSION;
    __IO uint32_t FRAMEOFFSET;
    __IO uint32_t COLORCOMPONENTBITS;
    __IO uint32_t COLORCOMPONENTSHIFT;
    __IO uint32_t CONTROL;
    __IO uint32_t ENCODECONTROL;
    __IO uint32_t DESTINATIONBUFFERLENGTH;
    __IO uint32_t START;
    __IO uint32_t ENCODERSTATUS;
    __IO uint32_t WRITEADDRESS;
    __IO uint32_t FRAMEPROPERTIES;
    __IO uint32_t BURSTBUFFERPROPERTIES;
    __IO uint32_t LASTCONTROLWORD;
    __IO uint32_t PERFCOUNTER;
    __IO uint32_t STATUS;
} DPU_STORE_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t PANICCOLOR;
    struct
    {
        __IO uint32_t EVALCONTROL;
        __IO uint32_t EVALUPPERLEFT;
        __IO uint32_t EVALLOWERRIGHT;
        __IO uint32_t SIGCRCREDREF;
        __IO uint32_t SIGCRCGREENREF;
        __IO uint32_t SIGCRCBLUEREF;
        __IO uint32_t SIGCRCRED;
        __IO uint32_t SIGCRCGREEN;
        __IO uint32_t SIGCRCBLUE;
    } WINDOW[8];

    __IO uint32_t SHADOWLOAD;
    __IO uint32_t CONTINUOUSMODE;
    __IO uint32_t SOFTWAREKICK;
    __IO uint32_t STATUS;
} DPU_SIG_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t CONTROL;
    __IO uint32_t RED[2];
    __IO uint32_t GREEN[2];
    __IO uint32_t BLUE[2];
    __IO uint32_t ALPHA[2];
    __IO uint32_t OFFSETVECTOR[2];
    __IO uint32_t LASTCONTROLWORD;
} DPU_MATRIX_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t CONTROL;
    __IO uint32_t DITHERCONTROL;
    __IO uint32_t RELEASE;
} DPU_DITHER_Type;

typedef struct
{
    __IO uint32_t LOCKUNLOCK;
    __IO uint32_t LOCKSTATUS;
    __IO uint32_t STATICCONTROL;
    __IO uint32_t LUTSTART;
    __IO uint32_t LUTDELTAS;
    __IO uint32_t CONTROL;
    __IO uint32_t STATUS;
    __IO uint32_t LASTCONTROLWORD;
} DPU_GAMMACOR_Type;

typedef struct
{
    DPU_FRAMEGEN_Type FRAMEGEN;
    uint8_t RESERVED_0[872];
    DPU_MATRIX_Type MATRIX;
    uint8_t RESERVED_1[964];
    DPU_GAMMACOR_Type GAMMACOR;
    uint8_t RESERVED_2[992];
    DPU_DITHER_Type DITHER;
    uint8_t RESERVED_3[1004];
    DPU_TCON_Type TCON;
    uint8_t RESERVED_4[632];
    DPU_SIG_Type SIG;
} DPU_DISPLAY_Type;

#define DPU_INT_GROUP_NUM 2U        /* Interrupt group number. */
#define DPU_DISPLAY_COUNT 2U        /* Display number. */
#define DPU_SIG_EAVL_WINDOW_COUNT 8 /* Signature unit evaluation window count. */

#define DPU_MAKE_DIMENSION(heigth, width) (((uint32_t)(heigth) << 16U) | ((uint32_t)(width) << 0U))
#define DPU_MAKE_POSITION(offsetY, offsetX) (((uint32_t)(offsetY) << 16U) | ((uint32_t)(offsetX) << 0U))
#define DPU_REG(base, offset) *((volatile uint32_t *)((uint32_t)(base) + (offset)))

/* Make the register <fetch_unit>_StaticControl */
#define DPU_MAKE_SOURCEBUFFERATTRIBUTES(bitsPerPixel, stride) \
    ((((uint32_t)(bitsPerPixel) << 16U) & 0x003F0000U) | (((uint32_t)((stride)-1U) << 0U) & 0x0000FFFFU))

#define DPU_MAKE_DESTINATIONBUFFERATTRIBUTES(bitsPerPixel, stride) \
    ((((uint32_t)(bitsPerPixel) << 24U) & 0x7F000000U) | (((uint32_t)((stride)-1U) << 0U) & 0x0001FFFFU))

#define DPU_SCALER_CONTROL_OUTPUTSIZE(output_size) ((((uint32_t)(output_size)) << 16U) & 0x3FFF0000U)
#define DPU_SCALER_CONTROL_SCALEMODE_MASK (0x10U)
#define DPU_SCALER_CONTROL_FILTERMODE_MASK (0x100U)
#define DPU_SCALER_CONTROL_MODE_MASK (0x01U)

/* Offset of register <unit>_StaticControl in each unit control block. */
#define DPU_STATIC_CONTROL_OFFSET 0x8U

/* Mask of the register feild ShdEn */
#define DPU_UNIT_SHDEN_MASK 0x01U

#define DPU_LAYERPROPERTY_SOURCEBUFFERENABLE_MASK 0x80000000U
#define DPU_LAYERPROPERTY_CLIPWINDOWENABLE_MASK 0x40000000U
#define DPU_LAYERPROPERTY_PALETTEENABLE_MASK 0x1U
#define DPU_LAYERPROPERTY_GAMMAREMOVE_MASK 0x100000U

#define DPU_EXTDST_STATICCONTROL_KICKMODE_MASK 0x100U
#define DPU_EXTDST_CONTROL_GAMMAAPPLYENABLE_MASK 0x01U

/* Register TiggerEnable offset in FetchWarp unit. */
#define DPU_FETCHWARP_TRIGGERENABLE_OFFSET 0x174U
/* Register TiggerEnable offset in FetchLayer unit. */
#define DPU_FETCHLAYER_TRIGGERENABLE_OFFSET 0x15CU
/* Register Request offset in pipeline unit. */
#define DPU_PIPELINE_REQUEST_OFFSET 0x10U
/* Register Trigger offset in pipeline unit. */
#define DPU_PIPELINE_TRIGGER_OFFSET 0x14U
/* Mask of clken in <unit>_dynamic register. */
#define DPU_DYNAMIC_CLKEN_MASK 0x03000000U
/* Offset of <unit>_BurstBufferManagement. */
#define DPU_BURSTBUFFERMANAGEMENT_OFFSET 0x0CU

#define DPU_PALETTE_OFFSET 0x400U

#if DPU_USE_GENERATE_HEADER
#define DPU_STORE9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.STORE9_DYNAMIC)
#define DPU_FETCHWARP9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.FETCHWARP9_DYNAMIC)
#define DPU_FETCHDECODE9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.FETCHDECODE9_DYNAMIC)
#define DPU_ROP9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.ROP9_DYNAMIC)
#define DPU_BLITBLEND9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.BLITBLEND9_DYNAMIC)
#define DPU_H_SCALER9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.HSCALER9_DYNAMIC)
#define DPU_V_SCALER9_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.VSCALER9_DYNAMIC)
#define DPU_EXTDST0_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.EXTDST0_DYNAMIC)
#define DPU_EXTDST4_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.EXTDST4_DYNAMIC)
#define DPU_EXTDST1_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.EXTDST1_DYNAMIC)
#define DPU_EXTDST5_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.EXTDST5_DYNAMIC)
#define DPU_FETCHWARP2_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.FETCHWARP2_DYNAMIC)
#define DPU_FETCHDECODE0_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.FETCHDECODE0_DYNAMIC)
#define DPU_FETCHDECODE1_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.FETCHDECODE1_DYNAMIC)
#define DPU_H_SCALER4_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.HSCALER4_DYNAMIC)
#define DPU_V_SCALER4_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.VSCALER4_DYNAMIC)
#define DPU_H_SCALER5_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.HSCALER5_DYNAMIC)
#define DPU_V_SCALER5_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.VSCALER5_DYNAMIC)
#define DPU_LAYERBLEND0_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.LAYERBLEND0_DYNAMIC)
#define DPU_LAYERBLEND1_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.LAYERBLEND1_DYNAMIC)
#define DPU_LAYERBLEND2_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.LAYERBLEND2_DYNAMIC)
#define DPU_LAYERBLEND3_DYNAMIC_OFFSET DPU_UNIT_OFFSET(PIXENGCFG.LAYERBLEND3_DYNAMIC)

#else /* DPU_USE_GENERATE_HEADER */

#define DPU_STORE9_DYNAMIC_OFFSET 0x094C
#define DPU_FETCHWARP9_DYNAMIC_OFFSET 0x0848
#define DPU_FETCHDECODE9_DYNAMIC_OFFSET 0x0828
#define DPU_ROP9_DYNAMIC_OFFSET 0x0868
#define DPU_BLITBLEND9_DYNAMIC_OFFSET 0x0928
#define DPU_H_SCALER9_DYNAMIC_OFFSET 0x08C8
#define DPU_V_SCALER9_DYNAMIC_OFFSET 0x08E8
#define DPU_EXTDST0_DYNAMIC_OFFSET 0x098C
#define DPU_EXTDST4_DYNAMIC_OFFSET 0x09CC
#define DPU_EXTDST1_DYNAMIC_OFFSET 0x0a0C
#define DPU_EXTDST5_DYNAMIC_OFFSET 0x0a4C
#define DPU_FETCHWARP2_DYNAMIC_OFFSET 0x0a68
#define DPU_FETCHDECODE0_DYNAMIC_OFFSET 0x0a88
#define DPU_FETCHDECODE1_DYNAMIC_OFFSET 0x0aA8
#define DPU_H_SCALER4_DYNAMIC_OFFSET 0x0b08
#define DPU_V_SCALER4_DYNAMIC_OFFSET 0x0b28
#define DPU_H_SCALER5_DYNAMIC_OFFSET 0x0b68
#define DPU_V_SCALER5_DYNAMIC_OFFSET 0x0b88
#define DPU_LAYERBLEND0_DYNAMIC_OFFSET 0x0bA8
#define DPU_LAYERBLEND1_DYNAMIC_OFFSET 0x0bC8
#define DPU_LAYERBLEND2_DYNAMIC_OFFSET 0x0bE8
#define DPU_LAYERBLEND3_DYNAMIC_OFFSET 0x0c08

#endif /* DPU_USE_GENERATE_HEADER */

#define DPU_STRUCT_OFFSET(structure, member) (uint32_t)(&((structure *)0)->member)

#define DPU_FETCHECO_FRAMEDIMENSIONS_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHECO_Type, FRAMEDIMENSIONS)
#define DPU_FETCHDECODE_FRAMEDIMENSIONS_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHDECODER_Type, FRAMEDIMENSIONS)
#define DPU_FETCHLAYER_FRAMEDIMENSIONS_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHLAYER_Type, FRAMEDIMENSIONS)
#define DPU_FETCHWARP_FRAMEDIMENSIONS_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHWARP_Type, FRAMEDIMENSIONS)
#define DPU_FETCHECO_FRAMERESAMPLING_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHECO_Type, FRAMERESAMPLING)
#define DPU_FETCHDECODE_FRAMERESAMPLING_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHDECODER_Type, FRAMERESAMPLING)
#define DPU_FETCHLAYER_FRAMERESAMPLING_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHLAYER_Type, FRAMERESAMPLING)
#define DPU_FETCHWARP_FRAMERESAMPLING_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHWARP_Type, FRAMERESAMPLING)
#define DPU_FETCHECO_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHECO_Type, CONTROL)
#define DPU_FETCHDECODE_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHDECODER_Type, CONTROL)
#define DPU_FETCHLAYER_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHLAYER_Type, CONTROL)
#define DPU_FETCHWARP_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHWARP_Type, CONTROL)
#define DPU_FETCHECO_SUBLAYER_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHECO_Type, LAYER)
#define DPU_FETCHDECODE_SUBLAYER_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHDECODER_Type, LAYER)
#define DPU_FETCHLAYER_SUBLAYER_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHLAYER_Type, LAYER)
#define DPU_FETCHWARP_SUBLAYER_CONTROL_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHWARP_Type, LAYER)
#define DPU_FETCHUNIT_BURSTBUFFERMANAGEMENT_OFFSET DPU_STRUCT_OFFSET(DPU_FETCHWARP_Type, BURSTBUFFERMANAGEMENT)

#define DPU_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS(x) (((uint32_t)(x) << 0U) & 0xffU)
#define DPU_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH(x) (((uint32_t)(x) << 8U) & 0x1f00U)

#define DPU_MAKE_FETCH_RESAMPLING(swap, deltaX, deltaY, startX, startY) \
    (((swap) << 24U) | ((deltaY) << 18U) | ((deltaX) << 12U) | ((startY) << 6U) | (startX))

#define DPU_FETCH_CONTROL_RASTERMODE_MASK 0x7U
#define DPU_FETCH_CONTROL_INPUTSELECT_MASK 0x18U
#define DPU_FETCH_CONTROL_YUV422UPSAMPLINGMODE_MASK 0x20U
#define DPU_FETCH_CONTROL_RAWPIXEL_MASK 0x80U
#define DPU_FETCH_CONTROL_PALETTEIDXWIDTH_MASK 0x700U
#define DPU_FETCH_CONTROL_CLIPCOLOR_MASK 0x10000U
#define DPU_FETCH_CONTROL_CLIPLAYER_MASK 0xE0000U
#define DPU_FETCH_CONTROL_FILTERMODE_MASK 0x700000U

#define DPU_FETCH_CONTROL_RASTERMODE(x) ((x))
#define DPU_FETCH_CONTROL_INPUTSELECT(x) ((uint32_t)(x) << 3U)
#define DPU_FETCH_CONTROL_YUV422UPSAMPLINGMODE(x) ((uint32_t)(x) << 5U)
#define DPU_FETCH_CONTROL_RAWPIXEL(x) ((uint32_t)(x) << 7U)
#define DPU_FETCH_CONTROL_PALETTEIDXWIDTH(x) ((uint32_t)(x) << 8U)
#define DPU_FETCH_CONTROL_CLIPCOLOR(x) ((uint32_t)(x) << 16U)
#define DPU_FETCH_CONTROL_CLIPLAYER(x) ((uint32_t)(x) << 17U)
#define DPU_FETCH_CONTROL_FILTERMODE(x) ((uint32_t)(x) << 20U)

#if DPU_USE_GENERATE_HEADER
#define DPU_PIPELINE_STATIC_SYNCMODE_MASK DPU_PIXENGCFG_EXTDST0_STATIC_extdst0_Sync_Mode_MASK
#define DPU_PIPELINE_STATIC_POWERDOWN_MASK DPU_PIXENGCFG_EXTDST0_STATIC_extdst0_powerdown_MASK
#define DPU_PIPELINE_STATIC_DIV(x) DPU_PIXENGCFG_EXTDST0_STATIC_extdst0_div(x)
#define DPU_PIPELINE_TRIGGER_SYNC_TRIGGER_MASK DPU_PIXENGCFG_EXTDST0_TRIGGER_extdst0_Sync_Trigger_MASK
#define DPU_PIPELINE_TRIGGER_SEQ_COMP_MASK DPU_PIXENGCFG_EXTDST0_TRIGGER_extdst0_trigger_sequence_complete_MASK;

#define DPU_FETCH_STATICCONTROL_SHDLDREQSTICKY_MASK DPU_FETCHLAYER_STATICCONTROL_ShdLdReqSticky_MASK

#else /* DPU_USE_GENERATE_HEADER */

#define DPU_PIPELINE_STATIC_SYNCMODE_MASK (1U << 8U)
#define DPU_PIPELINE_STATIC_POWERDOWN_MASK (1U << 4U)
#define DPU_PIPELINE_STATIC_DIV(x) (((uint32_t)(x) << 16U) & 0x00ff0000U)
#define DPU_PIPELINE_TRIGGER_SYNC_TRIGGER_MASK (1U << 0U)
#define DPU_PIPELINE_TRIGGER_SEQ_COMP_MASK (1U << 4U)
#define DPU_STORE_START_Start_MASK (1U << 0U)

#define DPU_FETCH_STATICCONTROL_SHDLDREQSTICKY_MASK 0xFF000000U

#define DPU_LAYERBLEND_STATICCONTROL_ShdEn_MASK (1U << 0U)
#define DPU_LAYERBLEND_STATICCONTROL_ShdTokSel(x) ((uint32_t)(x) << 3U)
#define DPU_LAYERBLEND_STATICCONTROL_ShdLdSel(x) ((uint32_t)(x) << 1U)

#define DPU_LAYERBLEND_BLENDCONTROL_BlendAlpha(x) (((uint32_t)(x)&0xFFU) << 16U)
#define DPU_LAYERBLEND_BLENDCONTROL_PRIM_C_BLD_FUNC(x) (((uint32_t)(x)&0x7U) << 0U)
#define DPU_LAYERBLEND_BLENDCONTROL_SEC_C_BLD_FUNC(x) (((uint32_t)(x)&0x7U) << 4U)
#define DPU_LAYERBLEND_BLENDCONTROL_PRIM_A_BLD_FUNC(x) (((uint32_t)(x)&0x7U) << 8U)
#define DPU_LAYERBLEND_BLENDCONTROL_SEC_A_BLD_FUNC(x) (((uint32_t)(x)&0x7U) << 12U)

#define DPU_LAYERBLEND_CONTROL_MODE_MASK (1U << 0U)
#define DPU_LAYERBLEND_CONTROL_AlphaMaskEnable_MASK (1U << 2U)
#define DPU_LAYERBLEND_CONTROL_AlphaMaskMode_MASK (7U << 4U)
#define DPU_LAYERBLEND_CONTROL_AlphaMaskEnable(x) (((uint32_t)(x)&0x1U) << 2U)
#define DPU_LAYERBLEND_CONTROL_AlphaMaskMode(x) (((uint32_t)(x)&0x7U) << 4U)

#define DPU_FETCHWARP_WARPCONTROL_WarpBitsPerPixel(x) (((uint32_t)(x) << 0U) & 0x3FU)
#define DPU_FETCHWARP_WARPCONTROL_WarpCoordinateMode(x) (((uint32_t)(x)&0x03U) << 8U)
#define DPU_FETCHWARP_WARPCONTROL_WarpSymmetricOffset(x) (((uint32_t)(x)&0x01U) << 12U)

#define DPU_FETCHWARP_ARBDELTA_ArbDeltaXX(x) ((uint32_t)(x) << 0U)
#define DPU_FETCHWARP_ARBDELTA_ArbDeltaXY(x) ((uint32_t)(x) << 8U)
#define DPU_FETCHWARP_ARBDELTA_ArbDeltaYX(x) ((uint32_t)(x) << 16U)
#define DPU_FETCHWARP_ARBDELTA_ArbDeltaYY(x) ((uint32_t)(x) << 24U)

#define DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderLeft(x) (((uint32_t)(x)&0x07U) << 8U)
#define DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderRight(x) (((uint32_t)(x)&0x07U) << 12U)
#define DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderMode(x) (((uint32_t)(x)&0x01U) << 0U)

#define DPU_BLITBLEND_CONTROL_Mode_MASK (1U << 0U)

#define DPU_TCON_TCON_CTRL_Bypass_MASK (1U << 3U)

#define DPU_TCON_SPGPOSON_SPGPSON_X(x) (((uint32_t)(x)&0x7FFFU) << 16U)
#define DPU_TCON_SPGPOSON_SPGPSON_Y(x) (((uint32_t)(x)&0xFFFFU) << 0U)
#define DPU_TCON_SPGPOSON_SPGPSON_X_MASK (0x7FFFU << 16U)
#define DPU_TCON_SPGPOSON_SPGPSON_Y_MASK (0xFFFFU << 0U)
#define DPU_TCON_SPGPOSOFF_SPGPSOFF_X(x) (((uint32_t)(x)&0x7FFFU) << 16U)
#define DPU_TCON_SPGPOSOFF_SPGPSOFF_Y(x) (((uint32_t)(x)&0xFFFFU) << 0U)
#define DPU_TCON_SPGPOSOFF_SPGPSOFF_X_MASK (0x7FFFU << 16U)
#define DPU_TCON_SPGPOSOFF_SPGPSOFF_Y_MASK (0xFFFFU << 0U)

#define DPU_TCON_SMXSIGS_SMXSIGS_S0(x) (((uint32_t)(x)&0x7U) << 0U)
#define DPU_TCON_SMXSIGS_SMXSIGS_S1(x) (((uint32_t)(x)&0x7U) << 3U)
#define DPU_TCON_SMXSIGS_SMXSIGS_S2(x) (((uint32_t)(x)&0x7U) << 6U)
#define DPU_TCON_SMXSIGS_SMXSIGS_S3(x) (((uint32_t)(x)&0x7U) << 9U)
#define DPU_TCON_SMXSIGS_SMXSIGS_S4(x) (((uint32_t)(x)&0x7U) << 12U)

#define DPU_FRAMEGEN_HTCFG1_Htotal(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_HTCFG1_Hact(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_HTCFG2_Hsync(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_HTCFG2_Hsbp(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_HTCFG2_HsEn_MASK (1U << 31U)
#define DPU_FRAMEGEN_VTCFG1_Vtotal(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_VTCFG1_Vact(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_VTCFG2_Vsync(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_VTCFG2_Vsbp(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_VTCFG2_VsEn_MASK (1U << 31U)

#define DPU_FRAMEGEN_PKICKCONFIG_PKickRow(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_PKICKCONFIG_PKickCol(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_PKICKCONFIG_PKickEn_MASK (1U << 31U)
#define DPU_FRAMEGEN_SKICKCONFIG_SKickRow(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_SKICKCONFIG_SKickCol(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_SKICKCONFIG_SKickEn_MASK (1U << 31U)

#define DPU_FRAMEGEN_PACFG_Pstartx(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_PACFG_Pstarty(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_SACFG_Sstartx(x) (((uint32_t)(x)&0x3FFFU) << 0U)
#define DPU_FRAMEGEN_SACFG_Sstarty(x) (((uint32_t)(x)&0x3FFFU) << 16U)
#define DPU_FRAMEGEN_FGINCTRL_FgDm(x) ((uint32_t)(x)&0x07U)
#define DPU_FRAMEGEN_FGINCTRL_EnSecAlpha_MASK (1U << 4U)
#define DPU_FRAMEGEN_FGINCTRL_EnPrimAlpha_MASK (1U << 3U)

#define DPU_FRAMEGEN_FGCCR_CcBlue(x) (((uint32_t)(x)&0x3FFU) << 0U)
#define DPU_FRAMEGEN_FGCCR_CcGreen(x) (((uint32_t)(x)&0x3FFU) << 10U)
#define DPU_FRAMEGEN_FGCCR_CcRed(x) (((uint32_t)(x)&0x3FFU) << 20U)
#define DPU_FRAMEGEN_FGCCR_CcAlpha(x) (((uint32_t)(x)&0x1U) << 30U)

#define DPU_FRAMEGEN_FGSLR_ShdTokGen_MASK (1U << 0U)
#define DPU_FRAMEGEN_FGENABLE_FgEn_MASK (1U << 0U)

#define DPU_SIG_STATICCONTROL_ErrThres_SHIFT 16U
#define DPU_SIG_STATICCONTROL_ErrThresReset_SHIFT 24U
#define DPU_SIG_STATICCONTROL_ShdEn_MASK (1U << 0U)
#define DPU_SIG_PANICCOLOR_PanicRed_SHIFT 24U
#define DPU_SIG_PANICCOLOR_PanicGreen_SHIFT 16U
#define DPU_SIG_PANICCOLOR_PanicBlue_SHIFT 8U
#define DPU_SIG_PANICCOLOR_PanicAlpha_MASK (1U << 7U)
#define DPU_SIG_CONTINUOUSMODE_EnCont_MASK (1U << 0U)

#endif /* DPU_USE_GENERATE_HEADER */

typedef union _u32_f32
{
    float f32;
    uint32_t u32;
} u32_f32_t;

/*! @brief LayerBlend unit shadow token generate mode. */
enum _dpu_layer_blend_shadow_token_mode
{
    kDPU_LayerBlendShadowTokenPrim, /*!< Generate shadow load token when token received from primary input. */
    kDPU_LayerBlendShadowTokenSec,  /*!< Generate shadow load token when token received from secondary input. */
    kDPU_LayerBlendShadowTokenBoth  /*!< Generate shadow load token when token received from any input. */
};

/*! @brief LayerBlend unit shadow load mode. */
enum _dpu_layer_blend_shadow_load_mode
{
    kDPU_LayerBlendShadowLoadPrim, /*!< Load shadows when token received from primary input. */
    kDPU_LayerBlendShadowLoadSec,  /*!< Load shadows when token received from secondary input. */
    kDPU_LayerBlendShadowLoadBoth  /*!< Load shadows when token received from any input. */
};

/*! @brief Fetch unit control input select, selects function for the frame input port. */
enum _dpu_fetch_ctrl_input_sel
{
    kDPU_FetchCtrlInputSelInactive = 0x00U,   /*!< Not used. */
    kDPU_FetchCtrlInputSelCompPack = 0x01U,   /*!< Used for component packing, e.g. UV or source alpha buffer. */
    kDPU_FetchCtrlInputSelAlphaMask = 0x02U,  /*!< Used for RGB and alpha pre-multiply stage (mask alpha buffer). */
    kDPU_FetchCtrlInputSelCoordinate = 0x03U, /*!< Used for arbitrary warping. */
};

/*! @brief Fetch unit control input select, selects function for the frame input port. */
enum _dpu_fetch_ctrl_raster_mode
{
    kDPU_FetchCtrlRasterNormal = 0x00U,
    kDPU_FetchCtrlRasterDecode = 0x01U,
    kDPU_FetchCtrlRasterArbitrary = 0x02U,
    kDPU_FetchCtrlRasterPerspective = 0x03U,
    kDPU_FetchCtrlRasterYUV422 = 0x04U,
    kDPU_FetchCtrlRasterAffine = 0x05U,
};

/*! @brief Fetch unit control filter mode. */
enum _dpu_fetch_ctrl_filter_mode
{
    kDPU_FetchCtrlFilterNearest = 0U,  /*!< Chooses pixel closest to sample point */
    kDPU_FetchCtrlFilterBilinear = 1U, /*!< Calculates result from 4 pixels closest to sample point */
    kDPU_FetchCtrlFilterFir2 = 2U,     /*!< FIR mode with 2 programmable pixel positions and coefficients */
    kDPU_FetchCtrlFilterFir4 = 3U,     /*!< FIR mode with 4 programmable pixel positions and coefficients */
    kDPU_FetchCtrlFilterHorLinear =
        4U, /*!< Calculates result from 2 pixels closest to the sample point and on the same line */
};

/*! @brief DPU unit dynamic register offset. */
typedef struct _dpu_unit_dynamic_reg_offset
{
    dpu_unit_t unit;
    uint32_t offset;
} dpu_unit_dynamic_reg_offset_t;

/*!
 * @name Macros for pixel format.
 *
 * In DPU, the pixel format are controlled by three parameters,
 * ColorComponentBits, ColorComponentShift, and PixelPerBits. These macros define
 * the parameter values for different pixel format, they could be used during
 * source buffer and Dstination buffer configuration.
 *
 * The width of color components as stored in memory can be setup individually to
 * any value between
 *   - 0 and 10 bits for RGB or YUV
 *   - 0 and 8 bits for Alpha and Index
 * Any bit position within the pixel word can be configured individually for each
 * component. There are no restrictions regarding sequence or overlaps.
 *
 * Example for RGB565 (16 bpp):
 * @verbatim
   |--------------- total size --------------------|
   |15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
   |---bluebits---|----greenbits----|---redbits----|
                  |---------- blue shift ----------|
                                    |-green shift -|
   @endverbatim
 *
 * The value for components that are setup to null size is taken from constant color.
 *
 * If the @p itu is set to 1, then the ITU656 compatibility mode is used, the max value
 * of 10-bit YUV codes is considered 1020 instead of 1023.
 * @{
 */
#define DPU_MAKE_COLORCOMPONENTBITS(itu, redbits, greenbits, bluebits, alphabits) \
    (((itu) << 31U) | ((redbits) << 24U) | ((greenbits) << 16U) | ((bluebits) << 8U) | (alphabits))
#define DPU_MAKE_COLORCOMPONENTSHIFT(redshift, greenshift, blueshift, alphashift) \
    (((redshift) << 24U) | ((greenshift) << 16U) | ((blueshift) << 8U) | (alphashift))
/*
 * @}
 */

/*!
 * @brief DPU pixel format definition.
 *
 * This table works together with @ref dpu_pixel_format_t.
 */
static const uint32_t s_dpuColorComponentFormats[][2] = {
    /* kDPU_PixelFormatGray8 */
    {
        DPU_MAKE_COLORCOMPONENTBITS(0, 8, 8, 8, 0), DPU_MAKE_COLORCOMPONENTSHIFT(0, 0, 0, 0),
    },
    /* kDPU_PixelFormatRGB565 */
    {
        DPU_MAKE_COLORCOMPONENTBITS(0, 5, 6, 5, 0), DPU_MAKE_COLORCOMPONENTSHIFT(11, 5, 0, 0),
    },
    /* kDPU_PixelFormatARGB8888 */
    {
        DPU_MAKE_COLORCOMPONENTBITS(0, 8, 8, 8, 8), DPU_MAKE_COLORCOMPONENTSHIFT(16, 8, 0, 24),
    },
    /* kDPU_PixelFormatRGB888 */
    {
        DPU_MAKE_COLORCOMPONENTBITS(0, 8, 8, 8, 0), DPU_MAKE_COLORCOMPONENTSHIFT(16, 8, 0, 0),
    },
};

/*! @brief Map to get the dynamic register in PIXENG for DPU unit.  */
static const dpu_unit_dynamic_reg_offset_t s_dpuUnitDynamicRegOffsetTable[] = {
    {
        .unit = kDPU_Store9, .offset = DPU_STORE9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_FetchWarp9, .offset = DPU_FETCHWARP9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_FetchDecode9, .offset = DPU_FETCHDECODE9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_BlitBlend9, .offset = DPU_BLITBLEND9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Rop9, .offset = DPU_ROP9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Hscaler9, .offset = DPU_H_SCALER9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Vscaler9, .offset = DPU_V_SCALER9_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Hscaler4, .offset = DPU_H_SCALER4_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Vscaler4, .offset = DPU_V_SCALER4_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Hscaler5, .offset = DPU_H_SCALER5_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_Vscaler5, .offset = DPU_V_SCALER5_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_ExtDst0, .offset = DPU_EXTDST0_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_ExtDst4, .offset = DPU_EXTDST4_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_ExtDst1, .offset = DPU_EXTDST1_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_ExtDst5, .offset = DPU_EXTDST5_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_FetchWarp2, .offset = DPU_FETCHWARP2_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_FetchDecode0, .offset = DPU_FETCHDECODE0_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_FetchDecode1, .offset = DPU_FETCHDECODE1_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_LayerBlend0, .offset = DPU_LAYERBLEND0_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_LayerBlend1, .offset = DPU_LAYERBLEND1_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_LayerBlend2, .offset = DPU_LAYERBLEND2_DYNAMIC_OFFSET,
    },
    {
        .unit = kDPU_LayerBlend3, .offset = DPU_LAYERBLEND3_DYNAMIC_OFFSET,
    },
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get the DPU instance from peripheral base address.
 *
 * @param base DPU peripheral base address.
 * @return DPU instance.
 */
uint32_t DPU_GetInstance(IRIS_MVPL_Type *base);

/*!
 * @brief Get the offset of the register <unit>_dynamic.
 *
 * Only the unit with attribute @ref kDPU_UnitAttrHasSrc could use this function.
 *
 * @param unit See @ref dpu_unit_t.
 * @return Offset of <unit>_dynamic.
 */
static uint32_t DPU_GetDynamicRegOffset(dpu_unit_t unit);

/*!
 * @brief Check whether the buffer is aligned for the pixel format.
 *
 * For source buffer and destination buffer, the buffer base address and stride
 * must be aligned to the bitsPerPixel. For example. if bitsPerPixel is 32, then
 * the buffer base address and stride should be 32-bit aligned.
 *
 * @param bitsPerPixel Bits per pixel of the buffer.
 * @param baseAddr Base address of the buffer.
 * @param strideBytes Stide of the buffer.
 * @retval true Buffer is aligned.
 * @retval false Buffer is not aligned.
 */
static bool DPU_CheckBufferAlignment(uint8_t bitsPerPixel, uint32_t baseAddr, uint32_t strideBytes);

/*!
 * @brief Get the sublayer control registers of fetch unit.
 *
 * The sublayer control register are defined by DPU_SUBLAYER_CONTROL_Type.
 *
 * @param base DPU peripheral base address.
 * @param unit See @ref dpu_unit_t.
 * @return Sublayer control registers.
 */
static DPU_SUBLAYER_CONTROL_Type *DPU_GetSubLayer(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer);

/*!
 * @brief Convert IEEE 754 float value to the value could be written to registers.
 *
 * This function converts the float value to integer value to set the scaler parameters.
 *
 * @param floatValue The float value to convert.
 * @param intBits Bits number of integer part in result.
 * @param fracBits Bits number of fractional part in result.
 * @return The value to set to register.
 */
static uint32_t DPU_ConvertFloat(float floatValue, uint8_t intBits, uint8_t fracBits);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint32_t s_displayOffsetArray[] = {DPU_FRAME_GEN0_OFFSET, DPU_FRAME_GEN1_OFFSET};

static IRIS_MVPL_Type *const s_dpuBases[] = IRIS_MVPL_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Array of DPU clock name. */
static const clock_ip_name_t s_dpuClock[] = DPU_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t DPU_GetInstance(IRIS_MVPL_Type *base)
{
    uint32_t instance;
    uint32_t dpuArrayCount = ARRAY_SIZE(s_dpuBases);

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < dpuArrayCount; instance++)
    {
        if (s_dpuBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < dpuArrayCount);

    return instance;
}

static bool DPU_CheckBufferAlignment(uint8_t bitsPerPixel, uint32_t baseAddr, uint32_t strideBytes)
{
    uint32_t mask = 0U;
    uint32_t i;

    /*
     * Frame buffer alignment restrictions:
     *   32 bpp: Base address and stride must be a multiple of 4 bytes.
     *   16 bpp: Base address and stride must be a multiple of 2 bytes.
     *   others: any byte alignment allowed
     */
    static const uint32_t s_dpuBufferAlignMask[][2] = {
        {32U, 0x03U}, {16U, 0x01U},
    };

    /* Find the mask to compare. */
    for (i = 0U; i < ARRAY_SIZE(s_dpuBufferAlignMask); i++)
    {
        if (s_dpuBufferAlignMask[i][0] == bitsPerPixel)
        {
            mask = s_dpuBufferAlignMask[i][1];
            break;
        }
    }

    if ((baseAddr & mask) | (strideBytes & mask))
    {
        return false;
    }
    else
    {
        return true;
    }
}

static uint32_t DPU_GetDynamicRegOffset(dpu_unit_t unit)
{
    uint32_t offset = 0U;
    uint32_t i;

    for (i = 0; i < ARRAY_SIZE(s_dpuUnitDynamicRegOffsetTable); i++)
    {
        if (s_dpuUnitDynamicRegOffsetTable[i].unit == unit)
        {
            offset = s_dpuUnitDynamicRegOffsetTable[i].offset;
            break;
        }
    }

    return offset;
}

static DPU_SUBLAYER_CONTROL_Type *DPU_GetSubLayer(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer)
{
    /* The sublayer register offset in the fetch unit control block. */
    uint32_t offset;

    /* If the fetch unit does not support sublayer, then configure sublayer 0. */
    if (!((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrSubLayer)))
    {
        sublayer = 0;
    }

    if (kDPU_FetchDecode == DPU_GET_UNIT_TYPE(unit))
    {
        offset = DPU_FETCHDECODE_SUBLAYER_CONTROL_OFFSET;
    }
    else
    {
        offset = DPU_FETCHLAYER_SUBLAYER_CONTROL_OFFSET;
    }

    return (DPU_SUBLAYER_CONTROL_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit) + offset +
                                         (uint32_t)sublayer * sizeof(DPU_SUBLAYER_CONTROL_Type));
}

static uint32_t DPU_ConvertFloat(float floatValue, uint8_t intBits, uint8_t fracBits)
{
    /* One bit reserved for sign bit. */
    assert(intBits + fracBits < 32);

    u32_f32_t u32_f32;
    uint32_t ret;

    u32_f32.f32 = floatValue;
    uint32_t floatBits = u32_f32.u32;
    int32_t expValue = (int32_t)((floatBits & 0x7F800000U) >> 23U) - 127;

    ret = (floatBits & 0x007FFFFFU) | 0x00800000U;
    expValue += fracBits;

    if (expValue < 0)
    {
        return 0U;
    }
    else if (expValue > 23)
    {
        /* should not exceed 31-bit when left shift. */
        assert((expValue - 23) <= 7);
        ret <<= (expValue - 23);
    }
    else
    {
        ret >>= (23 - expValue);
    }

    /* Set the sign bit. */
    if (floatBits & 0x80000000U)
    {
        ret = ((~ret) + 1U) & ~(((uint32_t)-1) << (intBits + fracBits + 1));
    }

    return ret;
}

void DPU_Init(IRIS_MVPL_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable dpu clock */
    CLOCK_EnableClock(s_dpuClock[DPU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void DPU_Deinit(IRIS_MVPL_Type *base)
{
    uint32_t i;

    /* Disable display output. */
    for (i = 0U; i < DPU_DISPLAY_COUNT; i++)
    {
        DPU_StopDisplay(base, i);
    }

    /* Power down all pipline. */
    DPU_DeinitPipeline(base, kDPU_PipelineExtDst0);
    DPU_DeinitPipeline(base, kDPU_PipelineExtDst1);
    DPU_DeinitPipeline(base, kDPU_PipelineExtDst4);
    DPU_DeinitPipeline(base, kDPU_PipelineExtDst5);
    DPU_DeinitPipeline(base, kDPU_PipelineStore9);

    /* Clear all pending interrupts. */
    for (i = 0; i < DPU_INT_GROUP_NUM; i++)
    {
        DPU_DisableInterrupts(base, i, 0xFFFFFFFFU);
        DPU_DisableUserInterrupts(base, i, 0xFFFFFFFFU);
        DPU_ClearInterruptsPendingFlags(base, i, 0xFFFFFFFFU);
        DPU_ClearUserInterruptsPendingFlags(base, i, 0xFFFFFFFFU);
    }

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable dpu clock */
    CLOCK_DisableClock(s_dpuClock[DPU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void DPU_PreparePathConfig(IRIS_MVPL_Type *base)
{
    const dpu_unit_t dpuPipeLines[] = {
        kDPU_PipelineStore9, kDPU_PipelineExtDst0, kDPU_PipelineExtDst1, kDPU_PipelineExtDst4, kDPU_PipelineExtDst5,
    };

    /* Disable shadow for all pipelines. */
    for (uint32_t i = 0; i < ARRAY_SIZE(dpuPipeLines); i++)
    {
        DPU_EnableShadowLoad(base, dpuPipeLines[i], false);
    }

    /* Set all pixel engine unit source to NULL. */
    const dpu_unit_t dpuUnits[] = {
        kDPU_FetchWarp9,   kDPU_FetchDecode9, kDPU_Hscaler9,    kDPU_Vscaler9,    kDPU_Rop9,     kDPU_BlitBlend9,
        kDPU_Store9,       kDPU_ExtDst0,      kDPU_ExtDst4,     kDPU_ExtDst1,     kDPU_ExtDst5,  kDPU_FetchWarp2,
        kDPU_FetchDecode0, kDPU_FetchDecode1, kDPU_Hscaler4,    kDPU_Vscaler4,    kDPU_Hscaler5, kDPU_Vscaler5,
        kDPU_LayerBlend0,  kDPU_LayerBlend1,  kDPU_LayerBlend2, kDPU_LayerBlend3,
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(dpuUnits); i++)
    {
        DPU_SetUnitSrc(base, dpuUnits[i], 0U);
    }
}

void DPU_EnableInterrupts(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->INTERRUPTENABLE[group] |= mask;
}

void DPU_DisableInterrupts(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->INTERRUPTENABLE[group] &= ~mask;
}

uint32_t DPU_GetInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group)
{
    assert(group < DPU_INT_GROUP_NUM);

    return ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->INTERRUPTSTATUS[group];
}

void DPU_ClearInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->INTERRUPTCLEAR[group] = mask;
}

void DPU_SetInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->INTERRUPTPRESET[group] = mask;
}

void DPU_SetUserInterruptsMask(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTMASK[group] |= mask;
}

void DPU_ClearUserInterruptsMask(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTMASK[group] &= ~mask;
}

void DPU_EnableUserInterrupts(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTENABLE[group] |= mask;
}

void DPU_DisableUserInterrupts(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTENABLE[group] &= ~mask;
}

uint32_t DPU_GetUserInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group)
{
    assert(group < DPU_INT_GROUP_NUM);

    return ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTSTATUS[group];
}

void DPU_ClearUserInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTCLEAR[group] = mask;
}

void DPU_SetUserInterruptsPendingFlags(IRIS_MVPL_Type *base, uint8_t group, uint32_t mask)
{
    assert(group < DPU_INT_GROUP_NUM);

    ((DPU_COMCTRL_Type *)((uint32_t)base + DPU_COMCTRL_OFFSET))->USERINTERRUPTPRESET[group] = mask;
}

status_t DPU_EnableShadowLoad(IRIS_MVPL_Type *base, dpu_unit_t unit, bool enable)
{
    /* Offset of the static control register to enable/disable shadow load. */
    uint32_t staticRegOffset;
    status_t status;

    if ((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrNoShdow))
    {
        status = kStatus_InvalidArgument;
    }
    else
    {
        staticRegOffset = DPU_GET_UNIT_OFFSET(unit) + DPU_STATIC_CONTROL_OFFSET;

        if (enable)
        {
            DPU_REG(base, staticRegOffset) |= DPU_UNIT_SHDEN_MASK;
        }
        else
        {
            DPU_REG(base, staticRegOffset) &= ~DPU_UNIT_SHDEN_MASK;
        }

        status = kStatus_Success;
    }

    return status;
}

void DPU_InitPipeline(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert(kDPU_Pipeline == DPU_GET_UNIT_TYPE(unit));

    DPU_PIPELINE_Type *pipeline = (DPU_PIPELINE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Set pipeline sync mode to single mode, power up, and enable shadow load. */
    pipeline->STATIC = DPU_PIPELINE_STATIC_DIV(0x80) | DPU_UNIT_SHDEN_MASK;
}

void DPU_DeinitPipeline(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert(kDPU_Pipeline == DPU_GET_UNIT_TYPE(unit));

    DPU_PIPELINE_Type *pipeline = (DPU_PIPELINE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Disable shadow load and powerdown the pipeline. */
    pipeline->STATIC = DPU_PIPELINE_STATIC_POWERDOWN_MASK;
}

void DPU_TriggerPipelineShadowLoad(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert(kDPU_Pipeline == DPU_GET_UNIT_TYPE(unit));

    DPU_PIPELINE_Type *pipeline = (DPU_PIPELINE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    pipeline->TRIGGER = DPU_PIPELINE_TRIGGER_SYNC_TRIGGER_MASK;
}

void DPU_TriggerPipelineCompleteInterrupt(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert(kDPU_Pipeline == DPU_GET_UNIT_TYPE(unit));

    DPU_PIPELINE_Type *pipeline = (DPU_PIPELINE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    pipeline->TRIGGER = DPU_PIPELINE_TRIGGER_SEQ_COMP_MASK;
}

void DPU_SetUnitSrc(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    assert((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrHasSrc));

    uint32_t reg;
    uint32_t offset = DPU_GetDynamicRegOffset(unit);

    reg = DPU_REG(base, offset) & DPU_DYNAMIC_CLKEN_MASK;
    DPU_REG(base, offset) = reg | (srcReg & ~DPU_DYNAMIC_CLKEN_MASK);
}

status_t DPU_SetColorPaletteIndexWidth(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t indexWidth)
{
    uint32_t offset;
    uint32_t reg;
    uint8_t type = DPU_GET_UNIT_TYPE(unit);

    if ((kDPU_FetchLayer != type) && (kDPU_FetchDecode != type))
    {
        return kStatus_InvalidArgument;
    }

    offset = ((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit);

    /* Set index width. */
    if (type == kDPU_FetchLayer)
    {
        reg = ((DPU_FETCHLAYER_Type *)offset)->CONTROL;
        reg = (reg & ~DPU_FETCH_CONTROL_PALETTEIDXWIDTH_MASK) | DPU_FETCH_CONTROL_PALETTEIDXWIDTH(indexWidth - 1U);
        ((DPU_FETCHLAYER_Type *)offset)->CONTROL = reg;
    }
    else
    {
        reg = ((DPU_FETCHDECODER_Type *)offset)->CONTROL;
        reg = (reg & ~DPU_FETCH_CONTROL_PALETTEIDXWIDTH_MASK) | DPU_FETCH_CONTROL_PALETTEIDXWIDTH(indexWidth - 1U);
        ((DPU_FETCHDECODER_Type *)offset)->CONTROL = reg;
    }

    return kStatus_Success;
}

status_t DPU_UpdateColorPalette(
    IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t startIndex, const uint32_t *palette, uint32_t count)
{
    assert(palette);
    assert((kDPU_FetchLayer == DPU_GET_UNIT_TYPE(unit)) || (kDPU_FetchDecode == DPU_GET_UNIT_TYPE(unit)));

    uint32_t offset;
    uint32_t i;

    /* Exceed the palette boundary. */
    if (startIndex + count > DPU_PALETTE_ENTRY_NUM)
    {
        return kStatus_InvalidArgument;
    }

    offset = ((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit);

    /* Set palette entery. */
    for (i = 0; i < count; i++)
    {
        ((volatile uint32_t *)(offset + DPU_PALETTE_OFFSET))[i + startIndex] = palette[i];
    }

    return kStatus_Success;
}

void DPU_EnableColorPalette(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer, bool enable)
{
    assert((kDPU_FetchLayer == DPU_GET_UNIT_TYPE(unit)) || (kDPU_FetchDecode == DPU_GET_UNIT_TYPE(unit)));

    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    if (enable)
    {
        control->LAYERPROPERTY |= DPU_LAYERPROPERTY_PALETTEENABLE_MASK;
    }
    else
    {
        control->LAYERPROPERTY &= ~DPU_LAYERPROPERTY_PALETTEENABLE_MASK;
    }
}

void DPU_FetchUnitGetDefaultConfig(dpu_fetch_unit_config_t *config)
{
    assert(config);

    config->srcReg = 0U;
    config->frameHeight = 320U;
    config->frameWidth = 480U;
}

void DPU_InitFetchUnit(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_fetch_unit_config_t *config)
{
    assert(config);
    assert((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrIsFetch));

    uint32_t reg;
    uint32_t offset;
    uint32_t type;
    uint32_t dimensionOffset;
    uint32_t controlOffset;
    uint32_t resamplingOffset;

    offset = DPU_GET_UNIT_OFFSET(unit);
    type = DPU_GET_UNIT_TYPE(unit);

    switch (type)
    {
        case kDPU_FetchDecode:
            dimensionOffset = DPU_FETCHDECODE_FRAMEDIMENSIONS_OFFSET;
            controlOffset = DPU_FETCHDECODE_CONTROL_OFFSET;
            resamplingOffset = DPU_FETCHDECODE_FRAMERESAMPLING_OFFSET;
            break;

        case kDPU_FetchEco:
            dimensionOffset = DPU_FETCHECO_FRAMEDIMENSIONS_OFFSET;
            controlOffset = DPU_FETCHECO_CONTROL_OFFSET;
            resamplingOffset = DPU_FETCHECO_FRAMERESAMPLING_OFFSET;
            break;

        case kDPU_FetchWarp:
            dimensionOffset = DPU_FETCHWARP_FRAMEDIMENSIONS_OFFSET;
            controlOffset = DPU_FETCHWARP_CONTROL_OFFSET;
            resamplingOffset = DPU_FETCHWARP_FRAMERESAMPLING_OFFSET;
            break;

        default:
            dimensionOffset = DPU_FETCHLAYER_FRAMEDIMENSIONS_OFFSET;
            controlOffset = DPU_FETCHLAYER_CONTROL_OFFSET;
            resamplingOffset = DPU_FETCHLAYER_FRAMERESAMPLING_OFFSET;
            break;
    }

    /*
     * Set <unit>_StaticControl
     * If the fetch unit supports sublayer, set the ShdLdReqSticky.
     */
    if ((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrSubLayer))
    {
        DPU_REG(base, offset + DPU_STATIC_CONTROL_OFFSET) =
            DPU_UNIT_SHDEN_MASK | DPU_FETCH_STATICCONTROL_SHDLDREQSTICKY_MASK;
    }
    else
    {
        DPU_REG(base, offset + DPU_STATIC_CONTROL_OFFSET) = DPU_UNIT_SHDEN_MASK;
    }

    /* Set input source. */
    if ((uint32_t)unit & DPU_MAKE_UNIT_ATTR(kDPU_UnitAttrHasSrc))
    {
        DPU_SetUnitSrc(base, unit, config->srcReg);
    }

    if (kDPU_FetchDecode == type)
    {
        /* Disable ring buffer. */
        ((DPU_FETCHDECODER_Type *)((uint32_t)base + offset))->RINGBUFSTARTADDR0 = 0U;
        ((DPU_FETCHDECODER_Type *)((uint32_t)base + offset))->RINGBUFWRAPADDR0 = 0U;
    }

    /* AXI burst management. */
    DPU_REG(base, offset + DPU_FETCHUNIT_BURSTBUFFERMANAGEMENT_OFFSET) =
        DPU_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS(16) | DPU_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH(16);

    /* Set <unit>_FrameDimensions. */
    DPU_REG(base, offset + dimensionOffset) = DPU_MAKE_DIMENSION(config->frameHeight - 1, config->frameWidth - 1);

    /* Set <unit>_Control. */
    reg = DPU_REG(base, offset + controlOffset) &
          ~(DPU_FETCH_CONTROL_YUV422UPSAMPLINGMODE_MASK | DPU_FETCH_CONTROL_RASTERMODE_MASK |
            DPU_FETCH_CONTROL_INPUTSELECT_MASK | DPU_FETCH_CONTROL_RAWPIXEL_MASK);

    reg |= DPU_FETCH_CONTROL_INPUTSELECT(kDPU_FetchCtrlInputSelInactive) |
           DPU_FETCH_CONTROL_RASTERMODE(kDPU_FetchCtrlRasterNormal);

    DPU_REG(base, offset + controlOffset) = reg;

    /* Set resampling. */
    DPU_REG(base, offset + resamplingOffset) = DPU_MAKE_FETCH_RESAMPLING(0, 4, 4, 0, 0);
}

void DPU_FetcUnitGetDefaultWarpConfig(dpu_warp_config_t *config)
{
    assert(config);

    config->srcReg = 0U;
    config->frameHeight = 320U;
    config->frameWidth = 480U;
    config->warpBitsPerPixel = 0U;
    config->enableSymmetricOffset = false;
    config->coordMode = kDPU_WarpCoordinateModePNT;
    config->arbStartX = 0U;
    config->arbStartY = 0U;
    config->arbDeltaYY = 0U;
    config->arbDeltaYX = 0U;
    config->arbDeltaXY = 0U;
    config->arbDeltaXX = 0U;
}

status_t DPU_InitFetchUnitWarp(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_warp_config_t *config)
{
    assert(config);

    uint32_t reg;
    DPU_FETCHWARP_Type *fetchWarp;

    if (kDPU_FetchWarp != DPU_GET_UNIT_TYPE(unit))
    {
        return kStatus_InvalidArgument;
    }

    fetchWarp = (DPU_FETCHWARP_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /*
     * Set <unit>_StaticControl
     * If the fetch unit supports sublayer, set the ShdLdReqSticky.
     */
    fetchWarp->STATICCONTROL = DPU_UNIT_SHDEN_MASK | DPU_FETCH_STATICCONTROL_SHDLDREQSTICKY_MASK;

    /* Set input source. */
    DPU_SetUnitSrc(base, unit, config->srcReg);

    /* Set <unit>_FrameDimensions. */
    fetchWarp->FRAMEDIMENSIONS = DPU_MAKE_DIMENSION(config->frameHeight - 1, config->frameWidth - 1);

    /* Set resampling. */
    fetchWarp->FRAMERESAMPLING = DPU_MAKE_FETCH_RESAMPLING(0, 4, 4, 0, 0);

    /* Setup warping. */
    fetchWarp->WARPCONTROL = DPU_FETCHWARP_WARPCONTROL_WarpBitsPerPixel(config->warpBitsPerPixel) |
                             DPU_FETCHWARP_WARPCONTROL_WarpCoordinateMode(config->coordMode) |
                             DPU_FETCHWARP_WARPCONTROL_WarpSymmetricOffset(config->enableSymmetricOffset);

    fetchWarp->ARBSTARTX = config->arbStartX;
    fetchWarp->ARBSTARTY = config->arbStartY;
    fetchWarp->ARBDELTA =
        DPU_FETCHWARP_ARBDELTA_ArbDeltaXX(config->arbDeltaXX) | DPU_FETCHWARP_ARBDELTA_ArbDeltaXY(config->arbDeltaXY) |
        DPU_FETCHWARP_ARBDELTA_ArbDeltaYX(config->arbDeltaYX) | DPU_FETCHWARP_ARBDELTA_ArbDeltaYY(config->arbDeltaYY);

    fetchWarp->BURSTBUFFERMANAGEMENT =
        DPU_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS(16) | DPU_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH(16);

    /* Set <unit>_Control. */
    reg = fetchWarp->CONTROL &
          ~(DPU_FETCH_CONTROL_YUV422UPSAMPLINGMODE_MASK | DPU_FETCH_CONTROL_RASTERMODE_MASK |
            DPU_FETCH_CONTROL_INPUTSELECT_MASK | DPU_FETCH_CONTROL_RAWPIXEL_MASK);
    fetchWarp->CONTROL = reg | DPU_FETCH_CONTROL_INPUTSELECT(kDPU_FetchCtrlInputSelCoordinate) |
                         DPU_FETCH_CONTROL_RASTERMODE(kDPU_FetchCtrlRasterArbitrary) |
                         DPU_FETCH_CONTROL_FILTERMODE(kDPU_FetchCtrlFilterBilinear);

    return kStatus_Success;
}

void DPU_CorrdinatesGetDefaultConfig(dpu_coordinates_config_t *config)
{
    assert(config);

    config->bitsPerPixel = 0U;
    config->strideBytes = 0x500U;
    config->baseAddr = 0U;
    config->frameHeight = 320U;
    config->frameWidth = 480U;
}

status_t DPU_InitWarpCoordinates(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_coordinates_config_t *config)
{
    assert(config);

    DPU_FETCHECO_Type *fetchEco;
    uint32_t dimension;
    uint32_t reg;

    if (!DPU_CheckBufferAlignment(config->bitsPerPixel, config->baseAddr, config->strideBytes))
    {
        return kStatus_InvalidArgument;
    }

    fetchEco = (DPU_FETCHECO_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    dimension = DPU_MAKE_DIMENSION(config->frameHeight - 1U, config->frameWidth - 1U);

    fetchEco->STATICCONTROL = DPU_UNIT_SHDEN_MASK;

    fetchEco->LAYER[0].BASEADDRESS = config->baseAddr;
    fetchEco->LAYER[0].SOURCEBUFFERATTRIBUTES =
        DPU_MAKE_SOURCEBUFFERATTRIBUTES(config->bitsPerPixel, config->strideBytes);
    fetchEco->LAYER[0].SOURCEBUFFERDIMENSION = dimension;
    fetchEco->LAYER[0].COLORCOMPONENTBITS = 0U;
    fetchEco->LAYER[0].COLORCOMPONENTSHIFT = 0U;
    fetchEco->LAYER[0].LAYEROFFSET = 0U;
    fetchEco->LAYER[0].LAYERPROPERTY |= DPU_LAYERPROPERTY_SOURCEBUFFERENABLE_MASK;
    fetchEco->FRAMEDIMENSIONS = dimension;

    fetchEco->BURSTBUFFERMANAGEMENT =
        DPU_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS(16) | DPU_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH(16);

    /* Enable Control.RawPixel. */
    reg = fetchEco->CONTROL & ~(DPU_FETCH_CONTROL_YUV422UPSAMPLINGMODE_MASK | DPU_FETCH_CONTROL_RASTERMODE_MASK);

    fetchEco->CONTROL =
        reg | DPU_FETCH_CONTROL_RASTERMODE(kDPU_FetchCtrlRasterNormal) | DPU_FETCH_CONTROL_RAWPIXEL_MASK;

    return kStatus_Success;
}

void DPU_SrcBufferGetDefaultConfig(dpu_src_buffer_config_t *config)
{
    assert(config);

    config->baseAddr = 0U;
    config->strideBytes = 0x500U;
    config->bitsPerPixel = 32U;
    config->pixelFormat = kDPU_PixelFormatARGB8888;
    config->bufferHeight = 0U;
    config->bufferWidth = 0U;
    config->constColor = DPU_MAKE_CONST_COLOR(0, 0, 0, 0);
}

status_t DPU_SetFetchUnitSrcBufferConfig(IRIS_MVPL_Type *base,
                                         dpu_unit_t unit,
                                         uint8_t sublayer,
                                         const dpu_src_buffer_config_t *config)
{
    assert(config);

    DPU_SUBLAYER_CONTROL_Type *control;

    /* Buffer should be aligned. */
    if (!(DPU_CheckBufferAlignment(config->bitsPerPixel, config->baseAddr, config->strideBytes)))
    {
        return kStatus_InvalidArgument;
    }

    control = DPU_GetSubLayer(base, unit, sublayer);

    control->BASEADDRESS = config->baseAddr;
    control->SOURCEBUFFERATTRIBUTES = DPU_MAKE_SOURCEBUFFERATTRIBUTES(config->bitsPerPixel, config->strideBytes);
    control->SOURCEBUFFERDIMENSION = DPU_MAKE_DIMENSION(config->bufferHeight - 1U, config->bufferWidth - 1U);
    control->COLORCOMPONENTBITS = s_dpuColorComponentFormats[config->pixelFormat][0];
    control->COLORCOMPONENTSHIFT = s_dpuColorComponentFormats[config->pixelFormat][1];
    control->CONSTANTCOLOR = config->constColor;

    return kStatus_Success;
}

void DPU_SetFetchUnitSrcBufferAddr(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer, uint32_t baseAddr)
{
    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    control->BASEADDRESS = baseAddr;
}

void DPU_SetFetchUnitFrameSize(IRIS_MVPL_Type *base, dpu_unit_t unit, uint16_t height, uint16_t width)
{
    uint32_t offset;
    uint32_t type;
    uint32_t dimensionOffset;

    offset = DPU_GET_UNIT_OFFSET(unit);
    type = DPU_GET_UNIT_TYPE(unit);

    switch (type)
    {
        case kDPU_FetchDecode:
            dimensionOffset = DPU_FETCHDECODE_FRAMEDIMENSIONS_OFFSET;
            break;

        case kDPU_FetchEco:
            dimensionOffset = DPU_FETCHECO_FRAMEDIMENSIONS_OFFSET;
            break;

        case kDPU_FetchWarp:
            dimensionOffset = DPU_FETCHWARP_FRAMEDIMENSIONS_OFFSET;
            break;

        default:
            dimensionOffset = DPU_FETCHLAYER_FRAMEDIMENSIONS_OFFSET;
            break;
    }

    /* Set <unit>_FrameDimensions. */
    DPU_REG(base, offset + dimensionOffset) = DPU_MAKE_DIMENSION(height - 1, width - 1);
}

void DPU_SetFetchUnitOffset(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer, uint16_t offsetX, uint16_t offsetY)
{
    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    control->LAYEROFFSET = DPU_MAKE_POSITION(offsetY, offsetX);
}

void DPU_EnableFetchUnitSrcBuffer(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer, bool enable)
{
    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    if (enable)
    {
        control->LAYERPROPERTY |= DPU_LAYERPROPERTY_SOURCEBUFFERENABLE_MASK;
    }
    else
    {
        control->LAYERPROPERTY &= ~DPU_LAYERPROPERTY_SOURCEBUFFERENABLE_MASK;
    }
}

void DPU_ClipWindowGetDefaultConfig(dpu_clip_window_config_t *config)
{
    assert(config);

    config->windowOffsetX = 0U;
    config->windowOffsetY = 0U;
    config->windowHeight = 0U;
    config->windowWidth = 0U;
}

void DPU_SetFetchUnitClipWindowConfig(IRIS_MVPL_Type *base,
                                      dpu_unit_t unit,
                                      uint8_t sublayer,
                                      const dpu_clip_window_config_t *config)
{
    assert(config);

    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    control->CLIPWINDOWOFFSET = DPU_MAKE_POSITION(config->windowOffsetY, config->windowOffsetX);
    control->CLIPWINDOWDIMENSIONS = DPU_MAKE_DIMENSION(config->windowHeight, config->windowWidth);
}

void DPU_EnableFetchUnitClipWindow(IRIS_MVPL_Type *base, dpu_unit_t unit, uint8_t sublayer, bool enable)
{
    DPU_SUBLAYER_CONTROL_Type *control = DPU_GetSubLayer(base, unit, sublayer);

    if (enable)
    {
        control->LAYERPROPERTY |= DPU_LAYERPROPERTY_CLIPWINDOWENABLE_MASK;
    }
    else
    {
        control->LAYERPROPERTY &= ~DPU_LAYERPROPERTY_CLIPWINDOWENABLE_MASK;
    }
}

void DPU_SetFetchUnitClipColor(IRIS_MVPL_Type *base,
                               dpu_unit_t unit,
                               dpu_clip_color_mode_t clipColorMode,
                               uint8_t sublayer)
{
    uint32_t reg;
    uint32_t offset;
    uint32_t type;
    uint32_t controlOffset;

    offset = DPU_GET_UNIT_OFFSET(unit);
    type = DPU_GET_UNIT_TYPE(unit);

    switch (type)
    {
        case kDPU_FetchDecode:
            controlOffset = DPU_FETCHDECODE_CONTROL_OFFSET;
            break;

        case kDPU_FetchEco:
            controlOffset = DPU_FETCHECO_CONTROL_OFFSET;
            break;

        case kDPU_FetchWarp:
            controlOffset = DPU_FETCHWARP_CONTROL_OFFSET;
            break;

        default:
            controlOffset = DPU_FETCHLAYER_CONTROL_OFFSET;
            break;
    }

    /* Set <unit>_Control. */
    reg =
        DPU_REG(base, offset + controlOffset) & ~(DPU_FETCH_CONTROL_CLIPCOLOR_MASK | DPU_FETCH_CONTROL_CLIPLAYER_MASK);

    if (kDPU_ClipColorNull != clipColorMode)
    {
        reg |= DPU_FETCH_CONTROL_CLIPCOLOR_MASK;
        reg |= DPU_FETCH_CONTROL_CLIPLAYER(sublayer);
    }

    DPU_REG(base, offset + controlOffset) = reg;
}

void DPU_InitExtDst(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    DPU_EXTDST_Type *extDst;

    assert(kDPU_ExtDst == DPU_GET_UNIT_TYPE(unit));

    extDst = (DPU_EXTDST_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Set KICK_MODE to external. */
    extDst->STATICCONTROL = DPU_EXTDST_STATICCONTROL_KICKMODE_MASK | DPU_UNIT_SHDEN_MASK;

    /* Set input source selection (the register dynamic). */
    DPU_SetUnitSrc(base, unit, srcReg);
}

void DPU_InitStore(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    DPU_STORE_Type *store;

    assert(kDPU_Store == DPU_GET_UNIT_TYPE(unit));

    store = (DPU_STORE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Enable Shadow. */
    store->STATICCONTROL = DPU_UNIT_SHDEN_MASK;

    /* Disable ring buffer. */
    store->RINGBUFSTARTADDR = 0U;
    store->RINGBUFWRAPADDR = 0U;

    /* Burst management. */
    store->BURSTBUFFERMANAGEMENT = DPU_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH(16);

    /* Set input source selection (the register dynamic). */
    DPU_SetUnitSrc(base, unit, srcReg);
}

status_t DPU_SetStoreDstBufferConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_dst_buffer_config_t *config)
{
    assert(config);

    /* Buffer should be aligned. */
    if (!(DPU_CheckBufferAlignment(config->bitsPerPixel, config->baseAddr, config->strideBytes)))
    {
        return kStatus_InvalidArgument;
    }

    DPU_STORE_Type *store = (DPU_STORE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    store->BASEADDRESS = config->baseAddr;
    store->DESTINATIONBUFFERATTRIBUTES =
        DPU_MAKE_DESTINATIONBUFFERATTRIBUTES(config->bitsPerPixel, config->strideBytes);
    store->DESTINATIONBUFFERDIMENSION = DPU_MAKE_DIMENSION(config->bufferHeight - 1U, config->bufferWidth - 1U);
    store->COLORCOMPONENTBITS = s_dpuColorComponentFormats[config->pixelFormat][0];
    store->COLORCOMPONENTSHIFT = s_dpuColorComponentFormats[config->pixelFormat][1];

    return kStatus_Success;
}

void DPU_DstBufferGetDefaultConfig(dpu_dst_buffer_config_t *config)
{
    assert(config);

    config->baseAddr = 0U;
    config->strideBytes = 0x500U;
    config->bitsPerPixel = 32U;
    config->pixelFormat = kDPU_PixelFormatARGB8888;
    config->bufferHeight = 0U;
    config->bufferWidth = 0U;
}

void DPU_SetStoreDstBufferAddr(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t baseAddr)
{
    DPU_STORE_Type *store = (DPU_STORE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    store->BASEADDRESS = baseAddr;
}

void DPU_SetStoreOffset(IRIS_MVPL_Type *base, dpu_unit_t unit, uint16_t offsetX, uint16_t offsetY)
{
    DPU_STORE_Type *store = (DPU_STORE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    store->FRAMEOFFSET = DPU_MAKE_POSITION(offsetY, offsetX);
}

void DPU_StartStore(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    DPU_STORE_Type *store = (DPU_STORE_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    store->START = DPU_STORE_START_Start_MASK;
}

void DPU_LayerBlendGetDefaultConfig(dpu_layer_blend_config_t *config)
{
    assert(config);

    config->constAlpha = 0U;
    config->secAlphaBlendMode = kDPU_BlendOne;
    config->primAlphaBlendMode = kDPU_BlendZero;
    config->secColorBlendMode = kDPU_BlendOne;
    config->primColorBlendMode = kDPU_BlendZero;
    config->enableAlphaMask = false;
    config->alphaMaskMode = kDPU_AlphaMaskPrim;
}

void DPU_InitLayerBlend(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    DPU_LAYERBLEND_Type *layerBlend;

    assert(kDPU_LayerBlend == DPU_GET_UNIT_TYPE(unit));

    layerBlend = (DPU_LAYERBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Shadow token generate mode: BOTH. Shadow load mode: Both. */
    layerBlend->STATICCONTROL = DPU_LAYERBLEND_STATICCONTROL_ShdEn_MASK |
                                DPU_LAYERBLEND_STATICCONTROL_ShdTokSel(kDPU_LayerBlendShadowTokenBoth) |
                                DPU_LAYERBLEND_STATICCONTROL_ShdLdSel(kDPU_LayerBlendShadowLoadBoth);

    /*
     * Disable second frame overlay position. Explaination from the spec:
     * Note: Technically the foreground plane can be setup with a smaller dimension
     * than the background and placed by the LayerBlend unit itself (Position.XPOS/YPOS),
     * however, this must not be done for the following reasons:
     *   - Layer overlay must be completely inside the background area then, otherwise
     *   display tearing may result. This would require clip window computations by SW.
     *   - Timing setup for certain use cases is not valid any longer, because Fetch unit
     *   starts fetching source buffer data for a layer not at position of first overlay
     *   pixel, but at first pixel of the background plane already.
     *   - Layer position is not part of the shadow load domain correlated to the layer,
     *   but to the stream.
     */
    layerBlend->POSITION = 0U;

    DPU_SetUnitSrc(base, unit, srcReg);
}

void DPU_SetLayerBlendConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_layer_blend_config_t *config)
{
    assert(config);

    DPU_LAYERBLEND_Type *layerBlend;

    layerBlend = (DPU_LAYERBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    /* Set layer blend control. */
    layerBlend->BLENDCONTROL = DPU_LAYERBLEND_BLENDCONTROL_BlendAlpha(config->constAlpha) |
                               DPU_LAYERBLEND_BLENDCONTROL_PRIM_C_BLD_FUNC(config->primColorBlendMode) |
                               DPU_LAYERBLEND_BLENDCONTROL_SEC_C_BLD_FUNC(config->secColorBlendMode) |
                               DPU_LAYERBLEND_BLENDCONTROL_PRIM_A_BLD_FUNC(config->primAlphaBlendMode) |
                               DPU_LAYERBLEND_BLENDCONTROL_SEC_A_BLD_FUNC(config->secAlphaBlendMode);

    /* Set alpha mask config. */
    layerBlend->CONTROL =
        ((layerBlend->CONTROL &
          ~(DPU_LAYERBLEND_CONTROL_AlphaMaskEnable_MASK | DPU_LAYERBLEND_CONTROL_AlphaMaskMode_MASK)) |
         DPU_LAYERBLEND_CONTROL_AlphaMaskEnable(config->enableAlphaMask) |
         DPU_LAYERBLEND_CONTROL_AlphaMaskMode(config->alphaMaskMode));
}

void DPU_EnableLayerBlend(IRIS_MVPL_Type *base, dpu_unit_t unit, bool enable)
{
    DPU_LAYERBLEND_Type *layerBlend = (DPU_LAYERBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    if (enable)
    {
        layerBlend->CONTROL |= DPU_LAYERBLEND_CONTROL_MODE_MASK;
    }
    else
    {
        layerBlend->CONTROL &= ~DPU_LAYERBLEND_CONTROL_MODE_MASK;
    }
}

void DPU_InitRop(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    assert(kDPU_Rop == DPU_GET_UNIT_TYPE(unit));

    DPU_ROP_Type *rop = (DPU_ROP_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    rop->STATICCONTROL = DPU_UNIT_SHDEN_MASK;

    DPU_SetUnitSrc(base, unit, srcReg);
}

void DPU_RopGetDefaultConfig(dpu_rop_config_t *config)
{
    assert(config);

    config->controlFlags = 0U;
    config->alphaIndex = 0U;
    config->blueIndex = 0U;
    config->greenIndex = 0U;
    config->redIndex = 0U;
}

void DPU_SetRopConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_rop_config_t *config)
{
    assert(config);

    DPU_ROP_Type *rop = (DPU_ROP_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    rop->RASTEROPERATIONINDICES =
        DPU_MAKE_CONST_COLOR(config->redIndex, config->greenIndex, config->blueIndex, config->alphaIndex);
    rop->CONTROL = (rop->CONTROL & DPU_ROP_CONTROL_Mode_MASK) | config->controlFlags;
}

void DPU_EnableRop(IRIS_MVPL_Type *base, dpu_unit_t unit, bool enable)
{
    DPU_ROP_Type *rop = (DPU_ROP_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    if (enable)
    {
        rop->CONTROL |= DPU_ROP_CONTROL_Mode_MASK;
    }
    else
    {
        rop->CONTROL &= ~DPU_ROP_CONTROL_Mode_MASK;
    }
}

void DPU_InitBlitBlend(IRIS_MVPL_Type *base, dpu_unit_t unit, uint32_t srcReg)
{
    assert(kDPU_BlitBlend == DPU_GET_UNIT_TYPE(unit));

    DPU_BLITBLEND_Type *blitBlend = (DPU_BLITBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    blitBlend->STATICCONTROL = DPU_UNIT_SHDEN_MASK;

    DPU_SetUnitSrc(base, unit, srcReg);
}

void DPU_BlitBlendGetDefaultConfig(dpu_blit_blend_config_t *config)
{
    assert(config);

    config->neutralBorderRightPixels = 0U;
    config->neutralBorderLeftPixels = 0U;
    config->neutralBorderMode = kDPU_BlitBlendNeutralBorderPrim;

    config->constColor = DPU_MAKE_CONST_COLOR(0, 0, 0, 0);

    config->redBlendFuncSrc = kDPU_BlitBlendFuncGlSrcColor;
    config->redBlendFuncDst = kDPU_BlitBlendFuncGlSrcColor;
    config->greenBlendFuncSrc = kDPU_BlitBlendFuncGlSrcColor;
    config->greenBlendFuncDst = kDPU_BlitBlendFuncGlSrcColor;
    config->blueBlendFuncSrc = kDPU_BlitBlendFuncGlSrcColor;
    config->blueBlendFuncDst = kDPU_BlitBlendFuncGlSrcColor;
    config->alphaBlendFuncSrc = kDPU_BlitBlendFuncGlSrcColor;
    config->alphaBlendFuncDst = kDPU_BlitBlendFuncGlSrcColor;
    config->redBlendMode = kDPU_BlitBlendModeGlFuncAdd;
    config->greenBlendMode = kDPU_BlitBlendModeGlFuncAdd;
    config->blueBlendMode = kDPU_BlitBlendModeGlFuncAdd;
    config->alphaBlendMode = kDPU_BlitBlendModeGlFuncAdd;
}

void DPU_SetBlitBlendConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_blit_blend_config_t *config)
{
    assert(config);

    DPU_BLITBLEND_Type *blitBlend = (DPU_BLITBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    blitBlend->NEUTRALBORDER = DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderLeft(config->neutralBorderLeftPixels) |
                               DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderRight(config->neutralBorderRightPixels) |
                               DPU_BLITBLEND_NEUTRALBORDER_NeutralBorderMode(config->neutralBorderMode);

    blitBlend->CONSTANTCOLOR = config->constColor;

    blitBlend->COLORREDBLENDFUNCTION =
        (((uint32_t)config->redBlendFuncDst) << 16U) | (uint32_t)(config->redBlendFuncSrc);
    blitBlend->COLORGREENBLENDFUNCTION =
        (((uint32_t)config->greenBlendFuncDst) << 16U) | (uint32_t)(config->greenBlendFuncSrc);
    blitBlend->COLORBLUEBLENDFUNCTION =
        (((uint32_t)config->blueBlendFuncDst) << 16U) | (uint32_t)(config->blueBlendFuncSrc);
    blitBlend->ALPHABLENDFUNCTION =
        (((uint32_t)config->alphaBlendFuncDst) << 16U) | (uint32_t)(config->alphaBlendFuncSrc);
    blitBlend->BLENDMODE1 = (((uint32_t)config->greenBlendMode) << 16U) | (uint32_t)(config->redBlendMode);
    blitBlend->BLENDMODE2 = (((uint32_t)config->alphaBlendMode) << 16U) | (uint32_t)(config->blueBlendMode);
}

void DPU_EnableBlitBlend(IRIS_MVPL_Type *base, dpu_unit_t unit, bool enable)
{
    DPU_BLITBLEND_Type *blitBlend = (DPU_BLITBLEND_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    if (enable)
    {
        blitBlend->CONTROL = DPU_BLITBLEND_CONTROL_Mode_MASK;
    }
    else
    {
        blitBlend->CONTROL = 0U;
    }
}

void DPU_InitConstFrame(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert(kDPU_ConstFrame == DPU_GET_UNIT_TYPE(unit));

    DPU_CONSTFRAME_Type *constFrame = (DPU_CONSTFRAME_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    constFrame->STATICCONTROL = DPU_UNIT_SHDEN_MASK;
}

void DPU_ConstFrameGetDefaultConfig(dpu_const_frame_config_t *config)
{
    assert(config);

    config->frameHeight = 320U;
    config->frameWidth = 480U;
    config->constColor = DPU_MAKE_CONST_COLOR(0xFF, 0xFF, 0xFF, 0xFF);
}

void DPU_SetConstFrameConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_const_frame_config_t *config)
{
    assert(config);

    DPU_CONSTFRAME_Type *constFrame = (DPU_CONSTFRAME_Type *)(((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit));

    constFrame->FRAMEDIMENSIONS = DPU_MAKE_DIMENSION(config->frameHeight - 1U, config->frameWidth - 1U);
    constFrame->CONSTANTCOLOR = config->constColor;
}

void DPU_InitScaler(IRIS_MVPL_Type *base, dpu_unit_t unit)
{
    assert((kDPU_VScaler == DPU_GET_UNIT_TYPE(unit)) || ((kDPU_HScaler == DPU_GET_UNIT_TYPE(unit))));

    DPU_EnableShadowLoad(base, unit, true);
}

void DPU_ScalerGetDefaultConfig(dpu_scaler_config_t *config)
{
    assert(config);

    config->srcReg = 0U;
    config->inputSize = 0U;
    config->outputSize = 0U;
}

void DPU_SetScalerConfig(IRIS_MVPL_Type *base, dpu_unit_t unit, const dpu_scaler_config_t *config)
{
    assert(config);

    float scaleFact;
    uint32_t control;
    uint32_t setup1;
    uint32_t offset = ((uint32_t)base) + DPU_GET_UNIT_OFFSET(unit);

    control = DPU_SCALER_CONTROL_OUTPUTSIZE(config->outputSize - 1U) | DPU_SCALER_CONTROL_FILTERMODE_MASK |
              DPU_SCALER_CONTROL_MODE_MASK;

    if (config->inputSize > config->outputSize)
    {
        scaleFact = (float)config->outputSize / (float)config->inputSize;
    }
    else
    {
        /* Output size is larger, use up-scaling mode. */
        control |= DPU_SCALER_CONTROL_SCALEMODE_MASK;
        scaleFact = (float)config->inputSize / (float)config->outputSize;
    }

    /*
     * The scale fact is a float value (0.0 to 1.0], it should be converted to
     * fixed 1.19 value and set to register SETUP1.
     */
    setup1 = DPU_ConvertFloat(scaleFact, 1, 19);

    if (0U == setup1)
    {
        setup1 = 1U;
    }
    else if (setup1 > 0x80000U)
    {
        setup1 = 0x80000U;
    }
    else
    {
    }

    ((DPU_VSCALER_Type *)offset)->SETUP1 = setup1;

    if (kDPU_VScaler == DPU_GET_UNIT_TYPE(unit))
    {
        ((DPU_VSCALER_Type *)offset)->CONTROL = control;
    }
    else
    {
        ((DPU_HSCALER_Type *)offset)->CONTROL = control;
    }

    DPU_SetUnitSrc(base, unit, config->srcReg);
}

void DPU_DisplayTimingGetDefaultConfig(dpu_display_timing_config_t *config)
{
    assert(config);

    config->flags = kDPU_DisplayDataEnableActiveHigh;
    config->width = 320U;
    config->hsw = 32U;
    config->hfp = 8U;
    config->hbp = 40U;
    config->height = 240U;
    config->vsw = 4U;
    config->vfp = 13U;
    config->vbp = 6U;
}

void DPU_InitDisplayTiming(IRIS_MVPL_Type *base, uint8_t displayIndex, const dpu_display_timing_config_t *config)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    uint32_t reg;
    uint16_t vtotal;
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);
    DPU_DISENGCONF_Type *diseng = (DPU_DISENGCONF_Type *)(((uint32_t)base) + DPU_DISENGCONF_OFFSET);

    display->FRAMEGEN.FGSTCTRL |= DPU_UNIT_SHDEN_MASK;

    /* Set signal polarity. */
    reg = config->flags & (kDPU_DisplayPixelActiveLow | kDPU_DisplayDataEnableActiveHigh | kDPU_DisplayHsyncActiveHigh |
                           kDPU_DisplayVsyncActiveHigh);

    diseng->DISPLAY[displayIndex].POLARITYCTRL = reg;

    /* Set timing. */
    vtotal = config->height + config->vfp + config->vbp + config->vsw - 1U;
    display->FRAMEGEN.HTCFG1 =
        DPU_FRAMEGEN_HTCFG1_Htotal(config->width + config->hfp + config->hbp + config->hsw - 1U) |
        DPU_FRAMEGEN_HTCFG1_Hact(config->width);

    display->FRAMEGEN.HTCFG2 = DPU_FRAMEGEN_HTCFG2_Hsync(config->hsw - 1U) |
                               DPU_FRAMEGEN_HTCFG2_Hsbp(config->hbp + config->hsw - 1U) | DPU_FRAMEGEN_HTCFG2_HsEn_MASK;

    display->FRAMEGEN.VTCFG1 = DPU_FRAMEGEN_VTCFG1_Vtotal(vtotal) | DPU_FRAMEGEN_VTCFG1_Vact(config->height);

    display->FRAMEGEN.VTCFG2 = DPU_FRAMEGEN_VTCFG2_Vsync(config->vsw - 1U) |
                               DPU_FRAMEGEN_VTCFG2_Vsbp(config->vbp + config->vsw - 1U) | DPU_FRAMEGEN_VTCFG2_VsEn_MASK;

    /* Set TCON for the signal mapping. */
    display->TCON.TCON_CTRL = display->TCON.TCON_CTRL & ~DPU_TCON_TCON_CTRL_Bypass_MASK;
    display->TCON.MAPBIT3_0 = 0x17161514U;
    display->TCON.MAPBIT7_4 = 0x1b1a1918U;
    display->TCON.MAPBIT11_8 = 0x0b0a1d1cU;
    display->TCON.MAPBIT15_12 = 0x0f0e0d0cU;
    display->TCON.MAPBIT19_16 = 0x13121110U;
    display->TCON.MAPBIT23_20 = 0x03020100U;
    display->TCON.MAPBIT27_24 = 0x07060504U;
    display->TCON.MAPBIT31_28 = 0x00000908U;
    display->TCON.MAPBIT34_32 = 0x00222120U;

    /* Set TCON for the SYNC signals. */
    display->TCON.SPG[0].SPGPOSON = DPU_TCON_SPGPOSON_SPGPSON_X(config->width + config->hfp);
    display->TCON.SPG[0].SPGMASKON = DPU_TCON_SPGPOSON_SPGPSON_Y_MASK;
    display->TCON.SPG[0].SPGPOSOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_X(config->width + config->hfp + config->hsw);
    display->TCON.SPG[0].SPGMASKOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_Y_MASK;

    display->TCON.SPG[1].SPGPOSON = DPU_TCON_SPGPOSON_SPGPSON_X(config->width + config->hfp) |
                                    DPU_TCON_SPGPOSON_SPGPSON_Y(config->height + config->vfp - 1);
    display->TCON.SPG[1].SPGMASKON = 0;
    display->TCON.SPG[1].SPGPOSOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_X(config->width + config->hfp) |
                                     DPU_TCON_SPGPOSOFF_SPGPSOFF_Y(config->height + config->vfp + config->vsw - 1);
    display->TCON.SPG[1].SPGMASKOFF = 0;

    display->TCON.SPG[2].SPGPOSON = DPU_TCON_SPGPOSON_SPGPSON_X(0U);
    display->TCON.SPG[2].SPGMASKON = DPU_TCON_SPGPOSON_SPGPSON_Y_MASK;
    display->TCON.SPG[2].SPGPOSOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_X(config->width);
    display->TCON.SPG[2].SPGMASKOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_Y_MASK;

    display->TCON.SPG[3].SPGPOSON = DPU_TCON_SPGPOSON_SPGPSON_Y(0U);
    display->TCON.SPG[3].SPGMASKON = DPU_TCON_SPGPOSON_SPGPSON_X_MASK;
    display->TCON.SPG[3].SPGPOSOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_Y(config->height);
    display->TCON.SPG[3].SPGMASKOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_X_MASK;

    display->TCON.SPG[4].SPGPOSON = DPU_TCON_SPGPOSON_SPGPSON_Y(config->height) | DPU_TCON_SPGPOSON_SPGPSON_X(0x0AU);
    display->TCON.SPG[4].SPGMASKON = 0U;
    display->TCON.SPG[4].SPGPOSOFF = DPU_TCON_SPGPOSOFF_SPGPSOFF_Y(config->height) | DPU_TCON_SPGPOSON_SPGPSON_X(0x2AU);
    display->TCON.SPG[4].SPGMASKOFF = 0U;

    /* HSYNC. */
    display->TCON.SMX[0].SMXSIGS = DPU_TCON_SMXSIGS_SMXSIGS_S0(0x02U);
    display->TCON.SMX[0].SMXFCTTABLE = 0x01U;

    /* VSYNC. */
    display->TCON.SMX[1].SMXSIGS = DPU_TCON_SMXSIGS_SMXSIGS_S0(0x03U);
    display->TCON.SMX[1].SMXFCTTABLE = 0x01U;

    /* Data output enable. */
    display->TCON.SMX[2].SMXSIGS = DPU_TCON_SMXSIGS_SMXSIGS_S0(0x04U) | DPU_TCON_SMXSIGS_SMXSIGS_S1(0x05U);
    display->TCON.SMX[2].SMXFCTTABLE = 0x08U;

    /* Sync signal to trigger shadow load. This should be at least 32 pixel clock after KICK signal. */
    display->TCON.SMX[3].SMXSIGS = DPU_TCON_SMXSIGS_SMXSIGS_S0(0x06U);
    display->TCON.SMX[3].SMXFCTTABLE = 0x02U;

    /* KICK signal set to start of last vertical blanking line. */
    display->FRAMEGEN.PKICKCONFIG = DPU_FRAMEGEN_PKICKCONFIG_PKickRow(config->height) |
                                    DPU_FRAMEGEN_PKICKCONFIG_PKickCol(config->width + 1) |
                                    DPU_FRAMEGEN_PKICKCONFIG_PKickEn_MASK;

    display->FRAMEGEN.SKICKCONFIG = DPU_FRAMEGEN_SKICKCONFIG_SKickRow(config->height) |
                                    DPU_FRAMEGEN_SKICKCONFIG_SKickCol(config->width + 1) |
                                    DPU_FRAMEGEN_SKICKCONFIG_SKickEn_MASK;
}

void DPU_DisplayGetDefaultConfig(dpu_display_config_t *config)
{
    assert(config);

    config->enablePrimAlpha = false;
    config->enableSecAlpha = false;
    config->displayMode = kDPU_DisplayTest;
    config->enablePrimAlphaInPanic = false;
    config->enableSecAlphaInPanic = false;
    config->displayModeInPanic = kDPU_DisplayTest;
    config->constRed = 0x3FFU;
    config->constGreen = 0x3FFU;
    config->constBlue = 0x3FFU;
    config->constAlpha = 1U;
    config->primAreaStartX = 1U;
    config->primAreaStartY = 1U;
    config->secAreaStartX = 1U;
    config->secAreaStartY = 1U;
}

void DPU_SetDisplayConfig(IRIS_MVPL_Type *base, uint8_t displayIndex, const dpu_display_config_t *config)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    uint32_t reg;
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    /* Area configure. */
    display->FRAMEGEN.PACFG =
        DPU_FRAMEGEN_PACFG_Pstartx(config->primAreaStartX) | DPU_FRAMEGEN_PACFG_Pstarty(config->primAreaStartY);
    display->FRAMEGEN.SACFG =
        DPU_FRAMEGEN_SACFG_Sstartx(config->secAreaStartX) | DPU_FRAMEGEN_SACFG_Sstarty(config->secAreaStartY);

    /* Input control. */
    reg = DPU_FRAMEGEN_FGINCTRL_FgDm(config->displayMode);

    if (config->enableSecAlpha)
    {
        reg |= DPU_FRAMEGEN_FGINCTRL_EnSecAlpha_MASK;
    }

    if (config->enablePrimAlpha)
    {
        reg |= DPU_FRAMEGEN_FGINCTRL_EnPrimAlpha_MASK;
    }

    display->FRAMEGEN.FGINCTRL = reg;

    reg = DPU_FRAMEGEN_FGINCTRL_FgDm(config->displayModeInPanic);

    if (config->enableSecAlphaInPanic)
    {
        reg |= DPU_FRAMEGEN_FGINCTRL_EnSecAlpha_MASK;
    }

    if (config->enablePrimAlphaInPanic)
    {
        reg |= DPU_FRAMEGEN_FGINCTRL_EnPrimAlpha_MASK;
    }
    display->FRAMEGEN.FGINCTRLPANIC = reg;

    /* Configure constant color. */
    display->FRAMEGEN.FGCCR =
        DPU_FRAMEGEN_FGCCR_CcBlue(config->constBlue) | DPU_FRAMEGEN_FGCCR_CcGreen(config->constGreen) |
        DPU_FRAMEGEN_FGCCR_CcRed(config->constRed) | DPU_FRAMEGEN_FGCCR_CcAlpha(config->constAlpha);
}

void DPU_StartDisplay(IRIS_MVPL_Type *base, uint8_t displayIndex)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->FRAMEGEN.FGENABLE = DPU_FRAMEGEN_FGENABLE_FgEn_MASK;
}

void DPU_StopDisplay(IRIS_MVPL_Type *base, uint8_t displayIndex)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->FRAMEGEN.FGENABLE = 0U;
}

void DPU_SetFrameGenInterruptConfig(IRIS_MVPL_Type *base,
                                    uint8_t displayIndex,
                                    uint8_t interruptIndex,
                                    uint32_t intConfig)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    /* Write the intConfig to INT0CONFIG, INT1CONFIG, INT2CONFIG, or INT3CONFIG. */
    display->FRAMEGEN.INTCONFIG[interruptIndex] = intConfig;
}

void DPU_TriggerDisplayShadowLoad(IRIS_MVPL_Type *base, uint8_t displayIndex)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->FRAMEGEN.FGSLR = DPU_FRAMEGEN_FGSLR_ShdTokGen_MASK;
}

void DPU_SignatureGetDefaultConfig(dpu_signature_config_t *config)
{
    assert(config);

    config->errorThreshold = 0U;
    config->errorResetThreshold = 8U;
    config->panicRed = 0U;
    config->panicGreen = 0U;
    config->panicBlue = 0U;
    config->panicAlpha = 0U;
}

void DPU_InitSignature(IRIS_MVPL_Type *base, uint8_t displayIndex, const dpu_signature_config_t *config)
{
    assert(displayIndex < DPU_DISPLAY_COUNT);
    uint32_t panicColor;

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->SIG.STATICCONTROL =
        (((uint32_t)config->errorThreshold) << DPU_SIG_STATICCONTROL_ErrThres_SHIFT) |
        (((uint32_t)config->errorResetThreshold) << DPU_SIG_STATICCONTROL_ErrThresReset_SHIFT) |
        DPU_SIG_STATICCONTROL_ShdEn_MASK;
    panicColor = ((uint32_t)config->panicRed << DPU_SIG_PANICCOLOR_PanicRed_SHIFT) |
                 ((uint32_t)config->panicGreen << DPU_SIG_PANICCOLOR_PanicGreen_SHIFT) |
                 ((uint32_t)config->panicBlue << DPU_SIG_PANICCOLOR_PanicBlue_SHIFT);

    if (0U != config->panicAlpha)
    {
        panicColor |= DPU_SIG_PANICCOLOR_PanicAlpha_MASK;
    }

    display->SIG.PANICCOLOR = panicColor;
    display->SIG.CONTINUOUSMODE = DPU_SIG_CONTINUOUSMODE_EnCont_MASK;
}

void DPU_SignatureWindowGetDefaultConfig(dpu_signature_window_config_t *config)
{
    assert(config);

    config->controlFlags = 0U;
    config->upperLeftX = 0U;
    config->upperLeftY = 0U;
    config->lowerRightX = 0U;
    config->lowerRightY = 0U;
}

void DPU_SetSignatureWindowConfig(IRIS_MVPL_Type *base,
                                  uint8_t displayIndex,
                                  uint8_t windowIndex,
                                  const dpu_signature_window_config_t *config)
{
    assert(windowIndex < DPU_SIG_EAVL_WINDOW_COUNT);

    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->SIG.WINDOW[windowIndex].EVALCONTROL =
        (display->SIG.WINDOW[windowIndex].EVALCONTROL &
         (DPU_SIG_EVALCONTROL_EnEvalWin_MASK | DPU_SIG_EVALCONTROL_EnCRC_MASK)) |
        config->controlFlags;
    display->SIG.WINDOW[windowIndex].EVALUPPERLEFT =
        ((uint32_t)config->upperLeftX << DPU_SIG_EVALUPPERLEFT_XEvalUpperLeft_SHIFT) |
        ((uint32_t)config->upperLeftY << DPU_SIG_EVALUPPERLEFT_YEvalUpperLeft_SHIFT);
    display->SIG.WINDOW[windowIndex].EVALLOWERRIGHT =
        ((uint32_t)config->lowerRightX << DPU_SIG_EVALLOWERRIGHT_XEvalLowerRight_SHIFT) |
        ((uint32_t)config->lowerRightY << DPU_SIG0_EVALLOWERRIGHT_YEvalLowerRight_SHIFT);
}

void DPU_EnableSignatureWindowCompute(IRIS_MVPL_Type *base, uint8_t displayIndex, uint8_t windowIndex, bool enable)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    if (enable)
    {
        display->SIG.WINDOW[windowIndex].EVALCONTROL |= DPU_SIG_EVALCONTROL_EnEvalWin_MASK;
    }
    else
    {
        display->SIG.WINDOW[windowIndex].EVALCONTROL &= ~DPU_SIG_EVALCONTROL_EnEvalWin_MASK;
    }
}

void DPU_EnableSignatureWindowCheck(IRIS_MVPL_Type *base, uint8_t displayIndex, uint8_t windowIndex, bool enable)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    if (enable)
    {
        display->SIG.WINDOW[windowIndex].EVALCONTROL |= DPU_SIG_EVALCONTROL_EnCRC_MASK;
    }
    else
    {
        display->SIG.WINDOW[windowIndex].EVALCONTROL &= ~DPU_SIG_EVALCONTROL_EnCRC_MASK;
    }
}

void DPU_SetSignatureWindowRefCrc(IRIS_MVPL_Type *base,
                                  uint8_t displayIndex,
                                  uint8_t windowIndex,
                                  uint32_t redCRC,
                                  uint32_t greenCRC,
                                  uint32_t blueCRC)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->SIG.WINDOW[windowIndex].SIGCRCREDREF = redCRC;
    display->SIG.WINDOW[windowIndex].SIGCRCGREENREF = greenCRC;
    display->SIG.WINDOW[windowIndex].SIGCRCBLUEREF = blueCRC;
}

void DPU_GetSignatureWindowCrc(IRIS_MVPL_Type *base,
                               uint8_t displayIndex,
                               uint8_t windowIndex,
                               uint32_t *redCRC,
                               uint32_t *greenCRC,
                               uint32_t *blueCRC)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    *redCRC = display->SIG.WINDOW[windowIndex].SIGCRCRED;
    *greenCRC = display->SIG.WINDOW[windowIndex].SIGCRCGREEN;
    *blueCRC = display->SIG.WINDOW[windowIndex].SIGCRCBLUE;
}

uint32_t DPU_GetSignatureStatus(IRIS_MVPL_Type *base, uint8_t displayIndex)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    return display->SIG.STATUS;
}

void DPU_TriggerSignatureShadowLoad(IRIS_MVPL_Type *base, uint8_t displayIndex)
{
    DPU_DISPLAY_Type *display = (DPU_DISPLAY_Type *)(((uint32_t)base) + s_displayOffsetArray[displayIndex]);

    display->SIG.SHADOWLOAD = DPU_SIG_SHADOWLOAD_ShdLdReq_MASK;
}
