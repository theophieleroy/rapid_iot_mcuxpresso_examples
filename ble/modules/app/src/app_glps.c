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
#include "app_ble.h"
#include "fsl_flash.h"
#include "prf_utils.h"
#include "NVM_Interface.h"
#if BLE_GL_SENSOR

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define GRM_SPACE_0_ADDR (0x21040000U)
#define GRM_SPACE_1_ADDR (0x21040800U)
#define GRM_SPACE_SIZE (2048U)

#define GRM_MAGIC_NUMBER   \
    {                      \
        'G', 'L', 'P', 'S' \
    };
#define GRM_MAGIC_NUMBER_LEN (4U)

#define GRM_STATUS_LEN (2U)
#define GRM_SN_LEN (2U)
#define GRM_MEAS_LEN (18U)
#define GRM_CTX_LEN (16U)
#define GRM_RECORD_LEN (GRM_STATUS_LEN + GRM_SN_LEN + GRM_MEAS_LEN + GRM_CTX_LEN)
#define GRM_RECORD_MAX 40

#define GRM_RECORD_VALID_MASK (0x0001)
#define GRM_RECORD_VALID (0x0000)
#define GRM_RECORD_NOT_VALID (0x0001)
#define GRM_RECORD_DELETED_MASK (0x0002)
#define GRM_RECORD_DELETED (0x0000)
#define GRM_RECORD_NOT_DELETED (0x0002)
#define GRM_RECORD_EMPTY_MASK (0xFFFF)
#define GRM_RECORD_EMPTY (0xFFFF)

#define GRM_IDX_TO_RECORD(idx) (grm_record_t *)(grm_env.base_addr + GRM_MAGIC_NUMBER_LEN + GRM_RECORD_LEN * (idx))

#define GRM_IS_RECORD_EMPTY(r) ((((r)->status) & GRM_RECORD_EMPTY_MASK) == GRM_RECORD_EMPTY)
#define GRM_IS_RECORD_OK(r) \
    ((((r)->status) & (GRM_RECORD_VALID_MASK | GRM_RECORD_DELETED_MASK)) == (GRM_RECORD_VALID | GRM_RECORD_NOT_DELETED))
#define GRM_SET_RECORD_OK(r)                                                  \
    ((((r)->status) & (~(GRM_RECORD_VALID_MASK | GRM_RECORD_DELETED_MASK))) | \
     (GRM_RECORD_VALID | GRM_RECORD_NOT_DELETED))
#define GRM_IS_RECORD_DELETED(r) ((((r)->status) & GRM_RECORD_DELETED_MASK) == GRM_RECORD_DELETED)
#define GRM_SET_RECORD_DELETED(r) ((((r)->status) & (~GRM_RECORD_DELETED_MASK)) | GRM_RECORD_DELETED)

#define MAX_SEQ_NUM 0xffff

typedef struct
{
    uint16_t seq_num;
    struct glp_meas meas;
    struct glp_meas_ctx meas_ctx;
} grm_record_t;

typedef struct
{
    uint32_t seq_num;
    uint32_t record_num;
} grm_env_t;

extern flash_config_t g_flash_cfg;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void APP_GlpsSendMeasWithoutCtxCmd(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas);

static void APP_GlpsSendMeasWithCtxCmd(uint16_t conhdl,
                                       uint16_t seq_num,
                                       struct glp_meas *meas,
                                       struct glp_meas_ctx *ctx);

static void APP_GlpsSendRspCmd(uint16_t conhdl, uint16_t num_of_record, uint8_t op_code, uint8_t status);

static void APP_GlpsSendMeas(uint16_t conhdl, void *record);

/*******************************************************************************
 * Variables
 ******************************************************************************/
grm_env_t grm_env = {0};
static grm_record_t g_grm_record;
struct app_glps_env_tag app_glps_env[BLE_CONNECTION_MAX];

grm_record_t *p_grm_record_t[GRM_RECORD_MAX] = {NULL};

const NVM_DataEntry_t NVM_DataTable[] = {
    {&p_grm_record_t, GRM_RECORD_MAX, GRM_RECORD_LEN, 2, gNVM_NotMirroredInRamAutoRestore_c},
    /* Required end-of-table marker. */
    {NULL, 0, 0, gNvEndOfTableId_c, 0}};

NVM_DataEntry_t *pNVM_DataTable = (NVM_DataEntry_t *)NVM_DataTable;

/*******************************************************************************
 * Code
 ******************************************************************************/
void GRM_Init(void)
{
    for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
    {
        if (p_grm_record_t[i] != NULL)
        {
            if (p_grm_record_t[i]->seq_num >= grm_env.seq_num)
            {
                grm_env.seq_num = p_grm_record_t[i]->seq_num;
            }
            grm_env.record_num++;
        }
    }
}

static uint16_t GRM_GetNumOfRecords(struct glp_filter *filter)
{
    uint32_t number = 0;
    uint16_t min, max;
    grm_record_t *record;
    /* Use sequence number as an example and user can change this function to support user facing time */
    assert((filter->filter_type == 0x01) || (filter->filter_type == 0x00));

    switch (filter->operator)
    {
        case GLP_OP_ALL_RECS:
            return grm_env.record_num;

        case GLP_OP_FIRST_REC:
        case GLP_OP_LAST_REC:
            return (grm_env.record_num == 0) ? 0 : 1;

        case GLP_OP_LT_OR_EQ:
            min = 0;
            max = filter->val.seq_num.max;
            break;

        case GLP_OP_GT_OR_EQ:
            min = filter->val.seq_num.min;
            max = MAX_SEQ_NUM;
            break;

        case GLP_OP_WITHIN_RANGE_OF:
            min = filter->val.seq_num.min;
            max = filter->val.seq_num.max;
            break;
    }

    for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
    {
        if (p_grm_record_t[i] != NULL)
        {
            record = p_grm_record_t[i];
            if (record->seq_num >= min && record->seq_num <= max)
            {
                if (++number >= grm_env.record_num)
                    break;
            }
        }
    }

    return number;
}

static bool GRM_GetRecord(struct glp_filter *filter, void **record_addr)
{
    grm_record_t *record = NULL;
    uint16_t min = 0;
    uint16_t max = 0;
    ;
    int32_t num = 0;
    uint32_t index = 0;
    bool find_record = false;
    uint32_t min_seq_num = MAX_SEQ_NUM;
    /* No records */
    if (grm_env.record_num == 0)
    {
        return false;
    }

    switch (filter->operator)
    {
        case GLP_OP_ALL_RECS:
            min = 0;
            max = MAX_SEQ_NUM;
            break;
        case GLP_OP_FIRST_REC:
            min = 0;
            max = MAX_SEQ_NUM;
            break;
        case GLP_OP_LT_OR_EQ:
            min = 0;
            max = filter->val.seq_num.max;
            break;
        case GLP_OP_GT_OR_EQ:
            min = filter->val.seq_num.min;
            max = MAX_SEQ_NUM;
            break;
        case GLP_OP_WITHIN_RANGE_OF:
            min = filter->val.seq_num.min;
            max = filter->val.seq_num.max;
            break;
        case GLP_OP_LAST_REC:
            *record_addr = NULL;
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    record = p_grm_record_t[i];
                    if (record->seq_num >= num)
                    {
                        num = record->seq_num;
                        *record_addr = (void *)p_grm_record_t[i];
                    }
                }
            }
            if (*record_addr)
                return true;
            else
                return false;
    }
    /* Find record by sequence number filter */
    for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
    {
        if (p_grm_record_t[i] != NULL)
        {
            record = p_grm_record_t[i];
            if ((record->seq_num >= min) && (record->seq_num <= max))
            {
                if (*record_addr)
                {
                    if ((record->seq_num < min_seq_num) &&
                        (record->seq_num > ((grm_record_t *)(*record_addr))->seq_num))
                    {
                        min_seq_num = record->seq_num;
                        index = i;
                        find_record = true;
                    }
                }
                else
                {
                    if (record->seq_num < min_seq_num)
                    {
                        min_seq_num = record->seq_num;
                        index = i;
                        find_record = true;
                    }
                }
            }
        }
    }
    if (find_record)
    {
        *record_addr = (void *)p_grm_record_t[index];
        return true;
    }
    else
    {
        /* No record is found */
        return false;
    }
}

static inline void GRM_DeleteOneRecord(void **ppData)
{
    NvErase(ppData);
    grm_env.record_num--;
}

static enum glp_racp_status GRM_DeleteRecords(struct glp_filter *filter)
{
    uint8_t delete_flag = 0;
    int32_t num = 0;
    uint32_t index = 0;
    if (grm_env.record_num == 0)
    {
        return (GLP_RSP_NO_RECS_FOUND);
    }

    switch (filter->operator)
    {
        case GLP_OP_ALL_RECS:
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    GRM_DeleteOneRecord((void **)&p_grm_record_t[i]);
                    delete_flag = 1;
                    if (grm_env.record_num == 0)
                        break;
                }
            }
            break;

        case GLP_OP_LT_OR_EQ:
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    if (p_grm_record_t[i]->seq_num <= filter->val.seq_num.max)
                    {
                        GRM_DeleteOneRecord((void **)&p_grm_record_t[i]);
                        delete_flag = 1;
                        if (grm_env.record_num == 0)
                            break;
                    }
                }
            }
            break;

        case GLP_OP_GT_OR_EQ:
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    if (p_grm_record_t[i]->seq_num >= filter->val.seq_num.min)
                    {
                        GRM_DeleteOneRecord((void **)&p_grm_record_t[i]);
                        delete_flag = 1;
                        if (grm_env.record_num == 0)
                            break;
                    }
                }
            }
            break;

        case GLP_OP_WITHIN_RANGE_OF:
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    if (p_grm_record_t[i]->seq_num >= filter->val.seq_num.min &&
                        p_grm_record_t[i]->seq_num <= filter->val.seq_num.max)
                    {
                        GRM_DeleteOneRecord((void **)&p_grm_record_t[i]);
                        delete_flag = 1;
                        if (grm_env.record_num == 0)
                            break;
                    }
                }
            }
            break;

        case GLP_OP_FIRST_REC:
            num = MAX_SEQ_NUM;
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    if (p_grm_record_t[i]->seq_num <= num)
                    {
                        num = p_grm_record_t[i]->seq_num;
                        index = i;
                        delete_flag = 1;
                    }
                }
            }
            if (delete_flag)
            {
                GRM_DeleteOneRecord((void **)&p_grm_record_t[index]);
            }
            break;

        case GLP_OP_LAST_REC:
            num = -1;
            for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
            {
                if (p_grm_record_t[i] != NULL)
                {
                    if (p_grm_record_t[i]->seq_num >= num)
                    {
                        num = p_grm_record_t[i]->seq_num;
                        index = i;
                        delete_flag = 1;
                    }
                }
            }
            if (delete_flag)
            {
                GRM_DeleteOneRecord((void **)&p_grm_record_t[index]);
            }
            break;
    }
    if (delete_flag)
        return (GLP_RSP_SUCCESS);
    else
        return (GLP_RSP_NO_RECS_FOUND);
}

bool GRM_GlpsAddRecord(struct glp_meas meas_add, struct glp_meas_ctx meas_ctx_add)
{
    if (grm_env.record_num == GRM_RECORD_MAX)
    {
        QPRINTF("No place in flash.Please delete some records!\r\n");
        return false;
    }

    for (uint32_t i = 0; i < GRM_RECORD_MAX; i++)
    {
        if (p_grm_record_t[i] == NULL)
        {
            memcpy(&(g_grm_record.meas), &meas_add, sizeof(struct glp_meas));
            memcpy(&(g_grm_record.meas_ctx), &meas_ctx_add, sizeof(struct glp_meas_ctx));
            g_grm_record.seq_num = grm_env.seq_num + 1;
            p_grm_record_t[i] = &g_grm_record;
            NvSaveOnIdle(&p_grm_record_t[i], false);
            grm_env.record_num++;
            grm_env.seq_num++;
            return true;
        }
    }
    return false;
}

void APP_GlpsAddProfileTask(void)
{
    struct glps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct glps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_GLPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct glps_db_cfg *)req->param;
    db_cfg->features = CFG_GLPS_FAETURES;
    db_cfg->meas_ctx_supported = CFG_GLPS_MEA_CTX_SUPPORT;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_GlpsEnableReq(uint16_t conhdl, uint8_t evt_cfg)
{
    struct glps_enable_req *msg =
        KE_MSG_ALLOC(GLPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     glps_enable_req);
    msg->evt_cfg = evt_cfg;
    APP_MsgSend(msg);
}

/*!
 * @brief Inform app that Glucose sensor event configuration configured by peer device
 *
 * @param[in] Glucose sensor event configuration (notification, indication) configured by peer
 * device (Bonded information)
 * - bit 0: Glucose measurement notifications enabled
 * - bit 1: Glucose measurement context notifications enabled
 * - bit 4: Record Access Control Point (RACP) indications enabled
 *
 * @return If the message was consumed or not.
 * @description
 * Event triggered when peer device modify notification/indication configuration of Glucose Sensor role
 * characteristics.
 */
static int APP_GlpsIndntfIndHandler(ke_msg_id_t const msgid,
                                    struct glps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("notification/indication configuration is %x.\r\n", param->evt_cfg);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief Glucose send measurement without context information.
 *
 * @param[in] conhdl Connection handle
 * @param[in] seq_num    Measurement Sequence Number
 * @param[in] meas       Glucose Measurement Structure
 *
 * @response GLPS_CMP_EVT
 * @description
 * This function is used by the application (which handles the Glucose device driver and measurements) to
 * send a glucose measurement without following measurement context information.
 */
static void APP_GlpsSendMeasWithoutCtxCmd(uint16_t conhdl, uint16_t seq_num, struct glp_meas *meas)
{
    struct glps_send_meas_without_ctx_cmd *msg = KE_MSG_ALLOC(
        GLPS_SEND_MEAS_WITHOUT_CTX_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPS), CONHDL2CONIDX(conhdl)),
        TASK_APP, glps_send_meas_without_ctx_cmd);

    msg->seq_num = seq_num;
    msg->meas = *meas;
    APP_MsgSend(msg);
}

/*!
 * @brief Glucose send measurement with context information.
 *
 * @param[in] conhdl     Connection handle
 * @param[in] seq_num    Measurement Sequence Number
 * @param[in] meas       Glucose Measurement Structure
 * @param[in] ctx        Glucose measurement context structure
 *
 * @response GLPS_CMP_EVT
 * @description
 * This function is used by the application (which handles the Glucose device driver and measurements) to
 * send a glucose measurement with following measurement context information.
 */
static void APP_GlpsSendMeasWithCtxCmd(uint16_t conhdl,
                                       uint16_t seq_num,
                                       struct glp_meas *meas,
                                       struct glp_meas_ctx *ctx)
{
    struct glps_send_meas_with_ctx_cmd *msg = KE_MSG_ALLOC(
        GLPS_SEND_MEAS_WITH_CTX_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPS), CONHDL2CONIDX(conhdl)), TASK_APP,
        glps_send_meas_with_ctx_cmd);
    msg->seq_num = seq_num;
    msg->meas = *meas;
    msg->ctx = *ctx;
    APP_MsgSend(msg);
}

static void APP_GlpsSendMeas(uint16_t conhdl, void *record_addr)
{
    grm_record_t *record = (grm_record_t *)(record_addr);
    struct glps_env_tag *glps_env = PRF_ENV_GET(GLPS, glps);

    if (glps_env->env[conhdl]->evt_cfg & GLPS_MEAS_CTX_NTF_CFG)
    {
        APP_GlpsSendMeasWithCtxCmd(conhdl, record->seq_num, &(record->meas), &(record->meas_ctx));
    }
    else
    {
        APP_GlpsSendMeasWithoutCtxCmd(conhdl, record->seq_num, &(record->meas));
    }
}

/*!
 * @brief Glucose send response command.
 *
 * @param[in] conhdl           Connection handle
 * @param[in] num_of_record    Number of records found (Should be set only if RACP operation code equals
 * GLP_REQ_REP_NUM_OF_STRD_RECS)
 * @param[in] op_code          RACP Request operation code
 * @param[in] status           RACP Request operation status code
 *
 * @response GLPS_CMP_EVT
 * @description
 * This function is used by the application to send Record Access Control Point (RACP) request response.
 * If requested operation is GLP_REQ_REP_NUM_OF_STRD_RECS, number of stored record should be set; else it
 * will be ignored by Glucose sensor role. Status code should be set according to Glucose profile error code.
 */
static void APP_GlpsSendRspCmd(uint16_t conhdl, uint16_t num_of_record, uint8_t op_code, uint8_t status)
{
    struct glps_send_racp_rsp_cmd *msg =
        KE_MSG_ALLOC(GLPS_SEND_RACP_RSP_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, glps_send_racp_rsp_cmd);
    msg->num_of_record = num_of_record;
    msg->op_code = op_code;
    msg->status = status;
    APP_MsgSend(msg);
}

/*!
 * @brief Inform app that Record Access Control Point (RACP) request has recived.
 *
 * @param[in] msgid     GLPS_RACP_REQ_RCV_IND
 * @param[in] param     Pointer to the struct glps_racp_req_rcv_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return  If the message was consumed or not.
 * @description
 * This message is trigged by glucose sensor role when peer collector request to perform a Record Access
 * Control Point (RACP) action.
 * This action could be report glucose measurements, report number of measurement, delete measurements
 * or abort an on-going operation. This action contains a filter describing which glucose
 * measurement are concerned by the operation.
 */
static int APP_GlpsRacpReqRcvIndHandler(ke_msg_id_t const msgid,
                                        struct glps_racp_req_rcv_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    uint8_t status = GLP_RSP_SUCCESS;

    QPRINTF("glps racp recieved indication,op_code=0x%x,oprater=0x%x\r\n", param->racp_req.op_code,
            param->racp_req.filter.operator);
    /* TODO: No free message */
    app_glps_env[conhdl].racp_req = param->racp_req;

    switch (param->racp_req.op_code)
    {
        case GLP_REQ_REP_STRD_RECS:
        {
            if (param->racp_req.filter.operator<GLP_OP_ALL_RECS || param->racp_req.filter.operator> GLP_OP_LAST_REC)
            {
                APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, GLP_RSP_INVALID_OPERATOR);
                break;
            }

            app_glps_env[conhdl].cur_record_offset = NULL;
            if (GRM_GetRecord(&app_glps_env[conhdl].racp_req.filter, &(app_glps_env[conhdl].cur_record_offset)))
            {
                APP_GlpsSendMeas(conhdl, app_glps_env[conhdl].cur_record_offset);
            }
            else
            {
                APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, GLP_RSP_NO_RECS_FOUND);
            }
        }
        break;

        case GLP_REQ_REP_NUM_OF_STRD_RECS:
        {
            if (param->racp_req.filter.operator<GLP_OP_ALL_RECS || param->racp_req.filter.operator> GLP_OP_LAST_REC)
            {
                APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, GLP_RSP_INVALID_OPERATOR);
            }
            else
            {
                uint16_t records_num = GRM_GetNumOfRecords(&param->racp_req.filter);
                APP_GlpsSendRspCmd(conhdl, records_num, param->racp_req.op_code, GLP_RSP_SUCCESS);
            }
        }
        break;

        case GLP_REQ_DEL_STRD_RECS:
            status = GRM_DeleteRecords(&param->racp_req.filter);
            APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, status);
            break;
        case GLP_REQ_ABORT_OP:
            APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, GLP_RSP_SUCCESS); /* abort notify */
            break;
        default:
            APP_GlpsSendRspCmd(conhdl, 0, param->racp_req.op_code, GLP_RSP_OP_CODE_NOT_SUP);
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief Glucose compelete event handler.
 *
 * @param[in] msgid     GLPS_CMP_EVT
 * @param[in] param     Pointer to the struct glps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPS
 *
 * @return  If the message was consumed or not.
 * @description
 * Confirmation response sent by Glucose sensor role when a requested action has been performed:
 * GLPS_SEND_MEAS_REQ_NTF_CMP: Glucose measurement notification sent completed
 * GLPS_SEND_RACP_RSP_IND_CMP: Record Access Control Point Response Indication sent completed
 */
static int APP_GlpsCmpEvtHandler(ke_msg_id_t const msgid,
                                 struct glps_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    // uint8_t status;
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    switch (param->request)
    {
        case GLPS_SEND_MEAS_REQ_NTF_CMP:
        {
            QPRINTF("Glucose measurement notification sent completed,request=0x%x,status=0x%x.\r\n", param->request,
                    param->status);

            if (app_glps_env[conhdl]
                    .racp_req.filter.
                    operator== GLP_OP_FIRST_REC || app_glps_env[conhdl]
                    .racp_req.filter.
                    operator== GLP_OP_LAST_REC)
            {
                APP_GlpsSendRspCmd(conhdl, 0, app_glps_env[conhdl].racp_req.op_code, GLP_RSP_SUCCESS);
            }
            else if (GRM_GetRecord(&app_glps_env[conhdl].racp_req.filter, &app_glps_env[conhdl].cur_record_offset))
            {
                APP_GlpsSendMeas(conhdl, app_glps_env[conhdl].cur_record_offset);
            }
            else
            {
                APP_GlpsSendRspCmd(conhdl, 0, app_glps_env[conhdl].racp_req.op_code, GLP_RSP_SUCCESS);
            }
        }
        break;
        case GLPS_SEND_RACP_RSP_IND_CMP:
            QPRINTF(
                "Glucose Record Access Control Point Response Indication sent completed,request=0x%x,status=0x%x.\r\n",
                param->request, param->status);
            break;
        default:
            break;
    }
    return (KE_MSG_CONSUMED);
}

const struct ke_msg_handler s_appGlpsMsgHandlerList[] = {
    {GLPS_CFG_INDNTF_IND, (ke_msg_func_t)APP_GlpsIndntfIndHandler},
    {GLPS_RACP_REQ_RCV_IND, (ke_msg_func_t)APP_GlpsRacpReqRcvIndHandler},
    {GLPS_CMP_EVT, (ke_msg_func_t)APP_GlpsCmpEvtHandler},
};

const struct ke_state_handler g_AppGlpsTableHandler = KE_STATE_HANDLER(s_appGlpsMsgHandlerList);

#endif
