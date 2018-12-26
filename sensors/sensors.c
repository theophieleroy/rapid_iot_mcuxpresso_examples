#include "board.h"
#include "sensors.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "FunctionLib.h"
#include "shell.h"

/* CMSIS Includes */
#include "Driver_I2C.h"
#include "fsl_i2c_cmsis.h"
#define I2C_S1_DRIVER Driver_I2C1
#define I2C_S1_SIGNAL_EVENT I2C1_SignalEvent_t
#define I2C_S1_DEVICE_INDEX I2C1_INDEX

/* I2C1 variables */
i2c_master_handle_t g_mi2c_handle;
volatile bool completionFlag = false;
volatile bool nakFlag = false;

/* I2C2 variables */
i2c_master_handle_t g_mi2c2_handle;
volatile bool completionFlag2 = false;
volatile bool nakFlag2 = false;

/* Touchpad */
sx9500_fct_t FCT_SX9500;

/* Sensors */
mpl3115_IoFunc_t MPL3115_sensor;
CCS811_fct_t CCS811_sensor;
ens210_IoFunc_t ENS210_sensor;
tsl2572_IoFunc_t TSL2572_sensor;
backlight_fct_t BACKLIGHT_fct;

/* Authentication */
a1006_IoFunc_t A1006_auth;

fxos8700_handle_t g_fxosHandle;
fxos8700_data_t fxos8700_data;
fxas21002_i2c_sensorhandle_t FXAS21002drv;

float g_dataScale = 0;

/*******************************************************************************
 * Constants
 ******************************************************************************/
/* Prepare the register write list to configure FXAS21002 in non-FIFO mode. */
const registerwritelist_t fxas21002_Config_Normal[] = {
    /*! Configure CTRL_REG1 register to put FXAS21002 to 12.5Hz sampling rate. */
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_DR_12_5HZ, FXAS21002_CTRL_REG1_DR_MASK},
    __END_WRITE_DATA__};

/* Prepare the register read list to read FXAS21002 DataReady status. */
const registerreadlist_t fxas21002_DRDY[] = {{.readFrom = FXAS21002_STATUS, .numBytes = 1}, __END_READ_DATA__};

/* Prepare the register read list to read the raw gyro data from the FXAS21002. */
const registerreadlist_t fxas21002_Output_Values[] = {
    {.readFrom = FXAS21002_OUT_X_MSB, .numBytes = FXAS21002_GYRO_DATA_SIZE}, __END_READ_DATA__};

/*****************************************************/
/* I2C1 callback                                     */
/*****************************************************/
extern void i2c1_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success) {
        completionFlag = true;
    }
    /* Signal transfer failure when received any other status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak) ||
        (status == kStatus_I2C_ArbitrationLost) || (status == kStatus_I2C_Timeout)) {
        nakFlag = true;
    }
}

/*****************************************************/
/* I2C1_init function                                */
/*****************************************************/
uint8_t I2C1_init(void){
    int32_t status;
    ARM_DRIVER_I2C *I2Cdrv = &I2C_S1_DRIVER; // or I2C_S_SIGNAL_EVENT?
    status = I2Cdrv->Initialize(I2C_S1_SIGNAL_EVENT);

    /* Initialize the I2C driver */
    if (ARM_DRIVER_OK != status) {
        shell_printf("Error while initializing I2C1!\r\n");
        return 1;
    }
    /* Set the I2C Power mode */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status) {
        shell_printf("Error while setting I2C1 power mode!\r\n");
        return 1;
    }
    /* Set the I2C bus speed at 400kHz */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status) {
        shell_printf("Error while setting I2C1 control mode!\r\n");
        return 1;
    }

    /* As the fsl_i2c_cmsis and fsl_i2c are used simultaneously: */
    /* - initialize I2C1 only through fsl_i2c_cmsis */
    /* - register the fsl_i2c callback before each call to I2C_MasterTransferNonBlocking */
    /*   hence I2C_MasterTransferCreateHandle is added in App_I2C1_Read and App_I2C1_Write */
    return 0;
}

/*****************************************************/
/* Low level I2C1_Write function                     */
/*****************************************************/
uint8_t App_I2C1_Write(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize) {
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = writeBuf;
    masterXfer.dataSize = writeSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferCreateHandle(BOARD_SENSORS_I2C_BASEADDR, &g_mi2c_handle, i2c1_master_callback, NULL);

    if (kStatus_Success != I2C_MasterTransferNonBlocking(BOARD_SENSORS_I2C_BASEADDR, &g_mi2c_handle, &masterXfer)) {
        return I2C_RESULT_FAIL;
    }

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag)) {
    }

    nakFlag = false;

    if (completionFlag == true) {
        completionFlag = false;
        return I2C_RESULT_OK;
    }
    else {
        return I2C_RESULT_FAIL;
    }
}

/*****************************************************/
/* Low level I2C1_Read function                      */
/*****************************************************/
uint8_t App_I2C1_Read(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize) {
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = *writeBuf;
    masterXfer.subaddressSize = writeSize;
    masterXfer.data = readBuf;
    masterXfer.dataSize = readSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferCreateHandle(BOARD_SENSORS_I2C_BASEADDR, &g_mi2c_handle, i2c1_master_callback, NULL);

    I2C_MasterTransferNonBlocking(BOARD_SENSORS_I2C_BASEADDR, &g_mi2c_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag)) {
    }

    nakFlag = false;

    if (completionFlag == true) {
        completionFlag = false;
        return I2C_RESULT_OK;
    }
    else {
        return I2C_RESULT_FAIL;
    }
}

/*****************************************************/
/* I2C2 callback                                     */
/*****************************************************/
extern void i2c2_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success) {
        completionFlag2 = true;
    }
    /* Signal transfer failure when received any other status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak) ||
        (status == kStatus_I2C_ArbitrationLost) || (status == kStatus_I2C_Timeout)) {
        nakFlag2 = true;
    }
}


/*****************************************************/
/* I2C2_init function                                */
/*****************************************************/
uint8_t I2C2_init(void){
    i2c_master_config_t masterConfig;

    I2C_MasterTransferCreateHandle(BOARD_SECURITY_I2C_BASEADDR, &g_mi2c2_handle, i2c2_master_callback, NULL);
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = SECURITY_I2C_BAUDRATE;
    I2C_MasterInit(BOARD_SECURITY_I2C_BASEADDR, &masterConfig, SECURITY_I2C_CLK_FREQ);
    return 0;
}

/*****************************************************/
/* Low level I2C2_Write function                     */
/*****************************************************/
uint8_t App_I2C2_Write(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize) {
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = writeBuf;
    masterXfer.dataSize = writeSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if (kStatus_Success != I2C_MasterTransferNonBlocking(BOARD_SECURITY_I2C_BASEADDR, &g_mi2c2_handle, &masterXfer)) {
        return I2C_RESULT_FAIL;
    }

    /*  wait for transfer completed. */
    while ((!nakFlag2) && (!completionFlag2)) {
    }

    nakFlag2 = false;

    if (completionFlag2 == true) {
        completionFlag2 = false;
        return I2C_RESULT_OK;
    }
    else {
        return I2C_RESULT_FAIL;
    }
}

/*****************************************************/
/* Low level I2C2_Read function                      */
/*****************************************************/
uint8_t App_I2C2_Read(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize) {
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;

    // Convert 8-bit unsigned integer buffer to 32-bit unsigned integer
    if (sizeof(uint32_t) >= writeSize)
    {
        masterXfer.subaddress = 0;
        uint32_t cmdByte = writeSize;
        for (uint8_t idx = 0; idx < writeSize; idx++)
        {
            uint32_t shift = --cmdByte * 8;
            masterXfer.subaddress |= writeBuf[idx] << shift;
        }
    }
    else
    {
        // Invalid writesize, return FAIL
        return I2C_RESULT_FAIL;
    }

    masterXfer.subaddressSize = writeSize;
    masterXfer.data = readBuf;
    masterXfer.dataSize = readSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(BOARD_SECURITY_I2C_BASEADDR, &g_mi2c2_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag2) && (!completionFlag2)) {
    }

    nakFlag2 = false;

    if (completionFlag2 == true)
    {
        completionFlag2 = false;
        return I2C_RESULT_OK;
    }
    else {
        return I2C_RESULT_FAIL;
    }
}

uint8_t get_ambient_light(uint8_t *buf, uint8_t *size)
{
    float fAmbLight;

    if(buf != NULL && size != NULL && *size > 3)
    {
        if (TSL2572_ReadAmbientLight(&fAmbLight) == 0)
        {
            FLib_MemCpy(buf, &fAmbLight, 4);
            *size = 4;

            if (TSL2572_ClearALSInterrupt() != 0)
            {
                return 1;
            }
            return 0;
        }
        else {
            return 1;
        }
    }

    return 1;
}

uint8_t get_air_quality(uint8_t *buf, uint8_t *size)
{
    uint8_t avail;
    static uint16_t uCO2;

    if (buf == NULL || size == NULL || *size <= 1) return 1;

    if (CCS811_dataAvailable(&avail) != CCS811_SUCCESS) return 1;
    if (avail == 1)
    {
        /* data is available */
        if (CCS811_readAlgorithmResults() != CCS811_SUCCESS) return 1;
        uCO2 = CCS811_getCO2(); /* store new static data */
    }
    FLib_MemCpy(buf, &uCO2, 2);
    *size = 2;

    return 0;
}

uint8_t get_pressure(uint8_t *buf, uint8_t *size)
{
    int32_t data;

    if(buf != NULL && size != NULL && *size > 3)
    {
        if (MPL_ReadRawData (MPL_MODE_PRESSURE, &data) == 0) {
            data /= 400; // in HPa (LSB = 0.25Pa)

            FLib_MemCpy(buf, &data, 4);
            *size = 4;

            return 0;
        }
        else {
            return 1;
        }
    }

    return 1;
}

uint8_t get_temperature(uint8_t *buf, uint8_t *size)
{
    ens210_meas_data_t ens210_data;
    uint8_t TempOffsetCelcius = 7;

    if(buf != NULL && size != NULL && *size > 3)
    {
        if(ENS210_Measure(mode_Tonly, &ens210_data) == 0)
        {
            float temp = ens210_data.T_mCelsius / 1000.0;
            temp -= TempOffsetCelcius;
            FLib_MemCpy(buf, &temp, 4);
            *size = 4;

            return 0;
        }
        else {
            return 1;
        }
    }

    return 1;
}

uint8_t get_humidity(uint8_t *buf, uint8_t *size)
{
    ens210_meas_data_t ens210_data;

    if(buf != NULL && size != NULL && *size > 3)
    {
        if(ENS210_Measure(mode_Honly, &ens210_data) == 0)
        {
            float humid = ens210_data.H_mPercent / 1000.0;
            FLib_MemCpy(buf, &humid, 4);
            *size = 4;

            return 0;
        }
        else {
            return 1;
        }
    }

    return 1;
}

uint8_t get_acceleration(uint8_t *buf, uint8_t *size)
{
    if(buf != NULL && size != NULL && *size > 11)
    {
        if (FXOS8700_ReadSensorData(&g_fxosHandle, &fxos8700_data) == kStatus_Success)
        {
            float accel[3]= {0};

            /*converting to units of  g*/
            accel[0] =  ((float)((int16_t)(((fxos8700_data.accelXMSB*256) + (fxos8700_data.accelXLSB)))>> 2));
            accel[1] =  ((float)((int16_t)(((fxos8700_data.accelYMSB*256) + (fxos8700_data.accelYLSB)))>> 2));
            accel[2] =  ((float)((int16_t)(((fxos8700_data.accelZMSB*256) + (fxos8700_data.accelZLSB)))>> 2));

            accel[0] *= g_dataScale;
            accel[1] *= g_dataScale;
            accel[2] *= g_dataScale;
            FLib_MemCpy(buf, accel, 12);

            *size = 12;
            return kStatus_Success;

        }
        else
        {
            return kStatus_Fail;
        }

        return 0;
    }

    return 1;
}

uint8_t get_magnetic_field(uint8_t *buf, uint8_t *size)
{
    if(buf != NULL && size != NULL && *size > 11)
    {
        if (FXOS8700_ReadSensorData(&g_fxosHandle, &fxos8700_data) == kStatus_Success)
        {
            float mag[3]={0, 0, 0};

            /*Converting to units of uT */
            mag[0] =  (float)((int16_t)((fxos8700_data.magXMSB*256) + (fxos8700_data.magXLSB))) * 0.1;
            mag[1] =  (float)((int16_t)((fxos8700_data.magYMSB*256) + (fxos8700_data.magYLSB))) * 0.1;
            mag[2] =  (float)((int16_t)((fxos8700_data.magZMSB*256) + (fxos8700_data.magZLSB))) * 0.1;

            FLib_MemCpy(buf, mag, 12);
            *size = 12;

            return kStatus_Success;
        }
        else
        {
            return kStatus_Fail;
        }
    }
    return 1;
}

uint8_t get_rotation_speed(uint8_t *buf, uint8_t *size)
{
    int32_t status;
    uint8_t data[FXAS21002_GYRO_DATA_SIZE];

    if(buf != NULL && size != NULL && *size > 5)
    {
        int16_t rotspeed[3] = {10, 15, -2000};
        /* Read the raw sensor data from the FXAS21002 */
        status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_Output_Values, data);
        if (ARM_DRIVER_OK != status) return 1;

        /* Convert the raw sensor data to signed 16-bit container */
        rotspeed[0] = ((int16_t)data[0] << 8) | data[1];
        rotspeed[1] = ((int16_t)data[2] << 8) | data[3];
        rotspeed[2] = ((int16_t)data[4] << 8) | data[5];

        FLib_MemCpy(buf, rotspeed, 6);
        *size = 6;

        return 0;
    }

    return 1;
}

bool motion_detected(void)
{
    /*This motion detected is using the FXOS8700CQ embedded transient detection function which is similar to motion detection function
     * but filters out static acceleration such as gravity*/

    uint8_t tmp[1];

    if(FXOS8700_ReadReg(&g_fxosHandle, FXOS8700_TRANSIENT_SRC_REG, tmp, 1) != kStatus_Success)
    {
        return 0;
    }

    if ((tmp[0] & FXOS8700_TEA_MASK) == FXOS8700_TEA_MASK)
    {
        return 1;
    }

    return 0;
}

bool freefall_detected(void)
{

    uint8_t tmp[1];

    if(FXOS8700_ReadReg(&g_fxosHandle, FXOS8700_FF_MT_SRC_REG, tmp, 1) != kStatus_Success)
    {
        return 0;
    }

    if ((tmp[0] & FXOS8700_EA_MASK) == FXOS8700_EA_MASK)
    {
        return 1;
    }

    return 0;
}

bool tap_detected(bool *doubletap)
{

    uint8_t tmp[1];
    *doubletap = 0;

    if(FXOS8700_ReadReg(&g_fxosHandle, FXOS8700_PULSE_SRC_REG, tmp, 1) != kStatus_Success)
    {
        return 0;
    }


    if ((tmp[0] & FXOS8700_PEA_MASK) == FXOS8700_PEA_MASK)
    {
        /* Check to see if it was a double tap */
        if ((tmp[0] & FXOS8700_DPE_MASK) == FXOS8700_DPE_MASK)
        {
            *doubletap = 1;
        }

        return 1;
    }

    return 0;
}

uint8_t Init_MotionDetect(void)
{
    /* This algorithm only detects motion from dynamic acceleration events. Static acceleration such as gravity is filtered out using a high-pass filter.
     * Use function FXOS8700_MotionDetect_Init() for motion detection that accounts for both static and dynamic acceleration.
     * Naming is a bit confusing but using embedded transient function makes more sense since the way our application wants motion detection to occur is more in
     * line with using the FXOS8700CQ embedded Transient Detect than using the Motion Detect.
     */
    if (FXOS8700_TransientDetect_Init(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

uint8_t Init_FreefallDetect(void)
{
     /* The freefall detection algorithm assumes z-axis is under influence of gravity. If device is mounted in an orientation where the z-axis is not under influence
     *  of gravity this will not work correctly. To correct, set the flag for freefall detect for the axis under the influence of gravity in FXOS8700_FreefallDetect_Init()
     */
    if (FXOS8700_FreefallDetect_Init(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}


uint8_t Init_TapDetect(void)
{
    if (FXOS8700_TapDetect_Init(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

uint8_t DeInit_MotionDetect(void)
{
    if (FXOS8700_TransientDetect_DeInit(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

uint8_t DeInit_FreefallDetect(void)
{
    if (FXOS8700_FreefallMotion_DeInit(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

uint8_t DeInit_TapDetect(void)
{
    if (FXOS8700_TapDetect_DeInit(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/* Authentication */
uint8_t get_auth_uid(uint8_t *buf)
{
    uint8_t ret = 1;

    if (NULL != buf)
    {
        ret = A1006_Get_Uid(buf);
    }

    return ret;
}

uint8_t get_auth_cert(uint8_t *buf)
{
    uint8_t ret = 1;

    if (NULL != buf)
    {
        ret = A1006_Get_Cert(buf);
    }

    return ret;
}

uint8_t set_auth_challenge(uint8_t *buf)
{
    uint8_t ret = 1;

    if (NULL != buf)
    {
        ret = A1006_Set_Challenge(buf);
    }

    return ret;
}

uint8_t get_auth_response(uint8_t *buf)
{
    uint8_t ret = 1;

    if (NULL != buf)
    {
        ret = A1006_Get_Response(buf);
    }

    return ret;
}

uint8_t set_auth_softreset(void)
{
    uint8_t ret = 1;

    ret = A1006_Soft_Reset();

    return ret;
}

uint8_t Init_touchpad(void)
{
    /* Init touchpad */
    FCT_SX9500.connect_hw = Touch_Controller_Connect;
    FCT_SX9500.disconnect_hw = Touch_Controller_Disconnect;
    FCT_SX9500.I2C_Write = App_I2C1_Write;
    FCT_SX9500.I2C_Read = App_I2C1_Read;
    FCT_SX9500.WaitMs = App_WaitMsec;

    SX9500_Init_Driver(&FCT_SX9500);

    if (SX9500_Init_Hw() != SX9500_SUCCESS)
    {
        return 1;
    }
    return 0;
}

uint8_t DeInit_touchpad(void)
{
    SX9500_Deinit_Driver();
    return 0;
}

uint8_t Init_ambient_light(void)
{
    /* Init ambient light driver */
    TSL2572_sensor.I2C_Read = App_I2C1_Read;
    TSL2572_sensor.I2C_Write = App_I2C1_Write;
    TSL2572_sensor.WaitMsec = App_WaitMsec;

    TSL2572_Init_Driver(&TSL2572_sensor);

    if(TSL2572_Init_Hw() != 0) {
        return 1;
    }
    return 0;
}

uint8_t DeInit_ambient_light(void)
{
    TSL2572_Deinit_Driver();
    return 0;
}

uint8_t Init_pressure(void)
{
    /* Init pressure sensors */
    MPL3115_sensor.I2C_Read = App_I2C1_Read;
    MPL3115_sensor.I2C_Write = App_I2C1_Write;
    MPL3115_sensor.WaitMsec = App_WaitMsec;

    MPL3115_Init_Driver(&MPL3115_sensor);

    return MPL3115_Init_Hw();
}

uint8_t DeInit_pressure(void)
{
    MPL3115_Deinit_Driver();
    return 0;
}

uint8_t Init_air_quality(void)
{
    /* Init air quality sensor */
    CCS811_sensor.connect_hw = CCS811_Connect;
    CCS811_sensor.disconnect_hw = CCS811_Disconnect;
    CCS811_sensor.I2C_Write = App_I2C1_Write;
    CCS811_sensor.I2C_Read = App_I2C1_Read;
    CCS811_sensor.WaitMsec = App_WaitMsec;

    CCS811_Init_Driver(&CCS811_sensor);

    if (CCS811_Init_Hw() != CCS811_SUCCESS)
    {
        return 1;
    }
    return 0;
}

uint8_t DeInit_air_quality(void)
{
    CCS811_Deinit_Driver();
    return 0;
}

uint8_t Init_air_humidity_temp(void)
{
    ENS210_Ids_t ID;

    ENS210_sensor.I2C_Read = App_I2C1_Read;
    ENS210_sensor.I2C_Write = App_I2C1_Write;
    ENS210_sensor.WaitMsec = App_WaitMsec;

    ENS210_Init_Driver(&ENS210_sensor);

    /* HW init of the sensor */
    if(ENS210_Init_Hw() == 0)
    {
        if (ENS210_Ids_Get(&ID) != 0)
        {
            return 1;
        }
        return 0;
    }
    return 1;
}

uint8_t DeInit_air_humidity_temp(void)
{
    ENS210_Deinit_Driver();
    return 0;
}

uint8_t Init_battery_sensor(void)
{
    battery_fct_t bat_sensor;

    bat_sensor.WaitMsec = App_WaitMsec;

    BatterySensor_Init_Driver(&bat_sensor);

    if (BatterySensor_Init_Hw())
        return 1;

    return 0;
}

uint8_t DeInit_battery_sensor(void)
{
    BatterySensor_Deinit_Driver();
    return 0;
}

uint8_t Init_accel_mag(void)
{
    g_fxosHandle.xfer.slaveAddress = FXOS8700_I2C_SLAVE_ADDRESS;
    g_fxosHandle.base = I2C1;
    g_fxosHandle.i2cHandle = &g_mi2c_handle;

    uint8_t g_sensorRange = 0;

    /* Init sensor */
    if (FXOS8700_Init(&g_fxosHandle) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* Get sensor range */
    if (FXOS8700_ReadReg(&g_fxosHandle, FXOS8700_XYZ_DATA_CFG_REG, &g_sensorRange, 1) != kStatus_Success)
    {
        return -1;
    }
    if(g_sensorRange == 0x00)
    {
        /* 0.244 mg/LSB */
        g_dataScale = 0.000244;
    }
    else if(g_sensorRange == 0x01)
    {
        /* 0.488 mg/LSB */
        g_dataScale = 0.000488;
    }
    else if(g_sensorRange == 0x10)
    {
        /* 0.976 mg/LSB */
        g_dataScale = 0.000976;
    }
    else
    {
    }
    return 0;
}

uint8_t DeInit_accel_mag(void)
{
    return 0;
}

uint8_t Init_rotation_speed(void)
{
    int32_t status;
    /* Initialize the FXAS21002 sensor driver */
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S1_DRIVER, I2C_S1_DEVICE_INDEX, FXAS21002_I2C_SLAVE_ADDRESS,
            FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status) return 1;
    /* Configure the FXAS21002 sensor driver */
    status = FXAS21002_I2C_Configure(&FXAS21002drv, fxas21002_Config_Normal);
    if (SENSOR_ERROR_NONE != status) return 1;

    return 0;
}

uint8_t Init_backlight(void)
{
    BACKLIGHT_fct.connect_hw = Backlight_Connect;
    BACKLIGHT_fct.disconnect_hw = Backlight_Disconnect;
    BACKLIGHT_fct.set_level = Backlight_Set_Level;

    Backlight_Init_Driver(&BACKLIGHT_fct);

    if (Backlight_Init_Hw())
        return 1;

    return 0;
}

uint8_t DeInit_rotation_speed(void)
{
    FXAS21002_I2C_Deinit(&FXAS21002drv);
    return 0;
}

/* Authentication */
uint8_t Init_authentication(void)
{
    /* Init authentication interface */
    A1006_auth.I2C_Read = App_I2C2_Read;
    A1006_auth.I2C_Write = App_I2C2_Write;
    A1006_auth.WaitMsec = App_WaitMsec;
    A1006_auth.i2c_Address = A1006_I2C_SLAVE_ADDRESS;
    A1006_auth.cert_slot = kCertificateOne;

    uint8_t ret = A1006_Init_Driver(&A1006_auth);

    return ret;
}

uint8_t Init_all_sensors(void)
{
    uint8_t ret = 0;

    /* Initialize I2C1 and I2C2 */
    I2C1_init();
    I2C2_init();

    if (Init_ambient_light())
    {
        shell_printf("Error while initializing ambient light sensor!\r\n");
        ret = 1;
    }
    if (Init_touchpad())
    {
        shell_printf("Error while initializing touchpad!\r\n");
        ret = 1;
    }
    if (Init_pressure())
    {
        shell_printf("Error while initializing pressure sensor!\r\n");
        ret = 1;
    }
    if (Init_air_quality())
    {
        shell_printf("Error while initializing air quality sensor!\r\n");
        ret = 1;
    }
    if (Init_air_humidity_temp())
    {
        shell_printf("Error while initializing humidity and temperature sensor!\r\n");
        ret = 1;
    }
    if (Init_battery_sensor())
    {
        shell_printf("Error while initializing battery sensor!\r\n");
        ret = 1;
    }
    if (Init_accel_mag())
    {
        shell_printf("Error while initializing Accel & Mag!\r\n");
        ret = 1;
    }
    if (Init_rotation_speed())
    {
        shell_printf("Error while initializing gyroscope!\r\n");
        ret = 1;
    }
    if (Init_backlight())
    {
        shell_printf("Error while initializing backlight!\r\n");
        ret = 1;
    }

#ifndef DISABLE_NTAG
    Connect_NTAG_A1006();           // Enable NTAG & A1006
    if (Init_authentication())
    {
        shell_printf("Error while initializing authentication device!\r\n");
        ret = 1;
    }
#endif
    return ret;
}
