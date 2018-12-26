/*!
* Copyright (c) 2018, NXP, Inc.
* All rights reserved.
*
* \file shell_peripherals.c
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
* o Neither the name of NXP, Inc. nor the names of its
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

#include <stdlib.h>
#include <stdio.h>

#include "rgb_led.h"
#include "shell_peripherals.h"
#include "peripherals.h"
#include "shell.h"
#include "sensors.h"
#include "app_config.h"
#include "board.h"

#if SHELL_DEBUGGER

static int8_t GetSensorData_Command(uint8_t argc, char *argv[]);
static int8_t SetSensorData_Command(uint8_t argc, char *argv[]);

const char mpGetSensorHelp[] = "\r\n"
                               "get temperature\r\n"
                               "get humidity\r\n"
                               "get pressure\r\n"
                               "get airQuality\r\n"
                               "get ambientLight\r\n"
                               "get battery\r\n"
                               "get gyroscope\r\n"
                               "get accelerometer\r\n"
                               "get magnetometer\r\n"
                               "get motiondetect\r\n"
                               "get freefalldetect\r\n"
                               "get tapcount\r\n";
const char mpSetSensorHelp[] = "\r\n"
                               "set led intensity color \r\n"
                               "set buzzer 0/1\r\n"
                               "set backlight level\r\n";

const cmd_tbl_t mSetCmd =
{
    .name = "set",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = SetSensorData_Command,
    .usage = (char *)mpSetSensorHelp,
    .help = "Contains commands for setting sensor. \r\n"
};

const cmd_tbl_t mGetCmd =
{
    .name = "get",
    .maxargs = 2,
    .repeatable = 1,
    .cmd = GetSensorData_Command,
    .usage = (char *)mpGetSensorHelp,
    .help = "Contains commands for getting sensor data."
};

void Cmd_Init(void)
{
    shell_register_function((cmd_tbl_t *)&mGetCmd);
    shell_register_function((cmd_tbl_t *)&mSetCmd);
}

static int8_t SetSensorData_Command(uint8_t argc, char *argv[])
{
    uint8_t status;

    if (argc < 1)
    {
        return CMD_RET_USAGE;
    }

    if (!strcmp((char *)argv[1], "led"))
    {
        if (argc == 4)
        {
            status = RGB_Led_Set_State(atoi(argv[2]), atoi(argv[3]));
            return (status == 0) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
        }
        else
            return CMD_RET_USAGE;
    }
    else if (!strcmp((char *)argv[1], "buzzer"))
    {
        if (argc == 3)
        {
            if (atoi(argv[2]) == 1)
            {
                BUZZER_ON;
                return CMD_RET_SUCCESS;
            }
            else if (atoi(argv[2]) == 0)
            {
                BUZZER_OFF;
                return CMD_RET_SUCCESS;
            }
            else
                return CMD_RET_USAGE;
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else if (!strcmp((char *)argv[1], "backlight"))
    {
        if (argc == 3)
        {
            status = Backlight_SetLevel(atoi(argv[2]));
            return (status == 0) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
        }
        else
            return CMD_RET_USAGE;
    }
    return CMD_RET_USAGE;
}

static int8_t GetSensorData_Command(uint8_t argc, char *argv[])
{
    if (argc < 2)
    {
        return CMD_RET_USAGE;
    }

    char buff[50]                 = {0};

    if (!strcmp((char *)argv[1], "temperature"))
    {
        float TemperatureValue = 0;
        uint8_t TempSize       = 4;

        if (get_temperature((uint8_t *)&TemperatureValue, &TempSize) == 0)
        {
            sprintf(buff, "%.2f", TemperatureValue);
            shell_write(buff);
        }
        else
            shell_write("\r\n--> Can not read Temperature value.");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "humidity"))
    {
        float HumidityValue  = 0;
        uint8_t HumiditySize = 4;

        if (get_humidity((uint8_t *)&HumidityValue, &HumiditySize) == 0)
        {
            sprintf(buff, "%.2f", HumidityValue);
            shell_write(buff);
        }
        else
            shell_write("\r\n--> Can not read Humidity value. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "pressure"))
    {
        int32_t PressureValue = 0;
        uint8_t PressureSize  = 4;

        if (get_pressure((uint8_t *)&PressureValue, &PressureSize) == 0)
        {
            sprintf(buff, "%d", PressureValue);
            shell_write(buff);
        }
        else
            shell_write("\r\n--> Can not read Pressure value. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "airQuality"))
    {
        uint16_t AirQualityValue = 0;
        uint8_t AirQualitySize   = 2;

        if (get_air_quality((uint8_t *)&AirQualityValue, &AirQualitySize) ==  0)
        {
            sprintf(buff, "%d", AirQualityValue);
            shell_write(buff);
        }
        else
            shell_write("\r\n--> Can not read Air Quality value. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "ambientLight"))
    {
        float AmbientLightValue  = 0;
        uint8_t AmbientLightSize = 4;

        if (get_ambient_light((uint8_t *)&AmbientLightValue, &AmbientLightSize) == 0)
        {
            sprintf(buff, "%.2f", AmbientLightValue);
            shell_write(buff);
        }
        else
            shell_write("\r\n-->  Notify Event: Can not read Ambient Light value. ");

        return CMD_RET_SUCCESS;
    }


    else if (!strcmp((char *)argv[1], "battery"))
    {
        uint8_t batPercentLevel;
        uint8_t batChargingState;

        if (BatterySensor_GetState(&batPercentLevel, &batChargingState) == 0)
        {
            sprintf(buff, "%d,%d", batPercentLevel, batChargingState);
            shell_write(buff);
        }
        else
            shell_write("\r\n-->  Notify Event: Can not read battery sensor values. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "gyroscope"))
    {
        int16_t GyroscopeValue[3] = {0};
        uint8_t GyroscopeSize     = 6;

        if (get_rotation_speed((uint8_t *)GyroscopeValue, &GyroscopeSize) == 0)
        {
            sprintf(buff, "\"X\":\"%d\",\"Y\":\"%d\",\"Z\":\"%d\"", GyroscopeValue[0], GyroscopeValue[1], GyroscopeValue[2]);
            shell_write(buff);
        }
        else
            shell_write("\r\n-->  Notify Event: Can not read Gyroscope value. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "accelerometer"))
    {
        float AccelerometerValue[3] = {0};
        uint8_t AccelerometerSize     = 12;

        if (get_acceleration((uint8_t *)AccelerometerValue, &AccelerometerSize) == 0)
        {
            sprintf(buff, "\"X\":\"%.6f\",\"Y\":\"%.6f\",\"Z\":\"%.6f\"", AccelerometerValue[0], AccelerometerValue[1], AccelerometerValue[2]);
            shell_write(buff);
        }
        else
            shell_write("\r\n-->  Notify Event: Can not read Accelerometer value. ");

        return CMD_RET_SUCCESS;
    }

    else if (!strcmp((char *)argv[1], "magnetometer"))
    {
        float MagnetometerValue[3] = {0};
        uint8_t MagnetometerSize     = 12;

        if (get_magnetic_field((uint8_t *)MagnetometerValue, &MagnetometerSize) == 0)
        {
            sprintf(buff, "\"X\":\"%.1f\",\"Y\":\"%.1f\",\"Z\":\"%.1f\"", MagnetometerValue[0], MagnetometerValue[1], MagnetometerValue[2]);
            shell_write(buff);
        }
        else
            shell_write("\r\n-->  Notify Event: Can not read Magnetometer value. ");

        return CMD_RET_SUCCESS;
    }
    else if (!strcmp((char *)argv[1], "motiondetect"))
    {

    	if (Init_MotionDetect()!=kStatus_Success)
    	{
    		shell_write("\r\n Motion Detection Init Failed");
    		return kStatus_Fail;
    	}

		shell_write("\r\n -------------Motion Detection Program Started------------- \n");
		shell_write("\r\n Move device along x,y or z axis with accel > 0.25g to trigger.");
		shell_write("\r\n Trigger a motion event to exit program. \n");
		shell_write("\r\n ");

        while (1)
        {
        	if (motion_detected())
        	{
        		shell_write("\r\n-->  Motion Detected ");
        		uint8_t DeInit_MotionDetect(void);
        		shell_write("\r\n\n Exited Motion Detection Program \n ");
        		return CMD_RET_SUCCESS;
        	}
            App_WaitMsec(100);
        }
    }
    else if (!strcmp((char *)argv[1], "freefalldetect"))
    {
    	if (Init_FreefallDetect()!=kStatus_Success)
    	{
    		shell_write("\r\n Freefall Detection Init Failed");
    		return kStatus_Fail;
    	}

		shell_write("\r\n -------------Freefall Detection Program Started------------- \n");
		shell_write("\r\n Drop device in any axis with accel < 0.25g to trigger.");
		shell_write("\r\n Catch device with hand underneath.");
		shell_write("\r\n Trigger a freefall event to exit program. \n");
		shell_write("\r\n ");

        while (1)
        {
        	if (freefall_detected())
        	{
        		shell_write("\r\n-->  Freefall Detected ");
        		uint8_t DeInit_FreefallDetect(void);
        		shell_write("\r\n\n Exited Freefall Detection Program \n");
        		return CMD_RET_SUCCESS;
        	}
            App_WaitMsec(100);
        }
    }
    else if (!strcmp((char *)argv[1], "tapcount"))
    {
    	uint16_t tapCount=0;
    	bool dtap;

    	if (Init_TapDetect()!=kStatus_Success)
    	{
    		shell_write("\r\n Tap Detection Init Failed");
    		return kStatus_Fail;
    	}

		shell_write("\r\n -------------Tap Count Program Started------------- \n");
		shell_write("\r\n Tap device till count = 8 to exit the program. \r\n");
		shell_write("\r\n ");

        while (1)
        {
        	if (tap_detected(&dtap))
        	{
        		tapCount++;
        		if (dtap)
        		{
            		tapCount++;
        		}
        		shell_write("\r\n-->  Tap Detected ");
                sprintf(buff, "Tap Count: %d", tapCount);
                shell_write(buff);
        	}

        	if (tapCount>7)
        	{
        		shell_write("\r\n\n Exited Tap Count Program \n");
        		return CMD_RET_SUCCESS;
        	}
            App_WaitMsec(20);
        }
    }
    return CMD_RET_USAGE;
}
#endif
