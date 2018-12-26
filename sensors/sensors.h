#ifndef SENSORS_H_
#define SENSORS_H_

#include "pcf2123.h"
#include "mpl3115.h"
#include "CCS811.h"
#include "sx9500.h"
#include "ens210.h"
#include "tsl2572.h"
#include "battery.h"
#include "backlight.h"
#include "fxos8700.h"
#include "fxas21002_drv.h"

/* Authentication */
#include "a100x_interface.h"

#include "fsl_i2c.h"
#include "fsl_gpio.h"

/* I2C error codes. */
#define I2C_RESULT_OK                    0
#define I2C_RESULT_FAIL                  1

/* I2C1 SENSORS configuration */
#define BOARD_SENSORS_I2C_BASEADDR       I2C1
#define SENSORS_I2C_CLK_FREQ             CLOCK_GetFreq(I2C1_CLK_SRC)
#define SENSORS_I2C_BAUDRATE             100000U

/* I2C2 SECURITY / TOUCH configuration */
#define BOARD_SECURITY_I2C_BASEADDR      I2C2
#define NTAG_I2C_SLAVE_BASEADDR          I2C2
#define SECURITY_I2C_CLK_FREQ            CLOCK_GetFreq(I2C2_CLK_SRC)
#define SECURITY_I2C_BAUDRATE            100000U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
uint8_t I2C1_init(void);
uint8_t App_I2C1_Write(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize);
uint8_t App_I2C1_Read(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize);

uint8_t I2C2_init(void);
uint8_t App_I2C2_Write(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize);
uint8_t App_I2C2_Read(uint8_t device_addr, uint8_t *writeBuf, uint32_t writeSize, uint8_t *readBuf, uint32_t readSize);

uint8_t get_ambient_light(uint8_t *buf, uint8_t *size);
uint8_t get_air_quality(uint8_t *buf, uint8_t *size);
uint8_t get_pressure(uint8_t *buf, uint8_t *size);
uint8_t get_temperature(uint8_t *buf, uint8_t *size);
uint8_t get_humidity(uint8_t *buf, uint8_t *size);
uint8_t get_acceleration(uint8_t *buf, uint8_t *size);
uint8_t get_magnetic_field(uint8_t *buf, uint8_t *size);
uint8_t get_rotation_speed(uint8_t *buf, uint8_t *size);

/* Authentication */
uint8_t get_auth_uid(uint8_t *buf);
uint8_t get_auth_cert(uint8_t *buf);
uint8_t set_auth_challenge(uint8_t *buf);
uint8_t get_auth_response(uint8_t *buf);
uint8_t set_auth_softreset(void);

uint8_t Init_touchpad(void);

uint8_t Init_all_sensors(void);
uint8_t Init_ambient_light(void);
uint8_t Init_air_quality(void);
uint8_t Init_pressure(void);
uint8_t Init_air_humidity_temp(void);
uint8_t Init_battery_sensor(void);
uint8_t Init_rgb_led(void);
uint8_t Init_accel_mag(void);
uint8_t Init_rotation_speed(void);
uint8_t Init_backlight(void);
uint8_t Init_MotionDetect(void);
uint8_t Init_FreefallDetect(void);
uint8_t Init_TapDetect(void);

uint8_t DeInit_ambient_light(void);
uint8_t DeInit_air_quality(void);
uint8_t DeInit_pressure(void);
uint8_t DeInit_air_humidity_temp(void);
uint8_t DeInit_battery_sensor(void);
uint8_t DeInit_rgb_led(void);
uint8_t DeInit_accel_mag(void);
uint8_t DeInit_rotation_speed(void);
uint8_t DeInit_backlight(void);
uint8_t DeInit_MotionDetect(void);
uint8_t DeInit_FreefallDetect(void);
uint8_t DeInit_TapDetect(void);

bool tap_detected(bool *doubletap);
bool motion_detected(void);
bool freefall_detected(void);

/* Authentication */
uint8_t Init_authentication(void);

#endif /* SENSORS_H_ */
