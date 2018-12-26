/*
 * Test_Ntag.h
 *
 *  Created on: Dec 13, 2017
 *      Author: frq05115
 */

#ifndef TEST_NTAG_H_
#define TEST_NTAG_H_

#include "stdint.h"
#include "stdbool.h"

#include "nfc_device.h"
#include "ntag_bridge.h"
#include "HAL_I2C_driver.h"
#include "TAG_Access.h"
#include "shell.h"

/*
 * Init NCF module and write default data
 */
bool Init_Ntag_NDEF(void);

/*
 * Write new data to NCF buffer
 */
bool Write_Ntag_NDEF(char *buf);

#endif /* TEST_NTAG_H_ */
