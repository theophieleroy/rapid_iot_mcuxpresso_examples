/*
 * rtc_driver.h
 *
 *  Created on: Sep 7, 2018
 *      Author: nxf47227
 */

#ifndef SOURCE_RTC_DRIVER_H_
#define SOURCE_RTC_DRIVER_H_

#include "spi_bus_share.h"
#include "pcf2123.h"
#include "board.h"
#include "fsl_dspi.h"
#include "spi_flash_driver.h"

/*
 * Init the rtc module with default values and
 * set read, write and wait functions for SPI
 */
uint8_t RTC_DateTime_Init(void);

#endif /* SOURCE_RTC_DRIVER_H_ */
