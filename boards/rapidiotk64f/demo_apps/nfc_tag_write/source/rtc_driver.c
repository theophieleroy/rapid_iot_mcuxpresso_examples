/*
 * rtc_driver.c
 *
 *  Created on: Sep 7, 2018
 *      Author: nxf47227
 */

#include "rtc_driver.h"

/*
 * Prepare and send data via SPI to RTC
 */
uint8_t RTC_DateTime_SpiWrite(uint8_t *writeBuf, uint32_t writeSize)
{
	uint8_t masterRxData[writeSize];
	dspi_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.txData = writeBuf;
	masterXfer.rxData = masterRxData;
	masterXfer.dataSize = writeSize;
	masterXfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	return (DSPI_MasterTransferBlocking(DSPI_MASTER_BASEADDR, &masterXfer));
}

/*
 * Prepare and read data via SPI from RTC
 */
uint8_t RTC_DateTime_SpiRead(uint8_t *writeBuf, uint8_t *readBuf, uint32_t readSize)
{
	uint8_t masterTxData[readSize+1];
	memset(masterTxData, 0, sizeof(masterTxData));
	dspi_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterTxData[0] = writeBuf[0];		// read + sub address

	masterXfer.txData = masterTxData;
	masterXfer.rxData = readBuf;
	masterXfer.dataSize = readSize+1;
	masterXfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	return (DSPI_MasterTransferBlocking(DSPI_MASTER_BASEADDR, &masterXfer));
}

void RTC_App_WaitMsec(uint32_t millisec){
	uint32_t i = 0;
	for (i = 0; i < millisec * 8000U; i++) {
		__NOP();
	}
}

void RTC_App_WaitUsec(uint32_t microsec){
	uint32_t i = 0;
	for (i = 0; i < microsec * 8U; i++) {
		__NOP();
	}
}

uint8_t RTC_DateTime_Init(void)
{
	pcf2123_IoFunc_t io;
	io.SPI_Read = RTC_DateTime_SpiRead;
	io.SPI_Write = RTC_DateTime_SpiWrite;
	io.WaitMsec = RTC_App_WaitMsec;
	PCF2123_Init_Driver(&io);

	settingsPCF_t PCFsettings;
	PCFsettings.mode12_24 = PCF2123_MODE_24HOURS;	// to be completed
	PCFsettings.Softreset = true;					// Force Software reset => Date and time need to be set
	PCFsettings.hours = 0x12;						// 12H = 12AM
	PCFsettings.minutes = 0x00;						// 0min
	PCFsettings.seconds = 0x00;						// 0s
	PCFsettings.weekdays = Thursday;
	PCFsettings.days = 0x12;
	PCFsettings.months = September;
	PCFsettings.years = 0x18;						// 2018
	PCFsettings.clockOutputFreq = clkoutFreq_32768;	// Set Clock output frequency to 32768Hz
	PCFsettings.MinInterrupt = false;				// Disable minute interrupt
	PCFsettings.SecInterrupt = true;				// Enable second interrupt
	PCFsettings.PulseInterrupt = true;				// interrupt pin generates a pulse

	RTC_App_WaitUsec(100);

	if (PCF2123_Init_Hw(&PCFsettings) != PCF2123_SUCCESS)
	{
		return 0;
	}

	if (PCF2123_SetClockOutputFreq(clkoutFreq_32768) != 0)
	{
		return 0;
	}
	return 1;
}
