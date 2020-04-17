/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief Implementation of FTDIDriver
 */

#include "jtaghal.h"

#ifdef HAVE_FTD2XX

#include <ftd2xx/ftd2xx.h>

#define FTDI_VID						0x0403	/* FTDI's USB vendor ID */
#define PID_232H_JTAG					0x8028	/* Product ID for azonenberg's FT232H based JTAG system */

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Connects to an FTDI interface

	@throw JtagException if the connection could not be established or the serial number is invalid

	@param serial		Serial number of the device to connect to
	@param layout		Adapter layout to use
 */
FTDIDriver::FTDIDriver(const string& serial, const string& layout)
{
	//Enable use of azonenberg's custom PID
	FT_STATUS err = FT_OK;
	if(FT_OK != (err = FT_SetVIDPID(FTDI_VID, PID_232H_JTAG)))
	{
		throw JtagExceptionWrapper(
			"FT_SetVIDPID() failed",
			"");
	}

	//Open the device
	if(FT_OK != (err = FT_OpenEx(
		const_cast<void*>(static_cast<const void*>(serial.c_str())),
		FT_OPEN_BY_SERIAL_NUMBER,
		&m_context)))
	{
		if(FT_OK != (err = FT_Open(
			std::stoi(serial.c_str()),
			&m_context)))
		{
			throw JtagExceptionWrapper(
				"FT_OpenEx() && FT_OPen() both failed",
				"");
		}
	}

	//Get some info
	DWORD type;
	DWORD id;
	char xserial[16];
	char desc[64];
	if(FT_OK != (err = FT_GetDeviceInfo(m_context, &type, &id, xserial, desc, NULL)))
	{
		throw JtagExceptionWrapper(
			"FT_GetDeviceInfo() failed",
			"");
	}
	m_serial = serial;
	m_userid = serial;
	m_name = desc;

	//Set clock rate to 10 MHz
	m_freq = 10000000;

	//Do the real init
	SharedCtorInit(type, layout);
}

FTDIDriver::~FTDIDriver()
{
	//Disable JTAG write enable and float pins
	//FIXME: this needs to be done layout dependent!!!
	SetGpioDirectionDeferred(3, true);
	SetGpioValueDeferred(3, false);
	WriteGpioState();

	//Commit pending operations
	Commit();

	if(m_context != NULL)
	{
		FT_Close(m_context);
		m_context = NULL;
	}
}


/**
	@brief Shared initialization used by all constructors
 */
void FTDIDriver::SharedCtorInit(uint32_t type, const string& layout)
{
	FT_STATUS err = FT_OK;

	//Get the chip type and append to the name (see FT_DEVICE enum)
	const char* chiptypes[]=
	{
		"BM",
		"AM",
		"100AX",
		"UNKNOWN",
		"2232C",
		"232R",
		"2232H",
		"4232H",
		"232H",
		"X_SERIES"
	};
	if(type <= static_cast<int>((sizeof(chiptypes) / sizeof(chiptypes[0]))))
		m_name += string(" (") + chiptypes[type] + ")";

	//Reset the adapter and purge buffers
	//TODO: reset device or only the port?
	if(FT_OK != (err = FT_ResetDevice(m_context)))
	{
		throw JtagExceptionWrapper(
			"FT_ResetDevice() failed",
			"");
	}
	if(FT_OK != (err = FT_Purge(m_context, FT_PURGE_RX | FT_PURGE_TX)))
	{
		throw JtagExceptionWrapper(
			"FT_Purge() failed",
			"");
	}

	//No need to set interface as with libftdi, we're opening the port directly rather than the device

	//Disable event/error characters
	if(FT_OK != (err = FT_SetChars(m_context, 0, 0, 0, 0)))
	{
		throw JtagExceptionWrapper(
			"FT_SetChars() failed",
			"");
	}

	//Set latency timer
	//Go as low as possible to improve RPC/DMA performance
	if(FT_OK != (err = FT_SetLatencyTimer(m_context, 2)))
	{
		throw JtagExceptionWrapper(
			"FT_SetLatencyTimer() failed",
			"");
	}

	//Set timeouts
	if(FT_OK != (err = FT_SetTimeouts(m_context, 1000, 1000)))
	{
		throw JtagExceptionWrapper(
			"FT_SetTimeouts() failed",
			"");
	}

	//Set USB transfer sizes
	if(FT_OK != (FT_SetUSBParameters(m_context, 1024, 4096)))
	{
		throw JtagExceptionWrapper(
			"FT_SetUSBParameters() failed",
			"");
	}

	//Reset MPSSE
	if(FT_OK != (err = FT_SetBitMode(m_context, 0x0, BIT_MODE_RESET)))
	{
		throw JtagExceptionWrapper(
			"FT_SetBitMode() failed",
			"");
	}

	//Enter bitbang mode
	//Pin modes set through MPSSE commands
	if(FT_OK != (err = FT_SetBitMode(m_context, 0x0, BIT_MODE_MPSSE)))
	{
		throw JtagExceptionWrapper(
			"FT_SetBitMode() failed",
			"");
	}
	Commit();

	//Sleep, as per AN129
	usleep(50 * 1000);

	//Send bogus command to synchronize the MPSSE (as per FTDI AN129)
	WriteData(MPSSE_INVALID_COMMAND);

	//Chip should respond with 0xFA then the bad command
	//Read until we get that
	unsigned char dummy_response[2] = {0x00, 0x00};
	while( (dummy_response[0] != MPSSE_INVALID_COMMAND_RESPONSE) || (dummy_response[1] != MPSSE_INVALID_COMMAND) )
	{
		dummy_response[0] = dummy_response[1];
		ReadData(dummy_response+1, 1);
	}

	//Initialize the MPSSE clock divider (see FTDI AN108)
	//TODO: Support stuff other than the -H types
	/*
	switch(m_context->type)
	{
	case TYPE_AM:
	case TYPE_BM:
	case TYPE_R:
		throw JtagExceptionWrapper(
			"The requested chip does not have a MPSSE, JTAG capability unavailable",
			"",
			JtagException::EXCEPTION_TYPE_ADAPTER);
		break;

	case TYPE_2232C:
		throw JtagExceptionWrapper(
			"FT2232C/D support not implemented",
			"",
			JtagException::EXCEPTION_TYPE_UNIMPLEMENTED);
		break;

	//libftdi 0.20 or later is required for 232H support
	case TYPE_232H:
	case TYPE_2232H:
	case TYPE_4232H:
		break;

	default:
		throw JtagExceptionWrapper(
			"Unknown FTDI chip type",
			"",
			JtagException::EXCEPTION_TYPE_UNIMPLEMENTED);
		break;
	}
	*/

	//Set clock rate to 10 MHz
	m_freq = 10000000;

	//General setup commands
	unsigned char cmd_setup[]=
	{
		MPSSE_DISABLE_DIV5,			//60 MHz base clock
		MPSSE_DISABLE_3PHA,			//No 3-phase clocking
		MPSSE_DISABLE_ADAPTIVE_CLK,	//No adaptive clocking
		MPSSE_SET_CLKDIV,
			0x02, 0x00,				//10 MHz
		MPSSE_DISABLE_LOOPBACK,		//No loopback mode
		MPSSE_FLUSH					//Flush buffers
	};
	WriteData(cmd_setup, sizeof(cmd_setup));
	Commit();

	//Initialize the GPIO pins
	//Start out by making everything inputs
	//TODO: have matching "tristate jtag pins on shutdown" config?
	for(int i=0; i<12; i++)
	{
		m_gpioValue.push_back(false);
		m_gpioDirection.push_back(false);
	}

	//Digilent HS1 and azonenberg's usb-jtag-mini (0403:8028)
	if(layout == "hs1")
	{
		//GPIOL3 is active HIGH output enable
		SetGpioDirectionDeferred(3, true);
		SetGpioValueDeferred(3, true);
	}

	//Digilent HS2
	//per https://sourceforge.net/p/xc3sprog/bugs/16/#1013/4c91/1c07
	//"HS1/HS2 DISABLE buffer by default, if D7 (HS1) or D7..D5(HS2) are not 1, then output drivers are not enabled"
	else if(layout == "hs2")
	{
		//GPIOL3...L1 are active HIGH output enable
		SetGpioDirectionDeferred(3, true);
		SetGpioDirectionDeferred(2, true);
		SetGpioDirectionDeferred(1, true);
		SetGpioValueDeferred(3, true);
		SetGpioValueDeferred(2, true);
		SetGpioValueDeferred(1, true);
	}

	//JTAGKey (or compatible bus blaster etc)
	else if(layout == "jtagkey")
	{
		//GPIOL0 is active LOW output enable
		SetGpioDirectionDeferred(0, true);
		SetGpioValueDeferred(0, false);

		//GPIOH0 is TRST_N, hold this high for now since we don't support TRST in the API yet
		SetGpioDirectionDeferred(4, true);
		SetGpioValueDeferred(4, true);

		//GPIOH2 is TRST_N_OE_N, hold this high so TRST is driven
		SetGpioDirectionDeferred(6, true);
		SetGpioValueDeferred(6, true);
	}

	else
	{
		throw JtagExceptionWrapper(
			string("Unrecognized FTDI layout ") + layout,
			"");
	}

	//Push GPIO config to activate outputs
	WriteGpioState();

	//Set timeouts
	if(FT_OK != (err = FT_SetTimeouts(m_context, 1000, 1000)))
	{
		throw JtagExceptionWrapper(
			"FT_SetTimeouts() failed",
			"");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device enumeration

/**
	@brief Checks if the requested FTDI device has a MPSSE (and is thus capable of being used for JTAG/SWD)

	Note that this function cannot tell if an MPSSE-capable chipset is actually configured for use as JTAG/SWD or
	as something else.

	@throw JtagException if the index is invalid or data could not be read

	@param index		Zero-based index of the device to test

	@return True if the device has a MPSSE, false otherwise.
 */
bool FTDIDriver::IsMPSSECapable(int index)
{
	FT_STATUS err = FT_OK;
	DWORD flags;
	DWORD type;
	DWORD id;
	DWORD loc;
	char serial[16];
	char desc[64];
	FT_HANDLE handle;
	if(FT_OK != (err = FT_GetDeviceInfoDetail(index, &flags, &type, &id, &loc, serial, desc, &handle)))
	{
		throw JtagExceptionWrapper(
			"FT_GetDeviceInfoDetail() failed",
			"");
	}

	LogDebug("device %d type %d desc %s serial %s flags %d\n", index, type, desc, serial, flags);

	if( (type == FT_DEVICE_2232H) || (type == FT_DEVICE_4232H) || (type == FT_DEVICE_232H) )
		return true;

	return false;
}

/**
	@brief Gets the version of the API

	@throw JtagException if the FTD2xx call fails

	@return FTDI driver and API version
 */
string FTDIDriver::GetAPIVersion()
{
	FT_STATUS err = FT_OK;
	DWORD lver;
	if(FT_OK != (err = FT_GetLibraryVersion(&lver)))
	{
		throw JtagExceptionWrapper(
			"FT_GetLibraryVersion() failed",
			"");
	}

	unsigned int build = lver & 0xff;
	unsigned int minor = (lver >> 8) && 0xff;
	unsigned int major = (lver >> 16) && 0xff;

	char sout[32];
	snprintf(sout, sizeof(sout)-1, "libftd2xx %u.%u.%u", major, minor, build);
	return sout;
}

/**
	@brief Gets the number of FTDI devices on the system (may include non-JTAG-capable devices)

	@throw JtagException if the FTD2xx call fails

	@return Number of interfaces found
 */
int FTDIDriver::GetInterfaceCount()
{
	//Enable use of azonenberg's custom PID
	FT_STATUS err = FT_OK;
	if(FT_OK != (err = FT_SetVIDPID(FTDI_VID, PID_232H_JTAG)))
	{
		throw JtagExceptionWrapper(
			"FT_SetVIDPID() failed",
			"");
	}

	DWORD ndev_raw;
	if(FT_OK != (err = FT_CreateDeviceInfoList(&ndev_raw)))
	{
		throw JtagExceptionWrapper(
			"FT_CreateDeviceInfoList() failed",
			"");
	}

	return ndev_raw;
}

/**
	@brief Returns the description of the Nth device

	@param index		Zero-based index of the device to test

	@throw JtagException if the index is invalid or data could not be read

	@return Serial number string
 */
string FTDIDriver::GetSerialNumber(int index)
{
	char serial[16];
	FT_STATUS err = FT_OK;
	if(FT_OK != (err = FT_ListDevices(reinterpret_cast<void*>(index), serial, FT_LIST_BY_INDEX | FT_OPEN_BY_SERIAL_NUMBER)))
	{
		throw JtagExceptionWrapper(
			"FT_ListDevices() failed",
			"");
	}
	return serial;
}

/**
	@brief Returns the description of the Nth device

	@throw JtagException if the index is invalid or data could not be read

	@return Description string
 */
string FTDIDriver::GetDescription(int index)
{
	char desc[64];
	FT_STATUS err = FT_OK;
	if(FT_OK != (err = FT_ListDevices(reinterpret_cast<void*>(index), desc, FT_LIST_BY_INDEX | FT_OPEN_BY_DESCRIPTION)))
	{
		throw JtagExceptionWrapper(
			"FT_ListDevices() failed",
			"");
	}
	return desc;
}

/**
	@brief Returns the default clock frequency of the Nth device

	@throw JtagException if the index is invalid or data could not be read

	@return Clock frequency
 */
int FTDIDriver::GetDefaultFrequency(int /*index*/)
{
	return 10000000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MPSSE synchronization

/**
	@brief Verifies that we're still in sync with the MPSSE

	@throw JtagException if an FTDI API call fails
 */
void FTDIDriver::SyncCheck()
{
	printf("    Sync check\n");

	//Send bogus command 0xAA to synchronize the MPSSE (as per FTDI AN129)
	WriteData(MPSSE_INVALID_COMMAND);

	//Chip should respond with 0xFA AA
	//Read until we get that
	int n=0;
	unsigned char dummy_response[2] = {0x00, 0x00};
	while( (dummy_response[0] != MPSSE_INVALID_COMMAND_RESPONSE) || (dummy_response[1] != MPSSE_INVALID_COMMAND) )
	{
		dummy_response[0] = dummy_response[1];
		ReadData(dummy_response+1, 1);

		if( (n == 0) && (dummy_response[1] != MPSSE_INVALID_COMMAND_RESPONSE) )
			LogError("    SYNC ERROR at position 0 - expected MPSSE_INVALID_COMMAND_RESPONSE (0xFA), got %02x\n",
				dummy_response[1] & 0xFF);
		else
			LogError("    Got 0x%02x\n", dummy_response[1] & 0xFF);
		n++;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO stuff

void FTDIDriver::ReadGpioState()
{
	unsigned char cmd[]=
	{
		MPSSE_GET_DATA_LOW,
		MPSSE_GET_DATA_HIGH
	};
	unsigned char buf[2];
	WriteData(cmd, sizeof(cmd));
	ReadData(buf, 2);

	//Unpack
	m_gpioValue[0]  = (buf[0] & 0x10) ? true : false;
	m_gpioValue[1]  = (buf[0] & 0x20) ? true : false;
	m_gpioValue[2]  = (buf[0] & 0x40) ? true : false;
	m_gpioValue[3]  = (buf[0] & 0x80) ? true : false;
	m_gpioValue[4]  = (buf[1] & 0x01) ? true : false;
	m_gpioValue[5]  = (buf[1] & 0x02) ? true : false;
	m_gpioValue[6]  = (buf[1] & 0x04) ? true : false;
	m_gpioValue[7]  = (buf[1] & 0x08) ? true : false;
	m_gpioValue[8]  = (buf[1] & 0x10) ? true : false;
	m_gpioValue[9]  = (buf[1] & 0x20) ? true : false;
	m_gpioValue[10] = (buf[1] & 0x40) ? true : false;
	m_gpioValue[11] = (buf[1] & 0x80) ? true : false;
}

void FTDIDriver::WriteGpioState()
{
	//Pack
	unsigned char value_low =
		(m_gpioValue[0] << 4) |
		(m_gpioValue[1] << 5) |
		(m_gpioValue[2] << 6) |
		(m_gpioValue[3] << 7);
	unsigned char dir_low =
		(m_gpioDirection[0] << 4) |
		(m_gpioDirection[1] << 5) |
		(m_gpioDirection[2] << 6) |
		(m_gpioDirection[3] << 7);
	unsigned char value_hi =
		(m_gpioValue[4]) |
		(m_gpioValue[5] << 1) |
		(m_gpioValue[6] << 2) |
		(m_gpioValue[7] << 3) |
		(m_gpioValue[8] << 4) |
		(m_gpioValue[9] << 5) |
		(m_gpioValue[10] << 6) |
		(m_gpioValue[11] << 7);
	unsigned char dir_hi =
		(m_gpioDirection[4]) |
		(m_gpioDirection[5] << 1) |
		(m_gpioDirection[6] << 2) |
		(m_gpioDirection[7] << 3) |
		(m_gpioDirection[8] << 4) |
		(m_gpioDirection[9] << 5) |
		(m_gpioDirection[10] << 6) |
		(m_gpioDirection[11] << 7);

	//Force low bits for JTAG pins
	//	Bit0 = TCK = output (1)
	//	Bit1 = TDI = output (1)
	//	Bit2 = TDO = input (0)
	//	Bit3 = TMS = output (1)
	//  TMS idles high, rest idle low
	value_low = (value_low & 0xF0) | 0x08;
	dir_low = (dir_low & 0xF0) | 0x0B;

	unsigned char cmd[] =
	{
		MPSSE_SET_DATA_LOW,
		value_low,
		dir_low,
		MPSSE_SET_DATA_HIGH,
		value_hi,
		dir_hi
	};
	WriteData(cmd, sizeof(cmd));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions

void FTDIDriver::Commit()
{
	if(!m_writeBuffer.empty())
	{
		WriteDataRaw(&m_writeBuffer[0], m_writeBuffer.size());
		m_writeBuffer.clear();
	}
}

/**
	@brief Writes FTDI MPSSE data to the interface.

	Writes may be deferred until Commit() is called to improve performance.

	@throw JtagException on failure

	@param data				Data to write
	@param bytesToWrite		Number of bytes to write
 */
void FTDIDriver::WriteData(const void* data, size_t bytesToWrite)
{
	const unsigned char* p = reinterpret_cast<const unsigned char*>(data);
	for(size_t i=0; i<bytesToWrite; i++)
		m_writeBuffer.push_back(p[i]);

	//Don't let the buffer get TOO big
	if(m_writeBuffer.size() >= 4096)
		Commit();
}

/**
	@brief Wrapper around FT_Write() to push the provided data buffer to hardware.

	Performs repeated write calls as needed to ensure the entire buffer is written.

	@throw JtagException on failure

	@param data				Data to write
	@param bytesToWrite		Number of bytes to write
 */
void FTDIDriver::WriteDataRaw(const void* data, size_t bytesToWrite)
{
	FT_STATUS status;
	DWORD bytesWritten;
	size_t bytes_left = bytesToWrite;

	//for some reason the buffer isn't a const... are we certain d2xx won't change it?
	unsigned char* pdata = reinterpret_cast<unsigned char*>(
								const_cast<void*>(
									data
									)
								);

	while(bytes_left != 0)
	{
		if(FT_OK != (status = FT_Write(m_context, pdata, bytes_left, &bytesWritten)))
		{
			throw JtagExceptionWrapper(
				"FT_Write() failed",
				"");
		}

		if(bytesWritten > bytes_left)
		{
			throw JtagExceptionWrapper(
				"FT_Write() wrote too much data",
				"");
		}

		bytes_left -= bytesWritten;
		pdata += bytesWritten;
	}
}

/**
	@brief Wrapper around WriteData() to send a single byte

	@throw JtagException on failure

	@param cmd		The single byte to write
 */
void FTDIDriver::WriteData(unsigned char cmd)
{
	WriteData(&cmd, 1);
}

/**
	@brief Wrapper around FT_Read() to work around some driver / API bugs

	@throw JtagException on failure

	@param data				Data to write
	@param bytesToRead		Number of bytes to read
 */
void FTDIDriver::ReadData(void* data, size_t bytesToRead)
{
	//Push outstanding writes
	Commit();

	unsigned char* p = (unsigned char*)data;
	DWORD bytesRead;
	size_t bytesTotal = bytesToRead;
	FT_STATUS status;
	int i = 0;
	int j=0;
	while(true)
	{
		j++;

		//Get the status of the device.
		//Apparently we need to call FT_GetStatus() before a second read operation will succeed.
		//See http://www.alecjacobson.com/weblog/?p=2934
		DWORD rxsize;
		DWORD txsize;
		DWORD evstat;
		if(FT_OK != (status = FT_GetStatus(m_context, &rxsize, &txsize, &evstat)))
		{
			throw JtagExceptionWrapper(
				"FT_GetStatus() failed",
				"");
		}

		//No data? Wait one USB packet time and try again
		if(rxsize == 0)
		{
			usleep(125);
			if( (j % 2000) == 1999)
			{
				printf("[FTDIDriver] Read is taking a long time, flushing... (j=%d, i=%d)\n", j, i);
				WriteData(MPSSE_FLUSH);
				Commit();
			}
			continue;
		}

		//If we get to this point data is ready to read
		if(FT_OK != (status = FT_Read(m_context, p, bytesToRead, &bytesRead)))
		{
			throw JtagExceptionWrapper(
				"FT_Read() failed",
				"");
		}

		//Note how many bytes actually got read
		bytesToRead -= bytesRead;
		p += bytesRead;
		if(bytesToRead == 0)
		{
			if(i != 0)
				printf("    Read completed OK\n");
			break;
		}

		//If not fully read, keep trying
		printf("[FTDIDriver] More data to read (iteration %d, %zu read this call, %zu left, %zu total)\n",
			++i, (size_t)bytesRead, bytesToRead, bytesTotal);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adapter information

/**
	@brief Gets the manufacturer-assigned name for this programming adapter
 */
string FTDIDriver::GetName()
{
	return m_name;
}

/**
	@brief Gets the manufacturer-assigned serial number for this programming adapter
 */
string FTDIDriver::GetSerial()
{
	return m_serial;
}

/**
	@brief Gets the user-assigned name for this programming adapter
 */
string FTDIDriver::GetUserID()
{
	return m_userid;
}

/**
	@brief Gets the clock frequency, in Hz, of the JTAG interface
 */
int FTDIDriver::GetFrequency()
{
	return m_freq;
}

#endif
