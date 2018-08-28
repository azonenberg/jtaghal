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

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

FTDIDriver::~FTDIDriver()
{

}

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device enumeration

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
	//	Bit2 = TDI = input (0)
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
