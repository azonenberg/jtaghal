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
	@brief Implementation of FTDISWDInterface
 */

#include "jtaghal.h"

#ifdef HAVE_FTD2XX

#include <ftd2xx/ftd2xx.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Connects to an FTDI JTAG interface

	@throw SWDException if the connection could not be establishes or the serial number is invalid

	@param serial		Serial number of the device to connect to
	@param layout		Adapter layout to use
 */
FTDISWDInterface::FTDISWDInterface(const string& serial, const string& layout)
	: FTDIDriver(serial, layout)
{
}

/**
	@brief Interface destructor

	Closes handles and disconnects from the adapter.
 */
FTDISWDInterface::~FTDISWDInterface()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Shim overrides to push SWDInterface functions into FTDIDriver

void FTDISWDInterface::Commit()
{
	FTDIDriver::Commit();
}

string FTDISWDInterface::GetName()
{
	return FTDIDriver::GetName();
}

string FTDISWDInterface::GetSerial()
{
	return FTDIDriver::GetSerial();
}

string FTDISWDInterface::GetUserID()
{
	return FTDIDriver::GetUserID();
}

int FTDISWDInterface::GetFrequency()
{
	return FTDIDriver::GetFrequency();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level

/**
	@brief Resets the SWD link layer
 */
void FTDISWDInterface::ResetInterface()
{
	uint8_t cmdbuf[] =
	{
		//Need to send 50+ clocks with SWDIO (TDO) high per 4.4.3
		//We send 56 clocks (7 bytes) so its a nice integer number of bytes
		MPSSE_TX_BYTES,
		0x06,										//offset by one, so sending 7 bytes
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

		//Need to send at least 2 clocks with SWDIO (TDO) low per 4.4.3
		//We send 8 clocks so its a nice integer number of bytes
		MPSSE_TX_BYTES,
		0x00,										//offset by one, so sending 1 byte
		0x00,

		MPSSE_FLUSH
	};

	WriteDataRaw(cmdbuf, sizeof(cmdbuf));
}

/**
	@brief Performs a SW-DP write transaction

	TODO: optimize to 1 transaction using overrun detection?
 */
void FTDISWDInterface::WriteWord(uint8_t reg_addr, bool ap, uint32_t wdata)
{
	//Format the header
	//LSB first: 1, AP / #DP, 0, A[2:3], Parity, 0, 1
	uint8_t header = 0x81;
	bool parity = false;
	if(ap)
	{
		header |= 0x02;
		parity ^= 1;
	}
	if(reg_addr & 4)
	{
		header |= 0x08;
		parity ^= 1;
	}
	if(reg_addr & 8)
	{
		header |= 0x10;
		parity ^= 1;
	}
	if(parity)
		header |= 0x20;

	//We need to switch the TDO/SWDIO pin back and forth from output to tristate a couple of times.
	//It's also a GPIO bitbang pin, though - so we need to reconfigure the low GPIO bank a bunch.
	//Bit 1 = TDI = output (1) by default
	unsigned char value_low =
		(m_gpioValue[0] << 4) |
		(m_gpioValue[1] << 5) |
		(m_gpioValue[2] << 6) |
		(m_gpioValue[3] << 7) |
		0x08;
	unsigned char dir_low =
		(m_gpioDirection[0] << 4) |
		(m_gpioDirection[1] << 5) |
		(m_gpioDirection[2] << 6) |
		(m_gpioDirection[3] << 7) |
		0x0B;

	//Calculate the parity for the data
	uint32_t dpartmp = (wdata >> 16) | (wdata >> 8) | (wdata >> 4) | (wdata >> 2) | (wdata >> 1) | wdata;

	uint8_t cmdbuf[] =
	{
		//The header
		header,

		//Tristate TDI
		MPSSE_SET_DATA_LOW,
		value_low,
		dir_low ^ 0x2,			//flip TDI to an input

		//Send a 1-bit bus turnaround as tristate
		MPSSE_DUMMY_CLOCK_BITS,
		0x00,

		//Read the three-bit ACK from the target
		MPSSE_TXRX_BYTES,
		0x02,
		0x00,
		0x00,					//TDI is tristated so send data doesn't matter

		//Send another bus turnaround cycle so the target can tristate the bus
		MPSSE_DUMMY_CLOCK_BITS,
		0x00,					//1 bit

		//Reconfigure TDI to actually drive again
		MPSSE_SET_DATA_LOW,
		value_low,
		dir_low
	};
	WriteDataRaw(cmdbuf, sizeof(cmdbuf));

	//Send everything up to the ACK then see what comes back (should be a single byte)
	uint8_t ack = 0;
	ReadData(&ack, 1);

	//WAIT request? TODO
	if(ack == 2)
	{
		LogError("TODO: Handle WAIT request from SW-DP\n");
		return;
	}

	//Some strange response that isn't what we expect
	else if(ack != 1)
	{
		LogError("Weird - we got something other than ACK or WAIT\n");
		return;
	}

	//If we get here, it was a good ACK.
	//Send the rest of the data
	uint8_t databuf[] =
	{
		//Write the actual data to the target
		MPSSE_TX_BYTES,
		0x03,
		0x00,
		(wdata >> 0) & 0xff,
		(wdata >> 8) & 0xff,
		(wdata >> 16) & 0xff,
		(wdata >> 24) & 0xff,

		//Write the parity bit for the transmit data
		MPSSE_TX_BITS,
		0x00,
		0x00,
		(dpartmp & 1) ? 0x01 : 0x00
	};
	WriteDataRaw(databuf, sizeof(databuf));
}

/**
	@brief Performs a SW-DP read transaction
 */
uint32_t FTDISWDInterface::ReadWord(uint8_t reg_addr, bool ap)
{
	//Host to target: 8 bit header
	//LSB first: 1, AP / #DP, 1, A[2:3], Parity, 0, 1
	uint8_t header = 0x85;
	bool parity = true;
	if(ap)
	{
		header |= 0x02;
		parity ^= 1;
	}
	if(reg_addr & 4)
	{
		header |= 0x08;
		parity ^= 1;
	}
	if(reg_addr & 8)
	{
		header |= 0x10;
		parity ^= 1;
	}
	if(parity)
		header |= 0x20;

	//We need to switch the TDO/SWDIO pin back and forth from output to tristate a couple of times.
	//It's also a GPIO bitbang pin, though - so we need to reconfigure the low GPIO bank a bunch.
	//Bit 1 = TDI = output (1) by default
	unsigned char value_low =
		(m_gpioValue[0] << 4) |
		(m_gpioValue[1] << 5) |
		(m_gpioValue[2] << 6) |
		(m_gpioValue[3] << 7) |
		0x08;
	unsigned char dir_low =
		(m_gpioDirection[0] << 4) |
		(m_gpioDirection[1] << 5) |
		(m_gpioDirection[2] << 6) |
		(m_gpioDirection[3] << 7) |
		0x0B;

	uint8_t cmdbuf[] =
	{
		//The header
		header,

		//Tristate TDI
		MPSSE_SET_DATA_LOW,
		value_low,
		dir_low ^ 0x2,			//flip TDI to an input

		//Send a 1-bit bus turnaround as tristate
		MPSSE_DUMMY_CLOCK_BITS,
		0x00,

		//Read the three-bit ACK from the target
		MPSSE_TXRX_BYTES,
		0x02,
		0x00,
		0x00					//TDI is tristated so send data doesn't matter
	};
	WriteDataRaw(cmdbuf, sizeof(cmdbuf));

	//Send everything up to the ACK then see what comes back (should be a single byte)
	uint8_t ack = 0;
	ReadData(&ack, 1);

	//WAIT request? TODO
	if(ack == 2)
	{
		LogError("TODO: Handle WAIT request from SW-DP\n");
		return 0;
	}

	//Some strange response that isn't what we expect
	else if(ack != 1)
	{
		LogError("Weird - we got something other than ACK or WAIT\n");
		return 0;
	}

	//Read data from target to host: 32 bits, LSB first
	//1 bit parity
	//1 bit bus turnaround (tristate)
	uint8_t rxd[5];
	uint8_t databuf[] =
	{
		//Read the data plus parity bits
		MPSSE_TXRX_BITS,
		0x20,								//33 total bits
		0x00, 0x00, 0x00, 0x00, 0x00,

		//Send a bus turnaround cycle so the target can tristate the bus
		MPSSE_DUMMY_CLOCK_BITS,
		0x00,								//1 bit

		//Reconfigure TDI to actually drive again
		MPSSE_SET_DATA_LOW,
		value_low,
		dir_low
	};
	WriteDataRaw(databuf, sizeof(databuf));
	ReadData(&rxd, 5);
}

#endif
