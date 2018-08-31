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
	@brief Implementation of FTDIJtagInterface
 */

#include "jtaghal.h"

#ifdef HAVE_FTD2XX

#include <ftd2xx/ftd2xx.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Connects to an FTDI JTAG interface

	@throw JtagException if the connection could not be establishes or the serial number is invalid

	@param serial		Serial number of the device to connect to
	@param layout		Adapter layout to use
 */
FTDIJtagInterface::FTDIJtagInterface(const string& serial, const string& layout)
	: FTDIDriver(serial, layout)
{
}

/**
	@brief Interface destructor

	Closes handles and disconnects from the adapter.
 */
FTDIJtagInterface::~FTDIJtagInterface()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Shim overrides to push JtagInterface functions into FTDIDriver

void FTDIJtagInterface::Commit()
{
	FTDIDriver::Commit();
}

string FTDIJtagInterface::GetName()
{
	return FTDIDriver::GetName();
}

string FTDIJtagInterface::GetSerial()
{
	return FTDIDriver::GetSerial();
}

string FTDIJtagInterface::GetUserID()
{
	return FTDIDriver::GetUserID();
}

int FTDIJtagInterface::GetFrequency()
{
	return FTDIDriver::GetFrequency();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low-level JTAG interface

void FTDIJtagInterface::ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count)
{
	double start = GetTime();

	m_perfShiftOps ++;
	m_perfDataBits += count;

	bool want_read = true;
	if(rcv_data == NULL)
		want_read = false;

	//Purge the output data with zeros (in case we arent receving an integer number of bytes)
	if(want_read)
	{
		int bytecount = ceil(count/8.0f);
		memset(rcv_data, 0, bytecount);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Bulk data transfers
	//4KB at a time
	//Do NOT send the last bit in this loop (for proper handling of last_tms)
	const int BITS_PER_BYTE = 8;
	const int BLOCK_SIZE_KBYTE = 4096;
	const int BLOCK_SIZE_KBIT = BLOCK_SIZE_KBYTE * BITS_PER_BYTE;
	while(count > BLOCK_SIZE_KBIT)
	{
		//Write command header, data block, and flush command
		unsigned char header[3]=
		{
			static_cast<unsigned char>(want_read ? MPSSE_TXRX_BYTES : MPSSE_TX_BYTES),
																	//Clock data out on negative clock edge
			0xFF,													//Length, little endian = 4095 = 0F FF (off by one)
			0x0F
		};
		WriteData(header, 3);
		WriteData(send_data, BLOCK_SIZE_KBYTE);
		WriteData(MPSSE_FLUSH);

		//Read data back
		if(want_read)
			ReadData(rcv_data, BLOCK_SIZE_KBYTE);

		//Bump pointers and mark space as used
		send_data += BLOCK_SIZE_KBYTE;
		if(want_read)
			rcv_data += BLOCK_SIZE_KBYTE;
		count -= BLOCK_SIZE_KBIT;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate and send the command packet for the rest of the data
	vector<unsigned char> cmd;
	GenerateShiftPacket(send_data, count, want_read, last_tms, cmd);
	WriteData(&cmd[0], cmd.size());

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Read the data
	if(want_read)
		DoReadback(rcv_data, count);

	m_perfShiftTime += GetTime() - start;
}

bool FTDIJtagInterface::IsSplitScanSupported()
{
	return true;
}

bool FTDIJtagInterface::ShiftDataWriteOnly(	bool last_tms,
											const unsigned char* send_data,
											unsigned char* rcv_data, size_t count)
{
	//If count is too big, don't pipeline
	if(count >= (8 * 4096))
	{
		ShiftData(last_tms, send_data, rcv_data, count);
		return false;
	}

	//Otherwise, send the write
	vector<unsigned char> cmd;
	GenerateShiftPacket(send_data, count, (rcv_data != NULL), last_tms, cmd);
	WriteData(&cmd[0], cmd.size());
	return true;
}

bool FTDIJtagInterface::ShiftDataReadOnly(unsigned char* rcv_data, size_t count)
{
	if(count >= (8 * 4096))
		return false;

	if(rcv_data != NULL)
		DoReadback(rcv_data, count);
	return true;
}

/**
	@brief Reads back data from a prior transaction

	@param rcv_data		Output data buffer
	@param count		Number of bits to read
 */
void FTDIJtagInterface::DoReadback(unsigned char* rcv_data, size_t count)
{
	int bytes_left = count / 8;
	if( (count & 7) == 0)
		bytes_left --;
	if(bytes_left > 0)
		count -= bytes_left * 8;
	int bl = count - 2;
	int nbit = count-1;

	WriteData(MPSSE_FLUSH);

	//Byte-oriented data
	if(bytes_left > 0)
	{
		ReadData(rcv_data, bytes_left);
		rcv_data += bytes_left;
	}

	//Bit-oriented data
	if(bl >= 0)
	{
		ReadData(rcv_data, 1);

		//Shift so we're right-aligned
		rcv_data[0] >>= (8 - count + 1);
	}

	//Last bit
	unsigned char tmp = 0;
	ReadData(&tmp, 1);
	PokeBit(rcv_data, nbit, (tmp & 0x80) ? true : false);
}

/**
	@brief Generates the MPSSE commands for a shift operation

	@param send_data	Data to send
	@param count		Number of bits to send (not bytes)
	@param want_read	True if read data is needed, false for a write-only transaction
	@param last_tms		TMS value to use at the end of the shift operation (all other bits have TMS=0)
	@param cmd_out		The generated command buffer
 */
void FTDIJtagInterface::GenerateShiftPacket(
	const unsigned char* send_data, size_t count,
	bool want_read,
	bool last_tms,
	vector<unsigned char>& cmd_out)
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Bulk data transfer is done. We now have less than 4KB left, but it might not be an even number of bytes
	//Send until we have <=8 bits left
	//Do *not* send the last bit here (for proper handling of last_tms)
	int bytes_left = count / 8;
	if( (count & 7) == 0)
		bytes_left --;

	//Send the byte-oriented data (subtract 1 from count)
	int bl = bytes_left - 1;
	if(bytes_left > 0)
	{
		cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_BYTES : MPSSE_TX_BYTES));
		cmd_out.push_back(0xFF & bl);
		cmd_out.push_back(bl >> 8);
		for(int i=0; i<bytes_left; i++)
			cmd_out.push_back(send_data[i]);

		//Bump pointers
		send_data += bytes_left;
		count -= bytes_left * 8;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Byte sending is done. We now have <=8 bits left. May or may not be an even number of bytes.
	//Send all but the last bit at this time.

	//Write header and data
	bl = count - 2;								//Header count is offset by 1, then subtract again to skip the last bit
	if(bl >= 0)
	{
		cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_BITS : MPSSE_TX_BITS));
		cmd_out.push_back(static_cast<unsigned char>(bl));
		cmd_out.push_back(send_data[0]);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Bit sending is done. We now have one bit left.
	//Send the last bit as TMS
	int nbit = count-1;
	int send_last = PeekBit(send_data, nbit);
	cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_TMS_BITS : MPSSE_TX_TMS_BITS));
												//Send data to TMS on falling edge, then read
	cmd_out.push_back(0);						//Send 1 bit
	cmd_out.push_back(static_cast<unsigned char>((send_last ? 0x80 : 0) | (last_tms ? 1 : 0)));
												//Bit 7 is last data bit to send
												//Bit 0 is TMS bit
}

void FTDIJtagInterface::ShiftTMS(bool tdi, const unsigned char* send_data, size_t count)
{
	double start = GetTime();

	m_perfShiftOps ++;
	m_perfModeBits += count;

	if(count > 7)
	{
		throw JtagExceptionWrapper(
			"ShiftTMS() not implemented for count > 7",
			"");
	}

	//Clock data to TMS, LSB first
	unsigned char command[3] =
	{
		MPSSE_TX_TMS_BITS,
		static_cast<unsigned char>(count - 1),
		static_cast<unsigned char>((send_data[0] & 0x7F) | (tdi ? 0x80 : 0))
	};
	WriteData(command, 3);

	m_perfShiftTime += GetTime() - start;
}

void FTDIJtagInterface::SendDummyClocks(size_t n)
{
	SendDummyClocksDeferred(n);

	double start = GetTime();

	//Dummy clocks are often used as a delay cycle
	//so force the write to complete now
	Commit();

	m_perfShiftTime += GetTime() - start;
}

void FTDIJtagInterface::SendDummyClocksDeferred(size_t n)
{
	double start = GetTime();

	m_perfShiftOps ++;
	m_perfDummyClocks += n;

	int nbytes = n / 8;
	if(nbytes >= 0xFFFF)
	{
		throw JtagExceptionWrapper(
			"SendDummyClocks() does not implement values > (0xFFFF * 8)",
			"");
	}

	//Bulk dummy clocks (in groups of 8)
	if(nbytes != 0)
	{
		//"This will pulse the clock for 8 to (8 x $10000) times given by length. A length of 0x0000 will do 8 clocks
		//and a length of 0xFFFF will do 524288 clocks"
		nbytes --;
		unsigned char command[3] =
		{
			MPSSE_DUMMY_CLOCK_BYTES,
			static_cast<unsigned char>(nbytes & 0xFF),
			static_cast<unsigned char>((nbytes >> 8) & 0xFF)
		};

		WriteData(command, 3);
	}

	//Finish off to the exact count requested
	//"This will pulse the clock for 1 to 8 times given by length. A length of 0x00 will do 1 clock and a length of
	//0x07 will do 8 clocks."
	int nbits = n & 7;
	nbits --;
	unsigned char command[2]=
	{
		MPSSE_DUMMY_CLOCK_BITS,
		static_cast<unsigned char>(nbits),
	};

	WriteData(command, 2);

	m_perfShiftTime += GetTime() - start;
}

#endif	//#ifdef HAVE_FTD2XX
