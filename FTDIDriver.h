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
	@brief Declaration of FTDIDriver
 */

#ifndef FTDIDriver_h
#define FTDIDriver_h

#ifdef HAVE_FTD2XX

#define FTDI_VID						0x0403	/* FTDI's USB vendor ID */
#define PID_232H_JTAG					0x8028	/* Product ID for azonenberg's FT232H based JTAG system */

#define BIT_MODE_RESET					0x00	/* Reset the MPSSE */
#define BIT_MODE_MPSSE					0x02	/* MPSSE mode */

/**
	@brief Common logic used by all FTDI adapters, regardless of transport layer selected
 */
class FTDIDriver : public GPIOInterface
{
public:
	virtual ~FTDIDriver();

	//Interface enumeration and discovery
public:
	virtual std::string GetName();
	virtual std::string GetSerial();
	virtual std::string GetUserID();
	virtual int GetFrequency();

	static bool IsMPSSECapable(int index);
	static int GetDefaultFrequency(int index);
	static std::string GetSerialNumber(int index);
	static std::string GetDescription(int index);
	static std::string GetAPIVersion();
	static int GetInterfaceCount();

	//Buffer management
public:
	virtual void Commit();

	//GPIO stuff
public:
	virtual void ReadGpioState();
	virtual void WriteGpioState();

	//Internal I/O helpers
protected:
	///@brief Cached name of this adapter
	std::string m_name;

	///@brief Cached serial number of this adapter
	std::string m_serial;

	///@brief Cached user ID of this adapter
	std::string m_userid;

	///@brief Cached clock frequency of this adapter
	int m_freq;

	///@brief Libftd2xx interface handle
	void* m_context;

	///@brief Buffer of data queued for the adapter, but not yet sent
	std::vector<unsigned char> m_writeBuffer;

	void SyncCheck();

	void ReadData(void* data, size_t bytesToRead);
	void WriteDataRaw(const void* data, size_t bytesToWrite);
	void WriteData(const void* data, size_t bytesToWrite);
	void WriteData(unsigned char cmd);

	//Internal constants
protected:
	enum MPSSE_Commands
	{
		MPSSE_TX_BYTES					= 0x19,
		MPSSE_TX_BITS					= 0x1b,
		MPSSE_TXRX_BYTES				= 0x39,
		MPSSE_TXRX_BITS					= 0x3b,
		MPSSE_TX_TMS_BITS				= 0x4b,
		MPSSE_TXRX_TMS_BITS				= 0x6b,
		MPSSE_SET_DATA_LOW				= 0x80,
		MPSSE_GET_DATA_LOW				= 0x81,
		MPSSE_SET_DATA_HIGH				= 0x82,
		MPSSE_GET_DATA_HIGH				= 0x83,
		MPSSE_DISABLE_LOOPBACK			= 0x85,
		MPSSE_SET_CLKDIV				= 0x86,
		MPSSE_FLUSH						= 0x87,
		MPSSE_DISABLE_DIV5				= 0x8a,
		MPSSE_DISABLE_3PHA 				= 0x8d,
		MPSSE_DUMMY_CLOCK_BITS			= 0x8e,
		MPSSE_DUMMY_CLOCK_BYTES			= 0x8f,
		MPSSE_DISABLE_ADAPTIVE_CLK		= 0x97,
		MPSSE_INVALID_COMMAND 			= 0xAA,		//Invalid command for resyncing
		MPSSE_INVALID_COMMAND_RESPONSE	= 0xFA
	};
};

#endif

#endif
