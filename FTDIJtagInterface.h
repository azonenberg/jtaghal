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
	@brief Declaration of FTDIJtagInterface
 */

#ifndef FTDIJtagInterface_h
#define FTDIJtagInterface_h

#include "JtagInterface.h"

#ifdef HAVE_FTD2XX

/**
	@brief A JTAG adapter using the FTDI chipset, accessed through libftd2xx (proprietary driver from FTDI)

	This adapter supports split scanning and queues up to 4096 bytes of command+data before comitting to hardware.

	GPIO pin mapping:

	Index	| Name
	--------|--------
		0	| GPIOL0 (ADBUS4)
		1	| GPIOL1 (ADBUS5)
		2	| GPIOL2 (ADBUS6)
		3	| GPIOL3 (ADBUS7)
		4	| GPIOH0 (ACBUS0)
		5	| GPIOH1 (ACBUS1)
		6	| GPIOH2 (ACBUS2)
		7	| GPIOH3 (ACBUS3)
		8	| GPIOH4 (ACBUS4)
		9	| GPIOH5 (ACBUS5)
		10	| GPIOH6 (ACBUS6)
		11	| GPIOH7 (ACBUS7)

	Supported layouts:

	Name | Example hardware | Pin configuration
	-----|------------------|--------------------
	hs1  | Digilent JTAG-HS1, Digilent JTAG-SMT2, azonenberg's usb-jtag-mini | ADBUS7 is active-high output enable
	hs2  | Digilent JTAG-HS2 | ADBUS7...5 are active-high output enable
	jtagkey | Amontec JTAGkey, Bus Blaster w/ JTAGkey compatible buffer | ADBUS4 is active-low output enable, ACBUS0 is TRST_N, ACBUS2 is active-low output enable for TRST_N

	\ingroup interfaces
 */
class FTDIJtagInterface : public JtagInterface
						, public GPIOInterface
{
public:
	FTDIJtagInterface(const std::string& serial, const std::string& layout);
	virtual ~FTDIJtagInterface();

	static int GetDefaultFrequency(int index);
	static bool IsJtagCapable(int index);
	static std::string GetSerialNumber(int index);
	static std::string GetDescription(int index);
	static std::string GetAPIVersion();
	static int GetInterfaceCount();

	//Setup stuff
	virtual std::string GetName();
	virtual std::string GetSerial();
	virtual std::string GetUserID();
	virtual int GetFrequency();

	//Low-level JTAG interface
	virtual void ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count);
	virtual void SendDummyClocks(size_t n);
	virtual void SendDummyClocksDeferred(size_t n);
	virtual void Commit();
	virtual bool IsSplitScanSupported();
	virtual bool ShiftDataWriteOnly(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count);
	virtual bool ShiftDataReadOnly(unsigned char* rcv_data, size_t count);

protected:
	virtual void ShiftTMS(bool tdi, const unsigned char* send_data, size_t count);

	//GPIO stuff
public:
	virtual void ReadGpioState();
	virtual void WriteGpioState();

protected:
	//Helpers for small scan operations
	void GenerateShiftPacket(
		const unsigned char* send_data, size_t count,
		bool want_read,
		bool last_tms,
		std::vector<unsigned char>& cmd_out);
	void DoReadback(unsigned char* rcv_data, size_t count);

	///@brief Buffer of data queued for the adapter, but not yet sent
	std::vector<unsigned char> m_writeBuffer;

protected:
	void SharedCtorInit(uint32_t type, const std::string& layout);

protected:
	///@brief Cached name of this adapter
	std::string m_name;

	///@brief Cached serial number of this adapter
	std::string m_serial;

	///@brief Cached user ID of this adapter
	std::string m_userid;

	///@brief Cached clock frequency of this adapter
	int m_freq;

	///@brief Libftdi interface handle
	void* m_context;

	void SyncCheck();

	void ReadData(void* data, size_t bytesToRead);
	void WriteDataRaw(const void* data, size_t bytesToWrite);
	void WriteData(const void* data, size_t bytesToWrite);
	void WriteData(unsigned char cmd);
};

#endif	//#ifdef HAVE_FTD2XX

#endif

