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
	@brief Declaration of STM32Device
 */

#ifndef STM32Device_h
#define STM32Device_h

#include "STMicroMicrocontroller.h"

#include <list>
#include <string>

/**
	@brief A STM32 microcontroller

	\ingroup libjtaghal
 */
class STM32Device
	: public STMicroMicrocontroller
	, public JtagDevice
	, public SerialNumberedDevice
{
public:
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Construction / destruction
	STM32Device(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);
	virtual ~STM32Device();

	static JtagDevice* CreateDevice(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MCU stuff

	virtual bool IsProgrammed();
	virtual void Erase();

	virtual void Program(FirmwareImage* image);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Serial numbering

	virtual bool ReadingSerialRequiresReset();
	virtual int GetSerialNumberLength();
	virtual int GetSerialNumberLengthBits();
	virtual void GetSerialNumber(unsigned char* data);
	virtual std::string GetPrettyPrintedSerialNumber();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// No NoC interfaces

	virtual bool HasRPCInterface();
	virtual bool HasDMAInterface();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helpers for chain manipulation
public:
	void SetIR(unsigned char irval)
	{ JtagDevice::SetIR(&irval, m_irlength); }

	unsigned int m_flashKB;
	unsigned int m_ramKB;

protected:
	ARMDebugPort* m_dap;

	//Serial number fields
	uint32_t m_waferX;
	uint32_t m_waferY;
	int m_waferNum;
	char m_waferLot[8];
	uint8_t m_serialRaw[12];
};

#endif
