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
	@brief Declaration of FreescaleIMXDevice
 */

#ifndef FreescaleIMXDevice_h
#define FreescaleIMXDevice_h

#include "FreescaleMicrocontroller.h"

#include <list>
#include <string>

/**
	@brief A Freescale i.mx applications processor

	\ingroup libjtaghal
 */
class FreescaleIMXDevice	: public FreescaleMicrocontroller
{
public:

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Construction / destruction
	FreescaleIMXDevice(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);
	virtual ~FreescaleIMXDevice();

	static JtagDevice* CreateDevice(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);

	virtual void PostInitProbes();

	/*

	///5-bit-wide JTAG instructions (from BSDL file and datasheet)
	enum instructions
	{
		///Standard JTAG bypass
		INST_BYPASS				= 0x1F,

		///Read ID code
		INST_IDCODE				= 0x01,

		///Read implementation code
		INST_IMPCODE			= 0x03,

		///Selects Microchip scan chain
		INST_MTAP_SW_MCHP		= 0x04,

		///Selects EJTAG scan chain
		INST_MTAP_SW_EJTAG		= 0x05,

		///Command to Microchip virtualized JTAG
		INST_MTAP_COMMAND		= 0x07,

		///Select address register for memory ops
		INST_ADDRESS			= 0x08,

		///Select data register for memory ops
		INST_DATA				= 0x09,

		///Control register of some sort?
		INST_CONTROL			= 0x0A,

		///Selects address, data, control end to end in one DR
		INST_ALL				= 0x0B,

		///Makes the CPU trap to debugger after a reset
		INST_DEBUGBOOT			= 0x0C,

		///Boot normally after a reset
		INST_NORMALBOOT			= 0x0D,

		///Register used for moving data to/from the debug bridge
		INST_FASTDATA			= 0x0E,

		//Sample the program counter (used for profiling... not implemented?)
		INST_PCSAMPLE			= 0x14
	};
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MCU stuff

	virtual bool IsProgrammed();
	virtual void Erase();

	virtual void Program(FirmwareImage* image);

	//FreescaleIMXDeviceStatusRegister GetStatusRegister();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helpers for chain manipulation
public:
	void SetIR(unsigned char irval)
	{ JtagDevice::SetIR(&irval, m_irlength); }

	/*
protected:
	void EnterMtapMode();
	uint8_t SendMchpCommand(uint8_t cmd);
	void EnterEjtagMode();
	void EnterSerialExecMode();
	void SerialExecuteInstruction(uint32_t insn, bool first = false);
	void SerialExecuteMemoryWrite(uint32_t addr, uint32_t data);
	uint32_t SerialExecuteMemoryRead(uint32_t addr);
	EjtagControlRegister WaitForEjtagMemoryOperation(bool first = false);

	void SerialExecHelper();

	FreescaleIMXDeviceStatusRegister GetStatus();
	EjtagImplementationCodeRegister GetImpCode();

protected:

	///Device ID code
	unsigned int m_devid;

	///Stepping number
	unsigned int m_stepping;

	///Device info
	const FreescaleIMXDeviceInfo* m_devinfo;*/
};

#endif
