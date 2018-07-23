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
class FreescaleIMXSmartDMA;

#include <list>
#include <string>

enum ImxDeviceIDs
{
	IMX_6_SOLO		= 0x891B,
	IMX_6_DUAL_LITE	= 0x891A
};

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

	///5-bit-wide JTAG instructions (from datasheet table 56-3)
	enum instructions
	{
		///Read ID code
		INST_IDCODE				= 0x00,

		///Boundary scan stuff
		INST_SAMPLE_PRELOAD		= 0x01,
		INST_EXTEST				= 0x02,
		INST_HIZ				= 0x03,
		INST_EXTEST_PULSE		= 0x08,
		INST_EXTEST_TRAIN		= 0x09,

		//Debug security
		INST_EXTRADEBUG			= 0x04,
		INST_ENTER_DEBUG		= 0x05,
		INST_SECURE_CHALL		= 0x0c,
		INST_SECURE_RESP		= 0x0d,

		//TAP selection mode
		INST_TAP_SELECT			= 0x07,

		//Not used
		INST_BYPASS				= 0x1f
	};

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
	*/
protected:

	///Device ID code
	ImxDeviceIDs m_devid;

	///Stepping number
	unsigned int m_stepping;

	//Pointers to our other JTAG devices
	ARMDebugPort* m_dap;
	FreescaleIMXSmartDMA* m_sdma;
};

#endif
