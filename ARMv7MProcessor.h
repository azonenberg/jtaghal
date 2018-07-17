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
	@brief Declaration of ARMv7MProcessor
 */

#ifndef ARMv7MProcessor_h
#define ARMv7MProcessor_h

class DebuggableDevice;
class ARMAPBDevice;

/**
	@brief An ARM Cortex-M CPU core supporting the ARMv7-M architecture, as seen over a CoreSight APB bus

	\ingroup libjtaghal
 */
class ARMv7MProcessor 	: public DebuggableDevice
						, public ARMAPBDevice
{
public:
	ARMv7MProcessor(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg);
	virtual ~ARMv7MProcessor();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Registers

	//register numbers, multiply by 4 to get address
	enum ARM_V7M_DEBUG_REGISTERS
	{
		//DBGDIDR			= 0,
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription() =0;
	virtual void PrintInfo() =0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory access via the default (AHB) MEM-AP

	virtual uint32_t ReadMemory(uint32_t addr);

protected:
	//void PrintIDRegister(ARMv7MDebugIDRegister did);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debugger control

	void EnterDebugState();
	void ExitDebugState();

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ID register state etc

	unsigned int m_breakpoints;
	unsigned int m_context_breakpoints;
	unsigned int m_watchpoints;
	bool m_hasDevid;
	bool m_hasSecExt;
	bool m_hasSecureHalt;
	unsigned int m_revision;
	unsigned int m_variant;
	//TODO: arch version

	ARMv7MDebugIDRegister m_deviceID;

	///Device-dependent address of the program counter sample register (PCSR)
	ARM_V7_DEBUG_REGISTERS m_pcsrIndex;
	*/
};

#endif


