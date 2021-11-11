/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	@brief Implementation of ARMv7MProcessor
 */

#include "jtaghal.h"
#include "DebuggableDevice.h"
#include "ARMAPBDevice.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMv7MProcessor.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMv7MProcessor::ARMv7MProcessor(DebuggerInterface* iface, ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: DebuggableDevice(iface)
	, ARMAPBDevice(ap, address, idreg)
	, m_fpb(NULL)
{

}

ARMv7MProcessor::~ARMv7MProcessor()
{
	if(m_fpb)
	{
		delete m_fpb;
		m_fpb = NULL;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug info

/**
	@brief Checks if the CPU is halted due to a fatal error
 */
bool ARMv7MProcessor::HaltedDueToUnrecoverableException()
{
	uint32_t dhcsr = ReadRegisterByIndex(DHCSR);
	if( (dhcsr >> 19) & 1)
		return true;
	return false;
}

const char* ARMv7MProcessor::GetRegisterName(ARM_V7M_CPU_REGISTERS reg)
{
	switch(reg)
	{
		case R0:
			return "R0";
		case R1:
			return "R1";
		case R2:
			return "R2";
		case R3:
			return "R3";
		case R4:
			return "R4";
		case R5:
			return "R5";
		case R6:
			return "R6";
		case R7:
			return "R7";
		case R8:
			return "R8";
		case R9:
			return "R9";
		case R10:
			return "R10";
		case R11:
			return "R11";
		case R12:
			return "R12";
		case SP:
			return "SP";
		case LR:
			return "LR";
		case DBGRA:
			return "DBGRA";
		case XPSR:
			return "xPSR";
		case MSP:
			return "MSP";
		case PSP:
			return "PSP";
		case CTRL:
			return "CTRL";
	}
	return "(invalid)";
}

uint32_t ARMv7MProcessor::ReadCPURegister(ARM_V7M_CPU_REGISTERS reg)
{
	//Request the read
	WriteRegisterByIndex(DCRSR, (0x0000 << 16) | reg);	//0000 = read, 0001 = write

	//Poll DHCSR.S_REGRDY until we're done
	while(true)
	{
		uint32_t dhcsr = ReadRegisterByIndex(DHCSR);
		if(dhcsr & 0x00010000)
			break;
		usleep(100);
	}

	//Read the actual data
	return ReadRegisterByIndex(DCRDR);
}

/**
	@brief Prints out all CPU registers
 */
void ARMv7MProcessor::PrintRegisters()
{
	ARM_V7M_CPU_REGISTERS all_regs[] =
	{
		R0, R1, R2, R3,  R4,  R5, R6,
		R7, R8, R9, R10, R11, R12,
		SP,
		LR,
		DBGRA,
		XPSR,
		MSP,
		PSP,
		CTRL
	};

	LogIndenter li;
	for(auto reg : all_regs)
		LogNotice("%10s: %08x\n", GetRegisterName(reg), ReadCPURegister(reg));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug commands

void ARMv7MProcessor::Reset()
{
	//Write AIRCR with a reboot request
	WriteMemory(0xe000ed0c, 0x05fa0004);
}

void ARMv7MProcessor::AddFlashPatchUnit(ARMFlashPatchBreakpoint* fpb)
{
	if(!m_fpb)
		m_fpb = fpb;
}

/**
	@brief Halts the CPU and enters debug state

	See ARMv7-M arch manual C1-6
 */
void ARMv7MProcessor::DebugHalt()
{
	LogTrace("Halting CPU to enter debug state...\n");
	LogIndenter li;

	//Set C_DEBUGEN and C_HALT on consecutive writes.
	WriteRegisterByIndex(DHCSR, 0xa05f0001);
	WriteRegisterByIndex(DHCSR, 0xa05f0003);

	//Poll DHCSR.S_HALT until the CPU stops
	while(true)
	{
		uint32_t dhcsr = ReadRegisterByIndex(DHCSR);
		if(dhcsr & 0x00020000)
			break;
		//LogTrace("DHCSR = %08x\n", dhcsr);
		usleep(1000);
	}
}


void ARMv7MProcessor::DebugResume()
{
	LogTrace("Resuming CPU...\n");
	LogIndenter li;

	//Leave debug state
	WriteRegisterByIndex(DHCSR, 0xa05f0000);

	//Poll DHCSR.S_HALT until the CPU is running
	while(true)
	{
		uint32_t dhcsr = ReadRegisterByIndex(DHCSR);
		if(! (dhcsr & 0x00020000) )
			break;
		//LogTrace("DHCSR = %08x\n", dhcsr);
		usleep(1000);
	}
}
