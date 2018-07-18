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
	@brief Implementation of ARMv7MProcessor
 */

#include "jtaghal.h"
#include "DebuggableDevice.h"
#include "ARMAPBDevice.h"
#include "ARMDebugPort.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMv7MProcessor.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMv7MProcessor::ARMv7MProcessor(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: ARMAPBDevice(ap, address, idreg)
{
	//LogTrace("Found ARMv7-M processor at %08x, probing...\n", address);

	/*
	//Read the Debug ID register and extract flags
	m_deviceID.word = ReadRegisterByIndex(DBGDIDR);
	m_breakpoints = m_deviceID.bits.bpoints_minus_one + 1;
	m_context_breakpoints = m_deviceID.bits.context_bpoints_minus_one + 1;
	m_watchpoints = m_deviceID.bits.wpoints_minus_one + 1;
	m_hasDevid = m_deviceID.bits.has_dbgdevid;
	m_hasSecExt = m_deviceID.bits.sec_ext;
	m_hasSecureHalt = m_deviceID.bits.sec_ext && !m_deviceID.bits.no_secure_halt;
	m_revision = m_deviceID.bits.revision;
	m_variant = m_deviceID.bits.variant;
	if(m_deviceID.bits.pcsr_legacy_addr)
		m_pcsrIndex = DBGPCSR_LEGACY;
	else
		m_pcsrIndex = DBGPCSR;

	//Verify the CPU is powered up
	uint32_t powerdown_status = ReadRegisterByIndex(DBGPRSR);
	LogTrace("DBGPRSR = %08x\n", powerdown_status);
	if(!powerdown_status & 1)
	{
		WriteRegisterByIndex(DBGPRCR, 0x00000008);	//Power up the CPU
		powerdown_status = ReadRegisterByIndex(DBGPRSR);
		LogTrace("DBGPRSR = %08x\n", powerdown_status);
	}*/
}

ARMv7MProcessor::~ARMv7MProcessor()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info
/*
void ARMv7MProcessor::PrintIDRegister(ARMv7MDebugIDRegister did)
{
	LogVerbose("CPU rev %u variant %u\n", did.bits.revision, did.bits.variant);

	const char* arches[]=
	{
		"reserved 0",
		"ARMv6, v6 debug arch",
		"ARMv6, v6.1 debug arch",
		"ARMv7M, v7M debug, full CP14",
		"ARMv7M, v7M debug, only baseline cp14",
		"ARMv7M, v7M.1 debug",
		"reserved 6",
		"reserved 7",
		"reserved 8",
		"reserved 9",
		"reserved a",
		"reserved b",
		"reserved c",
		"reserved d",
		"reserved e",
		"reserved f"
	};

	LogVerbose("Arch %s\n", arches[did.bits.debug_arch_version]);


	if(did.bits.sec_ext)
	{
		LogVerbose("Security extensions\n");
		if(did.bits.sec_ext && did.bits.no_secure_halt)
			LogDebug("    (but no secure halt)\n");
	}
	if(did.bits.pcsr_legacy_addr)
		LogVerbose("PCSR is at legacy address\n");
	if(did.bits.has_dbgdevid)
		LogVerbose("Has debug device ID\n");
	//TODO: arch version
	LogVerbose("%d breakpoints (%d with context matching)\n",
		did.bits.bpoints_minus_one+1, did.bits.context_bpoints_minus_one+1);
	LogVerbose("%d watchpoints\n", did.bits.wpoints_minus_one + 1);
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System memory access

uint32_t ARMv7MProcessor::ReadMemory(uint32_t addr)
{
	return m_ap->GetDebugPort()->ReadMemory(addr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugging

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

void ARMv7MProcessor::DumpRegisters()
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

	LogNotice("Dumping registers...\n");
	LogIndenter li;
	for(auto reg : all_regs)
		LogNotice("%8s: %08x\n", GetRegisterName(reg), ReadCPURegister(reg));
}

/**
	@brief Halts the CPU and enters debug state

	See ARMv7-M arch manual C1-6
 */
void ARMv7MProcessor::EnterDebugState()
{
	LogTrace("Halting CPU to enter debug state...\n");
	LogIndenter li;

	//Set C_DEBUGEN and C_HALT on consecutive writes
	//TODO: can we do both at once?
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


void ARMv7MProcessor::ExitDebugState()
{
	/*
	//Request a resume by writing to DBGDRCR.RRQ
	LogTrace("Restarting CPU...\n");
	LogIndenter li;
	WriteRegisterByIndex(DBGDRCR, 0x00000002);

	//Poll DBGDSCR.RESTARTED unti it gets to 1
	while(true)
	{
		uint32_t v = ReadRegisterByIndex(DBGDSCR_EXT);
		LogTrace("DBGDSCR = %08x\n", v);
		if(v & 2)
			break;
		usleep(1000);
	}
	*/
}
