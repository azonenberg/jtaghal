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
	@brief Implementation of ARMv8Processor
 */

#include "jtaghal.h"
#include "DebuggableDevice.h"
#include "ARMAPBDevice.h"
#include "ARMDebugPort.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMv8Processor.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMv8Processor::ARMv8Processor(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: ARMAPBDevice(ap, address, idreg)
{
	LogTrace("Found ARMv8 processor at %08x, probing...\n", address);

	/*
	auto idr = ReadRegisterByIndex(MIDR_EL1);
	LogDebug("MIDR_EL1 = %08x\n", idr);
	*/

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

ARMv8Processor::~ARMv8Processor()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info
/*
void ARMv8Processor::PrintIDRegister(ARMv8DebugIDRegister did)
{
	LogVerbose("CPU rev %u variant %u\n", did.bits.revision, did.bits.variant);

	const char* arches[]=
	{
		"reserved 0",
		"ARMv6, v6 debug arch",
		"ARMv6, v6.1 debug arch",
		"ARMv8, v8 debug, full CP14",
		"ARMv8, v8 debug, only baseline cp14",
		"ARMv8, v8.1 debug",
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System memory access*/
uint32_t ARMv8Processor::ReadMemory(uint64_t addr)
{
	return m_ap->GetDebugPort()->ReadMemory(addr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugging

/**
	@brief Halts the CPU and enters debug state

	See ARMv8-A/R arch ref manual, C11-2236
 */
/*
void ARMv8Processor::EnterDebugState()
{
	//Request a halt by writing to DBGDRCR.HRQ
	LogTrace("Halting CPU to enter debug state...\n");
	LogIndenter li;
	WriteRegisterByIndex(DBGDRCR, 0x00000001);

	//Poll DBGDSCR.HALTED until it gets to 1
	while(true)
	{
		uint32_t v = ReadRegisterByIndex(DBGDSCR_EXT);
		LogTrace("DBGDSCR = %08x\n", v);
		if(v & 1)
			break;
		usleep(1000);
	}
}

void ARMv8Processor::ExitDebugState()
{
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
}
*/
