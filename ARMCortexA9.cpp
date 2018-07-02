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
	@brief Implementation of ARMCortexA9
 */

#include "jtaghal.h"
#include "DebuggableDevice.h"
#include "ARMAPBDevice.h"
#include "ARMDebugPort.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMCortexA9.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMCortexA9::ARMCortexA9(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: ARMAPBDevice(ap, address, idreg)
{
	LogDebug("Cortex-A9 initializing at %08x\n", address);

	//Read the Debug ID register and extract flags
	ARMv7DebugIDRegister did;
	did.word = ReadRegisterByIndex(DBGDIDR);
	m_breakpoints = did.bits.bpoints_minus_one + 1;
	m_context_breakpoints = did.bits.context_bpoints_minus_one + 1;
	m_watchpoints = did.bits.wpoints_minus_one + 1;
	m_hasDevid = did.bits.has_dbgdevid;
	m_hasSecExt = did.bits.sec_ext;
	m_hasSecureHalt = did.bits.sec_ext && !did.bits.no_secure_halt;
	m_revision = did.bits.revision;
	m_variant = did.bits.variant;
	if(did.bits.pcsr_legacy_addr)
		m_pcsrIndex = DBGPCSR_LEGACY;
	else
		m_pcsrIndex = DBGPCSR;

	//Print it out
	PrintIDRegister(did);

	//DBGDSMCR turn off MMU

	//TODO: Write to DBGDRCR to halt the CPU (C11.11.17)

	//TODO: Write to DBGDSCCR to force write-through cache (C11.11.19)

	//DBGDEVID[3:0] 2226

	//Read DBGDSCR to get status stuff (TODO: make struct) for this
	//uint32_t dbgdscr = ReadRegisterByIndex(DBGDSCR_EXT);
	//LogDebug("DBGDSCR = %x\n", dbgdscr);

	//Pins of interest are MIO bank 1, pins 50/51

	//Read PSS_IDCODE
	//uint32_t pss_idcode = ReadMemory(0xF8000530);
	//uint32_t pss_idcode = m_ap->ReadWord(0xF8000530);
	//LogDebug("pss_idcode = %08x\n", pss_idcode);

	//MIO LED @ MIO7
	//MIO inputs at MIO50, 51
	//GPIO controller is at 0xe0000000
	//Read DIRM to see what
	//Read DATA_RO?

	//Read L0_SEL


	//Read the PC and dump the instruction at that address
	uint32_t pc = SampleProgramCounter();
	LogDebug("PC = %08x\n", pc);
	//uint32_t value = ReadMemory(0xE0000000);//m_ap->ReadWord(0x80000000); //ReadMemory(0xFC000000);

	//LogDebug("    value = %08x\n", value);
}

ARMCortexA9::~ARMCortexA9()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

string ARMCortexA9::GetDescription()
{
	char tmp[128];
	snprintf(
		tmp,
		sizeof(tmp),
		"ARM Cortex-A9 rev %d mod %d stepping %d",
		m_idreg.revnum,
		m_idreg.cust_mod,
		m_idreg.revand
		);

	return string(tmp);
}

void ARMCortexA9::PrintIDRegister(ARMv7DebugIDRegister did)
{
	LogDebug("    Rev %u variant %u\n", did.bits.revision, did.bits.variant);

	const char* arches[]=
	{
		"reserved 0",
		"ARMv6, v6 debug arch",
		"ARMv6, v6.1 debug arch",
		"ARMv7, v7 debug, full CP14",
		"ARMv7, v7 debug, only baseline cp14",
		"ARMv7, v7.1 debug",
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

	LogDebug("    Arch %s\n", arches[did.bits.debug_arch_version]);


	if(did.bits.sec_ext)
	{
		LogDebug("    Security extensions\n");
		if(did.bits.sec_ext && did.bits.no_secure_halt)
			LogDebug("        (but no secure halt)\n");
	}
	if(did.bits.pcsr_legacy_addr)
		LogDebug("    PCSR is at legacy address\n");
	if(did.bits.has_dbgdevid)
		LogDebug("    Has debug device ID\n");
	//TODO: arch version
	LogDebug("    %d breakpoints (%d with context matching)\n",
		did.bits.bpoints_minus_one+1, did.bits.context_bpoints_minus_one+1);
	LogDebug("    %d watchpoints\n", did.bits.wpoints_minus_one + 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System memory access

uint32_t ARMCortexA9::ReadMemory(uint32_t addr)
{
	return m_ap->GetDebugPort()->ReadMemory(addr);
}