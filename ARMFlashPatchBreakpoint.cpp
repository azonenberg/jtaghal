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
	@brief ARM Cortex-M Flash Patch/Breakpoint
 */
#include "jtaghal.h"
#include "ARMAPBDevice.h"
#include "ARMFlashPatchBreakpoint.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMFlashPatchBreakpoint::ARMFlashPatchBreakpoint(
	ARMv7MProcessor* cpu,
	ARMDebugMemAccessPort* ap,
	uint32_t address,
	ARMDebugPeripheralIDRegisterBits idreg)
	: ARMCoreSightDevice(ap, address, idreg)
	, m_cpu(cpu)
{
	//Assume RAM is at 0x20000000 for now.
	//TODO: is this always true for Cortex-M's?
	m_sramBase = 0x20000000;

	//Read the control register to get read-only config
	uint32_t ctrl = ReadRegisterByIndex(FP_CTRL);
	m_literalComparators = (ctrl >> 8) & 0xf;
	m_codeComparators = ( (ctrl >> 4) & 0xf ) | ( (ctrl >> 8) & 0xf0 );

	//Read the remap register to see if we can remap or just to breakpoints
	uint32_t remap = ReadRegisterByIndex(FP_REMAP);
	m_canRemap = false;
	if(remap & 0x20000000)
		m_canRemap = true;

	//Pull volatile config
	ProbeStatusRegisters();
}

ARMFlashPatchBreakpoint::~ARMFlashPatchBreakpoint()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pretty printing

void ARMFlashPatchBreakpoint::ProbeStatusRegisters()
{
	uint32_t ctrl = ReadRegisterByIndex(FP_CTRL);
	m_enabled = (ctrl & 1) ? true : false;

	uint32_t remap = ReadRegisterByIndex(FP_REMAP);
	m_tableBase = (remap & 0x1FFFFFE0) | m_sramBase;
}

void ARMFlashPatchBreakpoint::PrintInfo()
{
	ProbeStatusRegisters();

	//Heading
	LogNotice("%s rev %d.%d.%d\n",
		GetDescription().c_str(),
		m_idreg.revnum, m_idreg.cust_mod, m_idreg.revand);
	LogIndenter li;

	//LogNotice("Attached to CPU: %s\n", m_cpu->GetDescription().c_str());

	//Summary
	if(m_enabled)
		LogNotice("FPB enabled\n");
	else
		LogNotice("FPB disabled\n");

	if(m_canRemap)
	{
		LogNotice("Remap supported\n");
		LogNotice("Remap table is at 0x%08x\n", m_tableBase);
	}
	else
		LogNotice("Remap not supported, breakpoints only\n");

	//Code
	LogNotice("%d code comparators\n", m_codeComparators);
	for(uint32_t i=0; i<m_codeComparators; i++)
	{
		LogIndenter li2;
	}

	//Literals
	LogNotice("%d literal comparators\n", m_literalComparators);
}

string ARMFlashPatchBreakpoint::GetDescription()
{
	switch(m_idreg.partnum)
	{
		case 0x003:
			return "Cortex-M4 Flash Patch/Breakpoint";

		default:
			LogWarning("Unknown ARM FPB device (part number 0x%x)\n", m_idreg.partnum);
			return "unknown FPB device";
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fun commands that actually do stuff

void ARMFlashPatchBreakpoint::Enable()
{
	uint32_t ctrl = ReadRegisterByIndex(FP_CTRL);
	ctrl |= 1;		//set ENABLE
	ctrl |= 2;		//must also set KEY for writes to take effect
	WriteRegisterByIndex(FP_CTRL, ctrl);

	m_enabled = true;
}

void ARMFlashPatchBreakpoint::Disable()
{
	uint32_t ctrl = ReadRegisterByIndex(FP_CTRL);
	ctrl &= ~1;		//clear ENABLE
	ctrl |= 2;		//must also set KEY for writes to take effect
	WriteRegisterByIndex(FP_CTRL, ctrl);

	m_enabled = false;
}

void ARMFlashPatchBreakpoint::SetRemapTableBase(uint32_t base)
{
	//TODO: Sanity check that address is within SRAM region

	//Align to 8 word boundary
	base &= 0x1FFFFFE0;
	WriteRegisterByIndex(FP_REMAP, base);

	//Save the actual address including SRAM offset
	m_tableBase = base | m_sramBase;
}

void ARMFlashPatchBreakpoint::RemapFlashWord(uint32_t slot, uint32_t flashAddress, uint32_t newValue)
{
	if(flashAddress & 3)
	{
		LogWarning("ARM FPB requres word-aligned address\n");
		return;
	}

	//Write the patched data to the remap table
	m_ap->WriteWord(m_tableBase + slot*4, newValue);

	//Write to FP_COMPx to enable the comparator. Mask off unused address bits.
	flashAddress &= 0x1ffffffc;
	WriteRegisterByIndex(GetCodeComparatorIndex(slot), flashAddress | 1);
}
