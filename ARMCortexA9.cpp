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
	: ARMv7Processor(ap, address, idreg)
{

}

ARMCortexA9::~ARMCortexA9()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

void ARMCortexA9::PrintInfo()
{
	LogVerbose("%s\n", GetDescription().c_str());
	LogIndenter li;

	PrintIDRegister(m_deviceID);

	//Read DBGDSCR to get status stuff (TODO: make struct) for this
	//uint32_t dbgdscr = ReadRegisterByIndex(DBGDSCR_EXT);
	//LogDebug("DBGDSCR = %x\n", dbgdscr);

	//Pins of interest are MIO bank 1, pins 50/51

	//Read MCTRL

	//Read PSS_IDCODE from the zynq
	//uint32_t pss_idcode = ReadMemory(0xF8000530);
	//LogDebug("pss_idcode = %08x\n", pss_idcode);

	/*
	//Read MCTRL
	uint32_t mctrl = ReadMemory(0xF8007080);
	LogDebug("mctrl = %08x\n", mctrl);

	//Set MIO7 (MIO LED) to output
	m_ap->GetDebugPort()->WriteMemory(0xf800071c, 0x00000600);	//sclr.MIO_PIN_07
	m_ap->GetDebugPort()->WriteMemory(0xe000a204, 0x00000080);	//gpio.XGPIOPS_DIRM_OFFSET
	m_ap->GetDebugPort()->WriteMemory(0xe000a208, 0x00000080);	//gpio.XGPIOPS_OUTEN_OFFSET
	for(int i=0; i<10; i++)
	{
		LogDebug("toggle\n");
		m_ap->GetDebugPort()->WriteMemory(0xe000a040, 0x00000080);	//gpio.XGPIOPS_DATA_OFFSET
		usleep(500 * 1000);
		m_ap->GetDebugPort()->WriteMemory(0xe000a040, 0x00000000);	//gpio.XGPIOPS_DATA_OFFSET
		usleep(500 * 1000);
	}
	*/

	//MIO LED @ MIO7
	//MIO inputs at MIO50, 51
	//GPIO controller is at 0xe000a000
	//Input data (DATA_RO) is at +0x60 - 6c
	//

	//Read L0_SEL

	//Read the PC and dump the instruction at that address
	uint32_t pc = SampleProgramCounter();
	LogVerbose("PC = %08x\n", pc);
	//uint32_t value = ReadMemory(0xE0000000);//m_ap->ReadWord(0x80000000); //ReadMemory(0xFC000000);

	//LogDebug("    value = %08x\n", value);
}

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
