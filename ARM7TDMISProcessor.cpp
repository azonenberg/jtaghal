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
	@brief Implementation of ARM7TDMISProcessor
 */

#include "jtaghal.h"
#include "DebuggableDevice.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARM7TDMISProcessor::ARM7TDMISProcessor(
		unsigned int /*partnum*/,
		unsigned int rev,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos)
	: DebuggableDevice(NULL)
	, ARMDevice(idcode, iface, pos, 4)
	, m_rev(rev)
{
	m_selectedChain = 255;
}

ARM7TDMISProcessor::~ARM7TDMISProcessor()
{

}

void ARM7TDMISProcessor::PostInitProbes(bool /*quiet*/)
{
	//WriteIceRegister(DEBUG_CTRL, 0x3f);
	//uint32_t ret = ReadIceRegister(DEBUG_CTRL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Scan chain selection

void ARM7TDMISProcessor::SelectScanChain(uint8_t n)
{
	//Don't select the chain again if it's alreadty active
	if(m_selectedChain == n)
		return;

	uint8_t inst = SCAN_N;
	SetIR(&inst);
	ScanDR(&n, NULL, 4);

	m_selectedChain = n;
}

void ARM7TDMISProcessor::WriteIceRegister(uint8_t reg, uint32_t value)
{
	//Prepare to send data to the correct chain
	SelectIceRTChain();
	uint8_t inst = INTEST;
	SetIR(&inst);

	uint8_t wdata[5] =
	{
		static_cast<uint8_t>((value >> 0) & 0xff),
		static_cast<uint8_t>((value >> 8) & 0xff),
		static_cast<uint8_t>((value >> 16) & 0xff),
		static_cast<uint8_t>((value >> 24) & 0xff),
		static_cast<uint8_t>(0x20 | reg)
	};

	ScanDR(wdata, NULL, 38);
}

uint32_t ARM7TDMISProcessor::ReadIceRegister(uint8_t reg)
{
	//Prepare to send data to the correct chain
	SelectIceRTChain();
	uint8_t inst = INTEST;
	SetIR(&inst);

	uint8_t wdata[5] =
	{
		0x00,
		0x00,
		0x00,
		0x00,
		reg
	};

	uint8_t rdata[5] = {0};
	ScanDR(wdata, NULL, 38);
	ScanDR(wdata, rdata, 38);

	return (rdata[3] << 24) | (rdata[2] << 16) | (rdata[1] << 8) | rdata[0];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

void ARM7TDMISProcessor::PrintInfo()
{
	LogVerbose("%s\n", GetDescription().c_str());
	LogIndenter li;

	/*
	PrintIDRegister(m_deviceID);

	//Read DBGDSCR to get status stuff (TODO: make struct) for this
	//uint32_t dbgdscr = ReadRegisterByIndex(DBGDSCR_EXT);
	//LogDebug("DBGDSCR = %x\n", dbgdscr);

	//Pins of interest are MIO bank 1, pins 50/51

	//Read MCTRL

	//Read PSS_IDCODE from the zynq
	//uint32_t pss_idcode = ReadMemory(0xF8000530);
	//LogDebug("pss_idcode = %08x\n", pss_idcode);

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
	*/
}

string ARM7TDMISProcessor::GetDescription()
{
	char tmp[128];
	snprintf(
		tmp,
		sizeof(tmp),
		"ARM7TDMI-S rev %d",
		m_rev
		);

	return string(tmp);
}

/*
void ARM7TDMISProcessor::PrintIDRegister(ARM7TDMISDebugIDRegister did)
{
	LogVerbose("CPU rev %u variant %u\n", did.bits.revision, did.bits.variant);

	const char* arches[]=
	{
		"reserved 0",
		"ARMv6, v6 debug arch",
		"ARMv6, v6.1 debug arch",
		"ARM7TDMIS, v7M debug, full CP14",
		"ARM7TDMIS, v7M debug, only baseline cp14",
		"ARM7TDMIS, v7M.1 debug",
		"reserved 6",
		"reserved 7",
		"reserved 8",
		"reserved 9",
		"reserved a"
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
// Debugging

void ARM7TDMISProcessor::PrintRegisters()
{
	throw JtagExceptionWrapper(
		"Register printing not implemented for this CPU yet",
		"");
}

/**
	@brief Checks if the CPU is halted due to a fatal error
 */
/*
bool ARM7TDMISProcessor::HaltedDueToUnrecoverableException()
{
	uint32_t dhcsr = ReadRegisterByIndex(DHCSR);
	if( (dhcsr >> 19) & 1)
		return true;
	return false;
}

const char* ARM7TDMISProcessor::GetRegisterName(ARM_V7M_CPU_REGISTERS reg)
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

uint32_t ARM7TDMISProcessor::ReadCPURegister(ARM_V7M_CPU_REGISTERS reg)
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

void ARM7TDMISProcessor::DumpRegisters()
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
}*/

uint32_t ARM7TDMISProcessor::ReadMemory(uint32_t /*addr*/)
{
	throw JtagExceptionWrapper(
		"Memory access not implemented for this CPU yet",
		"");

	return 0;
}

void ARM7TDMISProcessor::WriteMemory(uint32_t /*addr*/, uint32_t /*value*/)
{
	throw JtagExceptionWrapper(
		"Memory access not implemented for this CPU yet",
		"");
}

void ARM7TDMISProcessor::Reset()
{
	throw JtagExceptionWrapper(
		"Soft reset not implemented for this CPU yet",
		"");
}

/**
	@brief Halts the CPU and enters debug state
 */
void ARM7TDMISProcessor::DebugHalt()
{
	/*
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
	}*/
	throw JtagExceptionWrapper(
		"Debug halt/resume not implemented for this CPU yet",
		"");
}


void ARM7TDMISProcessor::DebugResume()
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
	}*/
	throw JtagExceptionWrapper(
		"Debug halt/resume not implemented for this CPU yet",
		"");
}

bool ARM7TDMISProcessor::IsProgrammed()
{
	return false;
}

FirmwareImage* ARM7TDMISProcessor::LoadFirmwareImage(const unsigned char* /*data*/, size_t /*len*/)
{
	return NULL;
}

void ARM7TDMISProcessor::Erase()
{

}

void ARM7TDMISProcessor::Program(FirmwareImage* /*image*/)
{

}
