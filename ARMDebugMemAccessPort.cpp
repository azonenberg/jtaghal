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
	@brief Implementation of ARMDebugMemAccessPort
 */

#include "jtaghal.h"
#include "ARMAPBDevice.h"
#include "ARMCortexA9.h"
#include "ARMDebugPort.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMCoreSightDevice.h"

static const char* g_cswLenNames[]=
{
	"byte",
	"halfword",
	"word"
};

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMDebugMemAccessPort::ARMDebugMemAccessPort(ARMDebugPort* dp, uint8_t apnum, ARMDebugPortIDRegister id)
	: ARMDebugAccessPort(dp, apnum, id)
	, m_debugBusIsDedicated(false)
	, m_hasDebugRom(true)
{
	if(m_daptype >= DAP_INVALID)
	{
		throw JtagExceptionWrapper(
			"Invalid DAP type",
			"");
	}

	//If the access size is not 32-bit, make it 32-bit
	ARMDebugMemAPControlStatusWord csw = GetStatusRegister();
	if(csw.bits.size != ACCESS_WORD)
	{
		csw.bits.size = ACCESS_WORD;
		m_dp->APRegisterWrite(m_apnum, ARMDebugPort::REG_MEM_CSW, csw.word);
	}
}

void ARMDebugMemAccessPort::Initialize()
{
	//If we're enabled, try to load the ROM (if any)
	ARMDebugMemAPControlStatusWord csw = GetStatusRegister();
	if(csw.bits.enable)
	{
		LogTrace("Searching for debug ROMs in AP %d (%s)...\n", m_apnum, (m_daptype == DAP_AHB) ? "AHB" : "APB");
		LogIndenter li;
		FindRootRomTable();
		if(m_hasDebugRom)
			LoadROMTable(m_debugBaseAddress);
		else
			LogTrace("No debug ROM found\n");
	}

	//Looks like the AHB DAP for Zynq is used for main system memory access
	//and the APB DAP is used for CoreSight stuff
	//See UG585 page 718
}

ARMDebugMemAccessPort::~ARMDebugMemAccessPort()
{
	for(auto x : m_debugDevices)
		delete x;
	m_debugDevices.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

bool ARMDebugMemAccessPort::IsEnabled()
{
	return GetStatusRegister().bits.enable;
}

void ARMDebugMemAccessPort::PrintStatusRegister()
{
	ARMDebugMemAPControlStatusWord csw = GetStatusRegister();
	LogIndenter li;
	LogDebug("Status register for AP %u:\n", m_apnum);
	LogIndenter li2;
	if(csw.bits.size >= ACCESS_INVALID)
		LogDebug("Size               : UNDEFINED\n");
	else
		LogDebug("Size               : %s\n", g_cswLenNames[csw.bits.size]);
	LogDebug("Auto inc           : %u\n", csw.bits.auto_increment);
	LogDebug("Enable             : %u\n", csw.bits.enable);
	LogDebug("Busy               : %u\n", csw.bits.busy);
	LogDebug("Mode               : %u\n", csw.bits.mode);
	LogDebug("Secure debug       : %u\n", csw.bits.secure_priv_debug);
	LogDebug("Bus protection     : %u\n", csw.bits.bus_protect);
	LogDebug("Nonsecure transfer : %u\n", csw.bits.nonsecure_transfer);
}

string ARMDebugMemAccessPort::GetDescription()
{
	return "";
}

ARMDebugMemAPControlStatusWord ARMDebugMemAccessPort::GetStatusRegister()
{
	ARMDebugMemAPControlStatusWord csw;
	csw.word = m_dp->APRegisterRead(m_apnum, ARMDebugPort::REG_MEM_CSW);
	return csw;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory access

uint32_t ARMDebugMemAccessPort::ReadWord(uint32_t addr)
{
	//Write the address
	m_dp->APRegisterWrite(m_apnum, ARMDebugPort::REG_MEM_TAR, addr);

	//Read the data back
	return m_dp->APRegisterRead(m_apnum, ARMDebugPort::REG_MEM_DRW);
}

void ARMDebugMemAccessPort::WriteWord(uint32_t addr, uint32_t value)
{
	//Write the address
	m_dp->APRegisterWrite(m_apnum, ARMDebugPort::REG_MEM_TAR, addr);

	//Write data
	m_dp->APRegisterWrite(m_apnum, ARMDebugPort::REG_MEM_DRW, value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device enumeration

void ARMDebugMemAccessPort::FindRootRomTable()
{
	//Get the base address of the debug base
	uint32_t debug_base = m_dp->APRegisterRead(m_apnum, ARMDebugPort::REG_MEM_BASE);

	//If the debug base is 0xffffffff, then we can't do anything (no ROM)
	if(debug_base == 0xffffffff)
	{
		m_hasDebugRom = false;
		return;
	}

	//If bit 1 is set, then it's an ADIv5.1 address.
	//Mask off the low bits
	else if(debug_base & 2)
	{
		m_debugBaseAddress = debug_base & 0xfffff000;

		//If LSB not set, there's no ROM
		if(! (debug_base & 1) )
		{
			m_hasDebugRom = false;
			return;
		}
	}

	//Nope, it's a legacy address (no masking)
	else
		m_debugBaseAddress = debug_base;
}

void ARMDebugMemAccessPort::LoadROMTable(uint32_t baseAddress)
{
	LogTrace("Loading ROM table at address %08x\n", baseAddress);
	LogIndenter li;

	//Read ROM table entries until we get to an invalid one
	for(int i=0; i<960; i++)
	{
		//Read the next entry and stop if it's a terminator
		uint32_t entry = ReadWord(baseAddress + i*4);
		if(entry == 0)
			break;

		//If we hit 959 and it's not a terminator, something is wrong - should not be this many entries
		if(i == 959)
		{
			throw JtagExceptionWrapper(
				"Expected a terminator at end of ROM table, but none found",
				"");
		}

		//If it's not present, ignore it
		if(! (entry & 1) )
			continue;

		//If it's not a 32-bit entry fail (not yet implemented)
		if(! (entry & 2) )
		{
			throw JtagExceptionWrapper(
				"8-bit ROM tables not implemented",
				"");
		}

		//Calculate the actual address of this node
		uint32_t offset = entry >> 12;
		uint32_t address = (offset << 12) + baseAddress;
		if(entry & 0x80000000)
			address = baseAddress - (~(offset << 12) + 1);

		//Walk this table entry
		uint32_t compid_raw[4];
		for(int i=0; i<4; i++)
			compid_raw[i] = ReadWord(address + 0xff0 + 4*i);
		uint32_t compid =
			(compid_raw[3] << 24) |
			(compid_raw[2] << 16) |
			(compid_raw[1] << 8) |
			compid_raw[0];

		//Verify the mandatory component ID bits are in the right spots
		if( (compid & 0xffff0fff) != (0xb105000d) )
		{
			LogError("Invalid ROM table ID (wrong preamble), got %08x and expected something close to 0xb105000d\n",
				compid);
			continue;
		}

		//Figure out what it means
		unsigned int ccls = (compid >> 12) & 0xf;

		switch(ccls)
		{
			//Process CoreSight blocks
			case CLASS_CORESIGHT:
				ProcessDebugBlock(address);
				break;

			//Process "generic IP" blocks (ignore them)
			case CLASS_GENERIC_IP:
				LogTrace("Found generic IP block at %08x, ignoring\n", address);
				break;

			//Additional ROM table
			case CLASS_ROMTABLE:
				{
					LogTrace("Found extra ROM table at %08x, loading\n", address);
					LogIndenter li;

					//If the ROM table is a pointer to US, ignore it!
					if(address == baseAddress)
					{
						LogTrace("Actually it's a pointer back to the same table we're in. Ignoring it.\n");
						continue;
					}

					LoadROMTable(address);
				}
				break;

			//Don't know what to do with anything else
			default:
				LogWarning("Found unknown component class %x, skipping\n", ccls);
				throw JtagExceptionWrapper(
					"Unknown debug component class",
					"");
				break;
		}
	}
}

/**
	@brief Reads the ROM table for a debug block to figure out what's going on
 */
void ARMDebugMemAccessPort::ProcessDebugBlock(uint32_t base_address)
{
	//Read the rest of the ROM table header
	uint64_t periphid_raw[8];
	for(int i=0; i<4; i++)
		periphid_raw[i] = ReadWord(base_address + 0xfe0 + 4*i);
	for(int i=0; i<4; i++)
		periphid_raw[i+4] = ReadWord(base_address + 0xfd0 + 4*i);
	uint32_t memtype = ReadWord(base_address + 0xfcc);

	//See if the mem is dedicated or not
	m_debugBusIsDedicated = (memtype & 1) ? false : true;

	//Merge everything into a single peripheral ID register
	ARMDebugPeripheralIDRegister reg;
	reg.word =
		(periphid_raw[7] << 56) |
		(periphid_raw[6] << 48) |
		(periphid_raw[5] << 40) |
		(periphid_raw[4] << 32) |
		(periphid_raw[3] << 24) |
		(periphid_raw[2] << 16) |
		(periphid_raw[1] << 8)  |
		(periphid_raw[0] << 0);

	//TODO: handle legacy ASCII identity code
	if(!reg.bits.jep106_used)
	{
		throw JtagExceptionWrapper(
			"Bad ID in ROM (no JEP106 code)",
			"");
	}

	unsigned int blockcount = (1 << reg.bits.log_4k_blocks);
	LogTrace("Found debug component at %08x (rev %u.%u.%u, %u 4KB pages)\n",
		base_address, reg.bits.revnum, reg.bits.cust_mod, reg.bits.revand, blockcount);
	LogIndenter li;

	//Check IDCODE for Xilinx
	if( (reg.bits.jep106_cont == 0x00) && (reg.bits.jep106_id == 0x49) )
	{

		//See if it's a FTM (Zynq)
		//See Xilinx UG585 page 729 table 28-5
		if(reg.bits.partnum == 0x001)
		{
			LogTrace("Found Xilinx Fabric Trace Monitor, not yet implemented\n");
		}

		//unknown, skip it
		else
			LogWarning("Found unknown Xilinx CoreSight device (part number 0x%x)\n", reg.bits.partnum);

	}

	//Check IDCODE for ARM
	else if( (reg.bits.jep106_cont == 0x04) && (reg.bits.jep106_id == 0x3b) )
	{
		//Handle fully supported devices first, then just make a generic CoreSight device for the rest
		switch(reg.bits.partnum)
		{
			//Cortex-A9
			case 0xC09:
			{
				ARMCortexA9* cortex = new ARMCortexA9(this, base_address, reg.bits);
				m_dp->AddTarget(cortex);
				m_debugDevices.push_back(cortex);
			}
			break;

			//Unknown
			default:
			{
				LogDebug("Unknown CoreSight device\n");
				ARMCoreSightDevice* dev = new ARMCoreSightDevice(this, base_address, reg.bits);
				m_debugDevices.push_back(dev);
			}
		}
	}

	//Check IDCODE for Switchcore (?)
	else if( (reg.bits.jep106_cont == 0x03) && (reg.bits.jep106_id == 0x09) )
	{
		//LogWarning("Unknown Switchcore device (part number 0x%x)\n", reg.bits.partnum);
	}

	//Unknown vendor
	else
	{
		LogWarning("Unknown device (JEP106 %u:%x, part number 0x%x)\n",
			reg.bits.jep106_cont, reg.bits.jep106_id, reg.bits.partnum);
	}
}
