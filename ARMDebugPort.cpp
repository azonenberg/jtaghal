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
	@brief Implementation of ARMDebugPort
 */

#include "jtaghal.h"
#include "ARMDebugPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMAPBDevice.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMDebugPort::ARMDebugPort(
	unsigned int partnum,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
	: ARMDevice(idcode, iface, pos, 4)
	, m_rev(rev)
	, m_partnum(partnum)
{
	//No Mem-AP for now
	m_defaultMemAP 		= NULL;
	m_defaultRegisterAP	= NULL;
}

void ARMDebugPort::PostInitProbes(bool quiet)
{
	//Turn on the debug stuff
	EnableDebugging();

	//Figure out how many APs we have
	LogTrace("Found ARM JTAG-DP, probing...\n");
	LogIndenter li;
	LogTrace("Searching for APs...\n");
	uint8_t nap = 0;
	for(; nap<255; nap++)
	{
		LogIndenter li;
		ARMDebugPortIDRegister idr;
		idr.word = APRegisterRead(nap, REG_IDR);
		if(idr.word == 0)
			break;

		/*
		LogDebug("Found AP with ID 0x%08x\n", idr.word);
		{
			LogIndenter li;
			LogDebug("Type:         %x\n", idr.bits.type);
			LogDebug("Variant:      %x\n", idr.bits.variant);
			LogDebug("Reserved:     %x\n", idr.bits.reserved_zero);
			LogDebug("Is mem AP:    %x\n", idr.bits.is_mem_ap);
			LogDebug("Identity:     %x\n", idr.bits.identity);
			LogDebug("Continuation: %x\n", idr.bits.continuation);
			LogDebug("Revision:     %x\n", idr.bits.revision);
		}
		*/

		//Sanity check that it's a valid ARM DAP
		if( (idr.bits.continuation != 0x4) || (idr.bits.identity != 0x3b) )
		{
			throw JtagExceptionWrapper(
				"Unknown ARM debug device (not designed by ARM Ltd)",
				"");
		}

		//If it's a JTAG-AP, skip it for now
		//(this seems to be fpga config?)
		if(!idr.bits.is_mem_ap)
		{
			LogTrace("Found JTAG-AP rev %d at index %d\n", idr.bits.revision, nap);
			LogIndenter li;
			LogTrace("Not supported yet, ignoring\n");
			continue;
		}

		//Create a new MEM-AP
		else
		{
			ARMDebugMemAccessPort* ap = new ARMDebugMemAccessPort(this, nap, idr);
			m_aps[nap] = ap;

			if(ap->GetBusType() == ARMDebugAccessPort::DAP_AHB)
				LogTrace("Found AHB MEM-AP rev %d at index %d\n", idr.bits.revision, nap);
			else if(ap->GetBusType() == ARMDebugAccessPort::DAP_APB)
				LogTrace("Found APB MEM-AP rev %d at index %d\n", idr.bits.revision, nap);
			else if(ap->GetBusType() == ARMDebugAccessPort::DAP_AXI)
				LogTrace("Found AXI MEM-AP rev %d at index %d\n", idr.bits.revision, nap);

			//If it's an AHB Mem-AP, and we don't have a default Mem-AP, this one is probably RAM.
			//Use it as our default AP.
			if( (ap->GetBusType() == ARMDebugAccessPort::DAP_AHB) && (m_defaultMemAP == NULL) )
			{
				LogIndenter li;
				LogTrace("Using as default RAM Mem-AP\n");
				m_defaultMemAP = ap;
			}

			//If it's an AXI Mem-AP, and we don't have a default Mem-AP, this one is probably RAM.
			//Use it as our default AP.
			if( (ap->GetBusType() == ARMDebugAccessPort::DAP_AXI) && (m_defaultMemAP == NULL) )
			{
				LogIndenter li;
				LogTrace("Using as default RAM Mem-AP\n");
				m_defaultMemAP = ap;
			}

			//If it's an APB Mem-AP, and we don't have a default Mem-AP, this one is probably CoreSight debug registers.
			//Use it as our default AP.
			if( (ap->GetBusType() == ARMDebugAccessPort::DAP_APB) && (m_defaultRegisterAP == NULL) )
			{
				LogIndenter li;
				LogTrace("Using as default CoreSight Mem-AP\n");
				m_defaultRegisterAP = ap;
			}
		}
	}

	//If we have an AHB Mem-AP but not a CoreSight APB Mem-AP, use the AHB bus for CoreSight stuff too
	if(m_defaultMemAP && !m_defaultRegisterAP)
		m_defaultRegisterAP = m_defaultMemAP;

	//Initialize each of the APs once they are all open
	LogTrace("Initializing APs...\n");
	{
		LogIndenter li;
		for(auto it : m_aps)
			it.second->Initialize();
	}
}

ARMDebugPort::~ARMDebugPort()
{
	for(auto x : m_aps)
		delete x.second;
	m_aps.clear();
}

JtagDevice* ARMDebugPort::CreateDevice(
		unsigned int partnum,
		unsigned int rev,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos)
{
	switch(partnum)
	{
	case IDCODE_ARM_DAP_JTAG:
		return new ARMDebugPort(partnum, rev, idcode, iface, pos);

	default:
		LogError("ARMDebugPort::CreateDevice(partnum=%x, rev=%x, idcode=%08x)\n", partnum, rev, idcode);

		throw JtagExceptionWrapper(
			"Unknown ARM debug device (ID code not in database)",
			"");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization

void ARMDebugPort::EnableDebugging()
{
	//Clear any stale errors
	ARMDebugPortStatusRegister stat = GetStatusRegister();
	if(stat.bits.sticky_err)
	{
		LogDebug("    Error bit is set, clearing\n");
		ClearStatusRegisterErrors();
	}

	//Power up the system
	stat.word = 0;
	stat.bits.sys_pwrup_req = 1;
	stat.bits.debug_pwrup_req = 1;
	DPRegisterWrite(REG_CTRL_STAT, stat.word);

	//Verify it powered up OK
	stat = GetStatusRegister();
	if(!stat.bits.sys_pwrup_ack)
	{
		throw JtagExceptionWrapper(
			"Failed to get ACK to system powerup request",
			"");
	}
	if(!stat.bits.debug_pwrup_ack)
	{
		throw JtagExceptionWrapper(
			"Failed to get ACK to debug powerup request",
			"");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory access

///Read a single 32-bit word of memory (TODO support smaller sizes)
uint32_t ARMDebugPort::ReadMemory(uint32_t address)
{
	//Sanity check
	if(m_defaultMemAP == NULL)
	{
		throw JtagExceptionWrapper(
			"Cannot read memory because there is no AHB MEM-AP",
			"");
	}

	//Use the default Mem-AP to read it
	return m_defaultMemAP->ReadWord(address);
}

///Writes a single 32-bit word of memory (TODO support smaller sizes)
void ARMDebugPort::WriteMemory(uint32_t address, uint32_t value)
{
	//Sanity check
	if(m_defaultMemAP == NULL)
	{
		throw JtagExceptionWrapper(
			"Cannot write memory because there is no AHB MEM-AP",
			"");
	}

	//Use the default Mem-AP to write it
	m_defaultMemAP->WriteWord(address, value);
}

uint32_t ARMDebugPort::ReadDebugRegister(uint32_t address)
{
	//Sanity check
	if(m_defaultRegisterAP == NULL)
	{
		throw JtagExceptionWrapper(
			"Cannot read register because there is no APB MEM-AP",
			"");
	}

	//Use the default Mem-AP to read it
	return m_defaultRegisterAP->ReadWord(address);
}

///Writes a single 32-bit word of memory
void ARMDebugPort::WriteDebugRegister(uint32_t address, uint32_t value)
{
	//Sanity check
	if(m_defaultRegisterAP == NULL)
	{
		throw JtagExceptionWrapper(
			"Cannot write register because there is no APB MEM-AP",
			"");
	}

	//Use the default Mem-AP to write it
	m_defaultRegisterAP->WriteWord(address, value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device info

std::string ARMDebugPort::GetDescription()
{
	string devname = "(unknown)";

	switch(m_partnum)
	{
	case IDCODE_ARM_DAP_JTAG:
		devname = "JTAG-DP";
		break;

	default:
		throw JtagExceptionWrapper(
			"Unknown ARM device (ID code not in database)",
			"");
	}

	char srev[16];
	snprintf(srev, 15, "%u", m_rev);

	return string("ARM ") + devname + " rev " + srev;
}

void ARMDebugPort::PrintInfo()
{
	//Device descriptor
	LogNotice("%2d: %s\n", (int)m_pos, GetDescription().c_str());
	LogIndenter li;

	//Print the system MEM-AP for now
	if(m_defaultMemAP != NULL)
	{
		LogVerbose(
			"System memory bus (AHB): MEM-AP rev %d (at index %d)\n",
			m_defaultMemAP->GetVersion(),
			m_defaultMemAP->GetAPNumber()
			);
	}

	//Walk the APB MEM-AP and see what debug cores we have
	if(m_defaultRegisterAP != NULL)
	{
		if(m_defaultRegisterAP != m_defaultMemAP)
		{
			LogVerbose(
				"CoreSight bus (APB):     MEM-AP rev %d (at index %d)\n",
				m_defaultRegisterAP->GetVersion(),
				m_defaultRegisterAP->GetAPNumber()
				);
		}
		else
			LogVerbose("No dedicated CoreSight APB bus found, using system memory bus\n");

	}

	LogIndenter li2;
	for(size_t i=0; i<m_defaultRegisterAP->GetDeviceCount(); i++)
		m_defaultRegisterAP->GetDevice(i)->PrintInfo();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Chain querying

void ARMDebugPort::PrintStatusRegister(ARMDebugPortStatusRegister reg, bool children)
{
	LogDebug("DAP status: %08x\n", reg.word);
	LogDebug("    Sys pwrup ack:     %u\n", reg.bits.sys_pwrup_ack);
	LogDebug("    Sys pwrup req:     %u\n", reg.bits.sys_pwrup_req);
	LogDebug("    Debug pwrup ack:   %u\n", reg.bits.debug_pwrup_ack);
	LogDebug("    Debug pwrup req:   %u\n", reg.bits.debug_pwrup_req);
	LogDebug("    Debug reset ack:   %u\n", reg.bits.debug_reset_ack);
	LogDebug("    Debug reset req:   %u\n", reg.bits.debug_reset_req);
	//ignore reserved
	LogDebug("    Transaction count: %u\n", reg.bits.trans_count);
	LogDebug("    Mask lane:         %u\n", reg.bits.mask_lane);
	//LogDebug("    Write error:       %u\n", reg.bits.wr_data_err);
	//LogDebug("    Read OK:           %u\n", reg.bits.read_ok);
	LogDebug("    Sticky err:        %u\n", reg.bits.sticky_err);
	LogDebug("    Sticky compare:    %u\n", reg.bits.sticky_compare);
	LogDebug("    Transfer mode:     %u\n", reg.bits.transfer_mode);
	LogDebug("    Sticky overrun:    %u\n", reg.bits.sticky_overrun);
	LogDebug("    Sticky overrun en: %u\n", reg.bits.sticky_overrun_en);

	if(children)
	{
		for(auto x : m_aps)
			x.second->PrintStatusRegister();
	}
}

/**
	@brief Gets the status register
 */
ARMDebugPortStatusRegister ARMDebugPort::GetStatusRegister()
{
	ARMDebugPortStatusRegister stat;
	stat.word = DPRegisterRead(REG_CTRL_STAT);
	return stat;
}

void ARMDebugPort::ClearStatusRegisterErrors()
{
	ARMDebugPortStatusRegister stat;
	stat.word = DPRegisterRead(REG_CTRL_STAT);
	stat.bits.sticky_err = 1;
	DPRegisterWrite(REG_CTRL_STAT, stat.word);
}

/**
	@brief Reads from an AP register

	@param ap			The number of the AP to access
	@param addr			The ID of the AP register to read

	@return The value read
 */
uint32_t ARMDebugPort::APRegisterRead(uint8_t ap, ApReg addr)
{
	//Set the high bits of the address as the current bank
	uint32_t select = (ap << 24) | (addr & 0xf0);
	DPRegisterWrite(REG_AP_SELECT,  select );

	//Do the read
	SetIR(INST_APACC);

	uint32_t data_out;

	//Poll until we get a good read back
	int i = 0;
	int nmax = 50;
	for(; i<nmax; i++)
	{
		//Send the 3-bit A / RnW field to request the read
		uint8_t addr_flags = ((addr & 0x0c) >> 1) | OP_READ;
		uint8_t txd[5] = {0};
		for(int i=0; i<3; i++)
			PokeBit(txd, i, PeekBit(&addr_flags, i));
		uint8_t rxd[5];
		ScanDR(txd, rxd, 35);
		uint8_t ack_out = 0;
		for(int i=0; i<3; i++)
			PokeBit(&ack_out, i, PeekBit(rxd, i));

		//If we got data, crunch it.
		//Note that the first poll can never return the data since we haven't even done the read request yet!
		if((ack_out == OK_OR_FAULT) && (i > 0) )
		{
			for(int i=0; i<32; i++)
				PokeBit((unsigned char*)&data_out, i, PeekBit(rxd, i+3));
			break;
		}

		//No go? Try again after a millisecond
		if(i == 1)
			LogTrace("No go, trying again after 1 ms\n");
		if(i >= 1)
			usleep(1 * 1000);
	}
	if(i > 1)
		LogTrace("Poll ended after %d ms\n", i);

	//Give up if we still got nothing
	if(i == nmax)
	{
		DebugAbort();
		throw JtagExceptionWrapper(
			"Failed to read AP register (still waiting after way too long)",
			"");
	}

	//Verify the read was successful
	ARMDebugPortStatusRegister stat = GetStatusRegister();
	if(stat.bits.sticky_err)
	{
		LogError("Something went wrong (sticky error bit set when reading AP %d register %d)\n", ap, addr);
		//PrintStatusRegister(stat, false);

		DebugAbort();

		throw JtagExceptionWrapper(
			"Failed to read AP register",
			"");
	}

	return data_out;
}

/**
	@brief Aborts the current AP transaction
 */
void ARMDebugPort::DebugAbort()
{
	SetIR(INST_ABORT);

	//Write to the abort register
	uint8_t abort_value[5] = {0};
	PokeBit(abort_value, 3, true);
	unsigned char unused[5];
	ScanDR(abort_value, unused, 35);
	auto statreg = GetStatusRegister();
	if(statreg.bits.sticky_err)
	{
		LogTrace("Sticky error bit is set, clearing\n");
		ClearStatusRegisterErrors();
	}
}

/**
	@brief Writes to an AP register

	@param ap			The number of the AP to access
	@param addr			The ID of the AP register to read
	@param wdata		The value to write
 */
void ARMDebugPort::APRegisterWrite(uint8_t ap, ApReg addr, uint32_t wdata)
{
	//Set the high bits of the address as the current bank
	uint32_t select = (ap << 24) | (addr & 0xf0);
	DPRegisterWrite(REG_AP_SELECT,  select );

	//Do the read
	SetIR(INST_APACC);

	//Concatenate the 3-bit A / RnW field to request the write with the data itself
	uint8_t addr_flags = ((addr & 0x0c) >> 1) | OP_WRITE;
	//LogTrace("        addr_flags = %x\n", addr_flags);
	uint8_t* wdata_p = (uint8_t*)&wdata;
	uint8_t txd[5];
	for(int i=0; i<3; i++)
		PokeBit(txd, i, PeekBit(&addr_flags, i));
	for(int i=0; i<32; i++)
		PokeBit(txd, i+3, PeekBit(wdata_p, i));

	//Get the data back and extract the reply
	unsigned char rxd[5];
	ScanDR(txd, rxd, 35);
	uint8_t ack_out = 0;
	for(int i=0; i<3; i++)
		PokeBit(&ack_out, i, PeekBit(rxd, i));

	//If the original ACK-out was a "wait", we have to do something
	if(ack_out == WAIT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Send a dummy read to get the response code
	addr_flags = ((addr & 0x0c) >> 1) | OP_READ;
	for(int i=0; i<3; i++)
		PokeBit(txd, i, PeekBit(&addr_flags, i));
	for(int i=0; i<32; i++)
		PokeBit(txd, i+3, false);
	ScanDR(txd, rxd, 35);
	for(int i=0; i<3; i++)
		PokeBit(&ack_out, i, PeekBit(rxd, i));
	if(ack_out != OK_OR_FAULT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Verify the read was successful
	ARMDebugPortStatusRegister stat = GetStatusRegister();
	if(stat.bits.sticky_err)
	{
		/*
		LogError("Write of %08x to register %x on AP %d failed\n",
			wdata, addr, ap);
		m_aps[ap]->PrintStatusRegister();
		*/

		DebugAbort();
		throw JtagExceptionWrapper(
			"Failed to write AP register",
			"");
	}
}

/**
	@brief Reads from a DP register

	@param addr			The ID of the DP register to read

	@return The value read
 */
uint32_t ARMDebugPort::DPRegisterRead(DpReg addr)
{
	SetIR(INST_DPACC);
	uint32_t data_out;

	//Poll until we get a good read back
	int i = 0;
	int nmax = 50;
	for(; i<nmax; i++)
	{
		//Send the 3-bit A / RnW field to request the read
		uint8_t addr_flags = (addr << 1) | OP_READ;
		uint8_t txd[5] = {0};
		for(int i=0; i<3; i++)
			PokeBit(txd, i, PeekBit(&addr_flags, i));
		uint8_t rxd[5];
		ScanDR(txd, rxd, 35);
		uint8_t ack_out = 0;
		for(int i=0; i<3; i++)
			PokeBit(&ack_out, i, PeekBit(rxd, i));

		//If we got data, crunch it.
		//Note that the first poll can never return the data since we haven't even done the read request yet!
		if((ack_out == OK_OR_FAULT) && (i > 0) )
		{
			for(int i=0; i<32; i++)
				PokeBit((unsigned char*)&data_out, i, PeekBit(rxd, i+3));
			break;
		}

		//No go? Try again after a millisecond
		if(i == 1)
			LogTrace("No go, trying again after 1 ms\n");
		if(i >= 1)
			usleep(1 * 1000);
	}
	if(i > 1)
		LogTrace("Poll ended after %d ms\n", i);

	//Give up if we still got nothing
	if(i == nmax)
	{
		DebugAbort();
		throw JtagExceptionWrapper(
			"Failed to read DP register (still waiting after way too long)",
			"");
	}

	return data_out;
}

void ARMDebugPort::DPRegisterWrite(DpReg addr, uint32_t wdata)
{
	SetIR(INST_DPACC);

	//Send the 3-bit A / RnW field to request the write
	uint8_t addr_flags = (addr << 1) | OP_WRITE;
	uint8_t txd[5] = {0};
	uint8_t* wdata_p = (uint8_t*)&wdata;
	for(int i=0; i<3; i++)
		PokeBit(txd, i, PeekBit(&addr_flags, i));
	for(int i=0; i<32; i++)
		PokeBit(txd, i+3, PeekBit(wdata_p, i));
	unsigned char rxd[5];
	ScanDR(txd, rxd, 35);

	//Send a read request to get the response code
	addr_flags = (addr << 1) | OP_READ;
	for(int i=0; i<3; i++)
		PokeBit(txd, i, PeekBit(&addr_flags, i));
	for(int i=0; i<32; i++)
		PokeBit(txd, i+3, 0);
	ScanDR(txd, rxd, 35);
	uint8_t ack_out = 0;
	for(int i=0; i<3; i++)
		PokeBit(&ack_out, i, PeekBit(rxd, i));
	if(ack_out != OK_OR_FAULT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}
}
