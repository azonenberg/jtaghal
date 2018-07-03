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

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMDebugPort::ARMDebugPort(
	unsigned int partnum,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
	: ARMDevice(idcode, iface, pos)
	, m_rev(rev)
	, m_partnum(partnum)
{
	m_irlength = 4;

	//No Mem-AP for now
	m_defaultMemAP 		= NULL;
	m_defaultRegisterAP	= NULL;

	//Turn on the debug stuff
	EnableDebugging();

	//Figure out how many APs we have
	LogDebug("Found ARM JTAG-DP, probing...\n");
	LogIndenter li;
	LogDebug("Searching for APs...\n");
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
			LogDebug("Found JTAG-AP rev %d at index %d\n", idr.bits.revision, nap);
			LogIndenter li;
			LogDebug("Not supported yet, ignoring\n");
			continue;
		}

		//Create a new MEM-AP
		else
		{
			ARMDebugMemAccessPort* ap = new ARMDebugMemAccessPort(this, nap, idr);
			m_aps[nap] = ap;

			if(ap->GetBusType() == ARMDebugAccessPort::DAP_AHB)
				LogDebug("Found AHB MEM-AP rev %d at index %d\n", idr.bits.revision, nap);
			else if(ap->GetBusType() == ARMDebugAccessPort::DAP_APB)
				LogDebug("Found APB MEM-AP rev %d at index %d\n", idr.bits.revision, nap);

			//If it's an AHB Mem-AP, and we don't have a default Mem-AP, this one is probably RAM.
			//Use it as our default AP.
			if( (ap->GetBusType() == ARMDebugAccessPort::DAP_AHB) && (m_defaultMemAP == NULL) )
			{
				LogIndenter li;
				LogDebug("Using as default RAM Mem-AP\n");
				m_defaultMemAP = ap;
			}

			//If it's an APB Mem-AP, and we don't have a default Mem-AP, this one is probably CoreSight debug registers.
			//Use it as our default AP.
			if( (ap->GetBusType() == ARMDebugAccessPort::DAP_APB) && (m_defaultRegisterAP == NULL) )
			{
				LogIndenter li;
				LogDebug("Using as default CoreSight Mem-AP\n");
				m_defaultRegisterAP = ap;
			}
		}
	}

	//Initialize each of the APs once they are all open
	LogDebug("Initializing APs...\n");
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
		devname = "JTAG DAP";
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
	stat.word = 0;
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
		//printf("        addr_flags = %x\n", addr_flags);
		uint8_t ack_out;
		m_iface->EnterShiftDR();
		m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
		if(ack_out != OK_OR_FAULT)
			LogWarning("Don't know what to do with WAIT request from DAP\n");

		//Send some dummy data
		unsigned char unused[4] = {0};
		unsigned char unused2[4] = {0};
		m_iface->ShiftData(1, unused, unused2, 32);
		m_iface->LeaveExit1DR();

		//Send the same A / RnW field again to get the response data
		m_iface->EnterShiftDR();
		m_iface->ShiftData(0, &addr_flags, &ack_out, 3);

		//Send some dummy data and get the read value back
		m_iface->ShiftData(1, unused, (unsigned char*)&data_out, 32);
		m_iface->LeaveExit1DR();

		//If we got a success result, done
		if(ack_out == OK_OR_FAULT)
			break;

		//No go? Try again after a millisecond
		if(i == 0)
			LogDebug("No go, trying again after 1 ms\n");
		usleep(1 * 1000);
	}
	if(i > 0)
		LogDebug("Poll ended after %d ms\n", i);

	//Give up if we still got nothing
	if(i == nmax)
	{
		DebugAbort();
		ClearStatusRegisterErrors();
		throw JtagExceptionWrapper(
			"Failed to read AP register (still waiting after way too long)",
			"");
	}

	//Verify the read was successful
	ARMDebugPortStatusRegister stat = GetStatusRegister();
	if(stat.bits.sticky_err)
	{
		LogError("Something went wrong (sticky error bit set when reading AP %d register %d)\n", ap, addr);
		PrintStatusRegister(stat, false);

		DebugAbort();
		ClearStatusRegisterErrors();

		//

		throw JtagExceptionWrapper(
			"Failed to read AP register",
			"");
		exit(1);
	}

	return data_out;
}

/**
	@brief Aborts the current AP transaction
 */
void ARMDebugPort::DebugAbort()
{
	SetIR(INST_ABORT);
	m_iface->EnterShiftDR();

	//Write to the abort register
	//TODO: make a bit field or something
	uint32_t abort_value = 0x1;
	unsigned char unused[4] = {0};
	m_iface->ShiftData(1, (unsigned char*)&abort_value, unused, 32);
	m_iface->LeaveExit1DR();
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

	//Send the 3-bit A / RnW field to request the write
	uint8_t addr_flags = ((addr & 0x0c) >> 1) | OP_WRITE;
	//printf("        addr_flags = %x\n", addr_flags);
	uint8_t ack_out;
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);

	///Send the data being written
	unsigned char unused[4] = {0};
	m_iface->ShiftData(1, (unsigned char*)&wdata, unused, 32);
	m_iface->LeaveExit1DR();

	//If the original ACK-out was a "wait", we have to do something
	if(ack_out == WAIT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Send a dummy read to get the response code
	addr_flags = ((addr & 0x0c) >> 1) | OP_READ;
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
	if(ack_out != OK_OR_FAULT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Send some dummy data
	uint32_t data_out;
	m_iface->ShiftData(1, unused, (unsigned char*)&data_out, 32);
	m_iface->LeaveExit1DR();

	//Verify the read was successful
	ARMDebugPortStatusRegister stat = GetStatusRegister();
	if(stat.bits.sticky_err)
	{
		LogError("Write of %08x to register %x on AP %d failed\n",
			wdata, addr, ap);
		m_aps[ap]->PrintStatusRegister();

		DebugAbort();
		ClearStatusRegisterErrors();
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

	//Send the 3-bit A / RnW field to request the read
	uint8_t addr_flags = (addr << 1) | OP_READ;
	uint8_t ack_out;
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
	//ignore original ack out

	//Send some dummy data
	unsigned char unused[4] = {0};
	unsigned char unused2[4] = {0};
	m_iface->ShiftData(1, unused, unused2, 32);
	m_iface->LeaveExit1DR();

	//Send the same A / RnW field again to get the response data
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
	if(ack_out != OK_OR_FAULT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Send some dummy data
	uint32_t data_out;
	m_iface->ShiftData(1, unused, (unsigned char*)&data_out, 32);
	m_iface->LeaveExit1DR();

	return data_out;
}

void ARMDebugPort::DPRegisterWrite(DpReg addr, uint32_t wdata)
{
	SetIR(INST_DPACC);

	//Send the 3-bit A / RnW field to request the write
	uint8_t addr_flags = (addr << 1) | OP_WRITE;
	uint8_t ack_out;
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
	//ignore original ack out

	//Send the data being written
	unsigned char unused[4] = {0};
	m_iface->ShiftData(1, (unsigned char*)&wdata, unused, 32);
	m_iface->LeaveExit1DR();

	//Send a read request to get the response code
	addr_flags = (addr << 1) | OP_READ;
	m_iface->EnterShiftDR();
	m_iface->ShiftData(0, &addr_flags, &ack_out, 3);
	if(ack_out != OK_OR_FAULT)
	{
		throw JtagExceptionWrapper(
			"Don't know what to do with WAIT request from DAP",
			"");
	}

	//Send some dummy data
	unsigned char unused2[4] = {0};
	m_iface->ShiftData(1, unused2, unused, 32);
	m_iface->LeaveExit1DR();
}
