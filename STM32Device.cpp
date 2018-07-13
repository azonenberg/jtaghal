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
	@brief Implementation of STM32Device
 */

#include "jtaghal.h"
#include "STM32Device.h"
#include "STMicroDeviceID_enum.h"
#include "memory.h"

using namespace std;

STM32Device::STM32Device(
	unsigned int devid, unsigned int stepping,
	unsigned int idcode, JtagInterface* iface, size_t pos)
 : STMicroMicrocontroller(devid, stepping, idcode, iface, pos)
 , JtagDevice(idcode, iface, pos)
{
	m_irlength = 5;

	if(pos == 0)
	{
		throw JtagExceptionWrapper(
			"STM32Device boundary scan TAP must not be the first device in the scan chain. Where's the ARM DAP?",
			"");
	}

	//Get a pointer to our ARM DAP. This should always be one scan chain position before us.
	m_dap = dynamic_cast<ARMDebugPort*>(iface->GetDevice(pos-1));
	if(m_dap == NULL)
	{
		throw JtagExceptionWrapper(
			"STM32Device expects an ARM DAP one chain position prior",
			"");
	}

	//uint32_t id = m_dap->ReadMemory(0xe0042000);
	//LogDebug("id = %08x\n", id);

	//Check read lock status
	m_locksProbed = false;
	STM32Device::ProbeLocksNondestructive();

	//Extract serial number fields
	try
	{
		uint32_t serial[3];
		serial[0] = m_dap->ReadMemory(0x1fff7a10);
		serial[1] = m_dap->ReadMemory(0x1fff7a14);
		serial[2] = m_dap->ReadMemory(0x1fff7a18);
		m_waferX = serial[0] >> 16;
		m_waferY = serial[0] & 0xffff;
		m_waferNum = serial[1] & 0xff;
		m_waferLot[0] = (serial[1] >> 24) & 0xff;
		m_waferLot[1] = (serial[1] >> 16) & 0xff;
		m_waferLot[2] = (serial[1] >> 8) & 0xff;
		m_waferLot[3] = (serial[2] >> 24) & 0xff;
		m_waferLot[4] = (serial[2] >> 16) & 0xff;
		m_waferLot[5] = (serial[2] >> 8) & 0xff;
		m_waferLot[6] = (serial[2] >> 0) & 0xff;
		m_waferLot[7] = 0;
		m_serialRaw[0] = m_waferX >> 8;
		m_serialRaw[1] = m_waferX & 0xff;
		m_serialRaw[2] = m_waferY >> 8;
		m_serialRaw[3] = m_waferY & 0xff;
		m_serialRaw[4] = m_waferNum;
		for(int i=0; i<7; i++)
			m_serialRaw[5+i] = m_waferLot[i];
	}
	catch(const JtagException& e)
	{
		//If we can't read the serial number, that probably means the chip is locked.
		//Write zeroes rather than leaving it uninitialized.
		m_waferX = 0;
		m_waferY = 0;
		m_waferNum = 0;
		strncpy(m_waferLot, "???????", sizeof(m_waferLot));
		for(int i=0; i<12; i++)
			m_serialRaw[i] = 0;

		//Display a warning if the chip is NOT locked,  but we couldn't read the serial number anyway
		if(!IsDeviceReadLocked().GetValue())
			LogWarning("STM32Device: Unable to read serial number even though read protection doesn't seem to be set\n");

		else
			LogNotice("STM32Device: Cannot determine serial number because read protection is set\n");
	}

	//Look up size of flash memory
	try
	{
		m_flashKB = m_dap->ReadMemory(0x1fff7a20) >> 16;	//F_ID, flash size in kbytes
	}
	catch(const JtagException& e)
	{
		//If we fail, set flash size to zero so we don't try doing anything with it
		m_flashKB = 0;

		if(!IsDeviceReadLocked().GetValue())
			LogWarning("STM32Device: Unable to read flash memory size even though read protection doesn't seem to be set\n");

		else
			LogNotice("STM32Device: Cannot determine flash size because read protection is set\n");
	}

	//TODO: How portable are these addresses?
	m_flashSfrBase		= 0x40023C00;
	m_flashMemoryBase	= 0x08000000;
	m_sramMemoryBase	= 0x20000000;

	//Look up RAM size (TODO can we get this from descriptors somehow?)
	switch(devid)
	{
		case STM32F411E:
			m_ramKB = 128;
			break;

		default:
			m_ramKB = 0;
	}
}

/**
	@brief Destructor
 */
STM32Device::~STM32Device()
{
}

JtagDevice* STM32Device::CreateDevice(
	unsigned int devid, unsigned int stepping, unsigned int idcode, JtagInterface* iface, size_t pos)
{
	//TODO: Sanity checks
	return new STM32Device(devid, stepping, idcode, iface, pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device property queries

bool STM32Device::ReadingSerialRequiresReset()
{
	return false;
}

int STM32Device::GetSerialNumberLength()
{
	return 12;
}

int STM32Device::GetSerialNumberLengthBits()
{
	return 96;
}

void STM32Device::GetSerialNumber(unsigned char* data)
{
	for(int i=0; i<12; i++)
		data[i] = m_serialRaw[i];
}

string STM32Device::GetPrettyPrintedSerialNumber()
{
	char tmp[256];
	snprintf(tmp, sizeof(tmp),
		"Die (%d, %d), wafer %d, lot %s",
		m_waferX, m_waferY,
		m_waferNum,
		m_waferLot);

	return string(tmp);
}

string STM32Device::GetDescription()
{
	string name = "(unknown STM32";
	switch(m_devicetype)
	{
		case STM32F411E:
			name = "STM32F411E";
			break;
	}

	char srev[256];
	snprintf(srev, sizeof(srev), "ST %s (%u KB SRAM, %u KB flash, stepping %u)",
		name.c_str(),
		m_ramKB,
		m_flashKB,
		m_stepping);

	return string(srev);
}

bool STM32Device::IsProgrammed()
{
	//If we're read locked, we're obviously programmed in some way
	ProbeLocksNondestructive();
	if(m_protectionLevel != 0)
		return true;

	//If the first word of the interrupt vector table is blank, the device is not programmed
	//because no code can execute.
	//This is a lot faster than a full chip-wide blank check.
	return (m_dap->ReadMemory(m_flashMemoryBase) != 0xffffffff);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lock bit access

void STM32Device::ProbeLocksNondestructive()
{
	//Don't poke more than once
	if(m_locksProbed)
		return;

	LogTrace("Running non-destructive lock status tests\n");
	LogIndenter li;

	try
	{
		uint32_t optcr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
		uint32_t rdp = (optcr >> 8) & 0xff;

		LogTrace("OPTCR = %08x\n", optcr);
		LogTrace("OPTCR.RDP = 0x%02x\n", rdp);

		//If OPTCR is all 1s we probably just bulk-erased everything.
		//Treat this as unlocked.
		if(optcr == 0xffffffff)
			m_protectionLevel = 0;

		else
		{
			//TODO: query write protection

			//Not locked?
			if(rdp == 0xaa)
				m_protectionLevel = 0;

			//Full locked? we should never see this because it disables JTAG
			else if(rdp == 0xcc)
				m_protectionLevel = 2;

			//Level 1 lock
			//Unlikely to see this except right after programming the lock bit, since RDP disables access to OPTCR
			else
				m_protectionLevel = 1;
		}
	}
	catch(const JtagException& e)
	{
		//If wqe get here, reading one or more of the SFRs failed.
		//This is a probable indicator of level 1 read protection since we have limited JTAG access
		//but can still get ID codes etc (which rules out level 2).
		m_protectionLevel = 1;
	}

	m_locksProbed = true;
}

void STM32Device::ProbeLocksDestructive()
{
	//no destructive tests implemented yet
	return ProbeLocksNondestructive();
}

UncertainBoolean STM32Device::CheckMemoryAccess(uint32_t start, uint32_t end, unsigned int access)
{
	//Not yet implemented
	return UncertainBoolean(false, UncertainBoolean::USELESS);
}

UncertainBoolean STM32Device::IsDeviceReadLocked()
{
	ProbeLocksNondestructive();

	switch(m_protectionLevel)
	{
		case 2:
			return UncertainBoolean( true, UncertainBoolean::CERTAIN );

		case 0:
			return UncertainBoolean( false, UncertainBoolean::CERTAIN );

		case 1:
		default:
			return UncertainBoolean( true, UncertainBoolean::VERY_LIKELY );
	}
}

void STM32Device::SetReadLock()
{
	UnlockFlashOptions();

	LogTrace("Setting read lock...\n");
	LogIndenter li;

	//Read OPTCR
	uint32_t cr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
	LogTrace("Old OPTCR = %08x\n", cr);
	cr &= 0xffff00ff;
	cr |= 0x5500;		//CC = level 2 lock
						//AA = no lock
						//anything else = level 1 lock

	//Actually commit the write to the option register
	//If we don't do this, the SRAM register is changed but it won't persist across reboots. Pretty useless!
	cr |= 0x2;

	LogTrace("Setting OPTCR = %08x\n", cr);

	//Write it back
	m_dap->WriteMemory(m_flashSfrBase + 0x14, cr);
}

void STM32Device::ClearReadLock()
{
	/*
	UnlockFlash();
	UnlockFlashOptions();

	LogTrace("Clearing read lock...\n");
	LogIndenter li;

	//Read OPTCR
	uint32_t cr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
	LogTrace("Old OPTCR = %08x\n", cr);
	cr &= 0xffff00ff;*/

	//Try poking something into RAM
	m_dap->WriteMemory(m_sramMemoryBase, 0xfeedface);
	uint32_t v = m_dap->ReadMemory(m_sramMemoryBase);
	LogDebug("v = 0x%08x\n", v);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Programming

void STM32Device::UnlockFlashOptions()
{
	LogTrace("Unlocking Flash option register...\n");
	LogIndenter li;

	//Check CR
	uint32_t cr = 0x00000001;
	try
	{
		cr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
	}
	catch(JtagException& e)
	{
		LogWarning("Couldn't read intial OPTCR, guessing value of %08x\n", cr);
	}
	LogTrace("Initial OPTCR = %08x\n", cr);
	if(cr & 0x00000001)
	{
		LogTrace("Option register is curently locked, unlocking...\n");

		//Unlock flash
		m_dap->WriteMemory(m_flashSfrBase + 0x8, 0x08192A3B);
		m_dap->WriteMemory(m_flashSfrBase + 0x8, 0x4C5D6E7F);

		//Check CR
		try
		{
			cr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
		}
		catch(JtagException& e)
		{
			cr = 0x00000000;
			LogWarning("Couldn't read unlocked OPTCR, guessing value of %08x\n", cr);
		}
		LogTrace("Unlocked OPTCR = %08x\n", cr);
		if(cr & 0x00000001)
		{
			throw JtagExceptionWrapper(
				"STM32Device::UnlockFlashOptions() got OPTCR still locked after unlock sequence!!!",
				"");
		}
	}
	else
		LogTrace("Option register is already unlocked, no action required\n");
}

void STM32Device::UnlockFlash()
{
	LogTrace("Unlocking Flash memory...\n");
	LogIndenter li;

	//Check CR
	uint32_t cr = m_dap->ReadMemory(m_flashSfrBase + 0x10);
	LogTrace("Initial CR = %08x\n", cr);
	if(cr & 0x80000000)
	{
		LogTrace("Flash is curently locked\n");

		//Unlock flash
		m_dap->WriteMemory(m_flashSfrBase + 0x4, 0x45670123);
		m_dap->WriteMemory(m_flashSfrBase + 0x4, 0xCDEF89AB);

		//Check CR
		cr = m_dap->ReadMemory(m_flashSfrBase + 0x10);
		LogTrace("Unlocked CR = %08x\n", cr);
		if(cr & 0x80000000)
		{
			throw JtagExceptionWrapper(
				"STM32Device::UnlockFlash() got CR still locked after unlock sequence!!!",
				"");
		}
	}
	else
		LogTrace("Flash is already unlocked, no action required\n");
}

void STM32Device::PollUntilFlashNotBusy()
{
	LogTrace("Waiting for Flash to be ready...\n");
	LogIndenter li;

	//Poll FLASH_SR.BSY until it's clear
	uint32_t sr = m_dap->ReadMemory(m_flashSfrBase + 0x0c);
	//LogTrace("SR = %08x\n", sr);
	int interval = 1;
	while(sr & 0x00010000)
	{
		//LogTrace("SR = %08x\n", sr);
		usleep(100 * interval);
		sr = m_dap->ReadMemory(m_flashSfrBase + 0x0c);

		//Exponential back-off on polling interval
		interval *= 10;
	}
}

void STM32Device::Erase()
{
	LogTrace("Erasing...\n");

	//TODO: What other STM32 devices is this valid for?
	if(m_devicetype != STM32F411E)
	{
		throw JtagExceptionWrapper(
			"STM32Device::Erase() not tested for any parts other than STM32F411E",
			"");
	}

	//Look up FLASH_OPTCR
	uint32_t optcr = m_dap->ReadMemory(m_flashSfrBase + 0x14);
	LogTrace("FLASH_OPTCR = %08x\n", optcr);

	//Unlock flash and make sure it's ready
	UnlockFlash();
	PollUntilFlashNotBusy();

	//Do a full-chip erase with x32 parallelism
	//bit9:8 = 10 = x64
	//bit2 = mass erase
	//bit16 = go do it
	LogTrace("Mass erase...\n");
	m_dap->WriteMemory(m_flashSfrBase + 0x10, 0x10204);

	//Wait for it to finish the erase operation
	PollUntilFlashNotBusy();

	//Make sure we really erased it
	if(!BlankCheck())
	{
		throw JtagExceptionWrapper(
			"STM32Device::Erase() failed somehow. We did everything we could to erase but it's not blank!",
			"");
	}
}

bool STM32Device::BlankCheck()
{
	LogTrace("Blank checking...\n");
	LogIndenter li;

	bool quitImmediately = true;

	uint32_t flashBytes = m_flashKB * 1024;
	uint32_t addrMax = m_flashMemoryBase + flashBytes;
	uint32_t addr = m_flashMemoryBase;
	LogTrace("Checking address range from 0x%08x to 0x%08x...\n", addr, addrMax);
	bool blank = true;
	for(; addr<addrMax; addr += 4)
	{
		if( (addr & 0xffff) == 0)
		{
			float bytesDone = (addr - m_flashMemoryBase) * 1.0f / flashBytes;
			LogTrace("%08x (%.1f %%)\n", addr, bytesDone * 100.0f);
		}

		uint32_t rdata = m_dap->ReadMemory(addr);
		if(rdata != 0xffffffff)
		{
			LogNotice("Device is NOT blank. Found data 0x%08x at flash address 0x%08x\n",
				rdata, addr);
			if(quitImmediately)
				return false;
			blank = false;
		}
	}

	return blank;
}

void STM32Device::Program(FirmwareImage* /*image*/)
{
	throw JtagExceptionWrapper(
		"Not implemented",
		"");
}
