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

	uint32_t id = m_dap->ReadMemory(0xe0042000);
	LogDebug("id = %08x\n", id);

	uint32_t serial[3];
	serial[0] = m_dap->ReadMemory(0x1fff7a10);	//xy
	serial[1] = m_dap->ReadMemory(0x1fff7a14);	//lot, wafnum
	serial[2] = m_dap->ReadMemory(0x1fff7a18);	//lot

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
	LogDebug("WaferX/Y %d %d, wafer %d of lot %s\n", m_waferX, m_waferY, m_waferNum, m_waferLot);

	//Look up size of flash memory
	m_flashKB = m_dap->ReadMemory(0x1fff7a20) >> 16;	//F_ID, flash size in kbytes

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

bool STM32Device::HasRPCInterface()
{
	return false;
}

bool STM32Device::HasDMAInterface()
{
	return false;
}

bool STM32Device::IsProgrammed()
{
	LogWarning("STM32Device::IsProgrammed() not implemented\n");
	return true;
}

void STM32Device::Erase()
{
	LogWarning("STM32Device::Erase() not implemented\n");

	//TODO: What other STM32 devices is this valid for?
	if(m_devicetype != STM32F411E)
	{
		throw JtagExceptionWrapper(
			"STM32Device::Erase() not tested for any parts other than STM32F411E",
			"");
	}

	//Flash SFR base address
	uint32_t flash_sfr_base = 0x40023C00;
/*
	//Check CR
	uint32_t cr = m_dap->ReadMemory(flash_sfr_base + 0x10);
	LogDebug("Initial CR = %08x\n", cr);

	//Unlock flash
	m_dap->WriteMemory(flash_sfr_base + 0x4, 0x45670123);
	m_dap->WriteMemory(flash_sfr_base + 0x4, 0xCDEF89AB);

	//Check CR
	cr = m_dap->ReadMemory(flash_sfr_base + 0x10);
	LogDebug("Unlocked CR = %08x\n", cr);

	//Wait for FLASH_CR.BSY to be cleared
	//Set FLASH_CR.MER
	//Set FLASH_CR.STRT
	//Wait for FLASH_CR.BSY to be cleared
	*/
}

void STM32Device::Program(FirmwareImage* /*image*/)
{
	throw JtagExceptionWrapper(
		"Not implemented",
		"");
}
