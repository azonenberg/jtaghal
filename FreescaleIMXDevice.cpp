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
	@brief Implementation of FreescaleIMXDevice
 */

#include "jtaghal.h"
#include "FreescaleIMXDevice.h"
#include "memory.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

FreescaleIMXDevice::FreescaleIMXDevice(
	unsigned int devid, unsigned int stepping,
	unsigned int idcode, JtagInterface* iface, size_t pos)
 : FreescaleMicrocontroller(idcode, iface, pos, 5)
{

}

/**
	@brief Destructor
 */
FreescaleIMXDevice::~FreescaleIMXDevice()
{
}

void FreescaleIMXDevice::PostInitProbes()
{
	/*
	m_devid = devid;
	m_stepping = stepping;

	//Look up device info in the table and make sure it exists
	m_devinfo = NULL;
	for(auto& x : g_devinfo)
	{
		if(x.devid == devid)
			m_devinfo = &x;
	}

	if(!m_devinfo)
	{
		throw JtagExceptionWrapper(
			"Invalid PIC32 JTAG IDCODE",
			"");
	}

	//Reset both TAPS
	EnterMtapMode();
	ResetToIdle();
	EnterEjtagMode();
	ResetToIdle();

	//Get our implementation code
	GetImpCode();*/
}

JtagDevice* FreescaleIMXDevice::CreateDevice(
	unsigned int devid, unsigned int stepping, unsigned int idcode, JtagInterface* iface, size_t pos)
{
	//TODO: Sanity checks
	return new FreescaleIMXDevice(devid, stepping, idcode, iface, pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

string FreescaleIMXDevice::GetDescription()
{
	/*
	char srev[256];
	snprintf(srev, sizeof(srev), "Microchip %s (%u KB SRAM, %u KB code flash, %.2f KB boot flash, stepping %u)",
		m_devinfo->name,
		m_devinfo->sram_size,
		m_devinfo->program_flash_size,
		m_devinfo->boot_flash_size,
		m_stepping);

	return string(srev);
	*/
	return "i.mx unimplemented";
}

bool FreescaleIMXDevice::IsProgrammed()
{
	throw JtagExceptionWrapper(
		"Not implemented",
		"");
}

void FreescaleIMXDevice::Erase()
{
	throw JtagExceptionWrapper(
		"Not implemented",
		"");
}

void FreescaleIMXDevice::Program(FirmwareImage* /*image*/)
{
	throw JtagExceptionWrapper(
		"Not implemented",
		"");
}
