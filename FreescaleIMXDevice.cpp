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
 , m_stepping(stepping)
{
	if(pos < 2)
	{
		throw JtagExceptionWrapper(
			"FreescaleIMXDevice boundary scan TAP must not be the first or second device in the scan chain. Where's the ARM DAP or SDMA?",
			"");
	}

	//We are the System JTAG Controller (reference manual section 56.2.3)
	switch(devid)
	{
	case IMX_6_SOLO:
		m_devid = IMX_6_SOLO;
		break;

	case IMX_6_DUAL_LITE:
		m_devid = IMX_6_DUAL_LITE;
		break;

	default:
		throw JtagExceptionWrapper(
			"Invalid i.mx IDCODE",
			"");
		break;
	}

	//tie off pointers until PostInitProbes()
	m_dap = NULL;
	m_sdma = NULL;
}

/**
	@brief Destructor
 */
FreescaleIMXDevice::~FreescaleIMXDevice()
{
}

void FreescaleIMXDevice::PostInitProbes(bool /*quiet*/)
{
	//Get a pointer to our ARM DAP. This should always be one scan chain position before us.
	m_dap = dynamic_cast<ARMDebugPort*>(m_iface->GetDevice(m_pos-2));
	if(m_dap == NULL)
	{
		throw JtagExceptionWrapper(
			"FreescaleIMXDevice expects an ARM DAP two chain positions prior",
			"");
	}

	//One position back should be a dummy.
	//We need to swap that out with the SDMA
	size_t sdma_pos = m_pos-1;
	auto sdma_dummy = dynamic_cast<JtagDummy*>(m_iface->GetDevice(sdma_pos));
	if(sdma_dummy == NULL)
	{
		throw JtagExceptionWrapper(
			"FreescaleIMXDevice expects a dummy SDMA one chain position prior",
			"");
	}
	m_sdma = new FreescaleIMXSmartDMA(0x0, 0x0, 0x0, m_iface, sdma_pos);
	m_iface->SwapOutDummy(sdma_pos, m_sdma);
}

JtagDevice* FreescaleIMXDevice::CreateDevice(
	unsigned int devid, unsigned int stepping, unsigned int idcode, JtagInterface* iface, size_t pos)
{
	//TODO: Sanity checks
	return new FreescaleIMXDevice(devid, stepping, idcode, iface, pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// General device info

void FreescaleIMXDevice::Reset()
{
	throw JtagExceptionWrapper(
		"Soft reset not implemented for this CPU yet",
		"");
}

string FreescaleIMXDevice::GetDescription()
{
	const char* desc = "";
	switch(m_devid)
	{
		case IMX_6_DUAL_LITE:
			desc = "Dual Lite";
			break;

		case IMX_6_SOLO:
			desc = "Solo";
			break;
	}

	char srev[256];
	snprintf(srev, sizeof(srev), "Freescale i.mx6 %s (stepping %u)",
		desc,
		m_stepping);

	return string(srev);
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
