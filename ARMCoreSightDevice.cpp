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
	@brief Base class for ARM CoreSight components on a debug APB bus
 */
#include "jtaghal.h"
#include "ARMAPBDevice.h"
#include "ARMCoreSightDevice.h"

using namespace std;

ARMCoreSightDevice::ARMCoreSightDevice(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: ARMAPBDevice(ap, address, idreg)
{
}

ARMCoreSightDevice::~ARMCoreSightDevice()
{
}

void ARMCoreSightDevice::PrintInfo()
{
	LogVerbose("%s rev %d.%d.%d\n",
		GetDescription().c_str(),
		m_idreg.revnum, m_idreg.cust_mod, m_idreg.revand);
}

string ARMCoreSightDevice::GetDescription()
{
	switch(m_idreg.partnum)
	{
		case 0x003:
			return "Cortex-M4 Flash Patch/Breakpoint";
		case 0x00c:
			return "Cortex-M4 System Control Space";
		case 0x906:
			return "CoreSight Cross Trigger Interface";
		case 0x907:
			return "CoreSight Embedded Trace Buffer";
		case 0x908:
			return "CoreSight Trace Funnel";
		case 0x912:
			return "CoreSight Trace Port Interface Unit";

		//ID is 913, not 914. CoreSight Components TRM is wrong.
		//See ARM #TAC650738
		case 0x913:
			return "CoreSight Instrumentation Trace Macrocell";
		case 0x914:
			return "CoreSight Serial Wire Output";
		case 0x925:
			return "Cortex-M4 Embedded Trace Macrocell";
		case 0x950:
			return "Cortex-A9 Program Trace Macrocell";
		case 0x9A0:
			return "Cortex-A9 Performance Monitoring Unit";

		default:
			LogWarning("Unknown ARM device (part number 0x%x)\n", m_idreg.partnum);
			return "unknown CoreSight device";
	}
}
