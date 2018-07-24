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
	@brief Implementation of FreescaleIMXSmartDMA
 */

#include "jtaghal.h"
#include "FreescaleIMXSmartDMA.h"
#include "STMicroDeviceID_enum.h"
#include "memory.h"

using namespace std;

FreescaleIMXSmartDMA::FreescaleIMXSmartDMA(
	unsigned int /*devid*/, unsigned int /*stepping*/,
	unsigned int idcode, JtagInterface* iface, size_t pos)
 : FreescaleDevice(idcode, iface, pos, 4)
{
	//page 4753 has sdma info
	if(pos < 1)
	{
		throw JtagExceptionWrapper(
			"FreescaleIMX Smart DMA boundary scan TAP must not be the first device in the scan chain. Where's the ARM DAP?",
			"");
	}
}

void FreescaleIMXSmartDMA::PostInitProbes()
{

}

/**
	@brief Destructor
 */
FreescaleIMXSmartDMA::~FreescaleIMXSmartDMA()
{
}

JtagDevice* FreescaleIMXSmartDMA::CreateDevice(
	unsigned int devid, unsigned int stepping, unsigned int idcode, JtagInterface* iface, size_t pos)
{
	//TODO: Sanity checks
	return new FreescaleIMXSmartDMA(devid, stepping, idcode, iface, pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device property queries

string FreescaleIMXSmartDMA::GetDescription()
{
	return "Freescale i.mx6 Smart DMA";
}
