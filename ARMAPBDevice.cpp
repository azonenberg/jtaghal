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
	@brief Implementation of ARMAPBDevice
 */

#include "jtaghal.h"
#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMAPBDevice.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ARMAPBDevice::ARMAPBDevice(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg)
	: m_ap(ap)
	, m_idreg(idreg)
	, m_address(address)
{

}

ARMAPBDevice::~ARMAPBDevice()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

///reads a register given the offset from our base address
uint32_t ARMAPBDevice::ReadRegisterByOffset(uint32_t offset)
{
	return m_ap->ReadWord(m_address + offset);
}

///reads a register given the index into a 32-bit register space
uint32_t ARMAPBDevice::ReadRegisterByIndex(uint32_t index)
{
	return ReadRegisterByOffset(index*4);
}

void ARMAPBDevice::WriteRegisterByIndex(uint32_t index, uint32_t value)
{
	WriteRegisterByOffset(index*4, value);
}

void ARMAPBDevice::WriteRegisterByOffset(uint32_t offset, uint32_t value)
{
	m_ap->WriteWord(m_address + offset, value);
}
