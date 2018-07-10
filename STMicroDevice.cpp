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
	@brief Implementation of STMicroDevice
 */

#include "jtaghal.h"
#include "JEDECVendorID_enum.h"
#include "STMicroDeviceID_enum.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

STMicroDevice::STMicroDevice(unsigned int devicetype, unsigned int stepping)
	: m_devicetype(devicetype)
	, m_stepping(stepping)
{

}

/**
	@brief Default virtual destructor
 */
STMicroDevice::~STMicroDevice()
{

}

/**
	@brief Creates a STMicroDevice given an ID code

	@throw JtagException if the ID code supplied is not a valid STMicro device, or not a known family number

	@param idcode	The ID code of this device
	@param iface	The JTAG adapter this device was discovered on
	@param pos		Position in the chain that this device was discovered

	@return A valid JtagDevice object, or NULL if the vendor ID was not recognized.
 */
JtagDevice* STMicroDevice::CreateDevice(unsigned int idcode, JtagInterface* iface, size_t pos)
{
	//Save the original ID code to give to the derived class
	unsigned int idcode_raw = idcode;

	//Rightmost bit is always a one, ignore it
	idcode >>= 1;

	//Sanity check manufacturer ID
	if( (idcode & 0x7FF) != VENDOR_ID_STMICRO)
	{
		throw JtagExceptionWrapper(
			"Invalid IDCODE supplied (wrong JEDEC manufacturer ID, not a STMicro device)",
			"");
	}
	idcode >>= 11;

	//Next 16 bits are device type
	unsigned int devicetype = idcode & 0xffff;
	idcode >>= 16;

	//Revision
	unsigned int rev = idcode & 0xF;

	//Doesn't seem to be any family description bits? Just switch on the device ID
	switch(devicetype)
	{
		case STM32F411E:
			return STM32Device::CreateDevice(devicetype, rev, idcode_raw, iface, pos);

		default:
			throw JtagExceptionWrapper(
			"Unknown STMicro part - probably not yet supported",
			"");
	}

	return NULL;
}
