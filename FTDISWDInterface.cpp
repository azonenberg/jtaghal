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
	@brief Implementation of FTDISWDInterface
 */

#include "jtaghal.h"

#ifdef HAVE_FTD2XX

#include <ftd2xx/ftd2xx.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Connects to an FTDI JTAG interface

	@throw SWDException if the connection could not be establishes or the serial number is invalid

	@param serial		Serial number of the device to connect to
	@param layout		Adapter layout to use
 */
FTDISWDInterface::FTDISWDInterface(const string& serial, const string& layout)
	: FTDIDriver(serial, layout)
{
}

/**
	@brief Interface destructor

	Closes handles and disconnects from the adapter.
 */
FTDISWDInterface::~FTDISWDInterface()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Shim overrides to push SWDInterface functions into FTDIDriver

void FTDISWDInterface::Commit()
{
	FTDIDriver::Commit();
}

string FTDISWDInterface::GetName()
{
	return GetName();
}

string FTDISWDInterface::GetSerial()
{
	return GetSerial();
}

string FTDISWDInterface::GetUserID()
{
	return FTDIDriver::GetUserID();
}

int FTDISWDInterface::GetFrequency()
{
	return FTDIDriver::GetFrequency();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level

/**
	@brief Performs a SW-DP write transaction
 */
void FTDISWDInterface::WriteWord(uint8_t reg_addr, uint32_t wdata)
{

}

/**
	@brief Performs a SW-DP read transaction
 */
uint32_t FTDISWDInterface::ReadWord(uint8_t reg_addr)
{

}

#endif
