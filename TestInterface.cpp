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
	@brief Implementation of TestInterface
 */
#include "jtaghal.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

TestInterface::TestInterface()
{
}

TestInterface::~TestInterface()
{
	for(auto p : m_devices)
		delete p;
	m_devices.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Queue management

/**
	@brief Commits the outstanding transactions to the adapter.

	No-op unless the adapter supports queueing of multiple writes.

	This function is automatically called when any readback is performed. Most adapter classes will automatically call
	it when the transmit queue reaches a certain size.

	This function can be called at any time to ensure all pending operations have executed.

	@throw JtagException in case of error
 */
void TestInterface::Commit()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

/**
	@brief Gets the Nth device on the interface

	@throw JtagException if the index is out of range

	@param device Device index

	@return The device object
 */
TestableDevice* TestInterface::GetDevice(unsigned int device)
{
	if(device >= m_devices.size())
	{
		throw JtagExceptionWrapper(
			"Device index out of range",
			"");
	}
	return m_devices[device];
}
