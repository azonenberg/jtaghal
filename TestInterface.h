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
	@brief Declaration of TestInterface
 */

#ifndef TestInterface_h
#define TestInterface_h

class TestableDevice;

/**
	@brief An interface to an arbitrary test/debug protocol (may be JTAG, SWD, ICSP, etc)
 */
class TestInterface
{
public:
	TestInterface();
	virtual ~TestInterface();

	//Adapter info
public:
	/**
		@brief Gets the manufacturer-assigned name for this programming adapter.

		This is usually the model number but is sometimes something more generic like "Digilent Adept USB Device".

		@return The device name
	 */
	virtual std::string GetName() =0;

	/**
		@brief Gets the manufacturer-assigned serial number for this programming adapter, if any.

		This is a persistent, read-only string assigned by the manufacturer to identify this adapter.

		Derived classes may choose to return the user ID, an empty string, or another default value if no serial number
		has been assigned.

		@return The serial number
	 */
	virtual std::string GetSerial() =0;

	/**
		@brief Gets the user-assigned name for this programming adapter, if any.

		This is a persistent, user-writeable string ("hostname") used to identify this adapter.

		Derived classes may choose to return the serial number, an empty string, or another default value if no name
		has been assigned.

		@return The name for this adapter.
	 */
	virtual std::string GetUserID() =0;

	/**
		@brief Gets the clock frequency, in Hz, of the JTAG interface

		@return The clock frequency
	 */
	virtual int GetFrequency() =0;

	virtual void Commit();

	//Probing / enumeration of attached resources
public:
	virtual void InitializeChain(bool quiet = false) =0;		//name kept for compatibility with JTAG stuff
																//but we use the same function for non-JTAG interfaces

	/**
		@brief Returns the number of devices attached to the interface

		@return Device count
	 */
	size_t GetDeviceCount()
	{ return m_devices.size(); }

	TestableDevice* GetDevice(unsigned int device);

protected:

	///@brief Array of devices attached to this interface
	std::vector<TestableDevice*> m_devices;
};

#endif
