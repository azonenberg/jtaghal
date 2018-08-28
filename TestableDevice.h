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
	@brief Declaration of TestableDevice
 */

#ifndef TestableDevice_h
#define TestableDevice_h

#define RegisterConstant(c) m_constantMap[(#c)] = c

/**
	@brief A logical device connected to some sort of test interface (may or may not be JTAG).

	Every device class in libjtaghal should derive directly or indirectly from this class.

	A TestableDevice isn't good for much by itself; typically a user will use RTTI to figure out what interfaces
	the derived object implements and then cast to them.
 */
class TestableDevice
{
public:
	TestableDevice();
	virtual ~TestableDevice();

	virtual void PrintInfo() =0;

	/**
		@brief Gets a human-readable description of this device.

		Example: "Xilinx XC6SLX45 stepping 3"

		@return Device description
	 */
	virtual std::string GetDescription()=0;

	/**
		@brief Does a post-initialization probe of the device to read debug ROMs etc.

		@param quiet Do less noisy probing to reduce chance of triggering security lockdowns.
	 */
	virtual void PostInitProbes(bool quiet) =0;

	/**
		@brief Looks up the value for a named constant

		This is mostly used by jtagsh for allowing registers to be poked in a more human-friendly fashion.

		@param name		Name of the constant
		@param value	Value (if found)

		@return			True if found, false if not found

		TODO: consider refactoring this to a separate base class?
	 */
	bool LookupConstant(std::string name, uint32_t& value)
	{
		if(m_constantMap.find(name) != m_constantMap.end())
		{
			value = m_constantMap[name];
			return true;
		}
		else
			return false;
	}

protected:

	///Map of constant names to values, used by jtagsh and other scripting support
	std::map<std::string, uint32_t> m_constantMap;
};

#endif

