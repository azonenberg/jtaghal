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
	@brief Declaration of JtagDevice
 */

#ifndef JtagDevice_h
#define JtagDevice_h

class JtagInterface;

#define RegisterConstant(c) m_constantMap[(#c)] = c

/**
	@brief Represents a single device in the JTAG chain

	\ingroup libjtaghal
 */
class JtagDevice
{
public:
	JtagDevice(uint32_t idcode, JtagInterface* iface, size_t pos, size_t irlength);
	virtual ~JtagDevice();

	/**
		@brief Gets a human-readable description of this device.

		Example: "Xilinx XC6SLX45 stepping 3"

		@return Device description
	 */
	virtual std::string GetDescription()=0;

	unsigned int GetIDCode();

	static JtagDevice* CreateDevice(uint32_t idcode, JtagInterface* iface, size_t pos);

	virtual void PrintInfo();

	/**
		@brief Does a post-initialization probe of the device to read debug ROMs etc.
	 */
	virtual void PostInitProbes() =0;

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

public:
	//JTAG interface helpers
	void SetIR(const unsigned char* data)
	{ SetIR(data, m_irlength); }

	void SetIRDeferred(const unsigned char* data)
	{ SetIRDeferred(data, m_irlength); }

	void SetIR(const unsigned char* data, int count);
	void SetIRDeferred(const unsigned char* data, int count);
	void SetIR(const unsigned char* data, unsigned char* data_out, int count);
	void ScanDR(const unsigned char* send_data, unsigned char* rcv_data, int count);
	void ScanDRDeferred(const unsigned char* send_data, int count);
	bool IsSplitScanSupported();
	void ScanDRSplitWrite(const unsigned char* send_data, unsigned char* rcv_data, int count);
	void ScanDRSplitRead(unsigned char* rcv_data, int count);
	void SendDummyClocks(int n);
	void SendDummyClocksDeferred(int n);
	void ResetToIdle();
	void Commit();

	size_t GetIRLength()
	{ return m_irlength; }

	size_t GetChainIndex()
	{ return m_pos; }

	uint32_t GetIdcode()
	{ return m_idcode; }

	void EnterShiftDR();
	void ShiftData(const unsigned char* send_data, unsigned char* rcv_data, int count);

protected:
	///Length of this device's instruction register, in bits
	size_t m_irlength;

	///32-bit JEDEC ID code of this device
	uint32_t m_idcode;

	///The JTAGInterface associated with this device
	JtagInterface* m_iface;

	///Position of this device in the interface's scan chain
	size_t m_pos;

	///Cached IR
	unsigned char m_cachedIR[4];

	//Map of constant names to values, used by jtagsh and other scripting support
	std::map<std::string, uint32_t> m_constantMap;
};

#endif
