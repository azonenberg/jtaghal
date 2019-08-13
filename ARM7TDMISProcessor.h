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
	@brief Declaration of ARM7TDMISProcessor
 */

#ifndef ARM7TDMISProcessor_h
#define ARM7TDMISProcessor_h

class DebuggableDevice;
class ARMAPBDevice;

/**
	@brief An ARM7TDMI-S CPU core supporting the ARMv4 architecture, as seen over JTAG (no CoreSight support)

	\ingroup libjtaghal
 */
class ARM7TDMISProcessor 	: public DebuggableDevice
							, public ARMDevice
							, public ProgrammableDevice
{
public:
	ARM7TDMISProcessor(
		unsigned int partnum,
		unsigned int rev,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);
	virtual ~ARM7TDMISProcessor();

	//JTAG instructions
	enum JTAG_INSTRUCTIONS
	{
		SCAN_N	= 0x2,
		RESTART	= 0x4,
		INTEST 	= 0xc,
		IDCODE	= 0xe,
		BYPASS	= 0xf
	};

	virtual void PostInitProbes(bool quiet);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Internal debug stuff

protected:

	enum IceRegisters
	{
		DEBUG_CTRL		= 0x00,
		DEBUG_STAT		= 0x01,
		DCC_CTRL		= 0x04,
		DCC_DATA		= 0x05,
		WATCH0_ADDR		= 0x08,
		WATCH0_AMASK	= 0x09,
		WATCH0_DATA		= 0x0a,
		WATCH0_DMASK	= 0x0b,
		WATCH0_CTRL		= 0x0c,
		WATCH0_CMASK	= 0x0d,
		WATCH1_ADDR		= 0x10,
		WATCH1_AMASK	= 0x11,
		WATCH1_DATA		= 0x12,
		WATCH1_DMASK	= 0x13,
		WATCH1_CTRL		= 0x14,
		WATCH1_CMASK	= 0x15
	};

	void SelectScanChain(uint8_t n);

	void SelectDebugChain()
	{ SelectScanChain(1); }

	void SelectIceRTChain()
	{ SelectScanChain(2); }

	void WriteIceRegister(uint8_t reg, uint32_t value);
	uint32_t ReadIceRegister(uint8_t reg);

public:
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription();
	virtual void PrintInfo();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ProgrammableDevice

	virtual bool IsProgrammed();
	virtual FirmwareImage* LoadFirmwareImage(const unsigned char* data, size_t len);
	virtual void Erase();
	virtual void Program(FirmwareImage* image);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CPU register access
	/*
	uint32_t ReadCPURegister(ARM_V7M_CPU_REGISTERS reg);
	const char* GetRegisterName(ARM_V7M_CPU_REGISTERS reg);

protected:
	//void PrintIDRegister(ARM7TDMISDebugIDRegister did);

	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debugger control

	virtual void DebugHalt();
	virtual void DebugResume();

	virtual void Reset();

	virtual void PrintRegisters();

	virtual uint32_t ReadMemory(uint32_t addr);
	virtual void WriteMemory(uint32_t addr, uint32_t value);

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ID register state etc

	unsigned int m_breakpoints;
	unsigned int m_context_breakpoints;
	unsigned int m_watchpoints;
	bool m_hasDevid;
	bool m_hasSecExt;
	bool m_hasSecureHalt;
	unsigned int m_revision;
	unsigned int m_variant;
	//TODO: arch version

	ARM7TDMISDebugIDRegister m_deviceID;

	///Device-dependent address of the program counter sample register (PCSR)
	ARM_V7_DEBUG_REGISTERS m_pcsrIndex;
	*/

	unsigned int m_rev;

	unsigned int m_selectedChain;
};

#endif


