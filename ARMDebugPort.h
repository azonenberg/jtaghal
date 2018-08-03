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
	@brief Declaration of ARMDebugPort
 */

#ifndef ARMDebugPort_h
#define ARMDebugPort_h

/**
	@brief Base class for ARM debug ports (JTAG-DP, SWJ-DP, etc)

	\ingroup libjtaghal
 */
class ARMDebugPort		: public DebuggerInterface
{
public:
	ARMDebugPort();
	virtual ~ARMDebugPort();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Status

	virtual void PrintStatusRegister() =0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug register access via APB

	virtual uint32_t ReadDebugRegister(uint32_t address) =0;
	virtual void WriteDebugRegister(uint32_t address, uint32_t value) =0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// AP register access

	//Well-defined AP registers
	enum ApReg
	{
		REG_MEM_CSW		= 0x00,	//Control/status word
		REG_MEM_TAR		= 0x04,	//Transfer address register
		REG_MEM_DRW		= 0x0C,	//Data read/write
		REG_MEM_BASE	= 0xF8,	//Location of debug ROM

		REG_IDR			= 0xFC	//ID code
	};

protected:

	//need to be a friend so that the Mem-AP can poke registers
	//TODO: try to find a cleaner way to expose this?
	friend class ARMDebugMemAccessPort;
	virtual uint32_t APRegisterRead(uint8_t ap, ApReg addr) =0;
	virtual void APRegisterWrite(uint8_t ap, ApReg addr, uint32_t wdata) =0;
};

#endif
