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
	@brief ARMv7-M Flash Patch/Breakpoint Unit
 */
#ifndef ARMFlashPatchBreakpoint_h
#define ARMFlashPatchBreakpoint_h

class ARMv7MProcessor;

class ARMFlashPatchBreakpoint : public ARMCoreSightDevice
{
public:
	ARMFlashPatchBreakpoint(
		ARMv7MProcessor* cpu,
		ARMDebugMemAccessPort* ap,
		uint32_t address,
		ARMDebugPeripheralIDRegisterBits idreg);
	virtual ~ARMFlashPatchBreakpoint();

	virtual std::string GetDescription();
	virtual void PrintInfo();

	enum FpbRegisters
	{
		FP_CTRL		= 0,
		FP_REMAP	= 1,
		FP_COMP0	= 2	//plus comparator number
	};

	uint32_t GetCodeComparatorIndex(uint32_t i)
	{ return FP_COMP0 + i; }

	uint32_t GetLiteralComparatorIndex(uint32_t i)
	{ return FP_COMP0 + m_codeComparators + i; }

	uint32_t GetCodeComparatorCount()
	{ return m_codeComparators; }

	uint32_t GetLiteralComparatorCount()
	{ return m_literalComparators; }

	void Enable();
	void Disable();

	void SetRemapTableBase(uint32_t base);
	void RemapFlashWord(uint32_t slot, uint32_t flashAddress, uint32_t newValue);
	//TODO: literal comparator support

protected:
	ARMv7MProcessor* m_cpu;

	uint32_t	m_codeComparators;
	uint32_t	m_literalComparators;
	bool		m_enabled;
	bool		m_canRemap;
	uint32_t	m_tableBase;
	uint32_t	m_sramBase;

	void ProbeStatusRegisters();
};

#endif
