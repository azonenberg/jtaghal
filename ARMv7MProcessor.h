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
	@brief Declaration of ARMv7MProcessor
 */

#ifndef ARMv7MProcessor_h
#define ARMv7MProcessor_h

class DebuggableDevice;
class ARMAPBDevice;

/**
	@brief An ARM Cortex-M CPU core supporting the ARMv7-M architecture, as seen over a CoreSight APB bus

	\ingroup libjtaghal
 */
class ARMv7MProcessor 	: public DebuggableDevice
						, public ARMAPBDevice
{
public:
	ARMv7MProcessor(ARMDebugMemAccessPort* ap, uint32_t address, ARMDebugPeripheralIDRegisterBits idreg);
	virtual ~ARMv7MProcessor();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Registers

	//register numbers, multiply by 4 to get address
	enum ARM_V7M_SCS_REGISTERS
	{
		ACTLR		= 0x0002,	//Auxiliary Control Register
		STCSR		= 0x0004,	//SysTick Control/Status Register
		STRVR		= 0x0005,	//SysTick Reload Value Register
		STCVR		= 0x0006,	//SysTick Current Value Register
		STCR		= 0x0007,	//SysTick Calibration Value Register
		CPUID		= 0x0340,	//CPU identifier
		ICSR		= 0x0341,	//Interrupt Control and State Register
		VTOR		= 0x0342,	//Vector Table Offset Register
		AIRCR		= 0x0343,	//Application Interrupt and Reset Control Register
		SCR			= 0x0344,	//System Control Register
		CCR			= 0x0345,	//Configuration and Control Register
		SHPR1		= 0x0346,	//System Handler Priority Register 1
		SHPR2		= 0x0347,	//System Handler Priority Register 2
		SHPR3		= 0x0348,	//System Handler Priority Register 3
		SHCSR		= 0x0349,	//System Handler Control and State Register
		CFSR		= 0x034a,	//Configurable Fault Status Register
		HFSR		= 0x034b,	//Hard Fault Status Register
		DFSR		= 0x034c,	//Debug Fault Status Register
		MMFAR		= 0x034d,	//MemManage Address Register
		BFAR		= 0x034e,	//BusFault Address Register
		AFSR		= 0x034f,	//Auxiliary Fault Status Register
		ID_PFR0		= 0x0350,	//Processor Feature Register 0
		ID_PFR1		= 0x0351,	//Processor Feature Register 1
		ID_DFR0		= 0x0352,	//Debug Features Register 0
		ID_AFR0		= 0x0353,	//Auxiliary Features Register 0
		ID_MMFR0	= 0x0354,	//Memory Model Features Register 0
		ID_MMFR1	= 0x0355,	//Memory Model Features Register 1
		ID_MMFR2	= 0x0356,	//Memory Model Features Register 2
		ID_MMFR3	= 0x0357,	//Memory Model Features Register 3
		ID_ISAR0	= 0x0358,	//Instruction Set Attributes Register 0
		ID_ISAR1	= 0x0359,	//Instruction Set Attributes Register 1
		ID_ISAR2	= 0x035a,	//Instruction Set Attributes Register 2
		ID_ISAR3	= 0x035b,	//Instruction Set Attributes Register 3
		ID_ISAR4	= 0x035c,	//Instruction Set Attributes Register 4
		CPACR		= 0x0362,	//Coprocessor Access Control Register
		DHCSR		= 0x037c,	//Debug Halting Control/Status Register
		DCRSR		= 0x037d,	//Debug Core Register Selector
		DCRDR		= 0x037e,	//Debug Core Register Data Register
		DEMCR		= 0x037f,	//Debug Exception Monitor Control Register
		STIR		= 0x03c0	//Software Triggered Interrupt Register
	};

	//register numbers for DCRSR
	enum ARM_V7M_CPU_REGISTERS
	{
		R0		= 0,
		R1		= 1,
		R2		= 2,
		R3		= 3,
		R4		= 4,
		R5		= 5,
		R6		= 6,
		R7		= 7,
		R8 		= 8,
		R9		= 9,
		R10		= 10,
		R11		= 11,
		R12		= 12,
		SP		= 13,	//current stack pointer
		LR		= 14,	//link register
		DBGRA	= 15,	//debug return address
		XPSR	= 16,	//flags etc
		MSP		= 17,	//main SP
		PSP		= 18,	//process SP
		CTRL	= 20	//{CONTROL, FAULTMASK, BASEPRI, PRIMASK}
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription() =0;
	virtual void PrintInfo() =0;

	bool HaltedDueToUnrecoverableException();

	void DumpRegisters();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory access via the default (AHB) MEM-AP

	virtual uint32_t ReadMemory(uint32_t addr);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CPU register access

	uint32_t ReadCPURegister(ARM_V7M_CPU_REGISTERS reg);
	const char* GetRegisterName(ARM_V7M_CPU_REGISTERS reg);

protected:
	//void PrintIDRegister(ARMv7MDebugIDRegister did);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debugger control

	void EnterDebugState();
	void ExitDebugState();

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

	ARMv7MDebugIDRegister m_deviceID;

	///Device-dependent address of the program counter sample register (PCSR)
	ARM_V7_DEBUG_REGISTERS m_pcsrIndex;
	*/
};

#endif


