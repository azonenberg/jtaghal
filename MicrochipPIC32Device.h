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
	@brief Declaration of MicrochipPIC32Device
 */

#ifndef MicrochipPIC32Device_h
#define MicrochipPIC32Device_h

#include "MicrochipMicrocontroller.h"

#include <list>
#include <string>

/**
	@brief Status register for a Microchip PIC32 device

	\ingroup libjtaghal
 */
union MicrochipPIC32DeviceStatusRegister
{
	struct
	{
		unsigned int reset_active		:1;	//Device is being reset
		unsigned int flash_en			:1;	//Flash access enabled
		unsigned int flash_busy			:1;	//Flash controller is doing stuff
		unsigned int cfg_rdy			:1;	//Config is read, code protect status valid
		unsigned int reserved2			:1;	//unknown bit 4
		unsigned int nvm_error			:1;	//Error occurred
		unsigned int reserved1			:1;	//unknown bit 6
		unsigned int code_protect_off	:1;	//Code protect is NOT set
	} __attribute__ ((packed)) bits;

	///The raw status register value
	uint8_t word;
} __attribute__ ((packed));

/**
	@brief MIPS EJTAG implementation register

	\ingroup libjtaghal
 */
union EjtagImplementationCodeRegister
{
	struct
	{
		unsigned int processor_is_64	: 1;	//True if 64-bit CPU
		unsigned int reserved1			: 13;
		unsigned int no_ejtag_dma		: 1;	//Set means we don't do DMA over EJTAG
		unsigned int reserved2			: 1;
		unsigned int mips16_supported	: 1;
		unsigned int reserved3			: 4;
		unsigned int asid_size 			: 2;
		unsigned int reserved4			: 1;
		unsigned int dint_supported		: 1;
		unsigned int reserved5			: 3;
		unsigned int r3k_priv			: 1;	//R3000 privilege mode if set, R4000 if not set
		unsigned int ejtag_version		: 3;
	} __attribute__ ((packed)) bits;

	///The raw status register value
	uint32_t word;
} __attribute__ ((packed));

/**
	@brief PIC32 EJTAG control register
 */
union EjtagControlRegister
{
	struct
	{
		unsigned int reserved1			: 3;
		unsigned int debug_mode			: 1;
		unsigned int reserved2			: 8;
		unsigned int debug_irq			: 1;	//Set true to request a debug exception
		unsigned int reserved3			: 1;
		unsigned int debug_vector_pos	: 1;	//True = dmseg, false = normal memory
		unsigned int probe_enable		: 1;	//Set true to have debug probe service dmseg requests
		unsigned int proc_reset			: 1;
		unsigned int reserved4			: 1;
		unsigned int proc_access		: 1;	//true if pending CPU memory access
		unsigned int proc_we			: 1;	//true if pending CPU access is a write
		unsigned int periph_reset		: 1;
		unsigned int bus_halted			: 1;
		unsigned int low_power			: 1;
		unsigned int vpe_disable		: 1;
		unsigned int reserved5			: 5;
		unsigned int access_size		: 2;	//Pending CPU access size
												//0 = byte, 1 = half, 2 = word
		unsigned int reset_occurred		: 1;
	} __attribute__ ((packed)) bits;

	///The raw status register value
	uint32_t word;
} __attribute__ ((packed));

/**
	@brief Internal data structure storing properties of a single SKU in the PIC32 family
 */
struct MicrochipPIC32DeviceInfo
{
	///JTAG device ID
	uint16_t devid;

	///String name of device
	const char* name;

	///Device family
	unsigned int family;

	///CPU type
	unsigned int cpu;

	///SRAM capacity (kB)
	unsigned int sram_size;

	///Main program flash size (kB)
	unsigned int program_flash_size;

	///Boot flash size (kB)
	float boot_flash_size;
};

/**
	@brief A Microchip PIC32 microcontroller (MX, MZ, MM, etc)

	\ingroup libjtaghal
 */
class MicrochipPIC32Device	: public MicrochipMicrocontroller
{
public:
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Construction / destruction
	MicrochipPIC32Device(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);
	virtual ~MicrochipPIC32Device();

	static JtagDevice* CreateDevice(
		unsigned int devid,
		unsigned int stepping,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);

	virtual void PostInitProbes(bool quiet);

	///Device families
	enum families
	{
		FAMILY_MX12,		//PIC32MX 1xx/2xx
		FAMILY_MX34,		//PIC32MX 3xx/4xx
		FAMILY_MX567,		//PIC32MX 5xx/6xx/7xx

		FAMILY_MM,			//All PIC32MM devices

		FAMILY_MZ			//All PIC32MZ devices
	};

	///CPU types
	enum cpus
	{
		CPU_M4K,
		CPU_MAPTIV
	};

	///JTAG device IDs (from BSDL files and/or flash programming spec)
	enum deviceids
	{
		PIC32MX110F016B = 0x4a07,
		PIC32MX110F016C = 0x4a09,
		PIC32MX110F016D = 0x4a0b,
		PIC32MX120F032B = 0x4a06,
		PIC32MX120F032C = 0x4a08,
		PIC32MX120F032D = 0x4a0a,
		PIC32MX130F064B = 0x4d07,
		PIC32MX130F064C = 0x4d09,
		PIC32MX130F064D = 0x4d0b,
		PIC32MX150F128B = 0x4d06,
		PIC32MX150F128C = 0x4d08,
		PIC32MX150F128D = 0x4d0a,
		PIC32MX210F016B = 0x4a01,
		PIC32MX210F016C = 0x4a03,
		PIC32MX210F016D = 0x4a05,
		PIC32MX220F032B = 0x4a00,
		PIC32MX220F032C = 0x4a02,
		PIC32MX220F032D = 0x4a04,
		PIC32MX230F064B = 0x4d01,
		PIC32MX230F064C = 0x4d03,
		PIC32MX230F064D = 0x4d05,
		PIC32MX250F128B = 0x4d00,
		PIC32MX250F128C = 0x4d02,
		PIC32MX250F128D = 0x4d04,
		PIC32MX330F064H = 0x5600,
		PIC32MX330F064L = 0x5601,
		PIC32MX340F512H = 0x0916,
		PIC32MX350F128H = 0x570c,
		//PIC32MX350F128L = 0x570d,	//350F128L and 350F256H have same IDCODE... BSDL error?
		PIC32MX350F256H = 0x570d,
		PIC32MX350F256L = 0x5705,
		PIC32MX430F064H = 0x5602,
		PIC32MX430F064L = 0x5603,
		PIC32MX450F128H = 0x570e,
		PIC32MX450F128L = 0x570f,
		PIC32MX450F256H = 0x5706,
		PIC32MX450F256L = 0x5707,
		PIC32MX534F064H = 0x440c,	//H and L have same IDCODE... BSDL error?
		//PIC32MX534F064L = 0x440c,
		PIC32MX564F064H = 0x4401,
		PIC32MX564F064L = 0x440d,
		PIC32MX564F128H = 0x4403,
		PIC32MX564F128L = 0x440f,
		PIC32MX664F064H = 0x4405,
		PIC32MX664F064L = 0x4411,
		PIC32MX664F128H = 0x4407,
		PIC32MX664F128L = 0x4413,
		PIC32MX695F512L = 0x4341,
		PIC32MX764F128H = 0x440b,
		PIC32MX764F128L = 0x4417,
		PIC32MX795F512L = 0x4307,

		PIC32MM0016GPL020	= 0x6b04,
		PIC32MM0032GPL020	= 0x6b0c,
		PIC32MM0064GPL020	= 0x6b14,
		PIC32MM0016GPL028	= 0x6b02,
		PIC32MM0032GPL028	= 0x6b0a,
		PIC32MM0064GPL028	= 0x6b12,
		PIC32MM0016GPL036	= 0x6b06,
		PIC32MM0032GPL036	= 0x6b0b,
		PIC32MM0064GPL036	= 0x6b16,
		PIC32MM0064GPM028	= 0x7708,
		PIC32MM0128GPM028	= 0x7710,
		PIC32MM0256GPM028	= 0x7718,
		PIC32MM0064GPM036	= 0x770a,
		PIC32MM0128GPM036	= 0x7712,
		PIC32MM0256GPM036	= 0x771a,
		PIC32MM0064GPM048	= 0x772c,
		PIC32MM0128GPM048	= 0x7734,
		PIC32MM0256GPM048	= 0x773c,
		PIC32MM0064GPM064	= 0x770e,
		PIC32MM0128GPM064	= 0x7716,
		PIC32MM0256GPM064	= 0x771e
	};

	///5-bit-wide JTAG instructions (from BSDL file and datasheet)
	enum instructions
	{
		///Standard JTAG bypass
		INST_BYPASS				= 0x1F,

		///Read ID code
		INST_IDCODE				= 0x01,

		///Read implementation code
		INST_IMPCODE			= 0x03,

		///Selects Microchip scan chain
		INST_MTAP_SW_MCHP		= 0x04,

		///Selects EJTAG scan chain
		INST_MTAP_SW_EJTAG		= 0x05,

		///Command to Microchip virtualized JTAG
		INST_MTAP_COMMAND		= 0x07,

		///Select address register for memory ops
		INST_ADDRESS			= 0x08,

		///Select data register for memory ops
		INST_DATA				= 0x09,

		///Control register of some sort?
		INST_CONTROL			= 0x0A,

		///Selects address, data, control end to end in one DR
		INST_ALL				= 0x0B,

		///Makes the CPU trap to debugger after a reset
		INST_DEBUGBOOT			= 0x0C,

		///Boot normally after a reset
		INST_NORMALBOOT			= 0x0D,

		///Register used for moving data to/from the debug bridge
		INST_FASTDATA			= 0x0E,

		//Sample the program counter (used for profiling... not implemented?)
		INST_PCSAMPLE			= 0x14
	};

	///8-bit instructions for Microchip virtual TAP (write to INST_MTAP_COMMAND data register)
	enum mtap_instructions
	{
		///Get status
		MCHP_STATUS				= 0x00,

		///Begin chip reset
		MCHP_ASSERT_RST			= 0xD1,

		///End chip reset
		MCHP_DE_ASSERT_RST		= 0xD0,

		///Bulk-erase flash
		MCHP_ERASE				= 0xFC,

		///Enable connecting the CPU to flash
		MCHP_FLASH_ENABLE		= 0xFE,

		///Disconnect the CPU from flash
		MCHP_FLASH_DISABLE		= 0xFD,

		///Force re-read of device config
		MCHP_READ_CONFIG		= 0xFF
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MCU stuff

	virtual bool IsProgrammed();
	virtual void Erase();

	virtual void Reset();

	virtual void Program(FirmwareImage* image);

	//MicrochipPIC32DeviceStatusRegister GetStatusRegister();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helpers for chain manipulation
public:
	void SetIR(unsigned char irval)
	{ JtagDevice::SetIR(&irval, m_irlength); }

protected:
	void EnterMtapMode();
	uint8_t SendMchpCommand(uint8_t cmd);
	void EnterEjtagMode();
	void EnterSerialExecMode();
	void SerialExecuteInstruction(uint32_t insn, bool first = false);
	void SerialExecuteMemoryWrite(uint32_t addr, uint32_t data);
	uint32_t SerialExecuteMemoryRead(uint32_t addr);
	EjtagControlRegister WaitForEjtagMemoryOperation(bool first = false);

	void SerialExecHelper();

	MicrochipPIC32DeviceStatusRegister GetStatus();
	EjtagImplementationCodeRegister GetImpCode();

protected:

	///Device ID code
	unsigned int m_devid;

	///Stepping number
	unsigned int m_stepping;

	///Device info
	const MicrochipPIC32DeviceInfo* m_devinfo;
};

#endif
