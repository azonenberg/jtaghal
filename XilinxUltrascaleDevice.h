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
	@brief Declaration of XilinxUltrascaleDevice
 */

#ifndef XilinxUltrascaleDevice_h
#define XilinxUltrascaleDevice_h

/**
	@brief UltraScale configuration frame (see UG570 page 158)

	Same as 7 series

	\ingroup libjtaghal
 */
union XilinxUltrascaleDeviceConfigurationFrame
{
	struct
	{
		/**
			@brief Count field
		 */
		unsigned int count:11;

		///Reserved, must be zero
		unsigned int reserved:2;

		///Register address
		unsigned int reg_addr:14;

		/**
			@brief Opcode

			Must be one of the following:
			\li	XilinxUltrascaleDevice::CONFIG_OP_NOP
			\li XilinxUltrascaleDevice::CONFIG_OP_READ
			\li XilinxUltrascaleDevice::CONFIG_OP_WRITE
		 */
		unsigned int op:2;

		/**
			@brief Frame type

			Must be XilinxUltrascaleDevice::CONFIG_FRAME_TYPE_1
		 */
		unsigned int type:3;
	} __attribute__ ((packed)) bits;

	struct
	{
		/**
			@brief Count field
		 */
		unsigned int count:27;

		/**
			@brief Opcode

			Must be zero
		 */
		unsigned int op:2;

		/**
			@brief Frame type

			Must be XilinxUltrascaleDevice::CONFIG_FRAME_TYPE_2
		 */
		unsigned int type:3;
	} __attribute__ ((packed)) bits_type2;

	/// The raw configuration word
	uint32_t word;
} __attribute__ ((packed));

/**
	@brief UltraScale status register (see UG570 table 9-25)

	Very similar to the 7-series status register but with a few fields renamed.

	\ingroup libjtaghal
 */
union XilinxUltrascaleDeviceStatusRegister
{
	struct
	{
		///Indicates that the device failed to configure due to a CRC error
		unsigned int crc_err:1;

		///Indicates that the crypto subsystem is active
		unsigned int decryptor_enabled:1;

		///Indicates MMCMs and PLLs are locked
		unsigned int mmcm_lock:1;

		///Indicates DCI is matched
		unsigned int dci_match:1;

		///End-of-Startup signal
		unsigned int eos:1;

		///Status of GTS_CFG net
		unsigned int gts_cfg_b:1;

		///Status of GWE net
		unsigned int gwe:1;

		///Status of GHIGH_B net
		unsigned int ghigh_b:1;

		///Status of mode pins
		unsigned int mode_pins:3;

		///Internal init-finished signal
		unsigned int init_complete:1;

		///Status of INIT_B pin
		unsigned int init_b:1;

		///Indicates DONE was released
		unsigned int release_done:1;

		///Actual value on DONE pin
		unsigned int done:1;

		///Indicates an ID code error occurred (write with wrong bitstream)
		unsigned int id_error:1;

		///Security / crypto error
		unsigned int security_error:1;

		///Indicates board is too hot
		unsigned int sysmon_over_temp:1;

		///Status of startup state machine
		unsigned int startup_state:3;

		///Reserved
		unsigned int reserved_1:4;

		///Config bus width (see table 5-26)
		unsigned int bus_width:2;

		///Reserved
		unsigned int reserved_2:5;

	} __attribute__ ((packed)) bits;

	///The raw status register value
	uint32_t word;
} __attribute__ ((packed));

/**
	@brief A Xilinx Ultrascale or Ultrascale+ FPGA device

	\ingroup libjtaghal
 */
class XilinxUltrascaleDevice	: public XilinxFPGA
{
public:

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Construction / destruction
	XilinxUltrascaleDevice(
		unsigned int arraysize,
		unsigned int family,
		unsigned int rev,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);
	virtual ~XilinxUltrascaleDevice();

	static JtagDevice* CreateDevice(
		unsigned int arraysize,
		unsigned int family,
		unsigned int rev,
		unsigned int idcode,
		JtagInterface* iface,
		size_t pos);

	///JTAG device IDs
	enum deviceids
	{
		//XCVUxP
		VUPLUS_9		= 0x131,
	};

	///6-bit-wide JTAG instructions (see BSDL file). Seems to be same as 7 series
	//except for SLR_BYPASS (my name, there's no official name in the docs that I can find)
	//(TODO: is this the same as for SLR-based 7 series?)
	enum instructions
	{
		///Turn off the JTAG subsystem for a SLR we're not talking to
		INST_SLR_BYPASS			= 0x24,

		/*
		///User-defined instruction 1
		INST_USER1				= 0x02,

		///User-defined instruction 2
		INST_USER2				= 0x03,

		///User-defined instruction 3
		///Not same as Spartan-6
		INST_USER3				= 0x22,

		///User-defined instruction 4
		///Not same as Spartan-6
		INST_USER4				= 0x23,
		*/
		///Read configuration register
		INST_CFG_OUT			= 0x04,

		///Write configuration register
		INST_CFG_IN				= 0x05,

		/*
		///Read user ID code
		INST_USERCODE			= 0x08,

		///Read ID code
		INST_IDCODE				= 0x09,
		*/
		///Enters programming mode (erases FPGA configuration)
		INST_JPROGRAM			= 0x0B,

		///Runs the FPGA startup sequence (must supply dummy clocks after)
		INST_JSTART				= 0x0C,

		///Runs the FPGA shutdown sequence (must supply dummy clocks after)
		//INST_JSHUTDOWN			= 0x0D,

		///Enters In-System Configuration mode (must load INST_JPROGRAM before)
		INST_ISC_ENABLE			= 0x10,

		///Leaves In-System Configuration mode
		INST_ISC_DISABLE		= 0x16,

		///Read device DNA (must load INST_ISC_ENABLE before and INST_ISC_DISABLE after)
		///Same as 7-series but not same as Spartan-6
		INST_XSC_DNA			= 0x17,

		///Access to the ADC
		///Not present in Spartan-6
		//INST_XADC_DRP			= 0x37,

		///Standard JTAG bypass
		INST_BYPASS				= 0x3F
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General device info

	virtual std::string GetDescription();
	virtual void PrintStatusRegister();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// General programmable device properties

	virtual bool IsProgrammed();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FPGA-specific device properties

	virtual bool HasSerialNumber();
	virtual int GetSerialNumberLength();
	virtual int GetSerialNumberLengthBits();
	virtual void GetSerialNumber(unsigned char* data);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// On-chip debug helpers

	virtual size_t GetNumUserInstructions();
	virtual void SelectUserInstruction(size_t index);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Public configuration interface
public:
	virtual void Erase();
	virtual void InternalErase();
	virtual FirmwareImage* LoadFirmwareImage(const unsigned char* data, size_t len);
	virtual void Program(FirmwareImage* image);

	virtual void Reboot();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Internal configuration helpers
protected:
	uint32_t ReadWordConfigRegister(unsigned int reg);

	virtual XilinxFPGABitstream* ParseBitstreamInternals(
		const unsigned char* data,
		size_t len,
		XilinxFPGABitstream* bitstream,
		size_t fpos);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration type definitions
protected:

	/**
		@brief UltraScale configuration opcodes (see UG570 page 159). Same as for Spartan-6 and 7 series
	 */
	enum config_opcodes
	{
		CONFIG_OP_NOP	= 0,
		CONFIG_OP_READ	= 1,
		CONFIG_OP_WRITE	= 2
	};

	/**
		@brief UltraScale configuration frame types (see UG570 page 158). Same as for Spartan-6 and 7 series
	 */
	enum config_frame_types
	{
		CONFIG_FRAME_TYPE_1 = 1,
		CONFIG_FRAME_TYPE_2 = 2
	};

	/**
		@brief UltraScale configuration registers (see UG570 page 159). Seems to be same as 7 series.
	 */
	enum ultrascale_config_regs
	{
		CONFIG_REG_CRC		= 0x00,
		CONFIG_REG_FAR		= 0x01,
		CONFIG_REG_FDRI		= 0x02,
		CONFIG_REG_FDRO		= 0x03,
		CONFIG_REG_CMD		= 0x04,
		CONFIG_REG_CTL0		= 0x05,
		CONFIG_REG_MASK		= 0x06,
		CONFIG_REG_STAT		= 0x07,
		CONFIG_REG_LOUT		= 0x08,
		CONFIG_REG_COR0		= 0x09,
		CONFIG_REG_MFWR		= 0x0A,
		CONFIG_REG_CBC		= 0x0B,
		CONFIG_REG_IDCODE	= 0x0C,
		CONFIG_REG_AXSS		= 0x0D,
		CONFIG_REG_COR1		= 0x0E,
		//0x0F reserved or usused
		CONFIG_REG_WBSTAR	= 0x10,
		CONFIG_REG_TIMER		= 0x11,
		//0x12 reserved or unused
		//0x13 reserved. Vivado seems to write 0x00000000 to it. Wonder what it does?
		//0x14 reserved or unused
		//0x15 reserved or unused
		CONFIG_REG_BOOTSTS	= 0x16,
		//0x17 reserved or unused
		CONFIG_REG_CTL1		= 0x18,
		//0x19 and up reserved or unused

		CONFIG_REG_BSPI		= 0x1F,

		CONFIG_REG_MAX		//max config reg value
	};

	/**
		@brief UltraScale CMD register values (see UG570 page table 9-22).

		Seems to be  mostly same as 7 series but a few things changed (commented)
	 */
	enum cmd_values
	{
		CMD_NULL		= 0x00,
		CMD_WCFG		= 0x01,
		CMD_MFW			= 0x02,
		CMD_LFRM		= 0x03,	//also DGHIGH
		CMD_RCFG		= 0x04,
		CMD_START		= 0x05,
		//0x06 is reserved, was CMD_RCAP in 7 series
		CMD_RCRC		= 0x07,
		CMD_AGHIGH		= 0x08,
		CMD_SWITCH		= 0x09,
		CMD_GRESTORE	= 0x0a,
		CMD_SHUTDOWN	= 0x0b,
		//0x0c is reserved, was CMD_GCAPTURE in 7 series
		CMD_DESYNC		= 0x0d,
		//0x0e is reserved
		CMD_IPROG		= 0x0f,
		CMD_CRCC		= 0x10,
		CMD_LTIMER		= 0x11,
		CMD_BSPI_READ	= 0x12,	//new in ultrascale, restart bitstream read from flash)
		CMD_FALL_EDGE	= 0x13,	//new in ultrascale, switch to capturing bitstream data on negedge CCLK
		CMD_MAX
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helpers for chain manipulation

protected:

	void SetIRForMasterSLR(unsigned char irval, bool defer = false);
	void SetIRForAllSLRs(unsigned char irval, bool defer = false);

protected:

	///Array size (the specific device we are)
	unsigned int m_arraysize;

	///Family (Ultrascale or Ultrascale+)
	unsigned int m_family;

	///Stepping number
	unsigned int m_rev;

	///Number of SLRs in the device
	unsigned int m_slrCount;

	///Index of the master SLR (zero-based). Always 0 for monolithic devices.
	unsigned int m_masterSLR;
};

#endif
