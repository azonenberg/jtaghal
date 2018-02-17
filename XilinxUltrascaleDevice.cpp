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
	@brief Implementation of XilinxUltrascaleDevice
 */
#include "jtaghal.h"
#include "XilinxDeviceID_enum.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Initializes this device

	@param arraysize	Array size from JTAG ID code
	@param family		Family from JTAG ID code (needed since this class handles both ultrascale and ultrascale+)
	@param rev			Revision number from JTAG ID code
	@param idcode		The ID code of this device
	@param iface		The JTAG adapter this device was discovered on
	@param pos			Position in the chain that this device was discovered
 */
XilinxUltrascaleDevice::XilinxUltrascaleDevice(
	unsigned int arraysize,
	unsigned int family,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
: XilinxFPGA(idcode, iface, pos)
, m_arraysize(arraysize)
, m_family(family)
, m_rev(rev)
{
	//TODO: ultrascale
	if(m_family == XILINX_FAMILY_ULTRASCALE)
	{
		throw JtagExceptionWrapper(
			"Ultrascale SLR stuff not implemented",
			"");
	}

	//Figure out IR configuration for Ultrascale+ devices
	else
	{
		switch(arraysize)
		{
			case VUPLUS_9:
				m_slrCount = 3;
				m_masterSLR = 1;
				break;

			default:
				throw JtagExceptionWrapper(
					"Unknown UltraScale+ device (ID code not in database)",
					"");
		}
	}

	//Each SLR has its own instruction register but they're concatenated
	m_irlength = 6 * m_slrCount;
}

/**
	@brief Empty virtual destructor
 */
XilinxUltrascaleDevice::~XilinxUltrascaleDevice()
{
}

/**
	@brief Factory method
 */
JtagDevice* XilinxUltrascaleDevice::CreateDevice(
	unsigned int arraysize,
	unsigned int family,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
{
	if(family == XILINX_FAMILY_ULTRASCALE)
	{
		throw JtagExceptionWrapper(
			"Unknown UltraScale device (ID code not in database)",
			"");
	}

	else if(family == XILINX_FAMILY_USPLUS)
	{
		switch(arraysize)
		{
			case VUPLUS_9:
				return new XilinxUltrascaleDevice(arraysize, family, rev, idcode, iface, pos);

			default:
				throw JtagExceptionWrapper(
					"Unknown UltraScale+ device (ID code not in database)",
					"");
		}
	}

	else
	{
		throw JtagExceptionWrapper(
			"Unknown device passed to XilinxUltrascaleDevice::CreateDevice (neither UltraScale or UltraScale+)",
			"");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generic device information

string XilinxUltrascaleDevice::GetDescription()
{
	string devname = "(unknown UltraScale or UltraScale+)";

	if(m_family == XILINX_FAMILY_ULTRASCALE)
	{
	}

	//ultrascale+
	else
	{
		switch(m_arraysize)
		{
			case VUPLUS_9:
				devname = "XCVU9P";
				break;

			throw JtagExceptionWrapper(
				"Unknown UltraScale+ device (ID code not in database)",
				"");
		}
	}

	char sout[128];
	snprintf(sout, sizeof(sout), "Xilinx %s stepping %u (%d SLRs)", devname.c_str(), m_rev, m_slrCount);
	return string(sout);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA-specific device properties

bool XilinxUltrascaleDevice::HasSerialNumber()
{
	return true;
}

int XilinxUltrascaleDevice::GetSerialNumberLength()
{
	return 12;
}

int XilinxUltrascaleDevice::GetSerialNumberLengthBits()
{
	return 96;
}

void XilinxUltrascaleDevice::GetSerialNumber(unsigned char* data)
{
	InternalErase();

	//Enter ISC mode (wipes configuration)
	ResetToIdle();

	SetIRForAllSLRs(INST_ISC_ENABLE);

	//Read the DNA value
	SetIRForMasterSLR(INST_XSC_DNA);
	unsigned char zeros[12] = {0x00};
	ScanDR(zeros, data, 96);

	//Done
	SetIRForAllSLRs(INST_ISC_DISABLE);
}

bool XilinxUltrascaleDevice::IsProgrammed()
{
	XilinxUltrascaleDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
	return statreg.bits.done;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration

void XilinxUltrascaleDevice::SetIRForMasterSLR(unsigned char irval, bool defer)
{
	//Build the IR
	uint64_t final_op = 0;
	for(unsigned int i=0; i<m_slrCount; i++)
	{
		//Default to skipping this one
		uint32_t op = INST_SLR_BYPASS;

		if(i == m_masterSLR)
			op = irval;

		final_op = (final_op << 6) | op;
	}

	//Send it (this assumes a little endian host)
	if(defer)
		JtagDevice::SetIRDeferred((const unsigned char*)&final_op, m_irlength);
	else
		JtagDevice::SetIR((const unsigned char*)&final_op, m_irlength);
}

void XilinxUltrascaleDevice::SetIRForAllSLRs(unsigned char irval, bool defer)
{
	//Build the IR
	uint64_t final_op = 0;
	for(unsigned int i=0; i<m_slrCount; i++)
	{
		//Default to skipping this one
		uint32_t op = INST_SLR_BYPASS;

		if(i == m_masterSLR)
			op = irval;

		final_op = (final_op << 6) | op;
	}

	//Send it (this assumes a little endian host)
	if(defer)
		JtagDevice::SetIRDeferred((const unsigned char*)&final_op, m_irlength);
	else
		JtagDevice::SetIR((const unsigned char*)&final_op, m_irlength);
}

void XilinxUltrascaleDevice::Erase()
{
	ResetToIdle();
	InternalErase();
	SetIRForAllSLRs(INST_BYPASS);
}

void XilinxUltrascaleDevice::InternalErase()
{
	//Load the JPROGRAM instruction to clear configuration memory
	LogDebug("Clearing configuration memory...\n");
	SetIRForAllSLRs(INST_JPROGRAM);
	SendDummyClocks(32);

	//Poll status register until housecleaning is done
	XilinxUltrascaleDeviceStatusRegister statreg;
	int i;
	for(i=0; i<10; i++)
	{
		statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
		if(statreg.bits.init_b)
			break;

		//wait 500ms
		usleep(500 * 1000);
	}
	if(i == 10)
	{
		throw JtagExceptionWrapper(
			"INIT_B not high after 5 seconds, possible board fault",
			"");
	}
}


FirmwareImage* XilinxUltrascaleDevice::LoadFirmwareImage(const unsigned char* data, size_t len)
{
	return static_cast<FirmwareImage*>(XilinxFPGA::ParseBitstreamCore(data, len));
}

void XilinxUltrascaleDevice::Program(FirmwareImage* image)
{
	LogIndenter li;

	//Should be an FPGA bitstream
	FPGABitstream* bitstream = static_cast<FPGABitstream*>(image);
	if(bitstream == NULL)
	{
		throw JtagExceptionWrapper(
			"Invalid firmware image (not an FPGABitstream)",
			"");
	}

	throw JtagExceptionWrapper(
		"XilinxUltrascaleDevice::Program not implemented",
		"");

	/*
	//Verify the ID code matches the device we're plugged into
	//Mask out the stepping number since this is irrelevant
	if((0x0fffffff & bitstream->idcode) != (0x0fffffff & m_idcode) )
	{
		//Print verbose error message
		//TODO: sprintf and put in the exception?
		JtagDevice* dummy = JtagDevice::CreateDevice(bitstream->idcode, NULL, 0);
		if(dummy != NULL)
		{
			LogError("You are attempting to program a \"%s\" with a bitstream meant for a \"%s\"\n",
				GetDescription().c_str(), dummy->GetDescription().c_str());
			delete dummy;
		}

		throw JtagExceptionWrapper(
			"Bitstream ID code does not match ID code of this device",
			"");
	}
	LogVerbose("Checking ID code... OK\n");

	//If the ID code check passes it's a valid Xilinx bitstream so go do stuff with it
	XilinxFPGABitstream* xbit = dynamic_cast<XilinxFPGABitstream*>(bitstream);
	if(xbit == NULL)
	{
		throw JtagExceptionWrapper(
			"Valid ID code but not a XilinxFPGABitstream",
			"");
	}
	LogVerbose("Using %s\n", xbit->GetDescription().c_str());

	//Make a copy of the bitstream with the proper byte ordering for the JTAG API
	unsigned char* flipped_bitstream = new unsigned char[xbit->raw_bitstream_len];
	memcpy(flipped_bitstream, xbit->raw_bitstream, xbit->raw_bitstream_len);
	FlipBitArray(flipped_bitstream, xbit->raw_bitstream_len);

	//Reset the scan chain
	ResetToIdle();

	//Reset the FPGA to clear any existing configuration
	InternalErase();

	//Send the bitstream to the FPGA
	LogVerbose("Loading new bitstream...\n");
	SetIR(INST_CFG_IN);
	ScanDR(flipped_bitstream, NULL, xbit->raw_bitstream_len * 8);

	//Start up the FPGA
	SetIR(INST_JSTART);
	SendDummyClocks(64);

	//Clean up
	delete[] flipped_bitstream;
	flipped_bitstream = NULL;

	//Get the status register and verify that configuration was successful
	XilinxUltrascaleDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(X7_CONFIG_REG_STAT);
	if(statreg.bits.done != 1 || statreg.bits.gwe != 1)
	{
		if(statreg.bits.crc_err)
		{
			throw JtagExceptionWrapper(
				"Configuration failed (CRC error)",
				"");
		}
		else
		{
			PrintStatusRegister();
			throw JtagExceptionWrapper(
				"Configuration failed (unknown error)",
				"");
		}
	}
	*/
	ResetToIdle();
}

void XilinxUltrascaleDevice::PrintStatusRegister()
{
	XilinxUltrascaleDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
	LogIndenter li;

	LogVerbose(
		"Status register is 0x%04x\n"
		"  CRC error         : %u\n"
		"  Decrytor enabled  : %u\n"
		"  MMCM lock         : %u\n"
		"  DCI match         : %u\n"
		"  EOS               : %u\n"
		"  GTS               : %u\n"
		"  GWE               : %u\n"
		"  GHIGH             : %u\n"
		"  Mode pins         : %u\n"
		"  Init complete     : %u\n"
		"  INIT_B            : %u\n"
		"  Done released     : %u\n"
		"  Done pin          : %u\n"
		"  IDCODE error      : %u\n"
		"  Security error    : %u\n"
		"  SYSMON overheat   : %u\n"
		"  Startup state     : %u\n"
		"  Reserved          : %u\n"
		"  Bus width         : %u\n"
		"  Reserved          : %u\n",
		statreg.word,
		statreg.bits.crc_err,
		statreg.bits.decryptor_enabled,
		statreg.bits.mmcm_lock,
		statreg.bits.dci_match,
		statreg.bits.eos,
		statreg.bits.gts_cfg_b,
		statreg.bits.gwe,
		statreg.bits.ghigh_b,
		statreg.bits.mode_pins,
		statreg.bits.init_complete,
		statreg.bits.init_b,
		statreg.bits.release_done,
		statreg.bits.done,
		statreg.bits.id_error,
		statreg.bits.security_error,
		statreg.bits.sysmon_over_temp,
		statreg.bits.startup_state,
		statreg.bits.reserved_1,
		statreg.bits.bus_width,
		statreg.bits.reserved_2
		);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal configuration helpers

/**
	@brief Reads a single 32-bit word from a config register

	Reference: UG570 page 164

	Note that UltraScale devices expect data clocked in MSB first but the JTAG API clocks data LSB first.
	Some swapping is required as a result.

	Clock data into CFG_IN register
		Synchronization word
		Nop
		Read STAT register
		Two dummy words to flush packet buffer

	Read from CFG_OUT register

	@throw JtagException if the read fails

	@param reg The configuration register to read
 */
uint32_t XilinxUltrascaleDevice::ReadWordConfigRegister(unsigned int reg)
{
	//Send the read request
	//TODO: Should this be directed to the master SLR only, or all SLRs?
	SetIRForMasterSLR(INST_CFG_IN);
		XilinxUltrascaleDeviceConfigurationFrame frames[] =
	{
		{ word: 0xaa995566 },										//Sync word
		{ bits: {0, 0, 0,   CONFIG_OP_NOP,  CONFIG_FRAME_TYPE_1} },	//Dummy word
		{ bits: {1, 0, reg, CONFIG_OP_READ, CONFIG_FRAME_TYPE_1} },	//Read the register
		{ bits: {0, 0, 0,   CONFIG_OP_NOP,  CONFIG_FRAME_TYPE_1} },	//Two dummy words required
		{ bits: {0, 0, 0,   CONFIG_OP_NOP,  CONFIG_FRAME_TYPE_1} }
	};

	unsigned char* packet = (unsigned char*) frames;
	FlipBitAndEndian32Array(packet, sizeof(frames));		//MSB needs to get sent first
	ScanDR(packet, NULL, sizeof(frames) * 8);

	//Read the data
	SetIRForMasterSLR(INST_CFG_OUT);

	unsigned char unused[4] = {0};
	uint32_t reg_out = {0};
	ScanDR(unused, (unsigned char*)&reg_out, 32);
	FlipBitAndEndian32Array((unsigned char*)&reg_out, 4);

	SetIRForAllSLRs(INST_BYPASS);
	return reg_out;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bitstream parsing

XilinxFPGABitstream* XilinxUltrascaleDevice::ParseBitstreamInternals(
	const unsigned char* data,
	size_t len,
	XilinxFPGABitstream* bitstream,
	size_t fpos)
{
	LogDebug("Parsing bitstream internals...\n");

	//Expect aa 99 55 66 (sync word)
	unsigned char syncword[4] = {0xaa, 0x99, 0x55, 0x66};
	if(0 != memcmp(data+fpos, syncword, sizeof(syncword)))
	{
		delete bitstream;
		throw JtagExceptionWrapper(
			"No valid sync word found",
			"");
	}

	//Allocate space and copy the entire bitstream into raw_bitstream
	bitstream->raw_bitstream_len = len-fpos;
	bitstream->raw_bitstream = new unsigned char[bitstream->raw_bitstream_len];
	memcpy(bitstream->raw_bitstream, data+fpos, bitstream->raw_bitstream_len);

	//Skip the sync word
	fpos += sizeof(syncword);

	throw JtagExceptionWrapper(
		"XilinxUltrascaleDevice::ParseBitstreamInternals not implemented",
		"");

	/*

	//String names for config regs
	static const char* config_register_names[X7_CONFIG_REG_MAX]=
	{
		"CRC",
		"FAR",
		"FDRI",
		"FDRO",
		"CMD",
		"CTL0",
		"MASK",
		"STAT",
		"LOUT",
		"COR0",
		"MFWR",
		"CBC",
		"IDCODE",
		"AXSS",
		"COR1",
		"RESERVED_0F",
		"WBSTAR",
		"TIMER",
		"RESERVED_12",
		"RESERVED_13",
		"RESERVED_14",
		"RESERVED_15",
		"BOOTSTS",
		"RESERVED_17",
		"CTL1",
		"RESERVED_19",
		"RESERVED_1A",
		"RESERVED_1B",
		"RESERVED_1C",
		"RESERVED_1D",
		"RESERVED_1E",
		"BSPI"
	};

	static const char* config_opcodes[]=
	{
		"NOP",
		"READ",
		"WRITE",
		"INVALID"
	};
	static const char* cmd_values[]=
	{
		"NULL",
		"WCFG",
		"MFW",
		"LFRM",
		"RCFG",
		"START",
		"RCAP",
		"RCRC",
		"AGHIGH",
		"SWITCH",
		"GRESTORE",
		"SHUTDOWN",
		"GCAPTURE",
		"DESYNC",
		"RESERVED_0E"
		"IPROG",
		"CRCC",
		"LTIMER"
	};

	//Parse frames
	while(fpos < len)
	{
		//Pull and byte-swap the frame header
		XilinxUltrascaleDeviceConfigurationFrame frame =
			*reinterpret_cast<const XilinxUltrascaleDeviceConfigurationFrame*>(data + fpos);
		FlipEndian32Array((unsigned char*)&frame, sizeof(frame));

		//Go past the header
		fpos += 4;

		//Look at the frame type and process it
		if(frame.bits.type == X7_CONFIG_FRAME_TYPE_1)
		{
			//Skip nops, dont print details
			if(frame.bits.op == X7_CONFIG_OP_NOP)
			{
				//printf("NOP at 0x%x\n", (int)fpos);
				continue;
			}

			//Validate frame
			if(frame.bits.reg_addr >= X7_CONFIG_REG_MAX)
			{
				LogWarning("[XilinxUltrascaleDevice] Invalid register address 0x%x in config frame at bitstream offset %02x\n",
					frame.bits.reg_addr, (int)fpos);
				delete bitstream;
				throw JtagExceptionWrapper(
					"Invalid register address in bitstream",
					"");
			}

			//Print stats
			LogDebug("Config frame starting at 0x%x: Type 1 %s to register 0x%02x (%s), %u words\n",
				(int)fpos,
				config_opcodes[frame.bits.op],
				frame.bits.reg_addr,
				config_register_names[frame.bits.reg_addr],
				frame.bits.count
				);

			//See if it's a write, if so pull out some data
			if(frame.bits.op == X7_CONFIG_OP_WRITE)
			{
				LogIndenter li;

				//Look at the frame data
				switch(frame.bits.reg_addr)
				{
					case X7_CONFIG_REG_CMD:
					{
						//Expect 1 word
						if(frame.bits.count != 1)
						{
							delete bitstream;
							throw JtagExceptionWrapper(
								"Invalid write (not 1 word) to CMD register in config frame",
								"");
						}
						uint32_t cmd_value = GetBigEndianUint32FromByteArray(data, fpos);
						if(cmd_value >= X7_CMD_MAX)
							LogWarning("Undocumented command value %d in bitstream\n", cmd_value);
						else
							LogDebug("Command = %s\n", cmd_values[cmd_value]);

						//We're done, skip to the end of the bitstream
						if(cmd_value == X7_CMD_DESYNC)
						{
							fpos = len;
							continue;
						}
					}
					break;
					case X7_CONFIG_REG_IDCODE:
					{
						//Expect 1 word
						if(frame.bits.count != 1)
						{
							delete bitstream;
							throw JtagExceptionWrapper(
								"Invalid write (not 1 word) to IDCODE register in config frame",
								"");
						}

						//Pull the value
						uint32_t idcode = GetBigEndianUint32FromByteArray(data, fpos);
						LogDebug("ID code = %08x\n", idcode);
						bitstream->idcode = idcode;
					}
					break;


				default:

					if(frame.bits.count == 1)
					{
						uint32_t cmd_value = GetBigEndianUint32FromByteArray(data, fpos);
						LogDebug("Data = 0x%08x\n", cmd_value);
					}

					break;
				}
			}

			//Discard the contents
			fpos += 4*frame.bits.count;
		}
		else if(frame.bits.type == X7_CONFIG_FRAME_TYPE_2)
		{
			unsigned int framesize = frame.bits_type2.count;
			LogIndenter li;

			//Print stats
			LogDebug("Config frame starting at 0x%x: Type 2, %u words\n",
				(int)fpos,
				framesize
				);

			//Discard data + header
			//There seems to be an unused trailing word after the last data word
			fpos += 4*(2 + framesize);
		}
		else
		{
			delete bitstream;
			throw JtagExceptionWrapper(
				"Invalid frame type",
				"");
		}
	}*/

	//All OK
	return bitstream;
}

void XilinxUltrascaleDevice::Reboot()
{
	throw JtagExceptionWrapper(
		"XilinxUltrascaleDevice::Reboot not implemented",
		"");

	/*
	ResetToIdle();
	SetIR(INST_JPROGRAM);
	SetIR(INST_BYPASS);
	*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// On-chip debug helpers

size_t XilinxUltrascaleDevice::GetNumUserInstructions()
{
	return 4;
}

void XilinxUltrascaleDevice::SelectUserInstruction(size_t /*index*/)
{
	throw JtagExceptionWrapper(
		"XilinxUltrascaleDevice::SelectUserInstruction not implemented",
		"");

	/*
	switch(index)
	{
		case 0:
			SetIRDeferred(INST_USER1);
			break;

		case 1:
			SetIRDeferred(INST_USER2);
			break;

		case 2:
			SetIRDeferred(INST_USER3);
			break;

		case 3:
			SetIRDeferred(INST_USER4);
			break;
	}*/
}
