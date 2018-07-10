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

	//Load the ISC_NOOP instruction immediately after. This isn't mentioned in the ultrascale datasheet
	//but the generated SVF's do it.
	SetIRForAllSLRs(INST_ISC_NOOP);

	//Wait a minimum of 100 us for things to reset (per generated SVF comments)
	usleep(100 * 1000);

	//Send a bunch of dummy clocks to make sure everything propagated
	SendDummyClocks(10000);

	//Poll status register until housecleaning is done
	XilinxUltrascaleDeviceStatusRegister statreg;
	int i;
	for(i=0; i<10; i++)
	{
		statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
		if(statreg.bits.init_b && !statreg.bits.done)
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
	Xilinx3DFPGABitstream* bitstream = new Xilinx3DFPGABitstream;
	try
	{
		XilinxFPGA::ParseBitstreamCore(bitstream, data, len);
	}
	catch(const JtagException& ex)
	{
		delete bitstream;
		throw ex;
	}

	return static_cast<FirmwareImage*>(bitstream);
}

void XilinxUltrascaleDevice::Program(FirmwareImage* /*image*/)
{
	LogIndenter li;

	/*
	//Should be an FPGA bitstream
	FPGABitstream* bitstream = static_cast<FPGABitstream*>(image);
	if(bitstream == NULL)
	{
		throw JtagExceptionWrapper(
			"Invalid firmware image (not an FPGABitstream)",
			"");
	}

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
	SetIRForMasterSLR(INST_CFG_IN);
	//ScanDR(flipped_bitstream, NULL, xbit->raw_bitstream_len * 8);

	//Need to break up the scan op since it's so big!!
	if(m_iface->GetDeviceCount() > 1)
	{
		throw JtagExceptionWrapper(
			"Bypassing extra devices not yet supported!",
			"");
	}
	m_iface->EnterShiftDR();
	size_t bytes_sent = 0;
	size_t bytes_to_send = xbit->raw_bitstream_len;
	while(bytes_to_send != 0)
	{
		size_t block_bytes = bytes_to_send;
		const size_t one_megabyte = 1024 * 1024;
		if(block_bytes > one_megabyte)
			block_bytes = one_megabyte;

		//Need to toggle TMS at the end of the last block
		bool last_block = (block_bytes == bytes_to_send);

		m_iface->ShiftData(last_block, flipped_bitstream + bytes_sent, NULL, block_bytes * 8);

		LogDebug("Programming... 0x%08zx bytes sent, 0x%08zx bytes left\n",
			bytes_sent, bytes_to_send);

		//Done, update counts
		bytes_sent += block_bytes;
		bytes_to_send -= block_bytes;
	}
	m_iface->LeaveExit1DR();
	m_iface->Commit();

	//Start up the FPGA
	SetIRForAllSLRs(INST_JSTART);
	SendDummyClocks(2000);			//per UG570 table 6-5 line 25

	//Clean up
	delete[] flipped_bitstream;
	flipped_bitstream = NULL;

	//Get the status register and verify that configuration was successful
	XilinxUltrascaleDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
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

	ResetToIdle();
	*/
}

void XilinxUltrascaleDevice::PrintStatusRegister()
{
	XilinxUltrascaleDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_REG_STAT);
	LogIndenter li;

	LogVerbose(
		"Status register is 0x%04x\n"
		"  CRC error         : %u\n"
		"  Decryptor enabled : %u\n"
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

void XilinxUltrascaleDevice::ParseBitstreamInternals(
	const unsigned char* data,
	size_t len,
	XilinxFPGABitstream* bitstream,
	size_t fpos)
{
	LogDebug("Parsing bitstream internals...\n");
	LogIndenter li;

	Xilinx3DFPGABitstream* topbit = dynamic_cast<Xilinx3DFPGABitstream*>(bitstream);
	if(!topbit)
	{
		throw JtagExceptionWrapper(
			"Not a 3D FPGA bitstream!",
			"");
	}

	//We should have a whole sub-bitstream for each SLR.
	for(unsigned int i=0; i<m_slrCount; i++)
	{
		if(fpos >= len)
			break;

		LogDebug("SLR %u\n", i);
		LogIndenter li;

		size_t slrbase = fpos;

		//Create a new bitstream for this block
		//We don't yet know how long it is, so allocate enough to hold the entire remainder of the bitstream.
		//This wastes a bit of RAM, maybe at some point we can optimize to clean this up.
		XilinxFPGABitstream* slrbit = new XilinxFPGABitstream;
		size_t initial_length = len - fpos;
		slrbit->raw_bitstream = new unsigned char[initial_length];
		memcpy(slrbit->raw_bitstream, data + fpos, initial_length);
		topbit->m_bitstreams.push_back(slrbit);

		//Expect aa 99 55 66 (sync word)
		unsigned char syncword[4] = {0xaa, 0x99, 0x55, 0x66};
		if(0 != memcmp(data+fpos, syncword, sizeof(syncword)))
		{
			throw JtagExceptionWrapper(
				"No valid sync word found",
				"");
		}

		//Skip the sync word
		fpos += sizeof(syncword);

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
			if(frame.bits.type == CONFIG_FRAME_TYPE_1)
			{
				bool desync;
				if(!ParseType1ConfigFrame(frame, data, len, fpos, bitstream->idcode, desync))
				{
					throw JtagExceptionWrapper(
						"Failed to parse Type-1 frame",
						"");
				}

				//Handle trailing data after a desync request
				if(desync)
				{
					LogDebug("Got desync command!\n");

					//If this is the last SLR: TODO
					if(i+1 == m_slrCount)
					{
					}

					//Nope. Stop when we get to the next sync word
					else
					{
						size_t nnops = 0;
						while(fpos < len)
						{
							//Sync word means done
							if(0 == memcmp(data+fpos, syncword, sizeof(syncword)))
							{
								if(nnops)
									LogDebug("Found %zu trailing NOPs after desync\n", nnops);
								break;
							}

							//Should be type-1 NOP frames afterwards
							//We probably don't need this many but whatever
							else
							{
								frame =
									*reinterpret_cast<const XilinxUltrascaleDeviceConfigurationFrame*>(data + fpos);
								FlipEndian32Array((unsigned char*)&frame, sizeof(frame));

								//Go past the header
								fpos += 4;

								//Expecting a NOP
								if(
									(frame.bits.type == CONFIG_FRAME_TYPE_1) &&
									(frame.bits.op == XilinxUltrascaleDevice::CONFIG_OP_NOP)
								 )
								{
									//LogDebug("NOP\n");
									nnops ++;
								}
								else
									LogWarning("Unknown frame %08x\n", frame.word);
							}
						}
					}

					//After a desync we're done
					break;
				}
			}
			else if(frame.bits.type == CONFIG_FRAME_TYPE_2)
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
				throw JtagExceptionWrapper(
					"Invalid frame type",
					"");
			}
		}

		//Done parsing frames
		//Save the actual length
		slrbit->raw_bitstream_len = fpos - slrbase;
		LogDebug("SLR bitstream length: %zu bytes\n", slrbit->raw_bitstream_len);
	}

	exit(1);
}

bool XilinxUltrascaleDevice::ParseType1ConfigFrame(
	XilinxUltrascaleDeviceConfigurationFrame frame,
	const unsigned char* data,
	size_t /*len*/,
	size_t& fpos,
	uint32_t& idcode,
	bool& desync,
	bool flip_bit_order)
{
	desync = false;

	//String names for config regs
	static const char* config_register_names[CONFIG_REG_MAX]=
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
		"RESERVED_06_WAS_RCAP",
		"RCRC",
		"AGHIGH",
		"SWITCH",
		"GRESTORE",
		"SHUTDOWN",
		"RESERVED_0C_WAS_GCAPTURE",
		"DESYNC",
		"RESERVED_0E"
		"IPROG",
		"CRCC",
		"LTIMER",
		"BSPI_READ",
		"FALL_EDGE"
	};

	//Skip nops, dont print details
	if(frame.bits.op == CONFIG_OP_NOP)
	{
		//printf("NOP at 0x%x\n", (int)fpos);
		return true;
	}

	//Validate frame
	if(frame.bits.reg_addr >= CONFIG_REG_MAX)
	{
		LogWarning("[XilinxUltrascaleDevice] Invalid register address 0x%x in config frame at bitstream offset %02x\n",
			frame.bits.reg_addr, (int)fpos);
		return false;
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
	if(frame.bits.op == CONFIG_OP_WRITE)
	{
		LogIndenter li;

		//Look at the frame data
		switch(frame.bits.reg_addr)
		{
			case CONFIG_REG_CMD:
			{
				//Expect 1 word
				if(frame.bits.count != 1)
				{
					LogError("Invalid write (not 1 word) to CMD register in config frame");
					return false;
				}
				uint32_t cmd_value = GetBigEndianUint32FromByteArray(data, fpos);

				//Flip if reading from SVF bit order
				if(flip_bit_order)
					FlipBitArray((uint8_t*)&cmd_value, 4);

				if(cmd_value >= CMD_MAX)
					LogWarning("Undocumented command value %d in bitstream\n", cmd_value);
				else
					LogDebug("Command = %s\n", cmd_values[cmd_value]);

				//We're done, skip to the end of the bitstream
				if(cmd_value == CMD_DESYNC)
					desync = true;
			}
			break;

			case CONFIG_REG_IDCODE:
			{
				//Expect 1 word
				if(frame.bits.count != 1)
				{
					LogError("Invalid write (not 1 word) to IDCODE register in config frame");
					return false;
				}

				//Pull the value
				idcode = GetBigEndianUint32FromByteArray(data, fpos);

				//Flip if reading from SVF bit order
				if(flip_bit_order)
					FlipBitArray((uint8_t*)&idcode, 4);

				LogDebug("ID code = %08x\n", idcode);
			}
			break;


		default:

			if(frame.bits.count == 1)
			{
				uint32_t cmd_value = GetBigEndianUint32FromByteArray(data, fpos);

				//Flip if reading from SVF bit order
				if(flip_bit_order)
					FlipBitArray((uint8_t*)&cmd_value, 4);

				LogDebug("Data = 0x%08x\n", cmd_value);
			}

			break;
		}
	}

	//Discard the contents
	fpos += 4*frame.bits.count;

	return true;
}

void XilinxUltrascaleDevice::Reboot()
{
	InternalErase();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SVF stuff

bool XilinxUltrascaleDevice::GetSVFLine(FILE* fp, string& line)
{
	line = "";

	string whitespace = "";

	bool in_comment = false;
	while(!feof(fp))
	{
		char tmp = fgetc(fp);

		if(in_comment)
		{
			//Newline means "done"
			if(tmp == '\n')
			{
				in_comment = false;
				continue;
			}

			//otherwise ignore it
		}

		else
		{
			//Skip leading whitespace
			if( (line == "") && isspace(tmp) )
				continue;

			//C++ style comment means enter comment mode
			//TODO: support comments in the middle of a block
			if(line == "/" && tmp == '/')
			{
				line = "";
				in_comment = true;
				continue;
			}

			//Stop when we get a semicolon
			if(tmp == ';')
				return true;

			//Skip newlines
			if(tmp == '\n')
				continue;

			//Nope, add the incoming character.
			//Buffer whitespace and only add when we get a non-space character
			if(isspace(tmp))
				whitespace += tmp;
			else
			{
				line += whitespace;
				line += tmp;
				whitespace = "";
			}
		}
	}

	return false;
}

string XilinxUltrascaleDevice::GetSVFOpcode(string& line)
{
	string op;
	for(size_t i=0; i<line.length(); i++)
	{
		if(isspace(line[i]))
			break;
		else
			op += line[i];
	}

	return op;
}

void XilinxUltrascaleDevice::AnalyzeSVF(string path)
{
	LogVerbose("Analyzing SVF...\n");
	LogIndenter li;

	FILE* fp = fopen(path.c_str(), "r");
	if(!fp)
	{
		throw JtagExceptionWrapper(
			"XilinxUltrascaleDevice::AnalyzeSVF: file not found\n",
			"");
	}

	vector<uint8_t> cfg_data;

	string line;
	bool append_dr = false;
	while(GetSVFLine(fp, line))
	{
		//Parse out the opcode
		string opcode = GetSVFOpcode(line);

		//TRST: Must be OFF, ignore that
		if(opcode == "TRST")
		{
			if(line == "TRST OFF")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: TRST other than OFF not supported\n",
					line);
			}
		}

		//ENDIR: Must be IDLE, ignore that
		else if(opcode == "ENDIR")
		{
			if(line == "ENDIR IDLE")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: ENDIR other than IDLE not supported\n",
					line);
			}
		}

		//ENDDR: Must be IDLE or DRPAUSE
		else if(opcode == "ENDDR")
		{
			if(line == "ENDDR IDLE")
			{
				append_dr = false;
				continue;
			}
			else if(line == "ENDDR DRPAUSE")
			{
				append_dr = true;
				continue;
			}
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: ENDDR other than IDLE and DRPAUSE not supported\n",
					line);
			}
		}

		//HIR/HDR/TIR/TDR: Must be 0
		else if(opcode == "HIR")
		{
			if(line == "HIR 0")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: HIR other than 0 not supported\n",
					line);
			}
		}
		else if(opcode == "HDR")
		{
			if(line == "HDR 0")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: HDR other than 0 not supported\n",
					line);
			}
		}
		else if(opcode == "TIR")
		{
			if(line == "TIR 0")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: TIR other than 0 not supported\n",
					line);
			}
		}
		else if(opcode == "TDR")
		{
			if(line == "TDR 0")
				continue;
			else
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: TDR other than 0 not supported\n",
					line);
			}
		}

		//Ignore STATE for now
		else if(opcode == "STATE")
		{
			LogDebug("State change: %s\n", line.c_str());
			continue;
		}

		//Ignore frequency (no importance to use)
		else if(opcode == "FREQUENCY")
			continue;

		//Dummy clocks
		else if(opcode == "RUNTEST")
		{
			//Parse it
			float sec;
			int clocks;
			if(line.find("SEC") != string::npos)
			{
				sscanf(line.c_str(), "RUNTEST %f SEC", &sec);
				LogDebug("Sleep %.2f ms\n", sec * 1000);
			}
			else if(1 == sscanf(line.c_str(), "RUNTEST %d TCK", &clocks))
				LogDebug("Send %d dummy clocks\n", clocks);
		}

		//Shift INSTRUCTION REGISTER
		else if(opcode == "SIR")
		{
			int nclocks;
			unsigned int ir;
			if(2 != sscanf(line.c_str(), "SIR %d TDI (%x)", &nclocks, &ir))	//TODO: handle TDO/mask
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: Malformed Shift-IR line\n",
					line);
			}

			//Verify that we have the right size
			if(nclocks != m_irlength)
			{
				throw JtagExceptionWrapper(
					"XilinxUltrascaleDevice::AnalyzeSVF: Strange IR length\n",
					line);
			}

			//Parse it out, 6 bits at a time
			//Our convention is that the LEFTMOST slr in the IR is #0
			vector<unsigned int> irs;
			for(unsigned int i=0; i<m_slrCount; i++)
			{
				int shamt = (m_slrCount - 1 - i) * 6;
				irs.push_back( (ir >> shamt) & 0x3f );
			}

			string textir;
			for(unsigned int i=0; i<m_slrCount; i++)
			{
				switch(irs[i])
				{
				case INST_IDCODE:
					textir += "IDCODE";
					break;

				case INST_SLR_BYPASS:
					textir += "SLR_BYPASS";
					break;

				case INST_CFG_OUT:
					textir += "CFG_OUT";
					break;

				case INST_CFG_IN:
					textir += "CFG_IN";
					break;

				case INST_JPROGRAM:
					textir += "JPROGRAM";
					break;

				case INST_JSTART:
					textir += "JSTART";
					break;

				case INST_ISC_NOOP:
					textir += "ISC_NOOP";
					break;

				default:
					textir += "FIXME";
					break;
				}

				if(i+1 < m_slrCount)
					textir += ", ";
			}
			LogDebug("Set IR: { %s }\n", textir.c_str());
		}

		//Shift DATA REGISTER
		else if(opcode == "SDR")
		{
			//Read the length
			unsigned int len = 0;
			if(1 != sscanf(line.c_str(), "SDR %d TDI", &len))
			{
				LogWarning("Malformed SDR line\n");
				continue;
			}

			//Convert to bytes so we know how much hex to read
			unsigned int bytelen = len / 8;

			//Read the hex data a byte at a time until we get it all
			unsigned char* bytes = new unsigned char[bytelen];

			//Start by pulling out the hex data itself and removing fluff
			size_t start = line.find("(") + 1;
			unsigned int bval;
			for(size_t i=0; i<bytelen; i++)
			{
				char hex[3] = { line[2*i + start], line[2*i + start + 1], 0};
				sscanf(hex, "%x", &bval);
				bytes[i] = bval;
			}

			//SVF sends the rightmost byte first. This is kinda silly, since we can't stream it!
			FlipByteArray(bytes, bytelen);

			//Make a big array of everything sent, possibly across multiple transactions.
			//If we're appending, there's more data to come.
			//Otherwise, process it.
			for(size_t i=0; i<bytelen; i++)
				cfg_data.push_back(bytes[i]);
			delete[] bytes;

			if(append_dr)
				continue;

			LogDebug("Send %zu bytes\n", cfg_data.size());
			LogIndenter li;

			//Got everything. Parse it.
			bool found_sync = false;
			size_t fpos = 0;
			size_t nnops = 0;
			while(fpos<cfg_data.size())
			{
				if(!found_sync)
				{
					//Pull out four bytes at a time
					//Swap bit ordering (see UG570 p. 139)
					uint32_t block = *reinterpret_cast<uint32_t*>(&cfg_data[fpos]);
					FlipBitAndEndian32Array((unsigned char*)&block, 4);

					if(block == 0xffffffff)
						LogDebug("Dummy word\n");
					else if(block == 0x000000bb)
						LogDebug("Bus width sync word\n");
					else if(block == 0x11220044)
						LogDebug("Bus width detect word\n");
					else if(block == 0xaa995566)
					{
						if(nnops)
						{
							LogDebug("%zu NOPs\n", nnops);
							nnops = 0;
						}
						LogDebug("Bitstream sync word\n");
						found_sync = true;
					}
					else if(block == 0x20000000)
						nnops ++;
					else
						LogDebug("Block %08x\n", block);

					fpos += 4;
					continue;
				}

				//Pull and byte-swap the frame header
				XilinxUltrascaleDeviceConfigurationFrame frame =
					*reinterpret_cast<const XilinxUltrascaleDeviceConfigurationFrame*>(&cfg_data[fpos]);
				FlipBitAndEndian32Array((unsigned char*)&frame, sizeof(frame));
				fpos += 4;

				//See what it is
				uint32_t idcode;
				if(frame.bits_type2.type == XilinxUltrascaleDevice::CONFIG_FRAME_TYPE_1)
				{
					bool desync;
					if(!ParseType1ConfigFrame(frame, &cfg_data[0], cfg_data.size(), fpos, idcode, desync, true))
						break;
					if(desync)
					{
						found_sync = false;
						continue;
					}
				}
				else if(frame.bits.type == CONFIG_FRAME_TYPE_2)
				{
					unsigned int framesize = frame.bits_type2.count;
					LogIndenter li;

					//Print stats
					LogDebug("Config frame starting at 0x%x: Type 2, %u words\n",
						(int)fpos,
						framesize
						);

					//Discard data + header
					//There seems to be an unused trailing word after the last data word??
					fpos += 4*(2 + framesize);
				}

				else
				{
					LogError("Invalid frame type %d (0x%08x)\n", frame.bits.type, frame.word);
					fpos += 4;
					continue;
				}
			}

			//Clean up
			cfg_data.clear();
		}

		else
			LogDebug("Unsupported opcode: op=%s\n", opcode.c_str());

		//LogDebug("NEW LINE: %s\n", line.c_str());
	}

	fclose(fp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// On-chip debug helpers

size_t XilinxUltrascaleDevice::GetNumUserInstructions()
{
	return 0;	//not implemented yet
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
