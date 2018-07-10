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
	@brief Implementation of XilinxSpartan6Device
 */
#include "jtaghal.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Initializes this device

	@param arraysize	Array size from JTAG ID code
	@param rev			Revision number from JTAG ID code
	@param idcode		The ID code of this device
	@param iface		The JTAG adapter this device was discovered on
	@param pos			Position in the chain that this device was discovered
 */
XilinxSpartan6Device::XilinxSpartan6Device(
	unsigned int arraysize,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
: XilinxFPGA(idcode, iface, pos)
, m_arraysize(arraysize)
, m_rev(rev)
{
	m_irlength = 6;
}

/**
	@brief Empty virtual destructor
 */
XilinxSpartan6Device::~XilinxSpartan6Device()
{
}

/**
	@brief Factory method
 */
JtagDevice* XilinxSpartan6Device::CreateDevice(
	unsigned int arraysize,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
{
	switch(arraysize)
	{
	case SPARTAN6_LX9:
	case SPARTAN6_LX16:
	case SPARTAN6_LX25:
	case SPARTAN6_LX45:
		return new XilinxSpartan6Device(arraysize, rev, idcode, iface, pos);

	default:
		throw JtagExceptionWrapper(
			"Unknown Spartan-6 device (ID code not in database)",
			"");
		//fprintf(stderr, "Unknown Spartan-6 device (arraysize=%x, rev=%x)!\n", arraysize, rev);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generic device information

string XilinxSpartan6Device::GetDescription()
{
	string devname = "(unknown Spartan-6)";

	switch(m_arraysize)
	{
	case SPARTAN6_LX9:
		devname = "XC6SLX9";
		break;
	case SPARTAN6_LX16:
		devname = "XC6SLX16";
		break;
	case SPARTAN6_LX25:
		devname = "XC6SLX25";
		break;
	case SPARTAN6_LX45:
		devname = "XC6SLX45";
		break;
	default:
		throw JtagExceptionWrapper(
			"Unknown Spartan-6 device (ID code not in database)",
			"");
	}

	char srev[16];
	snprintf(srev, 15, "%u", m_rev);

	return string("Xilinx ") + devname + " stepping " + srev;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA-specific device properties

int XilinxSpartan6Device::GetSerialNumberLength()
{
	return 8;
}

int XilinxSpartan6Device::GetSerialNumberLengthBits()
{
	return 57;
}

void XilinxSpartan6Device::GetSerialNumber(unsigned char* data)
{
	InternalErase();

	//Enter ISC mode (wipes configuration)
	ResetToIdle();
	SetIR(INST_ISC_ENABLE);

	//Read the DNA value
	SetIR(INST_ISC_DNA);
	unsigned char zeros[8] = {0x00};
	ScanDR(zeros, data, 57);

	//Done
	SetIR(INST_ISC_DISABLE);
}

bool XilinxSpartan6Device::IsProgrammed()
{
	XilinxSpartan6DeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(S6_CONFIG_REG_STAT);
	return statreg.bits.done;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration

void XilinxSpartan6Device::Erase()
{
	ResetToIdle();
	InternalErase();
	SetIR(INST_BYPASS);
}

void XilinxSpartan6Device::InternalErase()
{
	//Load the JPROGRAM instruction to clear configuration memory
	LogDebug("Clearing configuration memory...\n");
	SetIR(INST_JPROGRAM);
	SendDummyClocks(32);

	//Poll status register until housecleaning is done
	XilinxSpartan6DeviceStatusRegister statreg;
	int i;
	for(i=0; i<10; i++)
	{
		statreg.word = ReadWordConfigRegister(S6_CONFIG_REG_STAT);
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

void XilinxSpartan6Device::Reboot()
{
	ResetToIdle();
	SetIR(INST_JPROGRAM);
	SetIR(INST_BYPASS);
}

FirmwareImage* XilinxSpartan6Device::LoadFirmwareImage(const unsigned char* data, size_t len)
{
	XilinxFPGABitstream* bitstream = new XilinxFPGABitstream;
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

void XilinxSpartan6Device::Program(FirmwareImage* image)
{
	LogIndenter li;

	//Should be an FPGA bitstream
	FPGABitstream* bitstream = dynamic_cast<FPGABitstream*>(image);
	if(bitstream == NULL)
	{
		throw JtagExceptionWrapper(
			"Invalid firmware image (not an FPGABitstream)",
			"");
	}

	//Verify the ID code matches the device we're plugged into
	//AND out the stepping number since this is irrelevant
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

	//Erase the configuration
	InternalErase();

	//Send the bitstream to the FPGA
	//See UG380 table 10-4 (page 161)
	LogVerbose("Loading new bitstream...\n");
	SetIR(INST_CFG_IN);
	ScanDR(flipped_bitstream, NULL, xbit->raw_bitstream_len * 8);

	//Start up the FPGA
	//Minimum of 16 clock cycles required according to UG380 page 161, do a few more to be safe
	SetIR(INST_JSTART);
	SendDummyClocks(64);

	//Clean up
	delete[] flipped_bitstream;
	flipped_bitstream = NULL;

	//Get the status register and verify that configuration was successful
	XilinxSpartan6DeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(S6_CONFIG_REG_STAT);
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
}

void XilinxSpartan6Device::PrintStatusRegister()
{
	LogIndenter li;

	XilinxSpartan6DeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(S6_CONFIG_REG_STAT);
	LogNotice(	"Status register is 0x%04x\n"
		"  CRC error         : %u\n"
		"  IDCODE error      : %u\n"
		"  DCM lock status   : %u\n"
		"  GTS_CFG_B status  : %u\n"
		"  GWE status        : %u\n"
		"  GHIGH status      : %u\n"
		"  Decryption error  : %u\n"
		"  Decryption enable : %u\n"
		"  HSWAPEN pin       : %u\n"
		"  M0 pin            : %u\n"
		"  M1 pin            : %u\n"
		"  Reserved          : %u\n"
		"  INIT_B pin        : %u\n"
		"  DONE pin          : %u\n"
		"  Suspend status    : %u\n"
		"  Fallback status   : %u\n",
		statreg.word,
		statreg.bits.crc_err,
		statreg.bits.idcode_err,
		statreg.bits.dcm_lock,
		statreg.bits.gts_cfg_b,
		statreg.bits.gwe,
		statreg.bits.ghigh,
		statreg.bits.decrypt_err,
		statreg.bits.decrypt_en,
		statreg.bits.hswapen,
		statreg.bits.m0,
		statreg.bits.m1,
		statreg.bits.reserved,
		statreg.bits.init_b,
		statreg.bits.done,
		statreg.bits.suspend,
		statreg.bits.fallback);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal configuration helpers

/**
	@brief Reads a single 16-bit word from a config register

	Reference: UG380 page 115-116
	Table 6-5

	Note that Spartan-6 devices expect data clocked in MSB first but the JTAG API clocks data LSB first.
	Some swapping is required as a result.

	Clock data into CFG_IN register
		Synchronization word
		Read STAT register
			Type = 1		001
			Op = Read		01
			Addr of reg		xxxxxx
			Word count = 1	00001
			= 0x2901
		Two dummy words to flush packet buffer

	Read from CFG_OUT register

	@throw JtagException if the read fails

	@param reg The configuration register to read
 */
uint16_t XilinxSpartan6Device::ReadWordConfigRegister(unsigned int reg)
{
	//Send the read request
	SetIR(INST_CFG_IN);
	XilinxSpartan6DeviceConfigurationFrame frames[] =
	{
		{ bits: {0,     0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} },	//Dummy word
		{ word: 0xaa99 },												//Sync word
		{ word: 0x5566 },
		{ bits: {1, reg, S6_CONFIG_OP_READ, S6_CONFIG_FRAME_TYPE_1} },	//Read the register
		{ bits: {0, 0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} },	//Two dummy words required
		{ bits: {0, 0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} }
	};
	unsigned char* packet = (unsigned char*) frames;
	FlipBitAndEndianArray(packet, sizeof(frames));		//MSB needs to get sent first
	ScanDR(packet, NULL, sizeof(frames) * 8);							//Table 6-5 of UG380 v2.4 says we need 160 clocks
																		//but it seems we only need 80. See WebCase 948541

	//Read the data
	SetIR(INST_CFG_OUT);
	unsigned char unused[2] = {0};
	uint16_t reg_out = {0};
	ScanDR(unused, (unsigned char*)&reg_out, 16);
	FlipBitAndEndianArray((unsigned char*)&reg_out, 2);

	return reg_out;
}

/**
	@brief Reads several 16-bit words from a config register.

	The current implementation uses type 1 packets and is thus limited to reading less than 32 words.

	@throw JtagException if the read fails

	@param reg		The configuration register to read
	@param dout 	Buffer to read into
	@param count	Number of 16-bit words to read
 */
void XilinxSpartan6Device::ReadWordsConfigRegister(unsigned int reg, uint16_t* dout, unsigned int count)
{
	if(count < 32)
	{
		//Send the read request
		SetIR(INST_CFG_IN);
		XilinxSpartan6DeviceConfigurationFrame frames[] =
		{
			{ bits: {0,     0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} },	//Dummy word
			{ word: 0xaa99 },													//Sync word
			{ word: 0x5566 },
			{ bits: {count, reg, S6_CONFIG_OP_READ, S6_CONFIG_FRAME_TYPE_1} },	//Read the register
			{ bits: {0,     0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} },	//Two dummy words required
			{ bits: {0,     0,   S6_CONFIG_OP_NOP,  S6_CONFIG_FRAME_TYPE_1} }
		};
		unsigned char* packet = (unsigned char*) frames;
		FlipBitAndEndianArray(packet, sizeof(frames));			//MSB needs to get sent first
		ScanDR(packet, NULL, sizeof(frames) * 8);								//Table 6-5 of UG380 v2.4 says we need 160 clocks
																				//but it seems we only need 80. See WebCase 948541

		//Read the data
		SetIR(INST_CFG_OUT);
		unsigned char unused[32];
		ScanDR(unused, (unsigned char*)dout, count*16);
		FlipBitAndEndianArray((unsigned char*)dout, count*2);
	}

	else
	{
		//TODO: use type 2 read
		throw JtagExceptionWrapper(
			"ReadWordsConfigRegister() not implemented for count>32",
			"");
	}
}

void XilinxSpartan6Device::WriteWordConfigRegister(unsigned int /*reg*/, uint16_t /*value*/)
{
	throw JtagExceptionWrapper(
		"WriteWordConfigRegister() not implemented",
		"");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bitstream parsing

void XilinxSpartan6Device::ParseBitstreamInternals(
	const unsigned char* data,
	size_t len,
	XilinxFPGABitstream* bitstream,
	size_t fpos)
{
	LogDebug("Parsing bitstream internals...\n");
	LogIndenter li;
	LogDebug("Using %s\n", bitstream->GetDescription().c_str());

	//Expect aa 99 55 66 (sync word)
	unsigned char syncword[4] = {0xaa, 0x99, 0x55, 0x66};
	if(0 != memcmp(data+fpos, syncword, sizeof(syncword)))
	{
		throw JtagExceptionWrapper(
			"No valid sync word found",
			"");
	}

	//Allocate space and copy the entire bitstream into raw_bitstream
	bitstream->raw_bitstream_len = len-fpos;
	bitstream->raw_bitstream = new unsigned char[bitstream->raw_bitstream_len];
	memcpy(bitstream->raw_bitstream, data+fpos, bitstream->raw_bitstream_len);

	//Skip the sync word
	size_t start = fpos;
	fpos += sizeof(syncword);

	//Multiboot stuff
	unsigned int multiboot_address = 0;

	//String names for config regs
	static const char* config_register_names[]=
	{
		"CRC",
		"FAR_MAJ",
		"FAR_MIN",
		"FDRI",
		"FDRO",
		"CMD",
		"CTL",
		"MASK",
		"STAT",
		"LOUT",
		"COR1",
		"COR2",
		"PWRDN",
		"FLR",
		"IDCODE",
		"CWDT",
		"HC_OPT",
		"UNDOCUMENTED",
		"CSB0",
		"GENERAL1",
		"GENERAL2",
		"GENERAL3",
		"GENERAL4",
		"GENERAL5",
		"MODE",
		"PU_GWE",
		"PU_GTS",
		"MFWR",
		"CCLK_FREQ",
		"SEU_OPT",
		"EXP_SIGN",
		"RDBK_SIGN",
		"BOOTSTS",
		"EYE_MASK",
		"CBC"
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
		"UNDOCUMENTED_6",
		"RCRC",
		"AGHIGH",
		"UNDOCUMENTED_9",
		"GRESTORE",
		"SHUTDOWN",
		"UNDOCUMENTED_C",
		"DESYNC",
		"IPROG",
		"UNDOCUMENTED_F"
	};

	//Parse frames
	while(fpos < len)
	{
		//Pull and byte-swap the frame header
		XilinxSpartan6DeviceConfigurationFrame frame =
			*reinterpret_cast<const XilinxSpartan6DeviceConfigurationFrame*>(data + fpos);
		FlipEndianArray((unsigned char*)&frame, sizeof(frame));

		//Skip nops, dont print details
		if(frame.bits.op == S6_CONFIG_OP_NOP)
		{
			//printf("NOP at 0x%x\n", (int)fpos);
			fpos += 2;
			continue;
		}

		//Validate frame
		if(frame.bits.reg_addr >= S6_CONFIG_REG_MAX)
		{
			LogError("[XilinxSpartan6Device] Invalid register address 0x%x in config frame at bitstream offset %02x\n",
				frame.bits.reg_addr, (int)fpos);
			throw JtagExceptionWrapper(
				"Invalid register address in bitstream",
				"");
		}

		//Print stats
		LogDebug("Config frame starting at 0x%x: Type %u %s to register 0x%02x (%s), ",
			(int)fpos,
			frame.bits.type,
			config_opcodes[frame.bits.op],
			frame.bits.reg_addr,
			config_register_names[frame.bits.reg_addr]
			);

		//Go past the header
		fpos += 2;

		//Look at the frame type and process it
		if(frame.bits.type == S6_CONFIG_FRAME_TYPE_1)
		{
			//Type 1 frame
			LogDebug("%u words\n", frame.bits.count);
			LogIndenter li;

			//See if it's a write, if so pull out some data
			if(frame.bits.op == S6_CONFIG_OP_WRITE)
			{
				//Look at the frame data
				switch(frame.bits.reg_addr)
				{
					case S6_CONFIG_REG_GENERAL1:
					{
						multiboot_address = (multiboot_address & 0xffff0000) |
							GetBigEndianUint16FromByteArray(data, fpos);

						LogDebug("Multiboot start address low: %x\n", multiboot_address);
					}
					break;

					case S6_CONFIG_REG_GENERAL2:
					{
						uint16_t value = GetBigEndianUint16FromByteArray(data, fpos);

						LogDebug("Multiboot SPI opcode: 0x%x\n", value >> 8);
						LogDebug("Multiboot start address high: 0x%x\n", value & 0xff);

						multiboot_address = (multiboot_address & 0x0000ffff) | ( (value & 0xFF) << 16);

					}
					break;

					case S6_CONFIG_REG_GENERAL4:
					{
						uint16_t value = GetBigEndianUint16FromByteArray(data, fpos);

						LogDebug("Golden SPI opcode: 0x%x\n", value >> 8);
					}
					break;

					case S6_CONFIG_REG_CMD:
					{
						//Expect 1 word
						if(frame.bits.count != 1)
						{
							throw JtagExceptionWrapper(
								"Invalid write (not 1 word) to CMD register in config frame",
								"");
						}

						uint16_t cmd_value = GetBigEndianUint16FromByteArray(data, fpos);
						if(cmd_value > 16)
							LogDebug("Invalid command value");
						else
							LogDebug("Command = %s\n", cmd_values[cmd_value]);

						//We're done, skip to the end of the bitstream
						if(cmd_value == S6_CMD_DESYNC)
						{
							fpos = len;
							continue;
						}

						//Reset to multiboot start
						if(cmd_value == S6_CMD_IPROG)
						{
							fpos = start + multiboot_address;

							LogDebug("IPROG reset (to address %x)\n", multiboot_address);

							//Read the sync word
							uint16_t syncword_hi = GetBigEndianUint16FromByteArray(data, fpos);
							uint16_t syncword_lo = GetBigEndianUint16FromByteArray(data, fpos+2);
							uint32_t syncword = (syncword_hi << 16) | syncword_lo;

							//Sanity check it
							LogDebug("Sync word = %x\n", syncword);
							if(syncword != 0xaa995566)
							{
								delete bitstream;
								throw JtagExceptionWrapper(
									"Invalid MultiBoot sync word",
									"");
							}

							//Skip the sync word
							fpos += 4;

							continue;
						}
					}
					break;

					case S6_CONFIG_REG_IDCODE:
					{
						//Expect 2 words
						if(frame.bits.count != 2)
						{
							throw JtagExceptionWrapper(
								"Invalid write (not 2 word) to IDCODE register in config frame",
								"");
						}

						//Pull the value
						uint16_t idcode_hi = GetBigEndianUint16FromByteArray(data, fpos);
						uint16_t idcode_lo = GetBigEndianUint16FromByteArray(data, fpos+2);
						uint32_t idcode = (idcode_hi << 16) | idcode_lo;
						LogDebug("ID code = %08x\n", idcode);
						bitstream->idcode = idcode;
					}
					break;

					case S6_CONFIG_REG_CRC:
					{
						//Expect 2 words
						if(frame.bits.count != 2)
						{
							throw JtagExceptionWrapper(
								"Invalid write (not 2 word) to CRC register in config frame",
								"");
						}

						//Pull the value
						uint16_t crc_hi = GetBigEndianUint16FromByteArray(data, fpos);
						uint16_t crc_lo = GetBigEndianUint16FromByteArray(data, fpos+2);
						uint32_t crc = (crc_hi << 16) | crc_lo;
						LogDebug("CRC = %08x\n", crc);
					}
					break;

					default:
						if(frame.bits.count == 1)
							LogDebug("Value = %x\n", GetBigEndianUint16FromByteArray(data, fpos));
						else if(frame.bits.count == 2)
						{
							//Pull the value
							uint16_t value_hi = GetBigEndianUint16FromByteArray(data, fpos);
							uint16_t value_lo = GetBigEndianUint16FromByteArray(data, fpos+2);
							uint32_t value = (value_hi << 16) | value_lo;
							LogDebug("Value = %08x\n", value);
						}
					break;

				}
			}

			//Discard the contents
			fpos += 2*frame.bits.count;
		}
		else if(frame.bits.type == S6_CONFIG_FRAME_TYPE_2)
		{
			LogIndenter li;

			//Type 2 frame - not implemented
			//Get the size (32 bits)
			uint16_t framesize_hi = GetBigEndianUint16FromByteArray(data, fpos);
			uint16_t framesize_lo = GetBigEndianUint16FromByteArray(data, fpos+2);
			uint32_t framesize = (framesize_hi << 16) | framesize_lo;
			fpos += 4;
			LogDebug("%d words\n", framesize);

			//Discard the contents
			fpos += 2*framesize;

			//TODO: last 4 seem to be invalid, are these part of the same frame? Or are we skipping a frame?
			fpos += 4;
		}
		else
		{
			throw JtagExceptionWrapper(
				"Invalid frame type",
				"");
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// On-chip debug helpers

size_t XilinxSpartan6Device::GetNumUserInstructions()
{
	return 4;
}

void XilinxSpartan6Device::SelectUserInstruction(size_t index)
{
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
	}
}

