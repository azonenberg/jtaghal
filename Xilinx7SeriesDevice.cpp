/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2016 Andrew D. Zonenberg                                                                          *
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
	@brief Implementation of Xilinx7SeriesDevice
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
Xilinx7SeriesDevice::Xilinx7SeriesDevice(
	unsigned int arraysize,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
: XilinxFPGA(idcode, iface, pos, 6)
, m_arraysize(arraysize)
, m_rev(rev)
{
	RegisterConstant(CONFIG_CRC);
	RegisterConstant(CONFIG_FAR);
	RegisterConstant(CONFIG_FDRI);
	RegisterConstant(CONFIG_FDRO);
	RegisterConstant(CONFIG_CMD);
	RegisterConstant(CONFIG_CTL0);
	RegisterConstant(CONFIG_MASK);
	RegisterConstant(CONFIG_STAT);
	RegisterConstant(CONFIG_LOUT);
	RegisterConstant(CONFIG_COR0);
	RegisterConstant(CONFIG_MFWR);
	RegisterConstant(CONFIG_CBC);
	RegisterConstant(CONFIG_IDCODE);
	RegisterConstant(CONFIG_AXSS);
	RegisterConstant(CONFIG_COR1);
	RegisterConstant(CONFIG_WBSTAR);
	RegisterConstant(CONFIG_TIMER);
	RegisterConstant(CONFIG_BOOTSTS);
	RegisterConstant(CONFIG_CTL1);
	RegisterConstant(CONFIG_BSPI);
}
/**
	@brief Empty virtual destructor
 */
Xilinx7SeriesDevice::~Xilinx7SeriesDevice()
{
}

/**
	@brief Factory method
 */
JtagDevice* Xilinx7SeriesDevice::CreateDevice(
	unsigned int arraysize,
	unsigned int rev,
	unsigned int idcode,
	JtagInterface* iface,
	size_t pos)
{
	switch(arraysize)
	{
	case SPARTAN7_6:
	case SPARTAN7_15:
	case SPARTAN7_25:
	case SPARTAN7_50:
	case SPARTAN7_75:
	case SPARTAN7_100:
	case ARTIX7_12T:
	case ARTIX7_15T:
	case ARTIX7_25T:
	case ARTIX7_35T:
	case ARTIX7_50T:
	case ARTIX7_75T:
	case ARTIX7_100T:
	case ARTIX7_200T:
	case KINTEX7_70T:
	case KINTEX7_160T:
	case ZYNQ_010:
		return new Xilinx7SeriesDevice(arraysize, rev, idcode, iface, pos);

	default:
		LogError("Xilinx7SeriesDevice::CreateDevice(arraysize=%x, rev=%x, idcode=%08x)\n", arraysize, rev, idcode);

		throw JtagExceptionWrapper(
			"Unknown 7-series device (ID code not in database)",
			"");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generic device information

string Xilinx7SeriesDevice::GetDescription()
{
	string devname = "(unknown 7-series)";

	switch(m_arraysize)
	{
	case SPARTAN7_6:
		devname = "XC7S6";
		break;
	case SPARTAN7_15:
		devname = "XC7S15";
		break;
	case SPARTAN7_25:
		devname = "XC7S25";
		break;
	case SPARTAN7_50:
		devname = "XC7S50";
		break;
	case SPARTAN7_75:
		devname = "XC7S75";
		break;
	case SPARTAN7_100:
		devname = "XC7S100";
		break;
	case ARTIX7_12T:
		devname = "XC7A12T";
		break;
	case ARTIX7_15T:
		devname = "XC7A15T";
		break;
	case ARTIX7_25T:
		devname = "XC7A25T";
		break;
	case ARTIX7_35T:
		devname = "XC7A35T";
		break;
	case ARTIX7_50T:
		devname = "XC7A50T";
		break;
	case ARTIX7_75T:
		devname = "XC7A75T";
		break;
	case ARTIX7_100T:
		devname = "XC7A100T";
		break;
	case ARTIX7_200T:
		devname = "XC7A200T";
		break;
	case KINTEX7_70T:
		devname = "XC7K70T";
		break;
	case KINTEX7_160T:
		devname = "XC7K160T";
		break;
	case ZYNQ_010:
		devname = "XC7Z010";
		break;
	default:
		throw JtagExceptionWrapper(
			"Unknown 7-series device (ID code not in database)",
			"");
	}

	char srev[16];
	snprintf(srev, 15, "%u", m_rev);

	return string("Xilinx ") + devname + " stepping " + srev;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA-specific device properties

int Xilinx7SeriesDevice::GetSerialNumberLength()
{
	return 8;
}

int Xilinx7SeriesDevice::GetSerialNumberLengthBits()
{
	return 57;
}

void Xilinx7SeriesDevice::GetSerialNumber(unsigned char* data)
{
	InternalErase();

	//Enter ISC mode (wipes configuration)
	ResetToIdle();
	SetIR(INST_ISC_ENABLE);

	//Read the DNA value
	SetIR(INST_XSC_DNA);
	unsigned char zeros[8] = {0x00};
	ScanDR(zeros, data, 57);

	//Done
	SetIR(INST_ISC_DISABLE);
}

bool Xilinx7SeriesDevice::IsProgrammed()
{
	Xilinx7SeriesDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_STAT);
	return statreg.bits.done;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration

void Xilinx7SeriesDevice::Erase()
{
	ResetToIdle();
	InternalErase();
	SetIR(INST_BYPASS);
}

void Xilinx7SeriesDevice::InternalErase()
{
	//Load the JPROGRAM instruction to clear configuration memory
	LogDebug("Clearing configuration memory...\n");
	SetIR(INST_JPROGRAM);
	SendDummyClocks(32);

	//Poll status register until housecleaning is done
	Xilinx7SeriesDeviceStatusRegister statreg;
	int i;
	for(i=0; i<10; i++)
	{
		statreg.word = ReadWordConfigRegister(CONFIG_STAT);
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


FirmwareImage* Xilinx7SeriesDevice::LoadFirmwareImage(const unsigned char* data, size_t len)
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

void Xilinx7SeriesDevice::Program(FirmwareImage* image)
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
	Xilinx7SeriesDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_STAT);
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

void Xilinx7SeriesDevice::PrintStatusRegister()
{
	Xilinx7SeriesDeviceStatusRegister statreg;
	statreg.word = ReadWordConfigRegister(CONFIG_STAT);
	LogIndenter li;
	LogVerbose(
		"Status register is 0x%04x\n"
		"  CRC error         : %u\n"
		"  Secured           : %u\n"
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
		"  Decrypt error     : %u\n"
		"  XADC overheat     : %u\n"
		"  Startup state     : %u\n"
		"  Reserved          : %u\n"
		"  Bus width         : %u\n"
		"  Reserved          : %u\n",
		statreg.word,
		statreg.bits.crc_err,
		statreg.bits.part_secured,
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
		statreg.bits.dec_error,
		statreg.bits.xadc_over_temp,
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

	Reference: UG470 page 87

	Note that 7-series devices expect data clocked in MSB first but the JTAG API clocks data LSB first.
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

uint32_t Xilinx7SeriesDevice::ReadWordConfigRegister(unsigned int reg)
{
	//Send the read request
	SetIR(INST_CFG_IN);
	Xilinx7SeriesDeviceConfigurationFrame frames[] =
	{
		{ word: 0xaa995566 },												//Sync word
		{ bits: {0, 0, 0,   X7_CONFIG_OP_NOP,  X7_CONFIG_FRAME_TYPE_1} },	//Dummy word
		{ bits: {1, 0, reg, X7_CONFIG_OP_READ, X7_CONFIG_FRAME_TYPE_1} },	//Read the register
		{ bits: {0, 0, 0,   X7_CONFIG_OP_NOP,  X7_CONFIG_FRAME_TYPE_1} },	//Two dummy words required
		{ bits: {0, 0, 0,   X7_CONFIG_OP_NOP,  X7_CONFIG_FRAME_TYPE_1} }
	};

	unsigned char* packet = (unsigned char*) frames;
	FlipBitAndEndian32Array(packet, sizeof(frames));		//MSB needs to get sent first
	ScanDR(packet, NULL, sizeof(frames) * 8);

	//Read the data
	SetIR(INST_CFG_OUT);
	unsigned char unused[4] = {0};
	uint32_t reg_out = {0};
	ScanDR(unused, (unsigned char*)&reg_out, 32);
	FlipBitAndEndian32Array((unsigned char*)&reg_out, 4);

	SetIR(INST_BYPASS);

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
/*
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
*/
//void XilinxSpartan6Device::WriteWordConfigRegister(unsigned int /*reg*/, uint16_t /*value*/)
/*
{
	throw JtagExceptionWrapper(
		"WriteWordConfigRegister() not implemented",
		"");
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bitstream parsing

void Xilinx7SeriesDevice::ParseBitstreamInternals(
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

	//String names for config regs
	static const char* config_register_names[CONFIG_MAX]=
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
		Xilinx7SeriesDeviceConfigurationFrame frame =
			*reinterpret_cast<const Xilinx7SeriesDeviceConfigurationFrame*>(data + fpos);
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
			if(frame.bits.reg_addr >= CONFIG_MAX)
			{
				LogWarning("[Xilinx7SeriesDevice] Invalid register address 0x%x in config frame at bitstream offset %02x\n",
					frame.bits.reg_addr, (int)fpos);
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
					case CONFIG_CMD:
					{
						//Expect 1 word
						if(frame.bits.count != 1)
						{
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
					case CONFIG_IDCODE:
					{
						//Expect 1 word
						if(frame.bits.count != 1)
						{
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
			throw JtagExceptionWrapper(
				"Invalid frame type",
				"");
		}
	}
}

void Xilinx7SeriesDevice::Reboot()
{
	ResetToIdle();
	SetIR(INST_JPROGRAM);
	SetIR(INST_BYPASS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// On-chip debug helpers

size_t Xilinx7SeriesDevice::GetNumUserInstructions()
{
	return 4;	//generated by fair dice roll
				//guaranteed to be random
				//Just kidding, we have USER1...USER4
}

void Xilinx7SeriesDevice::SelectUserInstruction(size_t index)
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
