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
	@brief Implementation of XilinxFPGA
 */

#include "jtaghal.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Initializes this device

	@param idcode	The ID code of this device
	@param iface	The JTAG adapter this device was discovered on
	@param pos		Position in the chain that this device was discovered

 */
XilinxFPGA::XilinxFPGA(unsigned int idcode, JtagInterface* iface, size_t pos, size_t irlength)
	: JtagFPGA(idcode, iface, pos, irlength)
{
}

void XilinxFPGA::PostInitProbes(bool /*quiet*/)
{

}

/**
	@brief Default virtual destructor
 */
XilinxFPGA::~XilinxFPGA()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

bool XilinxFPGA::ReadingSerialRequiresReset()
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bitstream parsing

/**
	@brief Parse a bitstream image (common to all Xilinx devices)

	@throw JtagException if the bitstream is malformed or for the wrong device family

	@param bitstream	The bitstream object being initialized
	@param data			Pointer to the bitstream data
	@param len			Length of the bitstream

	@return A bitstream suitable for loading into this device
 */
void XilinxFPGA::ParseBitstreamCore(
	XilinxFPGABitstream* bitstream,
	const unsigned char* data,
	size_t len)
{
	//TODO: Some kind of ID code in FPGABitstream object so we can tell what type it is?

	/**
		Bitstream format

		13 unknown bytes (magic number?)			00 09 0f f0 0f f0 0f f0 0f f0 00 00 01
		Records
			Record type (1 byte, lowercase letter)
			Null byte
			Record length (1 byte)
	 */

	//Sanity check the first few bytes
	unsigned char magic[13] =
	{
		0x00, 0x09, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x0f, 0xf0, 0x00, 0x00, 0x01
	};
	if(0 != memcmp(data, magic, sizeof(magic)))
	{
		throw JtagExceptionWrapper(
			"Magic number in bitstream doesn't make sense (invalid bitstream? version number change?)",
			"");
	}

	//Make sure it's larger than 4KB.
	//No valid bitstream is that small, and it means we don't need to length-check the headers as much
	if(len < 4096)
	{
		throw JtagExceptionWrapper(
			"Bitstream is way too small - truncated file?",
			"");
	}

	size_t fpos = sizeof(magic);

	//Read records
	while(true)
	{
		unsigned char record_type = data[fpos++];
		fpos++;	//throw away the null
		unsigned char record_length = data[fpos++];

		switch(record_type)
		{
			//Human-readable bitstream description
			case 'a':
				{
					bitstream->desc = "";
					for(int i=0; i<record_length; i++)
					{
						if(data[fpos] == '\0')
						{
							fpos++;
							break;
						}
						bitstream->desc += (char)data[fpos++];
					}
				}
				break;

			//Device name (minus the "XC")
			case 'b':
				{
					bitstream->devname = "";
					for(int i=0; i<record_length; i++)
					{
						if(data[fpos] == '\0')
						{
							fpos++;
							break;
						}
						bitstream->devname += (char)data[fpos++];
					}
				}
				break;

			//Compilation date
			case 'c':
				{
					bitstream->date = "";
					for(int i=0; i<record_length; i++)
					{
						if(data[fpos] == '\0')
						{
							fpos++;
							break;
						}
						bitstream->date += (char)data[fpos++];
					}
				}
				break;

			//Compilation time:
			case 'd':
				{
					bitstream->time = "";
					for(int i=0; i<record_length; i++)
					{
						if(data[fpos] == '\0')
						{
							fpos++;
							break;
						}
						bitstream->time += (char)data[fpos++];
					}
				}
				break;

			//Unknown
			case 'e':
				{
					//Not sure how to parse this - seems to partially overlap with the bitstream
					//For now, scan until we find an 0xFF FF followed by an 0xAA 99
					//LogVerbose("Type E header (length %d)\n", record_length);
					for(int i=0; i<record_length; i++)
					{
						fpos++;
						//printf("%02x ", data[fpos] & 0xff);
						if(data[fpos] == 0xff && data[fpos+1] == 0xaa && data[fpos+2] == 0x99)
							break;
					}
				}
				break;

			default:
				//no idea
				LogWarning("Unknown bitstream header block \"%c\"\n", record_type);
		}

		if(record_type == 'e')
			break;
	}

	//It appears that Spartan-3A bitstreams have some junk here, skip it.
	if(bitstream->devname.find("3s") != string::npos)
		fpos += 2;

	//Search for the sync word
	//Should be all FF's, then an AA
	if(data[fpos] != 0xff)
	{
		LogError(
			"[XilinxFPGA] Expected 0xFF filler at end of bitstream headers (position 0x%x), found 0x%02x instead\n",
			(int)fpos,
			data[fpos]);

		throw JtagExceptionWrapper(
			"Expected 0xFF filler at end of bitstream headers, found something else instead",
			"");
	}
	for(; fpos < len; fpos ++)
	{
		if(data[fpos] == 0xaa)
			break;
	}
	if(fpos == len)
	{
		throw JtagExceptionWrapper(
			"Unexpected end of bitstream found",
			"");
	}

	//Call the derived class function to read the bulk bitstream data
	ParseBitstreamInternals(data, len, bitstream, fpos);
}

/*
void XilinxFPGA::ProgramIndirect(
	ByteArrayFirmwareImage* image,
	int buswidth,
	bool reboot,
	unsigned int base_address,
	string prog_image)
{
	//Program the FPGA with the indirect bitstream
	uint16_t faddr = LoadIndirectProgrammingImage(buswidth, prog_image);

	ByteArrayFirmwareImage* bitstream = dynamic_cast<ByteArrayFirmwareImage*>(image);
	if(bitstream == NULL)
	{
		throw JtagExceptionWrapper(
			"Invalid bitstream (not a ByteArrayFirmwareImage)",
			"",
			JtagException::EXCEPTION_TYPE_GIGO);
	}

	//Get the flash size
	RPCMessage rxm;
	RPCAndDMANetworkInterface* iface = dynamic_cast<RPCAndDMANetworkInterface*>(this);
	iface->RPCFunctionCallWithTimeout(faddr, NOR_GET_SIZE, 0, 0, 0, rxm, 5);
	unsigned int sector_count = rxm.data[1] / (4096*8);
	unsigned int size_KB = rxm.data[1] / 8192;
	printf("    Flash size is %u KB (%u sectors)\n", size_KB, sector_count);

	//Get the bitstream size
	printf("    Programming flash with %s\n", bitstream->GetDescription().c_str());
	unsigned int sectorlen = ceil(static_cast<float>(bitstream->raw_bitstream_len)/4096.0f);
	unsigned int wordmax = ceil(static_cast<float>(bitstream->raw_bitstream_len)/4.0f);
	printf("    Bitstream size is %.2f KB (%u words, %u 4KB sectors)\n",
		bitstream->raw_bitstream_len / 1024.0f,
		wordmax,
		sectorlen);

	vector<unsigned int> promdata;

	//If it's a FPGA bitstream, add some filler before the start.
	//Don't do this for software images etc
	if(dynamic_cast<XilinxFPGABitstream*>(bitstream) != NULL)
	{
		for(unsigned int i=0; i<4; i++)
			promdata.push_back(0xFFFFFFFF);
	}

	//Add the data itself
	for(unsigned int i=0; i<bitstream->raw_bitstream_len; i+=4)
		promdata.push_back(GetBigEndianUint32FromByteArray(bitstream->raw_bitstream, i));
	unsigned int nmax = promdata.size();

	//Add some filler at the end of the bitstream so we can safely read up to one flash page beyond (for sector writes)
	for(unsigned int i=0; i<512; i++)
		promdata.push_back(0xFFFFFFFF);

	//Flip word ordering
	FlipEndian32Array((unsigned char*)&promdata[0], promdata.size()*4);

	//Actually write to the flash
	printf("    Erasing (using design specific sector erase)...\n");
	for(unsigned int sec=0; sec<sectorlen; sec++)
		iface->RPCFunctionCallWithTimeout(faddr, NOR_PAGE_ERASE, 0, base_address + sec*4096, 0, rxm, 5);

	//Debug code: erase the whole chip and stop
	//TODO: Make a function for doing this
	//for(unsigned int sec=0; sec<sector_count; sec++)
	//	iface->RPCFunctionCallWithTimeout(faddr, NOR_PAGE_ERASE, 0, sec*4096, 0, rxm, 5);
	//return;

	printf("    Programming...\n");
	for(unsigned int i=0; i<nmax; i+=64)
		iface->DMAWrite(faddr, base_address + i*4, 64, &promdata[i], NOR_WRITE_DONE, NOR_OP_FAILED);

	//Verify
	printf("    Verifying...\n");
	unsigned int rdata[512] = {0};
	for(unsigned int block=0; block<(sectorlen*2); block++)
	{
		iface->DMARead(faddr, base_address + (block*512)*4, 512, rdata, NOR_OP_FAILED);

		for(unsigned int i=0; i<512; i++)
		{
			unsigned int n = block*512 + i;
			if(n >= wordmax)
				break;
			if(promdata[n] != rdata[i])
			{
				printf("    Mismatch (at word address %u, got %08x, expected %08x)\n",
					 n, rdata[i], promdata[n]);
				throw JtagExceptionWrapper(
					"Got bad data back from board",
					"",
					JtagException::EXCEPTION_TYPE_FIRMWARE);
			}
		}
	}

	//Reset the FPGA and load the new bitstream
	if(reboot)
	{
		printf("    Resetting FPGA...\n");
		Reboot();

		//Wait for it to boot
		for(int i=0; i<5; i++)
		{
			usleep(5 * 1000 * 1000);
			if(IsProgrammed())
				return;
		}

		PrintStatusRegister();
		throw JtagExceptionWrapper(
			"Timed out after 25 sec waiting for FPGA to boot from flash",
			"",
			JtagException::EXCEPTION_TYPE_BOARD_FAULT);
	}
}
*/

/**
	@brief Loads an indirect programming image suitable for the given bus width
 */
/*
uint16_t XilinxFPGA::LoadIndirectProgrammingImage(int buswidth, string image_fname)
{
	//Only support QSPI for now
	if(buswidth != 4)
	{
		throw JtagExceptionWrapper(
			"Unsupported SPI bus width (BPI and non-quad SPI not implemented yet).",
			"",
			JtagException::EXCEPTION_TYPE_GIGO);
	}

	//Find the correct indirect-programming image (TODO: don't hard code path - how do we get this?)
	string basepath = "/nfs4/home/azonenberg/code/antikernel/trunk/splashbuild/xilinx-fpga-";

	if(image_fname.empty())
	{
		static struct
		{
			string name;
			string bit;
		} images[] =
		{
			{ "Xilinx XC6SLX9",  "spartan6-xc6slx9-2tqg144/IndirectFlash-xc6slx9-2tqg144.bit"},
			{ "Xilinx XC6SLX16", "spartan6-xc6slx16-2ftg256/IndirectFlash-xc6slx16-2ftg256.bit"},
			{ "Xilinx XC6SLX25", "spartan6-xc6slx25-2ftg256/IndirectFlash-xc6slx25-2ftg256.bit"},
			{ "Xilinx XC6SLX45", "spartan6-xc6slx45-2csg324/IndirectFlash-xc6slx45-2csg324.bit"},
			{ "Xilinx XC7A200T", "artix7-xc7a200t-1fbg676/IndirectFlash-xc7a200t-1fbg676.bit"},
			{ "Xilinx XC7K70T",  "kintex7-xc7k70t-1fbg484/IndirectFlash-xc7k70t-1fbg484.bit"}
		};

		bool found = false;
		for(auto x : images)
		{
			if(GetDescription().find(x.name) != string::npos)
			{
				basepath += x.bit;
				found = true;
				break;
			}
		}

		if(!found)
		{
			throw JtagExceptionWrapper(
				"The selected FPGA does not have an indirect SPI programming image available",
				"",
				JtagException::EXCEPTION_TYPE_FIRMWARE);
		}
	}
	else
		basepath = image_fname;

	//Load the SPI image
	FirmwareImage* spi_image = LoadFirmwareImage(basepath);
	printf("    Loading indirect programming image...\n");
	Program(spi_image);
	delete spi_image;

	//Verify we got something
	ProbeVirtualTAPs();
	if(!HasRPCInterface())
	{
		throw JtagExceptionWrapper(
			"No RPC network interface found",
			"",
			JtagException::EXCEPTION_TYPE_FIRMWARE);
	}

	RPCAndDMANetworkInterface* iface = dynamic_cast<RPCAndDMANetworkInterface*>(this);
	if(iface == NULL)
	{
		throw JtagExceptionWrapper(
			"Not an RPCAndDMANetworkInterface",
			"",
			JtagException::EXCEPTION_TYPE_FIRMWARE);
	}

	//Address lookup
	printf("    Looking up flash info\n");
	NameServer nameserver(iface);
	uint16_t faddr = nameserver.ForwardLookup("flash");
	printf("    Flash is at NoC address %04x\n", faddr);
	return faddr;

	return 0;
}

void XilinxFPGA::DumpIndirect(int buswidth, string fname)
{
	//Program the FPGA with the indirect bitstream
	uint16_t faddr = LoadIndirectProgrammingImage(buswidth);

	//Get the flash size
	RPCMessage rxm;
	RPCAndDMANetworkInterface* iface = dynamic_cast<RPCAndDMANetworkInterface*>(this);
	iface->RPCFunctionCallWithTimeout(faddr, NOR_GET_SIZE, 0, 0, 0, rxm, 5);
	unsigned int sector_count = rxm.data[1] / (4096*8);
	unsigned int read_count = sector_count * 2;
	unsigned int size_KB = rxm.data[1] / 8192;
	printf("    Flash size is %u KB (%u sectors, %u read blocks)\n", size_KB, sector_count, read_count);

	//Open output file
	FILE* fp = fopen(fname.c_str(), "wb");
	if(fp == NULL)
	{
		throw JtagExceptionWrapper(
			"Couldn't open output file",
			"",
			JtagException::EXCEPTION_TYPE_GIGO);
	}

	//Dump it
	uint32_t rdata[512];
	printf("    Dumping...\n");
	for(unsigned int block=0; block<read_count; block++)
	{
		iface->DMARead(faddr, block*2048, 512, rdata, NOR_OP_FAILED);
		fwrite(rdata, 512, 4,  fp);
	}

	fclose(fp);
}
*/
