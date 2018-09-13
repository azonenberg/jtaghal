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
	@brief Main library include file
 */

#ifndef jtaghal_h
#define jtaghal_h

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System headers

#define __STDC_FORMAT_MACROS

//Shared
#include <inttypes.h>

#ifdef _WIN32

//Windows
#include <ws2tcpip.h>
#include <windows.h>

#else

//POSIX
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// libc headers

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// libstdc++ headers

#include <list>
#include <map>
#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Miscellaneous utilities from other libraries we use

#include "../log/log.h"
#include "../xptools/Socket.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// File handle stuff

#ifdef _WIN32
	#define ZFILE_DESCRIPTOR HANDLE
#else
	#define ZFILE_DESCRIPTOR int
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class includes

//Error handling
#include "JtagException.h"

//Base interfaces
#include "TestInterface.h"
#include "TestableDevice.h"
#include "GPIOInterface.h"
#include "JtagDevice.h"
#include "JtagInterface.h"
#include "SWDDevice.h"
#include "SWDInterface.h"

//JTAG/SWD adapter drivers
#include "DigilentJtagInterface.h"
#include "FTDIDriver.h"
#include "FTDIJtagInterface.h"
#include "FTDISWDInterface.h"
#include "GlasgowSWDInterface.h"
#include "ServerInterface.h"
#include "NetworkedJtagInterface.h"
#include "PipeJtagInterface.h"
//#include "NocJtagInterface.h"

//Miscellaneous helper interfaces
#include "SerialNumberedDevice.h"
#include "LockableDevice.h"

//Programmable device helpers
#include "FirmwareImage.h"
#include "ByteArrayFirmwareImage.h"
#include "RawBinaryFirmwareImage.h"
#include "CPLDBitstream.h"
#include "FPGABitstream.h"

//Device classes
#include "DebuggerInterface.h"
#include "DebuggableDevice.h"
#include "ProgrammableDevice.h"
#include "ProgrammableLogicDevice.h"
#include "CPLD.h"
#include "FPGA.h"
#include "JtagDummy.h"
#include "JtagFPGA.h"
#include "Microcontroller.h"
#include "AttachedMemoryDevice.h"

//Vendor classes
#include "ARMDevice.h"
#include "FreescaleDevice.h"
#include "MicrochipDevice.h"
#include "STMicroDevice.h"
#include "XilinxDevice.h"

//Vendor device classes (and support stuff)
#include "ARMDebugPort.h"
#include "ARMJtagDebugPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMAPBDevice.h"
#include "ARMCoreSightDevice.h"
#include "ARMFlashPatchBreakpoint.h"
#include "ARMv7Processor.h"
#include "ARMv8Processor.h"
#include "ARMv7MProcessor.h"
#include "ARMCortexA57.h"
#include "ARMCortexA9.h"
#include "ARMCortexM4.h"
#include "ARM7TDMISProcessor.h"
#include "FreescaleMicrocontroller.h"
#include "FreescaleIMXDevice.h"
#include "FreescaleIMXSmartDMA.h"
#include "MicrochipMicrocontroller.h"
#include "STMicroMicrocontroller.h"
#include "STM32Device.h"
#include "XilinxCPLD.h"
#include "XilinxCPLDBitstream.h"
#include "XilinxCoolRunnerIIDevice.h"
#include "XilinxFPGABitstream.h"
#include "Xilinx3DFPGABitstream.h"
#include "XilinxFPGA.h"
#include "Xilinx7SeriesDevice.h"
#include "XilinxUltrascaleDevice.h"
#include "XilinxSpartan6Device.h"
#include "XilinxSpartan3ADevice.h"

//Debugging stuff
#include "DebuggableDevice.h"
#include "DebuggerInterface.h"

//NoC classes (TODO move to antikernel repo or something?)
//#include "RPCMessage.h"
//#include "NameServer.h"
//#include "NOCSwitchInterface.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global functions

//Byte manipulation
extern "C" bool PeekBit(const unsigned char* data, int nbit);
extern "C" void PokeBit(unsigned char* data, int nbit, bool val);
extern "C" unsigned char FlipByte(unsigned char c);

//Array manipulation
extern "C" void FlipByteArray(unsigned char* data, int len);
extern "C" void FlipBitArray(unsigned char* data, int len);
extern "C" void FlipEndianArray(unsigned char* data, int len);
extern "C" void FlipEndian32Array(unsigned char* data, int len);
extern "C" void FlipBitAndEndianArray(unsigned char* data, int len);
extern "C" void FlipBitAndEndian32Array(unsigned char* data, int len);

extern "C" void MirrorBitArray(unsigned char* data, int bitlen);

extern "C" uint16_t GetBigEndianUint16FromByteArray(const unsigned char* data, size_t offset);
extern "C" uint32_t GetBigEndianUint32FromByteArray(const unsigned char* data, size_t offset);

//Performance measurement
extern "C" double GetTime();

#endif
