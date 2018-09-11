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
	@author whitequark
	@brief Declaration of GlasgowSWDInterface
 */

#ifndef GlasgowSWDInterface_h
#define GlasgowSWDInterface_h

#if HAVE_LIBUSB

#include "SWDInterface.h"
#include <libusb-1.0/libusb.h>

/**
	@brief A SWD adapter using the Glasgow debug tool, accessed through libusb

	\ingroup interfaces
 */
class GlasgowSWDInterface  : public SWDInterface
{
public:
	static int GetInterfaceCount();
	static std::string GetAPIVersion();
	static std::string GetSerialNumber(int index);
	static std::string GetDescription(int index);

	GlasgowSWDInterface(const std::string& serial);
	virtual ~GlasgowSWDInterface();

	virtual void WriteWord(uint8_t reg_addr, bool ap, uint32_t wdata);
	virtual uint32_t ReadWord(uint8_t reg_addr, bool ap);
	virtual void ResetInterface();

	virtual std::string GetName();
	virtual std::string GetSerial();
	virtual std::string GetUserID();
	virtual int GetFrequency();

private:
	std::string m_serial;
	libusb_context *m_context = NULL;
	libusb_device *m_device = NULL;
	libusb_device_handle *m_handle = NULL;
	uint8_t m_ep_in = 0;
	uint8_t m_ep_out = 0;
	uint16_t m_packet_size_in = 0;
	uint16_t m_packet_size_out = 0;

	void WritePacket(std::vector<uint8_t> packet);
	std::vector<uint8_t> ReadPacket();
};

#endif

#endif
