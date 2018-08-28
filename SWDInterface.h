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
	@brief Declaration of SWDInterface
 */

#include "ARMDebugAccessPort.h"
#include "ARMDebugMemAccessPort.h"
#include "ARMDebugPort.h"

#ifndef SWDInterface_h
#define SWDInterface_h

/**
	@brief Abstract representation of a SWD adapter.

	A SWD adapter provides access to a single SW-DP (TODO: or SWJ-DP) on a single ARM SoC.

	In order to support a new "dumb" SWD adapter without any higher level protocol offload, create a new derived class
	and implement each of the following functions:

	\li GetName()
	\li GetSerial()
	\li GetUserID()
	\li GetFrequency()
 */
class SWDInterface	: public ARMDebugPort
					, public TestInterface
{
public:
	SWDInterface();
	virtual ~SWDInterface();

	//GetInterfaceCount() is a strongly recommended static member function for each derived class

	//Setup stuff
public:

	/**
		@brief Performs a SW-DP write transaction
	 */
	virtual void WriteWord(uint8_t reg_addr, uint32_t wdata) =0;

	/**
		@brief Performs a SW-DP read transaction
	 */
	virtual uint32_t ReadWord(uint8_t reg_addr) =0;

public:

	//Scanning stuff
public:

	/**
		@brief Connects to the interface and figures out what we have attached to us
	 */
	virtual void InitializeDevice();

	//Device(s) found on the SWD interface
	//For now: only support for one
	/*
protected:
	SWDDevice* m_device;

public:
	SWDDevice* GetDevice()
	{ return m_device; }*/
};

#endif
