/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
	@brief Implementation of PipeJtagInterface
 */
#include "jtaghal.h"
#include "jtagd_opcodes_enum.h"

using namespace std;

/**
	@brief Creates the interface object and connects to the pipes (TODO: support more than one)
 */
PipeJtagInterface::PipeJtagInterface()
{
	LogNotice("Opening write pipe\n");
	m_writepipe = fopen("/tmp/simreadpipe", "w");
	LogNotice("Opening read pipe\n");
	m_readpipe = fopen("/tmp/simwritepipe", "r");
	LogNotice("Pipes opened\n");
}

/**
	@brief Disconnects from the server
 */
PipeJtagInterface::~PipeJtagInterface()
{
	uint8_t op = JTAGD_OP_QUIT;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);

	if(m_readpipe)
	{
		fclose(m_readpipe);
		m_readpipe = NULL;
	}

	if(m_writepipe)
	{
		fclose(m_writepipe);
		m_writepipe = NULL;
	}
}

/**
	@brief Returns the protocol version
 */
string PipeJtagInterface::GetAPIVersion()
{
	return "1.0";
}

/**
	@brief Returns the constant 1.
 */
int PipeJtagInterface::GetInterfaceCount()
{
	return 1;
}

string PipeJtagInterface::ReadString()
{
	char tmp[1024];
	fscanf(m_readpipe, "%1023s", tmp);
	return string(tmp);
}

string PipeJtagInterface::GetName()
{
	uint8_t op = JTAGD_OP_GET_NAME;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
	return ReadString();
}

string PipeJtagInterface::GetSerial()
{
	uint8_t op = JTAGD_OP_GET_SERIAL;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
	return ReadString();
}

string PipeJtagInterface::GetUserID()
{
	uint8_t op = JTAGD_OP_GET_USERID;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
	return ReadString();
}

int PipeJtagInterface::GetFrequency()
{
	uint8_t op = JTAGD_OP_GET_FREQ;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);

	int freq;
	fscanf(m_readpipe, "%d", &freq);
	return freq;
}

void PipeJtagInterface::ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count)
{
	double start = GetTime();

	int bytesize =  ceil(count / 8.0f);

	//Opcode
	uint8_t op = JTAGD_OP_SHIFT_DATA;
	if(rcv_data == NULL)
		op = JTAGD_OP_SHIFT_DATA_WO;
	fprintf(m_writepipe, "%02x\n", op);

	//Last TMS value
	fprintf(m_writepipe, "%02x\n", last_tms);

	//Message length (in BITS, not bytes)
	fprintf(m_writepipe, "%08x\n", count);

	//Actual message data (one byte per line)
	for(int i=0; i<bytesize; i++)
		fprintf(m_writepipe, "%02x\n", send_data[i] & 0xff);

	fflush(m_writepipe);

	//Read response data
	if(rcv_data != NULL)
	{
		for(int i=0; i<bytesize; i++)
		{
			unsigned int tmp;
			fscanf(m_readpipe, "%02x", &tmp);
			rcv_data[i] = tmp;
		}
	}

	m_perfShiftTime += GetTime() - start;
}

bool PipeJtagInterface::IsSplitScanSupported()
{
	return false;
}

bool PipeJtagInterface::ShiftDataWriteOnly(
	bool /*last_tms*/,
	const unsigned char* /*send_data*/,
	unsigned char* /*rcv_data*/,
	size_t /*count*/)
{
	return false;
}

bool PipeJtagInterface::ShiftDataReadOnly(
	unsigned char* /*rcv_data*/,
	size_t /*count*/)
{
	return false;
}

void PipeJtagInterface::ShiftTMS(bool /*tdi*/, const unsigned char* /*send_data*/, size_t /*count*/)
{
	throw JtagExceptionWrapper(
		"PipeJtagInterface::ShiftTMS() is not supported (use state-level interface only)",
		"");
}

void PipeJtagInterface::SendDummyClocks(size_t /*n*/)
{
	LogError("SendDummyClocks not implemented\n");

	/*
	double start = GetTime();

	uint8_t op = JTAGD_OP_DUMMY_CLOCK;
	BufferedSend((unsigned char*)&op, 1);
	uint32_t c = n;
	BufferedSend((unsigned char*)&c, 4);
	Commit();

	m_perfShiftTime += GetTime() - start;
	*/
}

void PipeJtagInterface::SendDummyClocksDeferred(size_t n)
{
	SendDummyClocks(n);	//no deferral supported
}

void PipeJtagInterface::TestLogicReset()
{
	uint8_t op = JTAGD_OP_TLR;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::EnterShiftIR()
{
	uint8_t op = JTAGD_OP_ENTER_SIR;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::LeaveExit1IR()
{
	uint8_t op = JTAGD_OP_LEAVE_E1IR;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::EnterShiftDR()
{
	uint8_t op = JTAGD_OP_ENTER_SDR;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::LeaveExit1DR()
{
	uint8_t op = JTAGD_OP_LEAVE_E1DR;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::ResetToIdle()
{
	uint8_t op = JTAGD_OP_RESET_IDLE;
	fprintf(m_writepipe, "%02x\n", op);
	fflush(m_writepipe);
}

void PipeJtagInterface::Commit()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Performance profiling

size_t PipeJtagInterface::GetShiftOpCount()
{
	return 0;
}

size_t PipeJtagInterface::GetRecoverableErrorCount()
{
	return 0;
}

size_t PipeJtagInterface::GetDataBitCount()
{
	return 0;
}

size_t PipeJtagInterface::GetModeBitCount()
{
	return 0;
}

size_t PipeJtagInterface::GetDummyClockCount()
{
	return 0;
}

//GetShiftTime is measured clientside so no need to override
