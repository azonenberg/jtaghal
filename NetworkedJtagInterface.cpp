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
	@brief Implementation of NetworkedJtagInterface
 */
#include "jtaghal.h"
#include "ProtobufHelpers.h"

using namespace std;

/**
	@brief Creates the interface object but does not connect to a server.
 */
NetworkedJtagInterface::NetworkedJtagInterface()
{
}

/**
	@brief Disconnects from the server
 */
NetworkedJtagInterface::~NetworkedJtagInterface()
{

}

/**
	@brief Connects to a jtagd server.

	@throw JtagException if the connection could not be established

	@param server	Hostname of the server to connect to
	@param port		Port number (in host byte ordering) the server is running on
 */
void NetworkedJtagInterface::Connect(const string& server, uint16_t port)
{
	ServerInterface::DoConnect(server, port, Hello::TRANSPORT_JTAG);

	//Check if we support split scans
	JtaghalPacket packet;
	packet.mutable_splitrequest();
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send splitScanSupportedRequest",
			"");
	}
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get reply",
			"");
	}
	auto r = packet.inforeply();
	if(r.num())
		m_splitScanSupported = true;
	else
		m_splitScanSupported = false;
}

string NetworkedJtagInterface::GetName()
{
	return ServerInterface::GetName();
}

string NetworkedJtagInterface::GetSerial()
{
	return ServerInterface::GetSerial();
}

string NetworkedJtagInterface::GetUserID()
{
	return ServerInterface::GetUserID();
}

int NetworkedJtagInterface::GetFrequency()
{
	return ServerInterface::GetFrequency();
}

void NetworkedJtagInterface::ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count)
{
	double start = GetTime();
	size_t bytesize =  ceil(count / 8.0f);

	//Send the request data
	JtaghalPacket packet;
	auto r = packet.mutable_scanrequest();
	r->set_readrequested(rcv_data != NULL);
	r->set_totallen(count);
	r->set_settmsatend(last_tms);
	r->set_writedata(string((char*)send_data, bytesize));
	r->set_split(false);				//not doing a split transfer
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send scanRequest",
			"");
	}

	//Get the reply
	if(rcv_data != NULL)
	{
		//Get the reply
		if(!RecvMessage(m_socket, packet, JtaghalPacket::kScanReply))
		{
			throw JtagExceptionWrapper(
				"Failed to get scanReply",
				"");
		}
		auto r = packet.scanreply();
		if(r.readdata().size() != bytesize)
		{
			throw JtagExceptionWrapper(
				"RX byte length mismatch",
				"");
		}
		memcpy(rcv_data, r.readdata().c_str(), bytesize);
	}

	m_perfShiftTime += GetTime() - start;
}

bool NetworkedJtagInterface::IsSplitScanSupported()
{
	return m_splitScanSupported;
}

bool NetworkedJtagInterface::ShiftDataWriteOnly(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count)
{
	double start = GetTime();
	size_t bytesize =  ceil(count / 8.0f);

	//Send the request data
	JtaghalPacket packet;
	auto r = packet.mutable_scanrequest();
	r->set_readrequested(rcv_data != NULL);
	r->set_totallen(count);
	r->set_settmsatend(last_tms);
	r->set_writedata(string((char*)send_data, bytesize));
	r->set_split(true);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send scanRequest",
			"");
	}

	m_perfShiftTime += GetTime() - start;
	return true;
}

bool NetworkedJtagInterface::ShiftDataReadOnly(unsigned char* rcv_data, size_t count)
{
	double start = GetTime();
	size_t bytesize =  ceil(count / 8.0f);

	//Send the request data
	JtaghalPacket packet;
	auto r = packet.mutable_scanrequest();
	r->set_readrequested(true);
	r->set_totallen(count);
	//tms is a dontcare
	r->set_writedata("");
	r->set_split(true);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send scanRequest",
			"");
	}

	//Get the reply
	if(rcv_data != NULL)
	{
		//Get the reply
		if(!RecvMessage(m_socket, packet, JtaghalPacket::kScanReply))
		{
			throw JtagExceptionWrapper(
				"Failed to get scanReply",
				"");
		}
		auto r = packet.scanreply();
		if(r.readdata().size() != bytesize)
		{
			throw JtagExceptionWrapper(
				"RX byte length mismatch",
				"");
		}
		memcpy(rcv_data, r.readdata().c_str(), bytesize);
	}

	m_perfShiftTime += GetTime() - start;
	return true;
}

void NetworkedJtagInterface::ShiftTMS(bool /*tdi*/, const unsigned char* /*send_data*/, size_t /*count*/)
{
	throw JtagExceptionWrapper(
		"NetworkedJtagInterface::ShiftTMS() is not supported (use state-level interface only)",
		"");
}

void NetworkedJtagInterface::SendDummyClocks(size_t n)
{
	double start = GetTime();

	//Send the request data
	JtaghalPacket packet;
	auto r = packet.mutable_scanrequest();
	r->set_readrequested(false);
	r->set_totallen(n);
	//tms is a dontcare
	r->set_writedata("");
	r->set_split(false);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send scanRequest",
			"");
	}

	m_perfShiftTime += GetTime() - start;
}

void NetworkedJtagInterface::SendDummyClocksDeferred(size_t n)
{
	//TODO: separate protobuf API for this?
	SendDummyClocks(n);
}

void NetworkedJtagInterface::TestLogicReset()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::TestLogicReset);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::EnterShiftIR()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::EnterShiftIR);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::LeaveExit1IR()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::LeaveExitIR);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::EnterShiftDR()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::EnterShiftDR);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::LeaveExit1DR()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::LeaveExitDR);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::ResetToIdle()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_staterequest();
	r->set_state(JtagStateChangeRequest::ResetToIdle);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}
}

void NetworkedJtagInterface::Commit()
{
	//Send the flush request
	JtaghalPacket packet;
	packet.mutable_flushrequest();
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send flushRequest",
			"");
	}

	//TODO: should this be a barrier sync where we block?
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Performance profiling

size_t NetworkedJtagInterface::GetShiftOpCount()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_perfrequest();
	r->set_req(JtagPerformanceRequest::ShiftOps);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send perfRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().num();
}

size_t NetworkedJtagInterface::GetDataBitCount()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_perfrequest();
	r->set_req(JtagPerformanceRequest::DataBits);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send perfRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().num();
}

size_t NetworkedJtagInterface::GetModeBitCount()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_perfrequest();
	r->set_req(JtagPerformanceRequest::ModeBits);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send perfRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().num();
}

size_t NetworkedJtagInterface::GetDummyClockCount()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_perfrequest();
	r->set_req(JtagPerformanceRequest::DummyClocks);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send perfRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().num();
}

//GetShiftTime is measured clientside so no need to override
