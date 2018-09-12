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

bool SendMessage(Socket& s, const JtaghalPacket& msg)
{
	string buf;
	if(!msg.SerializeToString(&buf))
	{
		LogWarning("Failed to serialize protobuf\n");
		return false;
	}
	if(!s.SendPascalString(buf))
	{
		//LogWarning("Connection to %s dropped (while sending protobuf)\n", hostname.c_str());
		return false;
	}
	return true;
}

bool RecvMessage(Socket& s, JtaghalPacket& msg)
{
	string buf;
	if(!s.RecvPascalString(buf))
	{
		//LogWarning("Connection to %s dropped (while reading protobuf)\n", hostname.c_str());
		return false;
	}
	if(!msg.ParseFromString(buf))
	{
		LogWarning("Failed to parse protobuf\n");
		return false;
	}
	return true;
}

bool RecvMessage(Socket& s, JtaghalPacket& msg, JtaghalPacket::PayloadCase expectedType)
{
	if(!RecvMessage(s, msg))
		return false;
	if(msg.Payload_case() != expectedType)
	{
		LogWarning("Got incorrect message type\n");
		return false;
	}
	return true;
}

/**
	@brief Creates the interface object but does not connect to a server.
 */
NetworkedJtagInterface::NetworkedJtagInterface()
	: m_socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP)
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
	//Connect to the port
	if(!m_socket.Connect(server, port))
	{
		throw JtagExceptionWrapper(
			"Failed to connect to server",
			"");
	}

	//Set no-delay flag
	if(!m_socket.DisableNagle())
	{
		throw JtagExceptionWrapper(
			"Failed to set TCP_NODELAY",
			"");
	}

	//Send the ClientHello
	JtaghalPacket packet;
	auto h = packet.mutable_hello();
	h->set_magic("JTAGHAL");
	h->set_version(1);
	h->set_transport(Hello::TRANSPORT_JTAG);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send clienthello",
			"");
	}

	//Get the server-hello message
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kHello))
	{
		throw JtagExceptionWrapper(
			"Failed to get serverhello",
			"");
	}
	auto sh = packet.hello();
	if( (sh.magic() != "JTAGHAL") || (sh.version() != 1) )
	{
		throw JtagExceptionWrapper(
			"ServerHello has wrong magic/version",
			"");
	}

	//Make sure the server is JTAG
	if(sh.transport() != Hello::TRANSPORT_JTAG)
	{
		throw JtagExceptionWrapper(
			"Tried to connect to server as JTAG, but the server is using a different transport protocol",
			"");
	}

	//All good, query the GPIO stats
	if(IsGPIOCapable())
	{
		//Load the GPIO pin state from the server
		ReadGpioState();
	}

	//Check if we support split scans
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

/**
	@brief Disconnects from the server
 */
NetworkedJtagInterface::~NetworkedJtagInterface()
{
	try
	{
		if(m_socket.IsValid())
		{
			JtaghalPacket packet;
			packet.mutable_disconnectrequest();
			SendMessage(m_socket, packet);
		}
	}
	catch(const JtagInterface& ex)
	{
		//Ignore errors in the write_looped call since we're disconnecting anyway
	}
}

/**
	@brief Returns the protocol version
 */
string NetworkedJtagInterface::GetAPIVersion()
{
	return "1.0";
}

/**
	@brief Returns the constant 1.
 */
int NetworkedJtagInterface::GetInterfaceCount()
{
	return 1;
}

string NetworkedJtagInterface::GetName()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_inforequest();
	r->set_req(InfoRequest::HwName);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().str();
}

string NetworkedJtagInterface::GetSerial()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_inforequest();
	r->set_req(InfoRequest::HwSerial);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().str();
}

string NetworkedJtagInterface::GetUserID()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_inforequest();
	r->set_req(InfoRequest::Userid);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kInfoReply))
	{
		throw JtagExceptionWrapper(
			"Failed to get infoReply",
			"");
	}
	return packet.inforeply().str();
}

int NetworkedJtagInterface::GetFrequency()
{
	//Send the infoRequest
	JtaghalPacket packet;
	auto r = packet.mutable_inforequest();
	r->set_req(InfoRequest::Freq);
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send infoRequest",
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO stuff

/**
	@brief Checks whether the remote JTAG adapter provides GPIO capability.

	@return True if GPIO capable.
 */
bool NetworkedJtagInterface::IsGPIOCapable()
{
	//Send the gpioReadRequest
	JtaghalPacket packet;
	packet.mutable_gpioreadrequest();
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send gpioReadRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kBankState))
	{
		throw JtagExceptionWrapper(
			"Failed to get bankState",
			"");
	}
	auto state = packet.bankstate();
	if(state.states_size() != 0)
		return true;
	else
		return false;
}

void NetworkedJtagInterface::ReadGpioState()
{
	//Send the gpioReadRequest
	JtaghalPacket packet;
	packet.mutable_gpioreadrequest();
	if(!SendMessage(m_socket, packet))
	{
		throw JtagExceptionWrapper(
			"Failed to send gpioReadRequest",
			"");
	}

	//Get the reply
	if(!RecvMessage(m_socket, packet, JtaghalPacket::kBankState))
	{
		throw JtagExceptionWrapper(
			"Failed to get bankState",
			"");
	}
	auto state = packet.bankstate();

	//Enlarge our GPIO state buffer as needed
	m_gpioDirection.resize(state.states_size());
	m_gpioValue.resize(state.states_size());

	//Crunch the incoming data
	for(ssize_t i=0; i<state.states_size(); i++)
	{
		m_gpioValue[i] = state.states(i).value();
		m_gpioDirection[i] = state.states(i).is_output();
	}
}

void NetworkedJtagInterface::WriteGpioState()
{
	/*
	uint8_t op = JTAGD_OP_WRITE_GPIO_STATE;
	m_socket.SendLooped((unsigned char*)&op, 1);

	int count = m_gpioDirection.size();
	vector<uint8_t> pinstates;
	for(int i=0; i<count; i++)
	{
		pinstates.push_back(
			m_gpioValue[i] |
			(m_gpioDirection[i] << 1)
			);
	}
	m_socket.SendLooped((unsigned char*)&pinstates[0], count);
	*/
	throw JtagExceptionWrapper(
		"Unimplemented",
		"");
}
