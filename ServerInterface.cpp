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
	@brief Implementation of ServerInterface
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Creates the interface object but does not connect to a server.
 */
ServerInterface::ServerInterface()
	: m_socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP)
{
}

ServerInterface::~ServerInterface()
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization

/**
	@brief Connects to a jtagd server.

	@throw JtagException if the connection could not be established

	@param server	Hostname of the server to connect to
	@param port		Port number (in host byte ordering) the server is running on
 */
void ServerInterface::DoConnect(const string& server, uint16_t port, int transport)
{
	Hello_TransportType tp = (Hello_TransportType)transport;

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
	h->set_transport(tp);
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
	if(sh.transport() != tp)
	{
		throw JtagExceptionWrapper(
			"Server is using a different transport protocol",
			"");
	}

	//All good, query the GPIO stats
	if(IsGPIOCapable())
	{
		//Load the GPIO pin state from the server
		ReadGpioState();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

/**
	@brief Returns the protocol version
 */
string ServerInterface::GetAPIVersion()
{
	return "1.0";
}

/**
	@brief Returns the constant 1.
 */
int ServerInterface::GetInterfaceCount()
{
	return 1;
}

string ServerInterface::GetName()
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

string ServerInterface::GetSerial()
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

string ServerInterface::GetUserID()
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

int ServerInterface::GetFrequency()
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO stuff

/**
	@brief Checks whether the remote JTAG adapter provides GPIO capability.

	@return True if GPIO capable.
 */
bool ServerInterface::IsGPIOCapable()
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

void ServerInterface::ReadGpioState()
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

void ServerInterface::WriteGpioState()
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
