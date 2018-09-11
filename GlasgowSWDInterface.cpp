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
	@brief Implementation of GlasgowSWDInterface
 */

#include <functional>
#include "jtaghal.h"

#if HAVE_LIBUSB

#define VID_QIHW		0x20b7
#define PID_GLASGOW		0x9db1

using namespace std;

static void ForEachGlasgowDevice(std::function<bool(libusb_device_descriptor *, libusb_device_handle *)> fn)
{
	int ret;

	libusb_context *context;
	if((ret = libusb_init(&context)) != 0) {
		throw JtagExceptionWrapper(
			"libusb_init failed",
			libusb_error_name(ret));
	}

	ssize_t device_count;
	libusb_device **device_list;
	if((device_count = libusb_get_device_list(context, &device_list)) < 0) {
		libusb_exit(context);

		throw JtagExceptionWrapper(
			"libusb_get_device_list failed",
			libusb_error_name(device_count));
	}

	for(int i = 0; i < device_count; i++) {
		libusb_device *device = device_list[i];
		libusb_device_descriptor device_desc;
		if((ret = libusb_get_device_descriptor(device, &device_desc)) != 0) {
			LogError("Cannot get USB device descriptor for device %03d:%03d\n",
			         libusb_get_bus_number(device),
			         libusb_get_port_number(device));
			continue;
		}

		if(device_desc.idVendor != VID_QIHW || device_desc.idProduct != PID_GLASGOW)
			continue;

		libusb_device_handle *device_handle;
		if((ret = libusb_open(device, &device_handle)) != 0) {
			LogError("Cannot open Glasgow device %03d:%03d\n",
			         libusb_get_bus_number(device),
			         libusb_get_port_number(device));
			continue;
		}

		bool done = fn(&device_desc, device_handle);

		libusb_close(device_handle);

		if(done) break;
	}

	libusb_free_device_list(device_list, /*unref_devices=*/true);

	libusb_exit(context);
}

int GlasgowSWDInterface::GetInterfaceCount()
{
	int count = 0;

	ForEachGlasgowDevice([&](libusb_device_descriptor *, libusb_device_handle *) {
		count++;
		return false;
	});

	return count;
}

string GlasgowSWDInterface::GetAPIVersion()
{
	return "";
}

string GlasgowSWDInterface::GetSerialNumber(int index)
{
	int ret;
	int curr_index = 0;
	string serial;

	ForEachGlasgowDevice([&](libusb_device_descriptor *desc, libusb_device_handle *handle) {
		if(curr_index == index) {
			char serial_array[64] = {};
			if((ret = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber,
					(uint8_t *)serial_array, sizeof(serial_array))) < 0) {
				serial = string("(error: ") + libusb_error_name(ret) + ")";
			} else {
				serial = string(serial_array, ret);
			}

			return true;
		} else {
			index++;
			return false;
		}
	});

	return serial;
}

string GlasgowSWDInterface::GetDescription(int index)
{
	int curr_index = 0;
	uint16_t revision = 0;

	ForEachGlasgowDevice([&](libusb_device_descriptor *desc, libusb_device_handle *) {
		if(curr_index == index) {
			revision = desc->bcdDevice;

			return true;
		} else {
			index++;
			return false;
		}
	});

	string description = "Glasgow rev";
	description += 'A' + (revision & 0xff) - 1;
	if((revision >> 8) == 0xa0 || (revision >> 8) == 0x00) {
		description += " (no firmware)";
	}
	return description;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Connects to a Glasgow SWD interface

	The Glasgow SWD applet must be configured as the sole applet in the device and use exclusive access mode.

	E.g. using `glasgow run swd`.

	@throw SWDException if the connection could not be establishes or the serial number is invalid

	@param serial       Serial number of the device to connect to
 */
GlasgowSWDInterface::GlasgowSWDInterface(const string& serial)
{
	int ret;

	if((ret = libusb_init(&m_context)) != 0) {
		throw JtagExceptionWrapper(
			"libusb_init failed",
			libusb_error_name(ret));
	}

	ssize_t device_count;
	libusb_device **device_list;
	if((device_count = libusb_get_device_list(m_context, &device_list)) < 0) {
		throw JtagExceptionWrapper(
			"libusb_get_device_list failed",
			libusb_error_name(device_count));
	}

	for(int i = 0; i < device_count; i++) {
		libusb_device *device = device_list[i];
		libusb_device_descriptor device_desc;
		if((ret = libusb_get_device_descriptor(device, &device_desc)) != 0) {
			LogError("Cannot get USB device descriptor for device %03d:%03d\n",
			         libusb_get_bus_number(device),
			         libusb_get_port_number(device));
			continue;
		}

		if(device_desc.idVendor != VID_QIHW || device_desc.idProduct != PID_GLASGOW)
			continue;

		libusb_device_handle *device_handle;
		if((ret = libusb_open(device, &device_handle)) != 0) {
			LogError("Cannot open Glasgow device %03d:%03d\n",
			         libusb_get_bus_number(device),
			         libusb_get_port_number(device));
			continue;
		}

		char device_serial[64];
		if((ret = libusb_get_string_descriptor_ascii(device_handle, device_desc.iSerialNumber,
				(uint8_t *)device_serial, sizeof(device_serial))) < 0) {
			LogError("Cannot get serial number for Glasgow device %03d:%03d\n",
			         libusb_get_bus_number(device),
			         libusb_get_port_number(device));
			libusb_close(device_handle);
			continue;
		}

		if(serial == "" || serial == string(device_serial, ret)) {
			m_device = libusb_ref_device(device);
			m_handle = device_handle;
			m_serial = device_serial;
			break;
		}
	}

	libusb_free_device_list(device_list, /*unref_devices=*/true);

	if(m_handle == NULL) {
		if(serial.empty()) {
			throw JtagExceptionWrapper(
				"No Glasgow device found",
				"");
		} else {
			throw JtagExceptionWrapper(
				"No Glasgow device found with serial " + serial,
				"");
		}
	}

	libusb_config_descriptor *config_desc;
	if((ret = libusb_get_active_config_descriptor(m_device, &config_desc)) != 0) {
		throw JtagExceptionWrapper(
			"libusb_get_config_descriptor failed",
			libusb_error_name(ret));
	}

	if(config_desc->bNumInterfaces < 1 ||
			config_desc->interface[0].num_altsetting < 1 ||
			config_desc->interface[0].altsetting[0].bNumEndpoints != 2) {
		libusb_free_config_descriptor(config_desc);
		throw JtagExceptionWrapper(
			"Malformed USB configuration descriptor: interfaces or endpoints missing",
			"");
	}

	const libusb_interface_descriptor *interface_desc = &config_desc->interface[0].altsetting[0];
	for(int i = 0; i < interface_desc->bNumEndpoints; i++) {
		const libusb_endpoint_descriptor *endpoint_desc = &interface_desc->endpoint[i];
		if(endpoint_desc->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
			m_ep_in = endpoint_desc->bEndpointAddress;
			m_packet_size_in = endpoint_desc->wMaxPacketSize;
		} else {
			m_ep_out = endpoint_desc->bEndpointAddress;
			m_packet_size_out = endpoint_desc->wMaxPacketSize;
		}
	}

	libusb_free_config_descriptor(config_desc);
	if(m_ep_in == 0 || m_ep_out == 0) {
		throw JtagExceptionWrapper(
			"Malformed USB configuration descriptor: unexpected endpoints",
			"");
	}
}

/**
	@brief Interface destructor

	Closes handles and disconnects from the adapter.
 */
GlasgowSWDInterface::~GlasgowSWDInterface()
{
	if(m_handle != NULL)
		libusb_close(m_handle);
	if(m_context != NULL)
		libusb_exit(m_context);
}

void GlasgowSWDInterface::WritePacket(vector<uint8_t> packet) {
	int ret, transferred;
	if((ret = libusb_bulk_transfer(m_handle, m_ep_out, packet.data(), packet.size(), &transferred, 0)) != 0 ||
	   		(size_t)transferred != packet.size()) {
		throw JtagExceptionWrapper(
			"libusb_bulk_transfer OUT failed",
			libusb_error_name(ret));
	}
}

vector<uint8_t> GlasgowSWDInterface::ReadPacket() {
	vector<uint8_t> packet;
	packet.resize(m_packet_size_in);

	int ret, transferred;
	if((ret = libusb_bulk_transfer(m_handle, m_ep_in, packet.data(), packet.size(), &transferred, 0)) != 0) {
		throw JtagExceptionWrapper(
			"libusb_bulk_transfer IN failed",
			libusb_error_name(ret));
	}

	packet.resize((size_t)transferred);
	return packet;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Shim overrides to push SWDInterface functions into FTDIDriver

string GlasgowSWDInterface::GetName()
{
	return "Glasgow";
}

string GlasgowSWDInterface::GetSerial()
{
	return m_serial;
}

string GlasgowSWDInterface::GetUserID()
{
	return "";
}

int GlasgowSWDInterface::GetFrequency()
{
	return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level

/**
	@brief Resets the SWD link layer
 */
void GlasgowSWDInterface::ResetInterface()
{
	vector<uint8_t> command = { 0xff };
	WritePacket(command);

	vector<uint8_t> reply = ReadPacket();
	if(reply != vector<uint8_t> { 0xff }) {
		throw JtagExceptionWrapper(
			string("SWD reset returned unexpected result: ") + std::to_string(reply[0]),
			"");
	}
}

/**
	@brief Performs a SW-DP write transaction
 */
void GlasgowSWDInterface::WriteWord(uint8_t reg_addr, bool ap, uint32_t wdata)
{
	vector<uint8_t> command = {
		(uint8_t)(0x80 | (ap << 3) | reg_addr),
		(uint8_t)(wdata >>  0),
		(uint8_t)(wdata >>  8),
		(uint8_t)(wdata >> 16),
		(uint8_t)(wdata >> 24),
	};
	WritePacket(command);

	vector<uint8_t> reply = ReadPacket();
	if(reply != vector<uint8_t> { 0x04 }) {
		throw JtagExceptionWrapper(
			string("SWD write returned unexpected result: ") + std::to_string(reply[0]),
			"");
	}
}

/**
	@brief Performs a SW-DP read transaction
 */
uint32_t GlasgowSWDInterface::ReadWord(uint8_t reg_addr, bool ap)
{
	vector<uint8_t> command = {
		(uint8_t)(0x80 | (ap << 3) | (1 << 2) | reg_addr),
	};
	WritePacket(command);

	vector<uint8_t> reply = ReadPacket();
	if(reply.size() != 5 || reply[0] != 0x04) {
		throw JtagExceptionWrapper(
			string("SWD read returned unexpected result: ") + std::to_string(reply[0]),
			"");
	}

	uint32_t word =
		(reply[1] <<  0) |
		(reply[2] <<  8) |
		(reply[3] << 16) |
		(reply[4] << 24);
	return word;
}

#endif
