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
	@brief Declaration of SerialNumberedDevice
 */

#ifndef SerialNumberedDevice_h
#define SerialNumberedDevice_h

/**
	@brief Abstract base class for all devices that have a unique die serial number

	\ingroup libjtaghal
 */
class SerialNumberedDevice
{
public:
	SerialNumberedDevice();
	virtual ~SerialNumberedDevice();

	/**
		@brief True if reading this serial number requires a device reset.

		Applications may choose not to display the serial number to avoid disrupting the running code.
	 */
	virtual bool ReadingSerialRequiresReset() = 0;

	/**
		@brief Gets the length of the FPGA's unique serial number, in bytes (rounded up to the nearest whole byte).

		@return Serial number length
	 */
	virtual int GetSerialNumberLength() =0;

	/**
		@brief Gets the length of the FPGA's unique serial number, in bits.

		@return Serial number length
	 */
	virtual int GetSerialNumberLengthBits() =0;

	/**
		@brief Gets the FPGA's unique serial number.

		Note that some architectures, such as Spartan-6, cannot read the serial number over JTAG without erasing the
		FPGA configuration. If this is the case, calling this function will automatically erase the FPGA.

		Call ReadingSerialRequiresReset() to see if this is the case.

		@throw JtagException if an error occurs during the read operation

		@param data Buffer to store the serial number into. Must be at least as large as the size given by
		GetSerialNumberLength().
	 */
	virtual void GetSerialNumber(unsigned char* data) =0;
};

#endif

