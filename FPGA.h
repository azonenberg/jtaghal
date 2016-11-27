/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2016 Andrew D. Zonenberg                                                                          *
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
	@brief Declaration of FPGA
 */

#ifndef FPGA_h
#define FPGA_h

/**
	@brief Generic base class for all field-programmable gate array devices

	\ingroup libjtaghal
 */
class FPGA : public ProgrammableLogicDevice
{
public:
	virtual ~FPGA();

	/**
		@brief Determines if this device has a unique per-die serial number.

		@return true if a serial number is present, false if not
	 */
	virtual bool HasSerialNumber() =0;

	/**
		@brief Gets the length of the FPGA's unique serial number, in bytes (rounded up to the nearest whole byte).

		The result of calling this function if HasSerialNumber() returns false is undefined.

		@return Serial number length
	 */
	virtual int GetSerialNumberLength() =0;

	/**
		@brief Gets the length of the FPGA's unique serial number, in bits.

		The result of calling this function if HasSerialNumber() returns false is undefined.

		@return Serial number length
	 */
	virtual int GetSerialNumberLengthBits() =0;

	/**
		@brief Gets the FPGA's unique serial number.

		The result of calling this function if HasSerialNumber() returns false is undefined.

		Note that some architectures, such as Spartan-6, cannot read the serial number over JTAG without erasing the
		FPGA configuration. If this is the case, calling this function will automatically erase the FPGA.

		A future libjtaghal version may allow querying if this is the case.

		@throw JtagException if an error occurs during the read operation

		@param data Buffer to store the serial number into. Must be at least as large as the size given by
		GetSerialNumberLength().
	 */
	virtual void GetSerialNumber(unsigned char* data) =0;
};

#endif
