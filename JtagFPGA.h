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
	@brief Declaration of JtagFPGA
 */

#ifndef JtagFPGA_h
#define JtagFPGA_h

/**
	@brief Abstract base class for all JTAG-programmed FPGAs

	\ingroup libjtaghal
 */
class JtagFPGA	: public JtagDevice
				, public FPGA
{
public:
	JtagFPGA(unsigned int idcode, JtagInterface* iface, size_t pos, size_t irlength);
	virtual ~JtagFPGA();

	/**
		@brief Get the number of JTAG instructions which are routed to FPGA fabric
	 */
	virtual size_t GetNumUserInstructions() =0;

	/**
		@brief Sets the instruction register to the specified user instruction
	 */
	virtual void SelectUserInstruction(size_t index) = 0;

	/**
		@brief Gets the vendor/product code in USER1 (if implemented)

		Reference: https://github.com/azonenberg/jtaghal/wiki/FPGA-debug

		@return false on error, or if no USER instructions
	 */
	bool GetUserVIDPID(unsigned int& idVendor, unsigned int& idProduct);
};

#endif

