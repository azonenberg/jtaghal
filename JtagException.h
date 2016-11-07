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
	@brief Declaration of JtagException
 */
#ifndef JtagException_h
#define JtagException_h

/**
	@brief Base class for all exceptions thrown by libjtaghal

	\ingroup libjtaghal
 */
class JtagException
{
public:

	JtagException(
		std::string message,
		std::string library_error,
		std::string prettyfunction,
		std::string file,
		int line);

	std::string GetDescription() const;

	static void ThrowDummyException();

protected:

	///Error message
	std::string m_message;

	///String version of errno
	std::string m_system_error;

	///String version of library error
	std::string m_lib_error;

	///Pretty-printed function name
	std::string m_prettyfunction;

	///File name
	std::string m_file;

	///Line number
	int m_line;
};

/**
	@brief Wrapper for JtagException constructor that passes function, file, and line number automatically

	@param err			Human-readable error message. Include as much detail as reasonably possible.
	@param lib_err		Human-readable error string returned from a library (ex: libusb)
 */
#define JtagExceptionWrapper(err, lib_err) JtagException(err, lib_err, __PRETTY_FUNCTION__, __FILE__, __LINE__)

#endif
