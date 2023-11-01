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
	@brief Implementation of JtagException
 */
#include "jtaghal.h"

using namespace std;

/**
	@brief Constructor for an exception object.

	The JtagExceptionWrapper() macro may be used to pass the last three parameters automatically.

	@param message			Human-readable error message. Include as much detail as reasonably possible.
	@param library_error	Human-readable error string returned from a library (ex: libusb)
	@param prettyfunction	Pretty-printed name of the current function. Pass __PRETTY_FUNCTION__
	@param file				The current source file. Pass __FILE__
	@param line				The current line number. Pass __LINE__
 */
JtagException::JtagException(
	std::string message,
	std::string library_error,
	std::string prettyfunction,
	std::string file,
	int line)
	: m_message(message)
	, m_system_error(strerror(errno))
	, m_lib_error(library_error)
	, m_prettyfunction(prettyfunction)
	, m_file(file)
	, m_line(line)
{
}

/**
	@brief Gets the description of this exception.

	The file name is truncated to the last 3 components for cleaner output.

	Example output:

	\verbatim
	JtagException object thrown from static void JtagException::ThrowDummyException()
        File         : .../src/jtaghal/JtagException.cpp/
        Line         : 136
        Library error:
        System error : Permission denied
        Message      : Test exception
    \endverbatim

	@return Printable exception description
 */
string JtagException::GetDescription() const
{
	//Parse the pathname at slashes
	int j = 0;
	size_t i = m_file.length() - 1;
	// size_t may be unsigned, so we don't use a for(), and instead check i before we decrement it
	while (1)
	{
		//Separator
		if(m_file[i] == '/' || m_file[i] == '\\')
			j++;
		if (i == 0 || j == 3)
			break;
		i--;
	}

	char temp_buf[2048];
	snprintf(
		temp_buf,
		sizeof(temp_buf)-1,
		"JtagException object thrown from %s\n"
		"    File        : %s%s\n"
		"    Line        : %d\n"
		"    Library err : %s\n"
		"    System err  : %s\n"
		"    Message     : %s\n",
			m_prettyfunction.c_str(),
			(i > 0) ? "..." : "",
			m_file.c_str() + i,
			m_line,
			m_lib_error.c_str(),
			m_system_error.c_str(),
			m_message.c_str()
		);
	return string(temp_buf);
}

/**
	@brief Throws an exception to test error handling code
 */
void JtagException::ThrowDummyException()
{
	throw JtagExceptionWrapper("Test exception", "");
}
