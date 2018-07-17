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
	@brief Declaration of LockableDevice
 */

#ifndef LockableDevice_h
#define LockableDevice_h

/**
	@brief A boolean value with an attached level of uncertainty
 */
class UncertainBoolean
{
public:

	enum CertaintyLevel
	{
		USELESS,		//The value is useless and should be ignored.
						//The measurement may not have been possible to perform.
		INCONSISTENT,	//The value changed or was unreliable.
						//For example, multiple values may have been seen across an address range.

		VERY_LIKELY,	//Very likely but not 100% certain.

		CERTAIN			//Absolutely certain. There are no plausible scenarios in which this value might be wrong
	};

	UncertainBoolean(bool b, CertaintyLevel level)
		: m_value(b)
		, m_certainty(level)
	{
	}

	CertaintyLevel GetCertainty()
	{ return m_certainty; }

	bool GetValue()
	{ return m_value; }

	const char* GetCertaintyAsText()
	{
		switch(m_certainty)
		{
			case INCONSISTENT:
				return "inconsistent results";

			case VERY_LIKELY:
				return "high confidence";

			case CERTAIN:
				return "extremely high confidence";

			case USELESS:
			default:
				return "completely unsure";
		}
	}

protected:
	bool			m_value;
	CertaintyLevel	m_certainty;
};

/**
	@brief Generic base class for all devices which have some kind of read/write protection

	Note that sometimes due to protections, it's not possible to get definite answers to all queries.

	\ingroup libjtaghal
 */
class LockableDevice
{
public:
	LockableDevice();
	virtual ~LockableDevice();

	/**
		@brief Levels of access being requested (may be ORed together)
	 */
	enum AccessLevel
	{
		ACCESS_EXECUTE	= 1,
		ACCESS_WRITE	= 2,
		ACCESS_READ		= 4
	};

	/**
		@brief Queries lock status in a non-destructive fashion (contents of the chip are untouched)
	 */
	virtual void ProbeLocksNondestructive() =0;

	/**
		@brief Queries lock status in a more invasive fashion. Gives more accurate data but may involve write
		transactions to memory.
	 */
	virtual void ProbeLocksDestructive() =0;

	/**
		@brief Checks if a given physical address range has a given protection applied.
	 */
	virtual UncertainBoolean CheckMemoryAccess(uint32_t start, uint32_t end, unsigned int access) =0;

	/**
		@brief Checks if the device is globally read protected or not
	 */
	virtual UncertainBoolean IsDeviceReadLocked() =0;

	/**
		@brief Prints detailed information regarding the state of the read lock
	 */
	virtual void PrintLockProbeDetails() =0;

	/**
		@brief Sets a global read-protection lock on the entire device.

		This function only performs reversible locks that can be cleared with a bulk erase. Thus, it should not
		be able to brick the chip entirely.
	 */
	virtual void SetReadLock() =0;

	/**
		@brief Clears the global read-protection lock, if set in a non-permanent fashion.

		In most parts, this will trigger a bulk flash erase.
	 */
	virtual void ClearReadLock() =0;
};

#endif

