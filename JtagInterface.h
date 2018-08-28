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
	@brief Declaration of JtagInterface
 */

#ifndef JtagInterface_h
#define JtagInterface_h

/**
	@brief Abstract representation of a JTAG adapter.

	A JTAG adapter provides access to a single scan chain containing zero or more JtagDevice objects.

	The JtagInterface class provides three levels of abstraction for scan chain access.

	### Low level (bit level)

	This is the most basic way to access a JTAG adapter - clocking raw bits in and out.

	In order to support a new "dumb" JTAG adapter without any higher level protocol offload, create a new derived class
	and implement each of the following functions:

	\li GetName()
	\li GetSerial()
	\li GetUserID()
	\li GetFrequency()
	\li ShiftData()
	\li ShiftTMS()
	\li SendDummyClocks()

	The low-level interface also includes support for pipelined / queued command execution. This can improve
	performance when using adapters connected to high-latency links such as USB.

	The default implementations simply call the non-deferred versions and execute immediately. If an adapter supports
	queueing of commands, overriding these functions can result in higher performance.

	\li IsSplitScanSupported()
	\li SendDummyClocksDeferred()
	\li ShiftDataWriteOnly()
	\li ShiftDataReadOnly()
	\li Commit()

	### Mid level (state level)

	By default, these functions are simple wrappers around ShiftTMS() for changing between chain states.

	If the adapter supports server-side management of chain state, these can be overridden to simply push a command
	to the adapter.

	\li ResetToIdle()
	\li TestLogicReset()
	\li EnterShiftIR()
	\li LeaveExit1IR()
	\li EnterShiftDR()
	\li LeaveExit1DR()

	### High level (register level)

	These functions provide access to individual registers of TAPs, providing padding as necessary.

	By default these functions are simple wrappers around ShiftData() and the mid-level state functions.

	If the adapter supports server-side padding insertion/removal, these can be overridden to reduce overhead.

	The "deferred" versions of these functions may queue commands. To ensure that all previous queued commands have
	executed, call Commit() or any function which returns readback data from a scan transaction.

	\li SetIR()
	\li SetIRDeferred()
	\li ScanDR()
	\li ScanDRDeferred()
	\li ScanDRSplitRead()
	\li ScanDRSplitWrite()

	### Device management

	These functions provide access to individual devices on the chain.

	\li InitializeChain()
	\li GetDeviceCount()
	\li GetIDCode()
	\li GetDevice()
	\li SwapOutDummy()

	### Performance profiling

	These functions return stats on the amount of data shifted and the time spent waiting for the adapter.

	These may be useful to compare different programming algorithms and optimizations to reduce unnecessary activity.

	\li GetShiftOpCount()
	\li GetRecoverableErrorCount()
	\li GetDataBitCount()
	\li GetModeBitCount()
	\li GetDummyClockCount()
	\li GetShiftTime()

	### NOTES

	Devices are numbered such that TDI goes to device N and TDO goes to device 0.

	\ingroup interfaces
 */
class JtagInterface : public TestInterface
{
public:
	JtagInterface();
	virtual ~JtagInterface();

	//GetInterfaceCount() is a strongly recommended static member function for each derived class

	//Setup stuff
public:
	virtual void Commit();

	//Low-level JTAG interface (wire level)
public:
	/**
		@brief Shifts data through TDI to TDO.

		The LSB of send_data[0] is sent first; the MSB of send_data[0] is followed by the LSB of send_data[1].

		@param last_tms		Different TMS value to use for last bit
		@param send_data	Data to shift into TDI
		@param rcv_data		Data to shift out of TDO (may be NULL)
		@param count		Number of bits to shift
	 */
	virtual void ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count) =0;

	/**
		@brief Sends the requested number of dummy clocks with TMS=0 and flushes the command to the interface.

		Since dummy clocks are often used as a delay element for programming algorithms etc, this function flushes
		the write buffer to ensure immediate execution.

		@throw JtagException may be thrown if the scan operation fails

		@param n			 Number of dummy clocks to send
	 */
	virtual void SendDummyClocks(size_t n) =0;

	/**
		@brief Sends the requested number of dummy clocks with TMS=0 and does not flush the write pipeline.

		@throw JtagException may be thrown if the scan operation fails

		@param n			 Number of dummy clocks to send
	 */
	virtual void SendDummyClocksDeferred(size_t n);

protected:
	/**
		@brief Shifts data into TMS to change TAP state

		This is no longer a public API operation. It can only be accessed via the state-level interface.

		Implementations of this class may choose to implement EITHER this function (and use the default
		JtagInterface-provided state-level functions) OR override this function with a private no-op stub
		and override the state-level functions instead.

		@throw JtagException may be thrown if the scan operation fails

		@param tdi			Constant tdi value (normally 0)
		@param send_data	Data to shift into TMS. Bit ordering is the same as for ShiftData().
		@param count		Number of bits to shift
	 */
	virtual void ShiftTMS(bool tdi, const unsigned char* send_data, size_t count) =0;

	//High-performance pipelined scan interface (wire level)
public:
	virtual bool IsSplitScanSupported();

	/**
		@brief Shifts data through TDI to TDO.

		The LSB of send_data[0] is sent first; the MSB of send_data[0] is followed by the LSB of send_data[1].

		If split (pipelined) scanning is supported by the adapter, this function performs the write half of the shift
		operation only; the read is buffered in the JTAG adapter and no readback is performed until ShiftDataReadOnly()
		is called. This allows several shift operations to occur in sequence without incurring a USB turnaround delay
		or other driver latency overhead for each shift operation.

		If split scanning is not supported this call is equivalent to ShiftData() and ShiftDataReadOnly() is a no-op.

		This function MUST be followed by either another ShiftDataWriteOnly() call, a ShiftTMS() call, or a
		ShiftDataReadOnly() call. There must be exactly one ShiftDataReadOnly() call for each ShiftDataWriteOnly()
		call and they must be in order with the same rcv_data and count values. The result of doing otherwise is
		undefined.

		@return True if the read was deferred, false if not

		@param last_tms		Different TMS value to use for last bit
		@param send_data	Data to shift into TDI
		@param rcv_data		Data to shift out of TDO (may be NULL)
		@param count		Number of bits to shift
	 */
	virtual bool ShiftDataWriteOnly(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, size_t count);

	/**
		@brief Reads data from a ShiftDataWriteOnly() call.

		For more information on split (pipelined) scan operations see ShiftDataWriteOnly().

		@return True if the read was executed, false if a no-op

		@param rcv_data		Data to shift out of TDO (may be NULL)
		@param count		Number of bits to shift
	 */
	virtual bool ShiftDataReadOnly(unsigned char* rcv_data, size_t count);

	//Mid-level JTAG interface (state level)
	virtual void TestLogicReset();
	virtual void EnterShiftIR();
	virtual void LeaveExit1IR();
	virtual void EnterShiftDR();
	virtual void LeaveExit1DR();
	virtual void ResetToIdle();		//TODO: Make this protected as well? Not likely to be needed for anything in well-written code

	//High-level JTAG interface (register level)
	virtual void InitializeChain(bool quiet = false);
	unsigned int GetIDCode(unsigned int device);
	void SetIR(unsigned int device, const unsigned char* data, size_t count);
	void SetIRDeferred(unsigned int device, const unsigned char* data, size_t count);
	void SetIR(unsigned int device, const unsigned char* data, unsigned char* data_out, size_t count);
	void ScanDR(unsigned int device, const unsigned char* send_data, unsigned char* rcv_data, size_t count);
	void ScanDRDeferred(unsigned int device, const unsigned char* send_data, size_t count);
	void ScanDRSplitWrite(unsigned int device, const unsigned char* send_data, unsigned char* rcv_data, size_t count);
	void ScanDRSplitRead(unsigned int device, unsigned char* rcv_data, size_t count);

	JtagDevice* GetJtagDevice(unsigned int device)
	{ return dynamic_cast<JtagDevice*>(GetDevice(device)); }

protected:
	//Helpers for initialization
	void CreateDummyDevices();

public:
	void SwapOutDummy(size_t pos, JtagDevice* realdev);

protected:

	///@brief Total IR length of the chain
	size_t m_irtotal;

	///@brief Array of device ID codes
	std::vector<unsigned int> m_idcodes;

	//Performance profiling

	//Debug helpers
	void PrintChainFaultMessage();

protected:
	///Number of shift operations performed on this interface
	size_t m_perfShiftOps;

	///Number of link errors successfully recovered from
	size_t m_perfRecoverableErrs;

	///Number of data bits shifted
	size_t m_perfDataBits;

	///Number of mode bits shifted
	size_t m_perfModeBits;

	///Number of dummy clocks shifted
	size_t m_perfDummyClocks;

	///Total time spent on shift operations
	double m_perfShiftTime;

public:
	virtual size_t GetShiftOpCount();
	virtual size_t GetRecoverableErrorCount();
	virtual size_t GetDataBitCount();
	virtual size_t GetModeBitCount();
	virtual size_t GetDummyClockCount();

	virtual double GetShiftTime();
};

#endif
