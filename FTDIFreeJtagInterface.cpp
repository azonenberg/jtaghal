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
    @brief Implementation of FTDIFreeJtagInterface
 */

#include "jtaghal.h"

#ifdef HAVE_LIBFTDI

#include <libftdi1/ftdi.h>
#include <libusb-1.0/libusb.h>

using namespace std;

#define FTDI_VID                        0x0403  /* FTDI's USB vendor ID */
#define PID_232H_JTAG                   0x8028  /* Product ID for azonenberg's FT232H based JTAG system */

#define BIT_MODE_RESET                  0x00    /* Reset the MPSSE */
#define BIT_MODE_MPSSE                  0x02    /* MPSSE mode */

enum MPSSE_Commands
{
    MPSSE_TX_BYTES                  = 0x19,
    MPSSE_TX_BITS                   = 0x1b,
    MPSSE_TXRX_BYTES                = 0x39,
    MPSSE_TXRX_BITS                 = 0x3b,
    MPSSE_TX_TMS_BITS               = 0x4b,
    MPSSE_TXRX_TMS_BITS             = 0x6b,
    MPSSE_SET_DATA_LOW              = 0x80,
    MPSSE_GET_DATA_LOW              = 0x81,
    MPSSE_SET_DATA_HIGH             = 0x82,
    MPSSE_GET_DATA_HIGH             = 0x83,
    MPSSE_DISABLE_LOOPBACK          = 0x85,
    MPSSE_SET_CLKDIV                = 0x86,
    MPSSE_FLUSH                     = 0x87,
    MPSSE_DISABLE_DIV5              = 0x8a,
    MPSSE_DISABLE_3PHA              = 0x8d,
    MPSSE_DUMMY_CLOCK_BITS          = 0x8e,
    MPSSE_DUMMY_CLOCK_BYTES         = 0x8f,
    MPSSE_DISABLE_ADAPTIVE_CLK      = 0x97,
    MPSSE_INVALID_COMMAND           = 0xAA,     //Invalid command for resyncing
    MPSSE_INVALID_COMMAND_RESPONSE  = 0xFA
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device enumeration

/**
    @brief Gets the version of the API

    @throw JtagException if the FTD2xx call fails

    @return FTDI driver and API version
 */
string FTDIFreeJtagInterface::GetAPIVersion()
{
    struct ftdi_version_info lver = ftdi_get_library_version();

    char sout[128];
    snprintf(sout, sizeof(sout)-1, "libftdi %d.%d.%d (%s, git %s)", lver.major, lver.minor, lver.micro,
        lver.version_str, lver.snapshot_str);
    return sout;
}

/**
    @brief Gets the number of interfaces on the system (may include non-JTAG-capable devices)

    @throw JtagException if the FTD2xx call fails

    @return Number of interfaces found
 */
int FTDIFreeJtagInterface::GetInterfaceCount()
{
    struct ftdi_context *ctx = ftdi_new();
    if(!ctx)
    {
        throw JtagExceptionWrapper(
            "ftdi_new() failed",
            "");
    }
    //Enable use of azonenberg's custom PID
    struct ftdi_device_list *devlist;
    int ndev_raw = ftdi_usb_find_all(ctx, &devlist, FTDI_VID, PID_232H_JTAG);
    if(ndev_raw < 0)
    {
        ftdi_free(ctx);
        throw JtagExceptionWrapper(
            "ftdi_usb_find_all() failed",
            "");
    }
    ftdi_list_free(&devlist);
    ftdi_free(ctx);

    return ndev_raw;
}

bool FTDIFreeJtagInterface::IsJtagCapable(int index)
{
    struct ftdi_context *ctx = ftdi_new();
    if(!ctx)
    {
        throw JtagExceptionWrapper(
            "ftdi_new() failed",
            "");
    }
    //Enable use of azonenberg's custom PID
    int err = ftdi_usb_open_desc_index(ctx, FTDI_VID, PID_232H_JTAG, nullptr, nullptr, index);
    enum ftdi_chip_type type = ctx->type;
    ftdi_free(ctx);
    if(err < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_usb_open_desc_index() failed",
            "");
    }

    if( (type == TYPE_2232H) || (type == TYPE_4232H) || (type == TYPE_232H) )
        return true;

    return false;
}

/**
    @brief Returns the description of the Nth device

    @throw JtagException if the index is invalid or data could not be read

    @return Serial number string
 */
std::string FTDIFreeJtagInterface::GetSerialNumber(int index)
{
    char serial[16];
    struct ftdi_context *ctx = ftdi_new();
    struct ftdi_device_list *devlist;
    if(!ctx)
    {
        throw JtagExceptionWrapper(
            "ftdi_new() failed",
            "");
    }
    //Enable use of azonenberg's custom PID
    int err = ftdi_usb_find_all(ctx, &devlist, FTDI_VID, PID_232H_JTAG);
    if(err < 0)
    {
        ftdi_free(ctx);
        throw JtagExceptionWrapper(
            "ftdi_usb_find_all() failed",
            "");
    }
    err = ftdi_usb_get_strings(ctx, devlist[index].dev, nullptr, 0, nullptr, 0, serial, sizeof(serial));
    ftdi_list_free(&devlist);
    ftdi_free(ctx);
    if(err < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_usb_get_strings() failed",
            "");
    }

    return serial;
}

/**
    @brief Returns the description of the Nth device

    @throw JtagException if the index is invalid or data could not be read

    @return Description string
 */
std::string FTDIFreeJtagInterface::GetDescription(int index)
{
    char desc[64];
    struct ftdi_context *ctx = ftdi_new();
    struct ftdi_device_list *devlist;
    if(!ctx)
    {
        throw JtagExceptionWrapper(
            "ftdi_new() failed",
            "");
    }
    int err = ftdi_usb_find_all(ctx, &devlist, FTDI_VID, PID_232H_JTAG);
    if(err < 0)
    {
        ftdi_free(ctx);
        throw JtagExceptionWrapper(
            "ftdi_usb_find_all() failed",
            "");
    }
    err = ftdi_usb_get_strings(ctx, devlist[index].dev, nullptr, 0, desc, sizeof(desc), nullptr, 0);
    ftdi_list_free(&devlist);
    ftdi_free(ctx);
    if(err < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_usb_get_strings() failed",
            "");
    }
    return desc;
}

/**
    @brief Returns the default clock frequency of the Nth device

    @throw JtagException if the index is invalid or data could not be read

    @return Clock frequency
 */
int FTDIFreeJtagInterface::GetDefaultFrequency(int /*index*/)
{
    return 10000000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
    @brief Connects to an FTDI JTAG interface

    @throw JtagException if the connection could not be establishes or the serial number is invalid

    @param serial       Serial number of the device to connect to
 */
FTDIFreeJtagInterface::FTDIFreeJtagInterface(const std::string& serial)
{
    struct ftdi_context *ctx = ftdi_new();
    if(!ctx)
    {
        throw JtagExceptionWrapper(
            "ftdi_new() failed",
            "");
    }
    //Enable use of azonenberg's custom PID
    int err = ftdi_usb_open_desc(ctx, FTDI_VID, PID_232H_JTAG, nullptr, serial.c_str());
    if(err < 0)
    {
        ftdi_free(ctx);
        throw JtagExceptionWrapper(
            "ftdi_usb_open_desc() failed",
            "");
    }

    //Get some info
    char desc[64];
    err = ftdi_usb_get_strings2(ctx, libusb_get_device(ctx->usb_dev), nullptr, 0, desc, sizeof(desc), nullptr, 0);
    if(err < 0)
    {
        ftdi_free(ctx);
        throw JtagExceptionWrapper(
            "ftdi_usb_open_desc() failed",
            "");
    }
    m_context = ctx;
    m_serial = serial;
    m_userid = serial;
    m_name = desc;

    //Set clock rate to 10 MHz
    m_freq = 10000000;

    //Do the real init
    SharedCtorInit(ctx->type);
}

/**
    @brief Shared initialization used by all constructors
 */
void FTDIFreeJtagInterface::SharedCtorInit(uint32_t type)
{
    int err;
    ftdi_context *ctx = (ftdi_context*)m_context;

    //Get the chip type and append to the name (see FT_DEVICE enum)
    const char* chiptypes[]=
    {
        "BM",
        "AM",
        "100AX",
        "UNKNOWN",
        "2232C",
        "232R",
        "2232H",
        "4232H",
        "232H",
        "X_SERIES"
    };
    if(type <= static_cast<int>((sizeof(chiptypes) / sizeof(chiptypes[0]))))
        m_name += string(" (") + chiptypes[type] + ")";

    //Reset the adapter and purge buffers
    //TODO: reset device or only the port?
    if((err = ftdi_usb_reset(ctx)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_usb_reset() failed",
            "");
    }
    if((err = ftdi_usb_purge_buffers(ctx)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_usb_purge_buffers() failed",
            "");
    }

    //No need to set interface as with libftdi, we're opening the port directly rather than the device

    //Disable event/error characters
    if((err = ftdi_set_event_char(ctx, 0, 0)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_set_event_char() failed",
            "");
    }
    if((err = ftdi_set_error_char(ctx, 0, 0)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_set_error_char() failed",
            "");
    }

    //Set latency timer
    //Go as low as possible to improve RPC/DMA performance
    if((err = ftdi_set_latency_timer(ctx, 2)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_set_latency_timer() failed",
            "");
    }

    //Set timeouts
    ctx->usb_read_timeout = 1000;
    ctx->usb_write_timeout = 1000;

    //Set USB transfer sizes
    if((ftdi_write_data_set_chunksize(ctx, 4096)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_write_data_set_chunksize() failed",
            "");
    }
    if((ftdi_read_data_set_chunksize(ctx, 1024)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_read_data_set_chunksize() failed",
            "");
    }

    //Reset MPSSE
    if((err = ftdi_set_bitmode(ctx, 0x0, BITMODE_RESET)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_set_bitmode() failed",
            "");
    }

    //Enter bitbang mode
    //Pin modes set through MPSSE commands
    if((err = ftdi_set_bitmode(ctx, 0x0, BITMODE_MPSSE)) < 0)
    {
        throw JtagExceptionWrapper(
            "ftdi_set_bitmode() failed",
            "");
    }
    Commit();

    //Sleep, as per AN129
    usleep(50 * 1000);

    //Send bogus command to synchronize the MPSSE (as per FTDI AN129)
    WriteData(MPSSE_INVALID_COMMAND);

    //Chip should respond with 0xFA then the bad command
    //Read until we get that
    unsigned char dummy_response[2] = {0x00, 0x00};
    while( (dummy_response[0] != MPSSE_INVALID_COMMAND_RESPONSE) || (dummy_response[1] != MPSSE_INVALID_COMMAND) )
    {
        dummy_response[0] = dummy_response[1];
        ReadData(dummy_response+1, 1);
    }

    //Initialize the MPSSE clock divider (see FTDI AN108)
    //TODO: Support stuff other than the -H types
    /*
    switch(ctx->type)
    {
    case TYPE_AM:
    case TYPE_BM:
    case TYPE_R:
        throw JtagExceptionWrapper(
            "The requested chip does not have a MPSSE, JTAG capability unavailable",
            "",
            JtagException::EXCEPTION_TYPE_ADAPTER);
        break;

    case TYPE_2232C:
        throw JtagExceptionWrapper(
            "FT2232C/D support not implemented",
            "",
            JtagException::EXCEPTION_TYPE_UNIMPLEMENTED);
        break;

    //libftdi 0.20 or later is required for 232H support
    case TYPE_232H:
    case TYPE_2232H:
    case TYPE_4232H:
        break;

    default:
        throw JtagExceptionWrapper(
            "Unknown FTDI chip type",
            "",
            JtagException::EXCEPTION_TYPE_UNIMPLEMENTED);
        break;
    }
    */

    //Set clock rate to 10 MHz
    m_freq = 10000000;

    //General setup commands
    unsigned char cmd_setup[]=
    {
        MPSSE_DISABLE_DIV5,         //60 MHz base clock
        MPSSE_DISABLE_3PHA,         //No 3-phase clocking
        MPSSE_DISABLE_ADAPTIVE_CLK, //No adaptive clocking
        MPSSE_SET_CLKDIV,
            0x02, 0x00,             //10 MHz
        MPSSE_DISABLE_LOOPBACK,     //No loopback mode
        MPSSE_FLUSH                 //Flush buffers
    };
    WriteData(cmd_setup, sizeof(cmd_setup));
    Commit();

    //Initialize the GPIO pins
    //GPIOL3 has to be high by default in order to enable outputs in HS1 and usb-jtag-mini boards
    for(int i=0; i<12; i++)
    {
        m_gpioValue.push_back(false);
        m_gpioDirection.push_back(false);
    }
    SetGpioDirectionDeferred(3, true);
    SetGpioValueDeferred(3, true);
    WriteGpioState();

    //Set timeouts
    ctx->usb_read_timeout = 1000;
    ctx->usb_write_timeout = 1000;
}

/**
    @brief Interface destructor

    Closes handles and disconnects from the adapter.
 */
FTDIFreeJtagInterface::~FTDIFreeJtagInterface()
{
    //Disable JTAG write enable and float pins
    SetGpioDirectionDeferred(3, true);
    SetGpioValueDeferred(3, false);
    WriteGpioState();

    //Commit pending operations
    Commit();

    if(m_context != NULL)
    {
        ftdi_free((ftdi_context *)m_context);
        m_context = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions

void FTDIFreeJtagInterface::Commit()
{
    if(!m_writeBuffer.empty())
    {
        WriteDataRaw(&m_writeBuffer[0], m_writeBuffer.size());
        m_writeBuffer.clear();
    }
}

/**
    @brief Writes FTDI MPSSE data to the interface.

    Writes may be deferred until Commit() is called to improve performance.

    @throw JtagException on failure

    @param data             Data to write
    @param bytesToWrite     Number of bytes to write
 */
void FTDIFreeJtagInterface::WriteData(const void* data, size_t bytesToWrite)
{
    const unsigned char* p = reinterpret_cast<const unsigned char*>(data);
    for(size_t i=0; i<bytesToWrite; i++)
        m_writeBuffer.push_back(p[i]);

    //Don't let the buffer get TOO big
    if(m_writeBuffer.size() >= 4096)
        Commit();
}

/**
    @brief Wrapper around FT_Write()

    @throw JtagException on failure

    @param data             Data to write
    @param bytesToWrite     Number of bytes to write
 */
void FTDIFreeJtagInterface::WriteDataRaw(const void* data, size_t bytesToWrite)
{
    int status;
    int bytesWritten;
    size_t bytes_left = bytesToWrite;

    //for some reason the buffer isn't a const... are we certain d2xx won't change it?
    unsigned char* pdata = reinterpret_cast<unsigned char*>(
                                const_cast<void*>(
                                    data
                                    )
                                );

    while(bytes_left != 0)
    {
        if((status = ftdi_write_data((ftdi_context *)m_context, pdata, bytes_left)) < 0)
        {
            throw JtagExceptionWrapper(
                "ftdi_write_date() failed",
                "");
        }
        bytesWritten = status;

        if(bytesWritten > bytes_left)
        {
            throw JtagExceptionWrapper(
                "ftdi_write_date() wrote too much data",
                "");
        }

        bytes_left -= bytesWritten;
        pdata += bytesWritten;
    }
}

/**
    @brief Wrapper around FT_Write()

    @throw JtagException on failure

    @param cmd      The single byte to write
 */
void FTDIFreeJtagInterface::WriteData(unsigned char cmd)
{
    WriteData(&cmd, 1);
}

/**
    @brief Wrapper around FT_Read()

    @throw JtagException on failure

    @param data             Data to write
    @param bytesToRead      Number of bytes to read
 */
void FTDIFreeJtagInterface::ReadData(void* data, size_t bytesToRead)
{
    //Push outstanding writes
    Commit();

    unsigned char* p = (unsigned char*)data;
    int bytesRead;
    size_t bytesTotal = bytesToRead;
    int status;
    int i = 0;
    int j=0;
    while(true)
    {
        j++;

        // XXX is there anything analogous to this?
        // //Get the status of the device.
        // //Apparently we need to call FT_GetStatus() before a second read operation will succeed.
        // //See http://www.alecjacobson.com/weblog/?p=2934
        // DWORD rxsize;
        // DWORD txsize;
        // DWORD evstat;
        // if(FT_OK != (status = FT_GetStatus(m_context, &rxsize, &txsize, &evstat)))
        // {
        //     throw JtagExceptionWrapper(
        //         "FT_GetStatus() failed",
        //         "");
        // }

        // //No data? Wait one USB packet time and try again
        // if(rxsize == 0)
        // {
        //     usleep(125);
        //     if( (j % 2000) == 1999)
        //     {
        //         printf("[FTDIFreeJtagInterface] Read is taking a long time, flushing... (j=%d, i=%d)\n", j, i);
        //         m_perfRecoverableErrs ++;
        //         WriteData(MPSSE_FLUSH);
        //         Commit();
        //     }
        //     continue;
        // }

        //If we get to this point data is ready to read
        if((status = ftdi_read_data((ftdi_context*)m_context, p, bytesToRead)) < 0)
        {
            throw JtagExceptionWrapper(
                "ftdi_read_data() failed",
                "");
        }
        bytesRead = status;

        //Note how many bytes actually got read
        bytesToRead -= bytesRead;
        p += bytesRead;
        if(bytesToRead == 0)
        {
            if(i != 0)
                printf("    Read completed OK\n");
            break;
        }

        //If not fully read, keep trying
        printf("[FTDIFreeJtagInterface] More data to read (iteration %d, %zu read this call, %zu left, %zu total)\n",
            ++i, (size_t)bytesRead, bytesToRead, bytesTotal);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adapter information

/**
    @brief Gets the manufacturer-assigned name for this programming adapter
 */
std::string FTDIFreeJtagInterface::GetName()
{
    return m_name;
}

/**
    @brief Gets the manufacturer-assigned serial number for this programming adapter
 */
std::string FTDIFreeJtagInterface::GetSerial()
{
    return m_serial;
}

/**
    @brief Gets the user-assigned name for this programming adapter
 */
std::string FTDIFreeJtagInterface::GetUserID()
{
    return m_userid;
}

/**
    @brief Gets the clock frequency, in Hz, of the JTAG interface
 */
int FTDIFreeJtagInterface::GetFrequency()
{
    return m_freq;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low-level JTAG interface

void FTDIFreeJtagInterface::ShiftData(bool last_tms, const unsigned char* send_data, unsigned char* rcv_data, int count)
{
    double start = GetTime();

    m_perfShiftOps ++;
    m_perfDataBits += count;

    bool want_read = true;
    if(rcv_data == NULL)
        want_read = false;

    //Purge the output data with zeros (in case we arent receving an integer number of bytes)
    if(want_read)
    {
        int bytecount = ceil(count/8.0f);
        memset(rcv_data, 0, bytecount);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Bulk data transfers
    //4KB at a time
    //Do NOT send the last bit in this loop (for proper handling of last_tms)
    const int BITS_PER_BYTE = 8;
    const int BLOCK_SIZE_KBYTE = 4096;
    const int BLOCK_SIZE_KBIT = BLOCK_SIZE_KBYTE * BITS_PER_BYTE;
    while(count > BLOCK_SIZE_KBIT)
    {
        //Write command header, data block, and flush command
        unsigned char header[3]=
        {
            static_cast<unsigned char>(want_read ? MPSSE_TXRX_BYTES : MPSSE_TX_BYTES),
                                                                    //Clock data out on negative clock edge
            0xFF,                                                   //Length, little endian = 4095 = 0F FF (off by one)
            0x0F
        };
        WriteData(header, 3);
        WriteData(send_data, BLOCK_SIZE_KBYTE);
        WriteData(MPSSE_FLUSH);

        //Read data back
        if(want_read)
            ReadData(rcv_data, BLOCK_SIZE_KBYTE);

        //Bump pointers and mark space as used
        send_data += BLOCK_SIZE_KBYTE;
        if(want_read)
            rcv_data += BLOCK_SIZE_KBYTE;
        count -= BLOCK_SIZE_KBIT;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generate and send the command packet for the rest of the data
    std::vector<unsigned char> cmd;
    GenerateShiftPacket(send_data, count, want_read, last_tms, cmd);
    WriteData(&cmd[0], cmd.size());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Read the data
    if(want_read)
        DoReadback(rcv_data, count);

    m_perfShiftTime += GetTime() - start;
}

bool FTDIFreeJtagInterface::IsSplitScanSupported()
{
    return true;
}

bool FTDIFreeJtagInterface::ShiftDataWriteOnly( bool last_tms,
                                            const unsigned char* send_data,
                                            unsigned char* rcv_data, int count)
{
    //If count is too big, don't pipeline
    if(count >= (8 * 4096))
    {
        ShiftData(last_tms, send_data, rcv_data, count);
        return false;
    }

    //Otherwise, send the write
    std::vector<unsigned char> cmd;
    GenerateShiftPacket(send_data, count, (rcv_data != NULL), last_tms, cmd);
    WriteData(&cmd[0], cmd.size());
    return true;
}

bool FTDIFreeJtagInterface::ShiftDataReadOnly(unsigned char* rcv_data, int count)
{
    if(count >= (8 * 4096))
        return false;

    if(rcv_data != NULL)
        DoReadback(rcv_data, count);
    return true;
}

void FTDIFreeJtagInterface::DoReadback(unsigned char* rcv_data, int count)
{
    int bytes_left = count / 8;
    if( (count & 7) == 0)
        bytes_left --;
    if(bytes_left > 0)
        count -= bytes_left * 8;
    int bl = count - 2;
    int nbit = count-1;

    WriteData(MPSSE_FLUSH);

    //Byte-oriented data
    if(bytes_left > 0)
    {
        ReadData(rcv_data, bytes_left);
        rcv_data += bytes_left;
    }

    //Bit-oriented data
    if(bl >= 0)
    {
        ReadData(rcv_data, 1);

        //Shift so we're right-aligned
        rcv_data[0] >>= (8 - count + 1);
    }

    //Last bit
    unsigned char tmp = 0;
    ReadData(&tmp, 1);
    PokeBit(rcv_data, nbit, (tmp & 0x80) ? true : false);
}

void FTDIFreeJtagInterface::GenerateShiftPacket(
    const unsigned char* send_data, int count,
    bool want_read,
    bool last_tms,
    std::vector<unsigned char>& cmd_out)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Bulk data transfer is done. We now have less than 4KB left, but it might not be an even number of bytes
    //Send until we have <=8 bits left
    //Do *not* send the last bit here (for proper handling of last_tms)
    int bytes_left = count / 8;
    if( (count & 7) == 0)
        bytes_left --;

    //Send the byte-oriented data (subtract 1 from count)
    int bl = bytes_left - 1;
    if(bytes_left > 0)
    {
        cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_BYTES : MPSSE_TX_BYTES));
        cmd_out.push_back(0xFF & bl);
        cmd_out.push_back(bl >> 8);
        for(int i=0; i<bytes_left; i++)
            cmd_out.push_back(send_data[i]);

        //Bump pointers
        send_data += bytes_left;
        count -= bytes_left * 8;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Byte sending is done. We now have <=8 bits left. May or may not be an even number of bytes.
    //Send all but the last bit at this time.

    //Write header and data
    bl = count - 2;                             //Header count is offset by 1, then subtract again to skip the last bit
    if(bl >= 0)
    {
        cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_BITS : MPSSE_TX_BITS));
        cmd_out.push_back(static_cast<unsigned char>(bl));
        cmd_out.push_back(send_data[0]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Bit sending is done. We now have one bit left.
    //Send the last bit as TMS
    int nbit = count-1;
    int send_last = PeekBit(send_data, nbit);
    cmd_out.push_back(static_cast<unsigned char>(want_read ? MPSSE_TXRX_TMS_BITS : MPSSE_TX_TMS_BITS));
                                                //Send data to TMS on falling edge, then read
    cmd_out.push_back(0);                       //Send 1 bit
    cmd_out.push_back(static_cast<unsigned char>((send_last ? 0x80 : 0) | (last_tms ? 1 : 0)));
                                                //Bit 7 is last data bit to send
                                                //Bit 0 is TMS bit
}

void FTDIFreeJtagInterface::ShiftTMS(bool tdi, const unsigned char* send_data, int count)
{
    double start = GetTime();

    m_perfShiftOps ++;
    m_perfModeBits += count;

    if(count > 7)
    {
        throw JtagExceptionWrapper(
            "ShiftTMS() not implemented for count > 7",
            "");
    }

    //Clock data to TMS, LSB first
    unsigned char command[3] =
    {
        MPSSE_TX_TMS_BITS,
        static_cast<unsigned char>(count - 1),
        static_cast<unsigned char>((send_data[0] & 0x7F) | (tdi ? 0x80 : 0))
    };
    WriteData(command, 3);

    m_perfShiftTime += GetTime() - start;
}

void FTDIFreeJtagInterface::SendDummyClocks(int n)
{
    SendDummyClocksDeferred(n);

    double start = GetTime();

    //Dummy clocks are often used as a delay cycle
    //so force the write to complete now
    Commit();

    m_perfShiftTime += GetTime() - start;
}

void FTDIFreeJtagInterface::SendDummyClocksDeferred(int n)
{
    double start = GetTime();

    m_perfShiftOps ++;
    m_perfDummyClocks += n;

    int nbytes = n / 8;
    if(nbytes >= 0xFFFF)
    {
        throw JtagExceptionWrapper(
            "SendDummyClocks() does not implement values > (0xFFFF * 8)",
            "");
    }

    //Bulk dummy clocks (in groups of 8)
    if(nbytes != 0)
    {
        //"This will pulse the clock for 8 to (8 x $10000) times given by length. A length of 0x0000 will do 8 clocks
        //and a length of 0xFFFF will do 524288 clocks"
        nbytes --;
        unsigned char command[3] =
        {
            MPSSE_DUMMY_CLOCK_BYTES,
            static_cast<unsigned char>(nbytes & 0xFF),
            static_cast<unsigned char>((nbytes >> 8) & 0xFF)
        };

        WriteData(command, 3);
    }

    //Finish off to the exact count requested
    //"This will pulse the clock for 1 to 8 times given by length. A length of 0x00 will do 1 clock and a length of
    //0x07 will do 8 clocks."
    int nbits = n & 7;
    nbits --;
    unsigned char command[2]=
    {
        MPSSE_DUMMY_CLOCK_BITS,
        static_cast<unsigned char>(nbits),
    };

    WriteData(command, 2);

    m_perfShiftTime += GetTime() - start;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MPSSE synchronization

/**
    @brief Verifies that we're still in sync with the MPSSE

    @throw JtagException if an FTDI API call fails
 */
void FTDIFreeJtagInterface::SyncCheck()
{
    printf("    Sync check\n");

    //Send bogus command 0xAA to synchronize the MPSSE (as per FTDI AN129)
    WriteData(MPSSE_INVALID_COMMAND);

    //Chip should respond with 0xFA AA
    //Read until we get that
    int n=0;
    unsigned char dummy_response[2] = {0x00, 0x00};
    while( (dummy_response[0] != MPSSE_INVALID_COMMAND_RESPONSE) || (dummy_response[1] != MPSSE_INVALID_COMMAND) )
    {
        dummy_response[0] = dummy_response[1];
        ReadData(dummy_response+1, 1);

        if( (n == 0) && (dummy_response[1] != MPSSE_INVALID_COMMAND_RESPONSE) )
            printf("    SYNC ERROR at position 0 - expected MPSSE_INVALID_COMMAND_RESPONSE (0xFA), got %02x\n",
                dummy_response[1] & 0xFF);
        else
            printf("    Got 0x%02x\n", dummy_response[1] & 0xFF);
        n++;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO stuff

void FTDIFreeJtagInterface::ReadGpioState()
{
    unsigned char cmd[]=
    {
        MPSSE_GET_DATA_LOW,
        MPSSE_GET_DATA_HIGH
    };
    unsigned char buf[2];
    WriteData(cmd, sizeof(cmd));
    ReadData(buf, 2);

    //Unpack
    m_gpioValue[0]  = (buf[0] & 0x10) ? true : false;
    m_gpioValue[1]  = (buf[0] & 0x20) ? true : false;
    m_gpioValue[2]  = (buf[0] & 0x40) ? true : false;
    m_gpioValue[3]  = (buf[0] & 0x80) ? true : false;
    m_gpioValue[4]  = (buf[1] & 0x01) ? true : false;
    m_gpioValue[5]  = (buf[1] & 0x02) ? true : false;
    m_gpioValue[6]  = (buf[1] & 0x04) ? true : false;
    m_gpioValue[7]  = (buf[1] & 0x08) ? true : false;
    m_gpioValue[8]  = (buf[1] & 0x10) ? true : false;
    m_gpioValue[9]  = (buf[1] & 0x20) ? true : false;
    m_gpioValue[10] = (buf[1] & 0x40) ? true : false;
    m_gpioValue[11] = (buf[1] & 0x80) ? true : false;
}

void FTDIFreeJtagInterface::WriteGpioState()
{
    //Pack
    unsigned char value_low =
        (m_gpioValue[0] << 4) |
        (m_gpioValue[1] << 5) |
        (m_gpioValue[2] << 6) |
        (m_gpioValue[3] << 7);
    unsigned char dir_low =
        (m_gpioDirection[0] << 4) |
        (m_gpioDirection[1] << 5) |
        (m_gpioDirection[2] << 6) |
        (m_gpioDirection[3] << 7);
    unsigned char value_hi =
        (m_gpioValue[4]) |
        (m_gpioValue[5] << 1) |
        (m_gpioValue[6] << 2) |
        (m_gpioValue[7] << 3) |
        (m_gpioValue[8] << 4) |
        (m_gpioValue[9] << 5) |
        (m_gpioValue[10] << 6) |
        (m_gpioValue[11] << 7);
    unsigned char dir_hi =
        (m_gpioDirection[4]) |
        (m_gpioDirection[5] << 1) |
        (m_gpioDirection[6] << 2) |
        (m_gpioDirection[7] << 3) |
        (m_gpioDirection[8] << 4) |
        (m_gpioDirection[9] << 5) |
        (m_gpioDirection[10] << 6) |
        (m_gpioDirection[11] << 7);

    //Force low bits for JTAG pins
    //  Bit0 = TCK = output (1)
    //  Bit1 = TDI = output (1)
    //  Bit2 = TDI = input (0)
    //  Bit3 = TMS = output (1)
    //  TMS idles high, rest idle low
    value_low = (value_low & 0xF0) | 0x08;
    dir_low = (dir_low & 0xF0) | 0x0B;

    unsigned char cmd[] =
    {
        MPSSE_SET_DATA_LOW,
        value_low,
        dir_low,
        MPSSE_SET_DATA_HIGH,
        value_hi,
        dir_hi
    };
    WriteData(cmd, sizeof(cmd));
}

#endif  //#ifdef HAVE_FTD2XX
