# CMake build script for jtaghal.
# Intended to be integrated into a larger project, not built standalone.

set(JTAGHAL_SOURCES
	jtaghal.cpp

	JtagException.cpp

    FirmwareImage.cpp
    ByteArrayFirmwareImage.cpp
    RawBinaryFirmwareImage.cpp
    CPLDBitstream.cpp
    FPGABitstream.cpp

    DebuggableDevice.cpp
    ProgrammableDevice.cpp
    ProgrammableLogicDevice.cpp
    JtagDevice.cpp
    CPLD.cpp
    FPGA.cpp
    JtagFPGA.cpp
    Microcontroller.cpp

    DebuggerInterface.cpp
    GPIOInterface.cpp
    JtagInterface.cpp
    DigilentJtagInterface.cpp
    FTDIJtagInterface.cpp
    NetworkedJtagInterface.cpp
    PipeJtagInterface.cpp

    ARMAPBDevice.cpp
    ARMCoreSightDevice.cpp
    ARMCortexA9.cpp
    ARMDebugAccessPort.cpp
    ARMDebugMemAccessPort.cpp
    ARMDebugPort.cpp
    ARMDevice.cpp

    MicrochipDevice.cpp
    MicrochipMicrocontroller.cpp
    MicrochipPIC32Device.cpp

    XilinxDevice.cpp
    XilinxCPLD.cpp
    XilinxCoolRunnerIIDevice.cpp
    XilinxFPGA.cpp
    Xilinx7SeriesDevice.cpp
    XilinxUltrascaleDevice.cpp
    XilinxSpartan3ADevice.cpp
    XilinxSpartan6Device.cpp
    XilinxFPGABitstream.cpp
    Xilinx3DFPGABitstream.cpp
    XilinxCPLDBitstream.cpp
	)

add_library(jtaghal SHARED
	${JTAGHAL_SOURCES})
target_link_libraries(jtaghal log)
target_link_libraries(jtaghal xptools)
if(FTD2XX_LIB)
    target_link_libraries(jtaghal ftd2xx)
endif()
if(DJTG_LIB)
    target_link_libraries(jtaghal djtg)
endif()
set_property(TARGET jtaghal PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET log PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET xptools PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(jtaghal
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS jtaghal LIBRARY DESTINATION /usr/lib)
