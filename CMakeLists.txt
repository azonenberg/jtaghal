# CMake build script for jtaghal.
# Intended to be integrated into a larger project, not built standalone.

set(PROTOBUF_DIR ${CMAKE_BINARY_DIR}/protobufs)

set(PROTO_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/jtaghal-net.proto
)

add_custom_command(
	OUTPUT ${PROTOBUF_DIR}/jtaghal/jtaghal-net.pb.cc ${PROTOBUF_DIR}/jtaghal/jtaghal-net.pb.h
	COMMAND mkdir -p ${PROTOBUF_DIR}
	COMMAND protoc --proto_path=${CMAKE_SOURCE_DIR}/lib --cpp_out=${PROTOBUF_DIR} ${PROTO_FILES}
	COMMENT "Building protocol buffer"
	DEPENDS ${PROTO_FILES}
)

# disable warnings in generated protobuf code
set_source_files_properties(${PROTOBUF_DIR}/jtaghal/jtaghal-net.pb.cc PROPERTIES COMPILE_FLAGS -Wno-unused-parameter)

include_directories(${PROTOBUF_DIR})

set(JTAGHAL_SOURCES
	jtaghal.cpp

	${PROTOBUF_DIR}/jtaghal/jtaghal-net.pb.cc

	JtagException.cpp

    FirmwareImage.cpp
    ByteArrayFirmwareImage.cpp
    RawBinaryFirmwareImage.cpp
    CPLDBitstream.cpp
    FPGABitstream.cpp

    SerialNumberedDevice.cpp
    LockableDevice.cpp

	AttachedMemoryDevice.cpp
    DebuggableDevice.cpp
    ProgrammableDevice.cpp
    ProgrammableLogicDevice.cpp
    JtagDevice.cpp
    CPLD.cpp
    FPGA.cpp
    JtagDummy.cpp
    JtagFPGA.cpp
    Microcontroller.cpp
    TestableDevice.cpp

    DebuggerInterface.cpp
    GPIOInterface.cpp
    JtagInterface.cpp
    SWDInterface.cpp
    TestInterface.cpp
    DigilentJtagInterface.cpp
    FTDIDriver.cpp
    FTDIJtagInterface.cpp
    FTDISWDInterface.cpp
    GlasgowSWDInterface.cpp
    NetworkedJtagInterface.cpp
    PipeJtagInterface.cpp

    ARMAPBDevice.cpp
    ARMCoreSightDevice.cpp
    ARMFlashPatchBreakpoint.cpp
    ARMv7Processor.cpp
    ARMv7MProcessor.cpp
    ARMv8Processor.cpp
    ARMCortexA57.cpp
    ARMCortexA9.cpp
    ARMCortexM4.cpp
    ARM7TDMISProcessor.cpp
    ARMDebugAccessPort.cpp
    ARMDebugMemAccessPort.cpp
	ARMDebugPort.cpp
    ARMJtagDebugPort.cpp
    ARMDevice.cpp

    FreescaleDevice.cpp
    FreescaleMicrocontroller.cpp
    FreescaleIMXDevice.cpp
    FreescaleIMXSmartDMA.cpp

    MicrochipDevice.cpp
    MicrochipMicrocontroller.cpp
    MicrochipPIC32Device.cpp

    STMicroDevice.cpp
    STMicroMicrocontroller.cpp
    STM32Device.cpp

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

target_link_libraries(jtaghal log xptools ${PROTOBUF_LIBRARIES})
if(USB_LIB)
    target_link_libraries(jtaghal usb-1.0)
endif()
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
