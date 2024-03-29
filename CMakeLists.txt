cmake_minimum_required(VERSION 3.13)

# configuration command run automatically by cmake-tools extension. 
# /opt/cmake-3.22.0-rc2-linux-x86_64/bin/cmake 
    # --no-warn-unused-cli 
    # -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE 
    # -DCMAKE_BUILD_TYPE:STRING=Debug 
    # -DCMAKE_C_COMPILER:FILEPATH=/opt/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc 
    # -DCMAKE_CXX_COMPILER:FILEPATH=/opt/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-g++ 
    # -S/home/chris/Projects/Embedded/MIDIPresetControllerSW 
    # -B/home/chris/Projects/Embedded/MIDIPresetControllerSW/build 
    # -G Ninja

# this variable is only used by size/objcopy. compiler is set by cmake configuration command.
# set(TOOLCHAIN /opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD    23)
set(CMAKE_CXX_STANDARD  23)

set(BUILD_NAME          build.elf)
add_executable(${BUILD_NAME} "")

# toolchain definitions
include(SW/stm32.cmake)

# Declare the project 
project(MIDIPresetController VERSION 0.1 DESCRIPTION "MIDIPresetController")

message("Configuration is ${CMAKE_BUILD_TYPE}")

# define the configuration so software can use #if add/remove debug libs
target_compile_definitions(${BUILD_NAME} PRIVATE "DEBUG=$<IF:$<CONFIG:Debug>,1,0>")

# build dependencies
target_sources(${BUILD_NAME} PUBLIC
    SW/src/main.cpp
    SW/src/rcc.cpp
    SW/src/midi.cpp
    SW/src/tim.cpp

    SW/src/sys/syscalls.c
    SW/src/sys/system_stm32l0xx.c
    SW/src/sys/startup_stm32l011d4px.s

    # SEGGER RTT used to print output in debug config only
    $<$<CONFIG:Debug,$<TARGET_PROPERTY:COMPILE_DEFINITIONS>>:SW/RTT/SEGGER_RTT_V786g/RTT/SEGGER_RTT.c>
    $<$<CONFIG:Debug,$<TARGET_PROPERTY:COMPILE_DEFINITIONS>>:SW/RTT/SEGGER_RTT_V786g/RTT/SEGGER_RTT_printf.c>
    
)
target_include_directories(${BUILD_NAME} PUBLIC
    SW/inc/CMSIS
    SW/inc
    SW/RTT/SEGGER_RTT_V786g/RTT
    SW/RTT/SEGGER_RTT_V786g/Config
)

# workaround for "build.elf has a LOAD segment with RWX permissions" warning
# See https://community.st.com/s/question/0D53W00001vno1oSAA/warning-elffile-has-a-load-segment-with-rwx-permissions
target_link_options(${BUILD_NAME} PRIVATE "LINKER:--no-warn-rwx-segments")

# display ELF info
# add_custom_command(TARGET ${BUILD_NAME} POST_BUILD COMMAND ${TOOLCHAIN}readelf -l ${BUILD_NAME})
add_custom_command(TARGET ${BUILD_NAME} POST_BUILD COMMAND echo ------------------------------------------------------)
add_custom_command(TARGET ${BUILD_NAME} POST_BUILD COMMAND size ${BUILD_NAME})
add_custom_command(TARGET ${BUILD_NAME} POST_BUILD COMMAND echo ------------------------------------------------------)

# set output binary
add_custom_target(build.bin ALL DEPENDS ${BUILD_NAME} COMMAND objcopy -O ihex ${BUILD_NAME} ${HEX_NAME})
