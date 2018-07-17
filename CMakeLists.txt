cmake_minimum_required(VERSION 3.2)
project(pulseox-firmware)

if(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Cross compiling only. Please use -DCMAKE_TOOLCHAIN_FILE=/PATH/TO/TOOLCHAIN_FILE .")
endif(NOT CMAKE_CROSSCOMPILING)

# Uncomment to build debug firmware.
# set(DEBUG TRUE)

# Libopencm3.
set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)
add_custom_target(libopencm3 make WORKING_DIRECTORY ${LIBOPENCM3_DIR})
link_directories(${LIBOPENCM3_DIR}/lib)
include_directories(${LIBOPENCM3_DIR}/include)

# Target architecture compile flags.
add_definitions(-DSTM32F0)
if(DEBUG)
    add_definitions(-DPULSEOX_DEBUG)
endif(DEBUG)
# add_definitions(-DPULSEOX_BOARD_DIAGNOSTIC)
set(CMAKE_SYSTEM_PROCESSOR "cortex-m0")
set(TARGET_PLATFORM_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb -msoft-float")
set(TARGET_PLATFORM_LIBRARY "opencm3_stm32f0")

# Compile flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall ${TARGET_PLATFORM_FLAGS} -std=c99 -Os -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall --std=c++14 ${TARGET_PLATFORM_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${CMAKE_SOURCE_DIR}/ld/stm32f030f4p6.ld -nostartfiles -l${TARGET_PLATFORM_LIBRARY} -lc -lnosys -specs=rdimon.specs -specs=nano.specs -Wl,--gc-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${TARGET_PLATFORM_FLAGS}")

# Helpers.
function(add_bin_from_elf bin elf)
    add_custom_target(${bin}
            ALL ${CMAKE_OBJCOPY}
            -Obinary ${elf} ${bin} DEPENDS ${elf})
endfunction(add_bin_from_elf)

# Subdirectories.
add_subdirectory(src)

