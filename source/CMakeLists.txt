# Author: Dimitris Tassopoulos <dimtass@gmail.com>

cmake_minimum_required(VERSION 3.12)

project(stm32mp157c-rpmsg-adcsampler LANGUAGES C CXX)

option(USE_HAL_DRIVER "Use HAL library" OFF)
option(USE_LL_DRIVER "Use Low-Level (LL) library" OFF)
option(USE_OPENAMP "Use OpenAMP library" OFF)
option(USE_FREERTOS "Use FreeRTOS" OFF)
option(USE_SEMIHOSTING "Use semi-hosting" OFF)
option(USE_STTERM "Use st-term" OFF)
option(USE_DBGUART "Use debug UART" OFF)
option(USE_GDB "Enable GDB build for debugging" OFF)
option(USE_OVERCLOCKING "Enable overclocking to 128MHz" OFF)
option(USE_TINY_PRINTF "Use tiny printf to minimize firmware size" OFF)


# Set STM32 SoC specific variables
set(STM32_DEFINES " \
    -DIVECT_TAB_OFFSET=0x0 \
    -DUSE_HAL_DRIVER \
    -DCORE_CM4 \
    -DSTM32MP157Cxx \
    -DMETAL_INTERNAL \
    -DMETAL_MAX_DEVICE_REGIONS=2 \
    -DVIRTIO_SLAVE_ONLY \
    -D__LOG_UART_IO_ \
    -DNO_ATOMIC_64_SUPPORT \
    -DRPMSG_BUFFER_SIZE=32768 \
    ")

if (USE_SEMIHOSTING)
    message(STATUS "Using semhosting...")
    set(STM32_DEFINES "${STM32_DEFINES} -DUSE_SEMIHOSTING")
endif()

if (USE_STTERM)
    message(STATUS "Using st-term...")
    set(STM32_DEFINES "${STM32_DEFINES} -DUSE_STTERM")
endif()

if (USE_DBGUART)
    message(STATUS "Using debug UART...")
    set(STM32_DEFINES "${STM32_DEFINES} -DUSE_DBGUART")
endif()

if (USE_GDB)
    set(OPT_LEVEL "0")
else()
    set(OPT_LEVEL "3")
    set(EXTRA_OPT_FLAGS "-ffunction-sections -fdata-sections -fno-builtin")
endif()

if (USE_OVERCLOCKING)
    set(STM32_DEFINES "${STM32_DEFINES} -DUSE_OVERCLOCKING")
endif()

# set compiler optimisations
set(COMPILER_OPTIMISATION "-g -O${OPT_LEVEL}")

# Only build for arm-none-eabi
if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm-none-eabi")
    message(FATAL_ERROR "Invalid CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# CMSIS shared library
# Standard Peripheral Driver shared library
if (USE_HAL_DRIVER)
    include(cmake/hal_drivers.cmake)
endif()

if (USE_LL_DRIVER)
    include(cmake/ll_drivers.cmake)
endif()

if (USE_OPENAMP)
    include(cmake/openamp.cmake)
endif()
# include(cmake/bsp-disco.cmake)

# USB FS Device Driver Library
if (USE_STM32_USB_FS_LIB)
    include(cmake/stm32_usb_fs_driver.cmake)
endif()

# FreeRTOS Library
if (USE_FREERTOS)
    include(cmake/freertos.cmake)
endif()

# include (cmake/noarch_c_lib.cmake)
# include (cmake/stm32f4_dimtass_lib.cmake)

# force stm builds to debug.  This is a hack as when release is set, it automatically 
# sets the invalid -O3 flag on the assembler.
set(CMAKE_BUILD_TYPE Debug)

# Resolve the issue with Eclipse's indexer and C/C++ mixed source files
include(cmake/language_provider.cmake)
remove_from_SYSTEM_MACROS(CXX "__cplusplus;.*;")
create_language_provider("${CMAKE_BINARY_DIR}/.settings" "${GCC_PREFIX}-" "${CXX_STANDARD_FLAG}")

# enable asm for stm startup.s file
enable_language(ASM)

# Custom STM32 options for assembler and gcc
set(STM32_ASM_OPTIONS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
                        -mthumb -mthumb-interwork")

set(STM32_COMPILER_OPTIONS "${STM32_ASM_OPTIONS} \
                        ${EXTRA_OPT_FLAGS} \
                        -Wall -Werror -Wl,-Map=output.map -Wl,--gc-sections -lm -lc ")
# Set project options
SET(CMAKE_C_FLAGS "${STM32_COMPILER_OPTIONS} ${COMPILER_OPTIMISATION} -std=c11" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${STM32_COMPILER_OPTIONS} ${COMPILER_OPTIMISATION} \
                    -std=c++11 -fno-exceptions -fno-unwind-tables -fno-rtti \
                    -mabi=aapcs -fno-unroll-loops -ftree-vectorize "
                    CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${STM32_ASM_OPTIONS}" CACHE INTERNAL "asm compiler flags")

set(EXTRA_LINKER_FLAGS "-lm -lc --specs=nano.specs -u _printf_float")
if (USE_SEMIHOSTING)
    set(EXTRA_LINKER_FLAGS "${EXTRA_LINKER_FLAGS} --specs=nosys.specs --specs=rdimon.specs -lrdimon")
endif()
SET(CMAKE_EXE_LINKER_FLAGS "${STM32_COMPILER_OPTIONS} -Wl,-Map=linker.map -Wl,-cref " CACHE INTERNAL "exe link flags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_LINKER_FLAGS} -T${LINKER_FILE}")

message(STATUS "System Processor      : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS
    "BUILD FLAGS:\n"
    "   STM32_DEFINES   : ${STM32_DEFINES}\n"
    "   c flags         : ${CMAKE_C_FLAGS}\n"
    "   c++ flags       : ${CMAKE_CXX_FLAGS}\n"
    "   asm flags       : ${CMAKE_ASM_FLAGS}\n"
    "   ld flags        : ${CMAKE_EXE_LINKER_FLAGS}\n"
    "   optimizations   : ${COMPILER_OPTIMISATION}\n"
    "   CMSIS           : ${CMSIS_DIR}\n"
    "   HAL Drivers     : ${HAL_DIR}\n"
    "   FreeRTOS        : ${FREERTOS_INC_DIR}\n"
    "   semihosting     : ${SEMIHOSTING_LINKER_FLAGS}\n"
    "   st-term         : ${USE_STTERM}\n"
    "   debug UART      : ${USE_DBGUART}\n"
    "   Use GDB         : ${USE_GDB}\n"
    "   Overclocking    : ${USE_OVERCLOCKING}\n"
)

# add the source code directory
add_subdirectory(${SRC})
