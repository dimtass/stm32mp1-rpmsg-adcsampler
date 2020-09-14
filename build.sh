#!/bin/bash -e

echo "Building the project in Linux environment"

# Toolchain path
: ${TOOLCHAIN_DIR:="/opt/toolchains/gcc-arm-none-eabi-9-2019-q4-major"}
# select cmake toolchain
: ${CMAKE_TOOLCHAIN:=TOOLCHAIN_arm_none_eabi_cortex_m4.cmake}
# select to clean previous builds
: ${CLEANBUILD:=false}
# select to create eclipse project files
: ${ECLIPSE_IDE:=false}
# Select HAL lib use
: ${USE_HAL_DRIVER:="ON"}
# Select LL lib use
: ${USE_LL_DRIVER:="OFF"}
# Select OpenAMP lib use
: ${USE_OPENAMP:="ON"}
# Select FreeRTOS lib use
: ${USE_FREERTOS:="OFF"}
# Enable semi-hosting
: ${USE_SEMIHOSTING:="OFF"}
# Enable st-term
: ${USE_STTERM:="OFF"}
# Enable debug UART
: ${USE_DBGUART:="OFF"}
# Enable GDB build
: ${USE_GDB:="OFF"}
# Enable overclock?
: ${USE_OVERCLOCKING:="OFF"}
# Enable tiny printf
: ${USE_TINY_PRINTF:=OFF}
# Select source folder. Give a false one to trigger an error
: ${SRC:="src_hal"}

# Set default arch to stm32
ARCHITECTURE=stm32
# default generator
IDE_GENERATOR="Unix Makefiles"
# Current working directory
WORKING_DIR=$(pwd)
# cmake scripts folder
SCRIPTS_CMAKE="${WORKING_DIR}/source/cmake"
# Compile objects in parallel, the -jN flag in make
PARALLEL=$(nproc)

if [ ! -d "source/${SRC}" ]; then
    echo -e "You need to specify the SRC parameter to point to the source code"
    exit 1
fi

if [ "${ECLIPSE}" == "true" ]; then
	IDE_GENERATOR="Eclipse CDT4 - Unix Makefiles" 
fi

BUILD_ARCH_DIR=${WORKING_DIR}/build-${ARCHITECTURE}

if [ "${ARCHITECTURE}" == "stm32" ]; then
    CMAKE_FLAGS="${CMAKE_FLAGS} \
                -DTOOLCHAIN_DIR=${TOOLCHAIN_DIR} \
                -DCMAKE_TOOLCHAIN_FILE=${SCRIPTS_CMAKE}/${CMAKE_TOOLCHAIN} \
                -DUSE_HAL_DRIVER=${USE_HAL_DRIVER} \
                -DUSE_LL_DRIVER=${USE_LL_DRIVER} \
                -DUSE_OPENAMP=${USE_OPENAMP} \
                -DUSE_STM32_USB_FS_LIB=${USE_STM32_USB_FS_LIB} \
                -DUSE_FREERTOS=${USE_FREERTOS} \
                -DUSE_SEMIHOSTING=${USE_SEMIHOSTING} \
                -DUSE_STTERM=${USE_STTERM} \
                -DUSE_DBGUART=${USE_DBGUART} \
                -DUSE_GDB=${USE_GDB} \
                -DUSE_OVERCLOCKING=${USE_OVERCLOCKING} \
                -DUSE_TINY_PRINTF=${USE_TINY_PRINTF} \
                -DSRC=${SRC} \
                "
else
    >&2 echo "*** Error: Architecture '${ARCHITECTURE}' unknown."
    exit 1
fi

if [ "${CLEANBUILD}" == "true" ]; then
    echo "- removing build directory: ${BUILD_ARCH_DIR}"
    rm -rf ${BUILD_ARCH_DIR}
fi

echo "--- Pre-cmake ---"
echo "architecture      : ${ARCHITECTURE}"
echo "distclean         : ${CLEANBUILD}"
echo "parallel          : ${PARALLEL}"
echo "cmake flags       : ${CMAKE_FLAGS}"
echo "cmake scripts     : ${SCRIPTS_CMAKE}"
echo "IDE generator     : ${IDE_GENERATOR}"
echo "Threads           : ${PARALLEL}"
echo "Semihosting       : ${USE_SEMIHOSTING}"
echo "st-term           : ${USE_STTERM}"
echo "Debug UART        : ${USE_DBGUART}"

mkdir -p build-stm32
cd build-stm32

# setup cmake
cmake ../source -G"${IDE_GENERATOR}" ${CMAKE_FLAGS}

# build
# make -j${PARALLEL} --no-print-directory
make
