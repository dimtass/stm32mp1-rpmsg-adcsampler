
include(FetchContent)

FetchContent_Declare(STM32CubeMP1
    GIT_REPOSITORY  https://github.com/STMicroelectronics/STM32CubeMP1.git
    GIT_TAG         master
)
FetchContent_GetProperties(stm32cubemp1)
if(NOT stm32cubemp1_POPULATED)
    FetchContent_Populate(stm32cubemp1)
    set(stm32cubemp1_VERSION_STRING "1.2.0")
    message(STATUS "${stm32cubemp1_SOURCE_DIR}")
    message(STATUS "${stm32cubemp1_BINARY_DIR}")
endif()

set(OPENAMP_DIR ${stm32cubemp1_SOURCE_DIR}/Middlewares/Third_Party/OpenAMP)

include_directories(
    ${OPENAMP_DIR}/libmetal/lib/include
    ${OPENAMP_DIR}/open-amp/lib/include
    ${OPENAMP_DIR}/virtual_driver
)

set(OPENAMP_SRC
    ${OPENAMP_DIR}/libmetal/lib/system/generic/condition.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/generic_device.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/generic_init.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/generic_io.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/generic_shmem.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/time.c
    ${OPENAMP_DIR}/libmetal/lib/device.c
    ${OPENAMP_DIR}/libmetal/lib/init.c
    ${OPENAMP_DIR}/libmetal/lib/io.c
    ${OPENAMP_DIR}/libmetal/lib/log.c
    ${OPENAMP_DIR}/libmetal/lib/shmem.c
    ${OPENAMP_DIR}/open-amp/lib/remoteproc/remoteproc_virtio.c
    ${OPENAMP_DIR}/open-amp/lib/rpmsg/rpmsg.c
    ${OPENAMP_DIR}/open-amp/lib/rpmsg/rpmsg_virtio.c
    ${OPENAMP_DIR}/open-amp/lib/virtio/virtio.c
    ${OPENAMP_DIR}/open-amp/lib/virtio/virtqueue.c
    ${OPENAMP_DIR}/virtual_driver/virt_uart.c
    ${OPENAMP_DIR}/libmetal/lib/system/generic/cortexm/sys.c
)

set(STM32_DEFINES "${STM32_DEFINES}")

set_source_files_properties(${OPENAMP_SRC}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(openamp STATIC ${OPENAMP_SRC})

set_target_properties(openamp PROPERTIES LINKER_LANGUAGE C)

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} openamp)