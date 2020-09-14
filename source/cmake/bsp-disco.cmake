
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

set(BSP_DISCO_DIR ${stm32cubemp1_SOURCE_DIR}/Drivers/BSP/STM32MP15xx_DISCO)

include_directories(
    ${BSP_DISCO_DIR}/
    ${stm32cubemp1_SOURCE_DIR}/Drivers/STM32MP1xx_HAL_Driver/Inc
)

set(BSP_DISCO_SRC
    ${BSP_DISCO_DIR}/stm32mp15xx_disco.c
    ${BSP_DISCO_DIR}/stm32mp15xx_disco_bus.c
    ${BSP_DISCO_DIR}/stm32mp15xx_disco_stpmic1.c
)

set(STM32_DEFINES "${STM32_DEFINES}")

set_source_files_properties(${BSP_DISCO_SRC}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(bsp_disco STATIC ${BSP_DISCO_SRC})

set_target_properties(bsp_disco PROPERTIES LINKER_LANGUAGE C)

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} bsp_disco)