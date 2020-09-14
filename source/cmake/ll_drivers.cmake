
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

set(LL_DIR ${stm32cubemp1_SOURCE_DIR}/Drivers/STM32MP1xx_HAL_Driver)
set(CMSIS_DIR ${stm32cubemp1_SOURCE_DIR}/Drivers/CMSIS)
set(LINKER_SCRIPTS_DIR ${CMAKE_SOURCE_DIR}/config/LinkerScripts)

include_directories(
    ${CMAKE_SOURCE_DIR}/${SRC}/inc
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32MP1xx/Include
    ${LL_DIR}/Inc
)

set(LL_LIB_SRC
    ${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/system_stm32mp1xx.c
    ${LL_DIR}/Src/stm32mp1xx_ll_adc.c
    ${LL_DIR}/Src/stm32mp1xx_ll_delayblock.c
    ${LL_DIR}/Src/stm32mp1xx_ll_dma.c
    ${LL_DIR}/Src/stm32mp1xx_ll_exti.c
    ${LL_DIR}/Src/stm32mp1xx_ll_fmc.c
    ${LL_DIR}/Src/stm32mp1xx_ll_gpio.c
    ${LL_DIR}/Src/stm32mp1xx_ll_i2c.c
    ${LL_DIR}/Src/stm32mp1xx_ll_lptim.c
    ${LL_DIR}/Src/stm32mp1xx_ll_pwr.c
    ${LL_DIR}/Src/stm32mp1xx_ll_rcc.c
    ${LL_DIR}/Src/stm32mp1xx_ll_rtc.c
    ${LL_DIR}/Src/stm32mp1xx_ll_sdmmc.c
    ${LL_DIR}/Src/stm32mp1xx_ll_spi.c
    ${LL_DIR}/Src/stm32mp1xx_ll_tim.c
    ${LL_DIR}/Src/stm32mp1xx_ll_usart.c
    ${LL_DIR}/Src/stm32mp1xx_ll_utils.c
)

set(STM32_DEFINES "${STM32_DEFINES}")

set_source_files_properties(${LL_LIB_SRC}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(ll STATIC ${LL_LIB_SRC})

set_target_properties(ll PROPERTIES LINKER_LANGUAGE C)

# add startup and linker file
set(STARTUP_ASM_FILE "${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/gcc/startup_stm32mp15xx.s")
set_property(SOURCE ${STARTUP_ASM_FILE} PROPERTY LANGUAGE ASM)
set(LINKER_FILE "${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/gcc/linker/stm32mp15xx_m4.ld")

set(EXTERNAL_EXECUTABLES ${EXTERNAL_EXECUTABLES} ${STARTUP_ASM_FILE})

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ll)