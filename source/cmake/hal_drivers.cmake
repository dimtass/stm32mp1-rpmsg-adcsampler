
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

set(HAL_DIR ${stm32cubemp1_SOURCE_DIR}/Drivers/STM32MP1xx_HAL_Driver)
set(CMSIS_DIR ${stm32cubemp1_SOURCE_DIR}/Drivers/CMSIS)
set(LINKER_SCRIPTS_DIR ${CMAKE_SOURCE_DIR}/config/LinkerScripts)

message(STATUS "${CMAKE_SOURCE_DIR}/${SRC}/inc/stm32mp1xx_hal_conf.h")

include_directories(
    ${CMAKE_SOURCE_DIR}/${SRC}/inc
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32MP1xx/Include
    ${HAL_DIR}/Inc
)

set(HAL_LIB_SRC
    ${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/system_stm32mp1xx.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_adc.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_adc_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_cec.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_crc.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_crc_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_cryp.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_cryp_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dac.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dac_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dcmi.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dfsdm.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dfsdm_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dma.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_exti.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_fdcan.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_hash.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_hash_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_hsem.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_i2c.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_i2c_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_ipcc.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_lptim.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_mdios.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_mdma.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_msp_template.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_qspi.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_rng.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_rtc.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_rtc_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_sai.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_sai_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_sd.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_sd_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_smbus.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_spdifrx.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_spi.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_spi_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_sram.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_tim.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_timebase_tim_template.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_uart.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_uart_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_usart.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_usart_ex.c
    ${HAL_DIR}/Src/stm32mp1xx_hal_wwdg.c
)

set(STM32_DEFINES "${STM32_DEFINES}")

set_source_files_properties(${HAL_LIB_SRC}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(hal STATIC ${HAL_LIB_SRC})

set_target_properties(hal PROPERTIES LINKER_LANGUAGE C)

# add startup and linker file
set(STARTUP_ASM_FILE "${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/gcc/startup_stm32mp15xx.s")
set_property(SOURCE ${STARTUP_ASM_FILE} PROPERTY LANGUAGE ASM)
set(LINKER_FILE "${CMSIS_DIR}/Device/ST/STM32MP1xx/Source/Templates/gcc/linker/stm32mp15xx_m4.ld")

set(EXTERNAL_EXECUTABLES ${EXTERNAL_EXECUTABLES} ${STARTUP_ASM_FILE})

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} hal)