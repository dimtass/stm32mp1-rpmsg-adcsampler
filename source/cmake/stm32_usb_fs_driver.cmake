# In order to use this driver you also need a class driver.
# The class driver is not included in this cmake becaues it
# make sense as it's per project. Therefore, you need to download
# the examples from the following link and get the class driver
# that you need.
# STSW-STM32046: https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32-standard-peripheral-library-expansion/stsw-stm32046.html

set(STM32USBLIB_DIR ${CMAKE_SOURCE_DIR}/libs/STM32_USB_Device_Library)

# Make sure that git submodule is initialized and updated
if (NOT EXISTS "${STM32USBLIB_DIR}")
  message(FATAL_ERROR "STM32_USB_Device_Library submodule not found. Initialize with 'git submodule update --init' in the source directory")
endif()

include_directories(
    ${STM32USBLIB_DIR}/Core/inc
)

set(USTM32USBLIB_SRC
    ${STM32USBLIB_DIR}/Core/src/usbd_core.c
    ${STM32USBLIB_DIR}/Core/src/usbd_ioreq.c
    ${STM32USBLIB_DIR}/Core/src/usbd_req.c
)

set_source_files_properties(${USTM32USBLIB_SRC}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(stm32usblib STATIC ${USTM32USBLIB_SRC})

set_target_properties(stm32usblib PROPERTIES LINKER_LANGUAGE C)
