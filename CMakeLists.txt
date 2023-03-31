cmake_minimum_required(VERSION 3.0.0)

set(PRJ_NAME SpeedyBeeF405V3)

project(${PRJ_NAME}
  LANGUAGES ASM C CXX
)

set(EXECUTABLE ${PROJECT_NAME}.elf)
#set(EXECUTABLE ${PROJECT_NAME}.hex)
#set(EXECUTABLE ${PROJECT_NAME}.bin)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/src/bsp/ldscript/STM32F405RGTX_FLASH.ld)

# 해당 폴더의 파일만 찾는다.
file(GLOB SRC_FILES CONFIGURE_DEPENDS
  src/*.c
)

# 해당 폴더를 포함해서 하위 폴더까지의 파일도 찾는다.
file(GLOB_RECURSE SRC_FILES_RECURSE CONFIGURE_DEPENDS
  src/*.c
)

# Build the executable based on the source files
add_executable(${EXECUTABLE}  
  ${SRC_FILES}
  ${SRC_FILES_RECURSE}

  src/bsp/startup/startup_stm32f405rgtx.s
  )

target_compile_definitions(${EXECUTABLE} PRIVATE
  -DSTM32F405xx
  )

# List of includ directories
target_include_directories(${EXECUTABLE} PRIVATE
  src
  src/ap
  src/bsp
  src/common
  src/common/config
  src/common/core
  src/common/core/BMI270-Sensor-API
  src/common/core/scheduler
  src/common/core/rx
  src/common/hw/include
  src/hw
  src/hw/driver
  src/lib/Drivers/CMSIS/Include
  src/lib/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  src/lib/Drivers/STM32F4xx_HAL_Driver/Inc
  src/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
  src/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  src/lib/USB_DEVICE/App
  src/lib/USB_DEVICE/Target
  )

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
  -mcpu=cortex-m4
  -mthumb
  
  #-mfpu=fpv4-sp-d16
  #-mfloat-abi=hard

  -fdata-sections
  -ffunction-sections

  -Wall
  -O0
  -g3
  )

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
  -T${LINKER_FILE}
  -mcpu=cortex-m4
  -mthumb
  #-mfpu=fpv4-sp-d16
  #-mfloat-abi=hard
  -u_printf_float
  -specs=nano.specs
  -lc
  -lm
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
  -Xlinker -print-memory-usage -Xlinker
  )