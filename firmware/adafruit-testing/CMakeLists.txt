cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(PRJ_NAME "adafruit-bno085-i2c-tests")

if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting CMAKE_BUILD_TYPE=Debug")
  set(CMAKE_BUILD_TYPE Debug)
endif()

include(lib/pico-sdk/pico_sdk_init.cmake)

project(${PRJ_NAME})
pico_sdk_init()

add_executable(${PRJ_NAME})
pico_add_extra_outputs(${PRJ_NAME})

target_link_libraries(${PRJ_NAME}
  pico_stdlib
  hardware_i2c
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(${PRJ_NAME} PRIVATE
    BUILD_DEBUG
  )
endif()

add_subdirectory(src)
add_subdirectory(include)

pico_enable_stdio_usb(${PRJ_NAME} 1)
pico_enable_stdio_uart(${PRJ_NAME} 0)
