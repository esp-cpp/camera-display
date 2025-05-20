# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
)

# add compile definition ARDUINO_ARCH_ESP32, enabling jpegdec simd support
add_compile_definitions(ARDUINO_ARCH_ESP32)

add_compile_definitions(BOARD_HAS_PSRAM)

set(
  COMPONENTS
  "main esptool_py esp_psram jpegdec task format nvs esp-box t-deck monitor wifi socket rtsp mdns"
  CACHE STRING
  "List of components to include"
)

project(camera_display)

set(CMAKE_CXX_STANDARD 20)
