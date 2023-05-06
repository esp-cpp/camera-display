# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "components/espp/components"
)

add_compile_definitions(BOARD_HAS_PSRAM)

set(
  COMPONENTS
  "main esptool_py esp_psram jpegdec task format monitor display_drivers wifi socket rtsp"
  CACHE STRING
  "List of components to include"
)

project(camera_display)

set(CMAKE_CXX_STANDARD 20)
