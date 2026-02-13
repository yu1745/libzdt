cmake_minimum_required(VERSION 3.16)

# 添加 libzdt 组件的搜索路径 (上级目录)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/..")

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(libzdt_test)
