# ESP-IDF 集成测试项目
cmake_minimum_required(VERSION 3.16)

# 添加libzdt组件路径
set(EXTRA_COMPONENT_DIRS "..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_zdt_espidf)
