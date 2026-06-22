# libzdt - ZDT_X42S stepper motor command frame builder library
#
# 双模式 CMake：
#   - 在 ESP-IDF 工程下被作为 component 依赖时，走 idf_component_register 分支
#     （ESP-IDF 组件管理器会先读 idf_component.yml 拉取本仓库，再用本文件构建）
#   - 作为普通 CMake 库（gcc/cmake 直接构建 + 跑单测）时走 add_library 分支
if(IDF_VERSION)
    idf_component_register(
        SRCS "src/libzdt.c"
        INCLUDE_DIRS "src"
    )
else()
    cmake_minimum_required(VERSION 3.10)
    project(libzdt C)

    set(CMAKE_C_STANDARD 99)
    set(CMAKE_C_STANDARD_REQUIRED ON)

    add_library(zdt STATIC src/libzdt.c)
    target_include_directories(zdt PUBLIC src)

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/demo.c")
        add_executable(zdt_demo examples/demo.c)
        target_link_libraries(zdt_demo PRIVATE zdt)
    endif()

    if(BUILD_TESTING)
        enable_testing()
        add_executable(zdt_test tests/test_zdt.c)
        target_link_libraries(zdt_test PRIVATE zdt)
        add_test(NAME zdt_test COMMAND zdt_test)
    endif()
endif()
