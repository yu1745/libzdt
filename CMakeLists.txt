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