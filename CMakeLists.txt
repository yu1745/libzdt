cmake_minimum_required(VERSION 3.16)

# 所有源文件（由预处理器宏控制条件编译）
set(LIBZDT_CORE_SRCS
    "src/libzdt.c"
    "src/zdt_twai.c"
)

idf_component_register(
    SRCS ${LIBZDT_CORE_SRCS}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_driver_twai
)
