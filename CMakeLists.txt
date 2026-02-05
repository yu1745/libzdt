# 所有源文件（由预处理器宏控制条件编译）
set(LIBZDT_CORE_SRCS
    "src/zdt_can.c"
    "src/zdt_can_twai.c"
)

# 设置驱动实现类型
if(CONFIG_ZDT_ENABLE_TWAI_DRIVER)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        ZDT_CAN_DRIVER_IMPL=ZDT_CAN_DRIVER_TWAI
    )
endif()

idf_component_register(
    SRCS ${LIBZDT_CORE_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES driver
)
