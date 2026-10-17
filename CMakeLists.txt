idf_component_register(
    SRCS
        "src/zdt_common_motion.c"
        "src/zdt_common_sys.c"
        "src/zdt_common_param.c"
        "src/zdt_x.c"
        "src/zdt_emm.c"
        "src/zdt_twai.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai
    PRIV_REQUIRES esp_timer
)
