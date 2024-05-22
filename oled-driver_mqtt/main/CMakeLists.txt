idf_component_register(SRCS "app_main.c" "ssd1306.c"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
