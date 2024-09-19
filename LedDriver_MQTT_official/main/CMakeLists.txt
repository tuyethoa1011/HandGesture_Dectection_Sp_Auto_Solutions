idf_component_register(SRCS "led_strip_encoder.c" "app_main.c" 
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
