idf_component_register(SRCS "main.cpp" "imu.cpp" "env_sens.cpp"
                    INCLUDE_DIRS "" REQUIRES drone_controller esp32_Adafruit_BME280_Library esp32_Adafruit_GPS esp32_BNO08x eigen DShotRMT)

target_compile_options(${COMPONENT_LIB} PRIVATE 
                       "-Wno-gnu-array-member-paren-init" 
                       "-Wno-parentheses-equality"
                       "-Wno-template-in-declaration-name"
)
