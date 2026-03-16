idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "")


target_compile_options(${COMPONENT_LIB} PRIVATE 
                       "-Wno-gnu-array-member-paren-init" 
                       "-Wno-parentheses-equality"
                       "-Wno-template-in-declaration-name"
)
