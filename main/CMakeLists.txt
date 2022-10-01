#set(COMPONENT_SRCS "mcpwm.c")
#set(COMPONENT_SRCS "dcdc.c main.c wifi.c")
#set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS
        dcdc.cpp
        main.c
#        wifi.c
        INCLUDE_DIRS "."
        )


# add_definitions(-DMCPWM_CLK_PRESCL) # 15
#add_compile_definitions(camkasdf)
#target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-DESP32asd")

#register_component()