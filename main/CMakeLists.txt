#set(COMPONENT_SRCS "mcpwm.c")
#set(COMPONENT_SRCS "dcdc.c main.c wifi.c")
#set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(SRCS
        dcdc.cpp
        main.c
        wifi.c
        web.c
        INCLUDE_DIRS "."
        )


#if(CONFIG_EXAMPLE_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../front")
    if(EXISTS ${WEB_SRC_DIR}/dist)
        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
    else()
        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
    endif()
#endif()


# add_definitions(-DMCPWM_CLK_PRESCL) # 15
#add_compile_definitions(camkasdf)
#target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-DESP32asd")

#register_component()