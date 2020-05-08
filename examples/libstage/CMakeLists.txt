# export CPLUS_INCLUDE_PATH=/usr/include/python3.5m

set( stestbinarySrcs python_stage.cc)
set_source_files_properties( ${stestbinarySrcs} PROPERTIES COMPILE_FLAGS "${FLTK_CFLAGS}" )
add_executable( stestbinary ${stestbinarySrcs} )


set_target_properties( stestbinary PROPERTIES OUTPUT_NAME stest )
target_link_libraries( stestbinary stage )

IF(PROJECT_OS_LINUX)
  target_link_libraries( stestbinary stage pthread boost_python python3.5m)
ENDIF(PROJECT_OS_LINUX)

INSTALL(TARGETS stestbinary
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${PROJECT_LIB_DIR}
        )



add_library( stagesim MODULE ${stestbinarySrcs} )

target_link_libraries( stagesim
        stage
        pthread boost_python-py35 python3.5m
        )


# delete the "lib" prefix from the plugin
SET_TARGET_PROPERTIES( stagesim PROPERTIES
        PREFIX ""
        LINK_FLAGS "${FLTK_LDFLAGS}"
        )

INSTALL(TARGETS stagesim
        LIBRARY DESTINATION ${PROJECT_LIB_DIR}
        )

configure_file(run.py ${PROJECT_LIB_DIR} COPYONLY)