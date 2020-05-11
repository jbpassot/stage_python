function(python_setup)
  # Instead of calling find_package(PythonLibs PYTHON_VERSION REQUIRED) (e.g. find_package(PythonLibs 2.7 REQUIRED) find_package(PythonInterp 2.7 REQUIRED))
  # and polluting env with many variables and possibly breaking py2/py3 build,
  # we define variables for different py versions manually to have full control on what is used
  # Notice, that find_package(pybind11 2.2 REQUIRED) also calls find_package(PythonLibs) so it is avoided as well
  set(PYTHON27_INCLUDE_DIRS "/usr/include/python2.7")
  set(PYTHON27_INCLUDE_DIRS ${PYTHON27_INCLUDE_DIRS} "/usr/local/include/python2.7")
  set(PYTHON27_INCLUDE_DIRS ${PYTHON27_INCLUDE_DIRS} PARENT_SCOPE)
  set(PYTHON27_MODULE_PREFIX "" PARENT_SCOPE)
  set(PYTHON27_MODULE_EXTENSION ".so" PARENT_SCOPE)
  set(PYTHON27_INSTALL_DESTINATION "/usr/local/lib/python2.7/dist-packages" PARENT_SCOPE)
  set(PYTHON36_INCLUDE_DIRS "/home/jb/projects/shining_software/build/rootfs.shining.generic/usr/include/python3.6m")
  set(PYTHON36_INCLUDE_DIRS ${PYTHON36_INCLUDE_DIRS} "/home/jb/projects/shining_software/build/rootfs.shining.generic/usr/include/x86_64-linux-gnu/python3.6m")
  set(PYTHON36_INCLUDE_DIRS ${PYTHON36_INCLUDE_DIRS} "/home/jb/.local/lib/python3.6/site-packages/pybind11/include" )
  set(PYTHON36_INCLUDE_DIRS ${PYTHON36_INCLUDE_DIRS} PARENT_SCOPE)
  set(PYTHON36_MODULE_PREFIX "" PARENT_SCOPE)
  set(PYTHON36_MODULE_EXTENSION ".so" PARENT_SCOPE)
  set(PYTHON36_INSTALL_DESTINATION "/usr/lib/python3.6/site-packages" PARENT_SCOPE)
endfunction()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set( stage_srcs pybind_stage.cc)
set_source_files_properties( ${stage_srcs} PROPERTIES COMPILE_FLAGS "${FLTK_CFLAGS}" )
python_setup()
add_library( stagesim SHARED ${stage_srcs} )
add_library( stagesim_py3 SHARED ${stage_srcs})

target_include_directories(
        stagesim
        PRIVATE
        ${PYTHON27_INCLUDE_DIRS}
)

target_include_directories(
        stagesim_py3
        PRIVATE
        ${PYTHON36_INCLUDE_DIRS}
)

target_link_libraries( stagesim
        stage
        pthread
        ${PYTHON27_LIBRARIES}
)

target_link_libraries(stagesim_py3
        stage
        pthread
        ${PYTHON36_LIBRARIES}
)

# delete the "lib" prefix from the plugin
SET_TARGET_PROPERTIES( stagesim PROPERTIES
        LINK_FLAGS "${FLTK_LDFLAGS}"
        OUTPUT_NAME "stagesim"
        SUFFIX "${PYTHON27_MODULE_EXTENSION}"
        PREFIX "${PYTHON27_MODULE_PREFIX}")

# delete the "lib" prefix from the plugin
SET_TARGET_PROPERTIES( stagesim_py3 PROPERTIES
        LINK_FLAGS "${FLTK_LDFLAGS}"
        OUTPUT_NAME "stagesim_py3"
        SUFFIX "${PYTHON36_MODULE_EXTENSION}"
        PREFIX "${PYTHON36_MODULE_PREFIX}")

INSTALL(TARGETS stagesim
        LIBRARY DESTINATION ${PROJECT_LIB_DIR}
        )

INSTALL(TARGETS stagesim_py3
        LIBRARY DESTINATION ${PROJECT_LIB_DIR}
        )

