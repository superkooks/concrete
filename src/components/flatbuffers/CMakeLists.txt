idf_component_register()

include(ExternalProject)

set(FLATBUFFERS_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/flatbuffers_install)

externalproject_add(flatbuffers_proj
    URL https://github.com/google/flatbuffers/archive/refs/heads/master.zip
    CMAKE_ARGS
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}    
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DFLATBUFFERS_BUILD_TESTS=OFF
    
    USES_TERMINAL_DOWNLOAD TRUE
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE

    INSTALL_DIR ${FLATBUFFERS_INSTALL_DIR}
    BUILD_BYPRODUCTS "${FLATBUFFERS_INSTALL_DIR}/lib/libflatbuffers.a"
)

add_prebuilt_library(flatbuffers_lib "${FLATBUFFERS_INSTALL_DIR}/lib/libflatbuffers.a"
                     PRIV_REQUIRES cxx)
target_include_directories(flatbuffers_lib INTERFACE "${FLATBUFFERS_INSTALL_DIR}/include")
add_dependencies(flatbuffers_lib flatbuffers_proj)

target_link_libraries(${COMPONENT_LIB} INTERFACE flatbuffers_lib)
