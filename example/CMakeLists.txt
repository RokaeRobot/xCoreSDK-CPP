#-------------------------------------------------------------------------------
# example executables setup

project(xCoreSDK-examples CXX)

add_compile_definitions(
    # for M_PI in cmath
    $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
    # if linking the shared library, add a macro for MSVC dllimport attribute
    $<$<BOOL:${XCORE_LINK_SHARED_LIBS}>:XCORESDK_DLL>)

add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

# Here puts required example source files
set(EXAMPLES
    sdk_example
    move_example
    path_record
    rl_project
    read_robot_state
    )

set(EXAMPLE_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/print_helper.hpp
)

# threading
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)

foreach(example ${EXAMPLES})
    add_executable(${example} ${example}.cpp ${EXAMPLE_HEADERS})
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR})
    target_link_libraries(${example} Rokae::Rokae Threads::Threads eigen)

endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        xmatemodel_er3_er7p
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${EXAMPLE_HEADERS})
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae Threads::Threads eigen)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

# ------------------------------------------------------------------------------
add_subdirectory(rt)

#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${EXAMPLES} ${EXAMPLES_USE_XMATEMODEL}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    OPTIONAL
    )
