# Add shared/static libraries
add_library(${PROJECT_NAME}_lib SHARED IMPORTED GLOBAL)
add_library(${PROJECT_NAME}_static STATIC IMPORTED GLOBAL)
add_library(xmatemodel_lib STATIC IMPORTED)

if(WIN32)
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8) #64bit
        set(_ARCH_NAME "64bit")
    elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4) #32bit
        set(_ARCH_NAME "32bit")
    endif()

    set_target_properties(xmatemodel_lib PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/64bit/xMateModel.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/64bit/xMateModeld.lib)
    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_IMPLIB_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}.dll
        IMPORTED_IMPLIB_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}.dll
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        INTERFACE_LINK_LIBRARIES xmatemodel_lib # xMateModel模型库目前只支持Release类型编译
        )
elseif(UNIX)
    # example - CMake option: TARGET_ARCHITECTURE=11.3-aarch64
    if(TARGET_ARCHITECTURE)
        set(ARCH_NAME ${TARGET_ARCHITECTURE})
        set(USE_MODEL_OPT_DIR "")
    else()
        set(ARCH_NAME "x86_64")
        set_target_properties(xmatemodel_lib PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/${ARCH_NAME}/libxMateModel.a)
        if(XCORE_USE_XMATE_MODEL)
            set(USE_MODEL_OPT_DIR "")
            set_target_properties(${PROJECT_NAME}_lib ${PROJECT_NAME}_static PROPERTIES
                INTERFACE_LINK_LIBRARIES xmatemodel_lib)
        else()
            set(USE_MODEL_OPT_DIR "/NoModel")
        endif(XCORE_USE_XMATE_MODEL)
    endif()

    set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/${ARCH_NAME})

    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}${USE_MODEL_OPT_DIR}/lib${PROJECT_NAME}.so.${CMAKE_PROJECT_VERSION}
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.a
        )
endif()

if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_lib)
else(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_static)
endif(XCORE_LINK_SHARED_LIBS)

install(IMPORTED_RUNTIME_ARTIFACTS ${PROJECT_NAME}_lib
    RUNTIME
    DESTINATION ${CMAKE_INSTALL_BINDIR})
