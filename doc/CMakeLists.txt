# Helper functions
macro(path_linux_to_win MsysPath ResultingPath)
  string(REGEX REPLACE "^/([a-zA-Z])/" "\\1:/" ${ResultingPath} "${MsysPath}")
endmacro()
macro(path_win_to_linux MsysPath ResultingPath)
  string(REGEX REPLACE "^([a-zA-Z]):/" "/\\1/" ${ResultingPath} "${MsysPath}")
endmacro()

# enable if doxygen found
find_package(Doxygen 1.9.4)
message(STATUS "Can build doc?                            ${DOXYGEN_FOUND}")
if(DOXYGEN_FOUND)

    set(DOXYGEN_EXAMPLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../example/cpp)
    set(DOXYGEN_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    path_win_to_linux(${CMAKE_CURRENT_SOURCE_DIR} CMAKE_CURRENT_SOURCE_DIR_LINUX)
    path_win_to_linux(${PROJECT_SOURCE_DIR} PROJECT_SOURCE_DIR_LINUX)
    path_win_to_linux(${CMAKE_CURRENT_BINARY_DIR} CMAKE_CURRENT_BINARY_DIR_LINUX)

    # configure doxygen configuration file
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    # doc build only target, target is not in default build, so it must be triggered explicitly
    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        EXCLUDE_FROM_ALL
        VERBATIM)

    # Where docs will be installed.
    # Note: if docs shall be not a part of final release, leave this commented out
    # docs will be only available in [current_dir]/doc_html/index.html
    # install(
    #     DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    #     DESTINATION share/doc/${PROJECT_NAME}
    #     OPTIONAL)
endif()




