# Public headers
set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Here puts required headers
set(SHARED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/data_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/exception.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/motion_control_rt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rokae/utility.h
    )

set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")