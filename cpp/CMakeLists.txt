# set include & library paths variables for external commonroad libraries
set(CRDC_INCLUDE
        ${CRDC_DIR}/cpp/collision/include # drivability checker
        ${CRDC_DIR}/cpp/geometry/include) # curvilinear collision checker

set(CRDC_LIBRARY
        ${CRDC_DIR}/commonroad_dc/libcrcc.a # drivability checker
        ${CRDC_DIR}/commonroad_dc/libcrccosy.a # curvilinear collision checker
        ${CRDC_DIR}/commonroad_dc/libtriangle.a # curvilinear collision checker
        CACHE STRING "" FORCE)

# set a variable to hold all relevant include directories
set(REACH_INCLUDE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CRDC_INCLUDE}
        ${EIGEN3_INCLUDE_DIR}
        CACHE STRING "" FORCE)

# set a variable to hold all relevant cpp source files of reachable sets
set(REACH_SOURCE
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_vertex.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_polygon.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_node.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_line.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/configuration.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/segment_tree.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/lut_longitudinal_enlargement.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/enlargement.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/collision_checker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/reach_operation.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/sweep_line.cpp
        CACHE STRING "" FORCE)

# ========= building playground
# 1. create executables which uses the source files set above
add_executable(example
        ${CMAKE_CURRENT_LIST_DIR}/src/example.cpp
        ${REACH_SOURCE})
# 2. include directories
target_include_directories(example PRIVATE
        ${REACH_INCLUDE_DIR})
# 3. link to external libraries
target_link_libraries(example PRIVATE
        ${CRDC_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY}
        yaml-cpp
        pybind11::embed)

# ========= unit tests using Doctest
if(ADD_TESTS)
    # 1. create target
    add_executable(doctest_runner
            ${REACH_SOURCE}
            ${CMAKE_CURRENT_LIST_DIR}/tests/test_utility.cpp

            ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/reach/test_reach_polygon.cpp
            ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/reach/test_reach_node.cpp
            ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/reach/test_reach_set.cpp
            ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_segment_tree.cpp

            ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_reach_operation.cpp
            ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_sweep_line.cpp
            ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_collision_checker.cpp
            )
    # 2. include directories
    target_include_directories(doctest_runner PRIVATE
            ${REACH_INCLUDE_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/tests/)
    # 3. link to external libraries
    target_link_libraries(doctest_runner
            yaml-cpp
            ${CRDC_LIBRARY}
            fcl
            ccd
            ${S11N_LIBRARY})
endif()
