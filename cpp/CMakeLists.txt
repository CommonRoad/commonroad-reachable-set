# set include & library paths variables for external commonroad libraries
set(CRDC_INCLUDE
        ${CRDC_DIR}/cpp/collision/include) # drivability checker
set(CRDC_LIBRARY
        ${CRDC_DIR}/commonroad_dc/libcrcc.a CACHE STRING "" FORCE)

set(CRCCOSY_INCLUDE
        ${CRCCOSY_DIR}/cpp) # curvilinear collision checker
set(CRCCOSY_LIBRARY
        ${CRCCOSY_DIR}/commonroad_dc/libcrccosy.a CACHE STRING "" FORCE)

# set a variable to hold all relevant cpp source files of reachable sets
set(REACH_SOURCE
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_polygon.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_node.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_segment.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/configuration.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/segment_tree.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/reach_operation.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/sweep_line.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/miscellaneous.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/collision_checker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_interface.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reach_analysis.cpp

        # ${CMAKE_CURRENT_LIST_DIR}/src/semantic/data_structure/traffic_rule.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/src/semantic/data_structure/position_interval.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/src/semantic/data_structure/region.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/src/semantic/data_structure/semantic_model.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/src/semantic/semantic_reachability_analysis.cpp
        CACHE STRING "" FORCE)

# set a variable to hold all relevant include directories
set(REACH_INCLUDE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CRDC_INCLUDE}
        ${CRCCOSY_INCLUDE}
        ${EIGEN3_INCLUDE_DIR}
        CACHE STRING "" FORCE)
# ====
# 1. create executables which uses the source files set above
add_executable(playground_continuous
        ${CMAKE_CURRENT_LIST_DIR}/src/playground.cpp
        ${REACH_SOURCE})
# 2. include directories
target_include_directories(playground_continuous PRIVATE
        ${REACH_INCLUDE_DIR})
# 3. link to external libraries
target_link_libraries(playground_continuous PRIVATE
        yaml-cpp
        ${CRDC_LIBRARY}
        ${CRCCOSY_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY}
        pybind11::embed)

# ==== unit tests using Doctest
# 1. create target
add_executable(doctest_runner
        ${REACH_SOURCE}
        ${CMAKE_CURRENT_LIST_DIR}/tests/test_utility.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_polygon.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_base_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_node.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_segment_tree.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_reach_operation.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_minkowski.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_sweep_line.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_collision_checker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/common/test_reachable_set_interface.cpp
        ${CMAKE_CURRENT_LIST_DIR}/tests/continuous/test_continuous_reachability_analysis.cpp)
# 2. include directories
target_include_directories(doctest_runner PRIVATE
        ${REACH_INCLUDE_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/tests/)
# 3. link to external libraries
target_link_libraries(doctest_runner
        yaml-cpp
        ${CRDC_LIBRARY}
        ${CRCCOSY_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY})