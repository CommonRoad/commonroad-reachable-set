set(CR_REACH_TEST_SRC_FILES
        test_utility.cpp

        data_structure/reach/test_reach_polygon.cpp
        data_structure/reach/test_reach_node.cpp
        data_structure/reach/test_reach_set.cpp
        data_structure/test_segment_tree.cpp

        utility/test_reach_operation.cpp
        utility/test_sweep_line.cpp
        utility/test_collision_checker.cpp
)


# Create executable for test
if (CR_REACH_BUILD_TESTS)
    add_executable(crreach_test ${CR_REACH_TEST_SRC_FILES})

    target_include_directories(crreach_test PRIVATE .)

    set_property(TARGET crreach_test PROPERTY POSITION_INDEPENDENT_CODE ON)

    target_link_libraries(crreach_test
            PRIVATE
            crreach
    )
endif ()
