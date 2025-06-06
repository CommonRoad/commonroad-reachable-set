set(CR_REACH_SRC_FILES
        src/data_structure/reach/reach_vertex.cpp
        src/data_structure/reach/reach_polygon.cpp
        src/data_structure/reach/reach_node.cpp
        src/data_structure/reach/reach_line.cpp

        src/data_structure/configuration.cpp
        src/data_structure/segment_tree.cpp
        src/data_structure/reach/reach_set.cpp

        src/utility/lut_longitudinal_enlargement.cpp
        src/utility/enlargement.cpp
        src/utility/collision_checker.cpp
        src/utility/reach_operation.cpp
        src/utility/sweep_line.cpp
)

set(CR_REACH_HDR_FILES
        include/reachset/data_structure/reach/reach_vertex.hpp
        include/reachset/data_structure/reach/reach_polygon.hpp
        include/reachset/data_structure/reach/reach_node.hpp
        include/reachset/data_structure/reach/reach_line.hpp

        include/reachset/data_structure/configuration.hpp
        include/reachset/data_structure/segment_tree.hpp
        include/reachset/data_structure/reach/reach_set.hpp

        include/reachset/utility/lut_longitudinal_enlargement.hpp
        include/reachset/utility/enlargement.hpp
        include/reachset/utility/collision_checker.hpp
        include/reachset/utility/reach_operation.hpp
        include/reachset/utility/sweep_line.hpp
)

add_library(crreach ${CR_REACH_SRC_FILES})

target_sources(crreach PRIVATE ${CR_REACH_HDR_FILES})

if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(crreach
            INTERFACE
            FILE_SET crreach_headers
            TYPE HEADERS
            BASE_DIRS include
            FILES ${CR_REACH_HDR_FILES})
endif ()

set_property(TARGET crreach PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(crreach
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
)

target_link_libraries(crreach
        PRIVATE
        OpenMP::OpenMP_CXX
)

target_link_libraries(crreach
        PUBLIC
        Eigen3::Eigen
        Boost::headers
        Boost::geometry
        yaml-cpp::yaml-cpp
        CommonRoadDC::crcc
        CommonRoadCLCS::crccosy
)

target_precompile_headers(crreach
        PUBLIC
        $<BUILD_INTERFACE:<Eigen/Dense$<ANGLE-R>>
        $<BUILD_INTERFACE:<boost/geometry/geometry.hpp$<ANGLE-R>>
        $<BUILD_INTERFACE:<geometry/curvilinear_coordinate_system.h$<ANGLE-R>>
)

add_library(CommonRoadReach::crreach ALIAS crreach)

if (${PROJECT_NAME}_IS_TOP_LEVEL)
    foreach (target IN ITEMS crreach)
        # this adds many warnings to the build. They usually help to find some bugs
        # TODO: Check whether each warning flag is actually supported by the compiler before adding it
        if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wconversion
                    -pedantic -Wfatal-errors -Wno-unused-parameter)
        endif ()
        if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
            target_compile_options(${target} PRIVATE /W3)
        endif ()
    endforeach ()
endif ()
