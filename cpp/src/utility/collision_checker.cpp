#include "reachset/utility/collision_checker.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;
using namespace collision;
using namespace geometry;
using CurvilinearCoordinateSystemPtr = std::shared_ptr<geometry::CurvilinearCoordinateSystem>;

BufferConfig::BufferConfig(double const& buffer_distance) :
        buffer_distance(buffer_distance),
        distance_strategy(bg::strategy::buffer::distance_symmetric<double>(buffer_distance)),
        join_strategy(bg::strategy::buffer::join_round(points_per_circle)),
        end_strategy(bg::strategy::buffer::end_round(points_per_circle)),
        circle_strategy(bg::strategy::buffer::point_circle(points_per_circle)) {
}

CollisionCheckerPtr reach::create_cartesian_collision_checker(
        vector<Polyline> const& vec_polylines_static,
        map<int, vector<Polyline>> const& map_step_to_vec_polylines_dynamic,
        double const& radius_disc_vehicle,
        int const& num_omp_threads) {

    // set up buffer config for inflation (Minkwoski sum)
    auto buffer_config = reach::BufferConfig(radius_disc_vehicle);

    // shape group for all static obstacles
    auto shape_group_static = make_shared<ShapeGroup>();
    // TVO for all dynamic obstacles: at each step, the TVO contains a shape group of all dynamic AABBs
    auto tvo_dynamic = make_shared<TimeVariantCollisionObject>(map_step_to_vec_polylines_dynamic.cbegin()->first);


    // 1. process static obstacles - inflate each polyline and convert to collision Polygon
    auto vec_polygons_CART_static = create_cartesian_polygons_from_polylines(
            vec_polylines_static, num_omp_threads, buffer_config);

    // add the static polygons to a static shape group
    for (auto const& polygon: vec_polygons_CART_static) {
        shape_group_static->addToGroup(polygon);
    }

    // 2. process dynamic obstacles - create a shape group for each time step and add inflated Polygons (from polylines) into it
    for (auto const&[step, vec_polylines_dynamic]: map_step_to_vec_polylines_dynamic) {
        auto vec_polygons_CART_dynamic = create_cartesian_polygons_from_polylines(
                vec_polylines_dynamic, num_omp_threads, buffer_config);

        auto shape_group = make_shared<ShapeGroup>();
        for (auto const& polygon: vec_polygons_CART_dynamic) {
            shape_group->addToGroup(polygon);
        }
        tvo_dynamic->appendObstacle(shape_group);
    }

    // create the collision checker
    auto collision_checker = make_shared<collision::CollisionChecker>();
    collision_checker->addCollisionObject(shape_group_static);
    collision_checker->addCollisionObject(tvo_dynamic);

    return collision_checker;
}


CollisionCheckerPtr reach::create_curvilinear_collision_checker(
        vector<Polyline> const& vec_polylines_static,
        map<int, vector<Polyline>> const& map_step_to_vec_polylines_dynamic,
        CurvilinearCoordinateSystemPtr const& CLCS,
        double const& radius_disc_vehicle,
        int const& num_omp_threads,
        bool const& rasterize_obstacles,
        bool const& rasterize_exclude_static) {

    // set up buffer config for inflation (Minkwoski sum)
    auto buffer_config = reach::BufferConfig(radius_disc_vehicle);

    // shape group for all static obstacles
    auto shape_group_static = make_shared<ShapeGroup>();
    // TVO for all dynamic obstacles: at each step, the TVO contains a shape group of all dynamic AABBs
    auto tvo_dynamic = make_shared<TimeVariantCollisionObject>(map_step_to_vec_polylines_dynamic.cbegin()->first);

    if (not rasterize_obstacles) {
        // no rasterization: obstacles are overapproximated by one AABB after conversion to CVLN

        // 1. process static obstacles - convert each polyline into an aabb
        auto vec_aabb_CVLN_static = create_curvilinear_aabbs_from_cartesian_polylines(
                vec_polylines_static, CLCS, num_omp_threads, buffer_config);

        // add the static aabbs to a static shape group
        for (auto const& aabb: vec_aabb_CVLN_static) {
            shape_group_static->addToGroup(aabb);
        }

        // 2. process dynamic obstacles - create a shape group for each time step and add aabbs (from polylines) into it
        for (auto const&[step, vec_polylines_dynamic]: map_step_to_vec_polylines_dynamic) {
            auto vec_aabb_CVLN_dynamic = create_curvilinear_aabbs_from_cartesian_polylines(
                    vec_polylines_dynamic, CLCS, num_omp_threads, buffer_config);

            auto shape_group = make_shared<ShapeGroup>();
            for (auto const& aabb: vec_aabb_CVLN_dynamic) {
                shape_group->addToGroup(aabb);
            }
            tvo_dynamic->appendObstacle(shape_group);
        }
    } else {
        // rasterization: obstacles are rasterized in CVLN with multiple AABBs -> less overapproximation
        // rasterization of static obstacles can be excluded by setting rasterize_exclude_static=True

        // process static and dynamic obstacles - convert, rasterize and return rasterized AABBs (static and dynamic)
        auto const&[vec_aabb_CVLN_static, map_step_to_vec_aabb_CVLN_dynamic] =
                create_curvilinear_aabbs_from_cartesian_polylines_rasterized(
                    vec_polylines_static, map_step_to_vec_polylines_dynamic, CLCS, num_omp_threads, buffer_config,
                    rasterize_exclude_static);

        // add the static AABBs to a static shape group
        for (auto const& aabb: vec_aabb_CVLN_static) {
            shape_group_static->addToGroup(aabb);
        }

        // add dynamic AABBs to dynamic shape group
        for (auto const&[step, vec_aabb_CVLN_dynamic]: map_step_to_vec_aabb_CVLN_dynamic) {
            auto shape_group = make_shared<ShapeGroup>();
            for (auto const& aabb: vec_aabb_CVLN_dynamic) {
                shape_group->addToGroup(aabb);
            }
            tvo_dynamic->appendObstacle(shape_group);
        }
    }

    // create the collision checker
    auto collision_checker = make_shared<collision::CollisionChecker>();
    collision_checker->addCollisionObject(shape_group_static);
    collision_checker->addCollisionObject(tvo_dynamic);

    return collision_checker;
}

vector<collision::PolygonPtr> reach::create_cartesian_polygons_from_polylines(
        vector<Polyline> const& vec_polylines,
        int const& num_threads,
        BufferConfig const& buffer_config) {

    // initialize output vector of collision Polygons
    vector<PolygonPtr> vec_polygons;
    vec_polygons.reserve(vec_polylines.size());

#pragma omp parallel num_threads(num_threads) default(none) shared(vec_polylines, buffer_config, vec_polygons)
    {
        vector<PolygonPtr> vec_polygons_thread;
        vec_polygons_thread.reserve(vec_polylines.size());

#pragma omp for nowait
        for (auto const& polyline: vec_polylines) {
            auto polygon_geometry = convert_polyline_to_geometry_polygon(polyline);
            auto polygon_inflated = inflate_polygon(polygon_geometry, buffer_config);
            auto polyline_inflated = convert_geometry_polygon_to_polyline(polygon_inflated);

            if (polyline_inflated.size() >= 2) {
                vec_polygons_thread.emplace_back(create_polygon_from_polyline(polyline_inflated));
            }
        }

#pragma omp critical
        vec_polygons.insert(vec_polygons.end(),
                            std::make_move_iterator(vec_polygons_thread.begin()),
                            std::make_move_iterator(vec_polygons_thread.end()));

    }
    return vec_polygons;
}


PolygonPtr reach::create_polygon_from_polyline(Polyline const& polyline) {
    std::vector<Eigen::Vector2d> outer_vertices;
    std::vector<std::vector<Eigen::Vector2d>> hole_vertices;
    // std::vector<collision::TriangleConstPtr> mesh_triangles;

    for (auto const& vertex: polyline) {
        outer_vertices.push_back(Eigen::Vector2d(vertex[0], vertex[1]));
    }

    return std::make_shared<Polygon>(outer_vertices, hole_vertices, 0, collision::triangulation::TriangulationQuality());
}

tuple<vector<RectangleAABBPtr>, map<int, vector<RectangleAABBPtr>>>
    reach::create_curvilinear_aabbs_from_cartesian_polylines_rasterized(
        vector<Polyline> const& vec_polylines_static,
        map<int, vector<Polyline>> const& map_step_to_vec_polylines_dynamic,
        CurvilinearCoordinateSystemPtr const& CLCS,
        int const& num_threads,
        BufferConfig const& buffer_config,
        bool const& rasterize_exclude_static) {

    // boost geometry polygon for projection domain for pre-filtering
    auto proj_domain_polyline = CLCS->projectionDomainBorder();
    auto proj_domain_geometry_polygon = convert_polyline_to_geometry_polygon(proj_domain_polyline);

    // create output vector / map
    vector<RectangleAABBPtr> vec_aabbs_static;
    map<int, vector<RectangleAABBPtr>> map_step_to_vec_aabbs_dynamic;

    // inputs/outputs for rasterization function
    std::vector<Polyline> polylines_list_out;
    std::vector<int> polygon_groups_out;
    int group_count = 0;
    std::vector<std::vector<EigenPolyline>> transformed_polygon;
    std::vector<std::vector<EigenPolyline>> transformed_polygon_rasterized;

    // static obstacle polylines
    if (rasterize_exclude_static) {
        // do not rasterize static obstacles, only convert to CVLN AABBs
        auto vec_aabbs_CVLN_static = create_curvilinear_aabbs_from_cartesian_polylines(
                vec_polylines_static, CLCS, num_threads, buffer_config);

        // add the static aabbs to static output vector
        for (auto aabb: vec_aabbs_CVLN_static) {
            vec_aabbs_static.emplace_back(aabb);
        }

        // hack: create fake empty polyline for static input
        Polyline empty_polyline_static;
        polylines_list_out.emplace_back(empty_polyline_static);
        polygon_groups_out.push_back(0);
    } else{
        // rasterize static obstacles after conversion to CVLN
        for (auto const& polyline: vec_polylines_static) {
            auto polygon_geometry = convert_polyline_to_geometry_polygon(polyline);
            if (bg::intersects(polygon_geometry, proj_domain_geometry_polygon)) {
                auto polygon_inflated = inflate_polygon(polygon_geometry, buffer_config);
                auto polyline_inflated = convert_geometry_polygon_to_polyline(polygon_inflated);

                // add to polylines list and group
                polylines_list_out.emplace_back(polyline_inflated);
                polygon_groups_out.push_back(0);
            }
        }
    }

    // dynamic obstacle polylines
    group_count++;
    for (auto const&[step, vec_polylines_dynamic]: map_step_to_vec_polylines_dynamic) {
        std::vector<Polyline> polylines_list_out_dynamic;
        std::vector<int> polygon_groups_out_dynamic;

        for (auto const& polyline: vec_polylines_dynamic){
            auto polygon_geometry = convert_polyline_to_geometry_polygon(polyline);
            if (bg::intersects(polygon_geometry, proj_domain_geometry_polygon)) {
                auto polygon_inflated = inflate_polygon(polygon_geometry, buffer_config);
                auto polyline_inflated = convert_geometry_polygon_to_polyline(polygon_inflated);
                // add to dynamic polylines list and group
                polylines_list_out_dynamic.emplace_back(polyline_inflated);
                polygon_groups_out_dynamic.push_back(group_count);
            }
        }

        if (not polygon_groups_out_dynamic.empty()) {
            polylines_list_out.insert(polylines_list_out.end(), polylines_list_out_dynamic.begin(),
                polylines_list_out_dynamic.end());
            polygon_groups_out.insert(polygon_groups_out.end(), polygon_groups_out_dynamic.begin(),
                polygon_groups_out_dynamic.end());
            group_count++;
        }
        else {
        Polyline empty_polyline;
        polylines_list_out.push_back(empty_polyline);
        polygon_groups_out.push_back(group_count);
        group_count++;
        }
    }

    // convert to CVLN and rasterize
    CLCS->convertListOfPolygonsToCurvilinearCoordsAndRasterize(
        polylines_list_out, polygon_groups_out, group_count+1, num_threads, transformed_polygon,
        transformed_polygon_rasterized);

    // add rasterized static AABBs to output (if static should be rasterized)
    if (not rasterize_exclude_static){
        vec_aabbs_static.reserve(transformed_polygon_rasterized[0].size());
        for (auto polyline_CVLN: transformed_polygon_rasterized[0]) {
            vec_aabbs_static.emplace_back(create_aabb_from_polyline(polyline_CVLN));
        }
    }

    // add rasterized dynamic AABBs to output
    for (int i = 1; i<transformed_polygon_rasterized.size(); i++) {
        vector<RectangleAABBPtr> vec_aabbs_dynamic;
        vec_aabbs_dynamic.reserve(transformed_polygon_rasterized[i].size());
        for (auto polyline_CVLN: transformed_polygon_rasterized[i]) {
            vec_aabbs_dynamic.emplace_back(create_aabb_from_polyline(polyline_CVLN));
        }
        map_step_to_vec_aabbs_dynamic.insert({i, vec_aabbs_dynamic});
    }

    // tuple<vector<RectangleAABBPtr>, map<int, vector<RectangleAABBPtr>>> out_tuple;

    // if (rasterize_static) {
    //     out_tuple = make_tuple(vec_aabbs_static, map_step_to_vec_aabbs_dynamic);
    // } else {
    //     out_tuple = make_tuple(vec_aabbs_static, map_step_to_vec_aabbs_dynamic);
    // }

    // return (rasterized or unrasterized) static AABBs vector and rasterized dynamic AABBs map
    return make_tuple(vec_aabbs_static, map_step_to_vec_aabbs_dynamic);
}

vector<RectangleAABBPtr> reach::create_curvilinear_aabbs_from_cartesian_polylines(
        vector<Polyline> const& vec_polylines,
        CurvilinearCoordinateSystemPtr const& CLCS,
        int const& num_threads,
        BufferConfig const& buffer_config) {

    vector<RectangleAABBPtr> vec_aabbs;
    vec_aabbs.reserve(vec_polylines.size());

#pragma omp parallel num_threads(num_threads) default(none) shared(vec_polylines, buffer_config, CLCS, vec_aabbs)
    {
        vector<RectangleAABBPtr> vec_aabbs_thread;
        vec_aabbs_thread.reserve(vec_polylines.size());

#pragma omp for nowait
        for (auto const& polyline: vec_polylines) {
            auto polygon_geometry = convert_polyline_to_geometry_polygon(polyline);
            auto polygon_inflated = inflate_polygon(polygon_geometry, buffer_config);
            auto polyline_inflated = convert_geometry_polygon_to_polyline(polygon_inflated);

            auto polyline_CVLN = CLCS->convertListOfPointsToCurvilinearCoords(polyline_inflated, 1);
            if (polyline_CVLN.size() >= 2) {
                vec_aabbs_thread.emplace_back(create_aabb_from_polyline(polyline_CVLN));
            }
        }

#pragma omp critical
        vec_aabbs.insert(vec_aabbs.end(),
                         std::make_move_iterator(vec_aabbs_thread.begin()),
                         std::make_move_iterator(vec_aabbs_thread.end()));

    }
    return vec_aabbs;
}

GeometryPolygon reach::convert_polyline_to_geometry_polygon(Polyline const& polyline) {
    // prepare a vector of Boost.Geometry points
    vector<GeometryPoint> vec_points_geometry;
    vec_points_geometry.reserve(polyline.size());
    for (auto const& vertex: polyline) {
        vec_points_geometry.emplace_back(GeometryPoint{vertex.x(), vertex.y()});
    }

    // create Boost.Geometry polygon
    auto polygon_geometry = GeometryPolygon{};
    polygon_geometry.outer().assign(vec_points_geometry.begin(), vec_points_geometry.end());
    bg::correct(polygon_geometry);

    return polygon_geometry;
}

GeometryPolygon reach::inflate_polygon(GeometryPolygon const& polygon, BufferConfig const& buffer_config) {
    // declare the input and the output
    bg::model::multi_polygon<GeometryPolygon> multi_polygon_input;
    multi_polygon_input.emplace_back(polygon);

    bg::model::multi_polygon<GeometryPolygon> multi_polygon_output;

    // compute buffered polygon
    bg::buffer(multi_polygon_input, multi_polygon_output,
               buffer_config.distance_strategy, buffer_config.side_strategy,
               buffer_config.join_strategy, buffer_config.end_strategy, buffer_config.circle_strategy);


    if (multi_polygon_output.size() != 1) {
        throw std::logic_error("<CollisionChecker> Buffering polygon failed.");
    }

    return multi_polygon_output[0];
}

Polyline reach::convert_geometry_polygon_to_polyline(GeometryPolygon const& polygon) {
    Polyline polyline;

    for (auto& vertex: polygon.outer()) {
        polyline.emplace_back(Eigen::Vector2d(vertex.x(), vertex.y()));
    }

    return polyline;
}

RectangleAABBPtr reach::create_aabb_from_polyline(Polyline const& polyline) {
    auto[p_lon_min, p_lat_min, p_lon_max, p_lat_max] = obtain_extremum_coordinates_of_polyline(polyline);
    return create_aabb_from_coordinates(p_lon_min, p_lat_min, p_lon_max, p_lat_max);
}

tuple<double, double, double, double> reach::obtain_extremum_coordinates_of_polyline(Polyline const& polyline) {
    vector<double> vec_p_lon;
    vector<double> vec_p_lat;

    for (auto const& vertex: polyline) {
        vec_p_lon.emplace_back(vertex.x());
        vec_p_lat.emplace_back(vertex.y());
    }

    auto p_lon_min = std::min_element(vec_p_lon.cbegin(), vec_p_lon.cend());
    auto p_lat_min = std::min_element(vec_p_lat.cbegin(), vec_p_lat.cend());
    auto p_lon_max = std::max_element(vec_p_lon.cbegin(), vec_p_lon.cend());
    auto p_lat_max = std::max_element(vec_p_lat.cbegin(), vec_p_lat.cend());

    return make_tuple(*p_lon_min, *p_lat_min, *p_lon_max, *p_lat_max);
}

RectangleAABBPtr reach::create_aabb_from_coordinates(double const& p_lon_min, double const& p_lat_min,
                                                     double const& p_lon_max, double const& p_lat_max) {
    auto length = p_lon_max - p_lon_min;
    auto width = p_lat_max - p_lat_min;
    auto center_lon = (p_lon_min + p_lon_max) / 2.0;
    auto center_lat = (p_lat_min + p_lat_max) / 2.0;

    return std::make_shared<RectangleAABB>(length / 2.0, width / 2.0,
                                           Eigen::Vector2d(center_lon, center_lat));
}

void reach::print_vertices_polygon(vector<Polyline> const& vec_polylines_static) {
    for (auto const& polyline: vec_polylines_static) {
        cout << "New polyline" << endl;
        for (auto const& vertex: polyline) {
            cout << "(" << vertex.x() << ", " << vertex.y() << ")" << endl;
        }
    }
}

void reach::print_collision_checker(CollisionCheckerPtr const& collision_checker) {
    auto vec_obstacles = collision_checker->getObstacles();

    for (auto const& obs: vec_obstacles) {
        if (obs->getCollisionObjectClass() == collision::OBJ_CLASS_TVOBSTACLE) {
            cout << "TVO:" << endl;
            for (int step = 0; step < 10; ++step) {
                auto obj_at_time = obs->timeSlice(step, obs);
                auto aabb = obj_at_time->getAABB();
                cout << aabb->r_x() << ", " << aabb->r_y() << endl;

                cout << "\t" << step << ": " << aabb->center_x() << ", " << aabb->center_y() << endl;
            }
        }
    }
}
