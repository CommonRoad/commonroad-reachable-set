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

CollisionCheckerPtr reach::create_curvilinear_collision_checker(
        vector<Polyline> const& vec_polylines_static,
        map<int, vector<Polyline>> const& map_time_step_to_vec_polylines_dynamic,
        CurvilinearCoordinateSystemPtr const& CLCS,
        double const& radius_disc_vehicle,
        int const& num_omp_threads) {

    auto buffer_config = reach::BufferConfig(radius_disc_vehicle);

    // 1. process static obstacles - convert each polyline into an aabb
    auto vec_aabb_CVLN_static = create_curvilinear_aabbs_from_cartesian_polylines(
            vec_polylines_static, CLCS, num_omp_threads, buffer_config);
    // add the aabbs to a shape group
    auto shape_group_static = make_shared<ShapeGroup>();
    for (auto const& aabb: vec_aabb_CVLN_static) {
        shape_group_static->addToGroup(aabb);
    }

    // 2. process dynamic obstacles - create a shape group for each time step and add aabbs (from polylines) into it
    auto tvo_dynamic = make_shared<TimeVariantCollisionObject>(map_time_step_to_vec_polylines_dynamic.cbegin()->first);
    for (auto const&[time_step, vec_polylines_dynamic]: map_time_step_to_vec_polylines_dynamic) {
        auto vec_aabb_CVLN_dynamic = create_curvilinear_aabbs_from_cartesian_polylines(
                vec_polylines_dynamic, CLCS, num_omp_threads, buffer_config);

        auto shape_group = make_shared<ShapeGroup>();
        for (auto const& aabb: vec_aabb_CVLN_dynamic) {
            shape_group->addToGroup(aabb);
        }

        tvo_dynamic->appendObstacle(shape_group);
    }

    // create the collision checker
    auto collision_checker = make_shared<collision::CollisionChecker>();
    collision_checker->addCollisionObject(shape_group_static);
    collision_checker->addCollisionObject(tvo_dynamic);

    return collision_checker;
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
            for (int time_step = 0; time_step < 10; ++time_step) {
                auto obj_at_time = obs->timeSlice(time_step, obs);
                auto aabb = obj_at_time->getAABB();
                cout << aabb->r_x() << ", " << aabb->r_y() << endl;

                cout << "\t" << time_step << ": " << aabb->center_x() << ", " << aabb->center_y() << endl;
            }
        }
    }
}
