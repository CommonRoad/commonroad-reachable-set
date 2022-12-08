#include "reachset/utility/reach_operation.hpp"
#include "reachset/utility/sweep_line.hpp"
#include "reachset/utility/shared_using.hpp"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedLocalVariable"
using namespace reach;


/// *Steps*:
/// 1. prepare a bounding box to be intersected with halfspaces.
/// 2. compute coefficients of halfspaces and intersect them with the bounding polygon. We use three halfspaces to
/// approximate the upper bound of the polygon, this applies to the lower bound as well. A halfspace is dependent on
/// the given switching time (gamma).
ReachPolygonPtr reach::create_zero_state_polygon(double const& dt, double const& a_min, double const& a_max) {
    auto polygon = create_bounding_box(dt, a_min, a_max);

    for (auto& gamma: {0.0, 0.5, 1.0}) {
        auto [tuple_coefficients_upper, tuple_coefficients_lower] =
                compute_halfspace_coefficients(dt, a_min, a_max, gamma);
        polygon->intersect_halfspace(std::get<0>(tuple_coefficients_upper),
                                     std::get<1>(tuple_coefficients_upper),
                                     std::get<2>(tuple_coefficients_upper));

        polygon->intersect_halfspace(std::get<0>(tuple_coefficients_lower),
                                     std::get<1>(tuple_coefficients_lower),
                                     std::get<2>(tuple_coefficients_lower));
    }
    polygon->convexify();
    polygon->sort_vertices_bottom_left_first();

    return polygon;
}

ReachPolygonPtr reach::create_bounding_box(double const& dt, double const& a_min, double const& a_max) {

    tuple tuple_vertices{0.5 * a_min * pow(dt, 2), a_min * dt, 0.5 * a_max * pow(dt, 2), a_max * dt};

    return make_shared<ReachPolygon>(tuple_vertices);
}


/// Each halfspace has the form of ax + bv <= c. In the x-v plane, the x and v values are both dependent on gamma
/// (switching time, see
/// <a href="https://ieeexplore.ieee.org/abstract/document/8047450?casa_token=kfXwS_YEKoEAAAAA:k6TBp5W4m1BDxAZuMWI8S8T1I-fe38s3txJkzuNjiUMPv2T9Ccp1rzCmzNx8Z27VnnIgA4LU-Q">
/// *Computing the Drivable Area of Autonomous Road Vehicles in Dynamic Road Scenes*</a> , Sec.III.A.).
/// Given a gamma, the maximum and minimum reachable x and v can be computed, respectively. The halfspace crossing the
/// point x_ and v_ can be obtained through:\n
/// v - v_ = (dv_/dx_) * (x - x_) = (dv_/dgamma * dgamma/dx_) * (x - x_)\n
/// ==>\n
/// (dv_/dgamma) * x + (-dx_/dgamma) * v <= (dv_/dgamma * x_ - -dx_/dgamma * v_)
///
/// Essentially, the expanded form is performing the following computation:\n
/// 1) full braking (or acceleration) before the switching time
/// x_0 = 0
/// v_0 = 0
/// a = a_min (a_max)
/// t = gamma * dt - 0
///
/// x_t = 0.5 * a * t**2 + v_0 * t + x_0
/// v_t = a * t + v_0
///
/// 2) full acceleration (or braking) after the switching time
/// x_0 = x_t
/// v_0 = v_t
/// a = a_max (a_min)
/// t = dt - gamma * dt
///
/// x_f = 0.5 * a * t**2 + v_0 * t + x_0
/// v_f = a * t + v_0
///
/// dx_XXX & dv_XXX are derivatives with regard to gamma.
tuple<tuple<double, double, double>, tuple<double, double, double>>
reach::compute_halfspace_coefficients(double const& dt, double const& a_min, double const& a_max, double const& gamma) {
    auto x_upper = a_max * pow(dt, 2) * (0.5 - gamma + 0.5 * pow(gamma, 2)) +
                   a_min * pow(dt, 2) * (gamma - 0.5 * pow(gamma, 2));
    auto v_upper = a_min * gamma * dt + a_max * dt * (1 - gamma);
    auto dx_upper = a_max * pow(dt, 2) * (-1 + gamma) + a_min * pow(dt, 2) * (1 - gamma);
    auto dv_upper = a_min * dt - a_max * dt;

    auto x_lower = a_min * pow(dt, 2) * (0.5 - gamma + 0.5 * pow(gamma, 2)) +
                   a_max * pow(dt, 2) * (gamma - 0.5 * pow(gamma, 2));
    auto v_lower = a_max * gamma * dt + a_min * dt * (1 - gamma);
    auto dx_lower = a_min * pow(dt, 2) * (-1 + gamma) + a_max * pow(dt, 2) * (1 - gamma);
    auto dv_lower = a_max * dt - a_min * dt;

    return {{dv_upper, -dx_upper, dv_upper * x_upper - dx_upper * v_upper},
            {dv_lower, -dx_lower, dv_lower * x_lower - dx_lower * v_lower}};
}

tuple<double, double, double, double>
reach::generate_tuple_vertices_position_rectangle_initial(ConfigurationPtr const& config) {
    tuple tuple_vertices = {config->planning().p_lon_initial - config->planning().uncertainty_p_lon,
                            config->planning().p_lat_initial - config->planning().uncertainty_p_lat,
                            config->planning().p_lon_initial + config->planning().uncertainty_p_lon,
                            config->planning().p_lat_initial + config->planning().uncertainty_p_lat};
    return tuple_vertices;
}

tuple<tuple<double, double, double, double>, tuple<double, double, double, double>>
reach::generate_tuples_vertices_polygons_initial(ConfigurationPtr const& config) {
    tuple tuple_vertices_polygon_lon = {config->planning().p_lon_initial - config->planning().uncertainty_p_lon,
                                        config->planning().v_lon_initial - config->planning().uncertainty_v_lon,
                                        config->planning().p_lon_initial + config->planning().uncertainty_p_lon,
                                        config->planning().v_lon_initial + config->planning().uncertainty_v_lon};

    tuple tuple_vertices_polygon_lat = {config->planning().p_lat_initial - config->planning().uncertainty_p_lat,
                                        config->planning().v_lat_initial - config->planning().uncertainty_v_lat,
                                        config->planning().p_lat_initial + config->planning().uncertainty_p_lat,
                                        config->planning().v_lat_initial + config->planning().uncertainty_v_lat};

    return {tuple_vertices_polygon_lon, tuple_vertices_polygon_lat};
}

/// *Steps*:
/// 1. make a copy
/// 2. compute the linear mapping (zero-input response) of the polygon
/// 3. convexify
/// 4. compute the minkowski sum of zero-input and zero-state responses
/// 5. intersect with halfspaces to consider velocity limits
ReachPolygonPtr reach::propagate_polygon(ReachPolygonPtr const& polygon, ReachPolygonPtr const& polygon_zero_state,
                                         double const& dt, double const& v_min, double const& v_max) {
    // create a new object (does not modify the passed in pointer)
    auto polygon_propagated = polygon->clone();
    polygon_propagated->linear_mapping(1, dt, 0, 1);
    polygon_propagated->convexify();
    polygon_propagated->minkowski_sum(polygon_zero_state);
    polygon_propagated->intersect_halfspace(0, 1, v_max);
    polygon_propagated->intersect_halfspace(0, -1, -v_min);

    return polygon_propagated;
}

vector<ReachPolygonPtr>
reach::project_base_sets_to_position_domain(vector<ReachNodePtr> const& vec_base_sets_propagated) {
    vector<ReachPolygonPtr> vec_rectangles_projected;
    vec_rectangles_projected.reserve(vec_base_sets_propagated.size());

    for (auto const& base_set: vec_base_sets_propagated) {
        vec_rectangles_projected.emplace_back(make_shared<ReachPolygon>(base_set->polygon_lon->p_min(),
                                                                        base_set->polygon_lat->p_min(),
                                                                        base_set->polygon_lon->p_max(),
                                                                        base_set->polygon_lat->p_max()));
    }

    return vec_rectangles_projected;
}

/// *Steps*:
/// 1. obtain the minimum lon/lat positions of the list of rectangles.
/// 2. discretize rectangles
/// 3. repartition the rectangles into a new list of rectangles.
/// 4. restore the rectangles back to undiscretized ones.
vector<ReachPolygonPtr>
reach::create_repartitioned_rectangles(vector<ReachPolygonPtr> const& vec_rectangles, double const& size_grid) {
    if (vec_rectangles.size() <= 1) return vec_rectangles;

    auto tuple_p_min_rectangles = compute_minimum_positions_of_rectangles(vec_rectangles);

    auto vec_rectangles_discretized = discretize_rectangles(vec_rectangles, tuple_p_min_rectangles, size_grid);

    auto vec_rectangles_repartitioned = repartition_rectangle(vec_rectangles_discretized);

    auto vec_rectangles_undiscretized = undiscretize_rectangles(vec_rectangles_repartitioned,
                                                                tuple_p_min_rectangles, size_grid);

    return vec_rectangles_undiscretized;
}

tuple<double, double> reach::compute_minimum_positions_of_rectangles(vector<ReachPolygonPtr> const& vec_rectangles) {
    auto p_lon_min_rectangles = std::numeric_limits<double>::infinity();
    auto p_lat_min_rectangles = std::numeric_limits<double>::infinity();

    for (auto const& rectangle: vec_rectangles) {
        p_lon_min_rectangles = std::min(p_lon_min_rectangles, rectangle->p_lon_min());
        p_lat_min_rectangles = std::min(p_lat_min_rectangles, rectangle->p_lat_min());
    }

    return {p_lon_min_rectangles, p_lat_min_rectangles};
}

/// p_discretized = (p_undiscretized - p_min) / size_grid
/// floor for min values, and ceil for max values for over-approximation.
vector<ReachPolygonPtr> reach::discretize_rectangles(vector<ReachPolygonPtr> const& vec_rectangles,
                                                     tuple<double, double> const& tuple_p_min_rectangles,
                                                     double const& size_grid) {
    vector<ReachPolygonPtr> vec_rectangles_discretized;
    vec_rectangles_discretized.reserve(vec_rectangles.size());
    auto [p_lon_min_rectangles, p_lat_min_rectangles] = tuple_p_min_rectangles;
    double p_lon_min, p_lat_min, p_lon_max, p_lat_max;

    for (auto const& rectangle: vec_rectangles) {
        p_lon_min = floor((rectangle->p_lon_min() - p_lon_min_rectangles) / size_grid);
        p_lat_min = floor((rectangle->p_lat_min() - p_lat_min_rectangles) / size_grid);
        p_lon_max = ceil((rectangle->p_lon_max() - p_lon_min_rectangles) / size_grid);
        p_lat_max = ceil((rectangle->p_lat_max() - p_lat_min_rectangles) / size_grid);

        vec_rectangles_discretized.emplace_back(make_shared<ReachPolygon>(p_lon_min, p_lat_min, p_lon_max, p_lat_max));
    }

    return vec_rectangles_discretized;
}

/// p_undiscretized = p_discretized * size_grid + p_min
vector<ReachPolygonPtr> reach::undiscretize_rectangles(vector<ReachPolygonPtr> const& vec_rectangles,
                                                       tuple<double, double> const& tuple_p_min_rectangles,
                                                       double const& size_grid) {
    vector<ReachPolygonPtr> vec_rectangles_undiscretized;
    vec_rectangles_undiscretized.reserve(vec_rectangles.size());
    auto [p_lon_min_rectangles, p_lat_min_rectangles] = tuple_p_min_rectangles;
    double p_lon_min, p_lat_min, p_lon_max, p_lat_max;

    for (auto const& rectangle: vec_rectangles) {
        p_lon_min = rectangle->p_lon_min() * size_grid + p_lon_min_rectangles;
        p_lat_min = rectangle->p_lat_min() * size_grid + p_lat_min_rectangles;
        p_lon_max = rectangle->p_lon_max() * size_grid + p_lon_min_rectangles;
        p_lat_max = rectangle->p_lat_max() * size_grid + p_lat_min_rectangles;

        vec_rectangles_undiscretized.emplace_back(
                make_shared<ReachPolygon>(p_lon_min, p_lat_min, p_lon_max, p_lat_max));
    }

    return vec_rectangles_undiscretized;
}

/// *Steps*:
/// 1. Obtain a list of vertical segments representing the contour of the union of the rectangles.
/// 2. Create repartitioned rectangles from the list of vertical segments using sweep line algorithm.
vector<ReachPolygonPtr> reach::repartition_rectangle(vector<ReachPolygonPtr> const& vec_rectangles) {
    // Step 1
    auto vec_segments_vertical = SweepLine::obtain_vertical_segments_from_rectangles(vec_rectangles);

    // Step 2
    auto vec_rectangles_repartitioned = SweepLine::create_rectangles_from_vertical_segments(vec_segments_vertical);

    return vec_rectangles_repartitioned;
}

vector<ReachPolygonPtr> reach::check_collision_and_split_rectangles(int const& step,
                                                                    CollisionCheckerPtr const& collision_checker,
                                                                    vector<ReachPolygonPtr> const& vec_rectangles,
                                                                    double const& radius_terminal_split,
                                                                    int const& num_threads) {
    if (vec_rectangles.empty()) return {};

    vector<ReachPolygonPtr> vec_rectangles_collision_free;
    auto radius_terminal_squared = pow(radius_terminal_split, 2);
    // make a window query w.r.t rectangles to speed up the collision check
    auto collision_checker_sliced = collision_checker->timeSlice(step)->
            windowQuery(obtain_bounding_box_of_rectangles(vec_rectangles));

    for (auto const& rectangle: vec_rectangles) {
        auto aabb = convert_reach_polygon_to_collision_aabb(rectangle);

        auto vec_aabbs_split = create_collision_free_rectangles(collision_checker_sliced,
                                                                aabb, radius_terminal_squared);
        auto vec_rectangles_split = convert_collision_aabbs_to_reach_polygons(vec_aabbs_split);

        vec_rectangles_collision_free.insert(vec_rectangles_collision_free.end(),
                                             std::make_move_iterator(vec_rectangles_split.begin()),
                                             std::make_move_iterator(vec_rectangles_split.end()));
    }

    return vec_rectangles_collision_free;
}

vector<ReachPolygonPtr> reach::check_collision_and_split_rectangles(int const& step,
                                                                    CollisionCheckerPtr const& collision_checker,
                                                                    vector<ReachPolygonPtr> const& vec_rectangles,
                                                                    double const& radius_terminal_split,
                                                                    int const& num_threads,
                                                                    double const& circle_distance,
                                                                    const geometry::CurvilinearCoordinateSystem& cosy,
                                                                    const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                                    ReferencePoint reference_point) {
    if (vec_rectangles.empty()) return {};

    vector<ReachPolygonPtr> vec_rectangles_collision_free;
    auto radius_terminal_squared = pow(radius_terminal_split, 2);

    // loop over rectangles (ReachPolygons)
    for (auto const& rectangle: vec_rectangles) {
        // enlarge rectangle for windowQuery
        double x_lo, x_hi, y_lo, y_hi = 0.;
        std::tie(x_lo, x_hi, y_lo, y_hi) = computeEnlargedRectangle(cosy,
                                                                    lut_lon_enlargement,
                                                                    reference_point,
                                                                    circle_distance,
                                                                    std::make_tuple(rectangle->p_lon_min(),
                                                                                    rectangle->p_lon_max(),
                                                                                    rectangle->p_lat_min(),
                                                                                    rectangle->p_lat_max()));

        // create collision object RectangleAABB for enlarged rectangle
        collision::RectangleAABB enlarged_rect = collision::RectangleAABB{(x_hi - x_lo) / 2.0,
                                                                          (y_hi - y_lo) / 2.0,
                                                                          Eigen::Vector2d((x_hi + x_lo) / 2.0,
                                                                                          (y_hi + y_lo) / 2.0)};


        // make a window query w.r.t enlarged rectangle to speed up the collision check
        auto collision_checker_sliced = collision_checker->timeSlice(step)->windowQuery(enlarged_rect);

        // create collision object RectangleAABB from (non-enlarged) ReachPolygon rectangle
        auto aabb = convert_reach_polygon_to_collision_aabb(rectangle);

        // check for collisions considering enlargement of sets
        auto vec_aabbs_split = create_collision_free_rectangles(collision_checker_sliced,
                                                                aabb,
                                                                radius_terminal_squared,
                                                                circle_distance,
                                                                cosy,
                                                                lut_lon_enlargement,
                                                                reference_point);

        // convert collision-free rectangles back to ReachPolygons
        auto vec_rectangles_split = convert_collision_aabbs_to_reach_polygons(vec_aabbs_split);

        vec_rectangles_collision_free.insert(vec_rectangles_collision_free.end(),
                                             std::make_move_iterator(vec_rectangles_split.begin()),
                                             std::make_move_iterator(vec_rectangles_split.end()));
    }

    return vec_rectangles_collision_free;
}

tuple<double, double, double, double>
reach::obtain_extremum_coordinates_of_polygons(vector<ReachPolygonPtr> const& vec_polygons) {
    vector<double> vec_p_lon;
    vector<double> vec_p_lat;

    for (auto const& polygon: vec_polygons) {
        vec_p_lon.emplace_back(polygon->p_lon_min());
        vec_p_lon.emplace_back(polygon->p_lon_max());
        vec_p_lat.emplace_back(polygon->p_lat_min());
        vec_p_lat.emplace_back(polygon->p_lat_max());
    }

    auto p_lon_min = std::min_element(vec_p_lon.cbegin(), vec_p_lon.cend());
    auto p_lat_min = std::min_element(vec_p_lat.cbegin(), vec_p_lat.cend());
    auto p_lon_max = std::max_element(vec_p_lon.cbegin(), vec_p_lon.cend());
    auto p_lat_max = std::max_element(vec_p_lat.cbegin(), vec_p_lat.cend());

    return make_tuple(*p_lon_min, *p_lat_min, *p_lon_max, *p_lat_max);
}

collision::RectangleAABB reach::obtain_bounding_box_of_rectangles(vector<ReachPolygonPtr> const& vec_rectangles) {
    auto p_lon_min = std::numeric_limits<double>::infinity();
    auto p_lat_min = std::numeric_limits<double>::infinity();
    auto p_lon_max = -std::numeric_limits<double>::infinity();
    auto p_lat_max = -std::numeric_limits<double>::infinity();

    for (auto const& rectangle: vec_rectangles) {
        p_lon_min = std::min(p_lon_min, rectangle->p_lon_min());
        p_lat_min = std::min(p_lat_min, rectangle->p_lat_min());
        p_lon_max = std::max(p_lon_max, rectangle->p_lon_max());
        p_lat_max = std::max(p_lat_max, rectangle->p_lat_max());
    }

    auto length = (p_lon_max - p_lon_min);
    auto width = (p_lat_max - p_lat_min);
    auto center_lon = (p_lon_max + p_lon_min) / 2;
    auto center_lat = (p_lat_max + p_lat_min) / 2;

    return collision::RectangleAABB{length / 2, width / 2, Eigen::Vector2d(center_lon, center_lat)};
}

RectangleAABBPtr reach::convert_reach_polygon_to_collision_aabb(ReachPolygonPtr const& rectangle) {
    auto length = (rectangle->p_lon_max() - rectangle->p_lon_min());
    auto width = (rectangle->p_lat_max() - rectangle->p_lat_min());
    auto center_lon = (rectangle->p_lon_max() + rectangle->p_lon_min()) / 2;
    auto center_lat = (rectangle->p_lat_max() + rectangle->p_lat_min()) / 2;

    return make_shared<collision::RectangleAABB>(length / 2, width / 2, Eigen::Vector2d(center_lon, center_lat));
}

vector<ReachPolygonPtr>
reach::convert_collision_aabbs_to_reach_polygons(vector<RectangleAABBPtr> const& vec_rectangles) {
    vector<ReachPolygonPtr> vec_polygons;
    vec_polygons.reserve(vec_rectangles.size());

    for (auto const& rectangle: vec_rectangles) {
        auto p_lon_min = rectangle->center_x() - rectangle->r_x();
        auto p_lon_max = rectangle->center_x() + rectangle->r_x();
        auto p_lat_min = rectangle->center_y() - rectangle->r_y();
        auto p_lat_max = rectangle->center_y() + rectangle->r_y();

        vec_polygons.emplace_back(make_shared<ReachPolygon>(p_lon_min, p_lat_min, p_lon_max, p_lat_max));
    }

    return vec_polygons;
}

/// If a collision happens between a rectangle and other object, and that the diagonal of the rectangle is greater
/// than the terminal radius, it is split into two new rectangles in whichever edge (lon/lat) that is longer.
vector<RectangleAABBPtr> reach::create_collision_free_rectangles(CollisionCheckerPtr const& collision_checker,
                                                                 RectangleAABBPtr const& rectangle,
                                                                 double const& radius_terminal_squared) {
    // case 1: rectangle does not collide, return itself
    if (not collision_checker->collide(rectangle)) return {rectangle};
        // case 2: the diagonal is smaller than the terminal radius, return nothing
    else if (diagonal_squared(rectangle) < radius_terminal_squared) return {};
        // case 3: colliding but diagonal is long enough. split into two halves.
    else {
        auto [rectangle_split_a, rectangle_split_b] = split_rectangle_into_two(rectangle);

        auto vec_rectangles_split_a =
                create_collision_free_rectangles(collision_checker, rectangle_split_a, radius_terminal_squared);
        auto vec_rectangles_split_b =
                create_collision_free_rectangles(collision_checker, rectangle_split_b, radius_terminal_squared);

        vec_rectangles_split_a.insert(vec_rectangles_split_a.end(),
                                      std::make_move_iterator(vec_rectangles_split_b.begin()),
                                      std::make_move_iterator(vec_rectangles_split_b.end()));

        return vec_rectangles_split_a;
    }
}

vector<RectangleAABBPtr> reach::create_collision_free_rectangles(CollisionCheckerPtr const& collision_checker,
                                                                 RectangleAABBPtr const& rectangle,
                                                                 double const& radius_terminal_squared,
                                                                 double const& circle_distance,
                                                                 const geometry::CurvilinearCoordinateSystem& cosy,
                                                                 const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                                 ReferencePoint reference_point) {

    // enlarge rectangle for collision checking
    // center-radius representation of collision RectangleAABB
    double c_x = rectangle->center_x();
    double c_y = rectangle->center_y();
    double r_x = rectangle->r_x();
    double r_y = rectangle->r_y();

    // extremum points of enlarged rectangle
    double x_lo, x_hi, y_lo, y_hi = 0.;
    std::tie(x_lo, x_hi, y_lo, y_hi) = computeEnlargedRectangle(cosy,
                                                                lut_lon_enlargement,
                                                                reference_point,
                                                                circle_distance,
                                                                std::make_tuple(c_x - r_x,
                                                                                c_x + r_x,
                                                                                c_y - r_y,
                                                                                c_y + r_y));

    // create collision object (RectangleAABB) for enlarged rectangle
    collision::RectangleAABBConstPtr enlarged_rect = std::make_shared<collision::RectangleAABB>(
            (x_hi - x_lo) / 2.0,
            (y_hi - y_lo) / 2.0,
            Eigen::Vector2d((x_hi + x_lo) / 2.0, (y_hi + y_lo) / 2.0));

    // check for collision with enlarged rectangle
    bool col_res = false;
    col_res = collision_checker->collide(enlarged_rect);
    // case 1: enlarged rectangle does not collide, return original rectangle as collision free
    if (not col_res) return {rectangle};
        // case 2: the diagonal of the original rectangle is smaller than the terminal radius, return nothing
    else if (diagonal_squared(rectangle) < radius_terminal_squared) return {};
        // case 3: colliding but diagonal is long enough. split original rectangle into two halves.
    else {
        // split rectangles into two
        auto [rectangle_split_a, rectangle_split_b] = split_rectangle_into_two(rectangle);

        // recursion: check first rectangle for collision
        auto vec_rectangles_split_a =
                create_collision_free_rectangles(collision_checker,
                                                 rectangle_split_a,
                                                 radius_terminal_squared,
                                                 circle_distance,
                                                 cosy,
                                                 lut_lon_enlargement,
                                                 reference_point);

        // recursion: check second rectangle for collision
        auto vec_rectangles_split_b =
                create_collision_free_rectangles(collision_checker,
                                                 rectangle_split_b,
                                                 radius_terminal_squared,
                                                 circle_distance,
                                                 cosy,
                                                 lut_lon_enlargement,
                                                 reference_point);

        // add all collision-free rectangles to return vector
        vec_rectangles_split_a.insert(vec_rectangles_split_a.end(),
                                      std::make_move_iterator(vec_rectangles_split_b.begin()),
                                      std::make_move_iterator(vec_rectangles_split_b.end()));

        return vec_rectangles_split_a;
    }
}


/// Splits in the longer axis of the two.
tuple<RectangleAABBPtr, RectangleAABBPtr> reach::split_rectangle_into_two(RectangleAABBPtr const& rectangle) {
    RectangleAABBPtr rectangle_split_a;
    RectangleAABBPtr rectangle_split_b;

    if (rectangle->r_x() > rectangle->r_y()) {
        auto rx_half = rectangle->r_x() / 2;
        rectangle_split_a = make_shared<collision::RectangleAABB>(
                rx_half, rectangle->r_y(), Eigen::Vector2d(rectangle->center_x() - rx_half, rectangle->center_y()));
        rectangle_split_b = make_shared<collision::RectangleAABB>(
                rx_half, rectangle->r_y(), Eigen::Vector2d(rectangle->center_x() + rx_half, rectangle->center_y()));
    } else {
        auto ry_half = rectangle->r_y() / 2;
        rectangle_split_a = make_shared<collision::RectangleAABB>(
                rectangle->r_x(), ry_half, Eigen::Vector2d(rectangle->center_x(), rectangle->center_y() - ry_half));
        rectangle_split_b = make_shared<collision::RectangleAABB>(
                rectangle->r_x(), ry_half, Eigen::Vector2d(rectangle->center_x(), rectangle->center_y() + ry_half));
    }

    return {rectangle_split_a, rectangle_split_b};
}

/// *Steps*:
/// 1. examine the adjacency of drivable area and the propagated base sets. They are considered adjacent if they
///    overlap in the position domain.
/// 2. create a node from each drivable area and its adjacent propagated base sets.
vector<ReachNodePtr> reach::construct_reach_nodes(
        vector<ReachPolygonPtr> const& drivable_area,
        vector<ReachNodePtr> const& vec_base_sets_propagated, int const& num_threads) {

    vector<ReachNodePtr> reachable_set;
    reachable_set.reserve(drivable_area.size());

    vector<ReachPolygonPtr> vec_rectangles_base_sets;
    vec_rectangles_base_sets.reserve(vec_base_sets_propagated.size());

    for (auto const& base_set: vec_base_sets_propagated) {
        vec_rectangles_base_sets.emplace_back(base_set->position_rectangle());
    }
    auto& vec_rectangles_drivable_area = drivable_area;
    auto map_rectangle_adjacency = create_adjacency_map(vec_rectangles_drivable_area,
                                                        vec_rectangles_base_sets);

#pragma omp parallel num_threads(num_threads) default(none) shared(drivable_area, map_rectangle_adjacency, \
vec_rectangles_drivable_area, vec_base_sets_propagated, reachable_set)
    {
        vector<ReachNodePtr> reachable_set_step_current_thread;
        reachable_set_step_current_thread.reserve(drivable_area.size());

#pragma omp for nowait
        for (int idx = 0; idx < map_rectangle_adjacency.size(); idx++) {
            auto it = map_rectangle_adjacency.begin();
            std::advance(it, idx);

            auto const idx_drivable_area = it->first;
            auto const vec_idx_base_sets_adjacent = it->second;
            auto const& rectangle_drivable_area = vec_rectangles_drivable_area[idx_drivable_area];

            auto reach_node = construct_reach_node(rectangle_drivable_area,
                                                   vec_base_sets_propagated,
                                                   vec_idx_base_sets_adjacent);
            if (reach_node != nullptr)
                reachable_set_step_current_thread.emplace_back(reach_node);
        }
#pragma omp critical
        reachable_set.insert(reachable_set.end(),
                             std::make_move_iterator(reachable_set_step_current_thread.begin()),
                             std::make_move_iterator(reachable_set_step_current_thread.end()));
    }

    return reachable_set;
}

/// E.g.: {0 : [1, 2], 1 : [3, 4]} = rectangle_0 from 1st vector overlaps with rectangles_1 and _2 from the 2nd vector;
//  rectangle_1 from 1st vector overlaps with rectangles_3 and _4 from the 2nd vector.
unordered_map<int, vector<int>> reach::create_adjacency_map(vector<ReachPolygonPtr> const& vec_rectangles_a,
                                                            vector<ReachPolygonPtr> const& vec_rectangles_b) {
    unordered_map<int, vector<int>> map_idx_to_vec_idx;

    for (int idx_a = 0; idx_a < vec_rectangles_a.size(); idx_a++) {
        auto const& rectangle_a = vec_rectangles_a[idx_a];

        for (int idx_b = 0; idx_b < vec_rectangles_b.size(); idx_b++) {
            auto const& rectangle_b = vec_rectangles_b[idx_b];

            if (rectangle_a->intersects(rectangle_b)) { map_idx_to_vec_idx[idx_a].emplace_back(idx_b); }
        }
    }

    return map_idx_to_vec_idx;
}

/// Iterate through base sets that are adjacent to the drivable area, and cut the base sets down with position
/// constraints from the drivable area. A non-empty intersected polygon imply that it is a valid base set and is
/// considered as a parent of the rectangle (reachable from the node from which the base set is propagated).
ReachNodePtr reach::construct_reach_node(ReachPolygonPtr const& rectangle_drivable_area,
                                         vector<ReachNodePtr> const& vec_base_sets_propagated,
                                         vector<int> const& vec_idx_base_sets_adjacent) {
    vector<ReachNodePtr> vec_base_sets_parent;
    vector<Vertex> vec_vertices_polygon_lon_new;
    vector<Vertex> vec_vertices_polygon_lat_new;

    // iterate through each of the adjacent base sets
    for (auto const& idx_base_set_adjacent: vec_idx_base_sets_adjacent) {
        auto const& base_set_adjacent = vec_base_sets_propagated[idx_base_set_adjacent];
        auto polygon_lon = base_set_adjacent->polygon_lon->clone();
        auto polygon_lat = base_set_adjacent->polygon_lat->clone();
        // cut down to position range of the drivable area rectangle
        try {
            polygon_lon->intersect_halfspace(1, 0, rectangle_drivable_area->p_lon_max());
            polygon_lon->intersect_halfspace(-1, 0, -rectangle_drivable_area->p_lon_min());
            polygon_lat->intersect_halfspace(1, 0, rectangle_drivable_area->p_lat_max());
            polygon_lat->intersect_halfspace(-1, 0, -rectangle_drivable_area->p_lat_min());
        }
        catch (std::exception& e) {
            continue;
        }

        if (!polygon_lon->empty() and !polygon_lat->empty()) {
            // add to list if the intersected polygons are nonempty
            for (auto const& vertex: polygon_lon->vertices()) {
                vec_vertices_polygon_lon_new.emplace_back(Vertex(vertex.p_lon(), vertex.p_lat()));
            }

            for (auto const& vertex: polygon_lat->vertices()) {
                vec_vertices_polygon_lat_new.emplace_back(Vertex(vertex.p_lon(), vertex.p_lat()));
            }

            // the propagation of a node has only one source of propagation
            vec_base_sets_parent.emplace_back(base_set_adjacent->vec_nodes_source[0]);
        }

    }

    if (not vec_vertices_polygon_lon_new.empty() and not vec_vertices_polygon_lat_new.empty()) {
        try {
            // if there is at least one valid base set
            auto polygon_lon_new = make_shared<ReachPolygon>(vec_vertices_polygon_lon_new);
            auto polygon_lat_new = make_shared<ReachPolygon>(vec_vertices_polygon_lat_new);
            polygon_lon_new->convexify();
            polygon_lat_new->convexify();

            auto reach_node =
                    make_shared<ReachNode>(-1, polygon_lon_new, polygon_lat_new);
            reach_node->vec_nodes_source = vec_base_sets_parent;

            return reach_node;
        }
        catch (std::exception& e) {
            return nullptr;
        }
    } else
        return nullptr;
}

vector<ReachNodePtr> reach::connect_children_to_parents(int const& step,
                                                        vector<ReachNodePtr> const& vec_base_sets_adapted,
                                                        int const& num_threads) {
    for (auto& node_child: vec_base_sets_adapted) {
        node_child->step = step;
        // update parent-child relationship
        for (auto& node_parent: node_child->vec_nodes_source) {
            node_child->add_parent_node(node_parent);
            node_parent->add_child_node(node_child);
        }
    }

    return vec_base_sets_adapted;
}