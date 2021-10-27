import numpy as np
import pycrccosy
from commonroad.geometry.shape import Shape, ShapeGroup, Rectangle, Polygon
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad.scenario.trajectory import State


def convert_obstacles_to_curvilinear_coordinate_system(list_obstacles_CART, CLCS):
    """Returns a list of obstacles converted into curvilinear coordinate system.

    Splitting obstacles in the Cartesian coordinate system into smaller rectangles (rasterization) reduces over-
    approximation in the curvilinear coordinate system, since they are converted into axis-aligned rectangles.
    """

    list_obstacles_static_CART = [obs for obs in list_obstacles_CART if isinstance(obs, StaticObstacle)]
    list_obstacles_dynamic_CART = [obs for obs in list_obstacles_CART if isinstance(obs, DynamicObstacle)]

    list_obstacles_static_CVLN = convert_to_curvilinear_static_obstacles(list_obstacles_static_CART, CLCS)
    list_obstacles_dynamic_CVLN = convert_to_curvilinear_dynamic_obstacles(list_obstacles_dynamic_CART, CLCS)

    return list_obstacles_static_CVLN + list_obstacles_dynamic_CVLN


def convert_to_curvilinear_static_obstacles(list_obstacles_static_CART, CLCS: pycrccosy.CurvilinearCoordinateSystem):
    """Converts a list of static obstacles to obstacle under Curvilinear coordinate system."""
    list_obstacles_static_CVLN = []

    for obstacle in list_obstacles_static_CART:
        time_step_initial = obstacle.initial_state.time_step

        occupancy = obstacle.occupancy_at_time(time_step_initial)

        if isinstance(occupancy.shape, Shape):
            shape_obstacle_CVLN = convert_to_curvilinear_shape(occupancy.shape, CLCS)

            id_obstacle = obstacle.obstacle_id
            type_obstacle = obstacle.obstacle_type
            position = obstacle.initial_state.position
            position_obstacle_CVLN = CLCS.convert_to_curvilinear_coords(position[0], position[1])

            state_initial_obstacle_CVLN = State(position=position_obstacle_CVLN, orientation=0.00,
                                                time_step=time_step_initial)

            # feed in the required components to construct a static obstacle
            static_obstacle = StaticObstacle(id_obstacle, type_obstacle,
                                             shape_obstacle_CVLN, state_initial_obstacle_CVLN)

            list_obstacles_static_CVLN.append(static_obstacle)

        # elif isinstance(occupancy.shape, ShapeGroup):
        #     shape_group_CVLN = convert_to_curvilinear_shape_group(occupancy.shape)

    return list_obstacles_static_CVLN


def convert_to_curvilinear_shape(shape: Shape, CLCS):
    """Converts a rectangle or polygon to Curvilinear coordinate system."""
    if isinstance(shape, Rectangle):
        list_vertices_CVLN = convert_to_curvilinear_vertices(shape.vertices, CLCS)
        rectangle, position = create_rectangle_from_vertices(list_vertices_CVLN)

        return rectangle

    elif isinstance(shape, Polygon):
        list_vertices_CVLN = convert_to_curvilinear_vertices(shape.vertices, CLCS)
        polygon, position = create_polygon_from_vertices(list_vertices_CVLN)

        return polygon
    else:
        return None


def convert_to_curvilinear_vertices(list_vertices_CART, CLCS):
    """Converts a list of vertices to Curvilinear coordinate system."""
    try:
        list_vertices_CVLN = [
            CLCS.convert_to_curvilinear_coords(vertex[0], vertex[1])
            for vertex in list_vertices_CART
        ]
    except ValueError:
        return []
    else:
        return list_vertices_CVLN


def create_rectangle_from_vertices(list_vertices):
    """Returns a rectangle and its position for the given list of vertices."""
    if not list_vertices:
        return None, None

    list_p_lon = [vertex[0] for vertex in list_vertices]
    list_p_lat = [vertex[1] for vertex in list_vertices]

    p_lon_min = min(list_p_lon)
    p_lat_min = min(list_p_lat)
    p_lon_max = max(list_p_lon)
    p_lat_max = max(list_p_lat)

    length = p_lon_max - p_lon_min
    width = p_lat_max - p_lat_min
    position = np.array([(p_lon_min + p_lon_max) / 2.0, (p_lat_min + p_lat_max) / 2.0])

    return Rectangle(length=length, width=width), position


def create_polygon_from_vertices(list_vertices):
    """Returns a polygon and its position for the given list of vertices."""
    if not list_vertices:
        return None, None


def convert_to_curvilinear_static_obstacles(list_obstacles_static_CART, CLCS):
    list_obstacles_static_CVLN = []
    for obstacle in list_obstacles_static_CART:
        occupancy = obstacle.occupancy_at_time(obstacle.initial_state.time_step)

        if isinstance(occupancy.shape, Shape):
            shape_CVLN = convert_to_curvilinear_shape(occupancy.shape, CLCS)
        elif isinstance(occupancy.shape, ShapeGroup):
            shape_group_CVLN = convert_to_curvilinear_shape_group(occupancy.shape)

    return list_obstacles_static_CVLN


def convert_to_curvilinear_shape(shape: Shape, CLCS):
    if isinstance(shape, Rectangle):
        list_vertices_CVLN = convert_to_curvilinear_vertices(shape.vertices, CLCS)
        rectangle, coordinate = create_rectangle_from_vertices(list_vertices_CVLN)

        return rectangle, coordinate

    elif isinstance(shape, Polygon):
        list_vertices_CVLN = convert_to_curvilinear_vertices(shape.vertices, CLCS)
        polygon, coordinate = create_polygon_from_vertices(list_vertices_CVLN)

        return polygon, coordinate
    else:
        return None, None


def convert_to_curvilinear_vertices(list_vertices_CART, CLCS):
    try:
        list_vertices_CVLN = [
            CLCS.convert_to_curvilinear_coords(vertex[0], vertex[1])
            for vertex in list_vertices_CART
        ]
    except ValueError:
        return []
    else:
        return list_vertices_CVLN


def create_rectangle_from_vertices(list_vertices):
    if not list_vertices:
        return None, None

    list_p_lon = [vertex[0] for vertex in list_vertices]
    list_p_lat = [vertex[1] for vertex in list_vertices]

    p_lon_min = min(list_p_lon)
    p_lat_min = min(list_p_lat)
    p_lon_max = max(list_p_lon)
    p_lat_max = max(list_p_lat)

    length = p_lon_max - p_lon_min
    width = p_lat_max - p_lat_min
    coordinate = np.array([(p_lon_min + p_lon_max) / 2.0, (p_lat_min + p_lat_max) / 2.0])

    return Rectangle(length=length, width=width), coordinate


def create_polygon_from_vertices(list_vertices):
    if not list_vertices:
        return None, None


def convert_to_curvilinear_shape_group(shape_group: ShapeGroup):
    return None


def convert_to_curvilinear_dynamic_obstacles(list_obstacles_dynamic_CART, CLCS):
    return []


def convert_to_CVLN_static_obstacle(obstacle, CLCS):
    list_obstacles_CVLN = []

    list_list_vertices_obstacles = []
    occupancy = obstacle.occupancy_at_time(obstacle.initial_state.time_step)

    if isinstance(occupancy.shape, ShapeGroup):
        for shape in occupancy.shape.shapes:
            if isinstance(shape, Polygon) or isinstance(shape, Rectangle):
                list_list_vertices_obstacles.append(shape.vertices)

    elif isinstance(occupancy.shape, Polygon) or isinstance(occupancy.shape, Rectangle):
        list_list_vertices_obstacles.append(occupancy.shape.vertices)

    for list_vertices in list_list_vertices_obstacles:
        try:
            list_vertices_CVLN = [
                CLCS.convert_to_curvilinear_coords(vertex[0], vertex[1])
                for vertex in list_vertices
            ]
        except ValueError:
            continue
        else:
            static_obstacle = create_static_obstacle_from_vertices(list_vertices_CVLN, obstacle)
            list_obstacles_CVLN.append(static_obstacle)

    return list_obstacles_CVLN


def create_static_obstacle_from_vertices(list_vertices, obstacle):
    p_lon_min = min(vertex[0] for vertex in list_vertices)
    p_lat_min = min(vertex[1] for vertex in list_vertices)
    p_lon_max = max(vertex[0] for vertex in list_vertices)
    p_lat_max = max(vertex[1] for vertex in list_vertices)

    length = p_lon_max - p_lon_min
    width = p_lat_max - p_lat_min
    coordinate = np.array(
        [(p_lon_min + p_lon_max) / 2.0, (p_lat_min + p_lat_max) / 2.0]
    )

    static_obstacle_id = obstacle.obstacle_id + random.randint(0, 100000000)
    static_obstacle_type = obstacle.obstacle_type
    static_obstacle_shape = Rectangle(length=length, width=width)
    static_obstacle_initial_state = State(
        position=coordinate, orientation=0.0, time_step=obstacle.initial_state.time_step
    )
    static_obstacle = StaticObstacle(
        static_obstacle_id,
        static_obstacle_type,
        static_obstacle_shape,
        static_obstacle_initial_state,
    )

    return static_obstacle


def create_scenario_for_curvilinear_collision_check(self, config: Configuration):
    """Returns a scenario with obstacles in Curvilinear coordinate system.

    Elements included: obstacles, road boundaries.
    The obstacles (vehicles and road boundaries) are converted into Curvilinear coordinate system.
    """
    scenario: Scenario = config.scenario
    scenario_cc = Scenario(scenario.dt, scenario.scenario_id)

    list_obstacles_CART = []

    # add obstacles
    if config.reachable_set.consider_traffic:
        list_obstacles_CART += config.scenario.obstacles

    # add road boundary
    object_road_boundary, _ = boundary.create_road_boundary_obstacle(
        scenario_cc, method="triangulation"
    )
    list_obstacles_CART.append(object_road_boundary)

    # convert obstacles into curvilinear coordinate system
    list_obstacles_CVLN = self.convert_obstacles_to_curvilinear_coordinate_system(
        list_obstacles_CART, config.planning.CLCS
    )

    scenario_cc.add_objects(list_obstacles_CVLN)

    return scenario_cc


def create_curvilinear_collision_checker2(
        self,
        scenario: Scenario,
        lanelet_network_vehicle: LaneletNetwork,
        bounding_boxes: dict,
        vehicle_radius: float,
        cosy: pycrccosy.CurvilinearCoordinateSystem,
        start_time: int,
        end_time: int,
        consider_traffic=True,
        reduce_distance_to_road_boundary=0) -> pycrcc.CollisionChecker:
    scenario_cc: Scenario = Scenario(scenario.dt, scenario.scenario_id)
    scenario_cc.add_objects(lanelet_network_vehicle)

    if consider_traffic:
        scenario_cc.add_objects(scenario.obstacles)

    # add road boundary
    polygons_static = list()
    _, sg_road_boundary = boundary.create_road_boundary_obstacle(
        scenario_cc, method="triangulation"
    )
    print(len(sg_road_boundary.unpack()))
    for r in sg_road_boundary.unpack():
        triangle = r.vertices()
        triangle.append(triangle[0])
        triangle.reverse()
        if self.validate_vertices2(np.array(triangle), cosy):
            polygons_static.append(np.array(triangle))
    print(len(polygons_static))

    # add static obstacles
    for static in scenario_cc.static_obstacles:
        initial_time_step = static.initial_state.time_step
        occ = static.occupancy_at_time(initial_time_step)
        if isinstance(occ.shape, ShapeGroup):
            for s in occ.shape.shapes:
                if self.validate_vertices2(s.vertices, cosy):
                    polygons_static.append(s.vertices)
        else:
            if self.validate_vertices2(occ.shape.vertices, cosy):
                polygons_static.append(occ.shape.vertices)

    # add dynamic obstacles
    polygons_dynamic = defaultdict(list)
    for k, t in enumerate(range(start_time, end_time + 1)):
        occupancies = scenario_cc.occupancies_at_time_step(t, ObstacleRole.DYNAMIC)
        dynamic_polygons = list()
        for occ in occupancies:
            if isinstance(occ.shape, ShapeGroup) and len(occ.shape.shapes) == 0:
                continue
            if isinstance(occ.shape, ShapeGroup):
                for s in occ.shape.shapes:
                    if self.validate_vertices2(occ.shape.vertices, cosy):
                        dynamic_polygons.append(s.vertices)
            elif isinstance(occ.shape, Circle):
                # over-approximate circle with rectangle
                dynamic_polygons.append(
                    Rectangle(2 * occ.shape.radius, 2 * occ.shape.radius, occ.shape.center).vertices)
            else:
                if self.validate_vertices2(occ.shape.vertices, cosy):
                    dynamic_polygons.append(occ.shape.vertices)

        polygons_dynamic[t] = dynamic_polygons

    static_sg, tvo = pycrreach.create_curvilinear_collision_objects(
        polygons_static, polygons_dynamic, bounding_boxes, cosy, vehicle_radius, reduce_distance_to_road_boundary,
        4)

    cc = pycrcc.CollisionChecker()
    cc.add_collision_object(static_sg)
    cc.add_collision_object(tvo)

    return cc


# dict_time_to_list_vertices_obstacles_dynamic = self.create_dict_of_list_of_vertices_of_dynamic_obstacles(
#     scenario, time_start, time_end)
#
# CLCS = self.config.planning.CLCS
# radius_disk = self.config.vehicle.radius_disk
# dict_time_to_bounding_box = self.create_bounding_box_from_projection_domain(time_start, time_end, CLCS)
# shape_group_obstacles_static, time_variant_objects = pycrreach.create_curvilinear_collision_objects(
#     list_vertices_obstacles_static, dict_time_to_list_vertices_obstacles_dynamic, dict_time_to_bounding_box,
#     CLCS,
#     radius_disk,
#     0, 4)
#
# return dict_time_to_list_vertices_obstacles_dynamic

def create_list_of_vertices_of_static_obstacles(self, scenario_cc: Scenario, CLCS):
    list_vertices_obstacles_static = []

    # add vertices of triangles of road boundary
    object_road_boundary, _ = boundary.create_road_boundary_obstacle(
        scenario_cc, method="triangulation"
    )
    for triangle in object_road_boundary.occupancy_at_time(0).shape.shapes:
        if self.validate_vertices2(triangle.vertices, CLCS):
            list_vertices_obstacles_static.append(triangle.vertices)

    # add vertices of static obstacles
    for obstacle in scenario_cc.static_obstacles:
        time_step_initial = obstacle.initial_state.time_step
        occ = obstacle.occupancy_at_time(time_step_initial)

        if isinstance(occ.shape, ShapeGroup):
            for shape in occ.shape.shapes:
                list_vertices_obstacles_static.append(shape.vertices)

        elif isinstance(occ.shape, Rectangle) or isinstance(occ.shape, Polygon):
            list_vertices_obstacles_static.append(occ.shape.vertices)

    return list_vertices_obstacles_static


@staticmethod
def create_dict_of_list_of_vertices_of_dynamic_obstacles(scenario_cc: Scenario, time_start, time_end):
    dict_time_to_list_vertices_obstacles_dynamic = defaultdict(list)
    # add vertices of dynamic obstacles (vertices of different time steps are kept separately)

    for time_step in range(time_start, time_end):
        list_vertices_obstacles_dynamic = []
        occupancies = scenario_cc.occupancies_at_time_step(time_step, ObstacleRole.DYNAMIC)

        for occ in occupancies:
            if isinstance(occ.shape, ShapeGroup) and len(occ.shape.shapes) == 0:
                continue

            elif isinstance(occ.shape, ShapeGroup):
                for shape in occ.shape.shapes:
                    list_vertices_obstacles_dynamic.append(shape.vertices)

            elif isinstance(occ.shape, Circle):
                # over-approximate circles with rectangles
                list_vertices_obstacles_dynamic.append(
                    Rectangle(2 * occ.shape.radius, 2 * occ.shape.radius, occ.shape.center).vertices)
            else:
                list_vertices_obstacles_dynamic.append(occ.shape.vertices)

        dict_time_to_list_vertices_obstacles_dynamic[time_step] = list_vertices_obstacles_dynamic

    return dict_time_to_list_vertices_obstacles_dynamic


def create_bounding_box_from_projection_domain(time_start: int, time_end: int, CLCS):
    dict_time_to_bounding_box = defaultdict(list)

    for time_step in range(time_start, time_end):
        bounding_box_CART = CLCS.projection_domain()
        bounding_box_CART = [list(np.array(bounding_box_CART))]
        dict_time_to_bounding_box[time_step] = bounding_box_CART

    return dict_time_to_bounding_box

def validate_vertices2(list_vertices, cosy):
    try:
        s_min = np.infty
        s_max = -np.infty
        d_min = np.infty
        d_max = -np.infty

        for vertex in list_vertices:
            s, d = cosy.convert_to_curvilinear_coords(vertex[0], vertex[1])
            s_min = min(s_min, s)
            s_max = max(s_max, s)
            d_min = min(d_min, d)
            d_max = max(d_max, d)

    except ValueError:
        return False
    else:
        if s_min < 10 or s_max > 90:
            return False

        return True


def overapproximate_reachable_set(config, dt, time_steps):
    bounding_boxes = defaultdict(list)
    for i in range(0, time_steps + 1):
        cartesian_bb = config.planning.CLCS.projection_domain()
        cartesian_bb = [list(np.array(cartesian_bb))]

        bounding_boxes[i + 0] = cartesian_bb
    return bounding_boxes