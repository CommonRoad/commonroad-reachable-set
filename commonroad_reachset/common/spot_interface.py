import copy

import matplotlib.pyplot as plt
import numpy as np
import spot
from commonroad.geometry.shape import Rectangle, Polygon, ShapeGroup
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from commonroad_reachset.common.data_structure.configuration import Configuration


class SPOTInterface:
    """Interface to SPOT prediction."""

    def __init__(self, config: Configuration):
        """Steps: configuration, register, update, predict, postprocess, remove"""
        # ================    settings
        self.scenario = copy.deepcopy(config.scenario)
        self.planning_problem = config.planning_problem

        self.dt = config.planning.dt
        self.step_t = int(self.dt * 10)
        self.time_start = config.planning.time_step_start
        self.time_horizon = config.planning.time_steps_computation + config.sonia.time_steps_computation_extra
        self.time_end = self.time_start + self.time_horizon

        self.spot_scenario_id = 1
        self.field_of_view = np.empty([0, 2], float)
        self.num_threads = config.sonia.num_threads

        # list for storing the prediction result
        self.list_structs_obstacles_predicted = None
        # whether you want to print the velocity intervals of obstacles predicted by SPOT
        self.print_predicted_velocity_intervals = config.sonia.print_predicted_velocity_intervals
        # whether you want to print some computation info from SPOT
        self.print_operation_status = config.sonia.print_operation_status

        # ================    register scenario for spot prediction
        self.register_scenario()

        # ================    update prediction parameters
        # parameters for SPOT prediction
        update_dict = {
            "obstacles": {
                0: {  # 0 means that all obstacles will be affected
                    "a_max": config.vehicle.other.a_lon_max,
                    "v_max": config.vehicle.other.v_lon_max,
                    "compute_occ_m1": config.sonia.compute_assumption_m1,
                    "compute_occ_m2": config.sonia.compute_assumption_m2,
                    "compute_occ_m3": config.sonia.compute_assumption_m3,
                    "onlyInLane": True,
                    "constr_no_lane_change": False
                }
            },
            "EgoVehicle": {
                0: {  # ID is ignored for ego vehicle
                    "a_max": config.vehicle.ego.a_lon_max,
                    "length": config.vehicle.ego.length,
                    "width": config.vehicle.ego.width
                }
            }
        }
        # update parameters for spot
        self.update_parameters(update_dict)

    def register_scenario(self):
        """Registers scenario to SPOT."""
        status_register = spot.registerScenario(self.spot_scenario_id,
                                                self.scenario.lanelet_network,
                                                self.scenario.dynamic_obstacles,
                                                [self.planning_problem],
                                                self.field_of_view)
        if self.print_operation_status:
            print('Register scenario finished with status:', status_register)

    def update_parameters(self, update_dict):
        """Updates parameters."""
        status_update = spot.updateProperties(self.spot_scenario_id, update_dict)
        if self.print_operation_status:
            print('Update properties finished with status:', status_update)

    def predict_occupancies(self):
        """Predicts occupancies of registered obstacles."""
        if self.print_operation_status:
            print('Calling SPOT prediction with ', self.num_threads, 'threads and for time interval [',
                  self.time_start, ', ', self.time_end, '] with step size of ', self.step_t)

        self.list_structs_obstacles_predicted = spot.doOccupancyPrediction(self.spot_scenario_id,
                                                                           self.time_start * self.dt,
                                                                           self.step_t * self.dt,
                                                                           (self.time_end + self.step_t) * self.dt,
                                                                           self.num_threads)

    def postprocess_prediction(self):
        """Post process the SPOT prediction.

        A dictionary containing the desired information is created.
        Converts prediction into dynamic obstacles in commonroad scenario.
        """
        assert self.list_structs_obstacles_predicted is not None, "<SPOTInterface> SPOT prediction not called."

        if self.print_operation_status:
            print('Number of predicted obstacles:', len(self.list_structs_obstacles_predicted))

        k = 0  # iterator over dynamic_obstacles
        prediction_obstacles_id_begin = 50000000  # to avoid non-unique ids in scenario
        dict_final_output = {}  # dictionary to store prediction output

        for struct_obstacle in self.list_structs_obstacles_predicted:
            id_obstacle = struct_obstacle[0]
            list_structs_polygons = struct_obstacle[1]
            # list of predicted occupancies of the obstacle
            list_occupancies = []
            list_velocities = []

            for t in range(self.time_start, self.time_end + 1, self.step_t):
                # create an Occupancy at time (i+1) with an empty ShapeGroup
                occ = Occupancy(t, ShapeGroup([]))
                list_occupancies.append([t, occ])
                list_velocities.append([t, []])

            # all occupancy polygons (list_structs_obstacle[1]) are stored in one list; thus, we need to separate each polygon
            t = 0  # iterator over list_occupancies
            for struct_polygon_at_time_step in list_structs_polygons:
                # hint: struct_polygon_at_time_step[0][lane_idx][1,2] := velocities (min, max)
                # hint: struct_polygon_at_time_step[1] := vertices

                # print('Occupancy for time step :',i)
                # print('Size of vertices: ', len(struct_polygon_at_time_step[1]))
                # print(struct_polygon_at_time_step[1])

                j = 1  # iterator over struct_polygon_at_time_step
                b = 0  # index to select struct_polygon_at_time_step that are the start of a new polygon
                while j < len(struct_polygon_at_time_step[1]):
                    compare_vertices = struct_polygon_at_time_step[1][b]  # first vertices of next polygon
                    if compare_vertices[0] == struct_polygon_at_time_step[1][j][0] and compare_vertices[1] == \
                            struct_polygon_at_time_step[1][j][1]:
                        if (j + 1) - b < 3:
                            # polygon has less than 3 vertices
                            # try next vertex as the first vertex (in case of equal vertices directly after each other)
                            b += 1
                        else:
                            # polygon has at least 3 vertices
                            shape_obj = Polygon(np.array(struct_polygon_at_time_step[1][b:j + 1]))
                            # update list
                            list_occupancies[t][1].shape.shapes.append(shape_obj)
                            j += 1
                            b = j
                    j += 1
                assert b == j - 1, ('Last polygon not closed (at time_step = ', t, ', b = ', b)

                # obtain min/max velocities
                lane_idx = 0  # velocity is yet the same for all lanes
                v_min = struct_polygon_at_time_step[0][lane_idx][1]
                v_max = struct_polygon_at_time_step[0][lane_idx][2]

                list_velocities[t][1].append(v_min)
                list_velocities[t][1].append(v_max)
                # extract predicted velocity interval (not yet supported by commonroad)
                if self.print_predicted_velocity_intervals:
                    print("predicted velocity interval of obstacle with id =", id_obstacle,
                          "at time_step =", self.time_start + t, ": [", v_min, ",", v_max, "]")

                t += 1

            dict_final_output[id_obstacle] = {'occupancy_predicted': list_occupancies,
                                              'velocity_predicted': list_velocities}

            list_occ = []
            for element in list_occupancies:
                list_occ.append(element[1])

            # create new cr_obstacle if needed
            if k >= len(self.scenario.dynamic_obstacles):
                self.scenario.add_objects(DynamicObstacle(k + 1 + prediction_obstacles_id_begin, ObstacleType.UNKNOWN,
                                                          Rectangle(0, 0),
                                                          State(position=np.array([0, 0]), orientation=0, time_step=0)))

            # spot prediction is attached to dynamic obstacles in the scenario
            self.scenario.dynamic_obstacles[k].prediction = SetBasedPrediction(self.spot_scenario_id, list_occ)
            k += 1

        return dict_final_output

    def deregister_scenario(self):
        """Deregisters the previously registered scenario."""
        status_remove = spot.removeScenario(self.spot_scenario_id)
        if self.print_operation_status:
            print('Remove Spot Scenario finished with status:', status_remove)

    def plot(self, time_steps, xlim=None, ylim=None):

        for time_step in time_steps:
            fig, ax = plt.subplots(1, figsize=(20, 15))

            # plot everything
            # draw_object(self.scenario, draw_params={'time_begin': time_step})

            # plot network
            draw_object(self.scenario.lanelet_network, draw_params={'time_begin': time_step})

            # plot dynamic obstacles' occupancy
            for o in self.scenario.dynamic_obstacles:
                draw_object(o.occupancy_at_time(time_step), draw_params={'time_begin': time_step})

            ax.set_xlim([-10, 150] if not xlim else xlim)
            ax.set_ylim([-20, 15] if not ylim else ylim)
            ax.autoscale_view()
            # plt.axis('off')
            plt.gca().set_aspect('equal')
            plt.show()
