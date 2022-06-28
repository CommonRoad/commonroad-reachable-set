from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.trajectory import State
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
import numpy as np
from matplotlib import pyplot as plt

from commonroad_reach.utility.configuration import compute_disc_radius_and_distance


# choose CR vehicle model
params = parameters_vehicle3()
reference_point = "REAR"

# center position and axle positions
init_pos = np.array([0.0, 0.0])
rear_axle_pos = init_pos - [params.b, 0]
front_axle_pos = init_pos + [params.a, 0]

# create static obstacle
static_obstacle_id = 1
static_obstacle_type = ObstacleType.PARKED_VEHICLE
static_obstacle_shape = Rectangle(width=params.w, length=params.l)
static_obstacle_initial_state = State(position=init_pos, orientation=0.0, time_step=0)
static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape, static_obstacle_initial_state)

# disc radius computation
rad, dist = compute_disc_radius_and_distance(params.l, params.w, ref_point=reference_point, dist_axle_rear=params.b)

if reference_point == "REAR":
    center1 = rear_axle_pos
    center2 = rear_axle_pos + [dist/2, 0]
    center3 = rear_axle_pos + [dist, 0]
elif reference_point == "CENTER":
    center1 = init_pos - [dist/2, 0]
    center2 = init_pos
    center3 = init_pos + [dist/2, 0]
else:
    raise Exception("Invalid reference point specified")

disc1 = Circle(rad, center1)
disc2 = Circle(rad, center2)
disc3 = Circle(rad, center3)


# set draw params
draw_params_circles = {"opacity": 0.5, "facecolor": "grey", "edgecolor": "grey"}

# plot
rnd = MPRenderer(figsize=(25, 10))
static_obstacle.draw(rnd)
disc1.draw(rnd, draw_params=draw_params_circles)
disc2.draw(rnd, draw_params=draw_params_circles)
disc3.draw(rnd, draw_params=draw_params_circles)
rnd.render()

rnd.ax.scatter([init_pos[0]], [init_pos[1]], color='k', marker='o', zorder=21)
rnd.ax.scatter([rear_axle_pos[0], front_axle_pos[0]], [rear_axle_pos[1], front_axle_pos[1]], color='k', marker='X', s=80, zorder=21)
rnd.ax.scatter([center1[0], center2[0], center3[0]], [center1[1], center2[1], center3[1]], color='grey', marker='o', zorder=21)
rnd.ax.scatter([init_pos[0]-params.l/3, init_pos[0], init_pos[0]+params.l/3], [0, 0, 0], color='blue', marker='o', zorder=21)

plt.show()
