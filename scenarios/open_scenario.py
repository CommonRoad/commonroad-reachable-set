import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_dc.boundary import boundary

# load the CommonRoad scenario that has been created in the CommonRoad tutorial
file_path = os.path.join(os.getcwd(), 'ZAM_Tutorial-1_2_T-1.xml')

scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

# plot the scenario for each time step
plt.figure(figsize=(25, 10))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'time_begin': 0, 'lanelet': {'show_label': False}, 'dynamic_obstacle': {'show_label': True}})
planning_problem_set.draw(rnd)
rnd.render()
plt.show()

# create road boundary
road_boundary_obstacle, road_boundary_sg_rectangles = boundary.create_road_boundary_obstacle(scenario)

rnd = MPRenderer(figsize=(25, 10))
road_boundary_sg_rectangles.draw(rnd)
rnd.render()
plt.show()
