import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt
import numpy as np


def main():
    # ==== specify scenario
    # name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ZAM_Over-1_1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "USA_US101-6_1_T-1"
    name_scenario = "ZAM_Intersection-1_1_T-1"

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(name_scenario)
    config.update_configuration()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)

    rnd = MPRenderer(figsize=(20, 10))
    cc = reach_interface._reach._reach.collision_checker
    cc.draw(rnd)
    rnd.render()
    # curv_projection_domain_border = np.asarray(config.planning.CLCS.curvilinear_projection_domain())
    # rnd.ax.plot(curv_projection_domain_border[:, 0], curv_projection_domain_border[:, 1], zorder=100, color='orange')
    plt.show()

    rnd1 = MPRenderer(figsize=(20, 10))
    config.scenario.draw(rnd1, draw_params={"time_begin": 0})
    rnd1.render()
    rnd1.ax.plot(config.planning.reference_path[:, 0], config.planning.reference_path[:, 1],
                 color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

    proj_domain_border = np.asarray(config.planning.CLCS.projection_domain())
    rnd1.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], zorder=100, color='orange')
    plt.show()


if __name__ == "__main__":
    main()
