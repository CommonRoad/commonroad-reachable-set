import os
import time
from commonroad_reachset.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reachset.data_structure.reach.reachable_set_interface import PyReachableSetInterface
from commonroad_reachset.utility import visualization as util_visualization


def main():
    # ==== Entry point of reachable set computation
    # specify the name of scenario
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    # build configuration
    ConfigurationBuilder.set_root_path(os.path.join(os.getcwd(), ".."))
    config = ConfigurationBuilder.build_configuration(name_scenario)

    # ==== construct reachability interface and compute reachable sets
    reach_interface = PyReachableSetInterface(config)
    print("Computing...")
    time_start = time.time()
    reach_interface.compute()
    print("\tReachable sets computed.")
    print(f"Computation time: {time.time() - time_start:.3f}s")

    # save computation results
    util_visualization.draw_scenario_with_reachable_sets(config, reach_interface, as_svg=False)


if __name__ == "__main__":
    main()
