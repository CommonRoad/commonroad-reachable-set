import csv
import glob
import time
import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


def main():
    dict_result = {}
    counter_scenario = 0
    for path_scenario in glob.glob("../scenarios/runtime/*.xml"):
        name_scenario = path_scenario.split("/")[-1].split(".")[0]
        counter_scenario += 1

        config = ConfigurationBuilder.build_configuration(name_scenario)
        util_logger.initialize_logger(config)
        print(f"\tScenario number {counter_scenario}")
        config.print_configuration_summary()

        # ==== construct reachability interface and compute reachable sets
        reach_interface = ReachableSetInterface(config)
        time_computation = reach_interface.compute_reachable_sets()
        time_computation = round(time_computation, 3)

        dict_result[name_scenario] = [config.reachable_set.mode, time_computation]
        time.sleep(2)

    # save result as csv
    with open('computation_result.csv', 'w') as csvfile:
        writer_sheet = csv.writer(csvfile)
        writer_sheet.writerow(['scenario name', 'mode', 'computation time'])

        for name_scenario, result in dict_result.items():
            writer_sheet.writerow([name_scenario, result[0], result[1]])


if __name__ == "__main__":
    main()
