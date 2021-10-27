import os
import copy
import glob
from typing import Dict
from collections import defaultdict

import yaml
from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.utility import general as util_general


class ConfigurationBuilder:
    path_root: str = None
    path_config: str = None
    path_config_default: str = None
    dict_config_overridden: dict = None

    @classmethod
    def set_root_path(cls, path_root: str,
                      path_to_config: str = "configurations/", dir_configs_default: str = "defaults"):
        """Sets the path to the root directory.

        Args:
            path_root (str): root directory
            path_to_config (str): relative path of configurations to root path
            dir_configs_default (str): directory under root folder containing default config files.
        """
        cls.path_root = path_root
        cls.path_config = os.path.join(path_root, path_to_config)
        cls.path_config_default = os.path.join(cls.path_config, dir_configs_default)

    @classmethod
    def set_path_to_config(cls, path_to_config: str, dir_configs_default: str = "defaults"):
        """Sets the path to configuration.

        Args:
            path_to_config (str): relative path of configurations to root path
            dir_configs_default (str): directory under root folder containing default config files.
        """
        cls.path_config = path_to_config
        cls.path_root = os.path.join(path_to_config, "../../")
        cls.path_config_default = os.path.join(cls.path_config, dir_configs_default)

    @classmethod
    def build_configuration(cls, name_scenario: str, idx_planning_problem: int = -1) -> Configuration:
        """Builds configuration from default and scenario-specific config files.

        Steps:
            1. (Optional) Set root path of the configuration builder
            2. Load default config files
            3. Override if scenario-specific config file exists
            4. Build Configuration object
            5. Load scenario and planning problems
            6. Complete configuration with scenario and planning problem

        Args:
            name_scenario (str): considered scenario
            idx_planning_problem (int, optional): index of the planning problem. Defaults to 0.
        """
        if not cls.path_root:
            cls.set_root_path(os.path.join(os.getcwd(), ".."))

        dict_config_default = cls.construct_default_config_dict()

        cls.dict_config_overridden = cls.override_with_scenario_config(dict_config_default, name_scenario)

        # convert to default dict
        cls.dict_config_overridden = defaultdict(lambda: None, cls.dict_config_overridden)

        config = Configuration(cls.dict_config_overridden)

        scenario, planning_problem = util_general.load_scenario_and_planning_problem(config, idx_planning_problem)

        config.complete_configuration(scenario, planning_problem)

        return config

    @classmethod
    def construct_default_config_dict(cls) -> Dict:
        """Construct default config dictionary with partial config files.

        Collects all config files ending with 'yaml* under path_config_default.
        """
        dict_config_default = dict()
        for path_file in glob.glob(cls.path_config_default + "/*.yaml"):
            with open(path_file, "r") as file_config:
                try:
                    dict_config = yaml.load(file_config, Loader=yaml.Loader)
                except yaml.YAMLError as e:
                    print(e)
                else:
                    dict_config_default = {**dict_config_default, **dict_config}

        cls.rectify_directories(dict_config_default)

        return dict_config_default

    @classmethod
    def rectify_directories(cls, dict_config):
        """Rectifies directories (converts from relative path to absolute path). """
        for key, path in dict_config["config_general"].items():
            path_relative = os.path.join(cls.path_root, path)
            if os.path.exists(path_relative):
                dict_config["config_general"][key] = path_relative
            else:
                continue

    @classmethod
    def override_with_scenario_config(cls, dict_config_default: Dict, name_scenario: str) -> Dict:
        """Overrides default config file with scenario-specific config files.

        Args:
            dict_config_default (Dict): dictionary created from default config files
            name_scenario (str): considered scenario
        """
        dict_config_overridden = copy.deepcopy(dict_config_default)

        path_config_scenario = cls.path_config + f"/{name_scenario}.yaml"
        if os.path.exists(path_config_scenario):
            with open(path_config_scenario, "r") as file_config:
                try:
                    dict_config_scenario = yaml.load(file_config, Loader=yaml.Loader)
                except yaml.YAMLError as e:
                    print(e)
                else:
                    cls.override_nested_dicts(dict_config_overridden, dict_config_scenario)

        # add scenario name to the config file
        dict_config_overridden["config_general"]["name_scenario"] = name_scenario

        return dict_config_overridden

    @classmethod
    def override_nested_dicts(cls, dict1: Dict, dict2: Dict) -> Dict:
        """Recursively overrides a dictionary with another dictionary.

        Args:
            dict1 (Dict): dictionary to be overridden
            dict2 (Dict): dictionary to provide new values
        """
        for key, val in dict2.items():
            if isinstance(val, Dict):
                dict1[key] = cls.override_nested_dicts(dict1.get(key, {}), val)
            else:
                dict1[key] = val

        return dict1
