import glob
import os
from typing import Dict, Union

from omegaconf import OmegaConf, ListConfig, DictConfig

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility import general as util_general


class ConfigurationBuilder:
    path_root: str = None
    path_config: str = None
    path_config_default: str = None

    @classmethod
    def build_configuration(cls, name_scenario: str, idx_planning_problem: int = -1) -> Configuration:
        """Builds configuration from default and scenario-specific config files.

        Steps:
            1. (Optional) Set paths of the configuration builder
            2. Load default configuration files
            3. Load scenario-specific configuration files
            4. Load configuration from command line interface
            5. Merge all configurations, create a config object
            6. Complete configuration with scenario and planning problem

        Args:
            name_scenario (str): considered scenario
            idx_planning_problem (int, optional): index of the planning problem. Defaults to 0.
        """
        if cls.path_root is None:
            cls.set_paths(path_root=os.path.join(os.getcwd(), ".."))

        # default configurations
        config_default = cls.construct_default_configuration()

        # scenario-specific configurations
        config_scenario = cls.construct_scenario_configuration(name_scenario)

        # command line interface configurations
        config_cli = OmegaConf.from_cli()

        config_combined = OmegaConf.merge(config_default, config_scenario, config_cli)
        config = Configuration(config_combined)

        scenario, planning_problem = util_general.load_scenario_and_planning_problem(config, idx_planning_problem)
        config.complete_configuration(scenario, planning_problem)

        return config

    @classmethod
    def set_paths(cls, path_root: str, path_to_config: str = "configurations", dir_configs_default: str = "defaults"):
        """Sets root path, path to configurations, and path to default configurations.

        Args:
            path_root (str): root directory
            path_to_config (str): relative path of configurations to root path
            dir_configs_default (str): directory under root folder containing default config files.
        """
        cls.path_root = path_root
        cls.path_config = os.path.join(path_root, path_to_config)
        cls.path_config_default = os.path.join(cls.path_config, dir_configs_default)

    @classmethod
    def construct_default_configuration(cls) -> Union[ListConfig, DictConfig]:
        """Constructs default configuration by accumulating yaml files.

        Collects all configuration files ending with '.yaml' under path_config_default.
        """
        config_default = OmegaConf.create()
        for path_file in glob.glob(cls.path_config_default + "/*.yaml"):
            with open(path_file, "r") as file_config:
                try:
                    config_partial = OmegaConf.load(file_config)
                    name_file = path_file.split("/")[-1].split(".")[0]

                except Exception as e:
                    print(e)

                else:
                    config_default[name_file] = config_partial

        config_default = cls.convert_to_absolute_paths(config_default)

        return config_default

    @classmethod
    def convert_to_absolute_paths(cls, config_default: Union[ListConfig, DictConfig]):
        """Converts paths from relative to absolute."""

        for key, path in config_default["general"].items():
            path_relative = os.path.join(cls.path_root, path)
            if os.path.exists(path_relative):
                config_default["general"][key] = path_relative

        return config_default

    @classmethod
    def construct_scenario_configuration(cls, name_scenario: str):
        """Constructs scenario-specific configuration."""
        config_scenario = OmegaConf.create()

        path_config_scenario = cls.path_config + f"/{name_scenario}.yaml"
        if os.path.exists(path_config_scenario):
            with open(path_config_scenario, "r") as file_config:
                try:
                    config_scenario = OmegaConf.load(file_config)

                except Exception as e:
                    print(e)

                else:
                    # add scenario name to the config file
                    config_scenario["general"] = {"name_scenario": name_scenario}

        return config_scenario
