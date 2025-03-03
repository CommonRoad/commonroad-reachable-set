import glob
import logging
import os
from typing import Union

from omegaconf import OmegaConf, ListConfig, DictConfig

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration import Configuration

logger = logging.getLogger("REACH_LOGGER")


class ConfigurationBuilder:
    """Class to build a configuration."""

    path_root: str
    path_config: str
    path_config_default: str
    config_default: Union[ListConfig, DictConfig]

    def __init__(self, path_root: str = None, dir_config: str = "configurations",
                 dir_config_default: str = "defaults") -> None:
        """Initialize and configure the paths of a configuration builder.

        :param path_root: root path of the package (all paths are relative to this directory)
        :param dir_config: directory storing configurations (relative to path_root)
        :param dir_config_default: directory storing default configurations (relative to path_root/dir_config)
        """
        if path_root is None:
            path_root = os.path.abspath(os.getcwd())
        self.set_paths(path_root=path_root, dir_config=dir_config, dir_config_default=dir_config_default)

    def build_configuration(self, name_scenario: str) -> Configuration:
        """Builds configuration from default, scenario-specific, and commandline config files.

        :param name_scenario: name of the considered scenario
        :return: built configuration
        """

        config_scenario = self.construct_scenario_configuration(name_scenario)
        config_cli = OmegaConf.from_cli()
        # configurations coming after overrides the ones coming before
        config_merged = OmegaConf.merge(self.config_default, config_scenario, config_cli)
        config = Configuration(config_merged)

        return config

    def set_paths(self, path_root: str, dir_config: str, dir_config_default: str):
        """Sets necessary paths of the configuration builder.

        :param path_root: root directory
        :param dir_config: directory storing configurations
        :param dir_config_default: directory storing default configurations
        """
        # set root path (if directory exists)
        if os.path.isdir(path_root):
            self.path_root = path_root
        else:
            raise FileNotFoundError(f'Provided path_root ({path_root}) does not exist!')

        # set path to configurations directory
        self.path_config = os.path.join(path_root, dir_config)

        if not os.path.isdir(self.path_config):
            # set path config default to configurations_default in Python Package
            self.path_config_default = self._get_default_config_path()
        else:
            self.path_config_default = os.path.join(self.path_config, dir_config_default)
            if not os.path.isdir(self.path_config_default):
                # set path config default to configurations_default in Python Package
                self.path_config_default = self._get_default_config_path()

        msg = "\n# ===== CommonRoad-Reach Configuration Builder ===== #\n"
        msg += f"# Root path: \n"
        msg += f"# \t{self.path_root}\n"
        msg += f"# Using default configs from: \n"
        msg += f"# \t{self.path_config_default}"
        util_logger.print_and_log_info(logger, msg)

        # Update cached default configuration
        self.config_default = self.construct_default_configuration()

    def construct_default_configuration(self) -> Union[ListConfig, DictConfig]:
        """Constructs default configuration by accumulating yaml files.

        Collects all configuration files ending with '.yaml' in the directory storing default configurations.
        """
        config_default = OmegaConf.create()
        for path_file in glob.glob(self.path_config_default + "/*.yaml"):
            with open(path_file, "r") as file_config:
                try:
                    config_partial = OmegaConf.load(file_config)
                    name_file = path_file.split("/")[-1].split(".")[0]

                except Exception as e:
                    print(e)

                else:
                    config_default[name_file] = config_partial

        config_default = self.convert_to_absolute_paths(config_default)

        return config_default

    def convert_to_absolute_paths(self, config_default: Union[ListConfig, DictConfig]) -> Union[ListConfig, DictConfig]:
        """Converts relative paths to absolute paths."""
        for key, path in config_default["general"].items():
            path_relative = os.path.join(self.path_root, path)
            if os.path.exists(path_relative):
                config_default["general"][key] = path_relative

        return config_default

    def construct_scenario_configuration(self, name_scenario: str) -> Union[DictConfig, ListConfig]:
        """Constructs scenario-specific configuration."""
        config_scenario = OmegaConf.create()

        msg = ""
        path_config_scenario = self.path_config + f"/{name_scenario}.yaml"
        if os.path.exists(path_config_scenario):
            with open(path_config_scenario, "r") as file_config:
                try:
                    config_scenario = OmegaConf.load(file_config)

                except Exception as e:
                    print(e)
            msg += f"# Using scenario config from: \n"
            msg += f"# \t {path_config_scenario}"

        util_logger.print_and_log_info(logger, msg)

        # add scenario name to the config file
        config_scenario["general"] = {"name_scenario": name_scenario}

        return config_scenario

    @staticmethod
    def _get_default_config_path():
        """Returns path to default configurations in Python package."""
        return os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../configurations_default/"))
