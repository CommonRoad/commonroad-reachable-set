import os
import glob
from typing import Union

from omegaconf import OmegaConf, ListConfig, DictConfig
from commonroad_reach.data_structure.configuration import Configuration


class ConfigurationBuilder:
    """
    Class to build a configuration.
    """
    path_root: str = None
    path_config: str = None
    path_config_default: str = None

    @classmethod
    def build_configuration(cls, name_scenario: str, path_root: str = None,
                            dir_config: str = "configurations", dir_config_default: str = "defaults") -> Configuration:
        """
        Builds configuration from default, scenario-specific, and commandline config files.

        :param name_scenario: name of the considered scenario
        :param path_root: root path of the package
        :param dir_config: directory storing configurations
        :param dir_config_default: directory storing default configurations
        :return: built configuration
        """
        if path_root is None:
            try:
                path_root = os.path.normpath(os.path.join(os.path.dirname(__file__), "../.."))
            except:
                raise FileNotFoundError(f'The root directory of commonroad-reach was not found. We expect a directory that contains the follolwing directories from the gitlab repo of commonroad-reach: configurations, scenarios')

        if cls.path_root is None:
            cls.set_paths(path_root=path_root, dir_config=dir_config, dir_config_default=dir_config_default)
            
        
        if('configurations' not in os.listdir(path_root)):
            raise FileNotFoundError(f'root_path must contain a configurations subdir')
        if('scenarios' not in os.listdir(path_root)):
            raise FileNotFoundError(f'root_path must contain a scenarios subdir')
        

        config_default = cls.construct_default_configuration()
        config_scenario = cls.construct_scenario_configuration(name_scenario)
        config_cli = OmegaConf.from_cli()
        # configurations coming after overrides the ones coming before
        config_merged = OmegaConf.merge(config_default, config_scenario, config_cli)
        config = Configuration(config_merged)

        return config

    @classmethod
    def set_paths(cls, path_root: str, dir_config: str, dir_config_default: str):
        """
        Sets necessary paths of the configuration builder.

        :param path_root: root directory
        :param dir_config: directory storing configurations
        :param dir_config_default: directory storing default configurations
        """
        cls.path_root = path_root
        cls.path_config = os.path.join(path_root, dir_config)
        cls.path_config_default = os.path.join(cls.path_config, dir_config_default)

    @classmethod
    def construct_default_configuration(cls) -> Union[ListConfig, DictConfig]:
        """
        Constructs default configuration by accumulating yaml files.

        Collects all configuration files ending with '.yaml' in the directory storing default configurations.
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
    def convert_to_absolute_paths(cls, config_default: Union[ListConfig, DictConfig]) -> Union[ListConfig, DictConfig]:
        """
        Converts relative paths to absolute paths.
        """
        for key, path in config_default["general"].items():
            path_relative = os.path.join(cls.path_root, path)
            if os.path.exists(path_relative):
                config_default["general"][key] = path_relative

        return config_default

    @classmethod
    def construct_scenario_configuration(cls, name_scenario: str) -> Union[DictConfig, ListConfig]:
        """
        Constructs scenario-specific configuration.

        """
        config_scenario = OmegaConf.create()

        path_config_scenario = cls.path_config + f"/{name_scenario}.yaml"
        if os.path.exists(path_config_scenario):
            with open(path_config_scenario, "r") as file_config:
                try:
                    config_scenario = OmegaConf.load(file_config)

                except Exception as e:
                    print(e)

        # add scenario name to the config file
        config_scenario["general"] = {"name_scenario": name_scenario}

        return config_scenario
