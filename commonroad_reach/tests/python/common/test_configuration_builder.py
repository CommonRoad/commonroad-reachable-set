def test_config_dictionary_has_correct_keys(dict_config):
    assert (
            "config_debug" in dict_config
            and "config_general" in dict_config
            and "config_planning" in dict_config
            and "config_reachable_set" in dict_config
            and "config_spot" in dict_config
            and "config_vehicle" in dict_config
    )


def test_config_dictionary_has_correct_values(dict_config):
    assert dict_config["config_planning"]["time_steps_computation"] == 30
    assert dict_config["config_vehicle"]["ego"]["a_lon_min"] == -6.0
    assert dict_config["config_general"]["name_scenario"] == "DEU_IV21-1_1_T-1"


def test_scenario_config_overrides_default_config(dict_config):
    assert dict_config["config_planning"]["v_lon_desired"] == 15.5
    assert dict_config["config_vehicle"]["ego"]["v_lat_min"] == -4.0
    assert dict_config["config_vehicle"]["ego"]["v_lon_max"] == 33.3


def test_configuration_has_correct_values(config):
    assert config.planning.v_lon_desired == 15.5
    assert config.vehicle.ego.v_lon_min == 0.0
    assert config.vehicle.ego.v_lon_max == 33.3
    assert config.name_scenario == "DEU_IV21-1_1_T-1"
