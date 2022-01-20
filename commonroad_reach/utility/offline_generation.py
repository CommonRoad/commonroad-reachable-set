import os
import pickle
from collections import defaultdict

import numpy as np
from scipy import sparse

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_interface_offline import OfflineReachableSetInterface


def save_offline_computation(config: Configuration, reach_interface: OfflineReachableSetInterface):
    # check if the output path exists
    os.makedirs(config.general.path_offline_data, exist_ok=True)

    dict_data = dict()
    dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices = \
        extract_computation_information(reach_interface)

    dict_data["node_attributes"] = dict_time_to_list_tuples_reach_node_attributes
    dict_data["adjacency_matrices"] = dict_time_to_adjacency_matrices

    time_steps = config.planning.time_steps_computation
    size_grid = config.reachable_set.size_grid
    a_max = config.vehicle.ego.a_lon_max
    v_max = config.vehicle.ego.v_lon_max

    f = open(f"{config.general.path_offline_data}offline_{time_steps}_{size_grid}_{a_max}_{v_max}.pickle", 'wb')
    pickle.dump(dict_data, f)
    f.close()

    print("Offline computation result saved.")


def extract_computation_information(reach_interface: OfflineReachableSetInterface):
    dict_time_to_list_tuples_reach_node_attributes = defaultdict(list)
    dict_time_to_adjacency_matrices = dict()

    for time_step, list_nodes in reach_interface.dict_time_to_reachable_set.items():
        for node in list_nodes:
            tuple_attribute = (
                round(node.p_lon_min, 6), round(node.p_lat_min, 6), round(node.p_lon_max, 6), round(node.p_lat_max, 6),
                round(node.v_lon_min, 6), round(node.v_lat_min, 6), round(node.v_lon_max, 6), round(node.v_lat_max, 6))
            dict_time_to_list_tuples_reach_node_attributes[time_step].append(tuple_attribute)

        if time_step >= 1:
            list_nodes_parent = reach_interface.dict_time_to_reachable_set[time_step - 1]

            matrix_adjacency = list()
            for idx_node_child, node_child in enumerate(list_nodes):
                list_adjacency = [node_parent in node_child.list_nodes_parent for node_parent in list_nodes_parent]
                if not all(list_adjacency) and any(list_adjacency):
                    print(f"parent-child not adjacent: time_step: {time_step}, {idx_node_child}")
                    print(list_adjacency)
                matrix_adjacency.append(list_adjacency)

            matrix_adjacency_dense = np.array(matrix_adjacency)
            matrix_adjacency_sparse = sparse.csr_matrix(matrix_adjacency_dense, dtype=bool)
            dict_time_to_adjacency_matrices[time_step] = matrix_adjacency_sparse

    return dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices
