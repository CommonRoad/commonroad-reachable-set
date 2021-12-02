import pickle

import commonroad_dc.pycrcc as pycrcc
import cv2
import scipy.sparse as sparse
from scipy.sparse import csr_matrix, csc_matrix
import os

from pycrreach import Polyhedron2VRep, AARectDouble

from pruningPy.graph_reach.offline_graph_reach import ReachSetGridOffline
import math
from commonroad.scenario.trajectory import State
from commonroad.common.util import Interval
from commonroad.scenario.scenario import Scenario
from typing import Dict, Tuple, Optional
from pruningPy.graph_reach.utils import *
from pruningPy.graph_reach.collision_object_grid import ObstacleRegularGrid

from commonroad.common.validity import *
import json
USE_COLLISION = True


class GridNode():
    def __init__(self, time_idx: int, reach_set_ids: List[int], parents: List[ReachSetNode], children: List[ReachSetNode],
                 reachsetnode_list_prev, reachsetnode_list_next, node_grid_coordinate_list, grandchildren: Dict[int, List[int]] = {}, grandparents: Dict[int,List[int]] = {}):
        """
        :param reach_set_ids: ids referring to corresponding list element of reachboxes_list
        :param parents:
        :param childs:
        """
        self.time_idx = time_idx
        self.reach_set_ids = list(set(reach_set_ids))

        self.parents = [get_id_for_reachsetnode(parent, reachsetnode_list_prev) for parent in parents]
        self.children: List = [get_id_for_reachsetnode(child, reachsetnode_list_next) for child in children]
        self.parent_coords = []
        self.velocities_minmax_children_reachable: Dict[int, List[Interval]] = dict() # assigns to every child_id the velocity interval at this node's time step which makes the child reachable
        self.velocities_minmax_children_reached: Dict[int, List[Interval]] = dict()   # assigns to every child_id the reachable velocity interval at the child's time step

        if reachsetnode_list_prev:
            self.parent_coords = list(set([node_grid_coordinate_list[self.time_idx-1][id_parent] for id_parent in self.parents]))

        self.children_coords = []
        if reachsetnode_list_next:
            self.children_coords = list(
                set([node_grid_coordinate_list[self.time_idx + 1][id_children] for id_children in self.children]))

        # grandparent/children
        self.grandchildren = grandchildren
        self.grandparents = grandparents

        self.grandchildren_coords= defaultdict(list)
        self.grandparents_coords = defaultdict(list)

        for delta_timesteps, grandchild_ids in grandchildren.items():
            # try:
            #     for id_grandchildren in grandchild_ids:
            #         node_grid_coordinate_list[self.time_idx + delta_timesteps + 1][id_grandchildren]
            # except:
            #     iii=1
            self.grandchildren_coords[delta_timesteps] = list(set(
                [node_grid_coordinate_list[self.time_idx + delta_timesteps + 1][id_grandchildren] for id_grandchildren
                 in grandchild_ids]))

        for delta_timesteps, grandparent_ids in grandparents.items():
            self.grandparents_coords[delta_timesteps] = list(set([node_grid_coordinate_list[self.time_idx - delta_timesteps][id_grandparents] for id_grandparents in grandparent_ids]))

        # for online RA
        self.interval_updated = False
        self.reachable_children_coordinates = list()
        self.reached_velocity_intervals = [Interval(0,0), Interval(0,0)] # reached velocities [vx, vy]

    def reachable_children_for_velocity_intervals(self):
        """
        Compute children coordinates that can be reached from this node depending on the velocity interval at this node.
        !!NOTE: update_reached_velocity_intervals had to be executed first, with the reached veloty intervals from each parent that reaches this node
        :return:
        """
        assert self.interval_updated

        reachable_children_coordinates = list()
        reachable_children_intervals = list()

        for i_child, child_id in enumerate(self.children):
            velocity_intervals_reachable = self.velocities_minmax_children_reachable[child_id]
            child_reachable = True
            # append child if vx- and vy are overlap
            for i_state, interval_reachable in enumerate(velocity_intervals_reachable):
                interval_current = self.reached_velocity_intervals[i_state]
                if not interval_reachable.overlaps(interval_current):
                    child_reachable = False
                    break
                else:
                    # print('<protype>vel not reached')
                    iii=1
            if child_reachable:
                reachable_children_coordinates.append(self.children_coords[i_child])
                reachable_children_intervals.append(self.velocities_minmax_children_reached[child_id])

        self.reachable_children_coordinates = reachable_children_coordinates
        return reachable_children_coordinates, reachable_children_intervals

    def update_reached_velocity_intervals(self, new_intervals: List[Interval]):
        """
        Update velocity interval that can be reached at this node, depending on its parents ( reached_velocity_intervals = union(reachable_velocities(parents)))
        :param new_intervals: reached velocity interval from parent
        :return:
        """
        self.interval_updated = True
        for i_state, vel_interval in enumerate(new_intervals):
            if vel_interval.start < self.reached_velocity_intervals[i_state].start:
                self.reached_velocity_intervals[i_state].start = vel_interval.start
            if vel_interval.end > self.reached_velocity_intervals[i_state].end:
                self.reached_velocity_intervals[i_state].end = vel_interval.end

def get_id_for_reachsetnode(reachsetnode, reachsetnode_list):
    for i, node in enumerate(reachsetnode_list):
        if node == reachsetnode:
            return i


def get_grid_coords_for_node(id_node, list_of_coords):
    """

    :param id_node:
    :param list_of_coords: id->
    :return:
    """

class ReachSetGridOnline():
    def __init__(self, reach_set: Optional[ReachSetGridOffline] = None, velocity_active=True, sparse=False, debug=True, velocity=False, n_threads=-1):
        """

        :param reach_set:
        :param velocity_active:
        :param sparse:
        :param debug:
        :param velocity:
        :param n_threads: number of threads for discretization
        """
        if reach_set is None:
            # Has to be initialized with self.init_from_file
            self._reachability_matrix_offline_list = None
            self._reachability_matrix_offline_grandchildren_dict = None
            self.scenario_id = None
            return

        # set thread number (max. 8 if set to default number)
        if n_threads >= 0:
            cv2.setNumThreads(n_threads)
        elif n_threads < 0 and cv2.getNumThreads() > 8:
            cv2.setNumThreads(8)

        self.sparse = sparse
        self.velocity_active = velocity_active
        if self.velocity_active:
            self.velocities_minmax_list = reach_set.reached_velocities_minmax_list

        self.dt = reach_set.configuration['d_t']

        self.scenario_id = None
        # constants dt^2 * a_max / 2
        self.dt_a_x = 0.5 * reach_set.configuration['a_max_x'] * self.dt ** 2
        self.dt_a_y = 0.5 * reach_set.configuration['a_max_y'] * self.dt ** 2

        self.grid_x = reach_set.configuration['grid_x']
        self.grid_y = reach_set.configuration['grid_y']
        self.a_max_x = reach_set.configuration['a_max_x']
        self.a_max_y = reach_set.configuration['a_max_y']

        # bounding box coordinates of propagated reachsets at each timestep List[list]
        self.grid_reference_ll: List[tuple] = reach_set.grid_reference_lo
        self.grid_reference_ur: List[tuple] = reach_set.grid_reference_hi

        self.reachboxes_list = time_slice2shapes(reach_set.time_slice)
        self.n_time_steps = len(reach_set.time_slice) -1
        self.reach_set_grid_list, self.node_grid_coordinate_list = self._create_reach_set_grid(reach_set) # ids of nodes at [ix,iy], maps id to [x,y]
        self.node_grid_list = self._create_node_grid(reach_set) # for each timestep: grids [nx,ny] with nodes

        # nx,ny of grids at every time step
        self._grid_shapes = list()
        for node_grid in self.node_grid_list:
            self.grid_shapes.append(node_grid.shape)
        # for sparse matrix propagation insted of graph based
        # '_reachability_matrix_offline_list' or 'P' contains matrices for each time step, to propagate reachability grid with r_t+1 = P * r_t, r_t: boolean column vecor with reachability at corresponding cell
        # '_reachability_matrix_offline_grandchildren_dict': same as _reachability_matrix_offline_list, but for grandchildren, thus list of Dict[i_timestep, reachability_matrix_offline_grandchild]
        # 'grandparent_indices' contains for every timestep pairs of time indices of all grandparents and the corresponding time step delta, e.g. (index_gradnparents, i_timestep)
        self._reachability_matrix_offline_list, self._reachability_matrix_offline_grandchildren_dict, self.grandparent_indices = self.create_sparse_children_matrix()

        # initialized later
        self.occupied_grid_obs = defaultdict(None)
        self.occupied_grid_obs_static = defaultdict(None)
        self.obstacle_grid = None
        self._projection_2to4D = None
        self.velocity = velocity

        # for online RA
        self.reset_online_RA()

        if debug is False and sparse:
            # del self.reachboxes_list
            # del self.reach_set_grid_list
            del self.node_grid_list

    @property
    def parameters_online(self):
        return self._parameters_online

    @parameters_online.setter
    def parameters_online(self, parameters_online):
        if self.initial_state_locked == True:
            warnings.warn('Parameters cannot be changed after backward reachability analysis')
        else:
            self._parameters_online = parameters_online

    @property
    def reach_box_coordinates(self) -> Dict[int, List[np.ndarray]]:
        coordinates = defaultdict(list)
        r_x = self.grid_y / 2
        r_y = self.grid_y / 2
        for i_t in range(self._max_time_step):
            reachability_grid = self._reachability_grid_list[i_t]
            center = self._reachset_translation_list[i_t]
            trans_x = self._reachset_translation_list[i_t][0] + self.grid_reference_ll[i_t][0] + r_x
            trans_y = self._reachset_translation_list[i_t][1] + self.grid_reference_ll[i_t][1] + r_y
            for ind in range(0, reachability_grid.shape[0]):
                if reachability_grid[ind] == True:
                    x, y = i2xy(ind, self.grid_shapes[i_t][1])
                    coordinates[i_t].append(np.array([trans_x + x * self.grid_x, trans_y + y * self.grid_y]))

        return dict(coordinates)

    def init_scenario(self, collision_checker: pycrcc.CollisionChecker, scenario_id,
                      parameters_online: dict = None, backward_precomputation=False, dynamic_only=False):
        """
        Initialize with scenario for online RA or reset object for new online-RA (avoid time for loading pkl file from disk)
        :param scenario:
        :param parameters_online:
        :return:
        """
        if parameters_online is None and not hasattr(self, 'parameters_online'):
            raise TypeError('parameters_online must not be None if attribute has not been set before')
        elif hasattr(self, 'parameters_online'):
            self.parameters_online = parameters_online

        if 'collision_checker_params' in parameters_online:
            params_collision_checker = parameters_online['collision_checker_params']
        else:
            params_collision_checker = None

        self.obstacle_grid = ObstacleRegularGrid(self.grid_reference_ll, self.grid_reference_ur, collision_checker,
                                                 self.grid_x, self.grid_y,
                                                 reach_params = parameters_online,
                                                 a_x=self.a_max_x, a_y=self.a_max_y, t_f=self.dt * self.n_time_steps)

        if dynamic_only or hasattr(self, 'parameters_online') and 'initial_state' in self.parameters_online and self.scenario_id == scenario_id:
            # if static obstacle grid has been calculated before, reuse
            if self.is_equal_state(self.parameters_online['initial_state'], parameters_online['initial_state']):
                self.obstacle_grid.occupied_grid_obs_static = self.occupied_grid_obs_static
            else:
                self.occupied_grid_obs_static = defaultdict(None)
        else:
            self.occupied_grid_obs_static = defaultdict(None)

        if 'sparse' in parameters_online:
            self.sparse = parameters_online['sparse']
        if parameters_online['sensitivity'] is True:
            self.return_sensitivity = True
        else:
            self.return_sensitivity = False

        self.parameters_online = parameters_online
        self.reset_online_RA()

        if backward_precomputation is True:
            self.backward_RA_scenario(self.n_time_steps)

    def is_equal_state(self, state1: State, state2: State):
        if np.array_equal(state1.position, state2.position)\
        and state1.velocity == state2.velocity\
        and state1.orientation == state2.orientation:
            if hasattr(state1, 'acceleration') and hasattr(state2, 'acceleration'):
                if state1.acceleration==state2.acceleration:
                    return True
                else:
                    return False
            else:
                return True
        else:
            return False

    # def update_dyn_collision_objects(self, new_traj_list: List[List]):
    #     """Updates dynamic collision objects in obstacle_grid to new trajectories (to update a previously initialized scenario)"""
    #     self.obstacle_grid.update_dyn_collision_objects(new_traj_list)

    def reset_online_RA(self, reset_obstacle_grid=True):
        # reset variables from online-RA
        self.initial_state_locked = False

        self._current_time_step = 0  # next timestep that needs to be propagated
        self._reachability_grid_list: List[np.ndarray] = [None] * (self.n_time_steps)
        self._reachability_grid_list[0] = np.ones(self.grid_shapes[0],
                                                       dtype=bool)  # =1 if grid cell is reachable

        self.occupied_grid_obs = defaultdict(None)
        # self.occupied_grid_obs_static = defaultdict(None)
        # # for velocity_active
        # self.minmax_velocities_online_list = [np.empty_like(node_grid_tmp) for node_grid_tmp in self.node_grid_list]
        # self.minmax_velocities_online_list[0][0,0] = 0

        # stores translation due to initial velocity at every time step
        self._reachset_translation_list = list()

        # ratio between reachable area with/ area without obstacles at every time step
        self._sensitivity = np.empty(self.n_time_steps)

    
    @property
    def grid_shapes(self):
        if not hasattr(self, '_grid_shapes') or len(self._grid_shapes) == 0:
            self._grid_shapes = [(r.shape[0], r.shape[1]) for r in self._reachability_matrix_offline_list]
        return self._grid_shapes
    
    @grid_shapes.setter
    def grid_shapes(self, grid_shapes):
        self._grid_shapes = grid_shapes
        
    @property
    def sensitivity(self):
        if self.return_sensitivity is False:
            warnings.warn('<online_graph/sensitivity> sensitivity not computed')
        return self._sensitivity[0:self._max_time_step]

    @staticmethod
    def load_csr_from_npz(filename):
        data = np.load(filename, allow_pickle=True)
        # dt = data['dt']
        matrices = []
        projections = []
        for t in range(data['nt']):
            tmp = csr_matrix((np.ones_like(data['dt0_' + 'E_indices' + str(t)]),
                                     data['dt0_' + 'E_indices' + str(t)],
                                     data['dt0_' + 'E_indptr' + str(t)]),
                                    data['dt0_' + 'E_shape' + str(t)])
            matrices.append(tmp)

            tmp = csr_matrix((np.ones_like(data['dt0_' + 'P_indices' + str(t)]),
                              data['dt0_' + 'P_indices' + str(t)],
                              data['dt0_' + 'P_indptr' + str(t)]),
                             data['dt0_' + 'P_shape' + str(t)])
            projections.append(tmp)

        # for t in range(data['nt']):
        #     for delta_timestep in range(dt):
        #         if 'data_gr' + str(t) + '_' + str(delta_timestep) in data:
        #             tmp = csr_matrix((data['data_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indices_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indptr_gr' + str(t) + '_' + str(delta_timestep)]),
        #                              data['shape_gr' + str(t) + '_' + str(delta_timestep)])
        #             matrices.append(tmp)
        parameters = data['parameters'][()]
        parameters['grid_reference_ll'] = [tuple(row) for row in parameters['grid_reference_ll']]
        parameters['grid_reference_ur'] = [tuple(row) for row in parameters['grid_reference_ur']]
        return matrices, projections, parameters

    def save_to_pickle_file(self, path: str):
        if not path.endswith(('.pkl', '.pickle')):
            path += '.pkl'
        with open(path, 'wb') as f:  # Python 3: open(..., 'wb')
            pickle.dump(self, f)

    @classmethod
    def init_from_pickle_file(cls, path: str, n_threads=-1) -> 'ReachSetGridOnline':
        if not path.endswith(('.pkl', '.pickle')):
            path += '.pkl'

        with open(path, 'rb') as f:
            reachset_online = pickle.load(f)

        assert isinstance(reachset_online, ReachSetGridOnline)

        # set thread number (max. 8 if set to default number)
        if n_threads >= 0:
            cv2.setNumThreads(n_threads)
        elif n_threads < 0 and cv2.getNumThreads() > 8:
            cv2.setNumThreads(8)

        return reachset_online

    @classmethod
    def init_from_file(cls, path: str, n_threads=-1) -> 'ReachSetGridOnline':
        #init empty ReachSetGridOnline object
        reachset_online = cls(n_threads=n_threads)
        reachset_online._reachability_matrix_offline_list = list()
        reachset_online._reachability_matrix_offline_grandchildren_dict = list()

        if path.endswith('json'):
            with open(path, 'r') as f:
                datastore = json.load(f)

            parameters = datastore['parameters']
            for i_t, dict_t_tmp in datastore['matrices'].items():
                if i_t == 0:
                    continue
                reachset_online._reachability_matrix_offline_grandchildren_dict.append(defaultdict(None))
                for dt, dict_dt_tmp in dict_t_tmp.items():
                    if dt == '0':
                        reachset_online._reachability_matrix_offline_list.append(
                                sparse.csr_matrix((np.ones_like(dict_dt_tmp['indices'], dtype=bool),
                                                   dict_dt_tmp['indices'], dict_dt_tmp['indptr']),
                                                  shape=dict_dt_tmp['shape']))
                    else:
                        reachset_online._reachability_matrix_offline_grandchildren_dict[int(float(i_t))][
                            int(float(dt))] = sparse.csr_matrix(
                            (np.ones_like(dict_dt_tmp['indices'], dtype=bool),
                             dict_dt_tmp['indices'], dict_dt_tmp['indptr']),
                            shape=dict_dt_tmp['shape'])
        elif path.endswith('npz'):
            reachset_online._reachability_matrix_offline_list, projections, parameters = cls.load_csr_from_npz(path)
            reachset_online._projection_2to4D = projections
        else:
            raise ValueError('npz or json file required')

        # assign stored parameters to variable
        for var_name, value in parameters.items():
            setattr(reachset_online, var_name, value)
        reachset_online.reset_online_RA()
        reachset_online.n_time_steps = len(reachset_online.grid_reference_ll)
        return reachset_online

    def save_init_state(self, filename):
        data_dict = defaultdict(dict)
        for i_t, csr in enumerate(self._reachability_matrix_offline_list):
            tmp_dict = {'indices': [], 'indptr': [], 'data': []}
            tmp_dict['data_size'] = csr.data.size
            if False in csr.data.tolist():
                raise ValueError
            tmp_dict['indices'] = csr.indices.tolist()
            tmp_dict['indptr'] = csr.indptr.tolist()
            tmp_dict['shape'] = csr.shape
            data_dict[i_t][0] = tmp_dict

        for i_t, reachability_matrix_dict_tmp in enumerate(self._reachability_matrix_offline_grandchildren_dict):
            for delta_timestep, csr in reachability_matrix_dict_tmp.items():
                if delta_timestep == 0:
                    warnings.warn('timestep 0 shouldnt exist!!!')
                    continue

                tmp_dict = {'indices': [], 'indptr': [], 'data_size': []}
                if False in csr.data.tolist():
                    raise ValueError
                tmp_dict['data_size'] = csr.data.size
                tmp_dict['indices'] = csr.indices.tolist()
                tmp_dict['indptr'] = csr.indptr.tolist()
                tmp_dict['shape'] = csr.shape
                data_dict[i_t][delta_timestep] = tmp_dict

        parameters = {'grid_x': self.grid_x, 'grid_y': self.grid_y,
                       'grid_reference_ll': self.grid_reference_ll, 'grid_reference_ur': self.grid_reference_ur,
                       'n_time_steps': len(self._reachability_matrix_offline_list),
                      'grid_shapes':self.grid_shapes,
                      'dt':self.dt,
                      'grandparent_indices': self.grandparent_indices,
                      'a_max_x': self.a_max_x,
                      'a_max_y': self.a_max_y}

        with open(os.getcwd() + '/out/pruning/' + filename +'.json', 'w') as fp:
            json.dump({'parameters': parameters, 'matrices': data_dict}, fp, sort_keys=True)

        # np.savez(os.getcwd() + '/out/pruning/' + filename, np.array(self._reachability_matrix_offline_list[-2]).tolist())
        # with open(os.getcwd() + '/out/pruning/' + filename +'.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
        #     pickle.dump(self, f)


    ### ACTUAL ONLINE REACHABILITY ANALYSIS  ---------------------------------------------------------------------------

    def forward_steps(self, time_steps: int, n_multisteps: int = 0):
        for i_t in range(0,time_steps):
            self.forward_step(n_multisteps)

    def forward_step(self, n_multisteps: int = 0):

        if self._current_time_step == 0:
            self._sensitivity[0] = 0.0
            pos_init = self.parameters_online['initial_state'].position
            self._reachset_translation_list.append(pos_init)
            self.parameters_online.update({'d_t': self.dt})
            if self.sparse is False and self.velocity_active:
                inital_node: GridNode = self.node_grid_list[0][0,0]
                inital_node.update_reached_velocity_intervals([Interval(-100,100),Interval(-100,100)])

        if self._current_time_step >= self.n_time_steps:
            warnings.warn('Reached max number of timesteps')
            return

        # relative translation of *propagated* reachable set (-> needs to be translated by -reachset_translation_tmp)
        initial_state: State = self.parameters_online['initial_state']
        pos_init = initial_state.position
        v_init = np.array([np.cos(initial_state.orientation),np.sin(initial_state.orientation)]) * initial_state.velocity
        reachset_translation_tmp: np.array = pos_init + v_init * self.dt * (self._current_time_step + 1)
        self._reachset_translation_list.append(reachset_translation_tmp)

        # propagate current reachable grid
        reachability_grid_current = self._reachability_grid_list[self._current_time_step]
        reachability_grid_prop = np.zeros(self.grid_shapes[self._current_time_step + 1], dtype=bool)

        if n_multisteps > 0:
            reachability_grid_prop_2 = np.zeros(self.grid_shapes[self._current_time_step + 1], dtype=bool)

        if self.sparse is False:
            """Node based representation of graph (better for debugging, no backward reachability implemented)"""
            for ix in range(0, reachability_grid_current.shape[0]):
                for iy in range(0, reachability_grid_current.shape[1]):
                    if reachability_grid_current[ix, iy]:
                        if self.velocity_active:
                            # id_current = self.reach_set_grid_list[self._current_time_step][ix,iy][0]
                            try:
                                reachable_coords, reachable_intervals = self.node_grid_list[self._current_time_step][ix,iy].reachable_children_for_velocity_intervals()
                            except:
                                reachable_coords, reachable_intervals = self.node_grid_list[self._current_time_step][
                                    ix, iy].reachable_children_for_velocity_intervals()

                            # update reached velocity intervals in children nodes
                            for reach_coord, reach_intervals in zip(reachable_coords, reachable_intervals):
                                self.node_grid_list[self._current_time_step+1][reach_coord[0],reach_coord[1]].update_reached_velocity_intervals(reach_intervals)
                        else:
                            reachable_coords = self.node_grid_list[self._current_time_step][ix,iy].children_coords

                        for reach_coord in reachable_coords:
                            reachability_grid_prop[reach_coord[0], reach_coord[1]] = True

            # propagate grandparents
            multistep_available = False
            for i_multistep in range(1,n_multisteps):
                if i_multistep <= self._current_time_step:
                    multistep_available = True
                    reachability_grid_grandparents = self._reachability_grid_list[self._current_time_step - i_multistep]
                    for ix in range(0, reachability_grid_grandparents.shape[0]):
                        for iy in range(0, reachability_grid_grandparents.shape[1]):
                            if reachability_grid_grandparents[ix,iy]:
                                reachable_coords = self.node_grid_list[self._current_time_step - i_multistep][ix, iy].grandchildren_coords[i_multistep]
                                for reach_coord in reachable_coords:
                                    reachability_grid_prop_2[reach_coord[0], reach_coord[1]] = True

            if multistep_available:
                reachability_grid_prop_old = reachability_grid_prop
                reachability_grid_prop = np.logical_and(reachability_grid_prop, reachability_grid_prop_2)

                #testing:
                old_new = np.sum(reachability_grid_prop_old) - np.sum(
                    np.logical_or(reachability_grid_prop, reachability_grid_prop_old), axis=(0, 1))
                if old_new != 0:
                    pass
                    # print('MULTISTEPDIFFERENCE:',old_new)

            # get occupied cells from obstacles
            occupied_grid_obs, occupied_grid_obs_static = self.obstacle_grid.get_occupancy_grid_at_time(self._current_time_step + 1, reachset_translation_tmp, reachability_grid_prop)

            # intersect propagated cells with occupied cells
            reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop, occupied_grid_obs)

        else:
            """Adjacency matrix based representation of graph (see paper)"""
            reachability_grid_prop_pruned = self.sparse_propagation(n_multisteps, reachset_translation_tmp)

        self._reachability_grid_list[self._current_time_step + 1] = reachability_grid_prop_pruned
        self._current_time_step += 1
        self._max_time_step = self._current_time_step

    def sparse_propagation(self, n_multisteps, reachset_translation_tmp, repeated=False):
        """
        Propagates current reachability grid and excludes forbidden states
        :param n_multisteps:
        :param reachset_translation_tmp: translation of reachset center at current time step
        :return:
        """
        reachability_grid_prop = self._reachability_matrix_offline_list[self._current_time_step].dot(
                                 self._reachability_grid_list[self._current_time_step].reshape([-1, 1]))

        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # propagate grandparents:
        for index_grandparent, i_timestep in self.grandparent_indices[
            self._current_time_step + 1]:  # get time index of grandparent to propagate
            if i_timestep < n_multisteps:
                reachability_grid_prop_grandparent = \
                    self._reachability_matrix_offline_grandchildren_dict[index_grandparent][i_timestep].dot(
                    self._reachability_grid_list[self._current_time_step - i_timestep].reshape([-1, 1]))
                if sparse.issparse(reachability_grid_prop_grandparent):
                    prop_grandparent_tmp = reachability_grid_prop_grandparent.toarray()
                else:
                    prop_grandparent_tmp = reachability_grid_prop_grandparent

                # intersect with propagated cells of other timesteps
                reachability_grid_prop = np.logical_and(reachability_grid_prop, prop_grandparent_tmp)

        if self._current_time_step + 1 not in self.occupied_grid_obs:
            # get occupied cells from obstacles
            self.occupied_grid_obs[self._current_time_step+1], self.occupied_grid_obs_static[self._current_time_step+1] = self.obstacle_grid.get_occupancy_grid_at_time(
                self._current_time_step + 1, reachset_translation_tmp,
                reachability_grid_prop.reshape(self.grid_shapes[self._current_time_step + 1]))

        # intersect propagated cells with occupied cells
        reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                       self.occupied_grid_obs[self._current_time_step+1].reshape([-1, 1]))
        if self.return_sensitivity is True and repeated is False:
            reachable_area_with_obstacles = self.compute_area(reachability_grid_prop_pruned)
            reachable_area_without_dyn_obstacles = self.compute_area(np.logical_and(reachability_grid_prop.reshape([-1, 1]), self.occupied_grid_obs_static[self._current_time_step+1].reshape([-1,1])))
            self._sensitivity[self._current_time_step + 1] = (np.divide(reachable_area_with_obstacles, reachable_area_without_dyn_obstacles) - 1.0)

        return reachability_grid_prop_pruned

    def backward_RA_scenario(self, max_time: int):
        """
        Precompute backward reachability analysis considering static obstacles. Only valid for constant inital state!
        :param max_time: maximal considered time horizon (can be larger than prediction horizon)
        :return:
        """
        raise NotImplementedError
        self.initial_state_locked = True
        self.backward_steps(max_time)
        self._current_time_step(max_time)

    def backward_steps(self, time_steps):
        for i_t in range(0,time_steps):
            if self._current_time_step <= 1:
                continue
            self.backward_step()

    def backward_forward(self, n_multisteps):
        self._current_time_step -= 1
        self.backward_steps(self.n_time_steps-1)
        self.forward_steps_repeat(n_multisteps)
        # self.backward_steps(self.n_time_steps - 1)

    def forward_steps_repeat(self, n_multisteps):
        for i_t in range(self._current_time_step,self._max_time_step):
            self._current_time_step = i_t
            reachability_grid_prop_pruned = self.sparse_propagation(n_multisteps,None, repeated=True)
            self._reachability_grid_list[self._current_time_step + 1] = np.logical_and(self._reachability_grid_list[self._current_time_step + 1], reachability_grid_prop_pruned)
            self._current_time_step += 1

    def backward_step(self):
        assert self.sparse

        if self._current_time_step <= 0:
            warnings.warn('Reached max number of backward timesteps')
            return

        if self._reachset_translation_list[self._current_time_step] is None:
            # compute translation
            initial_state: State = self.parameters_online['initial_state']
            pos_init = np.array(initial_state.position)
            v_init = initial_state.velocity * np.array([[initial_state.velocity * np.cos(initial_state.orientation),], [initial_state.velocity * np.sin(initial_state.orientation),]])
            self.parameters_online.update({'d_t': self.dt})

            reachset_translation_tmp: np.array = pos_init + v_init * self.parameters_online['d_t'] * self._current_time_step
            self._reachset_translation_list[self._current_time_step] = reachset_translation_tmp
            # get occupied cells from obstacles
            occupied_grid_obs = self.obstacle_grid.get_occupancy_grid_at_time(self._current_time_step, reachset_translation_tmp, np.ones(self.grid_shapes[self._current_time_step]))
            self._reachability_grid_list[self._current_time_step] = occupied_grid_obs.reshape([-1, 1])

            # relative translation of *propagated* reachable set (-> needs to be translated by -reachset_translation_tmp)
            initial_state: State = self.parameters_online['initial_state']
            pos_init = np.array(initial_state.position)
            v_init = initial_state.velocity * np.array([[initial_state.velocity * np.cos(initial_state.orientation),], [initial_state.velocity * np.sin(initial_state.orientation),]])
            reachset_translation_tmp: np.array = pos_init + v_init * self.parameters_online['d_t'] * (self._current_time_step - 1)
            self._reachset_translation_list[self._current_time_step-1] = reachset_translation_tmp

        # # propagate current reachable grid
        # reachability_grid_current = self._reachability_grid_list[self._current_time_step]
        # reachability_grid_prop = np.zeros(self.grid_shapes[self._current_time_step - 1], dtype=bool)

        # if n_multisteps > 0:
        #     reachability_grid_prop_2 = np.zeros(self.grid_shapes[self._current_time_step + 1], dtype=bool)

        # actual step

        reachability_grid_prop = self._reachability_matrix_offline_list[self._current_time_step-1].transpose() * \
                                 self._reachability_grid_list[self._current_time_step].reshape([-1, 1])

        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # # propagate grandparents:
        # for index_grandparent, i_timestep in self.grandparent_indices[
        #     self._current_time_step + 1]:  # get time index of grandparent to propagate
        #     if i_timestep < n_multisteps:
        #         reachability_grid_prop_grandparent = \
        #         self._reachability_matrix_offline_grandchildren_dict[index_grandparent][i_timestep] * \
        #         self._reachability_grid_list[self._current_time_step - i_timestep].reshape([-1, 1])
        #         if sparse.issparse(reachability_grid_prop_grandparent):
        #             prop_grandparent_tmp = reachability_grid_prop_grandparent.toarray()
        #         else:
        #             prop_grandparent_tmp = reachability_grid_prop_grandparent
        #
        #         # intersect with propagated cells of other timesteps
        #         reachability_grid_prop = np.logical_and(reachability_grid_prop, prop_grandparent_tmp)

        # get occupied cells from obstacles
        if self.occupied_grid_obs[self._current_time_step-1] is not None:
            occupied_grid_obs = self.occupied_grid_obs[self._current_time_step-1]
        else:
            # occupancy has not been computed before (pure bw reachability)
            occupied_grid_obs = self.obstacle_grid.get_occupancy_grid_at_time(self._current_time_step - 1,
                                                                              reachset_translation_tmp,
                                                                              reachability_grid_prop.reshape(self.grid_shapes[self._current_time_step - 1]))
        #
        # intersect propagated cells with occupied cells
        reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                       occupied_grid_obs.reshape([-1, 1]))
        self._reachability_grid_list[self._current_time_step - 1] = np.logical_and(self._reachability_grid_list[self._current_time_step - 1], reachability_grid_prop_pruned)
        # self._reachability_grid_list[self._current_time_step - 1] = np.logical_and(self._reachability_grid_list[self._current_time_step - 1], reachability_grid_prop.reshape([-1, 1]))
        self._current_time_step -= 1



    def create_sparse_children_matrix(self):
        reachability_matrix_offline_list = list()
        reachability_matrix_offline_gc_list: List[Dict[int,np.array]] = list()
        grandparent_indices: List[List[Tuple[int,int]]] = [[] for i in range(len(self.node_grid_list))]  # contains for every timestep pairs of time indices of all grandparents and the corresponding time step delta, e.g. (index_gradnparents, i_timestep)

        for i_t, reachability_grid_current in enumerate(self.node_grid_list[:-1]):
            n_cells_current = self.node_grid_list[i_t  ].shape[0] * self.node_grid_list[i_t  ].shape[1]  # cells in current grid
            n_cells_next    = self.node_grid_list[i_t+1].shape[0] * self.node_grid_list[i_t+1].shape[1]  # cells in next grid
            ny_next = self.node_grid_list[i_t+1].shape[1]
            reachability_matrix_offline_tmp = np.zeros([n_cells_next, n_cells_current], dtype=bool)

            # initialze matrices for grandchildren (one for each generation)
            reachability_matrix_offline_gc_dict_tmp: Dict[int,np.array] = dict()  # {i_timestep: reachabiltiy_matrix,...}
            ny_timestep: Dict[int,int] = dict() # numer of columns at i-th grandchildren

            mask = self.node_grid_list[i_t] != None

            node_example = self.node_grid_list[i_t][mask][0]

            for i_timestep in list(node_example.grandchildren_coords.keys()):
                n_cells_i_timestep = self.node_grid_list[i_t + 1 + i_timestep].shape[0] * \
                                     self.node_grid_list[i_t + 1 + i_timestep].shape[1]  # cells at i-th grandchild
                ny_timestep.update({i_timestep: self.node_grid_list[i_t + 1 + i_timestep].shape[1]})
                reachability_matrix_offline_gc_dict_tmp.update({i_timestep: np.zeros([n_cells_i_timestep, n_cells_current], dtype=bool)})

            i_current = 0 # index in 1d-converted current grid
            for ix in range(0, reachability_grid_current.shape[0]):
                for iy in range(0, reachability_grid_current.shape[1]):
                    if self.node_grid_list[i_t][ix, iy] is None:
                        i_current += 1
                        continue
                    reachable_coords = self.node_grid_list[i_t][ix, iy].children_coords
                    for coord in reachable_coords:
                        reachability_matrix_offline_tmp[xy2i_conversion(coord,ny_next), i_current] = True  # matrix will determine reachability of a cell by row-wise any(M * r_current) = r_next

                    #grandchildren:
                    reachable_coords_grandchildren_dict = self.node_grid_list[i_t][ix, iy].grandchildren_coords
                    for i_timestep, reachable_coords_gc in reachable_coords_grandchildren_dict.items():  # one matrix for every grandchildren generation
                        for coord in reachable_coords_gc:
                            reachability_matrix_offline_gc_dict_tmp[i_timestep][xy2i_conversion(coord,ny_timestep[i_timestep]), i_current] = True

                    i_current += 1

            #convert to sparse csr matrix
            reachability_matrix_offline_list.append(sparse.csr_matrix(reachability_matrix_offline_tmp))
            for i_timestep, grid in reachability_matrix_offline_gc_dict_tmp.items():
                reachability_matrix_offline_gc_dict_tmp[i_timestep] = sparse.csr_matrix(grid)
                grandparent_indices[i_t+i_timestep+1].append( (i_t, i_timestep) )
            reachability_matrix_offline_gc_list.append(reachability_matrix_offline_gc_dict_tmp)

        return reachability_matrix_offline_list, reachability_matrix_offline_gc_list, grandparent_indices

    def create_sparse_grandchildren_matrices(self):
        reachability_matrix_offline_list = list()
        for i_t, reachability_grid_current in enumerate(self.node_grid_list[:-1]):
            n_cells_current = self.node_grid_list[i_t].shape[0] * self.node_grid_list[i_t].shape[
                1]  # cells in current grid
            n_cells_next = self.node_grid_list[i_t + 1].shape[0] * self.node_grid_list[i_t + 1].shape[
                1]  # cells in next grid
            ny_next = self.node_grid_list[i_t + 1].shape[1]
            reachability_matrix_offline_tmp = np.zeros([n_cells_next, n_cells_current], dtype=bool)

            i_current = 0
            for ix in range(0, reachability_grid_current.shape[0]):
                for iy in range(0, reachability_grid_current.shape[1]):
                    reachable_coords = self.node_grid_list[i_t][ix, iy].children_coords
                    for coord in reachable_coords:
                        reachability_matrix_offline_tmp[xy2i_conversion(coord,
                                                                        ny_next), i_current] = True  # matrix will determine reachability of a cell by row-wise any(M * r_current) = r_next

                    i_current += 1

            reachability_matrix_offline_list.append(sparse.csr_matrix(reachability_matrix_offline_tmp))
        return reachability_matrix_offline_list


    def delete_colliding_nodes(self):
        """
        Delete nodes with no children (apart from last time-step)
        :return: None
        """
        for id_t_rev, reachability_grid in enumerate(reversed(self._reachability_grid_list[1:self._current_time_step])):
            id_t = self._current_time_step - 2 - id_t_rev
            node_grid = self.node_grid_list[id_t]

            for ix, iy in np.ndindex(node_grid.shape):
                # if no children -> inevitible collision
                if reachability_grid[ix, iy]==1:
                    if node_grid[ix, iy].children == []:
                        reachability_grid[ix, iy] = 0

                if reachability_grid[ix, iy] == 0:
                    current_ids = node_grid[ix, iy].reach_set_ids
                    parents = node_grid[ix, iy].parents
                    for p in parents:
                        (px,py) =self.node_grid_coordinate_list[id_t-1][p]
                        for id in current_ids:
                            if id in self.node_grid_list[id_t-1][px,py].children:
                                self.node_grid_list[id_t - 1][px, py].children.remove(id)


    def _create_node_grid(self, reach_set: ReachSetGridOffline) -> List[np.ndarray]:
        node_grid_list = list()

        for i_t, reach_set_grid in enumerate(self.reach_set_grid_list):
            node_grid_tmp = np.empty(reach_set_grid.shape, dtype=object)
            for ix in range(0,reach_set_grid.shape[0]):
                for iy in range(0,reach_set_grid.shape[1]):
                    parents_tmp = list()
                    children_tmp = list()
                    if reach_set_grid[ix][iy] is not None:
                        for reach_set_node_id in self.reach_set_grid_list[i_t][ix][iy]:
                            reach_set_node: ReachSetNodeMultiGeneration = reach_set.time_slice[i_t][reach_set_node_id]
                            current_rect: AARectDouble = reach_set.reach_set_rects[i_t-1][reach_set_node_id]
                            parents_tmp += reach_set_node.parent_nodes
                            children_tmp += reach_set_node.child_nodes

                            grandchildren={}
                            grandparents = {}
                            if type(reach_set_node) == ReachSetNodeMultiGeneration:
                                if len(reach_set_node.grandchild_nodes.keys()) > 0: # grandchild_nodes belongs to subclass ReachSetNodeMultiGeneration
                                    grandchildren = reach_set_node.grandchild_nodes
                                    grandparents = reach_set_node.grandparent_nodes

                            # assign minmax velocities to parent nodes
                            if self.velocity_active:
                                if i_t > 0:
                                    for parent_id, vel_minmax in self.velocities_minmax_list[i_t][reach_set_node_id].items(): # i_t-1 or i_t?
                                        listt = reach_set.time_slice[i_t - 1]
                                        # try:
                                        parent_poly_x = reach_set.time_slice[i_t - 1][parent_id].reach_x
                                        parent_poly_y = reach_set.time_slice[i_t - 1][parent_id].reach_y

                                        parent_rect: AARectDouble = reach_set.reach_set_rects[i_t-2][parent_id]

                                        # velocity intervals, from which current node can be reached by respective parent
                                        velocity_interval_x, velocity_interval_y = self.compute_parents_minmax_velocities(parent_poly_x, parent_poly_y, current_rect, parent_rect)
                                        # try:
                                        # update parent nodes in gridlist with respective velocity intervals
                                        node_grid_list[i_t - 1][self.node_grid_coordinate_list[i_t-1][parent_id]].velocities_minmax_children_reachable.update({reach_set_node_id: [velocity_interval_x, velocity_interval_y]})
                                        node_grid_list[i_t - 1][self.node_grid_coordinate_list[i_t - 1][parent_id]].velocities_minmax_children_reached.update({reach_set_node_id: [velocity_interval_x, velocity_interval_y]})
                                        # except:
                                        #
                                        #     print()

                        node_grid_tmp[ix][iy] = GridNode(i_t, reach_set_grid[ix][iy], parents_tmp, children_tmp,
                                                         reach_set.time_slice[i_t - 1] if i_t > 0 else [], #prev and next time slice
                                                         reach_set.time_slice[i_t + 1] if i_t < len(reach_set.time_slice)-1 else [],
                                                         self.node_grid_coordinate_list, grandchildren, grandparents)

            node_grid_list.append(node_grid_tmp)

        return node_grid_list

    def compute_parents_minmax_velocities(self, parent_poly_x: Polyhedron2VRep, parent_poly_y: Polyhedron2VRep, child_rect_aa: AARectDouble, parent_rect) -> Tuple[Interval, Interval]:
        """
        Computes min/max velocities of parent's polygon borders for which at least one state can reach at least one location in child_rect_aa
        :param parent_poly_x:
        :param parent_poly_y:
        :param child_rect_aa:
        :return:
        """
        def compute_v_minmax(parent_poly, dt_a_max, chi_x_min, chi_x_max):
            par_x_min = parent_poly.min_x()
            par_x_max = parent_poly.max_x()
            # if xy == 0:
            #     par_x_min = parent_rect.x_lo
            #     par_x_max = parent_rect.x_hi
            # elif xy ==1:
            #     par_x_min = parent_rect.y_lo
            #     par_x_max = parent_rect.y_hi
            # else:
            #     raise ValueError

            distance = np.average([chi_x_min, chi_x_max]) - np.average([par_x_min, par_x_max])
            reverse = np.sign(distance)
            if reverse==0:
                reverse = 1

            dx_par = par_x_max - par_x_min
            dx_chi = chi_x_max - chi_x_min

            dx = (dx_par + dx_chi)/2

            delta_x_min = abs(distance) - dx
            delta_x_max = abs(distance) + dx

            v_1 = (delta_x_min - dt_a_max) / self.dt * reverse
            v_2 = (delta_x_max + dt_a_max) / self.dt * reverse

            v_min, v_max = tuple(sorted([v_1, v_2]))

            # if delta_x_min < delta_x_max:
            #     reverse = False
            #
            #     v_min = (delta_x_min - dt_a_max) / self.dt
            #     v_max = (delta_x_max + dt_a_max) / self.dt
            # else:
            # # parent X > child X -> reverse calculations
            #     reverse = True
            #     # delta_x_min = distance + dx
            #     # delta_x_max = distance - dx
            #
            #     v_min = -(delta_x_max + dt_a_max) / self.dt
            #     v_max = -(delta_x_min - dt_a_max) / self.dt

            ## TESTING:-------------------------
            # par_v_min = parent_poly.min_y()
            # par_v_max = parent_poly.max_y()
            # plt.ioff()
            # if not ((is_in_interval(min([v_min,v_max]), par_v_min, par_v_max)) or par_v_min >= min([v_min,v_max])):
            #     plt.figure()
            #     draw_object(parent_poly)
            #     draw_object([child_rect_aa, parent_rect])
            #
            #     plt.gca().autoscale()
            #     plt.show()
            #     raise ValueError
            # if not ((is_in_interval(max([v_min,v_max]),par_v_min, par_v_max)) or par_v_max <= max([v_min,v_max])):
            #     plt.figure()
            #     draw_object(parent_poly)
            #     draw_object([child_rect_aa, parent_rect])
            #     plt.gca().autoscale()
            #     plt.show()
            #     y_min_r = parent_rect.y_lo
            #     y_max_r = parent_rect.y_hi
            #     raise ValueError
            ##----------------------------------

            return Interval(min([v_min,v_max]), max([v_min,v_max]))

        vx_interval = compute_v_minmax(parent_poly_x, self.dt_a_x, child_rect_aa.x_lo, child_rect_aa.x_hi)
        vy_interval = compute_v_minmax(parent_poly_y, self.dt_a_y, child_rect_aa.y_lo, child_rect_aa.y_hi)

        return vx_interval, vy_interval

    def _create_reach_set_grid(self, reach_set: ReachSetGridOffline) -> (List[np.ndarray], List[Dict[int, tuple]]):
        reach_set_grid_list = list()
        node_grid_coordinate_list = list() # each line assigns the grid coordinates to the corresponding element in rech_set_nodes

        # root node:
        root_grid = np.empty([1,1], dtype=object)
        root_grid[0,0] = [0]
        reach_set_grid_list.append(root_grid)
        node_grid_coordinate_list.append({0: (0,0)})

        for i_t in range(0, len(reach_set.reach_set_rects)):
            grid_reference_lo = reach_set.grid_reference_lo[i_t+1]
            grid_reference_hi = reach_set.grid_reference_hi[i_t+1]

            nx = math.ceil((grid_reference_hi[0] - grid_reference_lo[0]) / reach_set.forward_step.grid_x - 0.000001)
            ny = math.ceil((grid_reference_hi[1] - grid_reference_lo[1]) / reach_set.forward_step.grid_y - 0.000001)

            reach_set_grid_tmp = np.empty((nx, ny), dtype=object)
            node_id_coordinate_mapping_tmp = dict()

            for id_rect, rect_i in enumerate(reach_set.reach_set_rects[i_t]):
                x_lo = rect_i.x_lo
                y_lo = rect_i.y_lo
                i_x = round((x_lo - grid_reference_lo[0]) / reach_set.forward_step.grid_x)
                i_y = round((y_lo - grid_reference_lo[1]) / reach_set.forward_step.grid_y)

                if reach_set_grid_tmp[i_x, i_y] is None:
                    reach_set_grid_tmp[i_x, i_y] = [id_rect]
                else:
                    reach_set_grid_tmp[i_x, i_y].append(id_rect)

                node_id_coordinate_mapping_tmp.update({id_rect: (i_x, i_y)})

            reach_set_grid_list.append(reach_set_grid_tmp)
            node_grid_coordinate_list.append(node_id_coordinate_mapping_tmp)
        return reach_set_grid_list, node_grid_coordinate_list

    def compute_area(self, reachability_grid: np.ndarray, box_area=None) -> float:
        """computes area of given reachability_grid"""
        if box_area is None:
            box_area = self.grid_x * self.grid_y
        return np.sum(reachability_grid) * box_area

    def area_profile(self):
        area_profile = []
        box_area = self.grid_x * self.grid_y
        for reachability_grid in self._reachability_grid_list:
            if reachability_grid is None:
                break
            area_profile.append(self.compute_area(reachability_grid))
        return np.array(area_profile)

    def create_reachset_list_for_plotting(self, inverse=False, time_range: List[int] = [0,-1], seperate_time_steps=False):
        """

        :param inverse:
        :param time_range:
        :param seperate_time_steps: if true create new list element for each time step, otherwise all rects in one list
        :return:
        """
        tmp_rectlist = list() #List[scenario.commonroad.shapes.Polygon]
        if seperate_time_steps:
            rectlist = list()
        for i, range_i in enumerate(time_range):
            if range_i < 0:
                time_range[i] = range_i + self._max_time_step+1

        if inverse == True:
            reachability_grid_tmp = reversed(self._reachability_grid_list)
        else:
            reachability_grid_tmp = self._reachability_grid_list

        r_x = self.grid_x / 2
        r_y = self.grid_y / 2
        for i_t, reachability_grid in enumerate(reachability_grid_tmp):
            if reachability_grid is None:
                warnings.warn('<create_reachset_list_for_plotting>: max. number of propagated timesteps was {}'.format(i_t - 1), stacklevel=2)
                # break

            if inverse is True:
                i_t = len(self._reachability_grid_list) - i_t - 1

            if self.sparse is False:
                if i_t in range(time_range[0],time_range[1]+1):
                    trans_x = self._reachset_translation_list[i_t][0] + self.grid_reference_ll[i_t][0] + r_x
                    trans_y = self._reachset_translation_list[i_t][1] + self.grid_reference_ll[i_t][1] + r_y
                    for ix in range(0,reachability_grid.shape[0]):
                        for iy in range(0,reachability_grid.shape[1]):
                            if reachability_grid[ix,iy] == True:
                                tmp_rectlist.append(AARectDouble(r_x, r_y, trans_x + x * self.grid_x, trans_y + y * self.grid_y))
                                # tmp_rectlist.append((self.reachboxes_list[i_t][self.reach_set_grid_list[i_t][ix,iy][0]]).translate_rotate(list(self._reachset_translation_list[i_t].tolist()),0.0))
                    if seperate_time_steps:
                        rectlist.append(tmp_rectlist)
                        tmp_rectlist = list()
            else:
                if i_t in range(time_range[0],time_range[1]+1):
                    trans_x = self._reachset_translation_list[i_t][0] + self.grid_reference_ll[i_t][0]# + r_x
                    trans_y = self._reachset_translation_list[i_t][1] + self.grid_reference_ll[i_t][1]# + r_y
                    for ind in range(0,reachability_grid.shape[0]):
                        if reachability_grid[ind] == True:
                            x, y = i2xy(ind, self.grid_shapes[i_t][1])
                            # tmp_rectlist.append(AARectDouble(r_x, r_y, trans_x + x*self.grid_x, trans_y + y*self.grid_y))
                            tmp_rectlist.append(Rectangle(self.grid_x, self.grid_y, np.array([trans_x + x * self.grid_x, trans_y + y * self.grid_y])))
                            # rectlist.append((self.reachboxes_list[i_t][self.reach_set_grid_list[i_t][x, y][0]]).translate_rotate(list(self._reachset_translation_list[i_t].tolist()), 0.0))
                    if seperate_time_steps:
                        rectlist.append(tmp_rectlist)
                        tmp_rectlist = list()

        if seperate_time_steps==False:
            rectlist = tmp_rectlist

        return rectlist


    # def _project_basesets_into_position_domain(poly_x_q, poly_y_q):
    #     projected_aarect_double = []
    #     for r in range(0, len(poly_x_q)):
    #         if poly_x_q[r].is_empty() or poly_y_q[r].is_empty():
    #             # print("Warning: dummy aa rect inserted")
    #             projected_aarect_double.append(None)
    #         else:
    #             projected_aarect_double.append(
    #                 pyfvks.reach.AARectDouble(poly_x_q[r].min_x(),
    #                                           poly_x_q[r].max_x(),
    #                                           poly_y_q[r].min_x(),
    #                                           poly_y_q[r].max_x()))
    #     return projected_aarect_double



