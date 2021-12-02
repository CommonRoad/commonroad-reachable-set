import commonroad_dc.pycrcc as pycrcc
import scipy.sparse as sparse
from scipy.sparse import csc_matrix, csr_matrix
import os
import time
from pycrreach import Polyhedron2VRep, AARectDouble
try:
    from sparse_dot_mkl import sparse_dot
except:
    pass

from pruningPy.graph_reach.offline_graph_reach import ReachSetGridOffline
import math
from commonroad.scenario.trajectory import State
from commonroad.common.util import Interval
from commonroad.scenario.scenario import Scenario
from typing import Dict, Tuple
from pruningPy.graph_reach.utils import *
from pruningPy.graph_reach.collision_object_grid import ObstacleRegularGrid

from commonroad.common.validity import *
import json


class GraphReachability4D():
    def __init__(self, _reachability_matrix_offline_list:List[csc_matrix], _projection_2to4D:List[csc_matrix], parameters:dict, debug=True, velocity=True):
        # Has to be initialized with self.init_from_file
        self._reachability_matrix_offline_list = None
        self._reachability_matrix_offline_grandchildren_dict = None
        self.dt = parameters['dt']

        # constants dt^2 * a_max / 2
        self.dt_a_x = 0.5 * parameters['a_max_x'] * self.dt ** 2
        self.dt_a_y = 0.5 * parameters['a_max_y'] * self.dt ** 2

        self.grid_x = parameters['grid_x']
        self.grid_y = parameters['grid_y']
        self.a_max_x = parameters['a_max_x']
        self.a_max_y = parameters['a_max_y']

        # bounding box coordinates of propagated reachsets at each timestep List[list]
        self.grid_reference_ll: List[tuple] = parameters['grid_reference_ll']
        self.grid_reference_ur: List[tuple] = parameters['grid_reference_ur']

        self.n_time_steps = parameters['n_time_steps']

        # nx,ny of grids at every time step
        self._grid_shapes_all = list()
        self.grid_shapes4d = parameters['shape_4d']
        self._reachability_matrix_offline_list = _reachability_matrix_offline_list
        # inverse projection from 2d to 4d coordinates for exapanding 2d occupancies
        self._projection_2to4D = _projection_2to4D
        self.velocity = velocity

        # initialized later
        self.occupied_grid_obs_static = defaultdict(None)
        self.obstacle_grid: ObstacleRegularGrid = None
        self._parameters_online = None

        # ratio between reachable area with/ area without obstacles at every time step
        self.return_sensitivity = False
        self._sensitivity = np.empty(self.n_time_steps)

        # for online RA
        self.initial_state_locked = False

        self._current_time_step = 0  # next timestep that needs to be propagated
        self._reachability_grid_list: List[csc_matrix] = [None] * (self.n_time_steps + 1)
        tmp = np.zeros((self.grid_shapes_all[0][1], 1), dtype=bool)
        [tmp.put(indices, True) for indices in _reachability_matrix_offline_list[0].nonzero()[1]]
        self._reachability_grid_list[0] = csc_matrix(tmp)  # =1 if grid cell is reachable
        # stores cells occupied by obstacles
        self.occupied_grid_obs: dict = dict()
        self.occupied_grid_obs4d: dict = dict()

        # stores translation due to initial velocity at every time step
        self._reachset_translation_list = list()

    @property
    def parameters_online(self):
        return self._parameters_online

    @parameters_online.setter
    def parameters_online(self, parameters_online):
        if self.initial_state_locked == True:
            warnings.warn('Parameters cannot be changed after backward reachability analysis')
        else:
            self._parameters_online = parameters_online

    def init_scenario(self, collision_checker: pycrcc.CollisionChecker,
                      parameters_online: dict = None, backward_precomputation=False):
        """
        Initialize with scenario for online RA or reset object for new online-RA (avoid time for loading pkl file from disk)
        :param scenario:
        :param parameters_online:
        :return:
        """
        if parameters_online is None and not hasattr(self, 'parameters_online'):
            raise ValueError('parameters_online must not be None if attribute has not been set before')
        elif hasattr(self, 'parameters_online'):
            self.parameters_online = parameters_online

        if 'collision_checker_params' in parameters_online:
            params_collision_checker = parameters_online['collision_checker_params']
        else:
            params_collision_checker = None

        self.obstacle_grid = ObstacleRegularGrid(self.grid_reference_ll, self.grid_reference_ur, collision_checker,
                                                 self.grid_x, self.grid_y,
                                                 dynamic_only=parameters_online['dynamic_only'],
                                                 reach_params = parameters_online,
                                                 a_x=self.a_max_x, a_y=self.a_max_y, t_f=self.dt * self.n_time_steps)

        if hasattr(self, 'parameters_online') and 'initial_state' in self.parameters_online:
            # if static obstacle grid has been calculated before, reuse
            if self.is_equal_state(self.parameters_online['initial_state'], parameters_online['initial_state']):
                self.obstacle_grid.occupied_grid_obs_static = self.occupied_grid_obs_static
            else:
                self.occupied_grid_obs_static = defaultdict(None)
        else:
            self.occupied_grid_obs_static = defaultdict(None)

        if parameters_online['sensitivity'] is True:
            self.return_sensitivity = True
        else:
            self.return_sensitivity = False

        self.parameters_online = parameters_online
        self.reset_online_RA()

        if backward_precomputation is True:
            self.backward_RA_scenario(self.n_time_steps)

    def is_equal_state(self, state1: State, state2: State):
        if np.array_equal(state1.position, state2.position) \
                and state1.velocity == state2.velocity \
                and state1.orientation == state2.orientation:
            if hasattr(state1, 'acceleration') and hasattr(state2, 'acceleration'):
                if state1.acceleration == state2.acceleration:
                    return True
                else:
                    return False
            else:
                return True
        else:
            return False

    def reset_online_RA(self):
        # reset variables from online-RA
        self.initial_state_locked = False

        self._current_time_step = 0  # next timestep that needs to be propagated
        self._reachability_grid_list: List[csc_matrix] = [None] * (self.n_time_steps + 1)
        tmp = np.zeros((self.grid_shapes_all[0][1], 1), dtype=bool)
        [tmp.put(indices, True) for indices in self._reachability_matrix_offline_list[0].nonzero()[1]]
        self._reachability_grid_list[0] = csc_matrix(tmp)  # =1 if grid cell is reachable
        # stores cells occupied by obstacles
        self.occupied_grid_obs: dict = dict()
        self.occupied_grid_obs4d: dict = dict()

        # stores translation due to initial velocity at every time step
        self._reachset_translation_list = list()

        # ratio between reachable area with/ area without obstacles at every time step
        self._sensitivity = np.empty(self.n_time_steps)

    @property
    def grid_shapes_all(self):
        if not hasattr(self, '_grid_shapes') or len(self._grid_shapes_all) == 0:
            self._grid_shapes_all = [(r.shape[0], r.shape[1]) for r in self._reachability_matrix_offline_list]
        return self._grid_shapes_all

    @grid_shapes_all.setter
    def grid_shapes_all(self, grid_shapes):
        self._grid_shapes_all = grid_shapes

    @property
    def grid_shapes_2d(self):
        if not hasattr(self, '_grid_shapes') or len(self._grid_shapes_all) == 0:
            self._grid_shapes_2d = [(_shape[0], _shape[2]) for _shape in self.grid_shapes4d]
        return self._grid_shapes_2d

    @grid_shapes_2d.setter
    def grid_shapes_2d(self, grid_shapes_2d):
        self._grid_shapes_2d = grid_shapes_2d

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
                             data['dt0_' + 'E_shape' + str(t)], dtype=bool)
            matrices.append(csc_matrix(tmp))

            tmp = csr_matrix((np.ones_like(data['dt0_' + 'P_indices' + str(t)]),
                              data['dt0_' + 'P_indices' + str(t)],
                              data['dt0_' + 'P_indptr' + str(t)]),
                             data['dt0_' + 'P_shape' + str(t)], dtype=bool)
            projections.append(csc_matrix(tmp))

        # for t in range(data['nt']):
        #     for delta_timestep in range(dt):
        #         if 'data_gr' + str(t) + '_' + str(delta_timestep) in data:
        #             tmp = csr_matrix((data['data_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indices_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indptr_gr' + str(t) + '_' + str(delta_timestep)]),
        #                              data['shape_gr' + str(t) + '_' + str(delta_timestep)])
        #             matrices.append(tmp)
        parameters = data['parameters'][()]
        parameters['grid_reference_ll'] = [tuple(row[[0,2]]) for row in parameters['grid_reference_ll']]
        parameters['grid_reference_ur'] = [tuple(row[[0,2]]) for row in parameters['grid_reference_ur']]
        parameters['shape_4d'] = data['shape_4d']
        return matrices, projections, parameters

    @classmethod
    def init_from_file(cls, path: str) -> 'GraphReachability4D':
        """Read offline matrices from file."""
        if path.endswith('npz'):
            reachability_matrix_offline_list, projections, parameters = cls.load_csr_from_npz(path)
            return cls(reachability_matrix_offline_list, projections, parameters)
        else:
            raise ValueError('.npz file required')

    ### ACTUAL ONLINE REACHABILITY ANALYSIS  ---------------------------------------------------------------------------
    def forward_steps(self, time_steps: int, n_multisteps: int = 0):
        for i_t in range(0, time_steps):
            self.forward_step(n_multisteps)

    def forward_step(self, n_multisteps: int = 0):
        if self._current_time_step == 0:
            self._sensitivity[0] = 0.0
            pos_init = self.parameters_online['initial_state'].position
            self._reachset_translation_list.append(pos_init)
            self.parameters_online.update({'d_t': self.dt})

        if self._current_time_step >= self.n_time_steps:
            warnings.warn('Reached max number of timesteps')
            return

        # relative translation of *propagated* reachable set (-> needs to be translated by -reachset_translation_tmp)
        initial_state: State = self.parameters_online['initial_state']
        pos_init = initial_state.position
        v_init = np.array(
            [np.cos(initial_state.orientation), np.sin(initial_state.orientation)]) * initial_state.velocity
        reachset_translation_tmp: np.array = pos_init + v_init * self.dt * (self._current_time_step + 1)
        self._reachset_translation_list.append(reachset_translation_tmp)

        # if n_multisteps > 0:
        #     reachability_grid_prop_2 = np.zeros(self.grid_shapes[self._current_time_step + 1], dtype=bool)

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


        # reachability_grid_prop = self._reachability_matrix_offline_list[self._current_time_step].dot(
        #                         self._reachability_grid_list[self._current_time_step])
        # print('init',time.time()-t_0)
        aa = self._reachability_matrix_offline_list[self._current_time_step].astype('float32')
        bb = self._reachability_grid_list[self._current_time_step].astype('float32')
        t_0 = time.time()
        reachability_grid_prop = sparse_dot.dot_product_mkl(aa,bb)
        # reachability_grid_prop = reachability_grid_prop.toarray().nonzero()
        # reachability_grid_prop = csc_matrix((np.ones_like(compr[0]), compr), shape=reachability_grid_prop.get_shape(), dtype=bool)

        # if self._current_time_step >= 20:
        #     # a = self._reachability_matrix_offline_list[self._current_time_step].astype('int')
        #     # bb  =self._reachability_grid_list[self._current_time_step].astype('int')
        #     # _ = a.dot(bb)
        #     t_0 = time.time()
        #     reachability_grid_prop = self._reachability_matrix_offline_list[self._current_time_step].dot(
        #         self._reachability_grid_list[self._current_time_step])
        #     print('csc',time.time() - t_0)
        #
        #     aa = self._reachability_matrix_offline_list[self._current_time_step].astype('float32')
        #     bb = self._reachability_grid_list[self._current_time_step].astype('float32')
        #     t_0 = time.time()
        #     _ = sparse_dot.dot_product_mkl(aa,bb)
        #     print('mkl',time.time() - t_0)

        # if sparse.issparse(reachability_grid_prop):
        #     reachability_grid_prop = reachability_grid_prop.toarray()
        # print(self._current_time_step)
        if self._current_time_step + 1 not in self.occupied_grid_obs:
            # get occupied cells from obstacles
            self.occupied_grid_obs[self._current_time_step + 1], self.occupied_grid_obs_static[
                self._current_time_step + 1] = self.obstacle_grid.get_occupancy_grid_at_time(
                self._current_time_step + 1, reachset_translation_tmp, None)

            self.occupied_grid_obs[self._current_time_step + 1] = \
                self.occupied_grid_obs[self._current_time_step + 1].reshape([-1, 1])

        if self.velocity is True:
            try:
                self.occupied_grid_obs4d[self._current_time_step + 1] = \
                    self._projection_2to4D[self._current_time_step + 1].dot(
                        self.occupied_grid_obs[self._current_time_step + 1])
            except:
                ii=0

            # intersect propagated cells with occupied cells
            reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.toarray(), self.occupied_grid_obs4d[self._current_time_step + 1])
        else:
            reachability_grid_prop_pruned =reachability_grid_prop + self.occupied_grid_obs[self._current_time_step + 1]

        # if self.return_sensitivity is True and repeated is False:
        #     reachable_area_with_obstacles = self.compute_area(reachability_grid_prop_pruned)
        #     reachable_area_without_dyn_obstacles = self.compute_area(
        #         np.logical_and(reachability_grid_prop.reshape([-1, 1]),
        #                        self.occupied_grid_obs_static[self._current_time_step + 1].reshape([-1, 1])))
        #     self._sensitivity[self._current_time_step + 1] = (
        #                 np.divide(reachable_area_with_obstacles, reachable_area_without_dyn_obstacles) - 1.0)

        return csc_matrix(reachability_grid_prop_pruned)

    def backward_RA_scenario(self, max_time: int):
        """
        Precompute backward reachability analysis considering static obstacles. Only valid for constant inital state!
        :param max_time: maximal considered time horizon (can be larger than prediction horizon)
        :return:
        """
        self.initial_state_locked = True
        self.backward_steps(max_time)
        self._current_time_step(max_time)

    def backward_steps(self, time_steps):
        for i_t in range(0, time_steps):
            if self._current_time_step <= 1:
                continue
            self.backward_step()

    def backward_forward(self, n_multisteps):
        self._current_time_step -= 1
        self.backward_steps(self.n_time_steps - 1)
        self.forward_steps_repeat(n_multisteps)
        self.backward_steps(self.n_time_steps - 1)

    def forward_steps_repeat(self, n_multisteps):
        for i_t in range(self._current_time_step, self._max_time_step):
            self._current_time_step = i_t
            reachability_grid_prop_pruned = self.sparse_propagation(n_multisteps, None, repeated=True)
            self._reachability_grid_list[self._current_time_step + 1] = np.logical_and(
                self._reachability_grid_list[self._current_time_step + 1], reachability_grid_prop_pruned)
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
            v_init = initial_state.velocity * np.array([[initial_state.velocity * np.cos(initial_state.orientation), ],
                                                        [initial_state.velocity * np.sin(initial_state.orientation), ]])
            self.parameters_online.update({'d_t': self.dt})

            reachset_translation_tmp: np.array = pos_init + v_init * self.parameters_online[
                'd_t'] * self._current_time_step
            self._reachset_translation_list[self._current_time_step] = reachset_translation_tmp
            # get occupied cells from obstacles
            occupied_grid_obs = self.obstacle_grid.get_occupancy_grid_at_time(self._current_time_step,
                                                                              reachset_translation_tmp, np.ones(
                    self.grid_shapes_all[self._current_time_step]))
            self._reachability_grid_list[self._current_time_step] = occupied_grid_obs.reshape([-1, 1])

            # relative translation of *propagated* reachable set (-> needs to be translated by -reachset_translation_tmp)
            initial_state: State = self.parameters_online['initial_state']
            pos_init = np.array(initial_state.position)
            v_init = initial_state.velocity * np.array([[initial_state.velocity * np.cos(initial_state.orientation), ],
                                                        [initial_state.velocity * np.sin(initial_state.orientation), ]])
            reachset_translation_tmp: np.array = pos_init + v_init * self.parameters_online['d_t'] * (
                        self._current_time_step - 1)
            self._reachset_translation_list[self._current_time_step - 1] = reachset_translation_tmp

        reachability_grid_prop = self._reachability_matrix_offline_list[self._current_time_step - 1].transpose() * \
                                 self._reachability_grid_list[self._current_time_step].reshape([-1, 1])

        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # get occupied cells from obstacles
        if self.occupied_grid_obs[self._current_time_step - 1] is not None:
            occupied_grid_obs = self.occupied_grid_obs[self._current_time_step - 1]
        else:
            # occupancy has not been computed before (pure bw reachability)
            occupied_grid_obs = self.obstacle_grid.get_occupancy_grid_at_time(self._current_time_step - 1,
                                                                              reachset_translation_tmp,
                                                                              reachability_grid_prop.reshape(
                                                                                  self.grid_shapes_all[
                                                                                      self._current_time_step - 1]))
        # intersect propagated cells with occupied cells
        reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                       occupied_grid_obs.reshape([-1, 1]))
        self._reachability_grid_list[self._current_time_step - 1] = np.logical_and(
            self._reachability_grid_list[self._current_time_step - 1], reachability_grid_prop_pruned)
        # self._reachability_grid_list[self._current_time_step - 1] = np.logical_and(self._reachability_grid_list[self._current_time_step - 1], reachability_grid_prop.reshape([-1, 1]))
        self._current_time_step -= 1

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

    def create_reachset_list_for_plotting(self, inverse=False, time_range: List[int] = [0, -1],
                                          seperate_time_steps=False):
        """

        :param inverse:
        :param time_range:
        :param seperate_time_steps: if true create new list element for each time step, otherwise all rects in one list
        :return:
        """
        tmp_rectlist = list()  # List[scenario.commonroad.shapes.Polygon]
        if seperate_time_steps:
            rectlist = list()
        for i, range_i in enumerate(time_range):
            if range_i < 0:
                time_range[i] = range_i + self._max_time_step + 1

        if inverse == True:
            reachability_grid_tmp = reversed(self._reachability_grid_list)
        else:
            reachability_grid_tmp = self._reachability_grid_list

        r_x = self.grid_x / 2
        r_y = self.grid_y / 2
        for i_t, reachability_grid in enumerate(reachability_grid_tmp):
            if reachability_grid is None:
                warnings.warn(
                    '<create_reachset_list_for_plotting>: max. number of propagated timesteps was {}'.format(i_t - 1),
                    stacklevel=2)
                break

            if self.velocity is True:
                # convert from 4d to 2d
                reachability_grid = (self._projection_2to4D[i_t].T).dot(reachability_grid)

            # reshape matrix from column shape to 2d shape
            n_ = int(np.sqrt(reachability_grid.shape[0]))
            reachability_grid = reachability_grid.toarray().reshape(self.grid_shapes_2d[i_t])

            if inverse is True:
                i_t = len(self._reachability_grid_list) - i_t - 1

            if i_t in range(time_range[0], time_range[1] + 1):
                trans_x = self._reachset_translation_list[i_t][0] + self.grid_reference_ll[i_t][0]# + r_x
                trans_y = self._reachset_translation_list[i_t][1] + self.grid_reference_ll[i_t][1]# + r_y
                for ix in range(0, reachability_grid.shape[0]):
                    for iy in range(0, reachability_grid.shape[1]):
                        if reachability_grid[ix, iy] == True:
                            tmp_rectlist.append(
                                Rectangle(self.grid_x, self.grid_y, center=np.array([trans_x + ix * self.grid_x, trans_y + iy * self.grid_y])))
                            # rectlist.append((self.reachboxes_list[i_t][self.reach_set_grid_list[i_t][ix,iy][0]]).translate_rotate(list(self._reachset_translation_list[i_t].tolist()),0.0))
                if seperate_time_steps:
                    rectlist.append(tmp_rectlist)
                    tmp_rectlist = list()

        if seperate_time_steps == False:
            rectlist = tmp_rectlist

        return rectlist