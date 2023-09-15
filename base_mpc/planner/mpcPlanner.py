import sys
import numpy as np
sys.path.append('/home/luzia/code/forces_pro_client/')  # todo remove
import pickle
import forcespro.nlp
from dataclasses import dataclass


@dataclass
class MpcConfiguration: # to do move to solver genration
    planning_horizon: int
    time_step: float
    n_discs: int
    r_disc: float
    disc_offset: float
    n_dyn_obstacles: int
    n_static_constraints: int
    weights: dict
    modules: dict
    slack: bool
    model_name: str
    name: str = 'mpc'
    debug: bool = False

class EmptyObstacle():
    def position(self):
        return [-100, -100, -100]

    def radius(self):
        return 0.0

class MPCPlanner(object):

    def __init__(self, robot_type, solver_dir, **kwargs):


        self.solver = forcespro.nlp.Solver.from_directory(solver_dir + "/AlbertFORCESNLPsolver") #todo add robot_type

        with open(solver_dir + "/AlbertFORCESNLPsolver/Albertparams.pkl", 'rb') as file:
            data = pickle.load(file)
            self._properties = data['properties']
            self._map_runtime_par = data['map_runtime_par']
            self._modules = data['modules']

        self._config = MpcConfiguration(**kwargs)

        self._nx = self._properties['nx']
        self._nu = self._properties['nu']
        self._npar = self._properties['npar']

    def reset(self):
        print("RESETTING PLANNER")
        self._x0 = np.zeros(shape=(self._config.planning_horizon, self._nx + self._nu))
        self._xinit = np.zeros(self._nx)
        self._params = np.zeros(shape=(self._npar * self._config.planning_horizon), dtype=float)

        for i in range(self._config.planning_horizon):
            for key, val in self._map_runtime_par.items():
                if 'W' in key:
                    weight_key = str(key).replace('_index', '')
                    self._params[self._npar * i + val] = self._config.weights[weight_key]

    def setRobot(self):
        for i in range(self._config.planning_horizon):
            self._params[self._npar * i + self._map_runtime_par["disc_r"]] = self._config.r_disc
            for j in range(self._config.n_discs):
                self._params[self._npar * i + self._map_runtime_par["disc_offset_" + str(j)]] = self._config.disc_offset

    def setDynObstacles(self, obsts):
        self._dyn_obst_constraint_type = self._config.modules['dyn_coll_avoid'] + "_obst_"

        for i in range(self._config.planning_horizon):
            for j in range(self._config.n_dyn_obstacles):
                if j < len(obsts):
                    obst = obsts[j]
                else:
                    obst = EmptyObstacle()
                for module in self._modules:
                    if self._config.modules['dyn_coll_avoid'] == 'ellipsoid':
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_x']
                        self._params[idx] = obst.position()[0]
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_y']
                        self._params[idx] = obst.position()[1]
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_z']
                        self._params[idx] = obst.position()[2]
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_psi']
                        self._params[idx] = 0.0 # heading
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_major']
                        self._params[idx] = obst.radius()
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_minor']
                        self._params[idx] = obst.radius()
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_chi']
                        self._params[idx] = 1.0
                        idx = self._npar * i + self._map_runtime_par[self._dyn_obst_constraint_type + str(j) + '_r']
                        self._params[idx] = obst.radius()

    def setStaticObstacles:
        print("not implemented yet")

    def setGoal(self, goal):
        for i in range(self._config.time_horizon):
            for j in range(self.m()):
                if j >= len(goal.position()):
                    position = 0
                else:
                    position = goal.position()[j]
                self._params[self._npar * i + self._paramMap["g"][j]] = position
    def updateDynamicObstacles(self, obstArray):
        print('not implemented yet')
    def setx0(self, xinit):
        print('not implemented yet')
    def solve(self, current_state):
        #self._xinit = current_state[0: self._nx]
        self.setX0(self._xinit)
        print('not implemented yet')