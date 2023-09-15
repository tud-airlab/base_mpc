import os
import re
import yaml

import numpy as np
from planner.mpcPlanner import MPCPlanner

def parse_setup(setup_file: str):
    with open(setup_file, "r") as setup_stream:
        setup = yaml.safe_load(setup_stream)
    return setup

envMap = {
    'planarArm': 'nLink-reacher-acc-v0',
    'diffDrive': 'ground-robot-acc-v0',
    'po1ntRobot': 'point-robot-acc-v0',
    'boxer': 'boxer-robot-acc-v0',
    'panda': 'panda-reacher-acc-v0',
}

class BaseMpcExample(object):
    def __init__(self, config_file_name: str):
        test_setup = os.path.dirname(os.path.realpath(__file__)) + "/" + config_file_name
        self._robot_type = re.findall('\/(\S*)M', config_file_name)[0]
        self._solver_directory = os.path.dirname(os.path.realpath(__file__)) + '/..'+  "/solvers"
        self._env_name = envMap[self._robot_type]
        setup = parse_setup(test_setup)

        # setup planner
        self._planner = MPCPlanner(
             self._robot_type,
             self._solver_directory,
             **setup['mpc'])

        self._planner.reset()
        self._n = 3
        self._render = True

    def set_mpc_parameter(self):
        self._planner.setRobot()
        self._planner.setDynObstacles(self._obstacles)
        self._planner.setGoal(self._goal)
        self._planner.setJointLimits(np.transpose(self._limits))