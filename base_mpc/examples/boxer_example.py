import sys
import numpy as np
import gymnasium as gym
import os
import re
import yaml
from planner.mpcPlanner import MPCPlanner
from base_mpc_example import BaseMpcExample
from urdfenvs.robots.generic_urdf.generic_diff_drive_robot import GenericDiffDriveRobot

from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.goals.goal_composition import GoalComposition

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
class TestMPC(BaseMpcExample):

    def initialize_environment(self):
        goal_dict = {
            "subgoal0": {
                "weight": 1.0,
                "is_primary_goal": True,
                "indices": [0, 1],
                "parent_link": 'origin',
                "child_link": 'ee_link',
                "desired_position": [0.0, -6.0],
                "epsilon": 0.1,
                "type": "staticSubGoal"
            }
        }
        self._goal = GoalComposition(name="goal1", content_dict=goal_dict)
        static_obst_dict = {
            "type": "sphere",
            "geometry": {"position": [4.0, -0.5, 0.0], "radius": 1.0},
        }
        obst1 = SphereObstacle(name="staticObst1", content_dict=static_obst_dict)
        self._obstacles = [obst1]
        self._r_body = 0.6
        self._limits = np.array([
                [-10, 10],
                [-10, 10],
                [-10, 10],
        ])
        robots = [
            GenericDiffDriveRobot(
                urdf="boxer.urdf",
                mode="vel",
                actuated_wheels=["wheel_right_joint", "wheel_left_joint"],
                castor_wheels=["rotacastor_right_joint", "rotacastor_left_joint"],
                wheel_radius=0.08,
                wheel_distance=0.494,
                spawn_rotation=np.pi / 2,
            ),
        ]
        self._env = gym.make(
            'urdf-env-v0',
            render=self._render,
            robots = robots,
            dt=self._planner._config.time_step
        )

        def run(self):
            q0 = np.median(self._limits)
            ob = self._env.reset(pos=q0)
            for obstacle in self._obstacles:
                self._env.add_obstacle(obstacle)
            self._env.add_goal(self._goal)
            n_steps = 1000
            for i in range(n_steps):
                q = ob['joint_state']['position']
                qdot = ob['joint_state']['velocity']
                vel = np.array((ob['joint_state']['forward_velocity'], qdot[2]), dtype=float)
                action = self._planner.computeAction(q, qdot, vel)
                ob, *_ = self._env.step(action)

def main():
    boxer_example = TestMPC('config/boxerMPCconfig.yaml')
    boxer_example.initialize_environment()
    boxer_example.set_mpc_parameter()
    boxer_example.run()

if __name__ == "__main__":
    main()
