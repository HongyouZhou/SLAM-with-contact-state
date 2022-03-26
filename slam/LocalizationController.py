import json
import os

import matplotlib.pyplot as plt
import yaml

import numpy as np
import pybullet as p
import networkx as nx
from MappingController import MappingController
import matplotlib.pyplot as plt
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot as Robot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.pybullet.pb_utils.pybullet_scene_object import PyBulletObject
from scipy.interpolate import make_interp_spline

import matplotlib.animation as animation
from matplotlib import style
from threading import Thread


class LocalizationController:
    # this class is used to perform localization and control the robot
    def __init__(self, config, mapping_controller: MappingController):
        self.config = config
        self.mapping_controller = mapping_controller  # Mapping class
        self.curt_cs = 1  # We should always perform exploration to estimate the cs
        self.num_actions = config["num_actions"]
        self.measurement_model = config["cs_measurement_model"]
        self.cs_action_map = config["cs_action_map"]
        self.action_to_measurement = config["action_to_measurement"]
        self.measurement_to_action = config["measurement_to_action"]
        # here we use array to represent belief state, which means the position of element in the array
        # represents the cs state, but note that there is no 0 cs, so to get cs from belief state array
        # we should always add one

        # controller parameters
        self.explore_vel_scalar = config["explore_vel_scalar"]

    def reduce_uncertainty(self):
        """
        select actions based on current belief state.
        """
        pass

    def get_action_by_belief(self) -> list:
        """
        based on the belief state to select most informative action
        :return: desired_actions
        """
        # Side effects:
        #   1. self.curt_cs
        # based on the uncertainty, we should evaluate what is the next action to be executed
        pass

    def predict(self):
        # update belief state.
        # new belief state = belief state * likelihood + white noise, we first ignore the influence of action
        noise_factor = 0.1
        pre_belief_state = self.belief_state + noise_factor * np.ones_like(self.belief_state)
        self.belief_state = pre_belief_state / sum(pre_belief_state)

    def step(self):
        # Side effects:
        #   1. self.last_desired_action
        #   2. self.belief_state
        #   3. self.belief_state
        # first we should predict and update belief_state
        pass

    def update(self, execution_results):
        """ base_trans_ee: observed displacement
        :param execution_results:
        :returns:
        """
        # Side effects:
        #   1. self.last_desired_action
        #   2. self.belief_state
        pass

    def get_measurement_model_mat(self) -> np.ndarray:
        pass

    def local_exploration(self, init_state=None) -> dict:
        """
        :param init_state:
        :returns: execution_results
        """
        # Side effects:
        #   1. self.loc_state_collector
        #   2. self.arm_ctrl
        pass

    def do_action(self, desired_action, init_state=None) -> dict:
        """
        :param desired_action: list, [[motion_direction_1], [motion_direction_2]...]
        :param init_state:
        :returns: execution_results
        """
        # Side effects:
        #   1. self.loc_state_collector
        #   2. self.arm_ctrl
        # save action execution result as a dictionary
        pass

    def run(self):
        pass


if __name__ == '__main__':
    import matplotlib

    matplotlib.use('TkAgg')
    pass
