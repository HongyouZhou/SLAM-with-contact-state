import json

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import yaml

from MappingController import MappingController
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot as Robot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.pybullet.pb_utils.pybullet_scene_object import PyBulletObject
from config.config import *


class LocalizationController:
    # this class is used to perform localization and control the robot
    def __init__(self, config, mapping_controller: MappingController, scene: Scene, pybullet_robot: Robot):
        self.config = config
        self.scene = scene
        self.robot = pybullet_robot
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
        self.belief_state = np.ones(len(self.mapping_controller.all_css)) / len(self.mapping_controller.all_css)
        self.last_desired_action = None

        # controller parameters
        self.explore_scalar = config["explore_scalar"]

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
        desired_actions = []
        if self.last_desired_action is not None:
            desired_action = self.last_desired_action
            desired_actions.append(desired_action)
        elif np.max(self.belief_state) >= 0.5:
            # enough certainty to pick deterministic action to go to the next state
            self.curt_cs = np.argmax(self.belief_state) + 1
            desired_action = self.cs_action_map[self.curt_cs]
            desired_actions.append(desired_action)
        else:
            to_clarify_css = np.squeeze(np.argwhere(self.belief_state >= 0.2) + 1)
            distinct_measurements = []
            base_cs = np.argmax(to_clarify_css)
            for cs in to_clarify_css:
                if cs != base_cs:
                    # find the most informative action to distinguish the belief state
                    measurement_model_mat = self.get_measurement_model_mat()
                    diff_in_measurement = measurement_model_mat[base_cs - 1] - measurement_model_mat[cs - 1]
                    informative_action = np.nonzero(diff_in_measurement)[0]
                    if informative_action.size != 0:
                        distinct_measurements = distinct_measurements + list(informative_action)
                # remove the same measurements
                distinct_measurements = list(set(distinct_measurements))

                # random pick a desired measurement to be validated
                desired_measurement = np.random.choice(distinct_measurements)

                # map the desired measurement to action
                desired_action = self.measurement_to_action[desired_measurement]
                desired_actions.append(desired_action)

        return desired_actions

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
        self.predict()
        # then execute an action
        desired_action = self.get_action_by_belief()
        execution_results = self.do_action(desired_action=desired_action)
        self.last_desired_action = desired_action
        # use the observation to update the belief
        self.update(execution_results)

    def update(self, execution_results):
        """ base_trans_ee: observed displacement
        :param execution_results:
        :returns:
        """
        # Side effects:
        #   1. self.last_desired_action
        #   2. self.belief_state
        desired_actions = []
        base_trans_ee_s = []
        dists = []
        for key in execution_results.keys():
            desired_actions.append(json.loads(key))
            before_p_ee = execution_results[key]["before_p_ee"]
            after_p_ee = execution_results[key]["after_p_ee"]
            base_trans_ee = after_p_ee[:3] - before_p_ee[:3]
            base_trans_ee_s.append(base_trans_ee)
            dists.append(np.linalg.norm(base_trans_ee))

        measurement_model_mat = self.get_measurement_model_mat()
        for i, base_trans_ee in enumerate(base_trans_ee_s):
            desired_action = desired_actions[i]

            moved_distance = np.linalg.norm(base_trans_ee)
            if moved_distance >= self.config["explore_min_dist_threshold"]:
                self.last_desired_action = desired_action
                norm_base_trans_ee = base_trans_ee / np.linalg.norm(base_trans_ee)
                # get measurement from three axes
                # for x
                if norm_base_trans_ee[0] > 0:
                    measurement_x = norm_base_trans_ee[0] * measurement_model_mat[:, 2]
                else:
                    measurement_x = abs(norm_base_trans_ee[0]) * measurement_model_mat[:, 3]
                # for y
                if norm_base_trans_ee[1] > 0:
                    measurement_y = norm_base_trans_ee[1] * measurement_model_mat[:, 4]
                else:
                    measurement_y = abs(norm_base_trans_ee[1]) * measurement_model_mat[:, 5]
                # for z
                if norm_base_trans_ee[2] > 0:
                    measurement_z = norm_base_trans_ee[2] * measurement_model_mat[:, 0]
                else:
                    measurement_z = abs(norm_base_trans_ee[2]) * measurement_model_mat[:, 1]

                sum_measurement = measurement_x + measurement_y + measurement_z
                self.belief_state = sum_measurement * self.belief_state
                self.belief_state = self.belief_state / sum(self.belief_state)

            else:
                # there is no displacement in desired action direction, this is another type of measurement
                # map the desired action back to the measurement model
                measurement_index = self.action_to_measurement[str(desired_action)]
                # because desired action cannot result in motion, we use the negative measurement model mat to
                # update the belief states
                sum_measurement = (1 - measurement_model_mat[:, measurement_index]) + 0.005  # plus noise term
                self.belief_state = sum_measurement * self.belief_state
                self.belief_state = self.belief_state / sum(self.belief_state)
                self.last_desired_action = None

    def get_measurement_model_mat(self) -> np.ndarray:
        measurement_model_mat = np.zeros([self.mapping_controller.num_cs, self.num_actions])
        for key in sorted(self.measurement_model):
            measurement_model_mat[key - 1, :] = self.measurement_model[key]
        return measurement_model_mat

    def local_exploration(self, init_p_ee=None) -> dict:
        """
        :param init_p_ee:
        :returns: execution_results
        """
        # Side effects:
        #   1. self.loc_state_collector
        #   2. self.arm_ctrl
        if init_p_ee is None:
            init_p_ee = np.array(self.robot.get_end_effector_pos())

        # save local exploration result as a dictionary
        exploration_results = dict()
        exploration_actions = self.config["measurement_to_action"].values()
        for explore in exploration_actions:
            # concert desired action to string so as to set it as key
            exploration_results[str(explore)] = dict()
            d_base_p_ee = (self.explore_scalar * np.array(explore)) + np.array(init_p_ee)
            # Record measurement before
            exploration_results[str(explore)]["before_p_ee"] = self.robot.get_end_effector_pos()
            # Do exploration
            self.robot.gotoCartPositionAndQuat(desiredPos=d_base_p_ee, desiredQuat=DESIRED_QUAT, duration=DURATION)
            # Record measurement after
            exploration_results[str(explore)]["after_p_ee"] = self.robot.get_end_effector_pos()
            # Back to original position
            self.robot.gotoCartPositionAndQuat(desiredPos=init_p_ee, desiredQuat=DESIRED_QUAT, duration=DURATION)

        return exploration_results

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
        execution_results = dict()
        execution_results[str(desired_action)] = dict()
        # concert desired action to string so as to set it as key
        execution_time = self.config["action_execution_time"]

        d_base_p_ee = (self.explore_scalar * np.array(desired_action)) + np.array(self.robot.get_end_effector_pos())
        # Record measurement before
        execution_results[str(desired_action)]["before_p_ee"] = self.robot.get_end_effector_pos()
        # Do exploration
        self.robot.gotoCartPositionAndQuat(desiredPos=d_base_p_ee, desiredQuat=DESIRED_QUAT, duration=DURATION)
        # Record measurement after
        execution_results[str(desired_action)]["after_p_ee"] = self.robot.get_end_effector_pos()

        return execution_results

    def run(self):
        self.initRobot()

        ax = plt.gca()
        x = np.arange(len(self.belief_state))
        desired_action = None
        init_cs_state = self.config["init_state"]
        self.belief_state[init_cs_state - 1] = 5
        # local exploration
        exploration_results = self.local_exploration()
        self.update(exploration_results)

        while True:
            self.curt_cs = np.argmax(self.belief_state) + 1
            # dist = np.linalg.norm(curt_state_df["base_p_ee"][:3] - end_pose)
            # first we should predict
            self.predict()
            desired_actions = self.get_action_by_belief()
            execution_results = self.do_action(desired_action=desired_actions[0])
            self.last_desired_action = desired_action
            # use the observation to update the belief
            self.update(execution_results)
            # bs_msg = Float64MultiArray()
            # bs_msg.data = self.belief_state
            # self.bs_publisher.publish(bs_msg)

    def getCartPosFromIndex(self, x, y):
        return MAZE_ORIGIN_OFFSET + \
               np.array([Y_CART_STEP_SIZE * y,
                         X_CART_STEP_SIZE * x, 0.02])

    def initRobot(self):
        # init_pos = PyBulletRobot.current_c_pos
        # init_or = PyBulletRobot.current_c_quat
        # init_joint_pos = PyBulletRobot.current_j_pos

        self.robot.ctrl_duration = 0.5
        self.robot.set_gripper_width = 0.04

        # move to the position 10cm above the object
        desired_cart_pos_1 = np.array(stick_pos) + np.array([-0.005, 0, 0.01])
        # desired_quat_1 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
        desired_quat_1 = [0, 1, 0,
                          0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)

        self.robot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_1, desiredQuat=desired_quat_1, duration=4)
        # there is no gripper controller. The desired gripper width will be executed right after the next controller
        # starts
        self.robot.set_gripper_width = 0.0

        # close the gripper and lift up the object
        desired_cart_pos_2 = desired_cart_pos_1 + np.array([0., 0, 0.02])
        self.robot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_2, desiredQuat=desired_quat_1, duration=4)

        desired_cart_pos_3 = self.getCartPosFromIndex(robot_grid_pos[0], robot_grid_pos[1])
        self.robot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_3, desiredQuat=desired_quat_1, duration=2)


if __name__ == '__main__':
    import matplotlib

    matplotlib.use('TkAgg')

    config = yaml.safe_load(open("./config/SlamConfig.yaml", "r"))
    mapping_controller = MappingController(config=config)

    maze = PyBulletObject(urdf_name=URDF_NAME,
                          object_name='maze',
                          position=MAZE_POS,
                          orientation=[0, 0, 0],
                          data_dir=None)
    # Initial stick model
    stick_pos = list(map(sum, zip(MAZE_POS, CS1_OFFSET)))
    stick = PyBulletObject(urdf_name='stick',
                           object_name='stick',
                           position=stick_pos,
                           orientation=[0, 0, 0],
                           data_dir=None)
    object_list = [maze, stick]
    # Initial scene
    scene = Scene(object_list=object_list)
    # Initial Robot
    PyBulletRobot = Robot(p, scene, gravity_comp=True)
    PyBulletRobot.use_inv_dyn = False
    loc = LocalizationController(config=config, mapping_controller=mapping_controller, scene=scene,
                                 pybullet_robot=PyBulletRobot)
    loc.run()

    print("finished")
