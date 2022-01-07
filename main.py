import numpy as np
import pybullet as p
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot as Robot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.pybullet.pb_utils.pybullet_scene_object import PyBulletObject

maze_grid = [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 1, 1, 0],
             [0, 1, 1, 0]]


def main():
    maze_pos = [0.5, -0.1, 0.91]
    maze = PyBulletObject(urdf_name='maze',
                          object_name='maze',
                          position=maze_pos,
                          orientation=[0, 0, 0],
                          data_dir=None)
    cs1_offset = [0.15, -0.06, 0.02]
    stick_pos = list(map(sum, zip(maze_pos, cs1_offset)))
    stick = PyBulletObject(urdf_name='stick',
                          object_name='stick',
                          position=stick_pos,
                          orientation=[0, 0, 0],
                          data_dir=None)

    object_list = [maze, stick]
    scene = Scene(object_list=object_list)

    PyBulletRobot = Robot(p, scene, gravity_comp=True)
    PyBulletRobot.use_inv_dyn = False

    init_pos = PyBulletRobot.current_c_pos
    init_or = PyBulletRobot.current_c_quat
    init_joint_pos = PyBulletRobot.current_j_pos

    PyBulletRobot.startLogging()
    # duration = 4
    duration = 2

    PyBulletRobot.ctrl_duration = duration
    PyBulletRobot.set_gripper_width = 0.04

    # move to the position 10cm above the object
    desired_cart_pos_1 = np.array(stick_pos) + np.array([-0.005, 0, 0.01])
    # desired_quat_1 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_1 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)

    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_1, desiredQuat=desired_quat_1, duration=duration)
    # there is no gripper controller. The desired gripper width will be executed right after the next controller
    # starts
    PyBulletRobot.set_gripper_width = 0.0

    # close the gripper and lift up the object
    desired_cart_pos_2 = desired_cart_pos_1 + np.array([0., 0, 0.02])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_2, desiredQuat=desired_quat_1, duration=duration)

    desired_cart_pos_3 = desired_cart_pos_2 + np.array([-0.12, 0, 0])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_3, desiredQuat=desired_quat_1, duration=duration)

    desired_cart_pos_4 = desired_cart_pos_3 + np.array([0, 0.12, 0])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_4, desiredQuat=desired_quat_1, duration=duration)

    desired_cart_pos_5 = desired_cart_pos_4 + np.array([0.12, 0, 0])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_5, desiredQuat=desired_quat_1, duration=duration)

    # get camera image
    robot_id = PyBulletRobot.robot_id
    client_id = scene.physics_client_id
    scene.inhand_cam.get_image(cam_id=robot_id, client_id=client_id)
    # scene.cage_cam.get_image(cam_id=robot_id, client_id=client_id)
    print("show camera image")

    PyBulletRobot.stopLogging()

    # PyBulletRobot.logger.plot(RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS)


if __name__ == '__main__':
    main()
