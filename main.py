import numpy as np
import pybullet as p
import networkx as nx
import matplotlib.pyplot as plt
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot as Robot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.pybullet.pb_utils.pybullet_scene_object import PyBulletObject

MAZE_GRID = [[0, 1, 1, 0],
             [0, 1, 1, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0]]
robot_grid_pos = [0, 0]

MAZE_POS = [0.5, -0.1, 0.91]
CS1_OFFSET = [0.15, -0.06, 0.02]
Y_CART_STEP_SIZE = -0.04
X_CART_STEP_SIZE = 0.04
MAZE_ORIGIN_OFFSET = np.array(MAZE_POS) + np.array(CS1_OFFSET)

# Actions lookup table up right down left
ACTION_X_STEP = [0, 1, 0, -1]
ACTION_Y_STEP = [1, 0, -1, 0]
NUM_ACTIONS = 4
CONSTRAINT = [0, 1]  # 0 for CLOCKWISE, 1 for COUNTERCLOCKWISE
G = nx.DiGraph()
nodeAttrs = dict()
edgeAttrs = dict()
lastConstraint = 0
goalNode = None

duration = 0.5
maze = PyBulletObject(urdf_name='maze',
                      object_name='maze',
                      position=MAZE_POS,
                      orientation=[0, 0, 0],
                      data_dir=None)

stick_pos = list(map(sum, zip(MAZE_POS, CS1_OFFSET)))
stick = PyBulletObject(urdf_name='stick',
                       object_name='stick',
                       position=stick_pos,
                       orientation=[0, 0, 0],
                       data_dir=None)
desired_quat_1 = [0, 1, 0, 0]

object_list = [maze, stick]
scene = Scene(object_list=object_list)

PyBulletRobot = Robot(p, scene, gravity_comp=True)
PyBulletRobot.use_inv_dyn = False


def getCartPosFromIndex(x, y):
    return MAZE_ORIGIN_OFFSET + np.array([Y_CART_STEP_SIZE * y, X_CART_STEP_SIZE * x, 0.02])


def robotGotoIndex(pos: list):
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=getCartPosFromIndex(pos[0], pos[1]),
                                          desiredQuat=desired_quat_1,
                                          duration=duration)


def isWall(x, y):
    return x < 0 or x > len(MAZE_GRID[0]) - 1 or \
           y < 0 or y > len(MAZE_GRID) - 1 or \
           MAZE_GRID[y][x] == 1


def getMeasurment():
    res = []
    for a in range(NUM_ACTIONS):
        pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[a]
        pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[a]
        if isWall(pos_after_x, pos_after_y):
            res.append(0)
        else:
            res.append(1)
    return res


def constraintFollowing(constraint):
    global lastConstraint
    if constraint == 0:
        s = 1
    elif constraint == 1:
        s = -1
    else:
        return

    a = lastConstraint
    for i in range(0, NUM_ACTIONS):
        if constraint == 0:
            constraint_dir = (a - 1) % NUM_ACTIONS
        else:
            constraint_dir = (a + 1) % NUM_ACTIONS
        pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[a]
        pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[a]
        pos_constraint_x = pos_after_x + ACTION_X_STEP[constraint_dir]
        pos_constraint_y = pos_after_y + ACTION_Y_STEP[constraint_dir]
        if isWall(pos_after_x, pos_after_y) or not isWall(pos_constraint_x, pos_constraint_y):
            a = (a + s) % NUM_ACTIONS
            continue
        else:
            lastConstraint = (a - s) % NUM_ACTIONS
            robot_grid_pos[0] = pos_after_x
            robot_grid_pos[1] = pos_after_y
            robotGotoIndex(robot_grid_pos)
            return keepMoingInDirection(a, constraint)


def keepMoingInDirection(direction, constraint):
    if constraint == 0:
        constraint_dir = (direction - 1) % NUM_ACTIONS
    else:
        constraint_dir = (direction + 1) % NUM_ACTIONS
    pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[direction]
    pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[direction]
    pos_constraint_x = robot_grid_pos[0] + ACTION_X_STEP[constraint_dir]
    pos_constraint_y = robot_grid_pos[1] + ACTION_Y_STEP[constraint_dir]
    while not isWall(pos_after_x, pos_after_y) and isWall(pos_constraint_x, pos_constraint_y):
        robot_grid_pos[0] = pos_after_x
        robot_grid_pos[1] = pos_after_y
        robotGotoIndex(robot_grid_pos)
        pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[direction]
        pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[direction]
        pos_constraint_x = robot_grid_pos[0] + ACTION_X_STEP[constraint_dir]
        pos_constraint_y = robot_grid_pos[1] + ACTION_Y_STEP[constraint_dir]
    return constraint_dir


def moveUntilWall(direction):
    pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[direction]
    pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[direction]
    while not isWall(pos_after_x, pos_after_y):
        robot_grid_pos[0] = pos_after_x
        robot_grid_pos[1] = pos_after_y
        robotGotoIndex(robot_grid_pos)
        pos_after_x = robot_grid_pos[0] + ACTION_X_STEP[direction]
        pos_after_y = robot_grid_pos[1] + ACTION_Y_STEP[direction]


def dummyHash(pos, measurement):
    return ''.join(str(e) for e in pos + measurement)


def initialMapping():
    measurement = getMeasurment()
    startNode = dummyHash(robot_grid_pos, measurement)
    nodeAttrs[startNode] = {"pos": robot_grid_pos, "measurement": measurement, "CS": len(G.nodes())}
    G.add_node(startNode)
    lastNode = startNode
    # Find all node
    while 1:
        constraint_dir = constraintFollowing(0)
        measurement = getMeasurment()
        curNode = dummyHash(robot_grid_pos, measurement)
        if curNode in nodeAttrs:
            G.add_edge(lastNode, curNode)
            edgeAttrs[(lastNode, curNode)] = constraint_dir
            break
        else:
            nodeAttrs[curNode] = {"pos": robot_grid_pos, "measurement": measurement, "CS": len(G.nodes())}
            G.add_node(curNode)
            G.add_edge(lastNode, curNode)
            edgeAttrs[(lastNode, curNode)] = constraint_dir
            lastNode = curNode

    lastNode = startNode
    while 1:
        constraint_dir = constraintFollowing(1)
        measurement = getMeasurment()
        curNode = dummyHash(robot_grid_pos, measurement)
        if curNode not in nodeAttrs:
            return
        else:
            G.add_edge(lastNode, curNode)
            edgeAttrs[(lastNode, curNode)] = constraint_dir
            lastNode = curNode
        if curNode == startNode:
            break
    print("success!")


def findAllByMeasurement(measurement):
    res = []
    for i in nodeAttrs:
        if nodeAttrs[i]["measurement"] == measurement:
            res.append(i)
    return res


def containsGoalNode(potentialNode):
    for n in potentialNode:
        if n == goalNode:
            return True
    return False


def findParentByConstraint(edges, constraint):
    for u, v in edges:
        if edgeAttrs[(u, v)] == constraint:
            return u
    return None


# [1010]
def pickOneDirection(measurement):
    constraintList = []
    for i, d in enumerate(measurement):
        if d == 1:
            if measurement[(i - 1) % NUM_ACTIONS] == 0:
                constraintList.append((i - 1) % NUM_ACTIONS)
            if measurement[(i + 1) % NUM_ACTIONS] == 0:
                constraintList.append((i + 1) % NUM_ACTIONS)
            return i, constraintList
    return None


def slam():
    global robot_grid_pos
    # robot_grid_pos[0] = 0
    # robot_grid_pos[1] = 2
    # robotGotoIndex(robot_grid_pos)
    # robot_grid_pos[0] = 1
    # robot_grid_pos[1] = 2
    # robotGotoIndex(robot_grid_pos)

    measurement = getMeasurment()
    potentialNode = findAllByMeasurement(measurement)
    preNode = None
    first_constraint = None
    if len(potentialNode) == 0:
        curNode = dummyHash(robot_grid_pos, measurement)
        nodeAttrs[curNode] = {"pos": robot_grid_pos, "measurement": measurement, "CS": len(G.nodes())}
        G.add_node(curNode)

        moveUntilWall(0)

        measurement = getMeasurment()
        potentialNode = findAllByMeasurement(measurement)
        preNode = curNode
        while len(potentialNode) == 0:
            curNode = dummyHash(robot_grid_pos, measurement)
            nodeAttrs[curNode] = {"pos": robot_grid_pos, "measurement": measurement, "CS": len(G.nodes())}
            G.add_node(curNode)
            G.add_edge(preNode, curNode)
            preNode = curNode
            d, constraintList = pickOneDirection(measurement)
            keepMoingInDirection(d, constraintList[0])  # Heuristic
            first_constraint = constraintList[0]
            measurement = getMeasurment()
            potentialNode = findAllByMeasurement(measurement)

    path = []
    while len(potentialNode) != 1:
        constraint_dir = constraintFollowing(0)
        path.append(constraint_dir)
        measurement = getMeasurment()
        potentialNode = findAllByMeasurement(measurement)

    if preNode is not None:
        targetNode = potentialNode[0]
        while len(path) != 0:
            constraint_dir = path.pop()
            targetNode = findParentByConstraint(G.in_edges(targetNode), constraint_dir)
        G.add_edge(preNode, targetNode)
        edgeAttrs[(preNode, targetNode)] = first_constraint

    return potentialNode[0]


def solveMaze():
    return None



def initRobot():
    # init_pos = PyBulletRobot.current_c_pos
    # init_or = PyBulletRobot.current_c_quat
    # init_joint_pos = PyBulletRobot.current_j_pos

    PyBulletRobot.ctrl_duration = duration
    PyBulletRobot.set_gripper_width = 0.04

    # move to the position 10cm above the object
    desired_cart_pos_1 = np.array(stick_pos) + np.array([-0.005, 0, 0.01])
    # desired_quat_1 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_1 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)

    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_1, desiredQuat=desired_quat_1, duration=4)
    # there is no gripper controller. The desired gripper width will be executed right after the next controller
    # starts
    PyBulletRobot.set_gripper_width = 0.0

    # close the gripper and lift up the object
    desired_cart_pos_2 = desired_cart_pos_1 + np.array([0., 0, 0.02])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_2, desiredQuat=desired_quat_1, duration=4)

    desired_cart_pos_3 = getCartPosFromIndex(robot_grid_pos[0], robot_grid_pos[1])
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_3, desiredQuat=desired_quat_1, duration=4)


def main():
    PyBulletRobot.startLogging()

    initRobot()
    initialMapping()
    goalNode = "031000"
    slam()
    pos = nx.spring_layout(G, seed=225)  # Seed for reproducible layout
    # nx.draw(G, pos, labels={node: node for node in G.nodes()})
    nx.draw(G, pos, labels={node: nodeAttrs[node]["CS"] for node in G.nodes()})
    nx.draw_networkx_edge_labels(
        G, pos,
        edge_labels=edgeAttrs,
        font_color='red'
    )
    plt.show()

    PyBulletRobot.stopLogging()

    # PyBulletRobot.logger.plot(RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS)


if __name__ == '__main__':
    main()
