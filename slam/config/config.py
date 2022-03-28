import numpy as np

URDF_NAME = 'maze'
# Robot postion in maze grid
robot_grid_pos = [0, 0]
# Maze cartesian position
MAZE_POS = [0.5, -0.1, 0.91]
# Cs1 cartesian offset
CS1_OFFSET = [0.15, -0.06, 0.02]
DESIRED_QUAT = [0, 1, 0, 0]
DURATION = 0.5
MAZE_ORIGIN_OFFSET = np.array(MAZE_POS) + np.array(CS1_OFFSET)
Y_CART_STEP_SIZE = -0.04
X_CART_STEP_SIZE = 0.04
