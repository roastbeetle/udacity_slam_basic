from math import *
from pathplan import *
from control import *

steering_noise    = 0.1  # 0.1
distance_noise    = 0.03 # 0.03
measurement_noise = 0.3  # 0.3
noise = [steering_noise, distance_noise, measurement_noise]

def main(grid, init, goal, noise, weight_data, weight_smooth, p_gain, d_gain):

    path = plan(grid, init, goal)
    path.astar()
    path.smooth(weight_data, weight_smooth)
    return run(grid, goal, path.spath, noise, [p_gain, d_gain])

    
grid = np.array([[0, 0, 0, 1, 0, 1],
                 [0, 1, 0, 1, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 1, 0, 1, 1, 0],
                 [0, 0, 0, 1, 0, 0]])

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]

steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3

weight_data       = 0.1
weight_smooth     = 0.075
p_gain            = 2.015625
d_gain            = 6.0

print(main(grid, init, goal, noise, weight_data, weight_smooth, p_gain, d_gain))






