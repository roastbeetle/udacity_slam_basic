import numpy as np
grid = np.array([[0,0,0,0,0,0],
                 [0,0,0,0,0,0],
                 [0,1,0,0,0,0],
                 [0,1,0,0,1,0],
                 [0,1,0,0,1,0]])

size = grid.shape
print size

init = [3,3]                      # x y head
cost = 1                     # right straight left
goal = [2,0]

delta = [[-1,0],[0,-1],[1,0],[0,1]]
delta_name = ['^','<','v','>']

def compute_value(init, grid, goal, cost):
    x = init[0]
    y = init[1]
    values = np.full(size,99)
    policy = np.full(size,' ')
    change = True
    while change:
        change = False
        for x in range(size[0]):
            for y in range(size[1]):
                if goal[0]==x and goal[1]==y:
                    if values[x][y] >0:         # find goal
                        values[x][y] = 0
                        change = True
                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x+delta[a][0]
                        y2 = y+delta[a][1]
                        if x2>=0 and x2<size[0] and y2>=0 and y2<size[1] and grid[x2][y2] == 0:
                            v2 = values[x2][y2] + cost
                            if v2<values[x][y]:
                                change = True
                                values[x][y] = v2
                                policy[x][y] = delta_name[a]
    print policy
    return values

print compute_value(init,grid,goal,cost)
