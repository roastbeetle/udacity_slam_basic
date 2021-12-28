import numpy as np
grid = np.array([[0, 1, 1, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0],
                 [0, 1, 1, 0]])

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']
size = grid.shape

def stochastic_value(init,grid,goal,cost_step,collision_cost,success_prob):
    failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
    values = np.full(size,collision_cost)
    policy = np.full(size,' ')
    x = init[0]
    y = init[1]
    change = True
    while change:
        change = False
        for x in range(size[0]):
            for y in range(size[1]):
                if goal[0]==x and goal[1]==y:   # find goal
                    if values[x][y]>0:
                        change=True
                        values[x][y]=0
                        policy[x][y]='*'
                elif grid[x][y] == 0:
                    for a in range(4):
                        v2 = cost_step
                        for i in range(-1,2):   # for every possible motion
                            a2 = (a+i)%4
                            x2 = x+delta[a2][0]
                            y2 = y+delta[a2][1]

                            if i==0:            # upward movement
                                p2 = success_prob
                            else:               # another movement
                                p2 = (1.0 - success_prob)/2.0
                            if x2>=0 and x2<size[0] and y2>=0 and y2<size[1] and grid[x2][y2]==0:   # if next step is empty gridcell
                                v2 += p2*values[x2][y2]
                            else:                                                                   # at wall
                                v2 += p2*collision_cost
                        if v2 < values[x][y]:   # need the lowest cost
                            change = True
                            values[x][y] = v2
                            policy[x][y] = delta_name[a]

    return values, policy

init = [0,0]
goal = [0, len(grid[0])-1] # Goal is in top right corner
cost_step = 1
collision_cost = 1000
success_prob = 0.5

value,policy = stochastic_value(init,grid,goal,cost_step,collision_cost,success_prob)

print value
print policy
