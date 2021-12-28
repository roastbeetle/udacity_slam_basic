import numpy as np
grid = np.array([[0,0,0,0,0,0],
                 [0,0,0,0,0,0],
                 [0,1,0,0,0,0],
                 [0,1,0,0,1,0],
                 [0,1,0,0,1,0]])

size = grid.shape
print size

init = [3,3,0]                      # x y head
cost = [1,1,20]                     # right straight left
goal = [2,0]

delta = [[-1,0],[0,-1],[1,0],[0,1]]
delta_name = ['^','<','v','>']

action = [-1,0,1]
action_name = ['R','#','L']

def optimum_policy2D(init, goal, grid, cost):
    x = init[0]
    y = init[1]
    ori = init[2]
    values = np.full([4,size[0],size[1]],999)
    policy = np.full([4,size[0],size[1]],' ')
    policy2D = np.full(size,' ')
    change = True
    while change:
        change = False
        for x in range(size[0]):
            for y in range(size[1]):
                for orientation in range(4):
                    if goal[0]==x and goal[1]==y:
                        if values[orientation][x][y]>0:
                            change = True
                            values[orientation][x][y]=0
                            policy[orientation][x][y]='*'
                    elif grid[x][y] == 0:
                        for i in range(3):
                            o2 = (orientation + action[i])%4
                            x2 = x + delta[o2][0]
                            y2 = y + delta[o2][1]

                            if x2>=0 and x2<size[0] and y2>=0 and y2<size[1] and grid[x2][y2]==0:
                                v2 = values[o2][x2][y2] + cost[i]
                                if v2<values[orientation][x][y]:
                                    values[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[i]
                                    change = True
    x = init[0]
    y = init[1]
    ori = init[2]

    policy2D[x][y] = policy[ori][x][y]
    while policy[ori][x][y] != '*':
        if policy[ori][x][y]=='#':
            o2 = ori
        elif policy[ori][x][y]=='R':
            o2 = (ori-1)%4
        if policy[ori][x][y]=='L':
            o2 = (ori+1)%4
        x = x + delta[o2][0]
        y = y + delta[o2][1]
        ori = o2
        policy2D[x][y] = policy[ori][x][y]
    print policy2D
    return

optimum_policy2D(init, goal, grid, cost)
