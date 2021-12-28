from math import *
import numpy as np

class plan:
    def __init__(self, grid, init, goal, cost = 1):
        self.cost = cost
        self.grid = grid
        self.gshape = grid.shape
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []
        self.pshape = [] 

    def make_heuristic(self, grid, goal, cost):        
        self.heuristic = np.zeros(self.gshape)
        for i in range(self.gshape[0]):
            for j in range(self.gshape[1]):
                self.heuristic[i][j] = abs(i - self.goal[0]) + abs(j - self.goal[1])

############# A* algorithm #############################
    def astar(self):
        if self.heuristic == []:
            raise ValueError("Heuristic must be defined to run A*")
        
        delta = [[-1,  0], # go up
                 [ 0,  -1], # go left
                 [ 1,  0], # go down
                 [ 0,  1]] # do right

        # open list elements are of the type: [f, g, h, x, y]
        
        closed = np.zeros(self.gshape)
        action = np.zeros(self.gshape, dtype = int)
        closed[self.init[0]][self.init[1]] = 1  # init point

        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g + h

        open = [[f, g, h, x, y]]

        found  = False # flag that is set when search complete
        resign = False # flag set if we can't find expand
        count  = 0

        while not found and not resign:
            if len(open) == 0:
                resign = True
                print('###### Search terminated without success')
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]


            if x == self.goal[0] and y == self.goal[1]:
                found = True
                #print('###### A* search successful')

            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2>=0 and x2<self.gshape[0] and y2>=0 and y2<self.gshape[1]:
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            g2 = g + self.cost
                            h2 = self.heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
            count += 1

        invpath = []
        x = self.goal[0]
        y = self.goal[1]
        invpath.append([x, y])
        while x != self.init[0] or y != self.init[1]:
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            x = x2
            y = y2
            invpath.append([x, y])

        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])

######### smooth algorithm ###########################
    def smooth(self, weight_data = 0.1, weight_smooth = 0.1, 
               tolerance = 0.000001):

        if self.path == []:
            raise ValueError("Run A* first before smoothing path")
        
        self.path = np.array(self.path)
        self.pshape = self.path.shape
        self.spath = np.zeros(self.pshape)
        for i in range(self.pshape[0]):
            for j in range(self.pshape[1]):
                self.spath[i][j] = self.path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, self.pshape[0]-1):
                for j in range(self.pshape[1]):
                    aux = self.spath[i][j]
                    
                    self.spath[i][j] += weight_data*(self.path[i][j] - self.spath[i][j])
                    
                    self.spath[i][j] += weight_smooth*(self.spath[i-1][j] + self.spath[i+1][j] - (2.0 * self.spath[i][j]))
                    
                    if i >= 2:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i-1][j] - self.spath[i-2][j] - self.spath[i][j])
                    if i <= len(self.path) - 3:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i+1][j] - self.spath[i+2][j] - self.spath[i][j])
                
            change += abs(aux - self.spath[i][j])