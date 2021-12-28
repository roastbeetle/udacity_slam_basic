from particlefilter import *
from robot import *
import matplotlib.pyplot as plt

def run(grid, goal, spath, noise, params, printflag = True, speed = 0.1, timeout = 1000):

    myrobot = robot()
    myrobot.set(0., 0., 0.)
    myrobot.set_noise(noise[0], noise[1], noise[2])
    filter = particles(myrobot.x, myrobot.y, myrobot.orientation, noise[0], noise[1], noise[2])

    cte  = 0.0
    err  = 0.0
    N    = 0

    index = 0 # index into the path

    while not myrobot.check_goal(goal) and N < timeout and index<len(spath)-1:

        diff_cte = - cte
        estimate = filter.get_position()
        
        dx = spath[index+1][0] - spath[index][0]
        dy = spath[index+1][1] - spath[index][1]
        rx = estimate[0] - spath[index][0]
        ry = estimate[1] - spath[index][1]

        u = (rx*dx + ry*dy) / (dx*dx + dy*dy)
        cte = (ry*dx - rx*dy) /  (dx*dx + dy*dy)
        
        if u > 1.0 :
            index += 1

        diff_cte += cte
        steer = - params[0] * cte - params[1] * diff_cte 
        myrobot = myrobot.move(grid, steer, speed)
        filter.move(grid, steer, speed)

        Z = myrobot.sense()
        filter.sense(Z)
        
        #if not 
        myrobot.check_collision(grid)
        #:
            #print('##### Collision ####')

        err += (cte ** 2)
        N += 1

        if printflag:
            #print(myrobot, cte, index, u)
            plt.scatter(myrobot.x, myrobot.y)
    plt.show()
    return [myrobot.check_goal(goal), myrobot.num_collisions, myrobot.num_steps]
