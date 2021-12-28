import random
from math import *
import numpy as np
import matplotlib.pyplot as plt

world_size = 100.0
landmarks = [[20.0, 80.0],[20.0, 20.0],[80.0, 20.0],[80.0, 80.0]]
max_steering_angle = pi/4.0
tolerance_xy = 15.0
tolerance_orientation = 0.25

class robot:
    def __init__(self, length):
        self.length = length
        self.x = random.random()*world_size
        self.y = random.random()*world_size
        self.orientation = random.random()*2.0*pi
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.sensor_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        if new_x<0 or new_x>=world_size:
            raise ValueError, 'X BOUND ERROR'
        if new_y<0 or new_y>=world_size:
            raise ValueError, 'Y BOUND ERROR'
        if new_orientation<0 or new_orientation>=2*pi:
            raise ValueError, 'ORIENT BOUND ERROR'
        self.x= float(new_x)
        self.y= float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_steering_noise, new_distance_noise, new_sensor_noise):
        self.steering_noise = float(new_steering_noise)
        self.distance_noise = float(new_distance_noise)
        self.sensor_noise   = float(new_sensor_noise)

    def sense(self ): # range data for each landmark
        Z=[]
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            dist += random.gauss(0.0, self.sensor_noise)
            Z.append(dist)
        return Z

    def move(self, motion, tolerance = 0.001):
        steering = motion[0]
        distance = motion[1]

        if abs(steering) > max_steering_angle:
            raise ValueError, 'Exceeding max steering angle'
        if distance < 0.0:
            raise ValueError, 'Moving backwards is not valid'

        res = robot(self.length)
        res.distance_noise = self.distance_noise
        res.steering_noise = self.steering_noise
        res.sensor_noise = self.sensor_noise

        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        turn = tan(steering2)*distance2/res.length
        if abs(turn) < tolerance:
            res.x = self.x +(distance2*cos(self.orientation))
            res.y = self.y +(distance2*sin(self.orientation))
            res.orientation = (self.orientation+turn)%(2.0*pi)
        else:
            radius = distance2 / turn
            cx = self.x - (sin(self.orientation)*radius)
            cy = self.y + (cos(self.orientation)*radius)
            res.orientation = (self.orientation+turn)%(2.0*pi)
            res.x = cx + (sin(res.orientation)*radius)
            res.y = cy - (cos(res.orientation)*radius)

        return res

    def Gaussian(self, mu, sigma, x):
        return exp(- ((mu - x)**2) / (sigma**2) / 2.0) / sqrt(2.0*pi * (sigma**2))

    def measurement_prob(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            prob *= self.Gaussian(dist, self.sensor_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))



def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))



###### main code ######
L = 20
motions = [[0.2,0.5] for row in range(1000)]
myrobot = robot(L)
myrobot.set(0.0, 0.0, 0.0)
N = 1000

#################### particle filter #####################

p = []
q = []
r = []
for i in range(0,N):                    # initial particles  
    x = robot(L)
    x.set_noise(0.5, 0.5, 5.0)
    p.append(x)

for t in range(len(motions)):
    myrobot = myrobot.move(motions[t])
    Z = myrobot.sense()

    p2 = []                             # moved particles
    for i in range(0,N):
        p2.append(p[i].move(motions[t]))
    p = p2

    w = []                              # filtering weight
    for i in range(0,N):
        w.append(p[i].measurement_prob(Z))

    p3 = []                             # filtered particle
    index = int(random.random()*N)
    beta=0.0
    mw = max(w)
    for i in range(0,N):                # stochastic resampling
        beta += random.random()*2.0*mw
        while beta >w[index]:
            beta -= w[index]
            index = (index+1)%N
        p3.append(p[index])
    p = p3
    q.append(p[2].x)
    r.append(p[2].y)

plt.plot(q,r )
plt.show()
########################################################

#for i in range(len(p)):
#    print p[i]
print myrobot

