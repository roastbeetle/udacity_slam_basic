import numpy as np

phit = 0.8 # sensor's right
pmiss = 0.1

#pexact = 0.8 # motor's right
#povershoot = 0.1
#pundershoot = 0.1
pshoot = [[0.05,0.05,0.05],
          [0.05, 0.6,0.05],
          [0.05,0.05,0.05]]

g = 'green'
r = 'red'

#world=[g,r,r,g,g]
world=[[g,g,g],
        [g,r,g],
        [g,g,g]]

#p = [0.2,0.2,0.2,0.2,0.2]
p=[[0.111,0.111,0.111],
   [0.111,0.111,0.111],
   [0.111,0.111,0.111]]

measurements = [r]
#motion = [1, 1] 
motion = [[0,0]]

def sense(p,Z):
    r=[]
    s=0
    for i in range(len(p)):
        q=[]
        for j in range(len(p[i])):
            hit = (Z==world[i][j])
            q.append(p[i][j]*(hit*phit+(1-hit)*pmiss))
        s+=sum(q)
        r.append(q)
    for i in range(len(r)):
        for j in range(len(r[i])): 
            r[i][j]=r[i][j]/s
    return r 

def move(p,U):
    print p 
    print pshoot
    p_a = np.array(p)
    print p_a
    ps_a = np.array(pshoot) 
    print ps_a
    r = p_a*ps_a
    return r

for i in range(len(measurements)):
    print ('*****************************\n')
    p = sense(p,measurements[i]) 
    print p
    p = move(p,motion[i])
    print p
    print ('*****************************\n')

