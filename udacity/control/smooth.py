import numpy as np
import matplotlib.pyplot as plt
import path
path = path.path
fix =[0]*300
size = path.shape
sz = size[0]
def smooth(fix, path,weight_data=0.5, weight_smooth=0.4, tolerance=0.01):
    newpath = np.full(size,0.0)
    for i in range(sz):
        for j in range(size[1]):
            newpath[i][j]=path[i][j]    # copy newpath = path 
    change = tolerance
    while change>=tolerance:            # first step : cha = tol = 0.0...01
        change = 0.0
        for i in range(sz):
            if fix[i] :
                newpath[i][j] = path[i][j]
            else:
                for j in range(size[1]):
                    aux = newpath[i][j]     # aux = y_old

                    # we need both  min(x-y)  , min(y_next - y)

                    # 1st : y_new = y_old + w(x_old - y_old)
                    newpath[i][j] += weight_data*(path[i][j]-newpath[i][j])
                    #newpath[i][j] += weight_smooth*(newpath[(i-1)%sz][j]+newpath[(i+1)%sz][j]-(2.0*newpath[i][j]))+\
                    #                (weight_smooth/2.0)*(2.0*newpath[(i-1)%sz][j]-newpath[(i-2)%sz][j]-newpath[i][j])+\
                    #                (weight_smooth/2.0)*(2.0*newpath[(i+1)%sz][j]-newpath[(i+2)%sz][j]-newpath[i][j])


                    # 2nd : y_new = y_old + w(y_next - y_old)
                    newpath[i][j] += weight_smooth*(newpath[(i-1)%size[0]][j]+newpath[(i+1)%size[0]][j]-(2.0*newpath[i][j]))
                    # 3rd : change = |y_old - y_new|
                    change +=abs(aux-newpath[i][j])
    return newpath

newpath = smooth(fix, path)
for i in range(0,size[0]):
    plt.scatter(path[i][0], path[i][1],color='r', marker = "p")
    plt.scatter(newpath[i][0], newpath[i][1],color='g', marker= "<")
plt.show()
