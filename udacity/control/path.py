import numpy as np
path1 = np.array([[x,0] for x in range(100)])
path2 = np.array([[100,x] for x in range(50)])
path3 = np.array([[100-x,50] for x in range(100)])
path4 = np.array([[0,50-x] for x in range(50)])
path = np.r_[path1,path2]
path = np.r_[path,path3]
path = np.r_[path,path4]

