import numpy as np

measurements = [[5., 10.], [6., 8.], [7., 6.], [8., 4.], [9., 2.], [10., 0.]]
xy = [4.,12.]
dt = 0.1
# X = (x y dx dy)
X = np.array([[xy[0]],[xy[1]],[0.],[0.]])

U = np.array([[0.], [0.], [0.], [0.]])  # exterior motion

F = np.array([[1.,0.,dt,0.],    # X' = FX
              [0.,1.,0.,dt],
              [0.,0.,1.,0.],
              [0.,0.,0.,1.]])
P = np.array([[0.,0.,0.,0.],    # initial variance
              [0.,0.,0.,0.],
              [0.,0.,1000.,0.],
              [0.,0.,0.,1000.]])
H = np.array([[1.,0.,0.,0.],    # measure location 
              [0.,1.,0.,0]])
R = np.array([[0.2,0],
              [0,0.2]])         # noise
I = np.eye(4,dtype=float)

def filter(x,P):
    for n in range(len(measurements)):
        # prediction
        x = F.dot(x)+U
        P = F.dot(P).dot(np.transpose(F))

        print('prediction X =')
        print x
        #print('prediction P =')
        #print P

        # update
        Z = np.array([measurements[n]])
        e = np.transpose(Z) - H.dot(x) 
        S = H.dot(P).dot(np.transpose(H)) + R
        Sinv = np.linalg.inv(S)
        K = P.dot(np.transpose(H)).dot(Sinv)
        x = x + K.dot(e)
        P = (I - K.dot(H)).dot(P)

        print('Update X =')
        print x
        #print('Update P =')
        #print P

filter(X,P)
