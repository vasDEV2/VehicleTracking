import numpy as np
from numpy import dot
from scipy.linalg import inv,block_diag

# initialize filter
dt = 0.1
x = np.zeros((6,1))
L = 15
# P = np.diag(L*np.ones(6))
a_var = 30
n = 121
F = np.array([[1.,dt,0.5*dt**2,0.,0.,0.],
              [0.,1.,dt,0.,0.,0.],
              [0.,0.,0.,1.,0.,0.],
              [0.,0.,0.,1.,dt,0.5*dt**2],
              [0.,0.,0.,0.,1.,dt],
              [0.,0.,0.,0.,0.,1.]])
Q = a_var*np.array([[dt**4//4,dt**3//2,dt**2//2,0.,0.,0.],
              [dt**3//2,dt**2,dt,0.,0.,0.],
              [dt**2//2,dt,1.,0.,0.,0.],
              [0.,0.,0.,dt**4//4,dt**3//2,dt**2//2],
              [0.,0.,0.,dt**3//2,dt**2,dt],
              [0.,0.,0.,dt**2//2,dt,1.]])
R = np.array([[n,0],
              [0,n]])
H = np.array([[1,0,0,0,0,0],
              [0,0,0,1,0,0]])
# P = np.diag(L*np.ones(6))
P = np.array([[500.,0.,0.,0.,0.,0.],
              [0.,500.,0.,0.,0.,0.],
              [0.,0.,500.,0.,0.,0.],
              [0.,0.,0.,500.,0.,0.],
              [0.,0.,0.,0.,500.,0.],
              [0.,0.,0.,0.,0.,500.]])

print(P)

# Prediction
def predict():
    global P,F,Q,R,H,x
    x = dot(F,x)
    P = dot(F,P).dot(F.T) + Q
    return x
def update(x_,y_):
    # print(R)
    global P,F,Q,R,H,x
    z = np.array([[x_],
                  [0],
                  [0],
                  [y_],
                  [0],
                  [0]])
    Z = dot(H,z)
    S =  R + dot(H,P).dot(H.T) 
    KalmanGain = dot(P,H.T).dot(inv(S))
    Y = Z - dot(H,x)
    x = x + dot(KalmanGain,Y)
    S_ = np.eye(6) - dot(KalmanGain,H)
    P = dot(S_,P).dot(S_.T) + dot(KalmanGain,R).dot(KalmanGain.T)