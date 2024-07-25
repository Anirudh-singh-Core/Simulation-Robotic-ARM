path = r"" #The execl file location
import pandas as pd
import numpy as np
from numpy import sin
from numpy import cos
from math import radians

df = pd.read_excel(path)

theta = list(df.Thetas)
d = list(df.z_displace)
alpha = list(df.Alphas)
a = list(df.x_displace)
N = len(d)

for i in range(N):
    theta[i] = radians(theta[i])
    alpha[i] = radians(alpha[i])

T = np.array([[1,0,0,0],
              [0,1,0,0],
              [0,0,1,0],
              [0,0,0,1]])

list_of_homgns = [T]
list_of_Rs = []
list_of_ds = []
jcol = []

for i in range(N):   
    i_1Ti = np.array([[cos(theta[i]),-sin(theta[i])*cos(alpha[i]),sin(theta[i])*sin(alpha[i]),a[i]*cos(theta[i])],
                      [sin(theta[i]),cos(theta[i])*cos(alpha[i]),-cos(theta[i])*sin(alpha[i]),a[i]*sin(theta[i])],
                      [0,sin(alpha[i]),cos(alpha[i]),d[i]],
                      [0,0,0,1]])
    T = np.dot(T,i_1Ti)
    list_of_homgns.append(T)

for i in range(N):
    '''   To calculate Ri-1's   '''
    Ri_1 = np.dot(list_of_homgns[i],[[0],[0],[1],[0]])  
    Ri_1 = np.delete(Ri_1,3,axis=0)
    list_of_Rs.append(Ri_1)
    '''   To calculate di-1's   '''
    di_1 = np.dot(list_of_homgns[i],[[0],[0],[0],[1]])
    di_1 = np.delete(di_1,3,axis=0)
    list_of_ds.append(di_1)
di_1 = np.dot(list_of_homgns[N],[[0],[0],[0],[1]])
di_1 = np.delete(di_1,3,axis=0)
list_of_ds.append(di_1)

for i in range(N):
    if theta[i] == 0:
        jv = list_of_Rs[i]
        jcol.append(np.append(jv,[[0],[0],[0]],axis=0))
    else:
        delta_d = list_of_ds[N]-list_of_ds[i]
        jv = (np.cross(list_of_Rs[i].T,delta_d.T)).T
        jcol.append(np.append(jv,list_of_Rs[i],axis=0))

jmat = np.array([[0],[0],[0],[0],[0],[0]])
for i in range(N):
    jmat = np.append(jmat,jcol[i],axis=1)
jmat = np.delete(jmat,0,axis=1)

print(jmat)

