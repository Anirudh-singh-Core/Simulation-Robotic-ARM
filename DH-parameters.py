import math
import numpy as np
from numpy import cos as c
from numpy import sin as s
import matplotlib.pyplot as plt
o=np.array([[0,0,0,1]])
di=0 # text file
ai=0 #text file        
f = open("para.txt", "r")  # Excel file location

val = f.read().splitlines()

golden = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

for i in val:
    x = i.split(" ")
    al = int(x[0])
    a = int(x[1])
    d = int(x[2])
    t = int(x[3])
    T = np.array([[c(t),-s(t)*c(al),s(t)*s(al),a*c(t)],
                  [s(t),c(t)*c(al),-c(t)*s(al),a*s(t)],
                  [0,s(al),c(al),d],[0,0,0,1]])
    golden = np.dot(golden,T)
    print(golden)

print(np.dot(o,golden.T))
