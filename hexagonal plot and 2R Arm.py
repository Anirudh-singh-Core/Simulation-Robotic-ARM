path = r"" #The execl file location
import numpy as np
import pygame
from pygame.locals import *
import pandas as pd
import math
from math import sin,cos,acos,pi,sqrt,radians

spd = 10
tol = 0.001
beta = 0.2

df = pd.read_excel(path)

theta = list(df.Theta)
d = list(df.z_displace)
alpha = list(df.Alpha)
a = list(df.x_displace)
N = len(d)

theta_var = []
for i in range(N):
    if math.isnan(float(theta[i])):
        theta[i] = radians(float(input(f'Enter value of theta{i+1}: ')))
        theta_var.append(i)
    else:
        theta[i] = radians(theta[i])

d_var = []
for i in range(N):
    if math.isnan(float(d[i])):
        d[i] = float(input(f"Enter value of d{i+1}: "))
        d_var.append(i)

pygame.init()

FPS = 60
fpsClock = pygame.time.Clock()
window = pygame.display.set_mode((900,800))
window.fill((255,255,255))

def get_hmgns():               
    global Ri_1_list,di_1_list,x_pos,y_pos,Ti_list
    Ti_list = []
    Ti = np.array([[1,0,0,0],
                   [0,1,0,0],
                   [0,0,1,0],
                   [0,0,0,1]])
    Ti_list.append(Ti)
    for i in range(N):
        i_1Ti = np.array([[cos(theta[i]),sin(theta[i]),0,0],
                          [-sin(theta[i])*cos(alpha[i]),cos(theta[i])*cos(alpha[i]),sin(alpha[i]),0],
                          [sin(theta[i])*sin(alpha[i]),-cos(theta[i])*sin(alpha[i]),cos(alpha[i]),0],
                          [a[i]*cos(theta[i]),a[i]*sin(theta[i]),d[i],1]])
        Ti = np.dot(i_1Ti,Ti)
        Ti_list.append(Ti)
        
    Ri_1_list = []
    di_1_list = []
    x_pos = []
    y_pos = []
    for i in range(N):
        Ri_1 = np.array([np.delete(Ti_list[i][2],[3])])
        Ri_1_list.append(Ri_1)
        x_pos.append(Ti_list[i][3][0])
        y_pos.append(Ti_list[i][3][1])
        di_1 = np.array([np.delete(Ti_list[i][3],[3])])
        di_1_list.append(di_1)
    di_1 = np.array([np.delete(Ti_list[N][3],[3])])
    di_1_list.append(di_1)
    x_pos.append(Ti_list[N][3][0])
    y_pos.append(Ti_list[N][3][1])

def get_jcbn():
    jcbn = np.array([[0,0,0]])
    for i in theta_var:
        R = Ri_1_list[i]
        disp = di_1_list[N] - di_1_list[i]
        Rxd = np.cross(R,disp)
        jcbn = np.append(jcbn,Rxd,axis=0)
    for i in d_var:
        R = Ri_1_list[i]
        jcbn = np.append(jcbn,R,axis=0)
    jcbn = np.delete(jcbn,(0),axis=0)
    return jcbn

def get_jnt_vel(eff_vel):
    jmat = get_jcbn()
    j_mat = np.linalg.pinv(jmat)
    jnt_vel = np.dot(eff_vel,j_mat)
    return jnt_vel

p = 32.2
    
def goal(n,h):
    k = int(h/n)
    h = h%n
    r = p*sqrt(n**2+h**2-n*h)
    theta = acos(p*(2*n-h)/(2*r)) + k*pi/3
    phi = round(math.degrees(theta))
    if n == 12 and phi%60 == 0:
        return 0
    elif n == 13 and r > 395.7:
        return 0
    elif n == 14 and (r >= 395.7 or (phi-30)%60 <= 0.01):
        return 0
    x = r*cos(theta)
    y = r*sin(theta)
    return [x,y]

def is_g_tube(g,marg):
    global switch1
    O = np.array([[0,0]])
    if np.linalg.norm(g-O)<marg:
        return O
    for n in range(1,15):
        for h in range(6*n):
            cntr = goal(n,h)
            if cntr == 0:
                continue
            cntr = np.array([cntr])
            r = np.linalg.norm(g-cntr)
            if r<marg:
                g = cntr
                switch1 = 1
                return g
    print('Select vaible location...')

window.fill((255,255,255))
pygame.draw.circle(window,(0,0,255),[450,400],6.3,0)
for n in range(1,15):
    for h in range(6*n):
        cntr = goal(n,h)
        if cntr == 0:
            continue
        cntr = [cntr[0]+450,cntr[1]+400]
        pygame.draw.circle(window,(0,0,255),cntr,6.3,0)
get_hmgns()
for i in range(N):
    pygame.draw.line(window,(0,0,0),
                     [x_pos[i]+450,y_pos[i]+400],
                     [x_pos[i+1]+450,y_pos[i+1]+400],
                     5)
pygame.display.update()

while True:
    switch1 = 0
    switch2 = 0
    global g
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                is_paused = True
                while is_paused:
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_SPACE:
                                is_paused = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                x,y = event.pos
                g = np.array([[x-450,y-400]])
                g = is_g_tube(g,15)
            '''
            for  following of the arc points
            
            if event.button == 2:
                x,y = event.pos
                for i in arc_points:
                    g = np.array([[i[0]-450,i[1]-400]])
                    g = is_g_tube(g,15)
                
            '''
                
                
    if switch1 == 0:
        continue
    pygame.draw.circle(window,(255,0,0),[g[0][0]+450,g[0][1]+400],15,4)
    pygame.display.update()
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    switch2 = 1
                    waiting = False
                elif event.key == pygame.K_x:
                    waiting = False
    if switch2 == 0:
        continue
    g = np.append(g,[[0]],axis=1)
    check = 0
    while True :
        check += 1
        window.fill((255,255,255))
        pygame.draw.circle(window,(0,0,255),[450,400],6.3,0)
        for n in range(1,15):
            for h in range(6*n):
                cntr = goal(n,h)
                if cntr == 0:
                    continue
                cntr = [cntr[0]+450,cntr[1]+400]
                pygame.draw.circle(window,(0,0,255),cntr,6.3,0)

        get_hmgns()
        for i in range(N):
            pygame.draw.line(window,(0,0,0),
                             [x_pos[i]+450,y_pos[i]+400],
                             [x_pos[i+1]+450,y_pos[i+1]+400],
                             5)
        pygame.display.update()
        fpsClock.tick(FPS)
        if check == 1000:
            print('Failed to converge...')
            break
        e = di_1_list[N]
        err = g - e
        E = np.linalg.norm(err)
        if tol<E:
            t = E/spd
            eff_vel = err/t
            delta_t = beta*t
            jnt_vel = get_jnt_vel(eff_vel)
            for i in range(len(theta_var)):
                theta[theta_var[i]] = theta[theta_var[i]] + delta_t*jnt_vel[0,i]
            for i in range(len(d_var)):
                d[d_var[i]] = d[d_var[i]] + delta_t*jnt_vel[0,len(theta_var)+i]
        else:
            break


