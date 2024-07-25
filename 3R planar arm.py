import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians, degrees

# Set initial parameters for 3R arm
t1 = radians(45)
t2 = radians(-90)
t3 = radians(30)
l1 = 1
l2 = 1
l3 = 1
x = 2
y = 2
b = 0.25
tol = 0.1
endpoint = np.array([x, y])

# Forward kinematics function
def fk(t1, t2, t3):
    x = l1*cos(t1) + l2*cos(t1 + t2) + l3*cos(t1 + t2 + t3)
    y = l1*sin(t1) + l2*sin(t1 + t2) + l3*sin(t1 + t2 + t3)
    return np.array([x, y])

# Error calculation function
def E(t1, t2, t3):
    Emat = endpoint - fk(t1, t2, t3)
    return Emat

# Inverse Jacobian function
def del_t(t1, t2, t3):
    J_mat = np.array([
        [-l1*sin(t1) - l2*sin(t1 + t2) - l3*sin(t1 + t2 + t3), -l2*sin(t1 + t2) - l3*sin(t1 + t2 + t3), -l3*sin(t1 + t2 + t3)],
        [l1*cos(t1) + l2*cos(t1 + t2) + l3*cos(t1 + t2 + t3), l2*cos(t1 + t2) + l3*cos(t1 + t2 + t3), l3*cos(t1 + t2 + t3)]
    ])
    J_Inmat = np.linalg.pinv(J_mat)
    Delta_t = np.dot(J_Inmat, E(t1, t2, t3)) * b
    return Delta_t

# Iterative loop function
def loop():
    global t1, t2, t3
    count = 0
    while np.linalg.norm(E(t1, t2, t3)) > tol:
        if count == 1000:
            print("Iteration failed to converge.")
            return
        delta_t = del_t(t1, t2, t3)
        t1 += delta_t[0]
        t2 += delta_t[1]
        t3 += delta_t[2]
        t1 = radians(degrees(t1))
        t2 = radians(degrees(t2))
        t3 = radians(degrees(t3))
        count += 1
    print("Success! Final angles: t1 = {:.2f}°, t2 = {:.2f}°, t3 = {:.2f}°".format(degrees(t1), degrees(t2), degrees(t3)))

# Function to plot the 3R planar arm
def plot_3r_arm(t1, t2, t3, l1, l2, l3, color='k', label=''):
    joint1 = (l1 * cos(t1), l1 * sin(t1))
    joint2 = (joint1[0] + l2 * cos(t1 + t2), joint1[1] + l2 * sin(t1 + t2))
    joint3 = fk(t1, t2, t3)
    plt.plot([0, joint1[0]], [0, joint1[1]], color+'-', lw=2, label=label) # Link 1
    plt.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], color+'-', lw=2) # Link 2
    plt.plot([joint2[0], joint3[0]], [joint2[1], joint3[1]], color+'-', lw=2) # Link 3
    plt.plot([joint1[0]], [joint1[1]], 'ro') # Joint 1
    plt.plot([joint2[0]], [joint2[1]], 'ro') # Joint 2
    plt.plot([joint3[0]], [joint3[1]], 'go') # End Effector

# Plot the original and final positions
plt.figure()
plot_3r_arm(radians(45), radians(-90), radians(30), l1, l2, l3, 'b', 'Original Position')
loop()  # Run the loop to find the final position
plot_3r_arm(t1, t2, t3, l1, l2, l3, 'r', 'Final Position')

# Plot the endpoint
plt.plot(endpoint[0], endpoint[1], 'rx', label='Endpoint', markersize=10)

# Set plot limits and labels
plt.xlim(-l1-l2-l3, l1+l2+l3)
plt.ylim(-l1-l2-l3, l1+l2+l3)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('3-Link Planar Arm Positions')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()


