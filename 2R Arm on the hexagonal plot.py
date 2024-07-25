import numpy as np
import pygame
from pygame.locals import *
from math import sin, cos, acos, pi, sqrt

pygame.init()

window = pygame.display.set_mode((1920, 1080))
pygame.display.set_caption('2-Link Arm with Hexagonal Pattern')
window.fill((255, 255, 255))

p = 32.2

def goal(n, h):
    r = p * sqrt(n**2 + h**2 - n * h)
    if r == 0:
        theta = 0
    else:
        theta = acos(p * (2 * n - h) / (2 * r))
    
    x = r * cos(theta)
    y = r * sin(theta)
    return np.array([x, y]), r, theta

for n in range(1, 15):
    for h in range(n):
        cntr, radius, theta = goal(n, h)
        skip_condition = False
        
        if n == 12 and theta % (pi / 3) == 0:
            skip_condition = True
        if n == 13 and radius >= 395.7:
            skip_condition = True
        if n == 14 and (radius >= 395.7 or (theta - (pi / 6)) % (pi / 3) == 0):
            skip_condition = True
        
        if not skip_condition:
            for i in range(6):
                ang = i * pi / 3
                R = np.array([[cos(ang), -sin(ang)],
                              [sin(ang), cos(ang)]])
                cntr1 = np.dot(cntr, R.T)
                cntr1 = (cntr1[0] + 960, cntr1[1] + 540)
                pygame.draw.circle(window, (0, 0, 255), (int(cntr1[0]), int(cntr1[1])), 6)

# Constants for 2-link arm
L1 = 210
L2 = 210

# Function to calculate forward kinematics of 2-link arm
def fk(theta1, theta2):
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return np.array([x, y])

# Function to calculate the Jacobian of the 2-link arm
def jacobian(theta1, theta2):
    j11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2)
    j12 = -L2 * sin(theta1 + theta2)
    j21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    j22 = L2 * cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])

# Function to draw the 2-link arm
def draw_arm(theta1, theta2):
    # Calculate joint positions
    joints = [(0, 0),
              (L1 * cos(theta1), L1 * sin(theta1)),
              (L1 * cos(theta1) + L2 * cos(theta1 + theta2), L1 * sin(theta1) + L2 * sin(theta1 + theta2))]
    
    # Convert to screen coordinates
    joints_screen = [(joint[0] + 960, joint[1] + 540) for joint in joints]
    
    # Draw links and joints
    pygame.draw.line(window, (0, 0, 0), joints_screen[0], joints_screen[1], 5)  # Link 1
    pygame.draw.line(window, (0, 0, 0), joints_screen[1], joints_screen[2], 5)  # Link 2
    pygame.draw.circle(window, (255, 0, 0), joints_screen[0], 10)  # Joint 1
    pygame.draw.circle(window, (255, 0, 0), joints_screen[1], 10)  # Joint 2

    # Calculate end effector position
    end_effector = fk(theta1, theta2)
    end_effector_screen = (end_effector[0] + 960, end_effector[1] + 540)
    pygame.draw.circle(window, (0, 255, 0), (int(end_effector_screen[0]), int(end_effector_screen[1])), 10)  # End effector

# Inverse kinematics function
def ik(goal_pos, theta1, theta2, tolerance=0.001, max_iterations=1000):
    for _ in range(max_iterations):
        current_pos = fk(theta1, theta2)
        error = goal_pos - current_pos
        if np.linalg.norm(error) < tolerance:
            break
        J = jacobian(theta1, theta2)
        dtheta = np.linalg.pinv(J) @ error
        theta1 += dtheta[0]
        theta2 += dtheta[1]
    return theta1, theta2

# Set the goal position (choose a point within the hexagonal pattern)
goal_position = goal(2,1)[0]  # Example: you can change this to any valid point in the pattern
goal_position = (goal_position[0] + 960, goal_position[1] + 540)  # Convert to screen coordinates

# Initial joint angles (in radians)
initial_theta1 = pi / 4
initial_theta2 = -pi / 2

# Convert the goal position back to the arm's coordinate system
goal_position_arm = (goal_position[0] - 960, goal_position[1] - 540)

# Calculate the joint angles using inverse kinematics
theta1_final, theta2_final = ik(goal_position_arm, initial_theta1, initial_theta2)

# Draw the final arm configuration
draw_arm(theta1_final, theta2_final)

# Update the display to show the 2-link arm above the hexagonal pattern
pygame.display.update()

# Wait until the window is closed
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

pygame.quit()


