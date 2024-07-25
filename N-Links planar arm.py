# Your personalized code
from numpy import sin as s
from numpy import cos as c
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def get_user_input():
    n = int(input("Enter the number of links: "))
    
    theta = []
    L = []

    for i in range(n):
        theta_input = input(f"Enter angle for joint {i+1} (in degrees, default 45): ")
        theta.append(np.radians(float(theta_input)) if theta_input else np.radians(45))

    for i in range(n):
        length_input = input(f"Enter length for link {i+1} (default 1): ")
        L.append(float(length_input) if length_input else 1)
    
    x_goal = float(input("Enter the goal position x coordinate: "))
    y_goal = float(input("Enter the goal position y coordinate: ") or 3.5)
    goal_pos = np.array([x_goal, y_goal])

    return theta, L, goal_pos

def forward_kinematics(theta, L):
    x = 0
    y = 0
    total_theta = 0
    for i in range(len(theta)):
        total_theta += theta[i]
        x += L[i] * c(total_theta)
        y += L[i] * s(total_theta)
    return x, y

def jacobian(theta, L):
    n = len(theta)
    J = np.zeros((2, n))
    for i in range(n):
        sum_theta = sum(theta[:i+1])
        J[0, i] = -sum([L[j] * s(sum(theta[:j+1])) for j in range(i, n)])
        J[1, i] = sum([L[j] * c(sum(theta[:j+1])) for j in range(i, n)])
    return J

def inverse_kinematics_animation(theta, L, goal_pos, tolerance, beta):
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim([-sum(L), sum(L)])  
    ax.set_ylim([-sum(L), sum(L)])  
    ax.scatter(goal_pos[0], goal_pos[1], color="red")

    # Calculate original position
    x_orig, y_orig = forward_kinematics(theta, L)
    ax.scatter(x_orig, y_orig, color="green", label="Original position")

    line, = ax.plot([], [], lw=2)
    arm_line, = ax.plot([], [], lw=3, color='blue')

    x_pos = []
    y_pos = []

    def init():
        line.set_data([], [])
        arm_line.set_data([], [])
        return line, arm_line

    def update(frame):
        nonlocal theta

        J = jacobian(theta, L)
        e = goal_pos - np.array(forward_kinematics(theta, L))
        invJ = np.linalg.pinv(J)
        delta_theta = invJ @ e
        theta = [theta[i] + beta * delta_theta[i] for i in range(len(theta))]

        x = [0]
        y = [0]
        total_theta = 0
        for i in range(len(theta)):
            total_theta += theta[i]
            x.append(x[-1] + L[i] * c(total_theta))
            y.append(y[-1] + L[i] * s(total_theta))

        x_pos.append(x[-1])
        y_pos.append(y[-1])

        line.set_data(x_pos, y_pos)
        arm_line.set_data(x, y)

        return line, arm_line

    ani = FuncAnimation(fig, update, frames=np.arange(0, 100), init_func=init, blit=True)
    plt.legend()
    plt.show()

    print("Final angles in degrees:", [angle * 180 / np.pi for angle in theta])

    # Check if goal position is within reach
    distance_to_goal = np.linalg.norm(goal_pos - np.array([x_orig, y_orig]))
    if distance_to_goal <= tolerance:
        print("Goal position is within reach.")
    else:
        print("Goal position is out of reach.")

# Get user input for n links and goal position
theta, L, goal_pos = get_user_input()

# Other parameters
tolerance = 0.1
beta = 0.1

# Run the animation
inverse_kinematics_animation(theta, L, goal_pos, tolerance, beta)


