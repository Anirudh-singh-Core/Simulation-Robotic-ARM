import pygame
import math

pygame.init()

WIDTH, HEIGHT = 900, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Tangent Points and Lines on Circle")

# Circle parameters
Cx, Cy = 433, 257
a = 16.1  # Circle radius

# Starting point
Px, Py = 450, 287

# Goal point
Gx, Gy = 416, 230

# Calculate distance between point P and circle center C
b = math.sqrt((Px - Cx)**2 + (Py - Cy)**2)

# Calculate angle theta
theta = math.acos(a / b)

# Calculate direction angle of vector from C to P
direction_angle = math.atan2(Py - Cy, Px - Cx)

# Calculate tangency angles
tangency_angle1 = direction_angle + theta
tangency_angle2 = direction_angle - theta

# Calculate tangency points for P
Tangent1 = (Cx + a * math.cos(tangency_angle1), Cy + a * math.sin(tangency_angle1))
Tangent2 = (Cx + a * math.cos(tangency_angle2), Cy + a * math.sin(tangency_angle2))

# Function to calculate arc length
def arc_length(angle1, angle2, radius):
    angle_diff = abs(angle1 - angle2)
    if angle_diff > math.pi:
        angle_diff = 2 * math.pi - angle_diff
    return angle_diff * radius

# Calculate distance between point G and circle center C
b1 = math.sqrt((Gx - Cx)**2 + (Gy - Cy)**2)

# Calculate angle theta for point G
theta1 = math.acos(a / b1)

# Calculate direction angle of vector from C to G
direction_angle1 = math.atan2(Gy - Cy, Gx - Cx)

# Calculate tangency angles for G
tangency_angle1_1 = direction_angle1 + theta1
tangency_angle2_1 = direction_angle1 - theta1

# Calculate tangency points for G
Tangent1_1 = (Cx + a * math.cos(tangency_angle1_1), Cy + a * math.sin(tangency_angle1_1))
Tangent2_1 = (Cx + a * math.cos(tangency_angle2_1), Cy + a * math.sin(tangency_angle2_1))

####################################################################################################

# Calculate arc lengths between Tangent1 and Tangent2 for P   
arc_length_P = arc_length(tangency_angle1, tangency_angle2_1, a)

# Calculate arc lengths between Tangent1_1 and Tangent2_1 for G
arc_length_G = arc_length(tangency_angle2, tangency_angle1_1, a)

# Determine the smallest arc length
smallest_arc = min(arc_length_P, arc_length_G)

print(f"The smallest arc length is: {smallest_arc}")

# Function to calculate points along the arc
def calculate_arc_points(center, radius, start_angle, end_angle, num_points):
    arc_points = []
    angle_step = (end_angle - start_angle) / num_points
    for i in range(num_points + 1):
        angle = start_angle + i * angle_step
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        arc_points.append((x, y))
    '''
    #print(arc_points)  End points to reach 2R
    for i in arc_points:
        print(i)
    '''
    return arc_points

# Calculate points along the smallest arc
if smallest_arc == arc_length_P:
    arc_points = calculate_arc_points((Cx, Cy), a, tangency_angle1, tangency_angle2_1, 50)
elif smallest_arc == arc_length_G:
    arc_points = calculate_arc_points((Cx, Cy), a, tangency_angle2, tangency_angle1_1, 50)

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                x, y = event.pos
                print(f"Mouse clicked at: ({x}, {y})")

    screen.fill(WHITE)

    # Draw the Starting point (P) and Goal point (G)
    pygame.draw.circle(screen, RED, (Px, Py), 6)
    pygame.draw.circle(screen, RED, (Gx, Gy), 6)

    # Draw the circle
    pygame.draw.circle(screen, BLACK, (Cx, Cy), int(a), 2)

    # Draw the tangent points and lines for P
    pygame.draw.circle(screen, BLACK, (int(Tangent1[0]), int(Tangent1[1])), 2)
    pygame.draw.circle(screen, BLACK, (int(Tangent2[0]), int(Tangent2[1])), 2)
    pygame.draw.line(screen, BLACK, (Cx, Cy), (int(Tangent1[0]), int(Tangent1[1])), 2)
    pygame.draw.line(screen, BLACK, (Cx, Cy), (int(Tangent2[0]), int(Tangent2[1])), 2)
    pygame.draw.line(screen, BLACK, (int(Tangent1[0]), int(Tangent1[1])), (Px, Py), 2)
    pygame.draw.line(screen, BLACK, (int(Tangent2[0]), int(Tangent2[1])), (Px, Py), 2)

    # Draw the tangent points and lines for G
    pygame.draw.circle(screen, BLACK, (int(Tangent1_1[0]), int(Tangent1_1[1])), 2)
    pygame.draw.circle(screen, BLACK, (int(Tangent2_1[0]), int(Tangent2_1[1])), 2)
    pygame.draw.line(screen, BLACK, (Cx, Cy), (int(Tangent1_1[0]), int(Tangent1_1[1])), 2)
    pygame.draw.line(screen, BLACK, (Cx, Cy), (int(Tangent2_1[0]), int(Tangent2_1[1])), 2)
    pygame.draw.line(screen, BLACK, (int(Tangent1_1[0]), int(Tangent1_1[1])), (Gx, Gy), 2)
    pygame.draw.line(screen, BLACK, (int(Tangent2_1[0]), int(Tangent2_1[1])), (Gx, Gy), 2)

    # Draw the smallest arc in red  <---------------------------------------------------
    pygame.draw.lines(screen, RED, False, arc_points, 3)

    pygame.display.flip()

pygame.quit()


