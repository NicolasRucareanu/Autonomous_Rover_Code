# importing Libraries
from qset_lib import Rover
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import math
count = 0
def ObstacleDetection(rover, goalX, goalY):
     j = 0
     infinity = float('inf')

     x_points = []
     y_points = []
     obstacles = []

     for dist in rover.laser_distances:
         if dist < infinity:
             xpt = (dist * math.cos((j * 6) + rover.heading)) + rover.x
             ypt = (dist * math.sin((j * 6) + rover.heading)) + rover.y
             # x_points.append((dist * math.cos((j * 6) + rover.heading)) + rover.x)
             # y_points.append((dist * math.sin((j * 6) + rover.heading)) + rover.y)
             laser_angle = (-90) + (j * 6.20689655)
             xpt = (dist * math.cos(math.radians(laser_angle + rover.heading))) + rover.x
             ypt = (dist * math.sin(math.radians(laser_angle + rover.heading))) + rover.y
             obstacles.append((xpt, ypt))
         j = j + 1
     sleep(0.01)

     print("Rover Heading: " + str(rover.heading))
     print("Rover X:" + str(rover.x))
     print("Rover Y:" + str(rover.y))

     return ObstacleAvoidance(obstacles, goalX, goalY, rover)

def ObstacleAvoidance(obstacle, goalX, goalY, rover):
    global count
    grid_dims = 10
    x = np.arange(-grid_dims, grid_dims, 1)
    y = np.arange(-grid_dims, grid_dims, 1)
    # Goal is at (40,40)
    goal = [goalX, goalY]
    # obstacle is at(25,25)
    # obstacle = [2, 1]
    X, Y = np.meshgrid(x, y)
    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)

    s = 5
    r = 0.5
    alpha = 3
    beta = 3
    alpha = 50
    beta = 175
    # obstacle = []
    for points in obstacle:
         for i in range(len(x)):
            for j in range(len(y)):
                # finding the goal distance and obstacle distance
                d_goal = np.sqrt((goal[0] - X[i][j]) ** 2 + ((goal[1] - Y[i][j])) ** 2)
                d_obstacle = np.sqrt((points[0] - X[i][j]) ** 2 + (points[1] - Y[i][j]) ** 2)
                # print(f"{i} and {j}")
                # finding theta correspoding to the goal and obstacle
                theta_goal = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
                theta_obstacle = np.arctan2(points[1] - Y[i][j], points[0] - X[i][j])
                if d_obstacle < r:
                    delx[i][j] = -1 * np.sign(np.cos(theta_obstacle)) + 0
                    dely[i][j] = -1 * np.sign(np.cos(theta_obstacle)) + 0
                elif d_obstacle > r + s:
                    delx[i][j] += 0
                    dely[i][j] += 0
                elif d_obstacle < r + s:
                    delx[i][j] += -beta * (s + r - d_obstacle) * np.cos(theta_obstacle)
                    dely[i][j] += -beta * (s + r - d_obstacle) * np.sin(theta_obstacle)
                if d_goal < r + s:
                    if delx[i][j] != 0:
                        delx[i][j] += (alpha * (d_goal - r) * np.cos(theta_goal))
                        dely[i][j] += (alpha * (d_goal - r) * np.sin(theta_goal))
                    else:
                        delx[i][j] = (alpha * (d_goal - r) * np.cos(theta_goal))
                        dely[i][j] = (alpha * (d_goal - r) * np.sin(theta_goal))
                if d_goal > r + s:
                    if delx[i][j] != 0:
                        delx[i][j] += alpha * s * np.cos(theta_goal)
                        dely[i][j] += alpha * s * np.sin(theta_goal)
                    else:
                        delx[i][j] = alpha * s * np.cos(theta_goal)
                        dely[i][j] = alpha * s * np.sin(theta_goal)
    print(delx)
    print(dely)
    if count % 10 == 0:
        rover.send_command(0,0)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.quiver(X, Y, delx, dely)
        ax.add_patch(plt.Circle((goalX, goalY), r, color='m'))
        for obs in obstacle:
            ax.add_patch(plt.Circle((obs[0], obs[1]), r, color='y'))
            ax.annotate("Obstacle", xy=obs, fontsize=8, ha="center")
        ax.annotate("Goal", xy=(goalX, goalY), fontsize=10, ha="center")

        ax.set_title('Combined Potential')

        plt.show()
        # plt.show()

    count = count + 1

    # Finding the next point to head to by using the vectors at the location of the rover
    next_x = delx[int(round(rover.x)) + grid_dims][int(round(rover.y)) + grid_dims] + rover.x
    next_y = dely[int(round(rover.x)) + grid_dims][int(round(rover.y)) + grid_dims] + rover.y
    if obstacle is None:
        print("no obstacle")
        return driveControl(rover, goalX, goalY, goalX, goalY)
    else:
        return driveControl(rover, next_x, next_y, goalX, goalY)


def driveControl(rover, nextX, nextY, goalX, goalY):
     # begin by moving in a straight line

     print("X: " + str(rover.x) + " Y: " + str(rover.y) + " Heading: " + str(rover.heading))  # print rover location

     # scan rover coordinates, needs editing but fine for now

     # assume rover never has to move in reverse to follow path/traverse around obstacles

     speed = 5
     currX = rover.x
     currY = rover.y
     left_side_speed = speed
     right_side_speed = speed
     if nextX > currX and nextY > currY:
         # move forward/turn right/left
         angle = math.degrees(math.atan2(nextY - currY, nextX - currX)) - rover.heading
         # forward right
         angle = math.degrees(math.atan2(nextY - currY, nextX - currX)) - abs((rover.heading))
         distance = math.sqrt((nextY - currY) ** 2 + (nextX - currX) ** 2)
         left_side_speed = speed
         right_side_speed = 0  # I'm not quite sure how this would work, the set speed would have to be variable
         left_side_speed = 0
         right_angle_error = angle  # calculate angle error
         right_speed_error = right_angle_error / 180.0
         right_side_speed = speed - (speed * right_speed_error)  # adjust speed proportionally to error
         print("Angle: " + str(angle) + " Distance: " + str(distance))
     elif nextX > currX and nextY == currY:
         # turn right
         left_side_speed = 0
         right_side_speed = speed
         angle = 90 - abs(rover.heading)
         distance = abs(nextX - currX)
         print("Angle: " + str(angle) + " Distance: " + str(distance))
     elif nextX < currX and nextY == currY:
         # turn left
         left_side_speed = speed
         right_side_speed = 0
         angle = 90 - rover.heading
         angle = 90 - abs(rover.heading)
         distance = abs(nextX - currX)
         print("Angle: " + str(angle) + " Distance: " + str(distance))
     elif nextX == currX and nextY > currY:
         # move forward in a straight line
         left_side_speed = speed
         right_side_speed = speed
         angle = 0
         distance = abs(nextY - currY)
         print("Angle: " + str(angle) + " Distance: " + str(distance))
     elif nextX < currX and nextY > currY:
         # turn right
         angle = -1 * (math.degrees(math.atan2(nextY - currY, currX - nextX)) + 90 - rover.heading)
         left_side_speed = 0
         right_side_speed = speed
         # forward left
         distance = math.sqrt((nextY - currY) ** 2 + (nextX - currX) ** 2)
         angle = -1 * (math.degrees(math.atan2(nextY - currY, currX - nextX)) + 90 - abs(rover.heading))
         right_side_speed = 0
         left_angle_error = angle  # calculate angle error
         left_speed_error = left_angle_error / 180.0
         left_side_speed = speed - (speed * left_speed_error)  # adjust speed proportionally to error
         print("Angle: " + str(angle) + " Distance: " + str(distance))

         print("Angle: " + str(angle) + " Distance: " + str(distance))

     print("Left: ", left_side_speed, " right: ", right_side_speed)

     rover.send_command(left_side_speed, right_side_speed)
     #   if distance < 0.1 and rover.heading < 1:
     #       return True
     #   else:
     #       # return False

     if goalX == round(rover.x) and goalY == round(rover.y):
         return True
     return False

def main():
     rover = Rover()
     goalX = input("Enter the goal's x coordinate")
     goalY = input("Enter the goal's y coordinate")
     goalX = input("Enter the goal's x coordinate: ")
     goalY = input("Enter the goal's y coordinate: ")
     while not ObstacleDetection(rover, goalX, goalY):
         sleep(0.1)