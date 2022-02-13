import math
import random
from copy import copy

import numpy as np
from pygame.math import Vector2


# calculate the slope of the line
def slope(wall):
    x1, y1 = wall[0]
    x2, y2 = wall[1]
    if x1 == x2:
        return 90
    elif y1 == y2:
        return 0
    else:
        return math.atan((y2 - y1) / (x2 - x1)) * 180 / math.pi


def right_intersection(point, wall_point1, wall_point2, sensor_start, sensor_end):
    x, y = point
    right = (x - wall_point1[0]) * (x - wall_point2[0]) <= 0 and \
            (y - wall_point1[1]) * (y - wall_point2[1]) <= 0 and \
            (x - sensor_start[0]) * (sensor_end[0] - sensor_start[0]) >= 0 and \
            (y - sensor_start[1]) * (sensor_end[1] - sensor_start[1]) >= 0
    return right


class Robot:

    def __init__(self, outer_wall, inner_wall, size):
        self.radius = int(size / 2)
        self.x, self.y = self.initPosition(outer_wall, inner_wall)
        self.frontX = self.x + self.radius
        self.frontY = self.y
        self.Vl = 0
        self.Vr = 0
        self.theta = 0
        self.speed = 0.1
        self.sensors, _ = self.distanceToSensors(outer_wall, inner_wall)
        self.safe_distance = 5
        self.stop = False
        self.last_wall = []

    def initPosition(self, outer_wall, inner_wall):
        x = random.randint(outer_wall[0][0] + self.radius, outer_wall[2][0] - self.radius)
        y = random.randint(outer_wall[0][1] + self.radius, outer_wall[2][1] - self.radius)
        while (inner_wall[0][0] - self.radius < x < inner_wall[2][0] + self.radius) and (
                inner_wall[0][1] - self.radius < y < inner_wall[2][1] + self.radius):
            x = random.randint(outer_wall[0][0] + self.radius, outer_wall[2][0] - self.radius)
            y = random.randint(outer_wall[0][1] + self.radius, outer_wall[2][1] - self.radius)
        return x, y

    def move(self, movement, delta_t, outer_wall, inner_wall):
        next_x = 0
        next_y = 0
        new_theta = self.theta
        # Check keys for movement
        # movement = [w, s, o, l, x, t, g]

        if movement[0] == 1 or movement[5] == 1:
            self.Vl += self.speed
        if movement[1] == 1 or movement[6] == 1:
            self.Vl -= self.speed
        if movement[2] == 1 or movement[5] == 1:
            self.Vr += self.speed
        if movement[3] == 1 or movement[6] == 1:
            self.Vr -= self.speed
        if movement[4] == 1:
            self.Vl = 0
            self.Vr = 0

        # If it's moving
        if self.Vr != 0 or self.Vl != 0:
            # Make sure not to get a division by zero when velocities are the same
            if self.Vr == self.Vl:
                R = 10000
                w = 0
                if not self.stop:
                    # self.x += ((self.Vl + self.Vr) / 2) * np.cos(self.theta) * delta_t
                    # self.y += ((self.Vl + self.Vr) / 2) * np.sin(self.theta) * delta_t
                    next_x = self.x + ((self.Vl + self.Vr) / 2) * np.cos(self.theta) * delta_t
                    next_y = self.y + ((self.Vl + self.Vr) / 2) * np.sin(self.theta) * delta_t
            else:
                R = self.radius * (self.Vl + self.Vr) / (self.Vr - self.Vl)
                w = (self.Vr - self.Vl) / (self.radius * 2)
                # Compute ICC
                ICC = [self.x - R * np.sin(self.theta), self.y + R * np.cos(self.theta)]
                result = np.transpose(np.matmul(
                    np.array([[np.cos(w * delta_t), -np.sin(w * delta_t), 0],
                              [np.sin(w * delta_t), np.cos(w * delta_t), 0],
                              [0, 0, 1]]),
                    np.transpose(np.array([self.x - ICC[0], self.y - ICC[1], self.theta]))) + np.array(
                    [ICC[0], ICC[1], w * delta_t])).transpose()
                next_x, next_y, new_theta = result[0], result[1], result[2]
            # update  sensors
            self.sensors, walls = self.distanceToSensors(outer_wall, inner_wall)

            # detect collision
            delta_t, collision = self.detectCollision(delta_t)
            # handle collision
            if collision:
                next_x, next_y = self.handleCollision(next_x, next_y, walls)
            # Transfer results from the ICC computation
            self.x = next_x
            self.y = next_y
            self.theta = new_theta
            self.frontX, self.frontY = self.rotate(self.theta, self.radius)

        return self.Vl, self.Vr, np.round(np.degrees(self.theta) % 360, 2), delta_t

    def distanceToSensors(self, outer_wall, inner_wall):
        dist = []
        walls = []
        angle = copy(self.theta)
        for i in range(12):
            min_dist_out_wall, wall_out = self.distance(outer_wall, angle)
            min_dist_in_wall, wall_in = self.distance(inner_wall, angle)
            if min_dist_out_wall > min_dist_in_wall:
                '''if the distance to closer out wall is bigger than the distance 
                to the closer in wall, then keep the in wall distance'''
                dist.append(min_dist_in_wall)
                walls.append(wall_in)
            else:
                dist.append(min_dist_out_wall)
                walls.append(wall_out)
            angle += math.pi / 6
        return dist, walls

    def rotate(self, angle, r):
        # Rotate the robot at a certain angle from the x-axis
        front_x = self.x + np.cos(angle) * r
        front_y = self.y + np.sin(-angle) * r
        return front_x, front_y

    def distance(self, wall, angle):
        dist = []
        select_wall = []
        for i in range(len(wall) - 1):
            # Out wall line
            point1 = wall[i]
            point2 = wall[i + 1]
            a1 = point2[1] - point1[1]
            b1 = point1[0] - point2[0]
            c1 = a1 * point1[0] + b1 * point1[1]

            # Sensor line
            point3 = [self.x, self.y]
            point4 = Vector2(self.rotate(angle, self.radius))
            a2 = point4[1] - point3[1]
            b2 = point3[0] - point4[0]

            c2 = a2 * point3[0] + b2 * point3[1]
            determinant = a1 * b2 - a2 * b1

            if abs(determinant) > 0.00000001:  # if there is an intersectioin
                new_x = -(b1 * c2 - b2 * c1) / determinant  # intersection coordinate
                new_y = (a1 * c2 - a2 * c1) / determinant
                if abs(round(new_x) - new_x) < 0.00000001:
                    new_x = round(new_x)
                if abs(round(new_y) - new_y) < 0.00000001:
                    new_y = round(new_y)
                # make sure intersection is in front of the sensor
                if right_intersection((new_x, new_y), point1, point2, point3, point4):
                    dist.append(math.sqrt((new_x - point4[0]) ** 2 + (new_y - point4[1]) ** 2))
                    select_wall.append([point1, point2])
        wall = []
        if len(dist):
            min_dist_out_wall = min(dist)  # CLoser wall to sensor
            wall_index = dist.index(min_dist_out_wall)
            wall = select_wall[wall_index]
        else:
            min_dist_out_wall = 1500
        return min_dist_out_wall, wall

    def decomposeMovement(self, move_x, move_y, wall):
        theta_wall = slope(wall)
        theta_move = slope([[self.x, self.y], [move_x, move_y]])
        theta_wall_move = theta_wall - theta_move
        move_distance = ((move_x - self.x) ** 2 + (move_y - self.y) ** 2) ** 0.5
        distance_parallel = move_distance * np.cos(theta_wall_move * math.pi / 180)
        distance_vertical = move_distance * np.sin(theta_wall_move * math.pi / 180)
        return distance_parallel, distance_vertical, theta_wall

    def parallelMove(self, distance_parallel, theta_wall):
        x = self.x
        y = self.y
        if theta_wall == 90:
            y = self.y + distance_parallel
        elif theta_wall == 0:
            x = self.x + distance_parallel
        else:
            y = self.y + distance_parallel * np.cos(theta_wall * math.pi / 180)
            x = self.x + distance_parallel * np.sin(theta_wall * math.pi / 180)
        return x, y

    def detectCollision(self, delta_t):
        collision_dis = min(self.sensors)
        collision = False
        if collision_dis < self.safe_distance:
            collision = True
            # keep delta_t more than 0.1
            # if delta_t > 0.1:
            #     delta_t = (collision_dis / self.safe_distance)
            # else:
            #     delta_t = 0.1
        # else:
        #     delta_t = 0.1
        return delta_t, collision

    def handleCollision(self, move_x, move_y, walls):
        next_x = move_x
        next_y = move_y
        # get the closest distance
        sensors_copy = self.sensors.copy()
        sensors_copy.sort()
        collision_dis = sensors_copy[0]
        second_collision_dis = sensors_copy[1]
        # get the closest wall
        wall_index = self.sensors.index(collision_dis)
        second_wall_index = self.sensors.index(second_collision_dis)
        first_wall = walls[wall_index]
        second_wall = walls[second_wall_index]
        for i in range(len(walls)):
            if first_wall == second_wall:
                second_collision_dis = sensors_copy[i + 1]
                second_wall_index = self.sensors.index(second_collision_dis)
                second_wall = walls[second_wall_index]
            else:
                break

        distance_parallel_first, distance_vertical_first, theta_wall_first = \
            self.decomposeMovement(next_x, next_y, first_wall)
        distance_parallel_second, distance_vertical_second, theta_wall_second = \
            self.decomposeMovement(next_x, next_y, first_wall)

        if (collision_dis < 4 and second_collision_dis < 4) or (
                collision_dis < distance_vertical_first and second_collision_dis < distance_vertical_second):
            if distance_vertical_first > 0 and distance_vertical_second > 0:
                self.stop = True
                next_x = self.x
                next_y = self.y
            elif distance_vertical_first > 0:
                if distance_parallel_first == 0.0:
                    self.stop = True
                    next_x = self.x
                    next_y = self.y
                else:
                    next_x, next_y = self.parallelMove(distance_parallel_first, theta_wall_first)
            elif distance_vertical_second > 0:
                if distance_parallel_second == 0.0:
                    self.stop = True
                    next_x = self.x
                    next_y = self.y
                else:
                    next_x, next_y = self.parallelMove(distance_parallel_second, theta_wall_second)
            else:
                print("move out the line")
        elif collision_dis < 4 or collision_dis < distance_vertical_first:
            if distance_parallel_first == 0.0:
                self.stop = True
                next_x = self.x
                next_y = self.y
            else:
                next_x, next_y = self.parallelMove(distance_parallel_first, theta_wall_first)
        elif second_collision_dis < 4 or second_collision_dis < distance_vertical_second:
            if distance_parallel_second == 0.0:
                self.stop = True
                next_x = self.x
                next_y = self.y
            else:
                next_x, next_y = self.parallelMove(distance_parallel_second, theta_wall_second)
        else:
            print(second_collision_dis, collision_dis, distance_vertical_second, distance_vertical_first)
        return next_x, next_y
