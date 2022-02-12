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
        return math.inf
    else:
        return -1 / np.tan((y2 - y1) / (x2 - x1))


class Robot:

    def __init__(self, outer_wall, inner_wall, size):
        self.radius = int(size / 2)
        self.x, self.y = self.initPosition(outer_wall, inner_wall)
        self.frontX = self.x + self.radius
        self.frontY = self.y
        self.Vl = 0
        self.Vr = 0
        self.theta = 0
        self.speed = 0.5
        self.sensors, _ = self.distanceToSensors(outer_wall, inner_wall)
        self.distance_limit = size

    def initPosition(self, outer_wall, inner_wall):
        x = random.randint(outer_wall[0][0] + self.radius, outer_wall[2][0] - self.radius)
        y = random.randint(outer_wall[0][1] + self.radius, outer_wall[2][1] - self.radius)
        while (inner_wall[0][0] < x < inner_wall[2][0]) and (inner_wall[0][1] < y < inner_wall[2][1]):
            x = random.randint(outer_wall[0][0] + self.radius, outer_wall[2][0] - self.radius)
            y = random.randint(outer_wall[0][1] + self.radius, outer_wall[2][1] - self.radius)
        return x, y

    def move(self, movement, delta_t, outer_wall, inner_wall):
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
            # update  sensors
            self.sensors, walls = self.distanceToSensors(outer_wall, inner_wall)
            self.detectCollision(self.sensors, walls, delta_t)
            # Transfer results from the ICC computation
            self.x = result[0]
            self.y = result[1]
            self.theta = result[2]
            self.frontX, self.frontY = self.rotate(self.theta, self.radius)

        return self.Vl, self.Vr, np.round(np.degrees(self.theta) % 360, 2), delta_t

    def distanceToSensors(self, outer_wall, inner_wall):
        dist = []
        walls = []
        angle = copy(self.theta)
        for i in range(12):
            min_dist_out_wall, wall_index_1 = self.distance(outer_wall, angle)
            min_dist_in_wall, wall_index_2 = self.distance(inner_wall, angle)
            if min_dist_out_wall > min_dist_in_wall:
                '''if the distance to closer out wall is bigger than the distance 
                to the closer in wall, then keep the in wall distance'''
                dist.append(min_dist_in_wall)
                walls.append(inner_wall[wall_index_2])
            else:
                dist.append(min_dist_out_wall)
                walls.append(outer_wall[wall_index_1])
            angle += math.pi / 6
        return dist, walls

    def rotate(self, angle, r):
        # Rotate the robot at a certain angle from the x-axis
        front_x = self.x + np.cos(angle) * r
        front_y = self.y + np.sin(angle) * r
        return front_x, front_y

    def detectCollision(self, dist, wall, delta_t):
        collision_distance = min(dist)
        collision_wall = dist.index(collision_distance)
        if collision_distance < self.distance_limit:
            delta_t = delta_t * 2
        return delta_t

    def distance(self, wall, angle):
        def right_intersection(point, wall_point1, wall_point2, sensor_start, sensor_end):
            x, y = point
            right = (x - wall_point1[0]) * (x - wall_point2[0]) <= 0 and \
                    (y - wall_point1[1]) * (y - wall_point2[1]) <= 0 and \
                    (x - sensor_start[0]) * (sensor_end[0] - sensor_start[0]) >= 0 and \
                    (y - sensor_start[1]) * (sensor_end[1] - sensor_start[1]) >= 0
            return right

        dist = []
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

            if determinant != 0:  # if there is an intersectioin
                new_x = -(b1 * c2 - b2 * c1) / determinant  # intersection coordinate
                new_y = (a1 * c2 - a2 * c1) / determinant
                if abs(round(new_x) - new_x) < 0.00000001:
                    new_x = round(new_x)
                if abs(round(new_y) - new_y) < 0.00000001:
                    new_y = round(new_y)
                # make sure intersection is in front of the sensor
                if right_intersection((new_x, new_y), point1, point2, point3, point4):
                    dist.append(math.sqrt((new_x - point4[0]) ** 2 + (new_y - point4[1]) ** 2))
        if len(dist):
            min_dist_out_wall = min(dist)  # CLoser wall to sensor
            wall_index = dist.index(min_dist_out_wall)
        else:
            min_dist_out_wall = 1500
            wall_index = 0
        return min_dist_out_wall, wall_index
