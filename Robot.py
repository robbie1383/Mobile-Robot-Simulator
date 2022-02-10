import math
import random
import numpy as np


# calculate the slope of the line
def slope(wall):
    x1, y1 = wall[0]
    x2, y2 = wall[1]
    if x1 == x2:
        return math.inf
    else:
        return -1 / np.tan((y2 - y1) / (x2 - x1))


class Robot:

    def __init__(self, WIDTH, HEIGHT, size):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.radius = int(size / 2)

        self.x = random.randint(4 + self.radius, WIDTH - self.radius - 4)
        self.y = random.randint(4 + self.radius, HEIGHT - self.radius - 4)

        self.frontX = self.x + self.radius
        self.frontY = self.y

        self.Vl = 0
        self.Vr = 0
        self.theta = 0
        self.speed = 0.05
        self.closest_wall = []

    def move(self, movement, delta_t):
        if self.hitWall():
            # Add collision check
            print("hit wall")

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

            # Transfer results from the ICC computation
            self.x = result[0]
            self.y = result[1]
            self.theta = result[2]
            self.rotate(self.theta)

        return self.Vl, self.Vr, np.round(np.degrees(self.theta) % 360, 2)

    def rotate(self, angle):
        # Rotate the robot at a certain angle from the x-axis
        self.frontX = self.x + np.cos(angle) * self.radius
        self.frontY = self.y + np.sin(angle) * self.radius

    def hitWall(self):
        # Check if the robot is hitting the wall
        return self.x <= 5 + self.radius or self.x >= self.WIDTH - self.radius - 5 \
               or self.y <= 5 + self.radius or self.y >= self.HEIGHT - self.radius - 5

    def detectCollision(self, outer_wall, inner_wall):
        distance_to_wall = []
        for i in range(len(outer_wall) - 1):
            a, b = outer_wall[i]
            c, d = outer_wall[i + 1]
            if a == c:
                distance = abs(self.x - a)
                distance_to_wall.append(distance)
            else:
                distance = abs(self.y - b)
                distance_to_wall.append(distance)

        ds = distance_to_wall.copy()
        ds.sort()
        shortest = ds[0]
        index = distance_to_wall.index(ds[0])
        self.closest_wall = [outer_wall[index], outer_wall[index + 1]]
        return shortest
