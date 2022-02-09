import random
import math
import numpy as np

class Robot():

    def __init__(self, WIDTH, HEIGHT, size):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.radius = int (size/2)

        self.x = random.randint(4 + self.radius, WIDTH - self.radius - 4)
        self.y = random.randint(4 + self.radius, HEIGHT - self.radius - 4)

        self.frontX = self.x + self.radius
        self.frontY = self.y

        self.Vl = 0
        self.Vr = 0
        self.theta = 0
        self.speed = 0.5

    def move(self, movement): #[W, S, O, L, X, T, G]
        if self.hitWall():
            print("hit wall")

        if movement[0] == 1:
            self.Vl += self.speed
        if movement[1] == 1:
            self.Vl -= self.speed
        if movement[2] == 1:
            self.Vr += self.speed
        if movement[3] == 1:
            self.Vr -= self.speed
        if movement[4] == 1:
            self.Vl = 0
            self.Vr = 0
        if movement[5] == 1:
            self.Vl += self.speed
            self.Vr += self.speed
        if movement[6] == 1:
            self.Vl -= self.speed
            self.Vr -= self.speed

        if self.Vr != 0 or self.Vl != 0:
            print("Vl ", self.Vl)
            print("Vr ", self.Vr)
            print()

            if self.Vr == self.Vl:
                R = 100
                w = 0

            else :
                R = self.radius * (self.Vl + self.Vr)/(self.Vr - self.Vl)
                w = (self.Vr - self.Vl)/(self.radius*2)
                ICC = [self.x - R * np.sin(self.theta), self.y + R * np.cos(self.theta)]

                result = np.transpose(np.matmul(
                    np.array([[np.cos(w), -np.sin(w), 0],
                             [np.sin(w), np.cos(w), 0],
                            [0, 0, 1]]),
                    np.transpose(np.array([self.x - ICC[0], self.y - ICC[1], self.theta]))) + np.array([ICC[0], ICC[1], w])).transpose()
                self.x = result[0]
                self.y = result[1]
                self.theta = result[2]
            self.rotate(self.theta)


    def rotate(self, angle):
        self.frontX = self.x + np.cos(angle) * self.radius
        self.frontY = self.y + np.sin(angle) * self.radius

    def hitWall(self):
        return self.x <= 5 + self.radius or self.x >= self.WIDTH - self.radius - 5 \
               or self.y <= 5 + self.radius or self.y >= self.HEIGHT - self.radius - 5