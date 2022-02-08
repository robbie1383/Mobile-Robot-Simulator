import random

class Robot():

    def __init__(self, WIDTH, HEIGHT, size):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.size = size

        self.x = random.randint(4 + size, WIDTH - size - 4)
        self.y = random.randint(4 + size, HEIGHT - size - 4)

        self.Vl = 0
        self.Vr = 0
        self.theta = 0

    def move(self, movement):
        if self.hitWall():
            print("hit wall")
            return

        if movement[0] == 1:
            self.Vl += 1
            self.theta += 1
            self.x += 1

        self.theta = self.theta % 361
        return self.theta

    def hitWall(self):
        return self.x <= 4 + self.size or self.x >= self.WIDTH - self.size - 4 \
               or self.y <= 4 + self.size or self.y >= self.HEIGHT - self.size - 4