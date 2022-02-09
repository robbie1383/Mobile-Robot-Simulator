import pygame

from Robot import Robot

WIDTH = 1000
HEIGHT = 1000

# A list of RGB values for the colours used in the game.
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LIGHTGRAY = (216, 220, 227)
DARKGRAY = (125, 130, 138)
GREEN = (120, 161, 88)
RED = (196, 96, 96)
BLUE = (86, 152, 209)
DARKBLUE = (52, 84, 112)
LIGHTBLUE = (129, 174, 214)
YELLOW = (245, 192, 47)
PURPLE = (114, 85, 163)

class Simulation ():

    def __init__(self):
        pygame.init()
        self.robot = Robot(WIDTH, HEIGHT, 101)
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Modular Robot Simulator")

        self.running = True
        self.clock = pygame.time.Clock()

    def run(self):
        """
        Runs the thread for the graphics.
        """
        while self.running:
            self.clock.tick(60)
            self.show()
            self.update()


    def show(self):
        """
        Fills the screen with the required graphics based on the map and the agents.
        """
        self.screen.fill(WHITE)
        pygame.draw.rect(self.screen, DARKGRAY, (1, 1, WIDTH-2, HEIGHT-2), 5)
        pygame.draw.circle(self.screen, PURPLE, (self.robot.x, self.robot.y), 50)
        pygame.draw.line(self.screen, BLACK, (self.robot.x, self.robot.y), (self.robot.frontX, self.robot.frontY), 1)
        pygame.display.flip()


    def update(self):
        """
        Updates the parameters for the game.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stop()
                pygame.quit()

        keys = pygame.key.get_pressed()
        movement = [keys[pygame.K_w], keys[pygame.K_s], keys[pygame.K_o], keys[pygame.K_l], keys[pygame.K_x], keys[pygame.K_t], keys[pygame.K_g]]

        self.robot.move(movement)


    def stop(self):
        """
        Stops the thread.
        """
        self.running = False
        exit()


def main():
    simulation = Simulation()
    simulation.run()


if __name__ == "__main__":
    main()