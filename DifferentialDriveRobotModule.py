import pygame
import math
from enum import Enum


class Colors(Enum):
    White = (255, 255, 255)
    Black = (0, 0, 0)
    Gray = (70, 70, 70)
    Red = (255, 0, 0)
    Green = (0, 255, 0)
    Blue = (0, 0, 255)
    Purple = (255, 0, 255)


class Environment:
    def __init__(self, dimension):
        self.height, self.width = dimension

        # Window settings
        self.map_window_name = 'Differential Drive Robot'
        pygame.display.set_caption(self.map_window_name)
        self.game_map = pygame.display.set_mode((self.width, self.height))

        # Text settings
        self.font = pygame.font.Font('freesansbold.ttf', 50)
        self.text = self.font.render('default', True, Colors.White.value, Colors.Black.value)
        self.text_rect = self.text.get_rect()
        self.text_rect.center = (dimension[1] - 600, dimension[0] - 100)

        self.trail_set = []

    def write_info(self, vel_left, vel_right, theta):
        txt = f'Vel_Left: {vel_left}, Vel_Right: {vel_right}, theta: {math.degrees(theta)}'
        self.text = self.font.render(txt, True, Colors.White.value, Colors.Black.value)
        self.game_map.blit(self.text, self.text_rect)

    def draw_trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.game_map, Colors.Purple.value, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]))

        if self.trail_set.__sizeof__() > 2000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def robot_frame(self, position, rotation):
        n = 80

        center_x, center_y = position
        x_axis = (center_x + n * math.cos(-rotation), center_y + n * math.sin(-rotation))
        y_axis = (center_x + n * math.cos(-rotation + math.pi / 2), center_y + n * math.sin(-rotation + math.pi / 2))

        pygame.draw.line(self.game_map, Colors.Red.value, (center_x, center_y), x_axis, 3)
        pygame.draw.line(self.game_map, Colors.Green.value, (center_x, center_y), y_axis, 3)


class Robot:
    def __init__(self, start_pos, robot_img, width):
        # Convert meters to pixels
        self.m2p = 3779.52

        # Robot dimensions
        self.width = width * self.m2p
        self.x, self.y = start_pos
        self.theta = 0
        self.a = 20
        self.u = 0.01 * self.m2p  # Linear speed
        self.w = 0  # Rotational Speed
        self.max_speed = 0.02 * self.m2p
        self.min_speed = -0.02 * self.m2p

        # Graphics
        self.img = pygame.image.load(robot_img)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, robot_map):
        robot_map.blit(self.rotated, self.rect)

    def move(self, event=None, delta_time=0, key_control=False):
        self.x += (self.u * math.cos(self.theta) - self.a * math.sin(self.theta) * self.w) * delta_time
        self.y += (self.u * math.sin(self.theta) + self.a * math.cos(self.theta) * self.w) * delta_time
        self.theta += self.w * delta_time

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        if key_control:
            if event is not None:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_KP4:
                        self.u += 0.01 * self.m2p
                    elif event.key == pygame.K_KP1:
                        self.u -= 0.01 * self.m2p
                    elif event.key == pygame.K_KP6:
                        self.w += 0.01 * self.m2p
                    elif event.key == pygame.K_KP3:
                        self.w -= 0.01 * self.m2p

            if self.u <= 0:
                self.u = 0
            if self.w <= 0:
                self.w = 0

    @staticmethod
    def distance(pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)

        px = (x1 - x2) ** 2
        py = (y1 - y2) ** 2

        return (px + py) ** 0.5

    def follow(self, coordinates):
        delta_x = coordinates[0] - self.x
        delta_y = coordinates[1] - self.y
        self.u = delta_x * math.cos(self.theta) + delta_y * math.sin(self.theta)
        self.w = (-1 / self.a) * math.sin(self.theta) * delta_x + (1 / self.a) * math.cos(self.theta) * delta_y


def main():
    pygame.init()

    start = (200, 200)
    dimensions = (800, 1200)
    delta_time = 0
    last_time = pygame.time.get_ticks()
    running = True
    env = Environment(dimensions)
    robot = Robot(start, 'Resources/car.png', 0.01)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            robot.move(event, delta_time, key_control=True)
        delta_time = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()
        pygame.display.update()
        env.game_map.fill(Colors.Black.value)
        robot.move(delta_time=delta_time)
        delta_time = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()
        # env.write_info(int(robot.vel_left), int(robot.vel_right), int(robot.theta))
        robot.draw(env.game_map)
        env.robot_frame((robot.x, robot.y), robot.theta)
        env.draw_trail((robot.x, robot.y))


if __name__ == '__main__':
    main()
