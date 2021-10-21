import random
import math
import pygame
from enum import Enum
import time


class Colors(Enum):
    White = (255, 255, 255)
    Black = (0, 0, 0)
    Gray = (70, 70, 70)
    Red = (255, 0, 0)
    Green = (0, 255, 0)
    Blue = (0, 0, 255)
    Purple = (255, 0, 255)


class RRTMap:
    def __init__(self, start, goal, dimensions, obstacle_dimension, obstacle_number):
        self.start = start
        self.goal = goal
        self.dimensions = dimensions
        self.map_height, self.map_width = self.dimensions

        # Window settings
        self.map_window_name = 'RRT Path Planning'
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_width, self.map_height))
        self.map.fill(Colors.White.value)
        self.node_rad = 2
        self.node_thickness = 0
        self.edge_thickness = 1

        self.obstacles = []
        self.obstacle_dim = obstacle_dimension
        self.obstacle_num = obstacle_number

    def draw_map(self):
        pygame.draw.circle(self.map, Colors.Green.value, self.start, self.node_rad + 5.0)
        pygame.draw.circle(self.map, Colors.Purple.value, self.goal, self.node_rad + 20.1)

    def draw_path(self, path):
        for node in path:
            pygame.draw.circle(self.map, Colors.Red.value, node, self.node_rad + 3, 0)

    def draw_obstacles(self, obstacles):
        for obs in obstacles:
            pygame.draw.rect(self.map, Colors.Gray.value, obs)


class RRTGraph:
    def __init__(self, start, goal, dimensions, obstacle_dim, obstacle_num):
        self.start = start
        self.goal = goal
        self.flag = False
        self.map_height, self.map_width = dimensions

        # Init tree
        self.x = [start[0]]
        self.y = [start[1]]
        self.parent = [0]

        # Obstacles
        self.obstacles = []
        self.obstacle_dim = obstacle_dim
        self.obstacle_num = obstacle_num

        # Path
        self.goal_state = None
        self.path = []

    def make_random_rect(self):
        upper_corner_x = int(random.uniform(0, self.map_width - self.obstacle_dim))
        upper_corner_y = int(random.uniform(0, self.map_height - self.obstacle_dim))

        return upper_corner_x, upper_corner_y

    def make_obs(self):
        obs = []

        for i in range(0, self.obstacle_num):
            rect = None
            start_goal_col = True
            while start_goal_col:
                upper = self.make_random_rect()
                rect = pygame.Rect(upper, (self.obstacle_dim, self.obstacle_dim))
                if not (rect.collidepoint(self.start) or rect.collidepoint(self.goal)):
                    start_goal_col = False
            obs.append(rect)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]

        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2

        return (px + py)**0.5

    def sample_env(self):
        x = int(random.uniform(0, self.map_width))
        y = int(random.uniform(0, self.map_height))
        return x, y

    def nearest(self, n):
        distance_min = self.distance(0, n)
        node_near = 0
        for i in range(0, n):
            if self.distance(i, n) < distance_min:
                distance_min = self.distance(i, n)
                node_near = i
        return node_near

    def is_free(self):
        n = self.number_of_nodes() - 1
        x, y = self.x[n], self.y[n]
        for obs in self.obstacles:
            if obs.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def cross_obstacle(self, x1, y1, x2, y2):
        for obs in self.obstacles:
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if obs.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]
        if self.cross_obstacle(x1, y1, x2, y2):
            self.remove_node(n2)
            return False
        self.add_edge(n1, n2)
        return True

    def step(self, node_near, random_node, dist_max=35):
        dist = self.distance(node_near, random_node)
        if dist > dist_max:
            x_near, y_near = self.x[node_near], self.y[node_near]
            x_rand, y_rand = self.x[random_node], self.y[random_node]
            px, py = x_rand - x_near, y_rand - y_near
            theta = math.atan2(py, px)
            x, y = int(x_near + dist_max * math.cos(theta)), int(y_near + dist_max * math.sin(theta))
            self.remove_node(random_node)
            goal_x, goal_y = self.goal
            if abs(x - goal_x) < dist_max and abs(y - goal_y) < dist_max:
                self.add_node(random_node, goal_x, goal_y)
                self.goal_state = random_node
                self.flag = True
            else:
                self.add_node(random_node, x, y)

    def get_path(self):
        if self.flag:
            self.path = []
            self.path.append(self.goal_state)
            new_pos = self.parent[self.goal_state]
            while new_pos:
                self.path.append(new_pos)
                new_pos = self.parent[new_pos]
            self.path.append(0)
        return self.flag

    def get_coordinates(self):
        coords = []
        for node in self.path:
            x, y = self.x[node], self.y[node]
            coords.append((x, y))
        return coords

    def bias(self, goal):
        n = self.number_of_nodes()
        self.add_node(n, goal[0], goal[1])
        node_near = self.nearest(n)
        self.step(node_near, n)
        self.connect(node_near, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(n, x, y)
        if self.is_free():
            x_nearest = self.nearest(n)
            self.step(x_nearest, n)
            self.connect(x_nearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        pass


def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obstacles_dim = 30
    obstacles_num = 50
    start_time = time.time()
    pygame.init()
    game_map = RRTMap(start, goal, dimensions, obstacles_dim, obstacles_num)
    graph = RRTGraph(start, goal, dimensions, obstacles_dim, obstacles_num)

    obstacles = graph.make_obs()

    game_map.draw_map()
    game_map.draw_obstacles(obstacles)

    iteration = 0
    while not graph.get_path():
        time_passed = time.time() - start_time
        start_time = time.time()

        if time_passed > 10:
            raise

        if not iteration % 10:
            x, y, parent = graph.bias(goal)
        else:
            x, y, parent = graph.expand()

        pygame.draw.circle(game_map.map, Colors.Gray.value, (x[-1], y[-1]), game_map.node_rad + 2,
                           0)
        pygame.draw.line(game_map.map, Colors.Blue.value, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]),
                         game_map.edge_thickness)

        if not iteration % 5:
            pygame.display.update()
        iteration += 1
    game_map.draw_path(graph.get_coordinates())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except IndexError:
            result = False
