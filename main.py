import pygame
from RRTModule import RRTMap
from RRTModule import RRTGraph
from RRTModule import Colors as color
from DifferentialDriveRobotModule import Robot
from DifferentialDriveRobotModule import Environment
import time

def main():
    pygame.init()
    running = True
    space_key_press = False
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obstacles_dim = 100
    obstacles_num = 20
    game_map = RRTMap(start, goal, dimensions, obstacles_dim, obstacles_num)
    graph = RRTGraph(start, goal, dimensions, obstacles_dim, obstacles_num)
    obstacles = graph.make_obs()

    game_map.draw_map()
    game_map.draw_obstacles(obstacles)

    robot = Robot(start, 'Resources/car.png', 0.01)
    iteration = 0
    delta_time = 0
    start_time = pygame.time.get_ticks()
    while not graph.get_path():
        delta_time = (pygame.time.get_ticks() - start_time) / 1000
        start_time = pygame.time.get_ticks()

        if delta_time > 10:
            raise

        if not iteration % 10:
            x, y, parent = graph.bias(goal)
        else:
            x, y, parent = graph.expand()

        pygame.draw.circle(game_map.map, color.Gray.value, (x[-1], y[-1]), game_map.node_rad + 2,
                           0)
        pygame.draw.line(game_map.map, color.Blue.value, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]),
                         game_map.edge_thickness)

        if not iteration % 5:
            pygame.display.update()
        iteration += 1
    coords = graph.get_coordinates()
    game_map.draw_path(coords)
    while not space_key_press:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    space_key_press = True
    game_map.map.fill(color.White.value)
    game_map.draw_map()
    new_obstacles = []
    for obs in obstacles:
        new_obstacles.append(obs.inflate(-90, -90))
    game_map.draw_obstacles(new_obstacles)
    for coord in coords[::-1]:
        while robot.distance((robot.x, robot.y), coord) > 5 and running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            delta_time = (pygame.time.get_ticks() - start_time) / 1000
            start_time = pygame.time.get_ticks()
            pygame.display.update()
            robot.follow(coord)
            robot.move(delta_time=delta_time)
            game_map.map.fill(color.White.value)
            game_map.draw_map()
            game_map.draw_obstacles(new_obstacles)
            game_map.draw_path(coords)
            robot.draw(game_map.map)
    pygame.event.clear()
    pygame.event.wait(0)
    # start_time = time.time()
    # pygame.init()
    # game_map = RRTMap(start, goal, dimensions, obstacles_dim, obstacles_num)
    # graph = RRTGraph(start, goal, dimensions, obstacles_dim, obstacles_num)
    #
    # obstacles = graph.make_obs()
    #
    # game_map.draw_map()
    # game_map.draw_obstacles(obstacles)
    #
    # iteration = 0
    # while not graph.get_path():
    #     time_passed = time.time() - start_time
    #     start_time = time.time()
    #
    #     if time_passed > 10:
    #         raise
    #
    #     if not iteration % 10:
    #         x, y, parent = graph.bias(goal)
    #     else:
    #         x, y, parent = graph.expand()
    #
    #     pygame.draw.circle(game_map.map, color.Gray.value, (x[-1], y[-1]), game_map.node_rad + 2,
    #                        0)
    #     pygame.draw.line(game_map.map, color.Blue.value, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]),
    #                      game_map.edge_thickness)
    #
    #     if not iteration % 5:
    #         pygame.display.update()
    #     iteration += 1
    # game_map.draw_path(graph.get_coordinates())
    # pygame.display.update()
    # pygame.event.clear()
    # pygame.event.wait(0)


if __name__ == '__main__':
    main()
