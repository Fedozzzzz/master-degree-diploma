import random

import numpy as np
import noise

from island_map import IslandMap
from island_map_gui import IslandMapGUI
from task_planner import TaskPlanner

# Размеры карты
width = 64
height = 64

# Масштаб и параметры шума
scale = 100.0
octaves = 5
persistence = 3.5
lacunarity = 1.8

# Масштаб и параметры шума
# scale = 100.0
# octaves = 6
# persistence = 3.5
# lacunarity = 1.75

# Создаем массив данных для карты высот
heightmap = np.zeros((height, width))

# Заполняем массив данных шумом Perlin noise
for y in range(height):
    for x in range(width):
        value = noise.pnoise2(x/scale, y/scale, octaves=octaves, persistence=persistence, lacunarity=lacunarity, repeatx=1024, repeaty=1024, base=0)
        heightmap[y][x] = value

island_map = np.zeros((height, width))
islands_coords = []
buildable_coords = []

# Добавляем острова на карту высот
for y in range(height):
    for x in range(width):
        if heightmap[y][x] > 0.005:
            island_map[y][x] = 1
            islands_coords.append((x, y))
        elif heightmap[y][x] > 0 and heightmap[y][x] < 0.004:
            island_map[y][x] = 0.5
            islands_coords.append((x, y))
            buildable_coords.append((x, y))


robot_builder_coord = (6, 6)
# robot_builder_coord = (9, 5)
robot_courier_coord = (5, 5)

destination_coord = (24, 26)
start_coord = (20, 6)

# island_map = IslandMap(heightmap, delivery_coords)
island_map = IslandMap(island_map)

delivery_coords = [(start_coord, destination_coord)]

buildable_coords = IslandMap.get_buildable_coords(island_map.island_map, width, height)

# robot_builder = RobotBuilderController(island_map, (width, height), robot_builder_coord, buildable_coords)

# robot_courier = RobotCourierController(island_map, (width, height), robot_courier_coord, delivery_coords)

# map_gui = IslandMapGUI(island_map, robots_builder=[robot_builder], robots_courier=[robot_courier])
map_gui = IslandMapGUI(island_map)

map_gui.draw()

# task_planner = TaskPlanner(island_map, [robot_courier], [robot_builder], gui=map_gui)
#
# task_planner.plan_and_execute_tasks()
# map_gui.draw()
