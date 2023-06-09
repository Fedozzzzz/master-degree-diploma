import random

import numpy as np
import noise

from island_map import IslandMap
from island_map_gui import IslandMapGUI
from performance_control import PerformanceControl
from robot_builder_controller import RobotBuilderController
from robot_courier_controller import RobotCourierController
from task_planner import TaskPlanner

# Размеры карты
width = 128
height = 128

# # Масштаб и параметры шума
# scale = 100.0
# octaves = 5
# persistence = 3.5
# lacunarity = 1.8

# Масштаб и параметры шума
scale = 100.0
octaves = 6
persistence = 3.5
lacunarity = 1.75

# scale = 20.0  # Масштаб карты
# octaves = 8  # Количество октав
# persistence = 0.6  # Влияние каждой октавы
# lacunarity = 2.0  # Увеличение частоты шума с каждой октавой
# seed = 42  # Семя для генерации случайных чисел

# Создаем массив данных для карты высот
heightmap = np.zeros((height, width))

# Заполняем массив данных шумом Perlin noise
for y in range(height):
    for x in range(width):
        value = noise.pnoise2(x / scale, y / scale, octaves=octaves, persistence=persistence, lacunarity=lacunarity,
                              repeatx=1024, repeaty=1024, base=0)
        heightmap[y][x] = value

island_map = np.zeros((height, width))
islands_coords = []
buildable_coords = []

# Добавляем острова на карту высот
for y in range(height):
    for x in range(width):
        if heightmap[y][x] > 0.00005:
            island_map[y][x] = 1
            islands_coords.append((x, y))
        elif heightmap[y][x] > 0 and heightmap[y][x] < 0.00005:
            island_map[y][x] = 0.5
            islands_coords.append((x, y))
            buildable_coords.append((x, y))


robot_builder_coords = [(23, 41), (33, 18)]
# robot_builder_coords = [(3, 9)]
# robot_builder_coords = [(23, 41)]

robot_courier_coords = [(59, 43), (3, 9)]
# robot_courier_coords = [(33, 18)]
# robot_courier_coords = [(59, 43)]

destination_coord = [(14, 21), (40, 21)]
start_coords = [(1, 1), (17, 0)]

# island_map = IslandMap(heightmap, delivery_coords)

# delivery_coords = [((1, 1), (14, 21)), ((17, 0), (40, 21)), ((20, 32), (51, 39)), ((30, 58), (3, 33))]
delivery_coords = [((1, 1), (14, 21)), ((17, 0), (40, 21)), ((20, 32), (51, 39))]
# delivery_coords = [((1, 1), (14, 21)), ((17, 0), (40, 21))]
# delivery_coords = [((17, 0), (40, 21))]
# delivery_coords = [((1, 1), (14, 21))]

island_map = IslandMap(island_map, delivery_coords)
islands_coords = island_map.get_islands_coords()

random_two = random.sample(islands_coords, 2)

print(island_map.island_map)

max_bridge_length = 18

performance_control = PerformanceControl()


for y in range(height):
    for x in range(width):
        if 0.5 < island_map.island_map[x][y] < 1:
            print(island_map.island_map[x][y])

# robots_builder = [RobotBuilderController(island_map, (width, height), start_pos=rbc, buildable_coords=buildable_coords,
#                                          max_bridge_length=max_bridge_length, performance_control=performance_control)
#                   for rbc in robot_builder_coords]
# print('it is all good')
#
# robots_courier = [RobotCourierController(island_map, (width, height), start_pos=rcc, delivery_coords=delivery_coords,
#                                          performance_control=performance_control)
#                   for rcc in robot_courier_coords]

# map_gui = IslandMapGUI(island_map, robots_builder=robots_builder, robots_courier=robots_courier)
map_gui = IslandMapGUI(island_map)
print('---------------------------------------')
# map_gui = IslandMapGUI(island_map)

map_gui.draw()

# for r in robots_builder:
#     r.set_map_gui(map_gui)
#
# for r in robots_courier:
#     r.set_map_gui(map_gui)

# task_planner = TaskPlanner(island_map, robots_courier, robots_builder, gui=map_gui)

# task_planner.plan_and_execute_tasks()

# steps_amount = performance_control.get_steps_count()

# print("final steps_amount: {}".format(steps_amount))

map_gui.draw()

