from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from math import sqrt
from PIL import Image


def find_shortest_path(start, end, island_map, is_all_walkable=False):
    # создание сетки карты
    grid = Grid(len(island_map[0]), len(island_map[0]))

    if not is_all_walkable:
        # добавление препятствий на картy
        for x in range(len(island_map[0])):
            for y in range(len(island_map[0])):
                if island_map[x][y] == 0:
                    # print('walkable false')
                    grid.node(x, y).walkable = False

    # определение начальной и конечной точки
    start_node = grid.node(start[0], start[1])

    # print('end point: {}', end)
    end_node = grid.node(end[0], end[1])

    # поиск кратчайшего пути
    finder = AStarFinder()
    path, _ = finder.find_path(start_node, end_node, grid)

    return path


def find_nearest_point(target, points):
    nearest_distance = float('inf')
    nearest_point = None
    for point in points:
        if point != target:
            distance = sqrt((point[0] - target[0]) ** 2 + (point[1] - target[1]) ** 2)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = point
    return nearest_point


def get_buildable_coords(island_map, width, height):
    res = []

    for x in range(width):
        for y in range(height):
            if island_map[x][y] == 0.5:
                res.append((y, x))

    return res


def get_departure_points(delivery_coords):
    return [dc[0] for dc in delivery_coords]


def find_destination_point(departure_point, delivery_coords):
    for dc in delivery_coords:
        if dc[0] == departure_point:
            return dc[1]

def print_map(new_map):
    new_map = list(map(list, zip(*new_map)))
    for el in new_map:
        print(el)


def update_map(island_map, updates):
    new_map = island_map

    for up in updates:
        x, y = up
        new_map[x][y] = 0.7

    return new_map


def show_map(island_map, path=None, robot_builder=None, robot_courier=None, point=None, destination_coord=None,
             start_coord=None):
    island_map = list(map(list, zip(*island_map)))
    width = len(island_map[0])
    height = len(island_map[0])
    image = Image.new("RGB", (width, height))

    for y in range(height):
        for x in range(width):
            if island_map[y][x] == 1:
                image.putpixel((x, y), (45, 105, 53))
            elif island_map[y][x] == 0.5:
                image.putpixel((x, y), (255, 247, 0))
            elif island_map[y][x] == 0.7:
                image.putpixel((x, y), (139, 69, 19))
            else:
                image.putpixel((x, y), (14, 61, 201))

    if path:
        for p in path:
            image.putpixel(p, (255, 153, 153))

    if robot_builder:
        image.putpixel(robot_builder, (255, 0, 0))

    if robot_courier:
        image.putpixel(robot_courier, (0, 0, 255))

    if point:
        image.putpixel(point, (153, 153, 153))

    if destination_coord:
        image.putpixel(destination_coord, (0, 0, 0))

    if start_coord:
        image.putpixel(start_coord, (153, 0, 153))

    image.show()
    # image.save("island_map_2.png", "PNG")


DELIVERY_TASK_TYPE = 'delivery'
BUILD_TASK_TYPE = 'build'
