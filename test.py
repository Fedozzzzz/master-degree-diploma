import random
import matplotlib.pyplot as plt

import numpy as np
import noise
import json

from constants import GRAPH_BASED_ALG, GREEDY_ALG
from island_map import IslandMap
from island_map_gui import IslandMapGUI
from performance_control import PerformanceControl
from robot_builder_controller import RobotBuilderController
from robot_courier_controller import RobotCourierController
from task_planner import TaskPlanner
from scipy.stats import f_oneway

def generate_island_map():
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
            value = noise.pnoise2(x / scale, y / scale, octaves=octaves, persistence=persistence, lacunarity=lacunarity,
                                  repeatx=1024, repeaty=1024, base=0)
            heightmap[y][x] = value

    island_map = np.zeros((height, width))
    islands_coords = []
    buildable_coords = []
    threshold = 0.015

    # Добавляем острова на карту высот
    for y in range(height):
        for x in range(width):
            if heightmap[y][x] > threshold:
                island_map[y][x] = 1
                islands_coords.append((x, y))
            elif 0 < heightmap[y][x] <= threshold:
                island_map[y][x] = 0.5
                islands_coords.append((x, y))
                buildable_coords.append((x, y))

    return island_map, buildable_coords


def generate_point_pairs(island_map, N, forbidden_points=None):
    island_coords = []
    for i in range(len(island_map)):
        for j in range(len(island_map[i])):
            if island_map[i][j] == 1:
                island_coords.append((i, j))

    point_pairs = []

    for _ in range(N):
        if len(island_coords) < 2:
            break

        island1 = random.choice(island_coords)
        island_coords.remove(island1)

        island2 = random.choice(island_coords)
        island_coords.remove(island2)

        if forbidden_points is not None:
            # Проверка, что выбранные точки не являются запрещенными
            while (island1, island2) in forbidden_points or (island2, island1) in forbidden_points:
                # Если выбранные точки запрещены, выберите новые
                island1 = random.choice(island_coords)
                island_coords.remove(island1)

                island2 = random.choice(island_coords)
                island_coords.remove(island2)

        point_pairs.append((island1, island2))

    return point_pairs


def generate_robot_points(island_map, N, delivery_coords=None, forbidden_coords=None):
    island_coords = []
    for i in range(len(island_map)):
        for j in range(len(island_map[i])):
            if island_map[i][j] == 1:
                island_coords.append((i, j))

    robot_points = []

    for _ in range(N):
        if len(island_coords) == 0:
            break

        robot_point = random.choice(island_coords)
        island_coords.remove(robot_point)

        if delivery_coords is not None:
            # Проверка, что точка робота не пересекается с точками доставки
            while any(robot_point in delivery_coord for delivery_coord in delivery_coords):
                # Если точка робота пересекается с точкой доставки, выберите новую точку
                if len(island_coords) == 0:
                    break

                robot_point = random.choice(island_coords)
                island_coords.remove(robot_point)

        # Проверка, что точка робота не пересекается с запрещенными точками
        if forbidden_coords is not None:
            while robot_point in forbidden_coords:
                if len(island_coords) == 0:
                    break

                robot_point = random.choice(island_coords)
                island_coords.remove(robot_point)

        robot_points.append(robot_point)

    return robot_points


def plot_result(delivery_points_nums, test_results_greedy, test_results_graph, title=None, xlabel=None, ylabel=None):
    plt.plot(delivery_points_nums, test_results_greedy, label="greedy", marker='o')  # Построение графика
    plt.plot(delivery_points_nums, test_results_graph, label="graph", marker='o')  # Построение графика
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    # Создание легенды
    plt.legend()

    plt.grid(True)  # Добавление сетки на график

    plt.show()  # Отображение графика


def get_performance(island_map_coords, current_buildable_coords, current_delivery_points, robots_builder_points,
                    robots_courier_points, with_buildable_plan=False):
    island_map = IslandMap(island_map_coords, current_delivery_points.copy(), with_buildable_plan=with_buildable_plan)
    width = len(island_map_coords)
    height = width

    max_bridge_length = 18

    performance_control = PerformanceControl()

    robots_builder = [
        RobotBuilderController(island_map, (width, height), start_pos=rbc, buildable_coords=current_buildable_coords,
                               max_bridge_length=max_bridge_length, performance_control=performance_control)
        for rbc in robots_builder_points]

    robots_courier = [
        RobotCourierController(island_map, (width, height), start_pos=rcc,
                               delivery_coords=current_delivery_points.copy(),
                               performance_control=performance_control)
        for rcc in robots_courier_points]

    building_algorithm = GRAPH_BASED_ALG if with_buildable_plan else GREEDY_ALG

    task_planner = TaskPlanner(island_map, robots_courier, robots_builder, building_algorithm=building_algorithm)
    task_planner.plan_and_execute_tasks()

    steps_amount = performance_control.get_steps_count()

    robots_courier_operations_cost = performance_control.courier_robot_operations_cost_sum / len(robots_courier_points)
    robots_builder_operations_cost = performance_control.builder_robot_operations_cost_sum / len(robots_builder_points)

    return steps_amount, robots_courier_operations_cost, robots_builder_operations_cost


''' меняем итерационно количество пар роботов. их положение фиксировано. пололжение точек доставки фиксировано '''


def test_num_of_pairs(island_map_coords, buildable_coords, num_of_pairs, delivery_points_initial=None,
                      num_of_delivery_points=10):
    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map_coords])))
    delivery_points = delivery_points_initial or generate_point_pairs(island_map_coords_reversed,
                                                                      num_of_delivery_points)
    print(delivery_points)

    robots_builder_points = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  delivery_coords=delivery_points)
    print(robots_builder_points)

    robots_courier_points = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  delivery_coords=delivery_points,
                                                  forbidden_coords=robots_builder_points)
    print(robots_courier_points)

    test_results_greedy = []
    test_results_graph = []
    num_of_pairs_arr = []

    test_results_couriers_greedy = []
    test_results_builders_greedy = []

    test_results_couriers_graph = []
    test_results_builders_graph = []

    for current_num_of_pairs in range(1, num_of_pairs + 1):
        current_delivery_points = delivery_points.copy()
        current_buildable_coords = buildable_coords.copy()

        curr_robots_builder_points = robots_builder_points[:current_num_of_pairs]
        curr_robots_courier_points = robots_courier_points[:current_num_of_pairs]

        # print('current_delivery_points: {}'.format(current_delivery_points))
        print('CURRENT NUM OF PAIRS: {}'.format(current_num_of_pairs))
        print('curr_robots_builder_points: {}'.format(curr_robots_builder_points))
        print('curr_robots_courier_points: {}'.format(curr_robots_courier_points))

        try:
            performance_greedy, couriers_operations_cost_greedy, builders_operations_cost_greedy, = get_performance(
                island_map_coords, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                curr_robots_builder_points.copy(), curr_robots_courier_points.copy())

            performance_graph, couriers_operations_cost_graph, builders_operations_cost_graph, = get_performance(
                island_map_coords, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                curr_robots_builder_points.copy(), curr_robots_courier_points.copy(),
                with_buildable_plan=True)

            test_results_couriers_greedy.append(couriers_operations_cost_greedy)
            test_results_builders_greedy.append(builders_operations_cost_greedy)

            test_results_couriers_graph.append(couriers_operations_cost_graph)
            test_results_builders_graph.append(builders_operations_cost_graph)

            test_results_greedy.append(performance_greedy)
            test_results_graph.append(performance_graph)

            num_of_pairs_arr.append(current_num_of_pairs)

            print("final performance greedy: {}".format(performance_greedy))
            print("final performance graph: {}".format(performance_graph))
        except:
            print('--------------TASK FAILED-----------------')
            print('PARAMS:')
            print('current_delivery_points: {}'.format(current_delivery_points))
            print('curr_robots_builder_points: {}'.format(curr_robots_builder_points))
            print('curr_robots_courier_points: {}'.format(curr_robots_courier_points))
            break

    print('num_of_pairs_arr: {}'.format(num_of_pairs_arr))
    print('test_results_greedy: {}'.format(test_results_greedy))
    print('test_results_graph: {}'.format(test_results_graph))
    print('====================================================')
    print('test_results_couriers_greedy: {}'.format(test_results_couriers_greedy))
    print('test_results_builders_greedy: {}'.format(test_results_builders_greedy))
    print('====================================================')
    print('test_results_couriers_graph: {}'.format(test_results_couriers_graph))
    print('test_results_builders_graph: {}'.format(test_results_builders_graph))

    title = 'График для {} точек доставки'.format(num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Сумма стоимости всех операций системы'
    # plot_result(num_of_pairs_arr, test_results_greedy, test_results_graph, title, xlabel, ylabel)

    title = 'Средяя стоимость операций роботов-курьеров для {} точек доставки'.format(num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    # plot_result(num_of_pairs_arr, test_results_couriers_greedy, test_results_couriers_graph, title, xlabel, ylabel)

    title = 'Средяя стоимость операций роботов-строителей для {} точек доставки'.format(num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    # plot_result(num_of_pairs_arr, test_results_builders_greedy, test_results_builders_graph, title, xlabel, ylabel)

    robots_results = {
        "greedy": {
            "courier": test_results_couriers_greedy,
            "builder": test_results_builders_greedy,
        },
        "graph": {
            "courier": test_results_couriers_graph,
            "builder": test_results_builders_graph,
        }
    }

    # with open("num_of_pairs_test_results_greedy_array.json", "w") as file:
    #     json.dump(test_results_greedy, file)
    #
    # with open("num_of_pairs_test_results_graph_array.json", "w") as file:
    #     json.dump(test_results_graph, file)
    #
    # with open("num_of_pairs_array.json", "w") as file:
    #     json.dump(num_of_pairs_arr, file)

    return test_results_greedy, test_results_graph, num_of_pairs_arr, robots_results


''' меняем итеративно только количество точек доставки. их положение фиксировано. пололжение роботов фиксировано '''


def test_num_of_delivery_points(island_map_coords, buildable_coords, robots_builder_points_initial=None,
                                robots_courier_points_initial=None, delivery_points_initial=None,
                                num_of_delivery_points=20, num_of_pairs=2):
    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map_coords])))

    # delivery_points = delivery_points_initial or generate_point_pairs(island_map_coords_reversed,
    #                                                                   num_of_delivery_points, forbidden_points=np.array(
    #         np.concatenate(robots_builder_points_initial, robots_courier_points_initial)))

    delivery_points = []
    print('robots_builder_points_initial: {}'.format(robots_builder_points_initial))
    print('robots_courier_points_initial: {}'.format(robots_courier_points_initial))
    robots_coords = None
    if robots_courier_points_initial is not None and robots_builder_points_initial is not None:
        robots_coords = np.concatenate(
            (np.array(robots_builder_points_initial), np.array(robots_courier_points_initial)))

    delivery_points = generate_point_pairs(island_map_coords_reversed, num_of_delivery_points,
                                           forbidden_points=robots_coords)

    print(delivery_points)

    robots_builder_points = robots_builder_points_initial or generate_robot_points(island_map_coords_reversed,
                                                                                   num_of_pairs,
                                                                                   delivery_coords=delivery_points)
    print(robots_builder_points)

    robots_courier_points = robots_courier_points_initial or generate_robot_points(island_map_coords_reversed,
                                                                                   num_of_pairs,
                                                                                   delivery_coords=delivery_points,
                                                                                   forbidden_coords=robots_builder_points)
    test_results_greedy = []
    test_results_graph = []
    delivery_points_nums = []

    test_results_couriers_greedy = []
    test_results_builders_greedy = []

    test_results_couriers_graph = []
    test_results_builders_graph = []

    for i in range(1, len(delivery_points) + 1):
        current_delivery_points = delivery_points[:i]
        current_buildable_coords = buildable_coords.copy()

        print('current_delivery_points: {}'.format(current_delivery_points))
        print('NUM OF DELIVERY POINTS: {}'.format(len(current_delivery_points)))

        print(robots_courier_points)

        try:
            performance_greedy, couriers_operations_cost_greedy, builders_operations_cost_greedy = get_performance(
                island_map_coords, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                robots_builder_points, robots_courier_points)

            performance_graph, couriers_operations_cost_graph, builders_operations_cost_graph = get_performance(
                island_map_coords, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                robots_builder_points, robots_courier_points, with_buildable_plan=True)

            test_results_couriers_greedy.append(couriers_operations_cost_greedy)
            test_results_builders_greedy.append(builders_operations_cost_greedy)

            test_results_couriers_graph.append(couriers_operations_cost_graph)
            test_results_builders_graph.append(builders_operations_cost_graph)

            test_results_greedy.append(performance_greedy)
            test_results_graph.append(performance_graph)

            delivery_points_nums.append(len(current_delivery_points))

            print("final performance greedy: {}".format(performance_greedy))
            print("final performance graph: {}".format(performance_graph))
        except:
            print('--------------TASK FAILED-----------------')
            print('PARAMS:')
            print('current_delivery_points: {}'.format(current_delivery_points))
            print('robots_builder_points: {}'.format(robots_builder_points))
            print('robots_courier_points: {}'.format(robots_courier_points))
            break

    print('delivery_points_nums: {}'.format(delivery_points_nums))
    print('test_results_greedy: {}'.format(test_results_greedy))
    print('test_results_graph: {}'.format(test_results_graph))
    print('====================================================')
    print('test_results_couriers_greedy: {}'.format(test_results_couriers_greedy))
    print('test_results_builders_greedy: {}'.format(test_results_builders_greedy))
    print('====================================================')
    print('test_results_couriers_graph: {}'.format(test_results_couriers_graph))
    print('test_results_builders_graph: {}'.format(test_results_builders_graph))

    title = 'График для {} пар роботов'.format(num_of_pairs)
    xlabel = 'Количество точек доставки'
    ylabel = 'Сумма стоимости всех операций системы'
    # plot_result(delivery_points_nums, test_results_greedy, test_results_graph, title, xlabel, ylabel)

    title = 'Средяя стоимость операций роботов-курьеров для {} пар'.format(num_of_pairs)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    # plot_result(delivery_points_nums, test_results_couriers_greedy, test_results_couriers_graph, title, xlabel, ylabel)

    title = 'Средяя стоимость операций роботов-строителей для {} пар'.format(num_of_pairs)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    # plot_result(delivery_points_nums, test_results_builders_greedy, test_results_builders_graph, title, xlabel, ylabel)

    robots_results = {
        "greedy": {
            "courier": test_results_couriers_greedy,
            "builder": test_results_builders_greedy,
        },
        "graph": {
            "courier": test_results_couriers_graph,
            "builder": test_results_builders_graph,
        }
    }

    # with open("num_of_dp_test_results_greedy_array.json", "w") as file:
    #     json.dump(test_results_greedy, file)
    #
    # with open("num_of_dp_test_results_graph_array.json", "w") as file:
    #     json.dump(test_results_graph, file)
    #
    # with open("delivery_points_nums_array.json", "w") as file:
    #     json.dump(delivery_points_nums, file)

    return test_results_greedy, test_results_graph, delivery_points_nums, robots_results


def test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs, num_of_delivery_points, num_of_iterations):
    result_test_greedy = []
    result_test_graph = []
    num_of_pairs_arr = None

    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map])))

    delivery_points = generate_point_pairs(island_map_coords_reversed, num_of_delivery_points)

    result_test_courier_greedy = []
    result_test_builder_greedy = []

    result_test_courier_graph = []
    result_test_builder_graph = []

    for i in range(num_of_iterations):
        print('//////////////////////////////////////////////////////////////////////////////')
        print('NUM OF ITERATION: {}'.format(i))
        # test_results_greedy, test_results_graph, num_of_pairs_arr = test_num_of_pairs(island_map, buildable_coords, num_of_pairs, num_of_delivery_points)
        test_results_greedy, test_results_graph, num_of_pairs_arr, robots_results = test_num_of_pairs(
            island_map,
            buildable_coords,
            num_of_pairs,
            delivery_points_initial=delivery_points.copy(),
            num_of_delivery_points=num_of_delivery_points)

        result_test_greedy.append(test_results_greedy)
        result_test_graph.append(test_results_graph)

        result_test_courier_greedy.append(robots_results["greedy"]["courier"])
        result_test_builder_greedy.append(robots_results["greedy"]["builder"])

        result_test_courier_graph.append(robots_results["graph"]["courier"])
        result_test_builder_graph.append(robots_results["graph"]["builder"])

        if i == 0:
            num_of_pairs_arr = num_of_pairs_arr.copy()
        print('//////////////////////////////////////////////////////////////////////////////')

    final_result_test_greedy = np.mean(result_test_greedy, axis=0)
    final_result_test_graph = np.mean(result_test_graph, axis=0)

    final_result_test_courier_greedy = np.mean(result_test_courier_greedy, axis=0)
    final_result_test_builder_greedy = np.mean(result_test_builder_greedy, axis=0)

    final_result_test_courier_graph = np.mean(result_test_courier_graph, axis=0)
    final_result_test_builder_graph = np.mean(result_test_builder_graph, axis=0)

    print('final_result_test_greedy: {}'.format(final_result_test_greedy))
    print('final_result_test_graph: {}'.format(final_result_test_graph))
    print('num_of_pairs_arr: {}'.format(num_of_pairs_arr))
    print('====================================================')
    print('final_result_test_courier_greedy: {}'.format(final_result_test_courier_greedy))
    print('final_result_test_builder_greedy: {}'.format(final_result_test_builder_greedy))
    print('====================================================')
    print('final_result_test_courier_graph: {}'.format(final_result_test_courier_graph))
    print('final_result_test_builder_graph: {}'.format(final_result_test_builder_graph))

    title = 'График для {} точек доставки для {} итераций '.format(
        num_of_delivery_points, num_of_iterations)
    xlabel = 'Количество пар роботов'
    ylabel = 'Сумма стоимости всех операций системы'
    plot_result(num_of_pairs_arr, final_result_test_greedy, final_result_test_graph, title, xlabel, ylabel)

    title = '{} пар роботов-курьеров, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    plot_result(num_of_pairs_arr, final_result_test_courier_greedy, final_result_test_courier_graph, title, xlabel,
                ylabel)

    title = '{} пар роботов-строителей, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    plot_result(num_of_pairs_arr, final_result_test_builder_greedy, final_result_test_builder_graph, title, xlabel,
                ylabel)

    # Примените однофакторный ANOVA
    statistics_greedy, p_value_greedy = f_oneway(*result_test_greedy)
    statistics_graph, p_value_graph = f_oneway(*result_test_graph)

    statistics_courier_greedy, p_value_courier_greedy = f_oneway(*result_test_courier_greedy)
    statistics_builder_greedy, p_value_builder_greedy = f_oneway(*result_test_builder_greedy)

    statistics_courier_graph, p_value_courier_graph = f_oneway(*result_test_courier_graph)
    statistics_builder_graph, p_value_builder_graph = f_oneway(*result_test_builder_graph)

    test_num_of_pairs_result = {
        "greedy": {
            "statistics_greedy": statistics_greedy,
            "p_value_greedy": p_value_greedy,
        },
        "graph": {
            "courier": statistics_graph,
            "builder": p_value_graph,
        },
        "courier_greedy": {
            "statistics_courier_greedy": statistics_courier_greedy,
            "p_value_courier_greedy": p_value_courier_greedy,
        },
        "builder_greedy": {
            "statistics_builder_greedy": statistics_builder_greedy,
            "p_value_builder_greedy": p_value_builder_greedy,
        },
        "courier_graph": {
            "statistics_courier_graph": statistics_courier_graph,
            "p_value_courier_graph": p_value_courier_graph,
        },
        "builder_graph": {
            "statistics_builder_greedy": statistics_builder_graph,
            "p_value_builder_graph": p_value_builder_graph,
        },
    }

    with open("test_num_of_pairs_result.json", "w") as file:
        json.dump(test_num_of_pairs_result, file)

    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_greedy:", statistics_greedy)
    print("p-value greedy:", p_value_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_graph:", statistics_graph)
    print("p-value graph:", p_value_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_courier_greedy:", statistics_courier_greedy)
    print("p_value_courier_greedy:", p_value_courier_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_builder_greedy:", statistics_builder_greedy)
    print("p_value_builder_greedy:", p_value_builder_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_courier_graph:", statistics_courier_graph)
    print("p_value_courier_graph:", p_value_courier_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_builder_graph:", statistics_builder_graph)
    print("p_value_builder_graph:", p_value_builder_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')


def test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_pairs, num_of_delivery_points,
                                        num_of_iterations):
    result_test_greedy = []
    result_test_graph = []
    delivery_points_nums = None

    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map])))

    robots_builder_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs)
    robots_courier_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  forbidden_coords=robots_builder_coords)

    result_test_courier_greedy = []
    result_test_builder_greedy = []

    result_test_courier_graph = []
    result_test_builder_graph = []

    for i in range(num_of_iterations):
        print('//////////////////////////////////////////////////////////////////////////////')
        print('NUM OF ITERATION: {}'.format(i))
        # test_results_greedy, test_results_graph, num_of_pairs_arr = test_num_of_pairs(island_map, buildable_coords, num_of_pairs, num_of_delivery_points)
        test_results_greedy, test_results_graph, num_of_pairs_arr, robots_results = test_num_of_delivery_points(
            island_map,
            buildable_coords,
            num_of_delivery_points=num_of_delivery_points,
            robots_builder_points_initial=robots_builder_coords,
            robots_courier_points_initial=robots_courier_coords,
            num_of_pairs=num_of_pairs)

        result_test_greedy.append(test_results_greedy)
        result_test_graph.append(test_results_graph)

        result_test_courier_greedy.append(robots_results["greedy"]["courier"])
        result_test_builder_greedy.append(robots_results["greedy"]["builder"])

        result_test_courier_graph.append(robots_results["graph"]["courier"])
        result_test_builder_graph.append(robots_results["graph"]["builder"])

        if i == 0:
            delivery_points_nums = num_of_pairs_arr.copy()
        print('//////////////////////////////////////////////////////////////////////////////')

    final_result_test_greedy = np.mean(result_test_greedy, axis=0)
    final_result_test_graph = np.mean(result_test_graph, axis=0)

    final_result_test_courier_greedy = np.mean(result_test_courier_greedy, axis=0)
    final_result_test_builder_greedy = np.mean(result_test_builder_greedy, axis=0)

    final_result_test_courier_graph = np.mean(result_test_courier_graph, axis=0)
    final_result_test_builder_graph = np.mean(result_test_builder_graph, axis=0)

    print('final_result_test_greedy: {}'.format(final_result_test_greedy))
    print('final_result_test_graph: {}'.format(final_result_test_graph))
    print('delivery_points_nums: {}'.format(delivery_points_nums))
    print('====================================================')
    print('final_result_test_courier_greedy: {}'.format(final_result_test_courier_greedy))
    print('final_result_test_builder_greedy: {}'.format(final_result_test_builder_greedy))
    print('====================================================')
    print('final_result_test_courier_graph: {}'.format(final_result_test_courier_graph))
    print('final_result_test_builder_graph: {}'.format(final_result_test_builder_graph))

    title = 'График для {} пар роботов для {} итераций '.format(
        num_of_pairs, num_of_iterations)
    xlabel = 'Количество точек доставки'
    ylabel = 'Сумма стоимости всех операций системы'
    plot_result(delivery_points_nums, final_result_test_greedy, final_result_test_graph, title, xlabel, ylabel)

    title = '{} пар роботов-курьеров, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    plot_result(delivery_points_nums, final_result_test_courier_greedy, final_result_test_courier_graph, title, xlabel,
                ylabel)

    title = '{} пар роботов-курьеров, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    plot_result(delivery_points_nums, final_result_test_builder_greedy, final_result_test_builder_graph, title, xlabel,
                ylabel)

    # Примените однофакторный ANOVA
    statistics_greedy, p_value_greedy = f_oneway(*result_test_greedy)
    statistics_graph, p_value_graph = f_oneway(*result_test_graph)

    statistics_courier_greedy, p_value_courier_greedy = f_oneway(*result_test_courier_greedy)
    statistics_builder_greedy, p_value_builder_greedy = f_oneway(*result_test_builder_greedy)

    statistics_courier_graph, p_value_courier_graph = f_oneway(*result_test_courier_graph)
    statistics_builder_graph, p_value_builder_graph = f_oneway(*result_test_builder_graph)

    test_num_of_delivery_points_result = {
        "greedy": {
            "statistics_greedy": statistics_greedy,
            "p_value_greedy": p_value_greedy,
        },
        "graph": {
            "courier": statistics_graph,
            "builder": p_value_graph,
        },
        "courier_greedy": {
            "statistics_courier_greedy": statistics_courier_greedy,
            "p_value_courier_greedy": p_value_courier_greedy,
        },
        "builder_greedy": {
            "statistics_builder_greedy": statistics_builder_greedy,
            "p_value_builder_greedy": p_value_builder_greedy,
        },
        "courier_graph": {
            "statistics_courier_graph": statistics_courier_graph,
            "p_value_courier_graph": p_value_courier_graph,
        },
        "builder_graph": {
            "statistics_builder_greedy": statistics_builder_graph,
            "p_value_builder_graph": p_value_builder_graph,
        },
    }

    with open("test_num_of_delivery_points_result.json", "w") as file:
        json.dump(test_num_of_delivery_points_result, file)

    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_greedy:", statistics_greedy)
    print("p-value greedy:", p_value_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_graph:", statistics_graph)
    print("p-value graph:", p_value_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_courier_greedy:", statistics_courier_greedy)
    print("p_value_courier_greedy:", p_value_courier_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_builder_greedy:", statistics_builder_greedy)
    print("p_value_builder_greedy:", p_value_builder_greedy)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_courier_graph:", statistics_courier_graph)
    print("p_value_courier_graph:", p_value_courier_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_builder_graph:", statistics_builder_graph)
    print("p_value_builder_graph:", p_value_builder_graph)
    print('++++++++++++++++++++++++++++++++++++++++++++++++')


island_map, buildable_coords = generate_island_map()
# test_num_of_delivery_points(island_map, buildable_coords)
# test_num_of_pairs(island_map, buildable_coords, 20)

# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=10, num_of_delivery_points=5, num_of_iterations=3)
# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=20, num_of_delivery_points=10,
#                           num_of_iterations=10)

# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=10, num_of_delivery_points=5,
#                           num_of_iterations=10)


test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_delivery_points=20, num_of_pairs=10,
                                    num_of_iterations=10)

test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=20, num_of_delivery_points=10,
                          num_of_iterations=10)

# test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_delivery_points=20, num_of_pairs=10, num_of_iterations=10)
