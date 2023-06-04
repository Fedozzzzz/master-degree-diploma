import random
import matplotlib.pyplot as plt

import numpy as np
import noise
import json

from constants import GRAPH_BASED_ALG, GREEDY_ALG, DEFAULT_ALG, REACHABILITY_ALG
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

    # plt.plot(delivery_points_nums, test_results_greedy, label="default", marker='o')  # Построение графика
    # plt.plot(delivery_points_nums, test_results_graph, label="modified", marker='o')  # Построение графика

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
    # island_map = IslandMap(island_map_coords, current_delivery_points.copy(), with_buildable_plan=True)
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
    delivery_algorithm = REACHABILITY_ALG if with_buildable_plan else DEFAULT_ALG

    task_planner = TaskPlanner(island_map, robots_courier, robots_builder, building_algorithm=building_algorithm,
                               delivery_algorithm=delivery_algorithm)

    # task_planner = TaskPlanner(island_map, robots_courier, robots_builder, building_algorithm=GRAPH_BASED_ALG,
    #                            delivery_algorithm=delivery_algorithm)

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
    num_of_pairs_arr_result = None

    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map])))

    delivery_points = generate_point_pairs(island_map_coords_reversed, num_of_delivery_points)

    result_test_courier_greedy = []
    result_test_builder_greedy = []

    result_test_courier_graph = []
    result_test_builder_graph = []

    num_of_iterations_max = num_of_iterations + 15

    success_count = 0
    i = 0
    while success_count < num_of_iterations:
        print('//////////////////////////////////////////////////////////////////////////////')
        print('NUM OF ITERATION: {}'.format(i))
        # test_results_greedy, test_results_graph, num_of_pairs_arr = test_num_of_pairs(island_map, buildable_coords, num_of_pairs, num_of_delivery_points)

        if i >= num_of_iterations_max:
            break

        i += 1
        try:
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

            num_of_pairs_arr_result = num_of_pairs_arr.copy()
            success_count += 1
        except Exception as e:
            print('--------------TASK FAILED-----------------')
            print('AN ERROR OCCURRED WHILE TEST RUNNING: {}'.format(e))
            print('PARAMS:')
            print('num of iteration: {}'.format(i))
            print('num_of_delivery_points: {}'.format(num_of_delivery_points))
            print('delivery_points: {}'.format(delivery_points))
            print("RUNNING TEST...")
            print('------------------------------------------')
            continue

        print('//////////////////////////////////////////////////////////////////////////////')

    print('RESULT NUM OF ITERATIONS: {}'.format(i))
    print('NUM OF SUCCESSFUL ITERATIONS: {}'.format(success_count))
    print('result_test_greedy: {}'.format(result_test_greedy))
    print('result_test_graph: {}'.format(result_test_graph))
    final_result_test_greedy = np.mean(result_test_greedy, axis=0)
    final_result_test_graph = np.mean(result_test_graph, axis=0)

    final_result_test_courier_greedy = np.mean(result_test_courier_greedy, axis=0)
    final_result_test_builder_greedy = np.mean(result_test_builder_greedy, axis=0)

    final_result_test_courier_graph = np.mean(result_test_courier_graph, axis=0)
    final_result_test_builder_graph = np.mean(result_test_builder_graph, axis=0)

    print('final_result_test_greedy: {}'.format(final_result_test_greedy))
    print('final_result_test_graph: {}'.format(final_result_test_graph))
    print('num_of_pairs_arr_result: {}'.format(num_of_pairs_arr_result))
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
    plot_result(num_of_pairs_arr_result, final_result_test_greedy, final_result_test_graph, title, xlabel, ylabel)

    title = '{} пар роботов-курьеров, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    plot_result(num_of_pairs_arr_result, final_result_test_courier_greedy, final_result_test_courier_graph, title,
                xlabel,
                ylabel)

    title = '{} пар роботов-строителей, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество пар роботов'
    ylabel = 'Средяя стоимость операций'
    plot_result(num_of_pairs_arr_result, final_result_test_builder_greedy, final_result_test_builder_graph, title,
                xlabel,
                ylabel)

    # Примените однофакторный ANOVA
    statistics_greedy, p_value_greedy = f_oneway(*result_test_greedy)
    statistics_graph, p_value_graph = f_oneway(*result_test_graph)

    statistics_courier_greedy, p_value_courier_greedy = f_oneway(*result_test_courier_greedy)
    statistics_builder_greedy, p_value_builder_greedy = f_oneway(*result_test_builder_greedy)

    statistics_courier_graph, p_value_courier_graph = f_oneway(*result_test_courier_graph)
    statistics_builder_graph, p_value_builder_graph = f_oneway(*result_test_builder_graph)

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

    performance_increase = [((graph - greedy) / greedy) * 100 for graph, greedy in
                            zip(final_result_test_graph, final_result_test_greedy)]
    performance_increase_percent = np.mean(performance_increase, axis=0)

    performance_increase_courier = [((graph - greedy) / greedy) * 100 for graph, greedy in
                                    zip(final_result_test_courier_graph, final_result_test_courier_greedy)]
    performance_increase_percent_courier = np.mean(performance_increase_courier, axis=0)

    performance_increase_builder = [((graph - greedy) / greedy) * 100 for graph, greedy in
                                    zip(final_result_test_builder_graph, final_result_test_builder_greedy)]
    performance_increase_percent_builder = np.mean(performance_increase_builder, axis=0)

    print('-------------------------------------------------')
    print('performance_increase_percent: {}'.format(performance_increase_percent))
    print('-------------------------------------------------')
    print('performance_increase_percent_courier: {}'.format(performance_increase_percent_courier))
    print('-------------------------------------------------')
    print('performance_increase_percent_builder: {}'.format(performance_increase_percent_builder))
    print('-------------------------------------------------')

    test_num_of_pairs_result = {
        "performance_increase_percent": performance_increase_percent,
        "performance_increase_percent_courier": performance_increase_percent_courier,
        "performance_increase_percent_builder": performance_increase_percent_builder,
        "greedy": {
            "statistics_greedy": statistics_greedy,
            "p_value_greedy": p_value_greedy,
        },
        "graph": {
            "statistics_graph": statistics_graph,
            "p_value_graph": p_value_graph,
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


def test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_pairs, num_of_delivery_points,
                                        num_of_iterations):
    result_test_greedy = []
    result_test_graph = []
    delivery_points_nums_result = None

    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map])))

    robots_builder_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs)
    robots_courier_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  forbidden_coords=robots_builder_coords)

    result_test_courier_greedy = []
    result_test_builder_greedy = []

    result_test_courier_graph = []
    result_test_builder_graph = []

    num_of_iterations_max = num_of_iterations + 20

    success_count = 0
    i = 0
    while success_count < num_of_iterations:
        print('//////////////////////////////////////////////////////////////////////////////')
        print('NUM OF ITERATION: {}'.format(i))
        # test_results_greedy, test_results_graph, num_of_pairs_arr = test_num_of_pairs(island_map, buildable_coords, num_of_pairs, num_of_delivery_points)
        if i >= num_of_iterations_max:
            break

        i += 1
        try:
            test_results_greedy, test_results_graph, delivery_points_nums, robots_results = test_num_of_delivery_points(
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

            delivery_points_nums_result = delivery_points_nums.copy()
            success_count += 1
        except Exception as e:
            print('--------------TASK FAILED-----------------')
            print('AN ERROR OCCURRED WHILE TEST RUNNING: {}'.format(e))
            print('PARAMS:')
            print('iteration: {}'.format(i))
            print('num_of_delivery_points: {}'.format(num_of_delivery_points))
            print('robots_builder_coords: {}'.format(robots_builder_coords))
            print('robots_courier_coords: {}'.format(robots_courier_coords))
            print('num_of_pairs: {}'.format(num_of_pairs))
            print("RUNNING TEST...")
            print('------------------------------------------')
            continue

        print('//////////////////////////////////////////////////////////////////////////////')

    print('RESULT NUM OF ITERATIONS: {}'.format(i))
    print('NUM OF SUCCESSFUL ITERATIONS: {}'.format(success_count))

    print('result_test_greedy: {}'.format(result_test_greedy))
    print('result_test_greedy.shape: {}'.format(np.array(result_test_greedy).shape))
    print('result_test_greedy len: {}'.format(len(result_test_greedy)))

    print('result_test_graph: {}'.format(result_test_graph))
    print('result_test_graph.shape: {}'.format(np.array(result_test_graph).shape))
    print('result_test_greedy len: {}'.format(len(result_test_graph)))

    final_result_test_greedy = np.mean(result_test_greedy, axis=0)
    final_result_test_graph = np.mean(result_test_graph, axis=0)

    final_result_test_courier_greedy = np.mean(result_test_courier_greedy, axis=0)
    final_result_test_builder_greedy = np.mean(result_test_builder_greedy, axis=0)

    final_result_test_courier_graph = np.mean(result_test_courier_graph, axis=0)
    final_result_test_builder_graph = np.mean(result_test_builder_graph, axis=0)

    print('final_result_test_greedy: {}'.format(final_result_test_greedy))
    print('final_result_test_graph: {}'.format(final_result_test_graph))
    print('delivery_points_nums_result: {}'.format(delivery_points_nums_result))
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
    plot_result(delivery_points_nums_result, final_result_test_greedy, final_result_test_graph, title, xlabel, ylabel)

    title = '{} пар роботов-курьеров, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    plot_result(delivery_points_nums_result, final_result_test_courier_greedy, final_result_test_courier_graph, title,
                xlabel,
                ylabel)

    title = '{} пар роботов-строителей, {} итераций, {} точек доставки'.format(
        num_of_pairs, num_of_iterations, num_of_delivery_points)
    xlabel = 'Количество точек доставки'
    ylabel = 'Средяя стоимость операций'
    plot_result(delivery_points_nums_result, final_result_test_builder_greedy, final_result_test_builder_graph, title,
                xlabel,
                ylabel)

    # Примените однофакторный ANOVA
    statistics_greedy, p_value_greedy = f_oneway(*result_test_greedy)
    statistics_graph, p_value_graph = f_oneway(*result_test_graph)

    statistics_courier_greedy, p_value_courier_greedy = f_oneway(*result_test_courier_greedy)
    statistics_builder_greedy, p_value_builder_greedy = f_oneway(*result_test_builder_greedy)

    statistics_courier_graph, p_value_courier_graph = f_oneway(*result_test_courier_graph)
    statistics_builder_graph, p_value_builder_graph = f_oneway(*result_test_builder_graph)

    statistics_total, p_value_total = f_oneway(*[final_result_test_greedy, final_result_test_graph])

    print('++++++++++++++++++++++++++++++++++++++++++++++++')
    print("ANOVA statistics_total:", statistics_total)
    print("p-value total:", p_value_total)
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

    # Выполните аппроксимацию линейной функцией для каждого графика
    coeffs_greedy = np.polyfit(delivery_points_nums_result, final_result_test_greedy,
                               1)  # аппроксимация для графика жадного алгоритма
    coeffs_graph = np.polyfit(delivery_points_nums_result, final_result_test_graph,
                              1)  # аппроксимация для графика на основе графов

    coeffs_courier_greedy = np.polyfit(delivery_points_nums_result, final_result_test_courier_greedy, 1)
    coeffs_builder_greedy = np.polyfit(delivery_points_nums_result, final_result_test_builder_greedy, 1)

    coeffs_courier_graph = np.polyfit(delivery_points_nums_result, final_result_test_courier_graph, 1)
    coeffs_builder_graph = np.polyfit(delivery_points_nums_result, final_result_test_builder_graph, 1)

    # Получите значения углов наклона
    slope_greedy = coeffs_greedy[0]  # угол наклона для графика жадного алгоритма
    slope_graph = coeffs_graph[0]  # угол наклона для графика на основе графов

    slope_courier_greedy = coeffs_courier_greedy[0]  # угол наклона для графика жадного алгоритма
    slope_builder_greedy = coeffs_builder_greedy[0]  # угол наклона для графика на основе графов

    slope_courier_graph = coeffs_courier_graph[0]  # угол наклона для графика жадного алгоритма
    slope_builder_graph = coeffs_builder_graph[0]  # угол наклона для графика на основе графов

    # Выведите результат
    # print("Угол наклона для графика жадного алгоритма: {:.2f}".format(slope_greedy))
    # print("Угол наклона для графика на основе графов: {:.2f}".format(slope_graph))

    print('-------------------------------------------------')
    print('slope_angle_greedy: {}'.format(slope_greedy))
    print('slope_angle_graph: {}'.format(slope_graph))
    print('-------------------------------------------------')
    print('slope_courier_greedy: {}'.format(slope_courier_greedy))
    print('slope_courier_graph: {}'.format(slope_courier_graph))
    print('-------------------------------------------------')
    print('slope_builder_greedy: {}'.format(slope_builder_greedy))
    print('slope_builder_graph: {}'.format(slope_builder_graph))
    print('-------------------------------------------------')

    performance_increase = [((graph - greedy) / greedy) * 100 for graph, greedy in
                            zip(final_result_test_graph, final_result_test_greedy)]
    performance_increase_percent = np.mean(performance_increase, axis=0)

    performance_increase_courier = [((graph - greedy) / greedy) * 100 for graph, greedy in
                                    zip(final_result_test_courier_graph, final_result_test_courier_greedy)]
    performance_increase_percent_courier = np.mean(performance_increase_courier, axis=0)

    performance_increase_builder = [((graph - greedy) / greedy) * 100 for graph, greedy in
                                    zip(final_result_test_builder_graph, final_result_test_builder_greedy)]
    performance_increase_percent_builder = np.mean(performance_increase_builder, axis=0)

    print('-------------------------------------------------')
    print('performance_increase_percent: {}'.format(performance_increase_percent))
    print('-------------------------------------------------')
    print('performance_increase_percent_courier: {}'.format(performance_increase_percent_courier))
    print('-------------------------------------------------')
    print('performance_increase_percent_builder: {}'.format(performance_increase_percent_builder))
    print('-------------------------------------------------')

    test_num_of_delivery_points_result = {
        "performance_increase_percent": performance_increase_percent,
        "performance_increase_percent_courier": performance_increase_percent_courier,
        "performance_increase_percent_builder": performance_increase_percent_builder,
        "slope_angle": {
            "greedy": {
                "slope_greedy": slope_greedy,
                "slope_courier_greedy": slope_courier_greedy,
                "slope_builder_greedy": slope_builder_greedy,
            },
            "graph": {
                "slope_graph": slope_graph,
                "slope_courier_graph": slope_courier_graph,
                "slope_builder_graph": slope_builder_graph,
            }
        },
        "greedy": {
            "statistics_greedy": statistics_greedy,
            "p_value_greedy": p_value_greedy,
        },
        "graph": {
            "statistics_graph": statistics_graph,
            "p_value_graph": p_value_graph,
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


def plot_3d_surface_result(x, y, z, title=None, xlabel=None, ylabel=None, zlabel=None):
    fig = plt.figure()
    # ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1 = fig.add_subplot(1, 1, 1, projection='3d')

    # Преобразование списков в массивы
    x = np.array(x)
    y = np.array(y)

    X, Y = np.meshgrid(x, y)

    z = np.array(z)

    ax1.set_xticks(x)
    ax1.set_yticks(y)
    # Построение поверхности
    ax1.plot_surface(X, Y, z, alpha=0.5, cmap='viridis')

    # Настройка осей и меток
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    ax1.set_zlabel(zlabel)

    ax1.scatter(X, Y, z, color='red')

    # Добавление заголовка
    ax1.set_title(title)

    plt.show()


def test_pairs_delivery_points(island_map, buildable_coords, num_of_pairs, num_of_delivery_points,
                               num_of_iterations):
    island_map_coords_reversed = list(map(list, zip(*[row[:] for row in island_map])))

    delivery_points = generate_point_pairs(island_map_coords_reversed, num_of_delivery_points)

    robots_builder_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  delivery_coords=delivery_points)
    robots_courier_coords = generate_robot_points(island_map_coords_reversed, num_of_pairs,
                                                  delivery_coords=delivery_points,
                                                  forbidden_coords=robots_builder_coords)

    result_test_courier_greedy = []
    result_test_builder_greedy = []

    result_test_courier_graph = []
    result_test_builder_graph = []

    result_performance_greedy = []
    result_performance_graph = []

    result_num_of_delivery_points = []
    result_num_of_pairs = []

    for curr_num_delivery_points in range(1, num_of_delivery_points + 1):

        curr_result_performance_greedy = []
        curr_result_performance_graph = []

        curr_result_test_courier_greedy = []
        curr_result_test_builder_greedy = []

        curr_result_test_courier_graph = []
        curr_result_test_builder_graph = []

        for curr_num_pairs in range(1, num_of_pairs + 1):
            current_delivery_points = delivery_points[:curr_num_delivery_points]
            current_buildable_coords = buildable_coords.copy()

            curr_robots_builder_points = robots_builder_coords[:curr_num_pairs]
            curr_robots_courier_points = robots_courier_coords[:curr_num_pairs]

            # print('current_delivery_points: {}'.format(current_delivery_points))
            print('CURRENT NUM OF DELIVERY POINTS: {}'.format(curr_num_delivery_points))
            print('CURRENT NUM OF PAIRS: {}'.format(curr_num_pairs))

            performance_greedy, couriers_operations_cost_greedy, builders_operations_cost_greedy = get_performance(
                island_map, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                robots_builder_points=curr_robots_builder_points, robots_courier_points=curr_robots_courier_points)

            performance_graph, couriers_operations_cost_graph, builders_operations_cost_graph = get_performance(
                island_map, current_buildable_coords.copy(),
                current_delivery_points.copy(),
                robots_builder_points=curr_robots_builder_points,
                robots_courier_points=curr_robots_courier_points, with_buildable_plan=True)

            curr_result_performance_greedy.append(performance_greedy)
            curr_result_performance_graph.append(performance_graph)

            curr_result_test_courier_greedy.append(couriers_operations_cost_greedy)
            curr_result_test_builder_greedy.append(builders_operations_cost_greedy)

            curr_result_test_courier_graph.append(couriers_operations_cost_graph)
            curr_result_test_builder_graph.append(builders_operations_cost_graph)

            if curr_num_delivery_points == 1:
                result_num_of_pairs.append(curr_num_pairs)

        result_performance_greedy.append(curr_result_performance_greedy)
        result_performance_graph.append(curr_result_performance_graph)

        result_test_courier_greedy.append(curr_result_test_courier_greedy)
        result_test_builder_greedy.append(curr_result_test_builder_greedy)

        result_test_courier_graph.append(curr_result_test_courier_graph)
        result_test_builder_graph.append(curr_result_test_builder_graph)

        result_num_of_delivery_points.append(curr_num_delivery_points)

    print("result_num_of_delivery_points: {}".format(result_num_of_delivery_points))
    print("result_num_of_pairs: {}".format(result_num_of_pairs))
    print('-----------------------------------------------------------------')
    print("result_performance_greedy: {}".format(result_performance_greedy))
    print("result_performance_graph: {}".format(result_performance_graph))
    print('-----------------------------------------------------------------')
    print("result_test_courier_greedy: {}".format(result_test_courier_greedy))
    print("result_test_builder_greedy: {}".format(result_test_builder_greedy))
    print('-----------------------------------------------------------------')
    print("result_test_courier_graph: {}".format(result_test_courier_graph))
    print("result_test_builder_graph: {}".format(result_test_builder_graph))

    # Поверхность для жадного алгоритма
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Total operation cost'
    title = 'Surface for GREEDY algorithm'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_performance_greedy, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)

    # Поверхность для алгоритма на графах
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Total operation cost'
    title = 'Surface for GRAPH algorithm'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_performance_graph, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)

    # Поверхность для жадного алгоритма (роботы-курьреры)
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Average operation cost'
    title = 'Surface for GREEDY algorithm couriers'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_test_courier_greedy, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)

    # Поверхность для алгоритма на графах (роботы-курьеры)
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Average operation cost'
    title = 'Surface for GRAPH algorithm couriers'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_test_courier_graph, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)

    # Поверхность для жадного алгоритма (роботы-строители)
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Average operation cost'
    title = 'Surface for GREEDY algorithm builders'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_test_builder_greedy, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)

    # Поверхность для алгоритма на графах (роботы-строители)
    xlabel = 'Num of pairs'
    ylabel = 'Num of Delivery points'
    zlabel = 'Average operation cost'
    title = 'Surface for GRAPH algorithm builders'
    plot_3d_surface_result(result_num_of_pairs, result_num_of_delivery_points, result_test_builder_graph, title=title,
                           xlabel=xlabel, ylabel=ylabel, zlabel=zlabel)


island_map, buildable_coords = generate_island_map()

test_pairs_delivery_points(island_map, buildable_coords, num_of_pairs=14, num_of_delivery_points=7,
                           num_of_iterations=1)

# test_num_of_delivery_points(island_map, buildable_coords)
# test_num_of_pairs(island_map, buildable_coords, 20)

# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=10, num_of_delivery_points=5, num_of_iterations=3)
# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=20, num_of_delivery_points=10,
#                           num_of_iterations=10)

# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=10, num_of_delivery_points=5,
#                           num_of_iterations=1)

# test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_delivery_points=20, num_of_pairs=4,
#                                     num_of_iterations=5)

# test_num_of_pairs_n_times(island_map, buildable_coords, num_of_pairs=20, num_of_delivery_points=10,
#                           num_of_iterations=20)
#
# test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_delivery_points=20, num_of_pairs=10,
#                                     num_of_iterations=20)
# test_num_of_delivery_points_n_times(island_map, buildable_coords, num_of_delivery_points=20, num_of_pairs=10, num_of_iterations=10)
