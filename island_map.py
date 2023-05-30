import networkx as nx
from matplotlib import pyplot as plt
import json

from constants import MAX_BRIDGE_LENGTH
from utils import find_shortest_path



class IslandMap:
    def __init__(self, map_array, delivery_coords=None, robots_builder=None, robots_courier=None, max_bridge_length=MAX_BRIDGE_LENGTH, with_buildable_plan=False):
        self.width = len(map_array[0])
        self.height = len(map_array)
        self.map_size = (self.width, self.height)
        self.island_map = list(map(list, zip(*[row[:] for row in map_array])))
        self.buildable_coords = self.get_buildable_coords(self.island_map, self.width, self.height)
        self.delivery_coords = delivery_coords
        self.map_states = {
            'island': 1.0,
            'bridge': 0.7,
            'buildable': 0.5,
            'water': 0.0,
        }
        if with_buildable_plan:
            self.islands = IslandMap.divide_into_islands(self.island_map)
            self.buildable_plan = IslandMap.create_buildable_plan(self.islands, self.island_map, max_bridge_length)

    def is_obstacle(self, x, y):
        return self.island_map[y][x] == 1

    def is_within_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def set_delivery_coords(self, delivery_coords):
        self.delivery_coords = delivery_coords

    def check_path_over_land(self, path):
        print('check_path_over_land path: {}', path)
        for p in path:
            x, y = p
            print('check_path_over_land: {}'.format(self.island_map[x][y]))
            if self.island_map[x][y] == 1.0 or self.island_map[x][y] == 0.7:
                return True
        return False

    def get_islands_coords(self):
        land_coords = []
        for i in range(len(self.island_map)):
            for j in range(len(self.island_map[0])):
                if self.island_map[i][j] == 1:
                    land_coords.append((i, j))
        return land_coords

    def get_not_transposed_map_matrix(self):
        return list(map(list, zip(*[row[:] for row in self.island_map])))

    def build_bridge(self, updates):
        for up in updates:
            x, y = up
            self.island_map[x][y] = 0.7

    def get_island_by_coord(self, coord):
        if not self.islands or not self.buildable_plan:
            return

        for island in self.buildable_plan.nodes():
            if coord in list(island):
                return island

    def get_local_buildable_plan(self, island_start, island_end):
        # shortest_path_dijkstra = nx.shortest_path(self.buildable_plan, island_start, island_end, weight='weight')
        # print("[get_local_buildable_plan] Shortest path (Dijkstra):", shortest_path_dijkstra)

        shortest_path_astar = nx.astar_path(self.buildable_plan, island_start, island_end, weight='weight')
        print("[get_local_buildable_plan] Shortest path (A*):", shortest_path_astar)

        shortest_path_astar_edges = [(shortest_path_astar[i], shortest_path_astar[i+1]) for i in range(len(shortest_path_astar)-1)]

        res = []
        for u, v in shortest_path_astar_edges:
            print("Vertex:", u)
            print("Edge:", self.buildable_plan[u][v])
            res.append(self.buildable_plan[u][v]['bridge'])

        return res

    @staticmethod
    def get_buildable_coords(island_map, width, height):
        res = []
        for x in range(width):
            for y in range(height):
                if island_map[x][y] == 0.5:
                    res.append((y, x))
        return res

    @staticmethod
    def get_buildable_coords_from_island(island_map, island):
        res = []
        for coord in island:
            x, y = coord
            if island_map[x][y] == 0.5:
                res.append((x, y))
        return res

    @staticmethod
    def divide_into_islands(island_map):
        # Создаем граф
        graph = nx.Graph()

        # Проходимся по каждой ячейке карты
        for i in range(len(island_map)):
            for j in range(len(island_map[i])):
                # Если ячейка является островом, добавляем ее в граф
                if island_map[i][j] == 1 or island_map[i][j] == 0.5:
                    graph.add_node((i, j))

                    # Проверяем соседние ячейки
                    if i > 0 and (island_map[i - 1][j] == 1 or island_map[i - 1][j] == 0.5):
                        graph.add_edge((i, j), (i - 1, j))
                    if j > 0 and (island_map[i][j - 1] == 1 or island_map[i][j - 1] == 0.5):
                        graph.add_edge((i, j), (i, j - 1))

        # Используем disjoint-set для разбиения графа на компоненты связности
        islands = list(nx.connected_components(graph))

        return islands

    @staticmethod
    def create_buildable_plan(islands_coords, island_map, max_bridge_length):
        try:
            # Открываем файл JSON
            with open("buildable_plan_graph.json", "r") as file:
                data = json.load(file)        # Создаем пустой граф
            graph = nx.Graph()

            # Добавление узлов
            for node_data in data['nodes']:
                node_id = tuple(map(tuple, node_data['id']))
                graph.add_node(node_id)

            # Добавление ребер
            for link_data in data['links']:
                # print('link_data: {}'.format(link_data))
                source = tuple(map(tuple, link_data['source']))
                target = tuple(map(tuple, link_data['target']))
                weight = link_data["weight"]
                bridge = link_data["bridge"]
                graph.add_edge(source, target, weight=weight, bridge=bridge)

            # Вывод информации о графе
            print("Узлы:", graph.nodes())
            print("Ребра:", graph.edges())

            # print("buildable_plan: {}".format(buildable_plan_graph))
            print("Файл успешно открыт и обработан.")
            return graph
        except:
            print("Не удалось открыть файл с графом")

        # Создаем пустой граф
        graph = nx.MultiGraph()
        labeldict = {}
        # Создание словаря с метками вершин
        labels = {}

        i = 1
        # Добавляем вершины в граф
        for island in islands_coords:
            graph.add_node(tuple(island))
            labels[tuple(island)] = 'I_{}'.format(i)
            i += 1

        # Формируем ребра графа
        for i in range(len(islands_coords)):
            for j in range(i + 1, len(islands_coords)):
                print('--------------------------------------------------------------------')
                island1_coordinates = tuple(islands_coords[i])
                island2_coordinates = tuple(islands_coords[j])
                island1_coordinates_buildable = IslandMap.get_buildable_coords_from_island(island_map, island1_coordinates)
                island2_coordinates_buildable = IslandMap.get_buildable_coords_from_island(island_map, island2_coordinates)
                print('island1_coordinates_buildable:', island1_coordinates_buildable)
                print('island2_coordinates_buildable:', island2_coordinates_buildable)

                for i1cb in island1_coordinates_buildable:
                    for i2cb in island2_coordinates_buildable:
                        if i1cb == i2cb:
                            continue

                        # Вычисляем длину моста как количество общих координат между островами
                        bridge_length = len(find_shortest_path(i1cb, i2cb, island_map, is_obstacles_reversed=True))

                        if bridge_length > max_bridge_length or bridge_length == 0:
                            continue

                        print('source: {}'.format(island1_coordinates))
                        print('target: {}'.format(island2_coordinates))
                        print('bridge_length: {}'.format(bridge_length))
                        bridge_options = {'start': i1cb, 'end': i2cb, 'expected_length': bridge_length}
                        graph.add_edge(island1_coordinates, island2_coordinates, weight=bridge_length,
                                       bridge=bridge_options)
                        print("added new edge!!")
                        # graph.add_edge()

                        # graph.add_edge(island1_coordinates, island2_coordinates, weight=2)

        G = graph
        # Создание остовного дерева
        T = nx.minimum_spanning_tree(G, algorithm='kruskal')

        T = nx.Graph(T)
        # Определение позиций вершин для отображения
        pos = nx.spring_layout(G)

        # Отображение вершин и ребер графа
        nx.draw_networkx(G, pos, with_labels=False, node_color='lightblue', node_size=500, alpha=0.8)
        nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5)

        # Отображение весов на ребрах остовного дерева
        edge_labels = nx.get_edge_attributes(T, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, label_pos=0.3, font_size=10)

        # Отображение остовного дерева
        nx.draw_networkx_edges(G, pos, edgelist=T.edges(), width=3.0, alpha=0.8, edge_color='r')
        nx.draw_networkx_labels(G, pos, labels=labels, font_size=12, font_color='black')

        # Отображение графика
        plt.axis('off')
        plt.show()

        # Преобразование мультиграфа в словарь
        graph_dict = nx.node_link_data(T)

        # Сохранение словаря в JSON-файл
        with open("buildable_plan_graph.json", "w") as f:
            json.dump(graph_dict, f)

        return T