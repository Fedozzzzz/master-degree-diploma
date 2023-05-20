import matplotlib.pyplot as plt
import networkx as nx
import math

import networkx as nx
import numpy as np

from utils import find_shortest_path


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
                if i > 0 and (island_map[i-1][j] == 1 or island_map[i-1][j] == 0.5):
                    graph.add_edge((i, j), (i-1, j))
                if j > 0 and (island_map[i][j-1] == 1 or island_map[i][j-1] == 0.5):
                    graph.add_edge((i, j), (i, j-1))
    # nx.draw(graph, with_labels=False)
    # plt.show()
    # Используем disjoint-set для разбиения графа на компоненты связности
    islands = list(nx.connected_components(graph))

    return islands


island_map = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]

# island_map = [
#     [1, 0.5, 0, 0, 0.5],
#     [1, 0, 0, 1, 0],
#     [0, 0, 0.5, 1, 1],
#     [0, 1, 1, 0, 0],
#     [0.5, 0, 1, 1, 1]
# ]

# Чтение массива из файла
# island_map = np.loadtxt('array.txt', delimiter=',')

island_map = list(map(list, zip(*[row[:] for row in island_map])))

islands_coords = divide_into_islands(island_map)

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
    graph.nodes[tuple(island)]['bridges'] = []
    i += 1
# # Добавляем вершины в граф
# for i in range(len(islands_coords)):
#     graph.add_node(i)

def get_buildable_coords(island_map, coords):
    res = []
    for coord in coords:
        x, y = coord
        if island_map[x][y] == 0.5:
            res.append((x, y))
    return res

MAX_BRIDGE_LENGTH = 18
# Формируем ребра графа
for i in range(len(islands_coords)):
    for j in range(i + 1, len(islands_coords)):
        print('--------------------------------------------------------------------')
        island1_coordinates = tuple(islands_coords[i])
        island2_coordinates = tuple(islands_coords[j])
        island1_coordinates_buildable = get_buildable_coords(island_map, island1_coordinates)
        island2_coordinates_buildable = get_buildable_coords(island_map, island2_coordinates)
        print('island1_coordinates_buildable:', island1_coordinates_buildable)
        print('island2_coordinates_buildable:', island2_coordinates_buildable)

        for i1cb in island1_coordinates_buildable:
            for i2cb in island2_coordinates_buildable:
                if i1cb == i2cb:
                    continue
                # print("i1cb: {}".format(i1cb))
                # print("i2cb: {}".format(i2cb))
                # print("path: {}".format(find_shortest_path(i1cb, i2cb, island_map, is_obstacles_reversed=True)))

                # Вычисляем длину моста как количество общих координат между островами
                bridge_length = len(find_shortest_path(i1cb, i2cb, island_map, is_obstacles_reversed=True))

                if bridge_length > MAX_BRIDGE_LENGTH or bridge_length == 0:
                    continue

                print('bridge_length: {}'.format(bridge_length))
                bridge_options = {'start': i1cb, 'end': i2cb, 'expected_length': bridge_length}
                graph.add_edge(island1_coordinates, island2_coordinates, weight=bridge_length, bridge=bridge_options)
                print("added new edge!!")

                # bridge1_options = {'start': i1cb, 'end': i2cb, 'expected_length': bridge_length}
                # bridge2_options = {'start': i2cb, 'end': i1cb, 'expected_length': bridge_length}
                #
                # graph.nodes[island1_coordinates]['bridges'].append(bridge1_options)
                # graph.nodes[island2_coordinates]['bridges'].append(bridge2_options)
                # graph.add_edge()

                # graph.add_edge(island1_coordinates, island2_coordinates, weight=2)

G=graph
pos = nx.spring_layout(G)
nx.draw_networkx_nodes(G, pos, node_color = 'r', node_size = 100, alpha = 1)
nx.draw_networkx_labels(G, pos, labels=labels, font_size=12, font_color='black')
ax = plt.gca()
for e in G.edges:
    ax.annotate("",
                xy=pos[e[0]], xycoords='data',
                xytext=pos[e[1]], textcoords='data',
                arrowprops=dict(arrowstyle="-", color="0.5",
                                shrinkA=5, shrinkB=5,
                                patchA=None, patchB=None,
                                connectionstyle="arc3,rad=rrr".replace('rrr',str(0.3*e[2])
                                ),
                                ),
                )
    edge_weight = G.edges[e[0], e[1], e[2]]['weight']
    x = (pos[e[0]][0] + pos[e[1]][0]) / 2  # x-координата для размещения метки
    y = (pos[e[0]][1] + pos[e[1]][1]) / 2  # y-координата для размещения метки
    plt.text(x, y, edge_weight, fontsize=8, ha='center', va='center')

plt.axis('off')
plt.show()

G = nx.Graph(G)
# Определение позиций вершин для отображения
pos = nx.spring_layout(G)

# Отображение вершин и ребер графа
nx.draw_networkx(G, pos, with_labels=False, node_color='lightblue', node_size=500, alpha=0.8)
nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5)

# Отображение весов на ребрах остовного дерева
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, label_pos=0.3, font_size=10)

# Отображение остовного дерева
nx.draw_networkx_edges(G, pos, edgelist=G.edges(), width=3.0, alpha=0.8, edge_color='r')
nx.draw_networkx_labels(G, pos, labels=labels, font_size=12, font_color='black')

# Отображение графика
plt.axis('off')
plt.show()

for n in G.nodes:
    coords = n
    print("Node: {}".format(n))
    print("Node type: {}".format(type(n)))
    print("Node type: {}".format(list(n)))


node_list = list(G.nodes)

first_node = node_list[0]
third_node = node_list[2]

shortest_path_astar = nx.astar_path(G, first_node, third_node, weight='weight')
print("[get_local_buildable_plan] Shortest path (A*):", shortest_path_astar)

# for n in shortest_path_astar:
#     print('node data', G.nodes[n]['bridges'])

G = graph
# Создание остовного дерева
T = nx.minimum_spanning_tree(G, algorithm='kruskal')

T = nx.Graph(T)
# Итерация по вершинам остовного дерева и получение дополнительных атрибутов
# for n, attributes in T.nodes(data=True):
#     print("----------------------------------------------")
#     print("Minimum spanning tree node: {}".format(n))
#     for attr_key, attr_value in attributes.items():
#         print("{}: {}".format(attr_key, attr_value))

# Получаем кратчайший путь в виде списка вершин
path_nodes = shortest_path_astar

# Восстанавливаем ребра на основе списка вершин
path_edges = [(path_nodes[i], path_nodes[i+1]) for i in range(len(path_nodes)-1)]

# Выводим вершины и ребра кратчайшего пути
for u, v in path_edges:
    print("Vertex:", u)
    print("Edge:", G[u][v])

#
# for u, v, data in T.edges(data=True):
#     attributes = data.keys()
#     print("Edge ({}, {}), attributes: {}".format(u, v, attributes))
#
#     for attr_key in attributes:
#         attr_value = data[attr_key]
#         print("{}: {}".format(attr_key, attr_value))


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


# G=T
# pos = nx.spring_layout(G)
# nx.draw_networkx_nodes(G, pos, node_color = 'r', node_size = 100, alpha = 1)
# nx.draw_networkx_labels(G, pos, labels=labels, font_size=12, font_color='black')
# ax = plt.gca()
# for e in G.edges:
#     ax.annotate("",
#                 xy=pos[e[0]], xycoords='data',
#                 xytext=pos[e[1]], textcoords='data',
#                 arrowprops=dict(arrowstyle="-", color="0.5",
#                                 shrinkA=5, shrinkB=5,
#                                 patchA=None, patchB=None,
#                                 connectionstyle="arc3,rad=rrr".replace('rrr',str(0.3*e[2])
#                                 ),
#                                 ),
#                 )
#     edge_weight = G.edges[e[0], e[1], e[2]]['weight']
#     x = (pos[e[0]][0] + pos[e[1]][0]) / 2  # x-координата для размещения метки
#     y = (pos[e[0]][1] + pos[e[1]][1]) / 2  # y-координата для размещения метки
#     plt.text(x, y, edge_weight, fontsize=8, ha='center', va='center')
#
# plt.axis('off')
# plt.show()
