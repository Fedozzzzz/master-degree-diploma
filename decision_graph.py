# import networkx
# import heapq
# import math
#
#
# class DecisionGraph:
#     def __init__(self, map_data, start, goals, courier_speed, builder_speed):
#         self.map_data = map_data
#         self.start = start
#         self.goals = goals
#         self.courier_speed = courier_speed
#         self.builder_speed = builder_speed
#         self.graph = self.build_graph()
#
#     def build_graph(self):
#         graph = networkx.Graph()
#
#         for y in range(len(self.map_data)):
#             for x in range(len(self.map_data[y])):
#                 if self.map_data[y][x] != '#':
#                     for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
#                         nx, ny = x + dx, y + dy
#                         if 0 <= nx < len(self.map_data[y]) and 0 <= ny < len(self.map_data) and self.map_data[ny][nx] != '#':
#                             cost = 1
#                             if self.map_data[y][x] == '~' or self.map_data[ny][nx] == '~':
#                                 cost = math.inf
#                             graph.add_edge((x, y), (nx, ny), weight=cost)
#         return graph
#
#     def plan_tasks(self):
#         tasks = []
#         for goal in self.goals:
#             courier_path = networkx.astar_path(self.graph, self.start, goal, heuristic=self.courier_heuristic)
#             courier_distance = self.calculate_path_distance(courier_path)
#             builder_start = courier_path[-1]
#             builder_goals = self.find_reachable_goals(builder_start)
#             if not builder_goals:
#                 continue
#             best_builder_goal = min(builder_goals, key=lambda goal: self.builder_heuristic(builder_start, goal))
#             builder_path = networkx.dijkstra_path(self.graph, builder_start, best_builder_goal)
#             builder_distance = self.calculate_path_distance(builder_path)
#             total_distance = courier_distance + builder_distance
#             tasks.append((total_distance, courier_path, builder_path))
#         tasks.sort()
#         return tasks
#
#     def find_reachable_goals(self, start):
#         goals = []
#         for goal in self.goals:
#             if networkx.has_path(self.graph, start, goal):
#                 goals.append(goal)
#         return goals
#
#     def courier_heuristic(self, node, goal):
#         return self.calculate_distance(node, goal) / self.courier_speed
#
#     def builder_heuristic(self, node, goal):
#         return self.calculate_distance(node, goal) / self.builder_speed
#
#     def calculate_distance(self, node1, node2):
#         return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)
#
#     def calculate_path_distance(self, path):
#         distance = 0
#         for i in range(len(path) - 1):
#             distance += self.graph[path[i]][path[i + 1]]['weight']
#         return distance


import networkx as nx
import numpy as np


class DecisionGraph:
    def __init__(self, map, courier_pos, builder_pos, deliveries):
        self.map = map
        self.courier_pos = courier_pos
        self.builder_pos = builder_pos
        self.deliveries = deliveries
        self.G = nx.DiGraph()

    def build_graph(self):
        # Add starting nodes for courier and builder
        self.G.add_node(('courier_start', self.courier_pos))
        self.G.add_node(('builder_start', self.builder_pos))

        # Add delivery nodes
        for i, delivery in enumerate(self.deliveries):
            self.G.add_node(('delivery', i), delivery=delivery)

        # Add edge between starting nodes and delivery nodes
        for i, delivery in enumerate(self.deliveries):
            courier_dist = nx.astar_path_length(self.map, self.courier_pos, delivery)
            builder_dist = nx.astar_path_length(self.map, self.builder_pos, delivery)
            total_time = max(courier_dist, builder_dist)
            self.G.add_edge(('courier_start', self.courier_pos), ('delivery', i), weight=courier_dist)
            self.G.add_edge(('builder_start', self.builder_pos), ('delivery', i), weight=builder_dist)

            # Add edge between delivery nodes based on heuristic
            for j in range(i + 1, len(self.deliveries)):
                delivery1 = self.G.nodes[('delivery', i)]['delivery']
                delivery2 = self.G.nodes[('delivery', j)]['delivery']
                dist = np.linalg.norm(np.array(delivery1) - np.array(delivery2))
                heuristic = dist
                self.G.add_edge(('delivery', i), ('delivery', j), weight=heuristic)

    def solve(self):
        # Find shortest path through decision graph using Dijkstra's algorithm
        path = nx.dijkstra_path(self.G, ('courier_start', self.courier_pos), ('builder_start', self.builder_pos),
                                weight='weight')

        # Extract deliveries from path
        deliveries = []
        for node in path:
            if node[0] == 'delivery':
                deliveries.append(self.G.nodes[node]['delivery'])

        return deliveries
