import matplotlib.pyplot as plt


class IslandMapGUI:
    def __init__(self, island_map, robots_builder=None, robots_courier=None):
        self.island_map = island_map

        self.robots_builder = robots_builder
        self.robots_courier = robots_courier
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([0, island_map.width])
        self.ax.set_ylim([0, island_map.height])
        self.ax.set_aspect('equal')
        self.terrain_colors = {
            'robot_builder': (1, 0, 0),
            'robot_courier': (0, 0, 0.5),
            'departure': (0.6, 0.0, 0.6),
            'destination': (0, 0, 0)
        }

    def draw(self):
        self.ax.clear()

        self.draw_map()

        if self.robots_courier and self.robots_builder:
            self.draw_robots()

        self.draw_delivery_points()

        self.fig.show()

    def draw_robots(self):
        for rb in self.robots_builder:
            x, y = rb.pos
            self.ax.scatter(x, y, color=self.terrain_colors['robot_builder'])

        for rc in self.robots_courier:
            x, y = rc.pos
            print(rc.pos)
            self.ax.scatter(x, y, color=self.terrain_colors['robot_courier'])

    def draw_delivery_points(self):
        delivery_points = self.island_map.delivery_coords
        if not delivery_points:
            return
        for departure, destination in delivery_points:
            rect = plt.Rectangle(departure, 1, 1, linewidth=1, edgecolor='none',
                                 facecolor=self.terrain_colors['departure'])
            self.ax.add_patch(rect)
            rect = plt.Rectangle(destination, 1, 1, linewidth=1, edgecolor='none',
                                 facecolor=self.terrain_colors['destination'])
            self.ax.add_patch(rect)

    def draw_map(self):
        cmap = plt.cm.colors.ListedColormap(['blue', 'yellow', 'brown', 'green'])
        bounds = [0, 0.4, 0.6, 0.8, 1]
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
        self.ax.imshow(self.island_map.get_not_transposed_map_matrix(), cmap=cmap, norm=norm)
        self.fig.canvas.draw()
