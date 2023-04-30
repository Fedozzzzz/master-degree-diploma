class IslandMap:
    def __init__(self, map_array, delivery_coords=None, robots_builder=None, robots_courier=None):
        self.width = len(map_array[0])
        self.height = len(map_array)
        self.map_size = (self.width, self.height)
        self.island_map = list(map(list, zip(*[row[:] for row in map_array])))
        self.buildable_coords = self.get_buildable_coords(self.island_map, self.width, self.height)
        self.delivery_coords = delivery_coords

    def is_obstacle(self, x, y):
        return self.island_map[y][x] == 1

    def is_within_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    @staticmethod
    def get_buildable_coords(island_map, width, height):
        res = []
        for x in range(width):
            for y in range(height):
                if island_map[x][y] == 0.5:
                    res.append((y, x))
        return res

    def get_not_transposed_map_matrix(self):
        return list(map(list, zip(*[row[:] for row in self.island_map])))

    def build_bridge(self, updates):
        for up in updates:
            x, y = up
            self.island_map[x][y] = 0.7
