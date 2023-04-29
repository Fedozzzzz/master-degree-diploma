from robot_controller import RobotController
from utils import get_buildable_coords, find_nearest_point, find_shortest_path, update_map, show_map, print_map

MAX_BRIDGE_LENGTH = 12


class RobotBuilderController(RobotController):
    def __init__(self, island_map, map_size, start_pos, buildable_coords):
        super().__init__(island_map, map_size, start_pos)
        self.buildable_coords = buildable_coords

        # self.analyze_map()

    def build_bridge_to_point(self, point):
        print('Build bridge to point: {}'.format(point))

        is_exists_path_to_point = len(self.find_path(point)) > 0

        print('is_exists_path_to_point: {}'.format(is_exists_path_to_point))

        if is_exists_path_to_point:
            return
        else:
            while not is_exists_path_to_point:
                can_build_bridge = self.can_build_bridge()
                print('Can build bridge at point: {} is {}'.format(point, can_build_bridge))

                if not can_build_bridge:
                    self.go_nearest_buildable_point()

                self.build_bridge()
                is_exists_path_to_point = len(self.find_path(point)) > 0

    def can_build_bridge(self):
        x, y = self.pos
        return self.map.island_map[x][y] == 0.5

    def go_nearest_buildable_point(self):
        print('Buildable coords array:{}'.format(self.buildable_coords))
        path = []

        buildable_coords_current = self.buildable_coords.copy()

        print('----------------------------------------------------------')
        while len(path) == 0:
            nearest_buildable_point = find_nearest_point(self.pos, buildable_coords_current)

            print('Nearest Buildable point:{}'.format(nearest_buildable_point))
            # show_map(self.map.island_map, point=nearest_buildable_point)
            show_map(self.map.island_map, point=self.pos)
            is_robot_arrived = self.pos == nearest_buildable_point
            print('is_robot_arrived: {}'.format(is_robot_arrived))
            print('self robot pos: {}'.format(self.pos))

            if nearest_buildable_point is None or is_robot_arrived:
                break

            if nearest_buildable_point:
                path = self.find_path(nearest_buildable_point)
                print('Path to the Nearest Buildable point:{}'.format(path))

                if path:
                    print('Path to the Nearest Buildable point:{}'.format(path))
                    # show_map(self.map, path, self.pos)

                    self.follow_path(path)
                    break
                else:
                    buildable_coords_current.remove(nearest_buildable_point)


    def build_bridge(self):
        x, y = self.pos
        can_build_bridge = self.map.island_map[x][y] == 0.5
        print('----------------------------------------------------------')

        print('Robot self position: {}'.format(self.pos))
        print('Map state at this position: {}'.format(self.map.island_map[x][y]))
        # # print('Map state at this position: {}'.format(self.map[y][x]))
        # print('Can build bridge at pos: {} is '.format(self.pos, can_build_bridge))

        if can_build_bridge:
            print('Building bridge at pos:{}'.format(self.pos))

            start_searching_buildable_point = True

            buildable_coords_real = self.buildable_coords

            print('buildable_coords_real: {}'.format(buildable_coords_real))

            while start_searching_buildable_point:
                nearest_point = find_nearest_point(self.pos, buildable_coords_real)

                # show_map(self.map, point=nearest_point)
                print('Nearest point: {}'.format(nearest_point))

                if nearest_point is None:
                    break

                bridge_path = find_shortest_path(self.pos, nearest_point, self.map.island_map, is_all_walkable=True)
                print('Bridge length: {}'.format(len(bridge_path)))

                if (len(bridge_path) > MAX_BRIDGE_LENGTH):
                    buildable_coords_real.remove(nearest_point)
                    continue

                start_searching_buildable_point = False

                self.map.update_map(bridge_path)

                print('Bridge builded!!')

                # print_map(self.map)

                self.buildable_coords.remove(nearest_point)
                self.buildable_coords.remove(self.pos)
                # show_map(self.map, robot=self.pos)

            # self.follow_path(bridge_path)

    def analyze_map(self):
        hard_break = False
        while len(self.buildable_coords) > 0 and not hard_break:
            print('Buildable coords array:{}'.format(self.buildable_coords))
            path = []

            buildable_coords_current = self.buildable_coords.copy()

            print('----------------------------------------------------------')
            while len(path) == 0:
                nearest_buildable_point = find_nearest_point(self.pos, buildable_coords_current)

                print('Nearest Buildable point:{}'.format(nearest_buildable_point))
                # show_map(self.map, point=nearest_buildable_point)
                if nearest_buildable_point is None:
                    hard_break = True
                    break

                if nearest_buildable_point:
                    path = self.find_path(nearest_buildable_point)

                    if path:
                        print('Path to the Nearest Buildable point:{}'.format(path))
                        # show_map(self.map, path, self.pos)

                        self.follow_path(path)
                        self.build_bridge()
                    else:
                        buildable_coords_current.remove(nearest_buildable_point)

            print('----------------------------------------------------------')
        show_map(self.map.island_map, robot_builder=self.pos)
