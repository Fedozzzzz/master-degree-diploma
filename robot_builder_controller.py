from constants import MAX_BRIDGE_LENGTH
from robot_controller import RobotController
from utils import find_nearest_point, find_shortest_path, show_map



class RobotBuilderController(RobotController):
    def __init__(self, island_map, map_size, start_pos, buildable_coords, max_bridge_length=None, gui=None, performance_control=None):
        super().__init__(island_map, map_size, start_pos, gui, performance_control)
        self.buildable_coords = buildable_coords
        self.max_bridge_length = max_bridge_length or MAX_BRIDGE_LENGTH
        self.buildable_coords_reachable, self.buildable_coords_unreachable = \
            self.divide_buildable_coords_by_reachability(buildable_coords)
        self.planned_bridges = []

    def set_buildable_coords_reachable(self, buildable_coords):
        res = []
        for bc in buildable_coords:
            is_exists_path_to_point = len(self.find_path(bc)) > 0
            if is_exists_path_to_point:
                res.append(bc)
        return res

    def plan_bridges(self, target_point):
        island_start = self.map.get_island_by_coord(self.pos)
        island_end = self.map.get_island_by_coord(target_point)
        local_buildable_plan = self.map.get_local_buildable_plan(island_start, island_end)
        self.planned_bridges = local_buildable_plan
        print('[plan_bridges] Planned bridges (local_buildable_plan): {}'.format(self.planned_bridges))

    def divide_buildable_coords_by_reachability(self, buildable_coords):
        res_reachable = []
        res_unreachable = []
        for bc in buildable_coords:
            is_exists_path_to_point = len(self.find_path(bc)) > 0
            if is_exists_path_to_point:
                res_reachable.append(bc)
            else:
                res_unreachable.append(bc)
        return res_reachable, res_unreachable

    def build_bridge_graph_based(self, target_point):
        print('[build_bridge_graph_based]: Build bridge to target point: {}'.format(target_point))
        is_exists_path_to_point = len(self.find_path(target_point)) > 0
        print('[build_bridge_graph_based]: is_exists_path_to_point: {}'.format(is_exists_path_to_point))

        if is_exists_path_to_point:
            return

        self.plan_bridges(target_point)

        # is_plan_completed = False
        # while not is_plan_completed:

        for bridge in self.planned_bridges:
            building_start = bridge['start']
            building_end = bridge['end']

            path_to_start = self.find_path(building_start)
            path_to_end = self.find_path(building_end)

            is_exists_path_to_start = len(path_to_start) > 0
            is_exists_path_to_end = len(path_to_end) > 0

            if is_exists_path_to_start:
                self.follow_path(path_to_start)
            elif is_exists_path_to_end:
                self.follow_path(path_to_end)
                temp = building_start
                building_start = building_end
                building_end = temp

            if self.can_build_bridge():
                bridge_path = find_shortest_path(self.pos, building_end, self.map.island_map, is_obstacles_reversed=True)
                print('[build_bridge_graph_based]: Bridge length: {}'.format(len(bridge_path)))

                if len(bridge_path) > self.max_bridge_length or len(bridge_path) == 0:
                    print('[build_bridge_graph_based]: Bridge was not built because of MAX_BRIDGE_LENGTH')
                    continue

                self.map.build_bridge(bridge_path)

        self.planned_bridges = []

    def build_bridge_to_point_modified(self, point):
        print('[build_bridge_to_point_modified]: Build bridge to point: {}'.format(point))

        is_exists_path_to_point = len(self.find_path(point)) > 0

        print('[build_bridge_to_point_modified]: is_exists_path_to_point: {}'.format(is_exists_path_to_point))

        if is_exists_path_to_point:
            return
        else:
            remaining_buildable_coords = self.buildable_coords_reachable.copy()
            print('[build_bridge_to_point_modified]: remaining_buildable_coords reachable: {}'.format(remaining_buildable_coords))
            while not is_exists_path_to_point:
                can_build_bridge = self.can_build_bridge()
                print('[build_bridge_to_point_modified]: Can build bridge at point: {} is {}'.format(self.pos, can_build_bridge))
                print('[build_bridge_to_point_modified]: remaining_buildable_coords reachable: {}'.format(remaining_buildable_coords))

                if not can_build_bridge:
                    self.go_nearest_buildable_point_modified(remaining_buildable_coords, point)

                is_bridge_built = self.build_bridge_modified(point)
                print('[build_bridge_to_point_modified]: Was bridge built: {}'.format(is_bridge_built))

                if not is_bridge_built:
                    remaining_buildable_coords.remove(self.pos)
                    self.go_nearest_buildable_point_modified(remaining_buildable_coords, point)
                else:
                    self.buildable_coords_reachable, self.buildable_coords_unreachable =\
                        self.divide_buildable_coords_by_reachability(self.buildable_coords)

                    remaining_buildable_coords = self.buildable_coords_reachable.copy()
                    is_exists_path_to_point = len(self.find_path(point)) > 0

    def go_nearest_buildable_point_modified(self, remaining_buildable_coords, point_goal):
        print('[go_nearest_buildable_point_modified]: Buildable coords array:{}'.format(remaining_buildable_coords))
        print('[go_nearest_buildable_point_modified]: Goal point coords:{}'.format(point_goal))
        path = []
        buildable_coords_current = remaining_buildable_coords.copy()

        print('----------------------------------------------------------')
        while len(path) == 0:
            nearest_buildable_point = find_nearest_point(point_goal, buildable_coords_current)

            print('[go_nearest_buildable_point_modified]: Nearest Buildable point:{}'.format(nearest_buildable_point))
            is_robot_arrived = self.pos == nearest_buildable_point
            print('[go_nearest_buildable_point_modified]: is_robot_arrived: {}'.format(is_robot_arrived))
            print('[go_nearest_buildable_point_modified]: self robot pos: {}'.format(self.pos))

            if nearest_buildable_point is None or is_robot_arrived:
                break

            if nearest_buildable_point:
                path = self.find_path(nearest_buildable_point)
                print('[go_nearest_buildable_point_modified]: Path to the Nearest Buildable point:{}'.format(path))

                if path:
                    print('[go_nearest_buildable_point_modified]: Path to the Nearest Buildable point:{}'.format(path))

                    self.follow_path(path)
                    break
                else:
                    buildable_coords_current.remove(nearest_buildable_point)

    def build_bridge_modified(self, point_goal):
        print('[build_bridge_modified]: trying to build bridge')
        print('[build_bridge_modified]: Goal point coords:{}'.format(point_goal))
        x, y = self.pos
        can_build_bridge = self.map.island_map[x][y] == 0.5
        print('----------------------------------------------------------')
        print('[build_bridge_modified]: Robot self position: {}'.format(self.pos))
        print('[build_bridge_modified]: Map state at this position: {}'.format(self.map.island_map[x][y]))

        if can_build_bridge:
            print('[build_bridge_modified]: Building bridge at pos:{}'.format(self.pos))

            start_searching_buildable_point = True

            buildable_coords_real = self.buildable_coords_unreachable.copy()

            print('[build_bridge_modified]: buildable_coords_unreachable_real: {}'.format(buildable_coords_real))

            while start_searching_buildable_point:
                nearest_point = find_nearest_point(point_goal, buildable_coords_real)

                print('[build_bridge_modified]: buildable_coords_unreachable_real: {}'.format(buildable_coords_real))
                print('[build_bridge_modified]: Nearest point: {}'.format(nearest_point))

                if nearest_point is None:
                    print('[build_bridge_modified]: Nearest point is None')
                    return False

                bridge_path = find_shortest_path(self.pos, nearest_point, self.map.island_map,
                                                 is_obstacles_reversed=True)
                print('[build_bridge_modified]: Bridge length: {}'.format(len(bridge_path)))

                if len(bridge_path) > self.max_bridge_length or len(bridge_path) == 0:
                    print('[build_bridge_modified]: Bridge was not built because of MAX_BRIDGE_LENGTH: {}'.format(len(bridge_path) > self.max_bridge_length))
                    print('[build_bridge_modified]: Bridge was not built because of bridge LENGTH IS NULL: {}'.format(len(bridge_path) == 0))
                    buildable_coords_real.remove(nearest_point)
                    continue

                start_searching_buildable_point = False

                self.map.build_bridge(bridge_path)

                print('[build_bridge_modified]: Bridge was built!!')

                self.buildable_coords.remove(nearest_point)
                self.buildable_coords.remove(self.pos)

                if self.performance_control:
                    self.performance_control.add_steps(len(bridge_path))

            return True

    def build_bridge_to_point(self, point):
        print('Build bridge to point: {}'.format(point))

        is_exists_path_to_point = len(self.find_path(point)) > 0

        print('is_exists_path_to_point: {}'.format(is_exists_path_to_point))

        if is_exists_path_to_point:
            return
        else:
            remaining_buildable_coords = self.buildable_coords.copy()
            print('remaining_buildable_coords: {}'.format(remaining_buildable_coords))
            while not is_exists_path_to_point:
                can_build_bridge = self.can_build_bridge()
                print('Can build bridge at point: {} is {}'.format(point, can_build_bridge))
                print('remaining_buildable_coords: {}'.format(remaining_buildable_coords))

                if not can_build_bridge:
                    self.go_nearest_buildable_point(remaining_buildable_coords)

                is_bridge_built = self.build_bridge()
                print('Was bridge built: {}'.format(is_bridge_built))

                if not is_bridge_built:
                    remaining_buildable_coords.remove(self.pos)
                    self.go_nearest_buildable_point(remaining_buildable_coords)
                else:
                    is_exists_path_to_point = len(self.find_path(point)) > 0

    def is_building_over_land(self, point):
        path = self.find_path(point)
        if len(path) > 0:
            return True
        else:
            return self.map.check_path_over_land(path)

    def can_build_bridge(self):
        x, y = self.pos
        return self.map.island_map[x][y] == 0.5

    def go_nearest_buildable_point(self, remaining_buildable_coords):
        print('Buildable coords array:{}'.format(self.buildable_coords))
        path = []

        # buildable_coords_current = self.buildable_coords.copy()
        buildable_coords_current = remaining_buildable_coords.copy()

        print('----------------------------------------------------------')
        while len(path) == 0:
            nearest_buildable_point = find_nearest_point(self.pos, buildable_coords_current)

            print('Nearest Buildable point:{}'.format(nearest_buildable_point))
            # show_map(self.map.island_map, point=nearest_buildable_point)

            # show_map(self.map.island_map, point=self.pos)
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

            buildable_coords_real = self.buildable_coords.copy()

            print('buildable_coords_real: {}'.format(buildable_coords_real))

            while start_searching_buildable_point:
                nearest_point = find_nearest_point(self.pos, buildable_coords_real)

                # show_map(self.map, point=nearest_point)
                print('buildable_coords_real: {}'.format(buildable_coords_real))
                print('Nearest point: {}'.format(nearest_point))

                if nearest_point is None:
                    return False

                is_building_over_land = self.is_building_over_land(nearest_point)
                print('Is building over land: {}'.format(is_building_over_land))

                if is_building_over_land:
                    buildable_coords_real.remove(nearest_point)
                    continue

                if len(buildable_coords_real) == 1 and buildable_coords_real[0] == nearest_point:
                    return False

                bridge_path = find_shortest_path(self.pos, nearest_point, self.map.island_map,
                                                 is_obstacles_reversed=True)
                print('Bridge length: {}'.format(len(bridge_path)))

                if len(bridge_path) > self.max_bridge_length or len(bridge_path) == 0:
                    buildable_coords_real.remove(nearest_point)
                    continue

                start_searching_buildable_point = False

                self.map.build_bridge(bridge_path)

                print('Bridge builded!!')

                # print_map(self.map)

                self.buildable_coords.remove(nearest_point)
                self.buildable_coords.remove(self.pos)

                if self.performance_control:
                    self.performance_control.add_steps(len(bridge_path))
            return True


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
