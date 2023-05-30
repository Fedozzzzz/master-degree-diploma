import math

from constants import DELIVERY_OPERATION_COST, COURIER_ROBOT
from robot_controller import RobotController
from utils import find_nearest_point, \
    get_departure_points, \
    find_destination_point


class RobotCourierController(RobotController):
    def __init__(self, map_, map_size, start_pos, delivery_coords, gui=None, performance_control=None):
        super().__init__(map_, map_size, start_pos, gui, performance_control, robot_type=COURIER_ROBOT)
        self.delivery_coords = delivery_coords

        self.is_in_delivery = False
        self.departure_point = None
        self.destination_point = None

        # self.analyze_map()

    def divide_coords_by_reachability(self, delivery_coords):
        res_reachable = []
        res_unreachable = []
        for dc in delivery_coords:
            is_exists_path_to_point = len(self.find_path(dc)) > 0
            if is_exists_path_to_point:
                res_reachable.append(dc)
            else:
                res_unreachable.append(dc)
        return res_reachable, res_unreachable

    def find_nearest_delivery_by_reachability(self, free_delivery_coords):
        departure_coords = get_departure_points(free_delivery_coords)
        departure_coords_reachable, departure_coords_unreachable, = self.divide_coords_by_reachability(
            departure_coords)

        # print('[find_nearest_delivery]: free_delivery_coords: {}'.format(free_delivery_coords))

        nearest_departure_point_reachable = find_nearest_point(self.pos, departure_coords_reachable)
        if nearest_departure_point_reachable:
            destination_point = find_destination_point(nearest_departure_point_reachable, free_delivery_coords)

            return (nearest_departure_point_reachable,
                    destination_point) if nearest_departure_point_reachable and destination_point else None

        nearest_departure_point_unreachable = find_nearest_point(self.pos, departure_coords_unreachable)

        # print('[find_nearest_delivery]: nearest_departure_point: {}'.format(nearest_departure_point))
        destination_point = find_destination_point(nearest_departure_point_unreachable, free_delivery_coords)

        # print('[find_nearest_delivery]: (nearest_departure_point, destination_point): {}'.format(
        #     (nearest_departure_point, destination_point)))
        return (
            nearest_departure_point_unreachable,
            destination_point) if nearest_departure_point_unreachable and destination_point else None

    def divide_robots_by_reachability(self, robots):
        res_reachable = []
        res_unreachable = []

        for robot in robots:
            is_reachable = False

            if len(self.find_path(robot.pos)) > 0:
                is_reachable = True

            if is_reachable:
                res_reachable.append(robot)
            else:
                res_unreachable.append(robot)

        return res_reachable, res_unreachable

    def find_nearest_robot_builder_by_reachability(self, robots_builder):
        print('[find_nearest_robot_builder_by_reachability] robots_builder:', robots_builder)

        builders_reachable, builders_unreachable, = self.divide_robots_by_reachability(robots_builder)

        print('[find_nearest_robot_builder_by_reachability] builders_reachable:', builders_reachable)
        print('[find_nearest_robot_builder_by_reachability] builders_unreachable:', builders_unreachable)

        nearest_builder_reachable = self.find_nearest_robot_builder(builders_reachable)

        if nearest_builder_reachable:
            return nearest_builder_reachable if nearest_builder_reachable else None

        nearest_builder_unreachable = self.find_nearest_robot_builder(builders_unreachable)

        return nearest_builder_unreachable if nearest_builder_unreachable else None

    def try_start_delivery(self, delivery_coords):
        self.departure_point, self.destination_point = delivery_coords

        can_pick_up_delivery = self.pos == self.departure_point

        if can_pick_up_delivery:
            self.start_delivery()
            return True
        else:
            # nearest_departure_point = self.find_nearest_departure_point()
            # self.departure_point = nearest_departure_point

            path = self.find_path(self.departure_point)
            is_exists_path_to_departure = len(path) > 0

            if is_exists_path_to_departure:
                self.follow_path(path)
                self.start_delivery()
                return True
                # self.set_destination_point()
            else:
                return False

    def try_finish_delivery(self):
        if not self.destination_point:
            self.set_destination_point()

        print('Destination point:{}'.format(self.destination_point))

        path = self.find_path(self.destination_point)
        print('Path to the Destination point:{}'.format(path))

        if path:
            self.follow_path(path)
            return self.finish_delivery()
        else:
            return False

    def set_destination_point(self):
        delivery_coords_current = self.map.delivery_coords.copy()
        self.destination_point = find_destination_point(self.departure_point, delivery_coords_current)

    def find_nearest_departure_point(self):
        departure_coords = get_departure_points(self.delivery_coords)
        nearest_departure_point = find_nearest_point(self.pos, departure_coords)
        return nearest_departure_point

    def find_nearest_robot_builder(self, robots_builder):
        nearest_builder = None
        min_distance = float('inf')
        for builder in robots_builder:
            builder_x, builder_y = builder.pos
            x, y = self.pos

            distance = math.sqrt((builder_x - x) ** 2 + (builder_y - y) ** 2)
            if distance < min_distance:
                nearest_builder = builder
                min_distance = distance
        return nearest_builder

    def find_nearest_delivery(self, free_delivery_coords):
        departure_coords = get_departure_points(free_delivery_coords)
        # print('[find_nearest_delivery]: free_delivery_coords: {}'.format(free_delivery_coords))
        nearest_departure_point = find_nearest_point(self.pos, departure_coords)

        # print('[find_nearest_delivery]: nearest_departure_point: {}'.format(nearest_departure_point))
        destination_point = find_destination_point(nearest_departure_point, free_delivery_coords)

        # print('[find_nearest_delivery]: (nearest_departure_point, destination_point): {}'.format(
        #     (nearest_departure_point, destination_point)))
        return (nearest_departure_point, destination_point) if nearest_departure_point and destination_point else None

    def start_delivery(self):
        can_pick_up_delivery = self.pos == self.departure_point

        if can_pick_up_delivery:
            self.is_in_delivery = True
            print('Picked up delivery at pos: {}'.format(self.pos))

            if self.performance_control:
                self.performance_control.add_steps(DELIVERY_OPERATION_COST)
                self.performance_control.increase_operation_cost_sum(DELIVERY_OPERATION_COST, self.robot_type)

    def finish_delivery(self):
        can_finish_delivery = self.pos == self.destination_point
        print('Can Finish delivery : {}'.format(can_finish_delivery))

        if can_finish_delivery:
            self.is_in_delivery = False
            print('Finished delivery at pos: {}'.format(self.pos))

            if self.performance_control:
                self.performance_control.add_steps(DELIVERY_OPERATION_COST)
                self.performance_control.increase_operation_cost_sum(DELIVERY_OPERATION_COST, self.robot_type)

            curr_delivery_point = (self.departure_point, self.destination_point)
            # print('curr_delivery_point: {}'.format(curr_delivery_point))
            # print('self.map.delivery_coords: {}'.format(self.map.delivery_coords))
            self.map.delivery_coords.remove(curr_delivery_point)
            # self.delivery_coords.remove(curr_delivery_point) # uncomment if native algorithm using
            self.departure_point = None
            self.destination_point = None
            return True
        return False

    def analyze_map(self):
        hard_break = False

        while len(self.delivery_coords) > 0 and not hard_break:
            delivery_coords_current = self.delivery_coords.copy()
            path = []

            if self.is_in_delivery:
                destination_point = find_destination_point(self.departure_point, delivery_coords_current)
                print('Destination point:{}'.format(destination_point))

                path = self.find_path(destination_point)
                # print('Path to the Destination point:{}'.format(path))

                if path:
                    self.destination_point = destination_point
                    self.follow_path(path)
                    self.finish_delivery()

            else:
                departure_coords = get_departure_points(delivery_coords_current)

                while len(path) == 0:
                    nearest_departure_point = find_nearest_point(self.pos, departure_coords)

                    if nearest_departure_point is None:
                        hard_break = True
                        break

                    print('Nearest departure point:{}'.format(nearest_departure_point))

                    if nearest_departure_point:
                        path = self.find_path(nearest_departure_point)
                        # print('Path to the Nearest Departure point:{}'.format(path))

                        if path:
                            # print('Path to the Nearest Departure point:{}'.format(path))
                            # show_map(self.map, path, self.pos)

                            self.departure_point = nearest_departure_point

                            self.follow_path(path)
                            self.start_delivery()
                        else:
                            departure_coords.remove(nearest_departure_point)
