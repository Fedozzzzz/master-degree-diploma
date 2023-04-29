import math

from robot_controller import RobotController
from utils import find_nearest_point, \
    show_map, \
    get_departure_points, \
    find_destination_point


class RobotCourierController(RobotController):
    def __init__(self, map_, map_size, start_pos, delivery_coords):
        super().__init__(map_, map_size, start_pos)
        self.delivery_coords = delivery_coords

        self.is_in_delivery = False
        self.departure_point = None
        self.destination_point = None

        # self.analyze_map()

    def try_start_delivery(self):
        can_pick_up_delivery = self.pos == self.departure_point

        if can_pick_up_delivery:
            self.is_in_delivery = True
            print('Picked up delivery at pos: {}'.format(self.pos))
        else:
            nearest_departure_point = self.find_nearest_departure_point()
            self.departure_point = nearest_departure_point

            path = self.find_path(nearest_departure_point)
            is_exists_path_to_departure = len(path) > 0

            if is_exists_path_to_departure:
                self.follow_path(path)
                self.start_delivery()
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
            self.finish_delivery()
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
            print('builder pos: {}'.format(builder.pos))

            distance = math.sqrt((builder_x - x) ** 2 + (builder_y - y) ** 2)
            if distance < min_distance:
                nearest_builder = builder
                min_distance = distance
        return nearest_builder

    def start_delivery(self):
        can_pick_up_delivery = self.pos == self.departure_point

        if can_pick_up_delivery:
            self.is_in_delivery = True
            print('Picked up delivery at pos: {}'.format(self.pos))

    def finish_delivery(self):
        can_finish_delivery = self.pos == self.destination_point
        print('Can Finish delivery : {}'.format(can_finish_delivery))

        if can_finish_delivery:
            self.is_in_delivery = False
            print('Finished delivery at pos: {}'.format(self.pos))

            curr_delivery_point = (self.departure_point, self.destination_point)
            self.map.delivery_coords.remove(curr_delivery_point)
            # self.delivery_coords.remove(curr_delivery_point) # uncomment if native algorithm using
            self.departure_point = None
            self.destination_point = None

    def analyze_map(self):
        hard_break = False

        while len(self.delivery_coords) > 0 and not hard_break:
            delivery_coords_current = self.delivery_coords.copy()
            path = []

            if self.is_in_delivery:
                destination_point = find_destination_point(self.departure_point, delivery_coords_current)
                print('Destination point:{}'.format(destination_point))

                path = self.find_path(destination_point)
                print('Path to the Destination point:{}'.format(path))

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
                            print('Path to the Nearest Departure point:{}'.format(path))
                            # show_map(self.map, path, self.pos)

                            self.departure_point = nearest_departure_point

                            self.follow_path(path)
                            self.start_delivery()
                        else:
                            departure_coords.remove(nearest_departure_point)