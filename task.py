from constants import DELIVERY_TASK_TYPE, BUILD_TASK_TYPE
from utils import find_destination_point


class Task:
    def __init__(self, task_type, robot, priority=None, options=None):
        if options is None:
            options = {}
        self.type = task_type
        self.robot = robot
        self.priority = priority
        self.options = options

    def try_execute_task(self):
        self.robot.is_busy = True
        if self.type == DELIVERY_TASK_TYPE:
            return self.execute_delivery_task()
        elif self.type == BUILD_TASK_TYPE:
            # return self.execute_building_task() TODO this line should work
            self.execute_building_task()
        self.robot.is_busy = False
        return True

    def execute_delivery_task(self):
        delivery_coords = self.options['delivery_coords']
        if not self.robot.is_in_delivery:
            is_delivery_started = self.robot.try_start_delivery(delivery_coords)
            # self.robot.is_busy = is_delivery_started
            print('(starting) Robot task type: {}, is_busy: {}'.format(self.type, self.robot.is_busy))
            return False
        else:
            is_delivery_finished = self.robot.try_finish_delivery()
            self.robot.is_busy = not is_delivery_finished
            print('(finishing) Robot task type: {}, is_busy: {}'.format(self.type, self.robot.is_busy))
            return is_delivery_finished

    def execute_building_task(self):
        print('Task options: {}'.format(self.options))

        nearest_courier_robot = self.options['nearest_courier_robot']
        print('nearest courier robot: {}'.format(nearest_courier_robot))
        print('nearest courier robot departure_point: {}'.format(nearest_courier_robot.departure_point))
        print('nearest courier robot destination_point: {}'.format(nearest_courier_robot.destination_point))

        nearest_courier_robot_coords = nearest_courier_robot.pos
        nearest_courier_robot_departure_point = nearest_courier_robot.departure_point
        nearest_courier_robot_destination_point = nearest_courier_robot.destination_point

        is_exists_path_to_courier_robot = None if nearest_courier_robot_coords is None \
            else len(self.robot.find_path(nearest_courier_robot_coords)) > 0

        is_exists_path_to_departure_point = None if nearest_courier_robot_departure_point is None \
            else len(self.robot.find_path(nearest_courier_robot_departure_point)) > 0

        is_exists_path_to_destination_point = None if nearest_courier_robot_destination_point is None \
            else len(self.robot.find_path(nearest_courier_robot_destination_point)) > 0

        print('is_exists_path_to_courier_robot: {}'.format(is_exists_path_to_courier_robot))
        print('is_exists_path_to_departure_point: {}'.format(is_exists_path_to_departure_point))
        print('is_exists_path_to_destination_point: {}'.format(is_exists_path_to_destination_point))

        if not is_exists_path_to_courier_robot and is_exists_path_to_departure_point is not None:
            print('Build bridge to point (nearest_courier_robot_coords):{}'.format(nearest_courier_robot_coords))

            if self.options['algorithm'] == 'graph':
                self.robot.build_bridge_graph_based(nearest_courier_robot_coords)

            else:
                # self.robot.build_bridge_to_point(nearest_courier_robot_coords)
                self.robot.build_bridge_to_point_modified(nearest_courier_robot_coords)
        elif not is_exists_path_to_departure_point and is_exists_path_to_departure_point is not None:
            print('Build bridge to point (nearest_courier_robot_departure_point):{}'.format(
                nearest_courier_robot_departure_point))

            if self.options['algorithm'] == 'graph':
                self.robot.build_bridge_graph_based(nearest_courier_robot_departure_point)
            else:
                # self.robot.build_bridge_to_point(nearest_courier_robot_departure_point)
                self.robot.build_bridge_to_point_modified(nearest_courier_robot_departure_point)
        elif not is_exists_path_to_destination_point and is_exists_path_to_destination_point is not None:
            print('Build bridge to point (is_exists_path_to_delivery_point):{}'.format(
                nearest_courier_robot_destination_point))

            if self.options['algorithm'] == 'graph':
                self.robot.build_bridge_graph_based(nearest_courier_robot_destination_point)
            else:
                # self.robot.build_bridge_to_point(nearest_courier_robot_destination_point)
                self.robot.build_bridge_to_point_modified(nearest_courier_robot_destination_point)

    def execute(self):
        self.robot.is_busy = True

        if self.type == DELIVERY_TASK_TYPE:
            self.robot.analyze_map()
        elif self.type == BUILD_TASK_TYPE:
            self.robot.analyze_map()

        self.robot.is_busy = False
