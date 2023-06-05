from constants import COURIER_ROBOT, BUILDER_ROBOT


class PerformanceControl:
    def __init__(self):
        self.steps_count = 0
        self.courier_robot_operations_cost_sum = 0
        self.builder_robot_operations_cost_sum = 0
        self.bridge_num = 0
        self.bridge_total_length = 0

    def increase_bridge_num(self):
        self.bridge_num += 1

    def increase_bridge_total_length(self, bridge_length):
        self.bridge_total_length += bridge_length

    def increase_operation_cost_sum(self, cost, robot_type):
        if robot_type == COURIER_ROBOT:
            self.courier_robot_operations_cost_sum += cost
        elif robot_type == BUILDER_ROBOT:
            self.builder_robot_operations_cost_sum += cost

    def add_steps(self, count):
        self.steps_count += count

    def get_steps_count(self):
        return self.steps_count
