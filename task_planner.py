# self.tasks = pd.DataFrame(columns=['task_type', 'task_location', 'robot_id'])
#
#
# def add_delivery_task(self, start_loc, end_loc):
#     # добавление задачи доставки
#     self.tasks = self.tasks.append({'task_type': 'delivery', 'task_location': end_loc}, ignore_index=True)
#     # поиск ближайшего свободного робота-курьера и добавление задачи на его исполнение
#     free_courier = self.find_closest_free_robot(start_loc, 'courier')
#     if free_courier is not None:
#         self.tasks.loc[len(self.tasks) - 1, 'robot_id'] = free_courier
#     else:
#         print('No free courier available.')
#
#
# def add_bridge_task(self, start_loc, end_loc):
#     # добавление задачи на строительство моста
#     self.tasks = self.tasks.append({'task_type': 'bridge', 'task_location': end_loc}, ignore_index=True)
#     # поиск свободного робота-строителя и добавление задачи на его исполнение
#     free_builder = self.find_free_robot('builder')
#     if free_builder is not None:
#         self.tasks.loc[len(self.tasks) - 1, 'robot_id'] = free_builder
#     else:
#         print('No free builder available.')
#
#
# def find_closest_free_robot(self, loc, robot_type):
#     # поиск ближайшего свободного робота заданного типа
#     min_distance = float('inf')
#     closest_robot = None
#     for robot in self.robots:
#         if robot.robot_type == robot_type and robot.busy == False:
#             distance = self.map.get_distance(robot.current_location, loc)
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_robot = robot.robot_id
#     if closest_robot is not None:
#         self.robots[closest_robot].busy = True
#     return closest_robot
#
#
# def find_free_robot(self, robot_type):
#     # поиск свободного робота заданного типа
#     for robot in self.robots:
#         if robot.robot_type == robot_type and robot.busy == False:
#             robot.busy = True
#             return robot.robot_id
#     return None
#
#
# def execute_tasks(self):
#     # выполнение всех задач в порядке добавления
#     for i, row in self.tasks.iterrows():
#         task_type = row['task_type']
#         task_location = row['task_location']
#         robot_id = row['robot_id']
#         if task_type == 'delivery':
#             if robot_id is None:
#                 print('No available courier to execute delivery task.')
#                 continue
#             self.robots[robot_id].move_to(task_location)
#             self.robots[robot_id].unload()
#         elif task_type == 'bridge':
#             if robot_id is None:
#                 print('No available builder to execute bridge task.')
#                 continue
#             self.robots[robot_id].build_bridge(task_location)
#         self.tasks = self.tasks.drop(index=i)

from task import Task
from utils import DELIVERY_TASK_TYPE, BUILD_TASK_TYPE, find_nearest_point


class TaskPlanner:
    def __init__(self, map_, robots_courier, robots_builder, gui=None):
        self.map = map_
        self.robots_courier = robots_courier
        self.robots_builder = robots_builder
        self.gui = gui
        # self.delivery_points = delivery_points
        self.tasks = []

    def execute_tasks(self):
        for task in self.tasks:
            print('--------------------------------------------')
            print('Task in execution: {}'.format(task.type))
            print('--------------------------------------------')
            task.execute()
            print('--------------------------------------------')
            print('Task finished: {}'.format(task.type))
            print('--------------------------------------------')

        print('--------------------------------------------')
        print('Tasks queue is empty')
        print('--------------------------------------------')

    def plan_tasks(self):
        for rb in self.robots_builder:
            new_task = Task(BUILD_TASK_TYPE, rb)
            self.tasks.append(new_task)

        for rc in self.robots_courier:
            new_task = Task(DELIVERY_TASK_TYPE, rc)
            self.tasks.append(new_task)

        print('--------------------------------------------')
        print('Task planned: {}'.format(self.tasks))
        print('--------------------------------------------')

    def get_free_robots_builder(self):
        res = []
        for rb in self.robots_builder:
            if not rb.is_busy:
                res.append(rb)
        return res

    def get_free_robots_courier(self):
        res = []
        for rc in self.robots_courier:
            if not rc.is_busy:
                res.append(rc)
        return res

    def plan_task(self, task):
        self.tasks.append(task)
        print('Task planned: {}'.format(task.type))
        # print('Tasks queue: {}'.format(self.tasks))
        self.print_task_queue()

    def remove_task(self, task):
        self.tasks.remove(task)
        print('Task removed: {}'.format(task.type))
        self.print_task_queue()

    def print_task_queue(self):
        print('----------------------------------------------------------')
        print('Tasks queue: [')
        for task in self.tasks:
            print('Tasks type: {}'.format(task.type))
            print('Tasks robot: {},'.format(task.robot))
        print(']')
        print('----------------------------------------------------------')

    def plan_and_execute_tasks(self):
        is_global_goal_achieved = len(self.map.delivery_coords) == 0
        print('Is global goal achieved: {}'.format(is_global_goal_achieved))

        while not is_global_goal_achieved:
            print('Is global goal achieved: {}'.format(is_global_goal_achieved))
            free_robots_courier = self.get_free_robots_courier()

            for rc in free_robots_courier:
                new_task = Task(DELIVERY_TASK_TYPE, rc)
                self.plan_task(new_task)

            for task in self.tasks:
                is_task_executed = task.try_execute_task()
                print('Is Task executed: {}'.format(is_task_executed))

                if not is_task_executed and task.type == DELIVERY_TASK_TYPE:
                    free_robots_builder = self.get_free_robots_builder()
                    nearest_robot_builder = task.robot.find_nearest_robot_builder(free_robots_builder)
                    task_options = {'nearest_courier_robot': task.robot}
                    new_task = Task(BUILD_TASK_TYPE, nearest_robot_builder, options=task_options)

                    self.plan_task(new_task)
                else:
                    if self.gui:
                        self.gui.draw()
                    self.remove_task(task)

            is_global_goal_achieved = len(self.map.delivery_coords) == 0

        print('Is global goal achieved: {}'.format(is_global_goal_achieved))






