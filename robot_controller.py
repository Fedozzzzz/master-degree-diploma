from utils import find_shortest_path


class RobotController:
    def __init__(self, initial_map, map_size, start_pos, gui=None, performance_control=None, robot_type=None):
        # self.map = list(map(list, zip(*initial_map)))  # необходимо транспорировать матрицу
        self.map = initial_map  # необходимо транспорировать матрицу
        self.map_size = map_size  # размер карты (x,y)
        self.pos = start_pos  # текущая позиция робота на карте (x,y)
        self.is_busy = False
        self.gui = gui
        self.performance_control = performance_control
        self.robot_type = robot_type

    def set_map_gui(self, gui):
        self.gui = gui

    def get_position(self):
        return self.pos

    def move_left(self, distance=1):
        x, y = self.pos
        # print(f"Moving left by {distance} units")
        self.pos = (x - distance, y)

    def move_right(self, distance=1):
        x, y = self.pos
        # print(f"Moving right by {distance} units")
        self.pos = (x + distance, y)

    def move_up(self, distance=1):
        x, y = self.pos
        # print(f"Moving up by {distance} units")
        self.pos = (x, y + distance)

    def move_down(self, distance=1):
        x, y = self.pos
        # print(f"Moving down by {distance} units")
        self.pos = (x, y - distance)

    def follow_path(self, path):
        for x, y in path:
            if x > self.pos[0]:
                self.move_right(x - self.pos[0])
            elif x < self.pos[0]:
                self.move_left(self.pos[0] - x)
            if y > self.pos[1]:
                self.move_up(y - self.pos[1])
            elif y < self.pos[1]:
                self.move_down(self.pos[1] - y)

            if self.performance_control:
                self.performance_control.add_steps(1)
                self.performance_control.increase_operation_cost_sum(1, self.robot_type)

        if self.gui:
            self.gui.draw()

    def find_path(self, point):
        return find_shortest_path(self.pos, point, self.map.island_map)
