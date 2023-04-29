# import tkinter as tk
#
#
# class RobotGUI:
#     def __init__(self, controller):
#         self.controller = controller
#         self.map = list(map(list, zip(*self.controller.map)))
#
#         # Создаем окно графического интерфейса
#         self.root = tk.Tk()
#         self.root.title("Robot Control")
#
#         # Создаем виджеты для ввода координат и кнопку "Переместить"
#         self.x_entry = tk.Entry(self.root, width=5)
#         self.y_entry = tk.Entry(self.root, width=5)
#         self.move_button = tk.Button(self.root, text="Переместить", command=self.move_robot)
#
#         # Размещаем виджеты на окне
#         self.x_entry.pack(side="left")
#         tk.Label(self.root, text="X:").pack(side="left")
#         self.y_entry.pack(side="left")
#         tk.Label(self.root, text="Y:").pack(side="left")
#         self.move_button.pack(side="left")
#
#         width, height = self.controller.map_size
#
#         # Создаем холст для отображения карты
#         self.canvas = tk.Canvas(self.root, width=width * 10, height=height * 10)
#         self.canvas.pack()
#
#         self.draw_map()
#
#         # Отображаем текущее положение робота
#         x, y = self.controller.pos
#         x = (x + 0.5) * 10
#         y = (y + 0.5) * 10
#         self.robot = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red")
#
#         # Запускаем главный цикл обработки событий
#         self.root.mainloop()
#
#     def draw_map(self):
#         for i in range(len(self.map)):
#             for j in range(len(self.map[0])):
#                 if self.map[i][j] == 1:
#                     self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="green")
#                 elif self.map[i][j] == 0.5:
#                     self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="yellow")
#                 else:
#                     self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="blue")
#
#     def move_robot(self):
#         # Получаем координаты из текстовых полей
#         x = int(self.x_entry.get()) // 10
#         y = int(self.y_entry.get()) // 10
#         point = (x, y)
#
#         print(point)
#
#         # Вычисляем путь до указанной точки
#         path = self.controller.find_path(point)
#
#         # Перемещаем робота по пути
#         self.controller.follow_path(path)
#
#         # Обновляем отображение положения робота на холсте
#         x, y = self.controller.pos
#
#         x *= 10
#         y *= 10
#         self.canvas.coords(self.robot, x - 5, y - 5, x + 5, y + 5)

import tkinter as tk
import time

from utils import find_shortest_path


class RobotGUI:
    def __init__(self, island_map, robots):
        self.island_map = island_map
        self.robots = robots
        self.map = list(map(list, zip(*self.island_map.island_map)))

        # Создаем окно графического интерфейса
        self.root = tk.Tk()
        self.root.title("Robot Control")

        # Создаем виджеты для ввода координат и кнопку "Переместить"
        self.x_entry = tk.Entry(self.root, width=5)
        self.y_entry = tk.Entry(self.root, width=5)
        # self.move_button = tk.Button(self.root, text="Переместить", command=self.move_robots)
        self.move_button = tk.Button(self.root, text="Переместить", command=self.move_robot_to_point)

        # Размещаем виджеты на окне
        self.x_entry.pack(side="left")
        tk.Label(self.root, text="X:").pack(side="left")
        self.y_entry.pack(side="left")
        tk.Label(self.root, text="Y:").pack(side="left")
        self.move_button.pack(side="left")

        width, height = self.island_map.map_size

        # Создаем холст для отображения карты
        self.canvas = tk.Canvas(self.root, width=width * 10, height=height * 10)
        self.canvas.pack()

        self.draw_map()

        # Отображаем текущее положение роботов
        self.robot_objs = []
        for i, robot in enumerate(self.robots):
            x, y = robot.pos
            x = (x + 0.5) * 10
            y = (y + 0.5) * 10
            color = ['red', 'blue', 'green', 'yellow'][i % 4]
            robot_obj = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color)
            self.robot_objs.append(robot_obj)

        # Запускаем главный цикл обработки событий
        self.root.mainloop()

    def draw_map(self):
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                if self.map[i][j] == 1:
                    self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="green")
                elif self.map[i][j] == 0.5:
                    self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="yellow")
                elif self.map[i][j] == 0.7:
                    self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="brown")
                else:
                    self.canvas.create_rectangle(j * 10, i * 10, j * 10 + 10, i * 10 + 10, fill="blue")

    def draw_robots(self):
        for robot in self.robots:
            x, y = robot.pos
            x = (x + 0.5) * 10
            y = (y + 0.5) * 10
            robot.shape = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red")

    def move_robot(self, robot, path):
        for x, y in path:
            x *= 10
            y *= 10
            dx = x - robot.pos[0] * 10
            dy = y - robot.pos[1] * 10
            self.canvas.move(robot.shape, dx, dy)
            robot.pos = (x // 10, y // 10)
            self.root.update()
            time.sleep(0.1)

    def update_robots(self, robots):
        for i, robot in enumerate(robots):
            print('update robot pos')
            x, y = robot.pos
            x = (x + 0.5) * 10
            y = (y + 0.5) * 10
            print('robot pos: {}, {}'.format(x, y))
            self.canvas.coords(self.robots[i], x - 5, y - 5, x + 5, y + 5)

        self.root.update()

    # def draw_robots(self):
    #     # Для каждого робота из массива robots создаем круг на карте
    #     self.robot_ovals = []
    #     for robot in self.robots:
    #         x, y = robot.pos
    #         x = (x + 0.5) * 10
    #         y = (y + 0.5) * 10
    #         robot_oval = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red")
    #         self.robot_ovals.append(robot_oval)
    #
    def move_robots(self):
        # Получаем координаты из текстовых полей
        x = int(self.x_entry.get()) // 10
        y = int(self.y_entry.get()) // 10
        point = (x, y)

        for robot in self.robots:
            # Вычисляем путь до указанной точки
            path = robot.find_path(point)

            # Перемещаем роботов по пути
            robot.follow_path(path)
            # # Обновляем отображение положения робота на холсте
            # x, y = robot.pos
            # x *= 10
            # y *= 10
            # self.canvas.coords(robot, x - 5, y - 5, x + 5, y + 5)

        # Обновляем отображение положения всех роботов на холсте
        self.update_robots(self.robots)

    def move_robot_to_point(self, robot_index=0):
        x = int(self.x_entry.get()) // 10
        y = int(self.y_entry.get()) // 10
        point = (x, y)

        robot = self.robots[robot_index]
        path = find_shortest_path(robot.pos, point, self.map, is_all_walkable=True)
        for position in path:
            robot.follow_path([position])
            self.draw_robots()
            time.sleep(0.5)
