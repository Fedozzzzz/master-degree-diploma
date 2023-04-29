# import pygame
# import noise
#
# # размеры карты
# MAP_WIDTH = 64
# MAP_HEIGHT = 64
#
# # параметры генерации шума
# OCTAVES = 6
# PERSISTENCE = 0.5
# LACUNARITY = 2.0
# SCALE = 50.0
#
# # инициализация Pygame
# pygame.init()
#
# # создание окна
# screen = pygame.display.set_mode((MAP_WIDTH, MAP_HEIGHT))
#
# # генерация шума
# world = []
# for i in range(MAP_HEIGHT):
#     row = []
#     for j in range(MAP_WIDTH):
#         value = noise.pnoise2(i/SCALE,
#                              j/SCALE,
#                              octaves=OCTAVES,
#                              persistence=PERSISTENCE,
#                              lacunarity=LACUNARITY,
#                              repeatx=MAP_WIDTH,
#                              repeaty=MAP_HEIGHT,
#                              base=0)
#         row.append(value)
#     world.append(row)
#
# # отображение карты
# for i in range(MAP_HEIGHT):
#     for j in range(MAP_WIDTH):
#         if world[i][j] < -0.2:
#             # океан
#             pygame.draw.rect(screen, (0, 0, 255), (j, i, 1, 1))
#         elif world[i][j] < 0:
#             # можно построить мост
#             pygame.draw.rect(screen, (128, 128, 128), (j, i, 1, 1))
#         else:
#             # остров
#             pygame.draw.rect(screen, (0, 255, 0), (j, i, 1, 1))
#
# # отображение окна
# pygame.display.flip()
#
# # ожидание закрытия окна
# done = False
# while not done:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             done = True
#
# # завершение работы Pygame
# pygame.quit()

# import noise
# from PIL import Image
#
# # Размер карты
# MAP_WIDTH = 64
# MAP_HEIGHT = 64
#
# # Масштабирование шума
# SCALE = 50.0
#
# # Генерируем шум
# world = []
# for i in range(MAP_WIDTH):
#     row = []
#     for j in range(MAP_HEIGHT):
#         # Получаем значение шума
#         noise_val = noise.pnoise2(i/SCALE, j/SCALE, octaves=6, persistence=0.5, lacunarity=2.0, repeatx=MAP_WIDTH, repeaty=MAP_HEIGHT, base=0)
#         # Определяем тип ландшафта в зависимости от значения шума
#         # if noise_val < -0.2:
#         #     row.append(0)  # Океан
#         # elif noise_val < 0.2:
#         #     row.append(0.5)  # Мост
#         # else:
#         #     row.append(1)  # Остров
#
#         row.append(noise_val)
#     world.append(row)
#
# # Создаем изображение
# im = Image.new("RGB", (MAP_WIDTH, MAP_HEIGHT), color=(255, 255, 255))
# pixels = im.load()
#
# # Задаем цвета для каждого типа ландшафта
# ocean_color = (0, 0, 128)
# island_color = (128, 128, 128)
# bridge_color = (255, 255, 255)
#
# # Проходимся по всей карте и задаем цвета пикселям
# for i in range(MAP_WIDTH):
#     for j in range(MAP_HEIGHT):
#         if world[i][j] == 0:
#             pixels[i, j] = ocean_color
#         elif world[i][j] == 0.5:
#             pixels[i, j] = bridge_color
#         else:
#             pixels[i, j] = island_color
#
# # Сохраняем карту в изображение
# im.save("island_map_64x64.png")
#

import numpy as np
import matplotlib.pyplot as plt
from noise import pnoise2

# Установка параметров для генерации карты
octaves = 6
freq = 16.0 * octaves
noise_scale = 1.0 / 128.0

# Генерация карты с помощью perlin noise
def generate_island_map(size, num_islands):
    island_map = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            x = i * noise_scale
            y = j * noise_scale
            noise_value = pnoise2(x * freq, y * freq, octaves)
            if noise_value < -0.1:
                island_map[i][j] = 0.0 # Океан
            else:
                island_map[i][j] = 1.0 # Остров
    # Добавление точек для строительства островов
    for i in range(num_islands):
        x = np.random.randint(size)
        y = np.random.randint(size)
        island_map[x][y] = 0.5
    return island_map

# Отображение карты с помощью matplotlib
def plot_map(island_map):
    fig, ax = plt.subplots()
    ax.imshow(island_map, cmap=plt.cm.Wistia, interpolation='nearest')
    plt.show()

# Генерация и отображение карты
island_map = generate_island_map(64, 8)
plot_map(island_map)
