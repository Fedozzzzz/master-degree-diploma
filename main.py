import random

import numpy as np
import noise

# Размеры карты
width = 128
height = 128

# # Масштаб и параметры шума
# scale = 100.0
# octaves = 5
# persistence = 4.0
# lacunarity = 1.8

# # Масштаб и параметры шума
# scale = 10.0  # Масштаб карты
# octaves = 6  # Количество октав
# persistence = 0.1  # Влияние каждой октавы
# lacunarity = 3.0   # Увеличение частоты шума с каждой октавой

scale = 20.0  # Масштаб карты
octaves = 8  # Количество октав
persistence = 0.6  # Влияние каждой октавы
lacunarity = 2.0  # Увеличение частоты шума с каждой октавой
seed = 42  # Семя для генерации случайных чисел

# Масштаб и параметры шума
# scale = 100.0
# octaves = 6
# persistence = 3.5
# lacunarity = 1.75

# Создаем массив данных для карты высот
heightmap = np.zeros((height, width))

# Заполняем массив данных шумом Perlin noise
for y in range(height):
    for x in range(width):
        value = noise.pnoise2(x/scale, y/scale, octaves=octaves, persistence=persistence, lacunarity=lacunarity, repeatx=1024, repeaty=1024, base=0)
        heightmap[y][x] = value


# import matplotlib.pyplot as plt
#
# plt.imshow(heightmap)
# plt.colorbar()
# plt.show()

def print_array(arr):
    """
    prints a 2-D numpy array in a nicer format
    """
    for a in arr:
        for elem in a:
            # print("{}".format(elem).rjust(3), end="")
            print(elem, end=" ")
        print(end="\n")


# Создаем массив данных для карты с островами
island_map = np.zeros((height, width))
islands_coords = []
buildable_coords = []

# Добавляем острова на карту высот
for y in range(height):
    for x in range(width):
        if heightmap[y][x] > 0.00005:
            island_map[y][x] = 1
            islands_coords.append((x, y))
        elif heightmap[y][x] > 0 and heightmap[y][x] < 0.000005:
            island_map[y][x] = 0.5
            islands_coords.append((x, y))
            buildable_coords.append((x, y))

# print_array(np.array(islands_coords))


import numpy as np
import matplotlib.pyplot as plt

# задаем цвета для каждого значения в карте
cmap = plt.cm.colors.ListedColormap(['blue', 'yellow', 'green'])
bounds = [0, 0.25, 0.75, 1]
norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

# создаем изображение
fig, ax = plt.subplots()
ax.imshow(island_map, cmap=cmap, norm=norm)

# # отображаем точки строительства мостов
# if bridges is not None:
#     for x, y in bridges:
#         ax.scatter(x, y, s=50, marker='o', facecolors='none', edgecolors='yellow')

plt.show()



robots_builders_num = 2

print('random.choice:')
# robots_builders_coord = random.sample(islands_coords, robots_builders_num)
robots_builders_coord = [(5, 47), (59, 80)]

print(robots_builders_coord)



res = []

for rb in robots_builders_coord:

    def dist(p1, p2=rb):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    res.append(min(buildable_coords, key=dist))

print(res)

# print(robots_builders_coord)

# for i in range(robots_builders_num):
#     while

# print_array(island_map)

from PIL import Image


image = Image.new("RGB", (width, height))

for y in range(height):
    for x in range(width):
        if island_map[y][x] > 0.5:
            image.putpixel((x, y), (45, 105, 53))
        elif island_map[y][x] == 0.5:
            image.putpixel((x, y), (255, 247, 0))
        else:
            image.putpixel((x, y), (14, 61, 201))

# for rb in robots_builders_coord:
#     image.putpixel(rb, (255, 0, 0))

#
# for r in res:
#     image.putpixel(r, (0, 0, 0))


# image.save("island_map.png", "PNG")
image.save("island_map_64x64.png", "PNG")
# image.show()


# for i in range(height):
#     for j in range(width):
#         print(island_map[i][j])
#     print('\n')


# Сохраняем карту с островами в np.array
np.save("island_map.npy", island_map)

# import noise
#
# width = 512
# height = 512
# scale = 100.0
#
# # Создаем новое изображение
# image = Image.new("RGB", (width, height))
#
# # Генерируем карту высот на основе Perlin noise
# for y in range(height):
#     for x in range(width):
#         value = noise.pnoise2(x/scale, y/scale, octaves=6, persistence=0.5, lacunarity=2.0, repeatx=1024, repeaty=1024, base=0)
#         image.putpixel((x, y), (int(value * 255), int(value * 255), int(value * 255)))
#
# # Добавляем острова на карту высот
# for y in range(height):
#     for x in range(width):
#         if image.getpixel((x, y))[0] > 128:
#             image.putpixel((x, y), (255, 255, 255))
#
# # Сохраняем изображение в файл
# image.save("island_map.png", "PNG")
