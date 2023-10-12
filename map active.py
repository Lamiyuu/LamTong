import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import folium
import heapq
from matplotlib.pyplot import figure
#Xây dựng đồ thị dạng lưới, với mỗi ô vuông là 1 đỉnh
#Tọa đô các điểm đến và kết thúc( hình tròn và hình tam giác)
grid = [[0 for _ in range(10)] for _ in range(10)]
grid_size = 10
cell_size = 1
# Định nghĩa map ban đầu (0 đại diện cho ô trống, 1 đại diện cho chướng ngại vật)
grid = np.array([
[0,0,0,0,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0,0,0],
[0,0,0,0,1,1,0,0,0,0],
[0,0,0,0,0,1,0,0,0,0],
[0,0,0,0,0,1,0,0,0,0],
[0,0,0,0,0,1,0,0,0,0],
[0,0,0,1,0,1,1,0,0,0],
[0,0,0,1,0,0,0,0,0,0],
[0,0,0,1,0,0,0,0,0,0],
[0,0,0,0,0,0,0,0,0,0]
])
# Khởi tạo ma trận lưới và đặt tất cả các giá trị là 1.0
occupancy_grid = [[1.0 for _ in range(grid_size)] for _ in range(grid_size)]
#khởi tạo giá trị cho các ô
for i in range(10):
  for j in range(10):
      if grid[i][j]==1:
         occupancy_grid[i][j] = 0.8
# Đặt giá trị của ô [9][0] là 0.0 để biểu thị nó là một không gian không được chiếm

# Tạo hình tròn màu xanh trong ô [9][0]
fig, ax = plt.subplots()
ax.imshow(occupancy_grid, cmap='gray', origin='lower', extent=[0, grid_size*cell_size, 0, grid_size*cell_size])
ax.add_patch(patches.Circle((0.5, 9.5), radius=0.4, color='blue'))


# Tạo hình tam giác trong ô

# Tạo các đỉnh của tam giác
vertices = np.array([[8, 4], [7, 4], [7.5,5]])

# Tạo patch hình tam giác
triangle = patches.Polygon(vertices, closed=True, facecolor='blue')

# Thêm tam giác vào hình
ax.add_patch(triangle)

# Hiển thị bản đồ
plt.imshow(occupancy_grid, cmap='gray', origin='lower', extent=[0, grid_size*cell_size, 0, grid_size*cell_size])

# Tạo đường kẻ ngang
for i in range(1, grid_size):
    plt.axhline(i, color='black', linewidth=0.5)

# Tạo đường kẻ dọc
for i in range(1, grid_size):
    plt.axvline(i, color='black', linewidth=0.5)
# Điểm bắt đầu và kết thúc
start = (9,0)  # Hình tròn
goal = (4,7)    # Hình tam giác

from collections import deque
#Xây dựng đồ thị với mỗi ô vuông là 1 đỉnh 
#Thuật toán
def heuristic(a,b):
    return abs(b[0] - a[0]) + abs(b[1]-a[1])
def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

    close_set = set()

    came_from = {}

    gscore = {start:0}

    fscore = {start:heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
 

    while oheap:

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:

                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:                

                    if array[neighbor[0]][neighbor[1]] == 1:

                        continue

                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue
 

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                continue
 

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))
 

    return False

route = astar(grid, start, goal)

route = route + [start]

route = route[::-1]

print(route)


##############################################################################

# plot the path

##############################################################################

 

#extract x and y coordinates from route list

x_coords = []

y_coords = []

for i in (range(0,len(route))):

    x = route[i][0]+0.5

    y = route[i][1]+0.5

    x_coords.append(x)

    y_coords.append(y)

ax.plot(y_coords,x_coords, color = "red")

plt.show()



         

    
