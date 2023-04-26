import numpy as np
import matplotlib.pyplot as plt

# 随机生成n个矩形障碍物
n = 5
obs = []
for i in range(n):
    x = np.random.randint(0, 50)
    y = np.random.randint(0, 50)
    w = np.random.randint(1, 10)
    h = np.random.randint(1, 10)
    obs.append([x, y, w, h])

# 创建一个50x50的网格作为地图
grid = np.zeros((50, 50))

# 对于每一个网格，计算到最近障碍物的距离
obs_dist = np.zeros((50, 50))
for i in range(50):
    for j in range(50):
        min_dist = np.inf
        for ob in obs:
            x, y, w, h = ob
            dx = max(0, abs(i - x) - w / 2)
            dy = max(0, abs(j - y) - h / 2)
            dist = np.sqrt(dx ** 2 + dy ** 2)
            min_dist = min(min_dist, dist)
        obs_dist[i][j] = min_dist

# 根据距离计算每个网格的概率
prob = np.exp(-obs_dist)

# 归一化概率
prob /= np.sum(prob)

# 随机选择一个网格
idx = np.random.choice(50 * 50, p=prob.reshape(-1))
x = idx // 50
y = idx % 50

# 可视化地图和随机选择的点
plt.imshow(grid, cmap='gray')
for ob in obs:
    x, y, w, h = ob
    plt.gca().add_patch(plt.Rectangle((x - w / 2, y - h / 2), w, h, fill=True, color='black'))
plt.scatter(y, x, color='red')
plt.show()