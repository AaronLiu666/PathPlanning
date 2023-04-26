import numpy as np
import matplotlib.pyplot as plt

# 定义地图大小为10x10，障碍物在(0, 0)到(1, 2)的范围内
map_size = (10, 10)
obstacle_pos = ((0, 0), (1, 2))

# 定义势函数，如果点在障碍物内，则返回负无穷，否则返回距离的倒数
def potential(pos):
    if obstacle_pos[0][0] <= pos[0] <= obstacle_pos[1][0] and \
       obstacle_pos[0][1] <= pos[1] <= obstacle_pos[1][1]:
        return 0
    else:
        distance = np.linalg.norm(np.array(pos) - np.array(obstacle_pos[0]))
        # 避免除以0的情况
        if distance == 0:
            return 0
        else:
            return 1.0 / distance


if __name__ == '__main__':
    # 计算整个地图上每个点的势值
    potential_map = np.zeros(map_size)
    for i in range(map_size[0]):
        for j in range(map_size[1]):
            pos = (i, j)
            potential_map[i][j] = potential(pos)

    # 计算所有点的势值总和，作为概率密度函数分母
    total_potential = np.sum(potential_map)

    # 计算每个点的概率密度函数值
    pdf = np.zeros(map_size)
    for i in range(map_size[0]):
        for j in range(map_size[1]):
            pdf[i][j] = potential_map[i][j] / total_potential

    # 使用概率密度函数进行随机采样
    sampled_pos = np.random.choice(range(map_size[0]*map_size[1]), p=pdf.reshape(-1))
    sampled_pos = np.unravel_index(sampled_pos, map_size)
    print("Sampled Position:", sampled_pos)

    plt.imshow(potential_map,cmap = 'cool', origin = 'lower')
    plt.colorbar()
    plt.show()

    
    

