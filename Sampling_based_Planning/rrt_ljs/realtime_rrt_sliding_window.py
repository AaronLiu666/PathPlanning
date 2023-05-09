import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    # map(-x,x,-y,y)
    def __init__(self, start, goal, obstacles, max_dist=0.5, max_iter=500, map=[-10, 10, -10, 10]):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # bottom left(x, y) and (width, height)
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.num_nodes = 1
        self.iter = 0
        self.goal_rate = 0.01
        self.map = map

    def add_node(self, q_new, q_near):
        q_new.parent = q_near
        self.nodes.append(q_new)
        self.num_nodes += 1

    def sample(self):
        if np.random.random() < self.goal_rate:
            return self.goal
        x = np.random.uniform(self.map[0], self.map[1])
        y = np.random.uniform(self.map[2], self.map[3])
        return Node(x, y)

    def nearest(self, q):
        distances = [np.sqrt((q.x - n.x)**2 + (q.y - n.y)**2)
                     for n in self.nodes]
        min_idx = np.argmin(distances)
        return self.nodes[min_idx]

    def steer(self, q_rand, q_near):
        dist = np.sqrt((q_rand.x - q_near.x)**2 + (q_rand.y - q_near.y)**2)
        if dist > self.max_dist:
            x = q_near.x + self.max_dist*(q_rand.x - q_near.x)/dist
            y = q_near.y + self.max_dist*(q_rand.y - q_near.y)/dist
            return Node(x, y)
        else:
            return q_rand

    def collision_free(self, q1, q2):
        q1 = [q1.x, q1.y]
        q2 = [q2.x, q2.y]
        for o in self.obstacles:
            if is_collision(q1, q2, o):
                return False
        return True

    def find_path(self):
        # nodes = self.num_nodes
        for i in range(self.max_iter):
            # if i % 100 == 0:
            #     diff = self.num_nodes-nodes
            # print(diff)
            # if diff < 40:
            #     self.max_dist = self.max_dist*1.05
            # elif diff > 60:
            #     self.max_dist = self.max_dist*0.95
            # nodes = self.num_nodes
            q_rand = self.sample()
            # print(f"{q_rand.x}, {q_rand.y}")
            q_near = self.nearest(q_rand)
            q_new = self.steer(q_rand, q_near)
            if self.collision_free(q_near, q_new):
                self.add_node(q_new, q_near)
                # 终止条件：如果goal在steer的范围内直接结束
                if np.sqrt((q_new.x - self.goal.x)**2 + (q_new.y - self.goal.y)**2) < self.max_dist:
                    self.add_node(self.goal, q_new)
                    break
        self.iter = i
        path = [self.goal]
        node = self.nodes[-1]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(self.start)
        return list(reversed(path))

    def plot_tree(self, ax, path=None, map=[-10, 10, -10, 10]):

        for o in self.obstacles:
            rect = plt.Rectangle((o[0], o[1]), o[2], o[3], color='gray')
            ax.add_patch(rect)
        for n in self.nodes:
            if n.parent is not None:
                ax.plot([n.x, n.parent.x], [n.y, n.parent.y], 'k-', lw=2)
        if path is not None:
            ax.plot([n.x for n in path], [n.y for n in path], 'r-', lw=3)
        ax.plot(self.start.x, self.start.y, 'bo', markersize=8)
        ax.plot(self.goal.x, self.goal.y, 'go', markersize=8)
        # ax.set_xlim([map[0], map[1]])
        # ax.set_ylim([map[2], map[3]])
        # ax.set_aspect('equal')
        # plt.show()


def in_obs(x, y, obs):
    for o in obs:
        if (x > o[0] and x < o[0]+o[2] and y > o[1] and y < o[1]+o[3]):
            return True
            break
    return False


def rrt_append(path_list, node_list):
    path = [path_list[0][0]]
    nodes = [node_list[0][0]]
    for i in range(2, len(path_list)):
        path_list[i][0].parent = path_list[i-1][-1]
        node_list[i][0].parent = node_list[i-1][-1]
        path.extend(path_list[i][1:])
        nodes.extend(node_list[i][1:])
    return path, nodes


def is_collision(q1, q2, rect):
    x, y, w, h = rect

    # Check if the segment is completely to the left or right of the rectangle
    if max(q1[0], q2[0]) < x or min(q1[0], q2[0]) > x+w:
        return False

    # Check if the segment is completely above or below the rectangle
    if max(q1[1], q2[1]) < y or min(q1[1], q2[1]) > y+h:
        return False

    # Calculate the slope and y-intercept of the segment
    if q2[0] - q1[0] == 0:  # vertical segment
        m = None
        b = q1[0]
    else:
        m = (q2[1] - q1[1]) / (q2[0] - q1[0])
        b = q1[1] - m * q1[0]

    # Check if the segment intersects the top or bottom edges of the rectangle
    y_intersect_top = m*x + b if m is not None else q1[1]
    y_intersect_bottom = m*(x+w) + b if m is not None else q1[1]
    if (y_intersect_top >= y and y_intersect_top <= y+h) or \
            (y_intersect_bottom >= y and y_intersect_bottom <= y+h):
        return True

    # Check if the segment intersects the left or right edges of the rectangle
    x_intersect_left = (y - b) / m if m != 0 else q1[0]
    x_intersect_right = (y+h - b) / m if m != 0 else q1[0]
    if (x_intersect_left >= x and x_intersect_left <= x+w) or \
            (x_intersect_right >= x and x_intersect_right <= x+w):
        return True

    # If none of the above conditions were met, then the segment does not intersect the rectangle
    return False


def is_in(point, map):
    if point[0] > map[0] and point[0] < map[1] and point[1] > map[2] and point[1] < map[3]:
        return True
    else:
        return False


def generate_temp(start, goal, sub_w):
    hori = goal[0]-start[0]
    vert = goal[1]-start[1]
    right = 1 if (hori > 0) else -1
    up = 1 if (vert > 0) else -1
    temp1 = start[0]+sub_w*right
    temp2 = start[1]+sub_w*up

    temp3 = min(temp1, start[0])
    temp4 = max(temp1, start[0])
    temp5 = min(temp2, start[1])
    temp6 = max(temp2, start[1])
    
    # if abs(vert/hori)<0.25:
    #     temp3-=right*0.5*sub_w
    #     temp4-=right*0.5*sub_w
        
    # if abs(vert/hori)>0.75:
    #     temp5-=up*0.5*sub_w
    #     temp6-=up*0.5*sub_w

    xmin = max(-10, temp3)
    xmax = min(10, temp4)
    ymin = max(-10, temp5)
    ymax = min(10, temp6)

    sub_map = [xmin, xmax, ymin, ymax]
    
    temp_goal = [np.random.uniform(xmin,xmax),np.random.uniform(ymin,ymax)]

    return temp_goal, sub_map


def ours(obs, plot=False, straight=False):
    w = 20
    start = (-8, -8)
    goal = (8, 8)
    obstacles = obs
    # obstacles=[(0,1,2,3)]
    # obstacles = [(-7.5, -10, 1, 7), (-7.5, 3, 1, 7),
    #              (6.5, 3, 1, 7), (6.5, -10, 1, 7), (-1, -2, 1, 7),]

    # start = (-2, -2)
    # goal = (2, 2)
    # obstacles = [(0,-1,0.5,3)]
    path_list = []
    node_list = []
    temp_start = (-999, -999)
    temp_goal = [0, 0]
    ratio = 0.6
    iter = 0
    sub_w = w*ratio
    # fig, ax = plt.subplots()
    sub_map=[0,0,0,0]
    time1=time.perf_counter()
    while True:
        if goal == temp_start:
            break
        # decide temp start
        if temp_start == (-999, -999):
            temp_start = start
        # select submap and temp_goal
        
        if is_in(goal,sub_map):
            if temp_goal==goal:
                while True:        
                    temp_goal, sub_map=generate_temp(temp_start,goal, sub_w)
                    if not in_obs(*temp_goal, obstacles):
                        break
            else:
                temp_goal=goal
        else:       
            while True:        
                temp_goal, sub_map=generate_temp(temp_start,goal, sub_w)
                if not in_obs(*temp_goal, obstacles):
                    break
        # print(f"start:{temp_start},goal:{temp_goal}")
        # print(f"new goal:{temp_goal}")

        # find temp path
        temp_rrt = RRT(temp_start, temp_goal, obstacles, 0.5, 500, sub_map)
        if straight:
            if temp_rrt.collision_free(Node(*temp_start), Node(*temp_goal)):
                temp_path = [Node(*temp_start), Node(*temp_goal)]
            else:
                temp_path = temp_rrt.find_path()
        else:
            temp_path = temp_rrt.find_path()
        iter += temp_rrt.iter
        
        node_list.append(temp_rrt.nodes)
        path_list.append(temp_path)
        # update　更新子起点
        if temp_rrt.iter == temp_rrt.max_iter-1:
            # print('get another')
            # 放弃这个循环中的temp_start，回退一个submap
            if len(path_list)>1:
                previous = path_list[-2]
                temp_start = [previous[0].x, previous[0].y]
            else:
                temp_start = start    
            del  path_list[-2:]
            # print('back')
            continue
        # print('found')
        
        temp_start = temp_goal
        # print("plot")
        if plot:
            temp_rrt.plot_tree(ax, temp_path, sub_map)
        # print(f"{temp_rrt.iter}, {len(temp_rrt.nodes)}")

    # print('found')
    time2=time.perf_counter()
    tolen = 1
    tonode = 1
    for path, nodes in zip(path_list, node_list):
        tolen += len(path)-1
        tonode += len(nodes)
    # print(f"Length: {tolen}, Iteration: {iter}, Nodes: {tonode}, Number of submap: {len(path_list)}")

    # print(f"Number of submap: {len(path_list)}")

    # ax.set_aspect('equal')
    # ax.set_xlim(-10, 10)
    # ax.set_ylim(-10, 10)
    # plt.title(f"Iteration: {iter}, Nodes: {tonode}, Number of submap: {len(path_list)}")
    # plt.show()
    return (iter, tonode, time2-time1)


if __name__ == '__main__':
    fig, ax = plt.subplots()
    iter, tonode, _=ours([(-7.5, -10, 1, 7), (-7.5, 3, 1, 7), (6.5, 3, 1, 7), (6.5, -10, 1, 7), (-1, -2, 1, 7),], True,True)
    print(f"Iteration: {iter}, Nodes: {tonode}")

    # print(f"Number of submap: {len(path_list)}")

    ax.set_aspect('equal')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    plt.title(f"Iteration: {iter}, Nodes: {tonode}")
    plt.show()
