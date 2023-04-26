import numpy as np
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    # map(-x,x,-y,y)
    def __init__(self, start, goal, obstacles, max_dist=0.2, max_iter=10000, map=[-10, 10, -10, 10]):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # bottom left(x, y) and (width, height)
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.num_nodes = 1
        self.iter = 0
        self.goal_rate = 0.05
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

def in_obs(x,y,obs):
    for o in obs:
        if (x>o[0] and x<o[0]+o[2] and y>o[1] and y<o[1]+o[3]):
            return True
            break
    return False

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
    if point[0]>map[0] and point[0]<map[1] and point[1]>map[2] and point[1]<map[3]:
        return True
    else:
        return False


if __name__ == '__main__':
    w = 18
    map=[-0.5*w, 0.5*w, -0.5*w, 0.5*w]
    start = (-7, -7)
    # goal = (7, 3)
    goal = (7, 7)
    # obstacles = [(-6, -7, 3,1), (-5, -3, 6, 1), (0, 2, 3, 2), (3, -4, 2, 6), (-2,-1,2,6), (-3, -5, 8, 0.5)]
    obstacles = [(-9, -3, 7, 1), (-3, -8, 1, 5), (4, 0, 1, 7),
                 (8, 4, 1, 3), (5, 4, 3, 1), (-1, 0, 5, 1), (-4, -1, 1, 12)]
    # obstacles = [(-7.9, -5, 5, 0.1)]
    path_list=[]
    temp_start = (-999, -999)
    temp_goal = [0,0]
    ratio = 0.5
    sub_w = w*ratio
    fig, ax = plt.subplots()
    while True:
        if goal == temp_start:
                break
        # decide temp start
        if temp_start == (-999, -999):
            temp_start = start
        left_flag = (temp_start[0]-goal[0])/abs(temp_start[0]-goal[0])
        up_flag = (temp_start[1]-goal[1])/abs(temp_start[1]-goal[1])

        # select submap
        sub_map = [temp_start[0]-0.5 * sub_w, temp_start[0]+0.5 * sub_w, temp_start[1]-0.5*sub_w, temp_start[1]+0.5 * sub_w]

        # decide temp  goal
        end_flag=False
        while True:
            
            if is_in(goal, sub_map):
                temp_goal = goal
                end_flag = True
            else:
                x = np.random.uniform(min(temp_start[0], temp_start[0]-left_flag*0.5*sub_w),max(temp_start[0], temp_start[0]-left_flag*0.5*sub_w))
                y = np.random.uniform(min(temp_start[1], temp_start[1]-up_flag*0.5*sub_w),max(temp_start[1], temp_start[1]-up_flag*0.5*sub_w))
                temp_goal=[x,y]
            if not in_obs(*temp_goal, obstacles):break
        
        print(f"new goal:{temp_goal}")
        
        # find temp path
        rrt = RRT(temp_start, temp_goal, obstacles, 0.2, 10000, sub_map)
        temp_path = rrt.find_path()
        path_list.append(temp_path)
        temp_start = temp_goal
        rrt.plot_tree(ax, temp_path, sub_map)
    ax.set_aspect('equal')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    plt.show()
        
    print(len(path_list))
    # for pat in path:
    #     print("Length of sublist:", len(pat))
    #     rrt.plot_tree(path=pat)
    # print(f'Iteration: {rrt.iter}, Node Number: {rrt.num_nodes}')
    # rrt = RRT(start, goal, obstacles, 0.2, 10000, map, ax)
    # path=[Node(*start)]
    # for pat in path_list:
    #     path.extend(pat[1:])
        
    # rrt.plot_tree(path=path, map=map)