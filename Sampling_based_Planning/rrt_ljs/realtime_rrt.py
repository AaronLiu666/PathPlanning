import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacles, max_dist=0.3, max_iter=100000):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # bottom left(x, y) and (width, height)
        self.max_dist = max_dist
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.num_nodes = 1
        self.iter = 0
        self.goal_rate= 0.01

    def add_node(self, q_new, q_near):
        q_new.parent = q_near
        self.nodes.append(q_new)
        self.num_nodes += 1

    def sample(self):
        if np.random.random() < self.goal_rate:
            return self.goal
        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)
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
        for i in range(self.max_iter):
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

    def plot_tree(self, path=None):
        fig, ax = plt.subplots()
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
        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 10])
        ax.set_aspect('equal')
        plt.title(f"Iteration: {self.iter}, Nodes: {self.num_nodes}")
        plt.show()


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


def basic_rrt(plot=False):
    time1 = time.perf_counter()
    start = (-8, -8)
    goal = (8, 8)
    # obstacles = [(-6, -7, 3,1), (-5, -3, 6, 1), (0, 2, 3, 2), (3, -4, 2, 6), (-2,-1,1,6), (-3, -5, 7, 0.1)]
    # obstacles = [(-7.9, -5, 5, 0.1)]
    # obstacles = [(-10,-3,5,1), (-6,-6,1,3), (-6,-10,1,2), (0,7,5,1),(4,4,1,3),(4,0,1,2),(0,0,5,1),(0,1,1,7)]
    # obstacles = [(-9, -3, 7, 1), (-3, -8, 1, 5), (4, 0, 1, 7),
    #             (8, 4, 1, 3), (5, 4, 3, 1), (-1, 0, 5, 1), (-4, -1, 1, 12)]
    obstacles = [(-7.5, -10, 1, 7), (-7.5, 3, 1, 7),
                 (6.5, 3, 1, 7), (6.5, -10, 1, 7), (-1, -2, 1, 7),]
    rrt = RRT(start, goal, obstacles, max_dist=0.8)
    path = rrt.find_path()
    if plot:
        print(f'Iteration: {rrt.iter}, Node Number: {rrt.num_nodes}')
        rrt.plot_tree(path=path)
    time2=time.perf_counter()
    
    
    return (rrt.iter, rrt.num_nodes,time2-time1)
if __name__ == '__main__':
    basic_rrt(True)

