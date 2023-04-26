import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

class RRTVisualizer:
    def __init__(self, start, goal, obstacle_list):
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list

    def draw_map(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim([0, 100])
        ax.set_ylim([0, 100])
        ax.set_aspect('equal', adjustable='box')

        start_circle = Circle((self.start[0], self.start[1]), radius=2.0, fc='g')
        goal_circle = Circle((self.goal[0], self.goal[1]), radius=2.0, fc='r')
        ax.add_patch(start_circle)
        ax.add_patch(goal_circle)

        for obs in self.obstacle_list:
            circle = Circle((obs[0], obs[1]), radius=obs[2], fc='k')
            ax.add_patch(circle)

        return fig, ax

    def plot_path(self, node_list):
        fig, ax = self.draw_map()

        x = [node.x for node in node_list]
        y = [node.y for node in node_list]

        ax.plot(x, y, 'b')

        plt.show()
