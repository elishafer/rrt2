import numpy as np
from math import sqrt
from math import pi
import random

debug = True

if debug:
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection


class ControlSpace(object):

    def __init__(self, obstacle_list, start, goal, xlimit, ylimit, vlimit):

        # Obtain the boundary limits.
        # Check if file exists.
        self.goal = goal
        self.start = start
        self.obstacle_list = obstacle_list
        self.xlimit = xlimit
        self.ylimit = ylimit
        self.psilimit = (-pi, pi)
        self.vlimit = vlimit
        self.map_bounds = [self.xlimit, self.ylimit]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

    def compute_distance(self, start_state, end_state):
        sum = 0
        for i, start_var in enumerate(start_state):
            sum += (start_var - end_state[i]) ** 2
        return sqrt(sum)

    def state_validity_checker(self, state):

        for (ox, oy, size) in self.obstacle_list:
            dx = ox - state[0]
            dy = oy - state[1]
            d = sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def edge_validity_checker(self, state1, state2):
        """
        circle and line intersection check
        https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
        :param state1:
        :param state2:
        :return:
        """
        d = state1 - state2
        for (ox, oy, r) in self.obstacle_list:
            f = state1 - np.array([ox, oy])
            a = 0
            b = 0
            c = 0
            for i in range(len(state2)):
                a += d[i] * d[i]
                b += f[i] * d[i]
                c += f[i] * f[i]
            b *= 2
            c -= r * r
            if b * b - 4 * a * c >= 0:
                return False
        return True

    def compute_heuristic(self, config):
        return self.compute_distance(config, self.goal)

    def sample(self, goal_sample_rate):
        # TODO
        pass
        if random.randint(0, 100) > goal_sample_rate:
            x_rand = [random.uniform(x[0], x[1]) for x in self.map_bounds]
        else:
            x_rand = self.goal
        x_rand = np.array(x_rand)
        return x_rand

    def visualize_plan(self, plan=None, visited=None, tree=None, title=None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        # TODO Plot tree
        # for node in self.nodes:
        #     if node.parent is not None:
        #         plt.plot([node.x[1], self.nodes[node.parent].x[1]], [
        #             node.x[0], self.nodes[node.parent].x[0]], "-g")

        # Plot obstacles
        circles = []
        fig = plt.gcf()
        ax = fig.gca()
        for (o_n, o_e, size) in self.obstacle_list:
            # plt.plot(ox, oy, "ok", ms=30 * size)
            circle = plt.Circle((o_e, o_n), size, fill=False)
            circles.append(circle)
        p = PatchCollection(circles)
        ax.add_collection(p)
        ax.set_aspect('equal')

        plt.plot(self.start[1], self.start[0], "xr")
        plt.plot(self.goal[1], self.goal[0], "xr")
        plt.axis([self.map_bounds[1][0], self.map_bounds[1][1],
                  self.map_bounds[0][0], self.map_bounds[0][1]])
        plt.grid(True)
