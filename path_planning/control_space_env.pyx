# cython: profile=True
# cython: language_level=3, boundscheck=False
import numpy as np
from math import sqrt
import random
from copy import deepcopy

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import cython


class ControlSpace(object):

    def __init__(self, obstacle_list, start, goal,
                 state_limits, input_limits=None):

        # Obtain the boundary limits.
        # Check if file exists.
        self.goal = goal
        self.start = start
        self.obstacle_list = obstacle_list
        self.input_limits = input_limits
        self.state_limits = state_limits
        self.map_bounds = [self.state_limits[0], self.state_limits[1]]
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits')

    @cython.profile(False)
    def compute_distance(self, start_state, end_state, w, squared=False):
        sum = 0
        for i, start_var in enumerate(start_state):
            if w[i] is not None:
                if w[i] == 0:
                    continue
                else:
                    sum += w[i] * (start_var - end_state[i]) ** 2
            else:
                sum += (start_var - end_state[i]) ** 2
        if squared:
            return sum
        else:
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
        xystate1 = state1[:2]
        xystate2 = state2[:2]
        d = xystate2 - xystate1
        for (ox, oy, r) in self.obstacle_list:
            f = xystate1 - np.array([ox, oy])
            a = 0
            b = 0
            c = 0
            for i in range(len(xystate2)):
                a += d[i] * d[i]
                b += f[i] * d[i]
                c += f[i] * f[i]
            b *= 2
            c -= r * r
            discriminant = b * b - 4 * a * c
            if discriminant >= 0:
                if 0 < (-b - sqrt(discriminant)) / (2 * a) <= 1:
                    return False
        return True

    def sample(self, goal_sample_rate):
        """
        Sample the control space.
        In the case of the AUV for LARS we only care if the AUV reaches the goal region.
        i.e. we don't care about the speed and orientation.
        :param goal_sample_rate:
        :return:
        """
        if random.randint(0, 100) > goal_sample_rate:
            x_rand = [random.uniform(*x) for x in self.state_limits]
        else:
            x_rand = list(self.goal)
            for i, var in enumerate(x_rand):
                if var is None:
                    x_rand[i] = random.uniform(*self.state_limits[i])
        x_rand = np.array(x_rand)
        return x_rand

    def sample_control(self):
        return [random.uniform(*limit) for limit in self.input_limits]

    def goal_radius_reached(self, state, r=5):
        goal_p = deepcopy(self.goal)
        goal_p = [state[i] if goal_var is None else goal_var for i, goal_var in enumerate(goal_p)]
        return self.compute_distance(goal_p, state, w=(None, None, 0, 0, 0, 0)) < r

    def visualize_plan(self, plan=None, visited=None, tree=None, title=None, rnd=None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        # TODO Plot tree
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[1], rnd[0], "^k")
        if tree is not None:
            nodes = tree.vertices
            for k, v in enumerate(tree.edges):
                plt.plot([nodes[k][1], nodes[v][1]], [
                    nodes[k][0], nodes[v][0]], "-g")

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

        if plan is not None:
            for i in range(np.shape(plan)[0] - 1):
                x = [plan[i, 0], plan[i + 1, 0]]
                y = [plan[i, 1], plan[i + 1, 1]]
                plt.plot(y, x, 'r')
            # u_e = np.cos(plan[:, 2]) * plan[:, 3] - np.sin(plan[:, 2]) * plan[:, 4]
            # v_e = np.sin(plan[:, 2]) * plan[:, 3] + np.cos(plan[:, 2]) * plan[:, 4]
            # plt.quiver(plan[:,:2], u_e, v_e)
