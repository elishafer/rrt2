#!/usr/bin/env python

__author__ = "Elisei Shafer"

import random
import numpy as np
import matplotlib.pylab as plt
from matplotlib.collections import PatchCollection
import math
import json
import copy
import pickle

show_animation = True
DYNAMIC_STEERING = True


class Rrt:

    def __init__(self, start, goal, c_space_bounds, obstacle_list, max_iterations=1000,
                 max_extend=2.0, goal_sample_rate=5):
        self.start = start
        self.goal = goal
        self.max_iterations = max_iterations
        self.nodes = [Node(start)]
        self.c_space_bounds = c_space_bounds
        self.obstacle_list = obstacle_list
        self.goal_sample_rate = goal_sample_rate
        self.max_extend = max_extend

    def sample_free(self, c_space_bounds):
        if random.randint(0, 100) > self.goal_sample_rate:
            x_rand = [random.uniform(x[0],x[1]) for x in c_space_bounds]
        else:
            x_rand = self.goal
        x_rand = np.array(x_rand)
        return x_rand

    def find_nearest(self, x_rand, nodes):
        angle_weight = 10.0
        dlist = []
        for node in nodes:
            weighted_vector = x_rand - node.x
            weighted_vector[2]%=math.pi*2
            weighted_vector[2] *= angle_weight
            dlist.append(np.linalg.norm(weighted_vector))
        minind = dlist.index(min(dlist))
        x_nearest = nodes[minind].x
        return x_nearest, minind

    def steer(self, x_nearest, x_rand, max_extend):
        vector_rand_near = x_rand - x_nearest
        extend_length = np.linalg.norm(vector_rand_near)
        if extend_length > max_extend:
            x_new = (vector_rand_near * max_extend) / extend_length + x_nearest
        else:
            x_new = x_rand

        if DYNAMIC_STEERING:
            u = 0.5
            delta_t = 8
            x_new = copy.deepcopy(x_nearest)
            if len(x_new) == 3:
                x_new[2] += random.uniform(-math.pi/5, math.pi/5)
                # x_new[2] %= 2*math.pi
            x_new[0] += u * math.cos(x_nearest[2] + x_new[2] / 2) * delta_t
            x_new[1] += u * math.sin(x_nearest[2] + x_new[2] / 2) * delta_t
            x_new[2] %= 2 * math.pi
            x_new = np.array(x_new)

        return x_new

    def is_obstacle_free(self, x_nearest, x_new, obstacle_list):

        for (ox, oy, size) in obstacle_list:
            dx = ox - x_new[0]
            dy = oy - x_new[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def algo(self,animation=True):

        for i in range(self.max_iterations):
            x_rand = self.sample_free(self.c_space_bounds)
            x_nearest, parent = self.find_nearest(x_rand, self.nodes)
            x_new = self.steer(x_nearest, x_rand, self.max_extend)

            if self.is_obstacle_free(x_nearest,x_new, self.obstacle_list):
                new_node = Node(x_new)
                new_node.parent = parent
                self.nodes.append(new_node)

            if np.linalg.norm(self.goal-x_new) <= self.max_extend:
                print('goal reached!')
                break

            if animation and i%30:
                self.draw_graph(x_rand)

        path = [[self.goal[0], self.goal[1]]]
        debug_path = [[self.goal]]
        last_index = len(self.nodes) - 1
        while self.nodes[last_index].parent is not None:
            node = self.nodes[last_index]
            path.append([node.x[0], node.x[1]])
            debug_path.append([node.x])
            last_index = node.parent
        path.append([self.start[0], self.start[1]])
        save_path(path)

        return path

    def draw_graph(self, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodes:
            if node.parent is not None:
                plt.plot([node.x[0], self.nodes[node.parent].x[0]], [
                         node.x[1], self.nodes[node.parent].x[1]], "-g")

        circles = []
        fig = plt.gcf()
        ax = fig.gca()
        for (ox, oy, size) in self.obstacle_list:
            # plt.plot(ox, oy, "ok", ms=30 * size)
            circle = plt.Circle((ox, oy), size, fill=False)
            circles.append(circle)
        p = PatchCollection(circles)
        ax.add_collection(p)
        ax.set_aspect('equal')

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.axis([self.c_space_bounds[0][0], self.c_space_bounds[0][1],
                  self.c_space_bounds[1][0], self.c_space_bounds[1][1]])
        plt.grid(True)
        plt.pause(0.01)

class Node():

    def __init__(self, x):
        self.x = x
        self.parent = None


def save_path(robot_path, dir_path="path.pkl"):
    robot_path.reverse()
    with open(dir_path, 'wb') as f:
        pickle.dump(robot_path, f)

def main(goal=[-15,80.0,math.pi/2], dimension='3d'):
    print("start " + __file__)

    # ====Search Path with RRT====
    with open('obstacle_list.json') as obstacle_file:
        obstacle_dict = json.load(obstacle_file)
        obstacle_list = obstacle_dict['shipwreck_2']
    # [x,y,size]
    # Set Initial parameters
    c_space_bounds = [(-40, 40), (0, 90), (-math.pi, math.pi)]
    start = [0, 0, math.pi / 2]
    if dimension == '2d':
        c_space_bounds = c_space_bounds[:2]
        start = start[:2]
        goal = goal[:2]

    rrt = Rrt(start=start, goal=goal,
              c_space_bounds=c_space_bounds,
              obstacle_list=obstacle_list)
    path = rrt.algo(animation=show_animation)

    # Draw final path
    rrt.draw_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    # fig, ax = plt.subplots(1, 1)
    # ax.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    # ax.grid(True)
    # ax.set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()