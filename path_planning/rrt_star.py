#!/usr/bin/env python

__author__ = "Elisei Shafer"

import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import json

show_animation = True


class RrtStar:

    def __init__(self, start, goal, c_space_bounds, obstacle_list, max_iterations=500,
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
            x_rand = [random.uniform(x[0], x[1]) for x in c_space_bounds]
        else:
            x_rand = self.goal
        x_rand = np.array(x_rand)
        return x_rand

    def find_nearest(self, x_rand, nodes):

        dlist = [np.linalg.norm(x_rand - node.x) for node in nodes]
        minind = dlist.index(min(dlist))

        return nodes[minind]

    def steer(self, x_nearest, x_rand, max_extend):
        vector_rand_near = x_rand - x_nearest
        extend_length = np.linalg.norm(vector_rand_near)
        if extend_length > max_extend:
            x_new = (vector_rand_near * max_extend) / extend_length + x_nearest
        else:
            x_new = x_rand

        return x_new

    def is_obstacle_free(self, x_nearest, x_new, obstacle_list):

        for (ox, oy, size) in obstacle_list:
            dx = ox - x_new[0]
            dy = oy - x_new[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

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

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.axis([self.c_space_bounds[0][0], self.c_space_bounds[0][1],
                  self.c_space_bounds[1][0], self.c_space_bounds[1][1]])
        plt.grid(True)
        plt.pause(0.01)

    def algo(self,animation=True):

        eta = self.max_extend * 2.0
        c_space_size = [x[1]-x[0] for x in self.c_space_bounds]
        gamma = 0.78 * np.prod(c_space_size)
        for i in range(self.max_iterations):
            x_rand = self.sample_free(self.c_space_bounds)
            x_nearest = self.find_nearest(x_rand, self.nodes)
            x_new = self.steer(x_nearest.x, x_rand, self.max_extend)

            if self.is_obstacle_free(x_nearest.x,x_new, self.obstacle_list):
                X_near, nearinds = self.near_nodes(self.nodes,x_new, eta, gamma)
                x_min = x_nearest.x
                c_min = x_nearest.cost + np.linalg.norm(x_new - x_nearest.x)
                x_min_ind = len(self.nodes) - 1

                for i, x_near in enumerate(X_near):
                    #TODO add collision check
                    c_i = x_nearest.cost + np.linalg.norm(x_new-x_near.x)

                    if c_i < c_min:
                        # x_min = x_near.x
                        c_min = c_i
                        nearinds[i-1]
                new_node = Node(x_new)
                new_node.parent = nearinds[x_min_ind]
                new_node.cost = c_min
                self.nodes.append(new_node)

                for i, x_near in enumerate(X_near):
                    c_i = new_node.cost + np.linalg.norm(x_new - x_near.x)
                    if c_i < x_near.cost:
                        self.nodes[nearinds[i]].parent = len(self.nodes)-1

            if animation:
                self.draw_graph(x_rand)

        path = [[self.goal[0], self.goal[1]]]
        last_index = len(self.nodes) - 1
        while self.nodes[last_index].parent is not None:
            node = self.nodes[last_index]
            path.append([node.x[0], node.x[1]])
            last_index = node.parent
        path.append([self.start[0], self.start[1]])

    def near_nodes(self, nodes, x_new, eta, gamma):
        nnode = len(nodes)
        r = min([gamma * math.sqrt((math.log(nnode + 1) / (nnode + 1))), eta])
        dlist = [np.linalg.norm(x_new - node.x) for node in nodes]
        nearinds = [dlist.index(i) for i in dlist if i <= r]
        X_near = [nodes[nearind] for nearind in nearinds]
        return X_near, nearinds


class Node():

    def __init__(self, x):
        self.x = x
        self.parent = None
        self.cost = 0.0


def main(goal=[-6,35.0,math.pi/2], dimension='3d'):
    print("start " + __file__)

    # ====Search Path with RRT====
    with open('obstacle_list.json') as obstacle_file:
        obstacle_dict = json.load(obstacle_file)
        obstacle_list = obstacle_dict['mlo']
    # [x,y,size]
    # Set Initial parameters
    c_space_bounds = [(-10, 10), (0, 40), (-math.pi, math.pi)]
    start = [0, 0, math.pi / 2]
    if dimension == '2d':
        c_space_bounds = c_space_bounds[:2]
        start = start[:2]
        goal = goal[:2]

    rrt = RrtStar(start=start, goal=goal,
              c_space_bounds=c_space_bounds,
              obstacle_list=obstacle_list)
    path = rrt.algo(animation=show_animation)

    # Draw final path
    rrt.draw_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
