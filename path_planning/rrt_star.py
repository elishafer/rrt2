#!/usr/bin/env python

__author__ = "Elisei Shafer"

import math
from rrt import Rrt
from rrt import Node
import numpy as np


class RrtStar(Rrt):

    def __init__(self, start, goal, c_space_bounds, obstacle_list, max_iterations=500,
                 max_extend=2.0, goal_sample_rate=5):
        super().__init__(start, goal, c_space_bounds, obstacle_list, max_iterations,
                         max_extend, goal_sample_rate)

    def algo(self,animation=True):

        eta = self.max_extend * 2.0
        c_space_size = [x[1]-x[0] for x in self.c_space_bounds]
        gamma = 0.78 * np.prod(c_space_size)
        for i in range(self.max_iterations):
            x_rand = self.sample_free(self.c_space_bounds)
            x_nearest, parent = self.find_nearest(x_rand, self.nodes)
            x_new = self.steer(x_nearest, x_rand, self.max_extend)

            if self.is_obstacle_free(x_nearest,x_new, self.obstacle_list):
                X_near = self.near_nodes(self.nodes,x_new, eta, gamma)
                x_min = x_nearest


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
        dlist = [np.linalg.norm(x_new - node) for node in nodes]
        nearinds = [dlist.index(i) for i in dlist if i <= r]
        return nearinds



def main(goal=[-6,35.0,math.pi/2], dimension='3d'):
    print("start " + __file__)

    # ====Search Path with RRT====
    with open('obstacle_list.json') as obstacle_file:
        obstacle_dict = json.load(obstacle_file)
        obstacle_list = obstacle_dict['shipwreck']
    # [x,y,size]
    # Set Initial parameters
    c_space_bounds = [(-10, 10), (0, 40), (-math.pi, math.pi)]
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
    plt.show()

        if __name__ == '__main__':
            main()
