#!/usr/bin/env python

__author__ = "Elisei Shafer"

import random
import numpy as np

class RRT():

    def __init__(self, start, goal, map_corners, obstacle_list, max_iterations=500,
                 max_extend=1.0, goal_sample_rate=5):
        self.max_iterations = max_iterations
        self.nodes = [Node(start)]
        self.map_corners = map_corners
        self.obstacle_list = obstacle_list
        self.goal_sample_rate = goal_sample_rate
        self.goal = goal
        self.max_extend = max_extend

    def sample_free(self, map_corners):
        if random.randint(0, 100) > self.goal_sample_rate:
            x_rand = [random.uniform(x[0],x[1]) for x in map_corners]
        else:
            x_rand = self.goal
        x_rand = np.array(x_rand)
        return x_rand

    def find_nearest(self, x_rand, nodes):

        dlist = [np.linalg.norm(x_rand-node.x) for node in nodes]
        minind = dlist.index(min(dlist))
        x_nearest = nodes[minind].x
        return x_nearest, minind

    def steer(self, x_nearest, x_rand, max_extend):
        vector_rand_near = x_rand - x_nearest
        extend_length = np.linalg.norm(vector_rand_near)
        if extend_length > max_extend:
            x_new = (vector_rand_near * max_extend) / extend_length
        else:
            x_new = x_rand

        return x_new

    def is_obstacle_free(self, x_nearest, x_new, obstacle_list):

        # for (ox, oy, size) in obstacleList:
        #     dx = ox - node.x
        #     dy = oy - node.y
        #     d = math.sqrt(dx * dx + dy * dy)
        #     if d <= size:
        #         return False  # collision

        return True  # safe

    def rrt_algo(self):

        for i in range(self.max_iterations):
            x_rand = self.sample_free(self.map_corners)
            x_nearest, parent = self.find_nearest(x_rand, self.nodes)
            x_new = self.steer(x_nearest, x_new)

            if self.is_obstacle_free(x_nearest,x_new, self.obstacle_list):
                new_node = Node(x_new)
                new_node.parent = parent
                self.nodes.append(new_node)


class Node():

    def __init__(self, x):
        self.x = x
        self.parent = None

def main(gx=5.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[gx, gy],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:  # pragma: no cover
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()