#!/usr/bin/env python

__author__ = "Elisei Shafer"

import math
from math import sqrt
import random
import numpy as np
import yaml
from primatives import mynorm, is_path_collision_free, safety_cost_func

show_animation = False
debug = False

if debug == True:
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection


class RrtStar:

    def __init__(self, start, goal, c_space_bounds, obstacle_list, max_iterations=700,
                 max_extend=4.0, goal_sample_rate=1):
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

        dlist = [mynorm(x_rand - node.x) for node in nodes]
        minind = dlist.index(min(dlist))

        return nodes[minind], minind

    def steer(self, x_nearest, x_rand, max_extend):
        vector_rand_near = x_rand - x_nearest
        extend_length = mynorm(vector_rand_near)
        if extend_length > max_extend:
            x_new = (vector_rand_near * max_extend) / extend_length + x_nearest
        else:
            x_new = x_rand

        return x_new

    def is_obstacle_free(self, x_new, obstacle_list):

        for (ox, oy, size) in obstacle_list:
            dx = ox - x_new[0]
            dy = oy - x_new[1]
            d = sqrt(dx * dx + dy * dy)
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
                plt.plot([node.x[1], self.nodes[node.parent].x[1]], [
                    node.x[0], self.nodes[node.parent].x[0]], "-g")

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
        plt.axis([self.c_space_bounds[1][0], self.c_space_bounds[1][1],
                  self.c_space_bounds[0][0], self.c_space_bounds[0][1]])
        plt.grid(True)
        # plt.pause(0.001)

    def algo(self, animation=False):

        eta = self.max_extend * 8.0
        c_space_size = [x[1] - x[0] for x in self.c_space_bounds]
        # Checkout gamma in Karaman Frazzoli 2011. Also in documentation
        gamma = 0.9972 * sqrt(np.prod(c_space_size))*1.2
        for i in range(self.max_iterations):
            x_rand = self.sample_free(self.c_space_bounds)
            x_nearest, x_min_ind = self.find_nearest(x_rand, self.nodes)
            x_new = self.steer(x_nearest.x, x_rand, self.max_extend)

            if self.is_obstacle_free(x_new, self.obstacle_list):
                X_near, nearinds = self.near_nodes(self.nodes, x_new, eta, gamma)
                x_min = x_nearest.x
                c_min = x_nearest.cost + mynorm(x_new - x_nearest.x)

                for i, x_near in enumerate(X_near):
                    # Check for lowest cost node to connect to.
                    if not is_path_collision_free(x_near.x, x_new, self.obstacle_list):
                        # Collision check in middle between 2 nodes
                        continue
                    # c_i = x_near.cost + mynorm(x_new-x_near.x)
                    c_i = self.cost_func(x_near.cost, x_new, x_near.x)

                    if c_i < c_min:
                        # x_min = x_near.x
                        c_min = c_i
                        x_min_ind = nearinds[i]

                new_node = Node(x_new)
                new_node.parent = x_min_ind
                new_node.cost = c_min
                self.nodes.append(new_node)

                for i, x_near in enumerate(X_near):
                    # Rewire adjacent nodes
                    if not is_path_collision_free(x_near.x, x_new, self.obstacle_list):
                        # Collision check in middle between 2 nodes
                        continue
                    # c_i = new_node.cost + mynorm(x_new - x_near.x)
                    c_i = self.cost_func(new_node.cost, x_near.x, x_new)
                    if c_i < x_near.cost:
                        self.nodes[nearinds[i]].parent = len(self.nodes) - 1

            if animation:  # and i%5:
                self.draw_graph(x_rand)

        path = [[self.goal[0], self.goal[1]]]
        last_index = self.get_best_last_index()
        while self.nodes[last_index].parent is not None:
            node = self.nodes[last_index]
            path.append([node.x[0], node.x[1]])
            last_index = node.parent
        path.append([self.start[0], self.start[1]])
        return path

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x) for node in self.nodes]
        goalinds = [disglist.index(i) for i in disglist if i <= self.max_extend]

        if not goalinds:
            print('goal not reached')
            return None

        mincost = min([self.nodes[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodes[i].cost == mincost:
                return i

        return None

    def calc_dist_to_goal(self, x):
        return mynorm(np.array(x) - np.array(self.goal))

    def near_nodes(self, nodes, x_new, eta, gamma):
        nnode = len(nodes)
        r = min([gamma * sqrt((math.log(nnode + 1) / (nnode + 1))), eta])
        dlist = [mynorm(x_new - node.x) for node in nodes]
        nearinds = [dlist.index(distance) for distance in dlist if distance <= r]
        X_near = [nodes[nearind] for nearind in nearinds]
        return X_near, nearinds

    def cost_func(self, prev_cost, x_new, x_prev):

        dist = mynorm(x_new - x_prev)
        safety_cost = safety_cost_func(x_new[0], x_new[1], self.obstacle_list)
        # safety cost function currently deactivated: uncomment line below to activate
        # safety_cost = 0
        new_cost = prev_cost + dist + safety_cost
        return new_cost

    # def safety_cost_func(self, x, y):
    #
    #     for (ox, oy, ro) in self.obstacle_list:
    #         # TODO change 123 to variable
    #         bounds = np.array([1, 2, 3])
    #         for r in bounds:
    #             if (ox + x) ** 2 + (oy + y) ** 2 < ((ro + r) ** 2):
    #                 return 4 - r
    #     return 0


def save_path(robot_path, dir_path="path.pkl"):
    robot_path.reverse()
    with open(dir_path, 'wb') as f:
        pickle.dump(robot_path, f)


class Node():

    def __init__(self, x):
        self.x = x
        self.parent = None
        self.cost = 0.0


def ros_path_planning_node(s, t, obstacle_list, step_size, ecef=False):
    """
    :param s:
    :param t:
    :param obstacle_list:
    :param step_size:
    :return:
    """
    # Note here all coordinates are in ned
    # We add padding of 10m around the start and target
    # Maybe should change this to use library.
    pad = 20.0
    if ecef:
        r_e = 6366707.0
        pad = 57.3 * pad / r_e
        step_size = 57.3 * step_size / r_e

    c_space_bounds = [(min(s[0], t[0]) - pad/2, max(s[0], t[0]) + pad),
                      (min(s[1], t[1]) - pad*2, max(s[1], t[1]) + pad*2)]
    rrt = RrtStar(start=s, goal=t,
                  c_space_bounds=c_space_bounds,
                  obstacle_list=obstacle_list,
                  max_extend=step_size
                  )
    path = rrt.algo(False)
    return path, rrt


def local_to_geo(local, geo_origin):
    return 57.3 * (np.array(local)) / 6366707.0 + np.array(geo_origin)

# TODO
def geo_to_local(geo_origin, geo_point):
    pass

def main(goal=[-15, 80.0]):
    print("start " + __file__)
    # goal = [32.826240, 34.956214]
    start = [32.827320, 34.954897]
    # goal = [50, 50]
    # start = [5, 5]

    # TODO refactor obstacle_list for lat lon.
    with open('obstacle_list.yaml') as obstacle_file:
        scenario_name = 'mlo_3'
        obstacle_dict = yaml.load(obstacle_file)
        obstacle_list = obstacle_dict[scenario_name]['obstacles']
        # obstacle_list = 57.3 * np.array(obstacle_dict[scenario_name]['obstacles']) / 6366707.0
        # start = 57.3 * np.array(obstacle_dict[scenario_name]['start']) / 6366707.0 + start
        local_goal = obstacle_dict[scenario_name]['goal']
        local_start = [0,0]
        # goal = 57.3 * goal  / 6366707.0 + start
        # obstacle_list = [[obstacle[1] + start[0], obstacle[0] + start[1], obstacle[2]] for obstacle in obstacle_list]
    # obstacle_list = [[0, 0, 0.001]]
    # [x,y,size]
    # Set Initial parameters
    path, rrt = ros_path_planning_node(local_start, local_goal, obstacle_list, 5.0)
    # save_path(path)
    if debug == True:
        # Draw final path
        rrt.draw_graph()
        plt.plot([e for (n, e) in path], [n for (n, e) in path], '-r')
        plt.grid(True)
        plt.show()

        plt.figure()
        path = local_to_geo(path, start)
        fig = plt.gcf()
        ax = fig.gca()
        ax.set_aspect('equal')
        plt.plot([e for (n, e) in path], [n for (n, e) in path], '-r')
        plt.show()

if __name__ == '__main__':
    main()
