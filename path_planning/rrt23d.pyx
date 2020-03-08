# cython: language_level=3, boundscheck=False

import numpy as np
from rrt_tree import RRTTree
import random
from math import sin, cos, pi
from copy import deepcopy
from time import time


class RRTPlanner3d(object):

    def __init__(self, planning_env, state_limits, goal_radius=5, control_type='force'):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.bounds = self.planning_env.map_bounds
        self.goal_radius = goal_radius
        self.control_type = control_type

    def plan(self, start_config, goal_config, timeout=10, goal_sample_rate=5, tmax=8, velocity_current=None,
             cmin=0, weights=(None, None, None)):
        # print(self.bounds)
        # Initialize an empty plan.
        plan = []
        # Start with adding the start configuration to the tree.
        self.tree.add_vertex(start_config)
        self.tree.set_cost(0, 0)
        v_min_id = None
        start_time = time()
        while True:
            if (time() - start_time > timeout):
                break
            x_rand = self.planning_env.sample(goal_sample_rate)
            c_rand = random.uniform(0, cmin)
            t_rand = random.uniform(2, tmax)
            u_rand = self.planning_env.sample_control()
            v_nearest_id, _ = self.tree.get_nearest_vertex(x_rand, c_rand, wx=weights, wy=0.5)
            v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.propagate(v_nearest, u_rand, t_rand, control_type=self.control_type,
                                   v_current=velocity_current)
            if self.planning_env.state_validity_checker(v_new):
                if self.planning_env.edge_validity_checker(v_nearest, v_new):
                    new_cost = self.tree.cost[v_nearest_id] + \
                               self.planning_env.compute_distance(v_nearest, v_new,
                                                                  w=weights)  # + t_rand * abs(v_new[4])
                    if cmin == 0 or new_cost < cmin:
                        v_new_id = len(self.tree.vertices)
                        self.tree.add_vertex(v_new)
                        self.tree.add_edge(v_nearest_id, v_new_id)
                        self.tree.set_cost(v_new_id, new_cost)

                        if self.planning_env.goal_radius_reached(v_new, r=self.goal_radius) and \
                                (v_min_id is None or self.tree.cost[v_new_id] < self.tree.cost[v_min_id]):
                            v_min_id = v_new_id
                            cmin = self.tree.cost[v_new_id]
                            v_min_id = self.tree.prune_tree(cmin, v_min_id)  # same vertex, new id

            # self.planning_env.visualize_plan(tree=self.tree, rnd=x_rand[:2])

        best_vid = v_min_id
        if best_vid is None:
            # print('goal not reached')
            return None, None, None
        # print('goal reached!')
        total_cost = self.tree.cost[best_vid]
        # print('Total cost: ', total_cost)
        plan.append(goal_config)
        last_index = best_vid
        while self.tree.edges[last_index] != 0:
            plan.append(self.tree.vertices[last_index])
            last_index = self.tree.edges[last_index]
        plan.append(start_config)

        return np.array(plan), total_cost, self.tree

    def propagate(self, x_nearest, u, t, control_type, v_current=None):
        """

        :param control_type:
        :param x_nearest: [x, y, psi, u, v, r]
        :param u: [X, Y, N] or [udot, vdot, rdot]
        :param t:
        :return:

        Xuu = -23.6
        Yvv   = -183.42
        Nrr   = -26.4
        m = 63.2
        I = 12.1
        Added mass:
        Nr = -11.75
        Xu = -29
        Yv = -20

        Forces:
        N = 15 (or maybe 150?)
        Y = 100
        X = 154

        umax  = 1.8; cruise = 0.5 ~ 0.7
        vmax = 0.5
        rmax = 0.3
        """

        x_new = deepcopy(x_nearest)
        # TODO create better integrator
        # Integration over time
        psi_0 = x_nearest[2]

        # surge
        surge = 0.5
        # sway
        sway = 0
        # Yaw speed
        # r = max(self.rlimit[0], min(vdot * t + r_0, self.rlimit[1]))
        # Yaw
        # TODO Change yaw rate to physical value
        psi = random.uniform(-0.1, 0.1) * t + psi_0
        psi = (psi + pi) % (2 * pi) - pi

        cpsi0 = cos(psi_0)
        spsi0 = sin(psi_0)
        cpsi = cos(psi)
        spsi = sin(psi)

        x_point = ((surge * cpsi - sway * spsi) +
                   (surge * cpsi0 - sway * spsi0)) / 2 * t + \
                  x_nearest[0]
        y_point = ((surge * spsi + sway * cpsi) +
                   (surge * spsi0 + sway * cpsi0)) / 2 * t + \
                  x_nearest[1]
        if v_current is not None:
            x_point += v_current[0] * t
            y_point += v_current[1] * t

        x_new = np.array([x_point, y_point, psi])

        return x_new
