import numpy as np
from rrt_tree import RRTTree
import random
from math import sin, cos, pi
from copy import deepcopy
from time import time
from control_space_env import ControlSpace


class RRTPlanner(object):

    def __init__(self, planning_env, state_limits):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.bounds = self.planning_env.map_bounds
        self.xlimit = state_limits[0]
        self.ylimit = state_limits[1]
        self.psilimit = state_limits[2]
        self.ulimit = state_limits[3]
        self.vlimit = state_limits[4]
        self.rlimit = state_limits[5]
        self.limits = state_limits

    def plan(self, start_config, goal_config, goal_sample_rate=5, timeout=4.0):
        print(self.bounds)
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.add_vertex(start_config)
        self.tree.set_cost(0, 0)
        cmin = 0
        tmax = 4
        v_min_id = None
        start_time = time()
        while True:
            if time() - start_time > timeout:
                break
            x_rand = self.planning_env.sample(goal_sample_rate)
            c_rand = random.uniform(0, cmin)
            t_rand = random.uniform(0, tmax)
            u_rand = self.planning_env.sample_control
            v_nearest_id, _ = self.tree.get_nearest_vertex(x_rand, c_rand)
            v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.propagate(v_nearest, u_rand, t_rand)
            if self.planning_env.state_validity_checker(v_new):
                if self.planning_env.edge_validity_checker(v_nearest, v_new):
                    v_new_id = len(self.tree.vertices)
                    self.tree.add_vertex(v_new)
                    self.tree.add_edge(v_nearest_id, v_new_id)
                    self.tree.set_cost(v_new_id,
                                       self.tree.cost[v_nearest_id] + self.planning_env.compute_distance(v_nearest,
                                                                                                         v_new))

                    if self.planning_env.compute_distance(goal_config, v_new) < 5 and \
                            (v_min_id is None or self.tree.cost[v_new_id] < self.tree.cost[v_min_id]):
                        v_min_id = v_new_id

        best_vid = v_min_id
        if best_vid is None:
            print('goal not reached')
            return None
        print('goal reached!')
        total_cost = self.tree.cost[best_vid]
        print('Total cost: ', total_cost)
        plan.append(goal_config)
        last_index = best_vid
        while self.tree.edges[last_index] != 0:
            plan.append(self.tree.vertices[last_index])
            last_index = self.tree.edges[last_index]
        plan.append(start_config)

        return np.array(plan), total_cost, self.tree

    def propagate(self, x_nearest, u, t):
        """

        :param x_nearest: [x, y, psi, u, v, r]
        :param u:
        :param t:
        :return:
        """
        x_new = deepcopy(x_nearest)
        # TODO create better integrator
        # Integration over time
        psi_0 = x_nearest[2]
        surge_0 = x_nearest[3]
        sway_0 = x_nearest[4]
        r_0 = x_nearest[5]

        # surge
        surge = max(self.ulimit[0], min(u[0] * t + surge_0, self.ulimit[1]))
        # sway
        # TODO introduce sway
        sway = 0
        # Yaw speed
        r = max(self.rlimit[0], min(u[1] * t + r_0, self.rlimit[1]))
        # Yaw
        psi = (u[1] * t * t + r_0) / 2 + psi_0
        psi = (psi + pi) % 2 - pi

        x = ((surge * cos(psi) - sway * sin(psi)) -
             (surge_0 * cos(psi_0) - sway_0 * sin(psi))) / 2 * t
        y = ((surge * sin(psi) - sway * cos(psi)) -
             (surge_0 * sin(psi_0) - sway_0 * cos(psi))) / 2 * t
        x_new = np.array([x, y, psi, surge, sway, r])

        return x_new


if __name__ == '__main__':
    import yaml

    with open('obstacle_list.yaml') as obstacle_file:
        scenario_name = 'mlo_3'
        obstacle_dict = yaml.load(obstacle_file)
        obstacle_list = obstacle_dict[scenario_name]['obstacles']
        # obstacle_list = 57.3 * np.array(obstacle_dict[scenario_name]['obstacles']) / 6366707.0
        # start = 57.3 * np.array(obstacle_dict[scenario_name]['start']) / 6366707.0 + start
        local_goal = obstacle_dict[scenario_name]['goal']
        local_start = [0, 0]
    start = (0, 0, 0, 0, 0, 0)
    goal = (80, -15, 0, 0, 0)
    xlimit = (0, 100)
    ylimit = (-100, 100)
    vlimit = (-0.25, 0.5)
    state_limits = [xlimit, ylimit, (-pi, pi), (-0.25, 0.5), (-1.0, 1.0), (-0.1, 0.1)]
    planning_env = ControlSpace(obstacle_list, start, goal,
                                xlimit, ylimit, vlimit)
    planner = RRTPlanner(planning_env, state_limits)
    planner.plan(start, goal)
