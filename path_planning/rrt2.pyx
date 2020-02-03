import numpy as np
from rrt_tree import RRTTree
import random
from math import sin, cos, pi
from copy import deepcopy
from time import time
from control_space_env import ControlSpace


class RRTPlanner(object):

    def __init__(self, planning_env, state_limits, goal_radius=5, control_type='force'):
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
        self.goal_radius = goal_radius
        self.control_type = control_type

    def plan(self, start_config, goal_config, goal_sample_rate=5, timeout=float(5), tmax=8, velocity_current=None,
             cmin=0):
        print(self.bounds)
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.add_vertex(start_config)
        self.tree.set_cost(0, 0)
        v_min_id = None
        start_time = time()
        while True:
            if time() - start_time > timeout:
                break
            x_rand = self.planning_env.sample(goal_sample_rate)
            c_rand = random.uniform(0, cmin)
            t_rand = random.uniform(0, tmax)
            u_rand = self.planning_env.sample_control()
            v_nearest_id, _ = self.tree.get_nearest_vertex(x_rand, c_rand, wy=0.5)
            v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.propagate(v_nearest, u_rand, t_rand, control_type=self.control_type,
                                   v_current=velocity_current)
            if self.planning_env.state_validity_checker(v_new):
                if self.planning_env.edge_validity_checker(v_nearest, v_new):
                    v_new_id = len(self.tree.vertices)
                    self.tree.add_vertex(v_new)
                    self.tree.add_edge(v_nearest_id, v_new_id)
                    self.tree.set_cost(v_new_id,
                                       self.tree.cost[v_nearest_id] +
                                       self.planning_env.compute_distance(v_nearest, v_new,
                                                                          w=(None, None, 10, None, None, None)) +
                                       t_rand * abs(v_new[4]))
                    # +
                    #                    t_rand)

                    if self.planning_env.goal_radius_reached(v_new, r=self.goal_radius) and \
                            (v_min_id is None or self.tree.cost[v_new_id] < self.tree.cost[v_min_id]):
                        v_min_id = v_new_id
                        cmin = self.tree.cost[v_new_id]

            # self.planning_env.visualize_plan(tree=self.tree, rnd=x_rand[:2])

        best_vid = v_min_id
        if best_vid is None:
            print('goal not reached')
            return None, None, self.tree
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
        Xu = -56.48 or -29 (adj)
        Yv = -60.817 or -20

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
        surge_0 = x_nearest[3]
        sway_0 = x_nearest[4]
        r_0 = x_nearest[5]

        # Set
        Xuu = -23.6
        Yvv = -183.42
        Nrr = -26.4
        m = 63.2
        I = 12.1
        Nr = 11.75
        Xu = 29
        Yv = 20

        if control_type == 'force':
            # Calculate accelerations:
            # TODO: Is it accurate to assume constant velocity?
            udot = (u[0] + Xuu * surge_0 * abs(surge_0)) / (m + Xu)
            vdot = (u[1] + Yvv * sway_0 * abs(sway_0)) / (m + Yv)
            rdot = (u[2] + Nrr * r_0 * abs(r_0)) / (I + Nr)
        elif control_type == 'acceleration':
            udot = u[0]
            vdot = u[1]
            rdot = u[2]
        else:
            raise Exception(
                'control type should be force or acceleration. Value of control_type was: {}'.format(control_type))
        # surge
        surge = max(self.ulimit[0], min(udot * t + surge_0, self.ulimit[1]))
        # sway
        sway = max(self.vlimit[0], min(vdot * t + sway_0, self.ulimit[1]))
        # Yaw speed
        r = max(self.rlimit[0], min(vdot * t + r_0, self.rlimit[1]))
        # Yaw
        psi = (rdot * t * t + r_0) / 2 + psi_0
        psi = (psi + pi) % (2 * pi) - pi

        cpsi0 = cos(psi_0)
        spsi0 = sin(psi_0)
        cpsi = cos(psi)
        spsi = sin(psi)
        # if v_current is not None:
        # u_c0 = cpsi0 * v_current[0] - spsi0 * v_current[1]
        # v_c0 = spsi0 * v_current[0] + cpsi0 * v_current[1]
        # u_c = cpsi * v_current[0] - spsi * v_current[1]
        # v_c = spsi * v_current[0] + cpsi * v_current[1]

        x_point = ((surge * cpsi - sway * spsi) +
                   (surge_0 * cpsi0 - sway_0 * spsi0)) / 2 * t + \
                  x_nearest[0]
        y_point = ((surge * spsi + sway * cpsi) +
                   (surge_0 * spsi0 + sway_0 * cpsi0)) / 2 * t + \
                  x_nearest[1]
        if v_current is not None:
            x_point += v_current[0] * t
            y_point += v_current[1] * t

        x_new = np.array([x_point, y_point, psi, surge, sway, r])

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
    goal = (80, -15, None, None, None, None)
    xlimit = (0, 100)
    ylimit = (-50, 50)
    vlimit = (-0.25, 0.5)
    state_limits = [xlimit, ylimit, (-pi, pi), (-0.25, 0.5), (-1.0, 1.0), (-0.1, 0.1)]
    planning_env = ControlSpace(obstacle_list, start, goal,
                                xlimit, ylimit, vlimit)
    planner = RRTPlanner(planning_env, state_limits)
    plan, total_cost, tree = planner.plan(start, goal)

    # planning_env.visualize_plan(plan, tree=tree)
    # planning_env.visualize_plan(plan)
    # plt.show()
