import numpy as np
from rrt_tree import RRTTree
import random

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.map_shape = self.planning_env.map.shape
        self.bounds = [(0, self.map_shape[0] - 1), (0,self.map_shape[1] - 1)]
        

    def Plan(self, start_config, goal_config, eta=float(5.0), goal_sample_rate=5, max_iterations=8000):
        print(self.bounds)
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        self.tree.SetCost(0,0)

        # TODO (student): Implement your planner here.
        # for i in range(max_iterations):
        while True:
            x_rand = self.sample(goal_sample_rate)
            v_nearest_id, dist = self.tree.GetNearestVertex(x_rand)
            v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.extend(v_nearest, x_rand, eta)
            v_new = np.int_(v_new.round())
            if self.planning_env.state_validity_checker(v_new):
                if self.planning_env.edge_validity_checker(v_nearest, v_new):
                    v_new_id = len(self.tree.vertices)
                    self.tree.AddVertex(v_new)
                    self.tree.AddEdge(v_nearest_id, v_new_id)
                    self.tree.SetCost(v_new_id,
                                      self.tree.cost[v_nearest_id] + self.planning_env.compute_distance(v_nearest, v_new))
                else:
                    continue
            else:
                continue

            if self.planning_env.compute_distance(goal_config, v_new) < 1:
                print('goal reached!')
                total_cost = self.tree.cost[v_new_id]
                break

        plan.append(goal_config)
        last_index = len(self.tree.vertices) - 1
        while self.tree.edges[last_index] is not 0:
            plan.append(self.tree.vertices[last_index])
            last_index = self.tree.edges[last_index]
        plan.append(start_config)

        return np.array(plan), total_cost, self.tree

    def extend(self, v_nearest, x_rand, eta):
        extend_length = self.planning_env.compute_distance(x_rand, v_nearest)
        if extend_length > eta:
            return ((x_rand - v_nearest)*eta) / extend_length + v_nearest
        else:
            return x_rand