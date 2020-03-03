# cython: profile=True
# cython: language_level=3, boundscheck=False

import operator
from math import sqrt


class RRTTree(object):

    def __init__(self, planning_env):

        self.planning_env = planning_env
        self.vertices = []
        self.edges = [0]
        self.cost = []

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def get_nearest_vertex(self, state, sample_cost, wx, wy=1):
        '''
        Returns the nearest state ID in the tree.
        :param sample_cost:
        :param state:
        '''
        dists = []
        for vid, v in enumerate(self.vertices):
            x_dist = self.planning_env.compute_distance(state, v, squared=True, w=wx)
            if sample_cost != 0:
                y_dist = (self.cost[vid] - sample_cost) ** 2
                dists.append(sqrt(x_dist + y_dist * wy))
            else:
                dists.append(sqrt(x_dist))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]

    def add_vertex(self, config):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def add_edge(self, sid, eid):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges.append(sid)

    def set_cost(self, vid, cost):
        self.cost.append(cost)

    def reset_tree(self):
        self.vertices = []
        self.edges = [0]
        self.cost = []

    def prune_tree(self, max_cost, v_min_id):
        i = len(self.vertices) - 1
        while i >= 0:
            if self.cost[i] > max_cost: # max_cost = cmin
                del self.cost[i]
                del self.vertices[i]
                del self.edges[i]
                for j in range(i, len(self.edges)):
                    if self.edges[j] > i:
                        self.edges[j] -= 1
                if v_min_id > i:
                    v_min_id -= 1
            i -= 1
        return v_min_id
