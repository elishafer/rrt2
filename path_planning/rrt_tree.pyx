import operator
import numpy as np
from math import sqrt


class RRTTree(object):

    def __init__(self, planning_env):

        self.planning_env = planning_env
        self.vertices = []
        self.edges = dict()
        self.cost = dict()

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def get_nearest_vertex(self, state, sample_cost, wx=(None, None, None, None, None, None), wy=1):
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

    def get_knn(self, config, k):
        '''
        Return k-nearest neighbors
        @param config Sampled configuration.
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance(config, v))

        dists = np.array(dists)
        k = min(k, len(dists) - 1)
        knn_ids = np.argpartition(dists, k)
        # knnDists = [dists[i] for i in knn_ids]

        return knn_ids[:k]  # , [self.vertices[vid] for vid in knn_ids]

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
        self.edges[eid] = sid

    def set_cost(self, vid, cost):
        self.cost[vid] = cost

    def reset_tree(self):
        self.vertices = []
        self.edges = dict()
        self.cost = dict()
