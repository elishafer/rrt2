import numpy as np
from matplotlib import pyplot as plt
from math import sqrt


class ControlSpace(object):

    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.goal = goal
        self.map = mapfile
        self.xlimit = ()
        self.ylimit = ()
        self.psilimit = ()
        self.vlimit = ()

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

    def compute_distance(self, start_state, end_state):
        sum = 0
        for i, start_var in enumerate(start_state):
            sum += (start_var - end_state[i]) ** 2
        return sqrt(sum)

    def state_validity_checker(self, state, obstacle_list):

        for (ox, oy, size) in obstacle_list:
            dx = ox - state[0]
            dy = oy - state[1]
            d = sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def edge_validity_checker(self, state1, state2):
        # TODO
        pass

    def compute_heuristic(self, config):
        return self.compute_distance(config, self.goal)

    def sample(self, goal_sample_rate):
        # TODO
        pass
        # if random.randint(0, 100) > goal_sample_rate:
        #     x_rand = [random.uniform(x[0], x[1]) for x in self.bounds]
        # else:
        #     x_rand = self.planning_env.goal
        # x_rand = np.array(x_rand)
        # return x_rand

    def visualize_plan(self, plan=None, visited=None, tree=None, title=None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        # TODO
        plt.imshow(self.map, interpolation='nearest', cmap='Greys')
        if visited is not None:
            plt.imshow(visited)
        elif tree is not None:
            nodes = tree.vertices
            for k, v in tree.edges.items():
                plt.plot([nodes[k][1], nodes[v][1]], [
                    nodes[k][0], nodes[v][0]], "-g")
        if plan is not None:
            for i in range(np.shape(plan)[0] - 1):
                x = [plan[i, 0], plan[i + 1, 0]]
                y = [plan[i, 1], plan[i + 1, 1]]
                plt.plot(y, x, 'r')
        if title: plt.title(title)
        plt.show()
