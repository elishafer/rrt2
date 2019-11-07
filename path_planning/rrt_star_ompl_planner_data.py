#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Ryan Luna

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from math import sqrt
import json

with open('obstacle_list.json') as obstacle_file:
    obstacle_dict = json.load(obstacle_file)
    obstacle_list = obstacle_dict['mlo_3']


def isStateValid(state):
    x = state.getX()
    y = state.getY()
    for (ox, oy, r) in obstacle_list:
        c = clearance(ox, oy, x, y, r)
        if c <= 0:
            return False
    return True


def clearance(ox, oy, x, y, r):
    return sqrt((ox - x)**2 + (oy - y)**2)-r


def plan():
    # construct the state space we are planning in
    space = ob.SE2StateSpace()

    # set the bounds for R^3 portion of SE(3)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0,-40)
    bounds.setHigh(0,40)
    bounds.setLow(1,0)
    bounds.setHigh(1,90)
    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # create a start state
    start = ob.State(space)
    start().setX(0)
    start().setY(0)
    start().setYaw(0)
    # start().setZ(-9)
    # start().rotation().setIdentity()

    # create a goal state
    goal = ob.State(space)
    goal().setX(-25)
    goal().setY(60)
    goal().setYaw(0)
    # goal().setZ(-9)
    # goal().rotation().setIdentity()

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # Lets use PRM.  It will have interesting PlannerData
    planner = og.RRTstar(ss.getSpaceInformation())
    ss.setPlanner(planner)
    ss.setup()

    # attempt to solve the problem
    solved = ss.solve(1.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath())

        # Extracting planner data from most recent solve attempt
        pd = ob.PlannerData(ss.getSpaceInformation())
        ss.getPlannerData(pd)

        # Computing weights of all edges based on state space distance
        pd.computeEdgeWeights()

        path = get_path(ss)
        draw_graph(path)
        plt.show()

def get_path(ss):
    path_geometric = ss.getSolutionPath()
    path_states = path_geometric.getStates()
    i=0
    out_path = []
    while i<path_geometric.getStateCount():
        out_path.append((path_states[i].getX(), path_states[i].getY()))
        i += 1
    return out_path


def draw_graph(path):
    plt.clf()

    # for node in self.nodes:
    #     if node.parent is not None:
    #         plt.plot([node.x[0], self.nodes[node.parent].x[0]], [
    #             node.x[1], self.nodes[node.parent].x[1]], "-g")

    circles = []
    fig = plt.gcf()
    ax = fig.gca()
    for (ox, oy, size) in obstacle_list:
        # plt.plot(ox, oy, "ok", ms=30 * size)
        circle = plt.Circle((ox, oy), size, fill=False)
        circles.append(circle)
    p = PatchCollection(circles)
    ax.add_collection(p)
    ax.set_aspect('equal')

    plt.plot(path[0][0], path[0][1], "xr")
    plt.plot(path[-1][0], path[-1][1], "xr")
    plt.axis([-90, 90,
              -90, 90])
    plt.grid(True)
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')




if __name__ == "__main__":
    plan()
