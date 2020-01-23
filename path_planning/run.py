# import argparse
import yaml
from math import pi
from matplotlib import pyplot as plt

from rrt2 import RRTPlanner
from control_space_env import ControlSpace


def main(planning_env, planner, start, goal):
    # Notify.
    # input('Press any key to begin planning')
    print('Starting plan')

    # Plan.
    plan, total_cost, tree = planner.plan(start, goal, timeout=10, tmax=10)

    # Visualize the final path.
    planning_env.visualize_plan(plan, tree=tree)
    plt.show()
    exit(0)


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description='script for testing planners')
    #
    # parser.add_argument('-m', '--map', type=str, default='mlo_3',
    #                     help='The environment to plan on')
    # parser.add_argument('-p', '--planner', type=str, default='rrt2',
    #                     help='The planner to run (rrt, rrt2, rrtstar)')
    # parser.add_argument('-s', '--start', nargs='+', type=tuple, default=(0, 0))
    # parser.add_argument('-g', '--goal', nargs='+', type=int, default=(80, -15))
    #
    # args = parser.parse_args()

    with open('obstacle_list.yaml') as obstacle_file:
        scenario_name = 'mlo_3'
        obstacle_dict = yaml.load(obstacle_file)
        obstacle_list = obstacle_dict[scenario_name]['obstacles']
        local_goal = obstacle_dict[scenario_name]['goal']
        local_start = [0, 0]
    start = (0, 0, 0, 0, 0, 0)
    goal = (80, -15, None, None, None, None)
    xlimit = (0, 100)
    ylimit = (-50, 50)
    ulimit = (-0.5, 1.2)
    vlimit = (-0.5, 0.5)
    rlimit = (-0.3, 0.3)
    # input_limits = [(-154, 154), (-50, 50), (-15, 15)]
    input_limits = [(-50, 154), (-50, 50), (-15, 15)]
    state_limits = [xlimit, ylimit, (-pi, pi), (-0.25, 0.5), (-1.0, 1.0), (-0.1, 0.1)]
    # setup the environment
    planning_env = ControlSpace(obstacle_list, start, goal,
                                xlimit, ylimit, vlimit,
                                ulimit, rlimit,
                                input_limits)
    planner = RRTPlanner(planning_env, state_limits, control_type='force')

    # # Next setup the planner
    # if args.planner == 'rrt2':
    #     planner = RRTPlanner(planning_env)
    # else:
    #     print('Unknown planner option: %s' % args.planner)
    #     exit(0)

    main(planning_env, planner, start, goal)
