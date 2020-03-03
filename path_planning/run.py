# import argparse
import yaml
from math import pi
from statistics import mean, stdev
from matplotlib import pyplot as plt
import numpy as np
from coordinate_conversion import AuvPath

from rrt2 import RRTPlanner
from control_space_env import ControlSpace


def main(planning_env, planner, start, goal):
	# Notify.
	# input('Press any key to begin planning')
	# print('Starting plan')

	# Plan.
	# cmin_values = [150, 250, 350, 450]
	cmin_values = [150]
	times = range(1, 16)
	num_iters = 10
	all_success_counts = np.zeros(len(times))
	all_total_costs = np.zeros(len(times))
	for cmin_val in cmin_values:
		for ni in range(num_iters):
			plan, is_success_list, total_costs, tree = planner.plan(start, goal, times, tmax=8, velocity_current=(0, 0.3), cmin=cmin_val, \
													  weights=(None, None, None, None, None, None), goal_sample_rate=5)
			all_success_counts += is_success_list
			all_total_costs += total_costs

	# avg_total_costs = all_total_costs / all_success_counts
	print(all_total_costs)
	print(all_success_counts)
	'''
	total_costs = []
	
	num_iters = 10
	for tm in range(1, 16):
		successes = 0
		for x in range(num_iters):
			plan, total_cost, tree = planner.plan(start, goal, timeout=tm, tmax=8, velocity_current=(0, 0.3), cmin=350,
												  weights=(None, None, None, None, None, None), goal_sample_rate=5)
			total_costs.append(total_cost)
			tree.reset_tree()
			successes += int(plan is not None)

		# print(plan)
		# print(total_cost)
		# print(total_costs)
		total_costs = [c for c in total_costs if c is not None]
		# print(f"Success rate: {float(successes)/10}")
		# print('avg', np.mean(total_costs))
		# print('std', np.std(total_costs, ddof=1))

		print(f"{tm} {float(successes)/float(num_iters)} {np.mean(total_costs)} {np.std(total_costs, ddof=1)}")
# 
	'''
	plan, total_cost, tree = planner.plan(start, goal, timeout=8, tmax=15, velocity_current=(0, 0.3), cmin=10,
												  weights=(None, None, None, None, None, None), goal_sample_rate=5)
	# Visualize the final path.
	if plan is not None:
	    # xml_plan = AuvPath(32.82732, 34.954897, local_path=np.flip(plan, 0))
	    # xml_plan.create_xml('/home/elisei/catkin_ws/src/cola2_sparus2/missions/my_plan.xml')
	
	    planning_env.visualize_plan(plan, tree=tree)
	    plan[0, 2] = 0
	    plan = plan.astype(float)
	    u_e = np.cos(plan[:, 2]) * plan[:, 3] - np.sin(plan[:, 2]) * plan[:, 4]
	    v_e = np.sin(plan[:, 2]) * plan[:, 3] + np.cos(plan[:, 2]) * plan[:, 4]
	    plt.quiver(plan[1:, 1], plan[1:, 0], v_e[1:], u_e[1:])
	else:
	    planning_env.visualize_plan(tree=tree)
	
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
		scenario_name = 'mlo_4'
		obstacle_dict = yaml.load(obstacle_file)
		obstacle_list = obstacle_dict[scenario_name]['obstacles']
		local_goal = obstacle_dict[scenario_name]['goal']
		local_start = [0, 0]
	start = (0, 0, 0, 0, 0, 0)
	# goal = (80, -15, None, 0, None, None)
	goal = (local_goal[0], local_goal[1], None, 0, None, None)
	xlimit = (0, 100)
	ylimit = (-50, 50)
	ulimit = (-0.5, 1.0)
	vlimit = (-0.5, 0.5)
	rlimit = (-0.3, 0.3)
	# input_limits = [(-154, 154), (-50, 50), (-15, 15)]
	# Forces:
	# N = 15( or maybe
	# 150?)
	# Y = 100
	# X = 154
	input_limits = [(0, 150), (-0.1, 0.1), (-1.5, 1.5)]
	# input_limits = [(-150, 150), (-100, 100), (-15, 15)]
	# input_limits = [(-, ), (-1, 1), (-1, 1)]
	state_limits = [xlimit, ylimit, (-pi, pi), ulimit, vlimit, rlimit]
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

	# planning_env.visualize_plan()
	# plt.show()


	main(planning_env, planner, start, goal)
