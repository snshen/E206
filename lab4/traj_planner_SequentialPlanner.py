# E206 Motion Planning

# Simple planner
# C Clark

import math
import statistics

import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np
import time
from traj_planner_ExpPlanner import *


class Planning_Problem():
    """ Class that holds a single motion planning problem for one robot.
    """

    def __init__(self, initial_state, desired_state, objects, walls):
        """ Constructor for planning problem.
            - Arguments:
              - initial_state (list of floats): The initial state of the robot (t, x, y, theta).
              - desired_state (list of floats): The desired state of the robot (t, x, y, theta)
              - objects (list of list of floats): The objects to avoid.
              - walls (list of list of floats): The walls to not drive through.
        """
        self.initial_state = initial_state
        self.desired_state = desired_state
        self.objects = objects
        self.walls = walls
        self.planner = Expansive_Planner()

    def add_trajs_as_obstacles(self, traj_set):
        """ Function to add trajs as obstacles to the pp.
            - Arguments
              - traj_set (list of list of floats): The trajectories to be treated as obstacles.
        """
        for traj in traj_set:
            self.objects.append(['traj', traj])


class Sequential_Planner():

    def __init__(self):
        pass

    def construct_traj_set(self, planning_problem_list):
        """ Function that creates a set of trajectories for several planning problems (1 per robot).
            - Parameters:
              - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
            - Returns:
              - traj_set (list of list of floats): One traj per robot.
        """

        traj_set_list = []
        traj_cost_list = []

        for robot in planning_problem_list:
            robot.add_trajs_as_obstacles(traj_set_list)
            temp_traj, traj_cost = robot.planner.construct_optimized_traj(robot.initial_state, robot.desired_state,
                                                                          robot.objects, robot.walls)
            # print("temp traj: ", temp_traj)
            traj_set_list.append(temp_traj)
            traj_cost_list.append(traj_cost)

        return traj_set_list, traj_cost_list


def random_pose(maxR):
    """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
        Parameters:
          - maxR (float): the half width of the square environment in meters.
        Returns:
          - random_pose (list of floats): The x, y, theta in (m, m, rad).
    """
    return [random.uniform(-maxR + 1, maxR - 1), random.uniform(-maxR + 1, maxR - 1), random.uniform(-math.pi, math.pi)]


def get_new_random_pose(pose_list, maxR, radius):
    """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
        Parameters:
          - pose_list (list of floats):
          - maxR (float): the half width of the square environment in meters.
          - radius (float): The objects' radius in meters.
        Returns:
          - random_pose (list of floats): The x, y, theta, radius in (m, m, rad, m).
    """
    new_pose_found = False
    while not new_pose_found:
        new_pose = random_pose(maxR)
        new_pose_found = True
        for pl in pose_list:
            effective_radius = radius + pl[3]
            if (abs(pl[0] - new_pose[1]) < effective_radius and abs(pl[1] - new_pose[2]) < effective_radius) or (
                    abs(pl[0] - new_pose[1]) < effective_radius and abs(pl[1] - new_pose[2]) < effective_radius):
                new_state_found = False

    return new_pose + [radius]


if __name__ == '__main__':
    num_robots = 3
    num_objects = 10
    maxR = 10
    obj_vel = 1

    cost_1 = []
    count_1 = []
    cost_2 = []
    count_2 = []
    cost_3 = []
    count_3 = []

    robot_initial_pose_list = []
    robot_initial_state_list = []
    for _ in range(num_robots):
        # fun_initial = [8, 0, 0.2, 0.4]
        # robot_initial_pose_list += [fun_initial]
        # robot_initial_state_list += [[0, fun_initial[0], fun_initial[1], fun_initial[2]]]
        ns = get_new_random_pose(robot_initial_pose_list, maxR, ROBOT_RADIUS)
        robot_initial_pose_list += [ns]
        robot_initial_state_list += [[0, ns[0], ns[1], ns[2]]]

    robot_desired_pose_list = []
    robot_desired_state_list = []
    for _ in range(num_robots):
        # fun_desired = [-8, 0, 0.2, 0.4]
        # robot_desired_pose_list += [fun_desired]
        # robot_desired_state_list += [[0, fun_desired[0], fun_desired[1], fun_desired[2]]]
        ns = get_new_random_pose(robot_desired_pose_list, maxR, ROBOT_RADIUS)
        robot_desired_pose_list += [ns]
        robot_desired_state_list += [[20, ns[0], ns[1], ns[2]]]

    object_list = []
    for _ in range(num_objects):
        object_radius = random.uniform(0.3, 1.0)
        obj_pose = get_new_random_pose(robot_initial_pose_list + robot_desired_pose_list, maxR, object_radius)
        obj_yaw = obj_pose[2]
        # obj_yaw = wrap_to_pi(4.7)
        object_list += [['obstacle', [obj_pose[0], obj_pose[1], 0.5, obj_vel, obj_yaw]]]

    walls = [[-maxR, maxR, maxR, maxR, 2 * maxR], [maxR, maxR, maxR, -maxR, 2 * maxR],
             [maxR, -maxR, -maxR, -maxR, 2 * maxR], [-maxR, -maxR, -maxR, maxR, 2 * maxR]]

    planning_problem_list = []
    for i in range(num_robots):
        pp = Planning_Problem(robot_initial_state_list[i], robot_desired_state_list[i], object_list, walls)
        planning_problem_list.append(pp)

    strategies = []
    planner = Sequential_Planner()
    traj_list, traj_cost_list = planner.construct_traj_set(planning_problem_list)

    # cost_1 = traj_cost_list
    print("perf:", statistics.mean(traj_cost_list))

    # print("PERFORMANCE:")
    # print("1:", statistics.mean(count_1), statistics.mean(naive_count))
    # print("2:", statistics.mean(weighted_cost), statistics.mean(weighted_count))
    # print("5:", statistics.mean(weighted_cost), statistics.mean(weighted_count))

    if len(traj_list) > 0:
        plot_traj_list(traj_list, object_list, walls)
