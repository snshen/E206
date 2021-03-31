# E206 Motion Planning

# Simple planner
# C Clark
import copy
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


def dist_between_i_d(pp):
    return math.sqrt(
        (pp.desired_state[1] - pp.initial_state[1]) ** 2 + (pp.desired_state[2] - pp.initial_state[2]) ** 2)

class Sequential_Planner():
    PLAN_TIME_BUDGET = 10.0  # s
    LARGE_NUMBER = 9999999

    def __init__(self):
        pass

    def construct_traj_set(self, planning_problem_list, strategy):
        """ Function that creates a set of trajectories for several planning problems (1 per robot).
            - Parameters:
              - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
            - Returns:
              - traj_set (list of list of floats): One traj per robot.
        """

        traj_set_list = []
        traj_cost_list = []
        traj_constr_list = []

        for robot in planning_problem_list:
            robot.add_trajs_as_obstacles(traj_set_list)
            temp_traj, traj_cost, constr_time = robot.planner.construct_optimized_traj(robot.initial_state, robot.desired_state,
                                                                          robot.objects, robot.walls, strategy)
            # print("temp traj: ", temp_traj)
            traj_set_list.append(temp_traj)
            traj_cost_list.append(traj_cost)
            traj_constr_list.append(constr_time)

        return traj_set_list, traj_cost_list, traj_constr_list

    def construct_traj_set_(self, planning_problem_list, selection_strategy, order_strategy):
        """ Function that creates a set of trajectories for several planning problems (1 per robot).
            - Parameters:
              - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
            - Returns:
              - traj_set (list of list of floats): One traj per robot.
        """

        traj_set_list = []
        traj_cost_list = []

        if order_strategy == 'static':
            pass
        elif order_strategy == 'random':
            random.shuffle(planning_problem_list)
        elif order_strategy == 'priority':
            planning_problem_list.sort(key=dist_between_i_d)

        for robot in planning_problem_list:
            robot.add_trajs_as_obstacles(traj_set_list)
            temp_traj, traj_cost = robot.planner.construct_traj(robot.initial_state, robot.desired_state,
                                                                          robot.objects, robot.walls, selection_strategy)
            traj_set_list.append(temp_traj)
            traj_cost_list.append(traj_cost)

        return traj_set_list, traj_cost_list

    def construct_optimized_traj_set(self, planning_problem_list, selection_strategy, order_strategy):
        """ Function that creates a set of trajectories for several planning problems (1 per robot).
                    - Parameters:
                      - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
                    - Returns:
                      - traj_set (list of list of floats): One traj per robot.
        """

        start_time = time.perf_counter()
        curr_time = start_time
        best_traj = []
        best_traj_cost = 10*[self.LARGE_NUMBER]
        construction_time = []

        while (curr_time - start_time) < self.PLAN_TIME_BUDGET:
            # print(curr_time - start_time)
            planning_problem_list_copy = copy.deepcopy(planning_problem_list)
            temp_traj, traj_cost = self.construct_traj_set_(planning_problem_list_copy, selection_strategy, order_strategy)
            if statistics.mean(traj_cost) < statistics.mean(best_traj_cost):
                best_traj = temp_traj
                best_traj_cost = traj_cost

            # update times
            temp_time = curr_time
            curr_time = time.perf_counter()
            construction_time.append(curr_time - temp_time)

        # print(f"average length to meas: {statistics.mean(construction_time)}")

        return best_traj, best_traj_cost, construction_time


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
    num_robots = 5
    num_objects = 10
    maxR = 10
    obj_vel = 1

    # dict_lengths = {"uniform": [], "random": [], "gaussian": []}
    # dict_cost = {"uniform": [], "random": [], "gaussian": []}
    dict_lengths = {"static": [], "random": [], "priority": []}
    dict_cost = {"static": [], "random": [], "priority": []}
    cost_1 = []
    # count_1 = []
    cost_2 = []
    # count_2 = []
    cost_3 = []
    # count_3 = []

    for j in range(5):
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

        # velocities = ["random", "uniform", "gaussian"]
        velocities = ["random"]
        order_strategy = ["static", "random", "priority"]
        for vel in velocities:
            for order in order_strategy:
                # print(vel)
                planner = Sequential_Planner()
                planning_problem_list_copy = copy.deepcopy(planning_problem_list)
                # traj_list, traj_cost_list, traj_constr_list = planner.construct_traj_set(planning_problem_list_copy, vel)
                traj_list, traj_cost_list, traj_constr_list = planner.construct_optimized_traj_set(planning_problem_list_copy, vel, order)
                dict_lengths[order].append(statistics.mean(traj_constr_list))
                dict_cost[order].append(sum(traj_cost_list))

        print("after run #", j)
        # for vel in ["random", "uniform", "gaussian"]:
        # for vel in ["random"]:
        for vel in ["static", "random", "priority"]:
            print(vel, ": ", statistics.mean(dict_lengths[vel]), ", ", statistics.mean(dict_cost[vel]))

    # print("PERFORMANCE:")
    # for vel in ["random"]:
    #     print(vel, ": ", statistics.mean(dict_lengths[vel]))

    # if len(traj_list) > 0:
    #     plot_traj_list(traj_list, object_list, walls)
