import gym
import gym_fetch
import time
import math
import random

from traj_planner_A_star import A_Star_Planner
from traj_planner_utils import *
from traj_tracker import *
import numpy as np

    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()
  planner = A_Star_Planner()
  desired_traj = planner.construct_traj(current_state, desired_state, objects, walls)
  print(desired_traj[:][0])
  # print(len(desired_traj[0]))

  # Construct an environment
  env = gym.make("fetch-v0") # <-- this we need to create
  env.set_parameters(TIME_STEP_SIZE, objects)
  env.render('human')
  env.reset()

  # Create the trajectory and tracking controller
  controller = PointTracker()
  traj_tracker = TrajectoryTracker(desired_traj)
      
  # Create the feedback loop
  time.sleep(1)
  current_time_stamp = 0
  observation = [0,0,0,0,0]
  actual_traj = []
  x_error_list = []
  y_error_list = []
  t_error_list = []
  while not traj_tracker.is_traj_tracked(controller):
      current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
      desired_state = traj_tracker.get_traj_point_to_track(current_state)
      desired_state[0] = current_time_stamp

      print("Cur:", current_state, "Des:", desired_state)
      action = controller.point_tracking_control(desired_state, current_state)
      observation, reward, done, dummy = env.step(action)
      env.render('human')
      actual_traj.append(current_state)
      current_time_stamp += TIME_STEP_SIZE
  time.sleep(2)
  plot_traj(desired_traj, actual_traj, objects, walls)

  np.save('data/des_traj.npy', desired_traj)

  env.close()
  
def create_motion_planning_problem():
  current_state = [-10, 0, 0, 0]
  desired_state = [20, 5.0, 2.0, 0]
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5], [2, 1.5, 1.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()

    
    
