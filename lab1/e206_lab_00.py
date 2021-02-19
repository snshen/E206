import gym
import gym_fetch
import time
import math
import numpy as np
import random
import statistics
from traj_planner_utils import *
from traj_tracker import *

    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()

  # MODULAR
  # desired_traj = construct_dubins_traj(current_state, desired_state)
  desired_traj = [desired_state]
  desired_traj = [[0, 2, 0, 0], [0, 0, 0, 0], [0, 2, 2, 0], [0, 0, 0, 0], 
                  [0, 0, 2, math.pi/2], [0, 0, 0, 0], [0, -2, 2, 0], [0, 0, 0, 0], 
                  [0, -2, 0, 0], [0, 0, 0, 0], [0, -2, -2, 0], [0, 0, 0, 0], 
                  [0, 0, -2, -math.pi/2], [0, 0, 0, 0], [0, 2, -2, 0]]
  
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

  # while not controller.point_tracked:
  while not traj_tracker.is_traj_tracked(controller):
    current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
    desired_state = traj_tracker.get_traj_point_to_track(current_state)

    action = controller.point_tracking_control(desired_state, current_state)
    observation, reward, done, dummy = env.step(action)
    env.render('human')
    actual_traj.append(current_state)
    current_time_stamp += TIME_STEP_SIZE

    # MODULAR: for traj tracking
    # error_list.append(np.sqrt((desired_state[1]-current_state[1])**2 + (desired_state[2]-current_state[2])**2))
  #   x_error_list.append(abs(desired_state[1]-current_state[1]))
  #   y_error_list.append(abs(desired_state[2]-current_state[2]))
  #   t_error_list.append(abs(desired_state[3]-current_state[3]))

  #   if traj_tracker.traj[-1][0]<current_time_stamp:
  #     traj_tracker.traj_tracked = True
  #   elif desired_state[0]<current_time_stamp:
  #     traj_tracker.current_point_to_track += 1
    
  # print("X Mean Squared Error", statistics.mean(x_error_list), statistics.stdev(x_error_list))
  # print("Y Mean Squared Error", statistics.mean(y_error_list), statistics.stdev(y_error_list))
  # print("Theta Mean Squared Error", statistics.mean(t_error_list), statistics.stdev(t_error_list))

  
  time.sleep(2)
  plot_traj(desired_traj, actual_traj, objects, walls)

  # np.save('data/2_2.npy', actual_traj)
  
  env.close()
  
def create_motion_planning_problem():
  # current_state = [0, 0, 0, 0]
  # desired_state = [20, 5, 0, 0]
  current_state = [0, 0, 0, 0]
  desired_state = [20, 5, 3, 0]
  # current_state = [0, 0, 0, 0]
  # desired_state = [20, 5, -3, 0]
  
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  # objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  objects = []
  
  return current_state, desired_state, objects, walls



if __name__ == '__main__':
    # np.load('data/2_0.npy')

    main()
    
    
